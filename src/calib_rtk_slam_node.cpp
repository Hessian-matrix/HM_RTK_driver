#include <iostream>
#include <vector>
#include <queue>
#include <mutex>
#include <thread>
#include <chrono>
#include <fstream>
#include <algorithm>
#include <cmath>

#include "HM_RTK/ros_adapter.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>

#include "HM_RTK/trajectory_aligner.hpp"
#include "gnss_comm/gnss_utility.hpp"

std::string package_path = "";

class RTKSlamCalibrator {
public:
    RTKSlamCalibrator(ros_adapter::NodeHandle& nh, const Sophus::SE3d& init_ex_rtk_slam)
        : nh_(nh), init_ex_rtk_slam_(init_ex_rtk_slam) {
        
#ifdef ROS1_BUILD
        // 订阅传统3DOF RTK
        rtk_sub_ = nh_.subscribe("/baton/rtk", 10, &RTKSlamCalibrator::rtkCallback, this);
        
        // 订阅6DOF RTK (新增)
        rtk_6dof_sub_ = nh_.subscribe("/baton/rtk_sixdof", 10, &RTKSlamCalibrator::rtk6DOFCallback, this);
        
        // 订阅SLAM
        slam_sub_ = nh_.subscribe("/baton/stereo3/odometry", 10, &RTKSlamCalibrator::slamCallback, this);
#else
        // 订阅传统3DOF RTK
        rtk_sub_ = nh_->create_subscription<NavSatFixMsg>("/baton/rtk", 10, 
            [this](const NavSatFixMsg::SharedPtr msg) { this->rtkCallback(msg); });
        
        // 订阅6DOF RTK (新增)
        rtk_6dof_sub_ = nh_->create_subscription<OdometryMsg>("/baton/rtk_sixdof", 10,
            [this](const OdometryMsg::SharedPtr msg) { this->rtk6DOFCallback(msg); });
        
        // 订阅SLAM
        slam_sub_ = nh_->create_subscription<OdometryMsg>("/baton/stereo3/imu_odom", 10,
            [this](const OdometryMsg::SharedPtr msg) { this->slamCallback(msg); });
#endif
        
        optimization_thread_ = std::thread(&RTKSlamCalibrator::optimizationLoop, this);
        
        ROS_INFO("RTK SLAM Calibrator started. Listening for both 3DOF RTK (/baton/rtk) and 6DOF RTK (/baton/rtk_sixdof)");
    }

    ~RTKSlamCalibrator() {
        // 停止优化线程
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            stop_optimization_ = true;
        }
        optimization_thread_.join();
    }

private:
#ifdef ROS1_BUILD
    void rtkCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        rtk_queue_.push(*msg);
    }

    void rtk6DOFCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        rtk_6dof_queue_.push(*msg);
        has_6dof_rtk_ = true;  // 标记收到6DOF RTK数据
    }

    void slamCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        slam_queue_.push(*msg);
    }
#else
    void rtkCallback(const NavSatFixMsg::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        rtk_queue_.push(*msg);
    }

    void rtk6DOFCallback(const OdometryMsg::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        rtk_6dof_queue_.push(*msg);
        has_6dof_rtk_ = true;  // 标记收到6DOF RTK数据
    }

    void slamCallback(const OdometryMsg::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        slam_queue_.push(*msg);
    }
#endif

    void optimizationLoop() {
        // Get the package path and create a directory for results
        std::string results_dir = package_path + "/results";
        std::string results_path = results_dir + "/rtk_slam_calib_results.csv";
        
        // Create the results directory if it doesn't exist
        int dir_status = system(("mkdir -p " + results_dir).c_str());
        if (dir_status != 0) {
            ROS_WARN("Failed to create results directory. Using current directory instead.");
            results_path = "rtk_slam_calib_results.csv";
        }
        
        // Open the file with the full path
        std::ofstream outResults(results_path);
        ROS_INFO("Saving results to: %s", results_path.c_str());

        outResults << "mode,num_sample,converge,yaw,err_ave,err_max,ex_x,ex_y,ex_z,ex_qw,ex_qx,ex_qy,ex_qz\n";

        std::vector<Sophus::SE3d> exs_rtk_slam;
        CalibrationMode current_mode = CalibrationMode::MODE_3DOF_RTK;
        
        while (ros_adapter::ok() && !stop_optimization_) {
            // 1. 数据同步：将队列数据转到临时vector中
            std::vector<NavSatFixMsg> rtk_data;
            std::vector<OdometryMsg> rtk_6dof_data;
            std::vector<OdometryMsg> slam_data;
            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                while (!rtk_queue_.empty()) {
                    rtk_data.push_back(rtk_queue_.front());
                    rtk_queue_.pop();
                }
                while (!rtk_6dof_queue_.empty()) {
                    rtk_6dof_data.push_back(rtk_6dof_queue_.front());
                    rtk_6dof_queue_.pop();
                }
                while (!slam_queue_.empty()) {
                    slam_data.push_back(slam_queue_.front());
                    slam_queue_.pop();
                }
            }
            
            // 更新全局数据
            rtk_all_.insert(rtk_all_.end(), rtk_data.begin(), rtk_data.end());
            rtk_6dof_all_.insert(rtk_6dof_all_.end(), rtk_6dof_data.begin(), rtk_6dof_data.end());
            slam_all_.insert(slam_all_.end(), slam_data.begin(), slam_data.end());
            
            // 确定当前使用的模式
            if (has_6dof_rtk_ && !rtk_6dof_all_.empty()) {
                current_mode = CalibrationMode::MODE_6DOF_RTK;
            } else {
                current_mode = CalibrationMode::MODE_3DOF_RTK;
            }
            
            if (slam_data.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            if (current_mode == CalibrationMode::MODE_3DOF_RTK) {
                // Mode 1: 3DOF RTK处理逻辑（保持原有逻辑）
                
                // 如未设置静态参考点，则取首条固定数据的ECEF作为初始参考
                if(static_ref_ecef_.isZero() && !rtk_all_.empty()){
                    static_ref_ecef_ = gnss_comm::geo2ecef(
                        Eigen::Vector3d(rtk_all_.front().latitude, rtk_all_.front().longitude, rtk_all_.front().altitude));
                }

                // 构造轨迹数据
                std::vector<Eigen::Vector3d> rtk_trajectory;
                std::vector<Eigen::Vector3d> slam_trajectory;
                std::vector<Eigen::Matrix3d> R_world_cam;
                
                auto compare_time = [](const NavSatFixMsg& a, const NavSatFixMsg& b) {
                    return GET_STAMP_SEC(a) < GET_STAMP_SEC(b);
                };

                double velocity_thresh = 0.2;
                int n_start = std::max(1, static_cast<int>(slam_all_.size()) - 25*10);
                for (size_t i = n_start; i < slam_all_.size(); ++i) {
                    // 速度检查
                    Eigen::Vector3d slam_pose(slam_all_[i].pose.pose.position.x,
                                              slam_all_[i].pose.pose.position.y,
                                              slam_all_[i].pose.pose.position.z);
                    Eigen::Vector3d slam_pose_prev(slam_all_[i-1].pose.pose.position.x,
                                                   slam_all_[i-1].pose.pose.position.y,
                                                   slam_all_[i-1].pose.pose.position.z);
                    double dt = GET_STAMP_SEC(slam_all_[i]) - GET_STAMP_SEC(slam_all_[i-1]);
                    if(dt < 1e-3) continue;
                    double velocity = (slam_pose - slam_pose_prev).norm() / dt;
                    if (velocity < velocity_thresh) continue;

                    // RTK数据插值
                    double curTime = GET_STAMP_SEC(slam_all_[i]);
                    NavSatFixMsg time_anchor; 
                    SET_STAMP_SEC(time_anchor, curTime);
                    auto it = std::lower_bound(rtk_all_.begin(), rtk_all_.end(), time_anchor, compare_time);
                    if(it == rtk_all_.begin() || it == rtk_all_.end()) continue;
                    auto it_prev = std::prev(it);
                    if((GET_STAMP_SEC(*it) - GET_STAMP_SEC(*it_prev)) < 1e-3) continue;
                    if(it->status.status < 2 || it_prev->status.status < 2) continue;
                    double weight = (curTime - GET_STAMP_SEC(*it_prev)) /
                                    (GET_STAMP_SEC(*it) - GET_STAMP_SEC(*it_prev));
                    if(weight < 0.0 || weight > 1.0) continue;
                    
                    Eigen::Vector3d ecef_before = gnss_comm::geo2ecef(
                        Eigen::Vector3d(it_prev->latitude, it_prev->longitude, it_prev->altitude));
                    Eigen::Vector3d ecef_after = gnss_comm::geo2ecef(
                        Eigen::Vector3d(it->latitude, it->longitude, it->altitude));
                    Eigen::Vector3d rtk_ecef = (1 - weight) * ecef_before + weight * ecef_after;

                    Eigen::Quaterniond slam_q(slam_all_[i].pose.pose.orientation.w,
                                              slam_all_[i].pose.pose.orientation.x,
                                              slam_all_[i].pose.pose.orientation.y,
                                              slam_all_[i].pose.pose.orientation.z);
                    rtk_trajectory.push_back(rtk_ecef);
                    slam_trajectory.push_back(slam_pose);
                    R_world_cam.push_back(slam_q.matrix());
                }

                // 轨迹对齐优化
                if (rtk_trajectory.size() > 100) {
                    ROS_INFO("Running 3DOF RTK trajectory alignment, size=%d", static_cast<int>(rtk_trajectory.size()));
                    TrajectoryAligner aligner(rtk_trajectory, slam_trajectory, R_world_cam);
                    aligner.SetInitialRef(static_ref_ecef_);
                    aligner.SetInitialEx(init_ex_rtk_slam_.translation());

                    bool isConverged = aligner.Solve();
                    auto error = aligner.getError();
                    double yaw = aligner.GetYaw();
                    Eigen::Vector3d ex = aligner.GetEx();
                    
                    // 计算方向分布
                    Eigen::Vector3d sum_direction = Eigen::Vector3d::Zero();
                    for(auto& R : R_world_cam){
                        Eigen::Vector3d d = R * Eigen::Vector3d::UnitZ();
                        sum_direction += d;
                    }
                    double angle_diff = sum_direction.norm()/R_world_cam.size();
                    exs_rtk_slam.emplace_back(Sophus::SE3d(Eigen::Quaterniond::Identity(), ex));

                    printf("Mode: 3DOF, n,%d,converge,%d,yaw,%f,ex,%f,%f,%f,error,%f,%f,%f\n",
                        static_cast<int>(rtk_trajectory.size()),
                        isConverged, yaw, ex.x(), ex.y(), ex.z(),
                        error.first, error.second, angle_diff);
                    outResults << "3DOF," << rtk_trajectory.size() << "," << isConverged << "," << yaw << ","
                               << error.first << "," << error.second << "," 
                               << ex.x() << "," << ex.y() << "," << ex.z() << "\n" << std::flush;
                } else {
                    std::cout << "\r[3DOF Mode] Collecting more moving data... Current trajectory size: " 
                              << rtk_trajectory.size() << std::flush;
                }
                
            } else {
                // Mode 2: 6DOF RTK处理逻辑
                
                // 构造SE3轨迹数据
                std::vector<Eigen::Vector3d> rtk_positions;
                std::vector<Sophus::SE3d> rtk_poses;
                std::vector<Sophus::SE3d> slam_poses;
                
                auto compare_time_6dof = [](const OdometryMsg& a, const OdometryMsg& b) {
                    return GET_STAMP_SEC(a) < GET_STAMP_SEC(b);
                };

                int n1=0, n2=0, n3=0, n4=0, n5=0, n6=0, n7=0;
                double velocity_thresh = 0.2;
                int n_start = std::max(1, static_cast<int>(slam_all_.size()) - 25*10);
                for (size_t i = n_start; i < slam_all_.size(); ++i) {
                    // 速度检查
                    Eigen::Vector3d slam_pose(slam_all_[i].pose.pose.position.x,
                                              slam_all_[i].pose.pose.position.y,
                                              slam_all_[i].pose.pose.position.z);
                    Eigen::Vector3d slam_pose_prev(slam_all_[i-1].pose.pose.position.x,
                                                   slam_all_[i-1].pose.pose.position.y,
                                                   slam_all_[i-1].pose.pose.position.z);
                    double dt = GET_STAMP_SEC(slam_all_[i]) - GET_STAMP_SEC(slam_all_[i-1]);
                    if(dt < 1e-3) { n1++; continue; }
                    double velocity = (slam_pose - slam_pose_prev).norm() / dt;
                    if (velocity < velocity_thresh) { n2++; continue; }

                    // 6DOF RTK数据插值
                    double curTime = GET_STAMP_SEC(slam_all_[i]);
                    OdometryMsg time_anchor; 
                    SET_STAMP_SEC(time_anchor, curTime);
                    auto it = std::lower_bound(rtk_6dof_all_.begin(), rtk_6dof_all_.end(), time_anchor, compare_time_6dof);
                    if(it == rtk_6dof_all_.begin() || it == rtk_6dof_all_.end()) { n3++; continue; }
                    auto it_prev = std::prev(it);
                    if((GET_STAMP_SEC(*it) - GET_STAMP_SEC(*it_prev)) < 1e-3) { n4++; continue; }

                    double weight = (curTime - GET_STAMP_SEC(*it_prev)) /
                                    (GET_STAMP_SEC(*it) - GET_STAMP_SEC(*it_prev));
                    if(weight < 0.0 || weight > 1.0) { n5++; continue; }

                    // 位置插值
                    Eigen::Vector3d pos_before(it_prev->pose.pose.position.x, it_prev->pose.pose.position.y, it_prev->pose.pose.position.z);
                    Eigen::Vector3d pos_after(it->pose.pose.position.x, it->pose.pose.position.y, it->pose.pose.position.z);
                    Eigen::Vector3d rtk_pos = (1 - weight) * pos_before + weight * pos_after;
                    
                    // 姿态插值（使用slerp）
                    Eigen::Quaterniond q_before(it_prev->pose.pose.orientation.w, it_prev->pose.pose.orientation.x, 
                                              it_prev->pose.pose.orientation.y, it_prev->pose.pose.orientation.z);
                    Eigen::Quaterniond q_after(it->pose.pose.orientation.w, it->pose.pose.orientation.x,
                                             it->pose.pose.orientation.y, it->pose.pose.orientation.z);
                    Eigen::Quaterniond rtk_q = q_before.slerp(weight, q_after);
                    
                    // 构建SE3位姿
                    Sophus::SE3d rtk_se3(rtk_q, rtk_pos);
                    
                    Eigen::Quaterniond slam_q(slam_all_[i].pose.pose.orientation.w,
                                              slam_all_[i].pose.pose.orientation.x,
                                              slam_all_[i].pose.pose.orientation.y,
                                              slam_all_[i].pose.pose.orientation.z);
                    Sophus::SE3d slam_se3(slam_q, slam_pose);
                    
                    rtk_positions.push_back(rtk_pos);
                    rtk_poses.push_back(rtk_se3);
                    slam_poses.push_back(slam_se3);
                }
                // std::cout << " >>>> all=" << slam_all_.size() << ", final=" << rtk_poses.size() << ", n1=" << n1 << ", n2=" << n2 
                //           << ", n3=" << n3 << ", n4=" << n4 << ", n5=" << n5 << std::endl;

                // SE3轨迹对齐优化
                if (rtk_poses.size() > 100) {
                    ROS_INFO("Running 6DOF RTK SE3 trajectory alignment, size=%d", static_cast<int>(rtk_poses.size()));
                    TrajectoryAligner aligner(rtk_poses, slam_poses);
                    
                    // 初始化SE3外参
                    aligner.SetInitialSE3Ex(init_ex_rtk_slam_);
                    aligner.SetInitialRef(rtk_poses.front().translation());
                    // aligner.SetYawFixed(true);
                    // aligner.SetRefFixed(true);
                    // aligner.SetSE3ExFixed(true);

                    bool isConverged = aligner.Solve();
                    auto error = aligner.getError();
                    double yaw = aligner.GetYaw();
                    Eigen::Vector3d ancher_ecef = aligner.GetRef();
                    Sophus::SE3d se3_ex = aligner.GetSE3Ex();
                    Eigen::Vector3d ex = se3_ex.translation();
                    Eigen::Quaterniond ex_q = se3_ex.unit_quaternion();

                    exs_rtk_slam.emplace_back(se3_ex);

                    printf("Mode: 6DOF, n,%d,converge,%d,yaw,%f,ancher,%f,%f,%f,ex,%f,%f,%f,q,%f,%f,%f,%f,error,%f,%f\n",
                        static_cast<int>(rtk_poses.size()), isConverged, yaw, 
                        ancher_ecef.x(), ancher_ecef.y(), ancher_ecef.z(),
                        ex.x(), ex.y(), ex.z(),
                        ex_q.w(), ex_q.x(), ex_q.y(), ex_q.z(),
                        error.first, error.second);
                    outResults << "6DOF," << rtk_poses.size() << "," << isConverged << "," << yaw << ","
                               << ex.x() << "," << ex.y() << "," << ex.z() << ","
                               << error.first << "," << error.second << ","
                               << ex_q.w() << "," << ex_q.x() << "," << ex_q.y() << "," << ex_q.z() << "\n" << std::flush;
                } else {
                    std::cout << "\r[6DOF Mode] Collecting more moving data... Current trajectory size: " 
                              << rtk_poses.size() << std::flush << std::endl;
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 优化频率
        }
        
        outResults.close();
        Sophus::SE3d ex_ave_se3;
        if(!exs_rtk_slam.empty()){
            Eigen::VectorXd ex_ave = Eigen::VectorXd::Zero(6);
            for(const auto& ex : exs_rtk_slam) ex_ave += ex.log();
            ex_ave /= exs_rtk_slam.size();
            ex_ave_se3 = Sophus::SE3d::exp(ex_ave);
        }
        std::cout << "ex_ave: " << ex_ave_se3.translation().transpose() 
                << ", q=" << ex_ave_se3.unit_quaternion().coeffs().transpose() 
                << ", size=" << exs_rtk_slam.size() << std::endl;
        std::cout << "results saved to: " << results_path << std::endl;
    }

private:
    ros_adapter::NodeHandle& nh_;
#ifdef ROS1_BUILD
    ros::Subscriber rtk_sub_;
    ros::Subscriber rtk_6dof_sub_;  // 新增：6DOF RTK订阅器
    ros::Subscriber slam_sub_;
#else
    ros_adapter::Subscriber_t<NavSatFixMsg> rtk_sub_;
    ros_adapter::Subscriber_t<OdometryMsg> rtk_6dof_sub_;  // 新增：6DOF RTK订阅器
    ros_adapter::Subscriber_t<OdometryMsg> slam_sub_;
#endif
    std::queue<NavSatFixMsg> rtk_queue_;
    std::queue<OdometryMsg> rtk_6dof_queue_;  // 新增：6DOF RTK队列
    std::queue<OdometryMsg> slam_queue_;
    std::vector<NavSatFixMsg> rtk_all_;
    std::vector<OdometryMsg> rtk_6dof_all_;  // 新增：6DOF RTK历史数据
    std::vector<OdometryMsg> slam_all_;
    std::mutex queue_mutex_;

    Eigen::Vector3d static_ref_ecef_ = Eigen::Vector3d::Zero();
    Sophus::SE3d init_ex_rtk_slam_;
    std::thread optimization_thread_;
    bool stop_optimization_ = false;
    bool has_6dof_rtk_ = false;  // 新增：标记是否收到6DOF RTK数据
};

int main(int argc, char** argv) {
    ros_adapter::init(argc, argv, "rtk_slam_calibrator");
    
#ifdef ROS1_BUILD
    ros_adapter::NodeHandle nh = ros_adapter::createNodeHandle("~");
#else
    auto nh = ros_adapter::createNodeHandle("rtk_slam_calibrator");
#endif

    // 在左目坐标系下， XYZ-右下前. y必须手量，因为在校正过程中高度差方向不客观，优化时固定。提供的初值即最终值。
    double x,y,z, yaw;
    ros_adapter::getParam(nh, "ex_rtk_slam_x", x, 0.03);
    ros_adapter::getParam(nh, "ex_rtk_slam_y", y, -0.13);
    ros_adapter::getParam(nh, "ex_rtk_slam_z", z, -0.21);
    ros_adapter::getParam(nh, "ex_rtk_slam_yaw", yaw, 30.0);
    ros_adapter::getParam(nh, "package_path", package_path, std::string("/home/ll/C/ws/zcf_GNSS_Driver_ws/outputs/"));

    Eigen::Vector3d init_ex_rtk_slam(x,y,z);
    Eigen::Matrix3d R_init; R_init << 0,-1,0, 0,0,-1, 1,0,0;
    Eigen::Matrix3d R_yaw_ex = Eigen::AngleAxisd(yaw/180.0*M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Sophus::SE3d init_se3_ex(R_init * R_yaw_ex, init_ex_rtk_slam);

    std::cout << "init_ex_rtk_slam: " << init_ex_rtk_slam.transpose() << std::endl;
    RTKSlamCalibrator calibrator(nh, init_se3_ex);

#ifdef ROS1_BUILD
    ros::spin();
#else
    rclcpp::spin(nh);
#endif
    return 0;
}

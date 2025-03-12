#include <iostream>
#include <vector>
#include <queue>
#include <mutex>
#include <thread>
#include <chrono>
#include <fstream>
#include <algorithm>
#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "HM_RTK/trajectory_aligner.hpp"
#include "gnss_comm/gnss_utility.hpp"

std::string package_path = "";
class RTKSlamCalibrator {
public:
    RTKSlamCalibrator(ros::NodeHandle& nh, const Eigen::Vector3d& init_ex_rtk_slam)
        : nh_(nh), init_ex_rtk_slam_(init_ex_rtk_slam) {
        rtk_sub_ = nh_.subscribe("/baton/rtk", 10, &RTKSlamCalibrator::rtkCallback, this);
        slam_sub_ = nh_.subscribe("/baton/stereo3/odometry", 10, &RTKSlamCalibrator::slamCallback, this);
        optimization_thread_ = std::thread(&RTKSlamCalibrator::optimizationLoop, this);
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
    void rtkCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        rtk_queue_.push(*msg);
    }

    void slamCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        slam_queue_.push(*msg);
    }

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
        
        outResults << "n,converge,yaw,ex_x,ex_y,ex_z,err_ave,err_max,directional_distribution\n";

        std::vector<Eigen::Vector3d> exs_rtk_slam;
        while (ros::ok() && !stop_optimization_) {
            // 1. 数据同步：将队列数据转到临时vector中
            std::vector<sensor_msgs::NavSatFix> rtk_data;
            std::vector<nav_msgs::Odometry> slam_data;
            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                while (!rtk_queue_.empty()) {
                    rtk_data.push_back(rtk_queue_.front());
                    rtk_queue_.pop();
                }
                while (!slam_queue_.empty()) {
                    slam_data.push_back(slam_queue_.front());
                    slam_queue_.pop();
                }
            }
            rtk_all_.insert(rtk_all_.end(), rtk_data.begin(), rtk_data.end());
            slam_all_.insert(slam_all_.end(), slam_data.begin(), slam_data.end());
            if (rtk_data.empty() || slam_data.empty()) {
                // std::cout << "\rno data, rtk=" << rtk_data.size() << ", slam=" << slam_data.size() << std::flush;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            // 如未设置静态参考点，则取首条固定数据的ECEF作为初始参考
            if(static_ref_ecef_.isZero() && !rtk_all_.empty()){
                static_ref_ecef_ = gnss_comm::geo2ecef(
                    Eigen::Vector3d(rtk_all_.front().latitude, rtk_all_.front().longitude, rtk_all_.front().altitude));
            }

            // 2. 动态数据处理与插值：构造当前轨迹数据
            std::vector<Eigen::Vector3d> rtk_trajectory;
            std::vector<Eigen::Vector3d> slam_trajectory;
            std::vector<Eigen::Matrix3d> R_world_cam;
            std::vector<double> slam_trajectory_time;
            auto compare_time = [](const sensor_msgs::NavSatFix& a, const sensor_msgs::NavSatFix& b) {
                return a.header.stamp.toSec() < b.header.stamp.toSec();
            };

            double velocity_thresh = 0.2;
            int n_start = std::max(1, static_cast<int>(slam_all_.size()) - 25*10); // 10s的数据足够了。再长SLAM精度会下降
            for (size_t i = n_start; i < slam_all_.size(); ++i) {
                Eigen::Vector3d slam_pose(slam_all_[i].pose.pose.position.x,
                                          slam_all_[i].pose.pose.position.y,
                                          slam_all_[i].pose.pose.position.z);
                Eigen::Vector3d slam_pose_prev(slam_all_[i-1].pose.pose.position.x,
                                               slam_all_[i-1].pose.pose.position.y,
                                               slam_all_[i-1].pose.pose.position.z);
                double dt = slam_all_[i].header.stamp.toSec() - slam_all_[i-1].header.stamp.toSec();
                if(dt < 1e-3) continue;
                double velocity = (slam_pose - slam_pose_prev).norm() / dt;
                if (velocity < velocity_thresh) continue;

                double curTime = slam_all_[i].header.stamp.toSec();
                sensor_msgs::NavSatFix time_anchor; 
                time_anchor.header.stamp.fromSec(curTime);
                auto it = std::lower_bound(rtk_all_.begin(), rtk_all_.end(), time_anchor, compare_time);
                if(it == rtk_all_.begin() || it == rtk_all_.end()) continue;
                auto it_prev = std::prev(it);
                if((it->header.stamp.toSec() - it_prev->header.stamp.toSec()) < 1e-3) continue;
                if(it->status.status < 2 || it_prev->status.status < 2) continue;
                double weight = (curTime - it_prev->header.stamp.toSec()) /
                                (it->header.stamp.toSec() - it_prev->header.stamp.toSec());
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
                slam_trajectory_time.push_back(curTime);
            }

            // 3. 轨迹对齐与优化：当数据足够时运行优化
            if (rtk_trajectory.size() > 100) {
                ROS_INFO("Running trajectory alignment, size=%d", static_cast<int>(rtk_trajectory.size()));
                TrajectoryAligner aligner(rtk_trajectory, slam_trajectory, R_world_cam);
                aligner.SetInitialRef(static_ref_ecef_);
                aligner.SetInitialEx(init_ex_rtk_slam_);
                // aligner.SetRefFixed(true); // 固定参考点
                // aligner.SetExFixed(true); // 固定外参位移

                bool isConverged = aligner.Solve();
                auto error = aligner.getError();
                double yaw = aligner.GetYaw();
                Eigen::Vector3d ex = aligner.GetEx();
                Eigen::Vector3d diff = aligner.GetRef() - static_ref_ecef_;
                Eigen::Vector3d sum_direction = Eigen::Vector3d::Zero();
                for(auto& R : R_world_cam){
                    Eigen::Vector3d d = R * Eigen::Vector3d::UnitZ();
                    sum_direction += d;
                }
                double angle_diff = sum_direction.norm()/R_world_cam.size();
                exs_rtk_slam.emplace_back(ex);

                printf("n,%d,converge,%d,yaw,%f,ex,%f,%f,%f,error,%f,%f,%f\n",
                    static_cast<int>(rtk_trajectory.size()),
                    isConverged, yaw, ex.x(), ex.y(), ex.z(),
                    error.first, error.second, angle_diff);
                outResults << rtk_trajectory.size() << "," << isConverged << "," << yaw << ","
                           << ex.x() << "," << ex.y() << "," << ex.z() << ","
                           << error.first << "," << error.second << "," << angle_diff << "\n" << std::flush;

                // 如有需要，可发布结果或写入日志
                if(false){
                    std::string out_path = "/home/ll/C/ws/zcf_GNSS_Driver_ws/Log/";
                    std::ofstream out_slam(out_path + "slam.csv");
                    for(int i=0; i<slam_trajectory.size(); ++i){
                        const auto& t = slam_trajectory[i];
                        out_slam << t[0] << "," << t[1] << "," << t[2] << "," << i << std::endl;
                    }

                    std::ofstream out_rtk(out_path + "rtk.csv");
                    Eigen::Matrix3d R_yaw_inv = Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
                    Eigen::Vector3d ref = static_ref_ecef_;
                    for(int i=0; i<rtk_trajectory.size(); ++i){
                        Eigen::Vector3d enu = gnss_comm::ecef2enu(gnss_comm::ecef2geo(ref), rtk_trajectory[i]-ref);
                        const auto& t = R_yaw_inv * enu;
                        out_rtk << t[0] << "," << t[1] << "," << t[2] << "," << i << std::endl;
                    }

                    std::vector<Eigen::Vector3d> aligned_trajectory = aligner.GetAlignedTrajectory();
                    std::ofstream out_rtk_aligned(out_path + "rtk_aligned.csv");
                    for(int i=0; i<aligned_trajectory.size(); ++i){
                        const auto& t = aligned_trajectory[i];
                        out_rtk_aligned << t[0] << "," << t[1] << "," << t[2] << "," << i << std::endl;
                    }
                    
                }
            } else {
                std::cout << "\rCollecting more moving data... Current trajectory size: " 
                          << rtk_trajectory.size() << std::flush;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 优化频率
        }
        outResults.close();
        Eigen::Vector3d ex_ave = Eigen::Vector3d::Zero();
        if(!exs_rtk_slam.empty()){
            for(const auto& ex : exs_rtk_slam) ex_ave += ex;
            ex_ave /= exs_rtk_slam.size();
        }
        std::cout << "ex_ave: " << ex_ave.transpose() << ", size=" << exs_rtk_slam.size() << std::endl;
        std::cout << "results saved to: " << results_path << std::endl;
    }

private:
    ros::NodeHandle& nh_;
    ros::Subscriber rtk_sub_;
    ros::Subscriber slam_sub_;
    std::queue<sensor_msgs::NavSatFix> rtk_queue_;
    std::queue<nav_msgs::Odometry> slam_queue_;
    std::vector<sensor_msgs::NavSatFix> rtk_all_;
    std::vector<nav_msgs::Odometry> slam_all_;
    std::mutex queue_mutex_;

    Eigen::Vector3d static_ref_ecef_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d init_ex_rtk_slam_ = Eigen::Vector3d::Zero();
    std::thread optimization_thread_;
    bool stop_optimization_ = false;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "rtk_slam_calibrator");
    ros::NodeHandle nh("~");

    // 在左目坐标系下， XYZ-右下前. y必须手量，因为在校正过程中高度差方向不客观，优化时固定。提供的初值即最终值。
    double x,y,z;
    nh.param<double>("ex_rtk_slam_x", x, 0.03);
    nh.param<double>("ex_rtk_slam_y", y, -0.13);
    nh.param<double>("ex_rtk_slam_z", z, -0.21);
    nh.param<std::string>("package_path", package_path, "");
    Eigen::Vector3d init_ex_rtk_slam(x,y,z);
    std::cout << "init_ex_rtk_slam: " << init_ex_rtk_slam.transpose() << std::endl;
    RTKSlamCalibrator calibrator(nh, init_ex_rtk_slam);

    ros::spin();
    return 0;
}

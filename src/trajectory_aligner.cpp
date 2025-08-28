#include "HM_RTK/trajectory_aligner.hpp"
#include <iostream>
#include <HM_RTK/utils.hpp>
#include "gnss_comm/gnss_utility.hpp"

using namespace Eigen;

Eigen::Matrix3d ComputeRotationEcefToEnu(const Eigen::Vector3d ref_ecef) {
    return gnss_comm::ecef2rotation(ref_ecef).transpose();
}

// Mode 1: 3DOF RTK构造函数（保持不变）
TrajectoryAligner::TrajectoryAligner(
    const std::vector<Vector3d>& rtk_trajectory,
    const std::vector<Vector3d>& slam_trajectory,
    const std::vector<Matrix3d>& R_world_cam)
    : mode_(CalibrationMode::MODE_3DOF_RTK),
      rtk_trajectory_(rtk_trajectory),
      slam_trajectory_(slam_trajectory),
      R_world_cam_(R_world_cam) {
    
    if (rtk_trajectory.empty()) {
        throw std::invalid_argument("Input trajectories cannot be empty");
    }
    if (rtk_trajectory.size() != slam_trajectory.size() || 
        rtk_trajectory.size() != R_world_cam.size()) {
        throw std::invalid_argument("Input trajectories must have same size");
    }
}

// Mode 2: 6DOF RTK构造函数（新增）
TrajectoryAligner::TrajectoryAligner(
    const std::vector<Sophus::SE3d>& rtk_poses,
    const std::vector<Sophus::SE3d>& slam_poses)
    : mode_(CalibrationMode::MODE_6DOF_RTK),
      rtk_poses_(rtk_poses),
      slam_poses_(slam_poses) {
    
    if (rtk_poses.empty()) {
        throw std::invalid_argument("Input poses cannot be empty");
    }
    if (rtk_poses.size() != slam_poses.size()) {
        throw std::invalid_argument("RTK and SLAM poses must have same size");
    }
    
    // 初始化参考SE3位姿
    if (!rtk_poses_.empty()) {
        ref_ecef_ = rtk_poses_[0].translation();
        yaw_ = 0;
    }
}

template <typename T>
Vector3d getVec3d(Matrix<T,3,1> ref_ecef) {
    if constexpr (std::is_floating_point<T>::value) {
        return ref_ecef.template cast<double>();
    } else { // T is ceres::Jet type
        return Eigen::Vector3d(ref_ecef[0].a, ref_ecef[1].a, ref_ecef[2].a);
    }
}

template <typename T>
Matrix<T,3,3> getR_yaw_inv(const T& yaw) {
    T cos_yaw = ceres::cos(yaw);
    T sin_yaw = ceres::sin(yaw);
    Matrix<T,3,3> R_yaw_inv;
    R_yaw_inv << cos_yaw,  sin_yaw, T(0),
                -sin_yaw,  cos_yaw, T(0),
                 T(0),     T(0),    T(1);
    return R_yaw_inv;
}
template <typename T>
Matrix<T,3,3> getR_yaw(const T& yaw) {
    T cos_yaw = ceres::cos(yaw);
    T sin_yaw = ceres::sin(yaw);
    Matrix<T,3,3> R_yaw;
    R_yaw << cos_yaw, -sin_yaw, T(0),
             sin_yaw,  cos_yaw, T(0),
             T(0),     T(0),    T(1);
    return R_yaw;
}

// Mode 1: 位置对齐残差函数 (保持不变)
template <typename T>
bool TrajectoryAligner::PositionAlignmentResidual::operator()(
    const T* const yaw,
    const T* const ref_ecef,
    const T* const t_ex,
    T* residual) const 
{
    // 增加零向量检查
    if (rtk_pose_.norm() < 1e-6 || slam_pose_.norm() < 1e-6) {
        return false;
    }

    // Yaw旋转矩阵
    Matrix<T,3,3> R_yaw_inv = getR_yaw_inv(*yaw);
    Matrix<T,3,1> t_ex_vec(t_ex[0], (T)y_t_ex_init_, t_ex[2]);
    Matrix<T,3,1> ref(ref_ecef[0], ref_ecef[1], ref_ecef[2]);

    // 坐标变换
    Matrix<T,3,1> p_ecef(rtk_pose_.template cast<T>());
    Matrix<T,3,3> R_world_cam = R_world_cam_.template cast<T>();
    Matrix<T,3,3> R_enu_ecef = ComputeRotationEcefToEnu(getVec3d(ref)).template cast<T>();
    Matrix<T,3,1> predicted = R_yaw_inv * R_enu_ecef * (p_ecef - ref) - R_world_cam * t_ex_vec;
    Matrix<T,3,1> target = slam_pose_.template cast<T>();

    // 增加数值稳定性检查
    if (ceres::abs(predicted.norm()) > T(1e6) || ceres::abs(target.norm()) > T(1e6)) {
        return false;
    }
    
    residual[0] = predicted.x() - target.x();
    residual[1] = predicted.y() - target.y();
    residual[2] = predicted.z() - target.z();
    return true;
}

// Mode 2: SE3对齐残差函数 (分离参数化版本)
template <typename T>
bool TrajectoryAligner::SE3AlignmentResidual::operator()(
    const T* const yaw,
    const T* const ref_ecef,
    const T* const quat_ex,     // 外参四元数 [x,y,z,w]
    const T* const trans_ex,    // 外参平移 [x,y,z]
    T* residual) const 
{
    using Vector3T = Eigen::Matrix<T, 3, 1>;
    using Matrix3T = Eigen::Matrix<T, 3, 3>;
    using QuatT = Eigen::Quaternion<T>;
    using SE3T = Sophus::SE3<T>;
    
    // 构建外参变换 (四元数格式: [x,y,z,w])
    QuatT q_ex(quat_ex[3], quat_ex[0], quat_ex[1], quat_ex[2]); // [w,x,y,z]
    Vector3T t_ex(trans_ex[0], T(y_t_ex_init_), trans_ex[2]);
    SE3T T_ex(q_ex, t_ex);

    // Yaw旋转（仅绕Z轴）
    Matrix3T R_yaw = getR_yaw(*yaw);
    SE3T T_yaw(Sophus::SO3<T>(R_yaw), Vector3T::Zero());
    
    // ECEF到ENU变换
    Vector3T ref_ecef_vec(ref_ecef[0], ref_ecef[1], ref_ecef[2]);
    Matrix3T R_enu_ecef = ComputeRotationEcefToEnu(getVec3d(ref_ecef_vec)).template cast<T>();
    Matrix3T R_nwu_enu = Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ()).toRotationMatrix().template cast<T>();

    SE3T T_rtk = rtk_pose_.template cast<T>();
    SE3T T_slam = slam_pose_.template cast<T>();
    SE3T T_rtk_innwu = T_rtk; 
    T_rtk_innwu.translation() = R_nwu_enu * R_enu_ecef * (T_rtk.translation() - ref_ecef_vec); 
    SE3T T_slam_innwu = T_yaw * T_slam * T_ex;
    SE3T T_error = T_rtk_innwu * T_slam_innwu.inverse();
    // Vector3T err_t = T_error.translation();
    Vector3T err_t = T_rtk_innwu.translation() - T_slam_innwu.translation();
    Vector3T err_r = T_error.so3().log();

    residual[0] = err_t[0];
    residual[1] = err_t[1];
    residual[2] = err_t[2];
    residual[3] = err_r[0];
    residual[4] = err_r[1];
    residual[5] = err_r[2];

    return true;
}

bool TrajectoryAligner::Solve() {
    ceres::Problem problem;
    
    // 声明SE3模式用的变量
    std::vector<double> quat_ex(4);  // [x,y,z,w]
    std::vector<double> trans_ex(3); // [x,y,z]
    
    if (mode_ == CalibrationMode::MODE_3DOF_RTK) {
        // Mode 1: 现有逻辑保持不变
        problem.AddParameterBlock(&yaw_, 1);
        problem.AddParameterBlock(ref_ecef_.data(), 3);
        problem.AddParameterBlock(t_ex_.data(), 3);
        
        if (yaw_fixed_) problem.SetParameterBlockConstant(&yaw_);
        if (ref_fixed_) problem.SetParameterBlockConstant(ref_ecef_.data());
        if (ex_fixed_) problem.SetParameterBlockConstant(t_ex_.data());
        
        for (size_t i = 0; i < rtk_trajectory_.size(); ++i) {
            ceres::CostFunction* cost_function =
                new ceres::AutoDiffCostFunction<PositionAlignmentResidual, 3, 1, 3, 3>(
                    new PositionAlignmentResidual(rtk_trajectory_[i], 
                                                slam_trajectory_[i],
                                                R_world_cam_[i],
                                                y_t_ex_init_));
            
            problem.AddResidualBlock(cost_function, nullptr, 
                                   &yaw_, ref_ecef_.data(), t_ex_.data());
        }
        
    } else {
        // Mode 2: SE3优化 (分离参数化版本)
        Eigen::Quaterniond q = se3_ex_.unit_quaternion();
        Eigen::Vector3d t = se3_ex_.translation();
        
        // 分离参数化: 四元数[x,y,z,w] + 平移[x,y,z]
        quat_ex[0] = q.x(); quat_ex[1] = q.y(); 
        quat_ex[2] = q.z(); quat_ex[3] = q.w();
        trans_ex[0] = t[0]; trans_ex[1] = t[1]; trans_ex[2] = t[2];
        
        problem.AddParameterBlock(&yaw_, 1);
        problem.AddParameterBlock(ref_ecef_.data(), 3);
        problem.AddParameterBlock(quat_ex.data(), 4);
        problem.AddParameterBlock(trans_ex.data(), 3);
        
        // 四元数约束（单位四元数）- 适配Ceres 2.2.0
        // 在新版本中，四元数自动规范化，不需要显式设置Manifold
        
        if (yaw_fixed_) problem.SetParameterBlockConstant(&yaw_);
        if (ref_fixed_) problem.SetParameterBlockConstant(ref_ecef_.data());
        if (se3_ex_fixed_) {
            problem.SetParameterBlockConstant(quat_ex.data());
            problem.SetParameterBlockConstant(trans_ex.data());
        }
        
        for (size_t i = 0; i < rtk_poses_.size(); ++i) {
            ceres::CostFunction* cost_function =
                new ceres::AutoDiffCostFunction<SE3AlignmentResidual, 6, 1, 3, 4, 3>(
                    new SE3AlignmentResidual(rtk_poses_[i], 
                                           slam_poses_[i],
                                           y_t_ex_init_));
            
            problem.AddResidualBlock(cost_function, nullptr,
                                   &yaw_, ref_ecef_.data(), quat_ex.data(), trans_ex.data());
        }
    }
    
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    
    // 优化完成后，更新SE3外参（仅SE3模式）
    if (mode_ == CalibrationMode::MODE_6DOF_RTK) {
        // 重新构建SE3外参（四元数会在构造时自动规范化）
        Eigen::Quaterniond q_result(quat_ex[3], quat_ex[0], quat_ex[1], quat_ex[2]); // [w,x,y,z]
        Eigen::Vector3d t_result(trans_ex[0], trans_ex[1], trans_ex[2]);
        se3_ex_ = Sophus::SE3d(q_result, t_result);
        t_ex_ = t_result;
    }
    
    std::cout << summary.BriefReport() << "\n";
    return summary.IsSolutionUsable();
}

static int frame_count = 0;
std::vector<Eigen::Vector3d> TrajectoryAligner::GetAlignedTrajectory() const{
    std::vector<Eigen::Vector3d> aligned_trajectory;
    
    if (mode_ == CalibrationMode::MODE_3DOF_RTK) {
        // Mode 1: 现有逻辑
        aligned_trajectory.reserve(rtk_trajectory_.size());
        
        if (ref_ecef_.hasNaN()) {
            throw std::runtime_error("Invalid reference ECEF coordinates");
        }

        Eigen::Matrix3d R_yaw_inv = getR_yaw_inv(yaw_);
        Eigen::Vector3d t_ex_vec(t_ex_[0], y_t_ex_init_, t_ex_[2]);
        Eigen::Matrix3d R_enu_ecef = ComputeRotationEcefToEnu(ref_ecef_);
        for (size_t i = 0; i < rtk_trajectory_.size(); ++i) {
            Eigen::Vector3d predicted = R_yaw_inv * R_enu_ecef * (rtk_trajectory_[i] - ref_ecef_) - R_world_cam_[i] * t_ex_vec;
            aligned_trajectory.push_back(predicted);
        }
    } else {
        // Mode 2: SE3模式 - 实现完整的对齐轨迹计算
        aligned_trajectory.reserve(rtk_poses_.size());
        
        Eigen::Matrix3d R_yaw = getR_yaw(yaw_);
        Eigen::Matrix3d R_enu_ecef = ComputeRotationEcefToEnu(ref_ecef_);
        Eigen::Matrix3d R_nwu_enu = Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        
        
        for (size_t i = 0; i < rtk_poses_.size(); ++i) {
            // 变换链：T_predicted = T_yaw_inv * T_enu_ecef * (T_rtk * T_ref_inv) - T_ex
            Eigen::Vector3d pos_nwu = R_nwu_enu * R_enu_ecef * (rtk_poses_[i].translation() - ref_ecef_);
            Sophus::SE3d T_nwu_rtk(rtk_poses_[i].so3(), pos_nwu);
            Sophus::SE3d T_local_slam = Sophus::SE3d(R_yaw, Eigen::Vector3d::Zero()).inverse() * T_nwu_rtk * se3_ex_.inverse();

            aligned_trajectory.push_back(T_local_slam.translation());
        }

        // auto print_pose = [](std::ostream& os, const Sophus::SE3d& pose, int i) {
        //     const double arrow_length = 0.05;
        //     Eigen::Matrix3d R = pose.so3().matrix();
        //     const Eigen::Vector3d pt = pose.translation();
        //     const Eigen::Vector3d px = pt + R * Eigen::Vector3d(arrow_length, 0, 0);
        //     const Eigen::Vector3d py = pt + R * Eigen::Vector3d(0, arrow_length, 0);
        //     const Eigen::Vector3d pz = pt + R * Eigen::Vector3d(0, 0, arrow_length);

        //     os << pt[0] << "," << pt[1] << "," << pt[2] << "," << i << ",0" << std::endl;
        //     os << px[0] << "," << px[1] << "," << px[2] << "," << i << ",1" << std::endl;
        //     os << py[0] << "," << py[1] << "," << py[2] << "," << i << ",2" << std::endl;
        //     os << pz[0] << "," << pz[1] << "," << pz[2] << "," << i << ",3" << std::endl;
        // };

        // {
        //     frame_count++;
        //     Sophus::SE3d T_yaw_slam(R_yaw, Eigen::Vector3d::Zero());
        //     std::cout << "current yaw=" << yaw_*180/M_PI << ", frame=" << frame_count << std::endl;

        //     std::string outputs_path = "/home/ll/C/ws/zcf_GNSS_Driver_ws/outputs/";
        //     std::ofstream nwu_slam_tri(outputs_path + "nwu_slam_" + std::to_string(frame_count) + ".txt");
        //     std::ofstream local_slam_tri(outputs_path + "local_slam_" + std::to_string(frame_count) + ".txt");
        //     for(int i=0; i<slam_poses_.size(); i++){
        //         Sophus::SE3d T_nwu_rtk = T_yaw_slam * slam_poses_[i] * se3_ex_;
        //         print_pose(nwu_slam_tri, T_nwu_rtk, i);
        //         print_pose(local_slam_tri, slam_poses_[i], i);
        //     }nwu_slam_tri.close(); local_slam_tri.close();

        //     std::ofstream nwu_rtk_tri(outputs_path + "nwu_rtk_" + std::to_string(frame_count) + ".txt");
        //     std::ofstream local_rtk_tri(outputs_path + "local_rtk_" + std::to_string(frame_count) + ".txt");
        //     for(int i=0; i<rtk_poses_.size(); i++){
        //         Eigen::Matrix3d R = rtk_poses_[i].so3().matrix();
        //         const Eigen::Vector3d pt = R_nwu_enu * R_enu_ecef *(rtk_poses_[i].translation() - ref_ecef_);
        //         Sophus::SE3d T_nwu_rtk(R, pt);
        //         print_pose(nwu_rtk_tri, T_nwu_rtk, i);

        //         Sophus::SE3d T_local_slam = T_yaw_slam.inverse() * T_nwu_rtk * se3_ex_.inverse();
        //         print_pose(local_rtk_tri, T_local_slam, i);
        //     }nwu_rtk_tri.close(); local_rtk_tri.close();

        // }
    }
    return aligned_trajectory;
}

std::pair<double, double> TrajectoryAligner::getError() const {
    if (mode_ == CalibrationMode::MODE_3DOF_RTK) {
        // Mode 1: 现有逻辑
        std::vector<Eigen::Vector3d> aligned_trajectory = GetAlignedTrajectory();
        if (aligned_trajectory.empty()) {
            return {0.0, 0.0};
        }
        double error = 0;
        double max_error = 0;
        for (size_t i = 0; i < rtk_trajectory_.size(); ++i) {
            double current_error = (aligned_trajectory[i] - slam_trajectory_[i]).norm();
            error += current_error;
            max_error = std::max(max_error, current_error);
        }
        return {error/slam_trajectory_.size(), max_error};
    } else {
        // Mode 2: SE3模式的误差计算 - 使用对齐后的轨迹
        std::vector<Eigen::Vector3d> aligned_trajectory = GetAlignedTrajectory();
        if (aligned_trajectory.empty()) {
            return {0.0, 0.0};
        }
        double error = 0;
        double max_error = 0;
        for (size_t i = 0; i < rtk_poses_.size(); ++i) {
            // 比较对齐后的RTK轨迹与SLAM轨迹
            double current_error = (aligned_trajectory[i] - slam_poses_[i].translation()).norm();
            error += current_error;
            max_error = std::max(max_error, current_error);
        }
        return {error/rtk_poses_.size(), max_error};
    }
}
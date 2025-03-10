#include "HM_RTK/trajectory_aligner.hpp"
#include <iostream>
#include <HM_RTK/utils.hpp>
#include "gnss_comm/gnss_utility.hpp"

using namespace Eigen;

Eigen::Matrix3d ComputeRotationEcefToEnu(const Eigen::Vector3d ref_ecef) {
    return gnss_comm::ecef2rotation(ref_ecef).transpose();
}

TrajectoryAligner::TrajectoryAligner(
    const std::vector<Vector3d>& rtk_trajectory,
    const std::vector<Vector3d>& slam_trajectory,
    const std::vector<Matrix3d>& R_world_cam)
    : rtk_trajectory_(rtk_trajectory),
      slam_trajectory_(slam_trajectory),
      R_world_cam_(R_world_cam){

        if (rtk_trajectory.empty()) {
            throw std::invalid_argument("Input trajectories cannot be empty");
        }
        if (rtk_trajectory.size() != slam_trajectory.size() || 
            rtk_trajectory.size() != R_world_cam.size()) {
            throw std::invalid_argument("Input trajectories must have same size");
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

// p_{cam} = [R_{yaw}^{-1}R_{ecef}^{enu}(p_{ecef} - ref_{ecef})] - R_{cam}^{world}*t_{ex}
template <typename T>
bool TrajectoryAligner::AlignmentResidual::operator()(
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

bool TrajectoryAligner::Solve() {
    ceres::Problem problem;
    
    // 添加参数块
    problem.AddParameterBlock(&yaw_, 1);
    problem.AddParameterBlock(ref_ecef_.data(), 3);
    problem.AddParameterBlock(t_ex_.data(), 3);
    
    // 设置参数固定
    if (yaw_fixed_) problem.SetParameterBlockConstant(&yaw_);
    if (ref_fixed_) problem.SetParameterBlockConstant(ref_ecef_.data());
    if (ex_fixed_) problem.SetParameterBlockConstant(t_ex_.data());
    
    // 添加残差项
    for (size_t i = 0; i < rtk_trajectory_.size(); ++i) {
        ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<AlignmentResidual, 3, 1, 3, 3>(
                new AlignmentResidual(rtk_trajectory_[i], 
                                    slam_trajectory_[i],
                                    R_world_cam_[i],
                                    y_t_ex_init_));
        
        problem.AddResidualBlock(cost_function, 
                               nullptr, 
                               &yaw_, 
                               ref_ecef_.data(),
                               t_ex_.data());
    }
    
    // 求解配置
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    
    std::cout << summary.BriefReport() << "\n";

    return summary.IsSolutionUsable();
}

std::vector<Eigen::Vector3d> TrajectoryAligner::GetAlignedTrajectory() const{
    std::vector<Eigen::Vector3d> aligned_trajectory;
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
    return aligned_trajectory;
}

std::pair<double, double> TrajectoryAligner::getError() const {
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
}
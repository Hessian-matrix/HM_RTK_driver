// trajectory_aligner.hpp
#ifndef TRAJECTORY_ALIGNER_HPP
#define TRAJECTORY_ALIGNER_HPP

#include <vector>
#include <Eigen/Core>
#include <ceres/ceres.h>

// TrajectoryAligner用于利用RTK与SLAM轨迹数据，通过ceres进行外参优化
class TrajectoryAligner {
public:
    // 构造函数要求输入的轨迹数据均大小相同
    TrajectoryAligner(const std::vector<Eigen::Vector3d>& rtk_trajectory,
                      const std::vector<Eigen::Vector3d>& slam_trajectory,
                      const std::vector<Eigen::Matrix3d>& R_world_cam);

    // 参数初始化接口
    void SetInitialYaw(double yaw) { yaw_ = yaw; }
    void SetInitialRef(const Eigen::Vector3d& ref) { ref_ecef_ = ref; }
    // 同时设置t_ex以及内部用于残差计算的y方向初值
    void SetInitialEx(const Eigen::Vector3d& ex) { t_ex_ = ex; y_t_ex_init_ = ex[1]; }

    // 参数固定设置
    void SetRefFixed(bool fixed) { ref_fixed_ = fixed; }
    void SetExFixed(bool fixed) { ex_fixed_ = fixed; }
    void SetYawFixed(bool fixed) { yaw_fixed_ = fixed; }

    // 优化求解，返回true表示解有效
    bool Solve();

    // 结果接口：获取yaw、参考ECEF及外参
    double GetYaw() const { return yaw_; }
    Eigen::Vector3d GetRef() const { return ref_ecef_; }
    Eigen::Vector3d GetEx() const { return Eigen::Vector3d(t_ex_[0], t_ex_[1], t_ex_[2]); }

    // 获取对齐后轨迹数据及误差（平均误差和最大误差）
    std::vector<Eigen::Vector3d> GetAlignedTrajectory() const;
    std::pair<double, double> getError() const;

private:
    // 定义残差计算类，供ceres AutoDiff使用
    struct AlignmentResidual {
        AlignmentResidual(const Eigen::Vector3d& rtk_pose,
                          const Eigen::Vector3d& slam_pose,
                          const Eigen::Matrix3d& R_world_cam,
                          const double& y_t_ex_init)
            : rtk_pose_(rtk_pose), slam_pose_(slam_pose),
              R_world_cam_(R_world_cam), y_t_ex_init_(y_t_ex_init) {}

        template <typename T>
        bool operator()(const T* const yaw,
                        const T* const ref_ecef,
                        const T* const t_ex,
                        T* residual) const;

        Eigen::Vector3d rtk_pose_;
        Eigen::Vector3d slam_pose_;
        Eigen::Matrix3d R_world_cam_;
        double y_t_ex_init_;
    };

    // 参数存储
    double yaw_ = 0.0;
    Eigen::Vector3d ref_ecef_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d t_ex_ = Eigen::Vector3d::Zero();
    double y_t_ex_init_ = 0.0;

    // 输入数据引用（注意：这些引用须在构造时初始化）
    const std::vector<Eigen::Vector3d>& rtk_trajectory_;
    const std::vector<Eigen::Vector3d>& slam_trajectory_;
    const std::vector<Eigen::Matrix3d>& R_world_cam_;

    // 固定标志
    bool ref_fixed_ = false;
    bool ex_fixed_ = false;
    bool yaw_fixed_ = false;
};

#endif // TRAJECTORY_ALIGNER_HPP
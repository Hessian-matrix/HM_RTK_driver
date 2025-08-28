// trajectory_aligner.hpp
#ifndef TRAJECTORY_ALIGNER_HPP
#define TRAJECTORY_ALIGNER_HPP

#include <vector>
#include <Eigen/Core>
#include <ceres/ceres.h>
#include <sophus/se3.hpp>

enum class CalibrationMode {
    MODE_3DOF_RTK = 0,  // 现有模式：3DOF RTK (仅位置)
    MODE_6DOF_RTK = 1   // 升级模式：6DOF RTK (位置+姿态)
};

// TrajectoryAligner用于利用RTK与SLAM轨迹数据，通过ceres进行外参优化
class TrajectoryAligner {
public:
    // Mode 1: 3DOF RTK构造函数
    TrajectoryAligner(const std::vector<Eigen::Vector3d>& rtk_trajectory,
                      const std::vector<Eigen::Vector3d>& slam_trajectory,
                      const std::vector<Eigen::Matrix3d>& R_world_cam);

    // Mode 2: 6DOF RTK构造函数
    TrajectoryAligner(const std::vector<Sophus::SE3d>& rtk_poses,
                      const std::vector<Sophus::SE3d>& slam_poses);

    // 参数初始化接口
    void SetInitialYaw(double yaw) { yaw_ = yaw; }
    void SetInitialRef(const Eigen::Vector3d& ref) { ref_ecef_ = ref; }
    void SetInitialEx(const Eigen::Vector3d& ex) { t_ex_ = ex; y_t_ex_init_ = ex[1]; }
    
    // 新增：SE3外参初始化
    void SetInitialSE3Ex(const Sophus::SE3d& se3_ex) { 
        se3_ex_ = se3_ex; 
        t_ex_ = se3_ex_.translation();
        y_t_ex_init_ = t_ex_[1];
    }

    // 参数固定设置
    void SetRefFixed(bool fixed) { ref_fixed_ = fixed; }
    void SetExFixed(bool fixed) { ex_fixed_ = fixed; }
    void SetYawFixed(bool fixed) { yaw_fixed_ = fixed; }
    void SetSE3ExFixed(bool fixed) { se3_ex_fixed_ = fixed; }  // 新增

    // 优化求解，返回true表示解有效
    bool Solve();

    // 结果获取
    double GetYaw() const { return yaw_; }
    Eigen::Vector3d GetRef() const { return ref_ecef_; }
    Eigen::Vector3d GetEx() const { return t_ex_; }
    Sophus::SE3d GetSE3Ex() const { return se3_ex_; }  // 新增
    CalibrationMode GetMode() const { return mode_; }  // 新增

    // 获取对齐后轨迹数据及误差（平均误差和最大误差）
    std::vector<Eigen::Vector3d> GetAlignedTrajectory() const;
    std::pair<double, double> getError() const;

private:
    // Mode 1: 位置对齐残差函数 (现有)
    struct PositionAlignmentResidual {
        PositionAlignmentResidual(const Eigen::Vector3d& rtk_pose,
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

    // Mode 2: SE3对齐残差函数 (分离参数化版本)
    struct SE3AlignmentResidual {
        SE3AlignmentResidual(const Sophus::SE3d& rtk_pose,
                            const Sophus::SE3d& slam_pose,
                            double y_t_ex_init)
            : rtk_pose_(rtk_pose), 
              slam_pose_(slam_pose),
              y_t_ex_init_(y_t_ex_init) {}

        template <typename T>
        bool operator()(const T* const yaw,
                        const T* const ref_ecef,
                        const T* const quat_ex,     // 外参四元数 [x,y,z,w]
                        const T* const trans_ex,    // 外参平移 [x,y,z]
                        T* residual) const;

        Sophus::SE3d rtk_pose_;
        Sophus::SE3d slam_pose_;
        double y_t_ex_init_;
    };

    CalibrationMode mode_;
    
    // 通用参数
    double yaw_ = 0.0;
    Eigen::Vector3d ref_ecef_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d t_ex_ = Eigen::Vector3d::Zero();
    double y_t_ex_init_ = 0.0;
    
    // Mode 2 专用参数
    Sophus::SE3d se3_ex_ = Sophus::SE3d();
    
    // Mode 1 数据存储
    std::vector<Eigen::Vector3d> rtk_trajectory_;
    std::vector<Eigen::Vector3d> slam_trajectory_;
    std::vector<Eigen::Matrix3d> R_world_cam_;
    
    // Mode 2 数据存储
    std::vector<Sophus::SE3d> rtk_poses_;
    std::vector<Sophus::SE3d> slam_poses_;
    
    // 固定标志
    bool ref_fixed_ = false;
    bool ex_fixed_ = false;
    bool yaw_fixed_ = false;
    bool se3_ex_fixed_ = false;
};

#endif // TRAJECTORY_ALIGNER_HPP
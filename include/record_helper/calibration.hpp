#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>

#include "record_helper/types.hpp"

namespace rh::cal {

// R_AB 把 B 中向量旋到 A，p_AB 是 B 原点在 A 中；Compose(A→B, B→C) 得到 A→C。
[[nodiscard]] RigidTransform Compose(const RigidTransform& a_to_b, const RigidTransform& b_to_c);
[[nodiscard]] RigidTransform Invert(const RigidTransform& a_to_b);

// 极分解（SVD）把设备给的 float 精度矩阵拉回正交阵；返回修正前的最大元素误差。
double OrthonormalizeRotation(Eigen::Matrix3d* rotation);
[[nodiscard]] double RotationError(const Eigen::Matrix3d& rotation);

// EuRoC 的 T_BS：行主序 4x4、行步长 4，因此旋转在索引 0,1,2/4,5,6/8,9,10，平移在 3,7,11。
[[nodiscard]] std::vector<double> FlattenT_BS(const RigidTransform& camera_to_body);
[[nodiscard]] RigidTransform TransformFromT_BS(const std::vector<double>& flat);

struct StereoGeometrySummary {
    double baseline_m{0.0};
    // 两目光轴的相对夹角：接近 0 说明设备交付的已经是修正过的平行图像（rectified）。
    double relative_rotation_deg{0.0};
    // 基线在右目系 y/z 分量相对 x 分量的比例：平行修正后应接近 0。
    double off_axis_baseline_ratio{0.0};
    double max_abs_distortion{0.0};
    double reported_k3_max{0.0};
    std::string interpretation;
};

StereoGeometrySummary SummarizeStereoGeometry(const BodyCalibration& calibration);

struct ValidationReport {
    std::vector<std::string> problems;
    std::vector<std::string> warnings;
    [[nodiscard]] bool ok() const { return problems.empty(); }
};

// 复刻 unav_vio data_contracts.cpp::validate_calibration 的判据，外加录制端才有的检查
// （例如两目光轴相对角过大意味着像素对不是同一时刻的同一视角，回放会卡在极线门控）。
ValidationReport
ValidateCalibration(const BodyCalibration& calibration, const ImuNoise& noise, double expected_g);

// 静止段 ‖a‖ 均值与 unav_vio 侧硬编码 g 的一致性：偏差超过 max_static_accel_norm_error 时
// 初始化会拒绝零速度假设，因此这是「能不能初始化」的前置条件，而不是精度问题。
struct GravityCheck {
    double mean_norm{0.0};
    double error{0.0};
    bool pass{false};
};
GravityCheck CheckGravity(double mean_accel_norm, double expected_g, double tolerance);

// 静止标定阶段设备是平放、镜头朝上的，所以世界上就是光学系 +z，静止段平均比力（重力反力，
// 指向世界上）在 body 系里必须落在 T_BS 旋转的第三列上。
//
// 这条查的是「IMU 数据到底交在哪个系里」，与 CheckGravity 的模长检查正交：设备外参表、SDK
// 取数路径、内核驱动三者中任何一个换了轴向口径，模长都照样好看，而这条会直接跳到 90°/180°。
// 恒等外参本身不一定错——它取决于该路径交数据的系，因此只能实测，不能照抄文档。
struct GravityDirectionCheck {
    // 实测平均比力与 T_BS 第三列的夹角，度。measured_valid 为 false 时无意义。
    double angle_deg{180.0};
    Eigen::Vector3d measured{Eigen::Vector3d::Zero()};
    Eigen::Vector3d expected{Eigen::Vector3d::Zero()};
    bool measured_valid{false};
    bool pass{false};
};
GravityDirectionCheck CheckGravityDirection(const Eigen::Vector3d& mean_accel,
                                            const Eigen::Matrix3d& camera_to_body_rotation,
                                            double tolerance_deg);

} // namespace rh::cal

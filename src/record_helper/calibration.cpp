#include "record_helper/calibration.hpp"

#include <cmath>

namespace rh::cal {
namespace {

constexpr double kRotationTolerance = 1e-6;
constexpr double kMinimumBaselineMeters = 1e-9;

bool Finite(const Eigen::Vector3d& value) {
    return std::isfinite(value.x()) && std::isfinite(value.y()) && std::isfinite(value.z());
}

void ValidateCamera(const CameraIntrinsics& camera,
                    const std::string& name,
                    ValidationReport* report) {
    if (camera.width == 0 || camera.height == 0) {
        report->problems.push_back(name + ": 分辨率缺失或为 0");
        return;
    }
    if (!std::isfinite(camera.fx) || !std::isfinite(camera.fy) || camera.fx <= 0.0 ||
        camera.fy <= 0.0) {
        report->problems.push_back(name + ": fx/fy 必须有限且为正");
    }
    if (!(camera.cx >= 0.0) || camera.cx >= static_cast<double>(camera.width)) {
        report->problems.push_back(name + ": cx 必须落在 [0, width) 内");
    }
    if (!(camera.cy >= 0.0) || camera.cy >= static_cast<double>(camera.height)) {
        report->problems.push_back(name + ": cy 必须落在 [0, height) 内");
    }
    for (const double coefficient : camera.distortion) {
        if (!std::isfinite(coefficient)) {
            report->problems.push_back(name + ": 畸变系数必须是有限数");
            break;
        }
    }
}

void ValidateTransform(const RigidTransform& transform,
                       const std::string& name,
                       ValidationReport* report) {
    if (!Finite(transform.t)) {
        report->problems.push_back(name + ": 平移含非有限数");
    }
    const double error = RotationError(transform.r);
    if (error > kRotationTolerance) {
        report->problems.push_back(name + ": 旋转不满足正交性，误差 " + std::to_string(error) +
                                   " > 1e-6（unav_vio 会直接拒绝该标定）");
    }
    if (std::abs(transform.r.determinant() - 1.0) > kRotationTolerance) {
        report->problems.push_back(name + ": 旋转行列式偏离 +1");
    }
}

} // namespace

RigidTransform Compose(const RigidTransform& a_to_b, const RigidTransform& b_to_c) {
    RigidTransform result;
    result.r = b_to_c.r * a_to_b.r;
    result.t = b_to_c.r * a_to_b.t + b_to_c.t;
    return result;
}

RigidTransform Invert(const RigidTransform& a_to_b) {
    RigidTransform result;
    result.r = a_to_b.r.transpose();
    result.t = -(result.r * a_to_b.t);
    return result;
}

double OrthonormalizeRotation(Eigen::Matrix3d* rotation) {
    if (rotation == nullptr) {
        return 0.0;
    }
    const double before = RotationError(*rotation);
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(*rotation, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d corrected = svd.matrixU() * svd.matrixV().transpose();
    if (corrected.determinant() < 0.0) {
        // 反射不是旋转：翻转最小奇异值对应的轴，保证 det = +1。
        Eigen::Matrix3d diagonal = Eigen::Matrix3d::Identity();
        diagonal(2, 2) = -1.0;
        corrected = svd.matrixU() * diagonal * svd.matrixV().transpose();
    }
    *rotation = corrected;
    return before;
}

double RotationError(const Eigen::Matrix3d& rotation) {
    return (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff();
}

std::vector<double> FlattenT_BS(const RigidTransform& camera_to_body) {
    const Eigen::Matrix3d& r = camera_to_body.r;
    const Eigen::Vector3d& t = camera_to_body.t;
    // 行主序 4x4：上游按 t[0..2],t[4..6],t[8..10] 取旋转、t[3],t[7],t[11] 取平移。
    return {r(0, 0),
            r(0, 1),
            r(0, 2),
            t.x(),
            r(1, 0),
            r(1, 1),
            r(1, 2),
            t.y(),
            r(2, 0),
            r(2, 1),
            r(2, 2),
            t.z(),
            0.0,
            0.0,
            0.0,
            1.0};
}

RigidTransform TransformFromT_BS(const std::vector<double>& flat) {
    RigidTransform transform;
    if (flat.size() != 16) {
        return transform;
    }
    transform.r << flat[0], flat[1], flat[2], flat[4], flat[5], flat[6], flat[8], flat[9], flat[10];
    transform.t = Eigen::Vector3d(flat[3], flat[7], flat[11]);
    return transform;
}

StereoGeometrySummary SummarizeStereoGeometry(const BodyCalibration& calibration) {
    StereoGeometrySummary summary;
    const Eigen::Matrix3d& r0 = calibration.camera0_to_body.r;
    const Eigen::Matrix3d& r1 = calibration.camera1_to_body.r;
    const Eigen::Matrix3d relative = r0.transpose() * r1; // C0 系看 C1 的姿态
    const double cosine = std::max(-1.0, std::min(1.0, (relative.trace() - 1.0) / 2.0));
    summary.relative_rotation_deg = std::acos(cosine) * 180.0 / M_PI;

    const Eigen::Vector3d baseline_in_c0 =
        r0.transpose() * (calibration.camera1_to_body.t - calibration.camera0_to_body.t);
    summary.baseline_m = baseline_in_c0.norm();
    const double along_axis = std::abs(baseline_in_c0.x());
    const double across_axis = std::hypot(baseline_in_c0.y(), baseline_in_c0.z());
    summary.off_axis_baseline_ratio = along_axis > 1e-12 ? across_axis / along_axis : 1.0;

    for (const CameraIntrinsics* camera : {&calibration.camera0, &calibration.camera1}) {
        for (const double coefficient : camera->distortion) {
            summary.max_abs_distortion =
                std::max(summary.max_abs_distortion, std::abs(coefficient));
        }
        summary.reported_k3_max = std::max(summary.reported_k3_max, std::abs(camera->reported_k3));
    }

    if (summary.relative_rotation_deg < 0.05 && summary.off_axis_baseline_ratio < 0.01) {
        summary.interpretation =
            "两目光轴平行且基线近似水平：设备交付的红外图已是修正图，畸变应写 0 或使用设备报告值";
    } else if (summary.max_abs_distortion > 0.0) {
        summary.interpretation = "两目存在非平凡的相对旋转：交付图未修正，需连同畸变系数一起使用";
    } else {
        summary.interpretation = "几何关系介于两者之间，建议用 rs verify 的极线门控再判断";
    }
    return summary;
}

ValidationReport
ValidateCalibration(const BodyCalibration& calibration, const ImuNoise& noise, double expected_g) {
    ValidationReport report;
    ValidateCamera(calibration.camera0, "camera0", &report);
    ValidateCamera(calibration.camera1, "camera1", &report);
    if (calibration.camera0.width != calibration.camera1.width ||
        calibration.camera0.height != calibration.camera1.height) {
        report.warnings.push_back("两目分辨率不同：产品前端可工作，但与既有回放夹具的假设不一致");
    }
    ValidateTransform(calibration.camera0_to_body, "camera0_to_body", &report);
    ValidateTransform(calibration.camera1_to_body, "camera1_to_body", &report);

    const double baseline = (calibration.camera1_to_body.t - calibration.camera0_to_body.t).norm();
    if (!(baseline > kMinimumBaselineMeters)) {
        report.problems.push_back("基线为 0：双目退化，unav_vio 会拒绝标定");
    } else if (baseline < 0.01 || baseline > 0.3) {
        report.warnings.push_back("基线 " + std::to_string(baseline) + " m 超出常见双目范围");
    }

    const auto require_positive = [&](const std::string& name, double value) {
        if (!std::isfinite(value) || value <= 0.0) {
            report.problems.push_back(name + " 必须是有限的正数（unav_vio 强制 > 0）");
        }
    };
    require_positive("sigma_a", noise.sigma_a);
    require_positive("sigma_g", noise.sigma_g);
    require_positive("sigma_ba", noise.sigma_ba);
    require_positive("sigma_bg", noise.sigma_bg);
    require_positive("g", noise.g);
    if (std::abs(noise.g - expected_g) > 1e-9) {
        report.warnings.push_back("g 与 unav_vio 回放侧硬编码的 " + std::to_string(expected_g) +
                                  " 不一致，静止初始化会用 9.81 判零速度");
    }
    if (!noise.measured) {
        report.warnings.push_back("IMU 噪声参数不是本次实测值（回放能吃，但权重不可信）");
    }
    return report;
}

GravityCheck CheckGravity(double mean_accel_norm, double expected_g, double tolerance) {
    GravityCheck check;
    check.mean_norm = mean_accel_norm;
    check.error = mean_accel_norm - expected_g;
    check.pass = std::abs(check.error) <= tolerance;
    return check;
}

GravityDirectionCheck CheckGravityDirection(const Eigen::Vector3d& mean_accel,
                                            const Eigen::Matrix3d& camera_to_body_rotation,
                                            double tolerance_deg) {
    GravityDirectionCheck check;
    check.measured = mean_accel;
    check.expected = camera_to_body_rotation.col(2);
    const double measured_norm = mean_accel.norm();
    const double expected_norm = check.expected.norm();
    // 静止段一个样本都没有（或外参退化）时不下结论，交给调用方按「未测」处理。
    if (!Finite(mean_accel) || !Finite(check.expected) || measured_norm < 1.0 ||
        expected_norm < 1e-6) {
        return check;
    }
    check.measured_valid = true;
    const double cosine = std::max(
        -1.0, std::min(1.0, mean_accel.dot(check.expected) / (measured_norm * expected_norm)));
    check.angle_deg = std::acos(cosine) * 180.0 / M_PI;
    check.pass = check.angle_deg <= tolerance_deg;
    return check;
}

} // namespace rh::cal

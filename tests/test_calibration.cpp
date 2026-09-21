#include "check.hpp"
#include "record_helper/calibration.hpp"
#include "synthetic.hpp"

#include <cmath>
#include <vector>

using rh::BodyCalibration;
using rh::ImuNoise;
using rh::RigidTransform;
using rh::testing::SyntheticCalibration;
using rh::testing::SyntheticNoise;

namespace {

RigidTransform CameraToBodyStub() {
    Eigen::Matrix3d rotation;
    rotation << 0.0, 0.0, -1.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0;
    RigidTransform transform;
    transform.r = rotation;
    transform.t = Eigen::Vector3d(-0.0055, 0.021, 0.0117);
    return transform;
}

} // namespace

RH_TEST(calibration_t_bs_flatten_round_trip) {
    const auto source = CameraToBodyStub();
    const auto flat = rh::cal::FlattenT_BS(source);
    CHECK(flat.size() == 16);
    const auto restored = rh::cal::TransformFromT_BS(flat);
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            CHECK_NEAR(restored.r(row, column), source.r(row, column), 1e-15);
        }
    }
    // 上游按行主序 4x4、行步长 4 取：旋转在 0,1,2/4,5,6/8,9,10，平移在 3,7,11。
    CHECK_NEAR(flat[3], source.t.x(), 1e-15);
    CHECK_NEAR(flat[7], source.t.y(), 1e-15);
    CHECK_NEAR(flat[11], source.t.z(), 1e-15);
    CHECK_NEAR(flat[15], 1.0, 0.0);
    CHECK_NEAR(restored.t.z(), source.t.z(), 1e-15);
}

RH_TEST(calibration_compose_and_invert) {
    const auto camera_to_body = CameraToBodyStub();
    const auto round_trip = rh::cal::Compose(camera_to_body, rh::cal::Invert(camera_to_body));
    CHECK_NEAR(rh::cal::RotationError(round_trip.r), 0.0, 1e-12);
    CHECK_NEAR(round_trip.t.norm(), 0.0, 1e-12);
    // Compose(A→B, B→C) 与手写矩阵乘法一致：p_B = R·p_A + t。
    const Eigen::Vector3d point(0.1, -0.2, 0.3);
    const Eigen::Vector3d direct = camera_to_body.r * point + camera_to_body.t;
    const auto identity = RigidTransform{};
    const auto composed = rh::cal::Compose(rh::cal::Compose(identity, camera_to_body), identity);
    CHECK_NEAR((composed.r * point + composed.t - direct).norm(), 0.0, 1e-12);
}

RH_TEST(calibration_orthonormalize) {
    // 设备外参是 float 精度，直接写会踩上游的 1e-6 正交性验收。
    Eigen::Matrix3d rotation;
    rotation << 1.0, 3e-3, -2e-3, -3e-3, 1.0, 5e-3, 2e-3, -5e-3, 1.0;
    CHECK(rh::cal::RotationError(rotation) > 1e-6);
    rh::cal::OrthonormalizeRotation(&rotation);
    CHECK(rh::cal::RotationError(rotation) <= 1e-9);
    CHECK_NEAR(rotation.determinant(), 1.0, 1e-9);
}

RH_TEST(calibration_validation_catches_replay_blockers) {
    BodyCalibration calibration = SyntheticCalibration();
    ImuNoise noise = SyntheticNoise();
    CHECK(rh::cal::ValidateCalibration(calibration, noise, 9.81).ok());

    calibration.camera0.fx = -1.0;
    CHECK(!rh::cal::ValidateCalibration(calibration, noise, 9.81).ok());
    calibration.camera0.fx = 110.0;

    calibration.camera1.cx = 1e6;
    CHECK(!rh::cal::ValidateCalibration(calibration, noise, 9.81).ok());

    calibration = SyntheticCalibration();
    calibration.camera1_to_body = calibration.camera0_to_body; // 零基线
    CHECK(!rh::cal::ValidateCalibration(calibration, noise, 9.81).ok());

    calibration = SyntheticCalibration();
    calibration.camera1_to_body.r = Eigen::Matrix3d::Identity() * 1.5; // 非正交
    CHECK(!rh::cal::ValidateCalibration(calibration, noise, 9.81).ok());

    calibration = SyntheticCalibration();
    noise.sigma_bg = 0.0;
    CHECK(!rh::cal::ValidateCalibration(calibration, noise, 9.81).ok()); // 上游要求严格 > 0
    noise = SyntheticNoise();
    noise.measured = false;
    const auto report = rh::cal::ValidateCalibration(calibration, noise, 9.81);
    CHECK(report.ok());
    CHECK(!report.warnings.empty());
}

RH_TEST(calibration_stereo_geometry_summary) {
    const auto summary = rh::cal::SummarizeStereoGeometry(SyntheticCalibration());
    CHECK_NEAR(summary.baseline_m, 0.05, 1e-9);
    CHECK_NEAR(summary.relative_rotation_deg, 0.0, 1e-9);
    CHECK_NEAR(summary.off_axis_baseline_ratio, 0.0, 1e-9);

    // 未修正的双目：两目光轴有夹角，且基线不再只沿相机 x 轴。
    BodyCalibration tilted = SyntheticCalibration();
    tilted.camera1_to_body.r =
        tilted.camera0_to_body.r *
        Eigen::AngleAxisd(2.0 * M_PI / 180.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
    tilted.camera1_to_body.t = tilted.camera0_to_body.t +
                               tilted.camera0_to_body.r * Eigen::Vector3d(-0.05, 0.0, 0.0) +
                               Eigen::Vector3d(0.0, 0.0, 0.006);
    const auto tilted_summary = rh::cal::SummarizeStereoGeometry(tilted);
    CHECK(tilted_summary.relative_rotation_deg > 1.0);
    CHECK(tilted_summary.off_axis_baseline_ratio > 0.01);
}

RH_TEST(calibration_gravity_check) {
    CHECK(rh::cal::CheckGravity(9.81, 9.81, 0.5).pass);
    CHECK(rh::cal::CheckGravity(9.45, 9.81, 0.5).pass);
    CHECK(!rh::cal::CheckGravity(8.9, 9.81, 0.5).pass);
    CHECK_NEAR(rh::cal::CheckGravity(10.0, 9.81, 0.5).error, 0.19, 1e-9);
}

RH_TEST(calibration_gravity_direction_check) {
    const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
    constexpr double kTol = 25.0;

    // 平放镜头朝上 + 恒等外参：世界上 = 光学 +z = body +z。
    const auto aligned =
        rh::cal::CheckGravityDirection(Eigen::Vector3d(0.0, 0.0, 9.81), identity, kTol);
    CHECK(aligned.measured_valid);
    CHECK(aligned.pass);
    CHECK_NEAR(aligned.angle_deg, 0.0, 1e-9);

    // 模长完全正常、方向差 90°/180°：CheckGravity 拦不住，这条必须拦住。
    CHECK(!rh::cal::CheckGravityDirection(Eigen::Vector3d(0.0, 9.81, 0.0), identity, kTol).pass);
    CHECK(!rh::cal::CheckGravityDirection(Eigen::Vector3d(0.0, 0.0, -9.81), identity, kTol).pass);
    CHECK_NEAR(
        rh::cal::CheckGravityDirection(Eigen::Vector3d(0.0, 0.0, -9.81), identity, kTol).angle_deg,
        180.0,
        1e-6);

    // 外参是约 90° 置换时，同一份数据就该换一种摆法才对：跟着第三列走即通过。
    const Eigen::Matrix3d permuted =
        Eigen::AngleAxisd(-90.0 * M_PI / 180.0, Eigen::Vector3d::UnitX()).toRotationMatrix();
    const Eigen::Vector3d expected_body = permuted.col(2) * 9.81;
    CHECK(rh::cal::CheckGravityDirection(expected_body, permuted, kTol).pass);

    // 手摆歪 10° 不算错；容差内放行。
    const Eigen::Vector3d tilted =
        Eigen::AngleAxisd(10.0 * M_PI / 180.0, Eigen::Vector3d::UnitX()).toRotationMatrix() *
        Eigen::Vector3d(0.0, 0.0, 9.81);
    const auto tilted_check = rh::cal::CheckGravityDirection(tilted, identity, kTol);
    CHECK_NEAR(tilted_check.angle_deg, 10.0, 1e-6);
    CHECK(tilted_check.pass);

    // 静止段没样本 / 数据退化时不下结论。
    CHECK(!rh::cal::CheckGravityDirection(Eigen::Vector3d::Zero(), identity, kTol).measured_valid);
    CHECK(!rh::cal::CheckGravityDirection(Eigen::Vector3d(0.0, 0.0, 0.5), identity, kTol)
               .measured_valid);
    CHECK(!rh::cal::CheckGravityDirection(Eigen::Vector3d(NAN, 0.0, 9.81), identity, kTol)
               .measured_valid);
}

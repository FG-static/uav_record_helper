#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace rh {

// 时间一律使用录制起点的相对纳秒（非负、严格递增）。unav_vio 的 TimePoint 就是 int64 ns，
// 只要求 >= 0，且全程只做差值，所以不需要 Unix epoch。
using TimeNs = std::int64_t;

// 相机内参：pinhole + radtan，畸变顺序 k1、k2、p1、p2（unav_vio 不支持 k3）。
// 像素是畸变坐标，u 向右、v 向下，必须与真实落盘图像的分辨率一致。
struct CameraIntrinsics {
    std::uint32_t width{0};
    std::uint32_t height{0};
    double fx{0.0};
    double fy{0.0};
    double cx{0.0};
    double cy{0.0};
    std::array<double, 4> distortion{{0.0, 0.0, 0.0, 0.0}};
    // 设备原始报告的 k3，仅用于清单留档；写出时必须是 0 才允许保留。
    double reported_k3{0.0};
    std::string distortion_model;
    std::string frame_id;
};

// R 把 from 系中的向量旋转到 to 系，p 是 from 原点在 to 系中的坐标（unav_vio 的 R_AB 约定）。
// 因此 camera_to_body 就是 R_BC / p_BC：相机光学系相对 IMU/body 系。
struct RigidTransform {
    Eigen::Matrix3d r{Eigen::Matrix3d::Identity()};
    Eigen::Vector3d t{Eigen::Vector3d::Zero()};
};

// 加表系为 body(B)。陀螺与加表各自的设备外参都留档，但回放契约只有一个 B 系。
struct BodyCalibration {
    CameraIntrinsics camera0;
    CameraIntrinsics camera1;
    RigidTransform camera0_to_body;
    RigidTransform camera1_to_body;
    RigidTransform gyro_to_body{RigidTransform{}};
    bool gyro_matches_accel_frame{true};
    std::string device_serial;
    std::string device_name;
    std::string firmware;
    std::string calibration_device_serial;
};

// IMU 噪声参数。sigma_* 是噪声密度，sigma_b* 是 bias 随机游走；g 供 unav_vio 侧硬编码的
// 9.81 对齐检查使用（录制端只能校验 ‖a‖ 与之相符，不能改写）。
struct ImuNoise {
    double sigma_a{0.0};
    double sigma_g{0.0};
    double sigma_ba{0.0};
    double sigma_bg{0.0};
    double g{9.81};
    bool measured{false};
};

struct GyroSample {
    TimeNs t_ns{0};
    Eigen::Vector3d w{Eigen::Vector3d::Zero()};
};

struct AccelSample {
    TimeNs t_ns{0};
    Eigen::Vector3d a{Eigen::Vector3d::Zero()};
};

// 统一栅格后的一行，即 imu0/data.csv 的一条记录；gyro 与 accel 必须共用同一时刻。
struct ImuRow {
    TimeNs t_ns{0};
    Eigen::Vector3d w{Eigen::Vector3d::Zero()};
    Eigen::Vector3d a{Eigen::Vector3d::Zero()};
};

// 一对硬同步双目红外图像。t_ns 是写进两份 data.csv 的共享曝光时间戳（取 Camera0 为权威）；
// t1_ns 保留设备报告的右目时间戳，仅用于统计左右偏斜并在 manifest 中留档。
struct StereoPairRecord {
    TimeNs t_ns{0};
    TimeNs t1_ns{0};
    std::uint64_t frame_counter0{0};
    std::uint64_t frame_counter1{0};
    std::vector<std::uint8_t> left;
    std::vector<std::uint8_t> right;
};

struct DepthRecord {
    TimeNs t_ns{0};
    std::uint32_t width{0};
    std::uint32_t height{0};
    std::vector<std::uint16_t> millimetres;
};

// 设备端 HL-SLAM 位姿流，作为伪 GT 使用。t_ns/p/q 已换算到录制起点相对时间与 IMU(body) 系。
struct PoseRecord {
    TimeNs t_ns{0};
    Eigen::Vector3d p{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};
    double confidence{0.0};
};

struct ImuAlignStats {
    std::size_t gyro_raw{0};
    std::size_t accel_raw{0};
    std::size_t rows{0};
    std::size_t dropped_non_increasing{0};
    std::size_t accel_interpolated{0};
    double grid_dt_ms{0.0};
    double max_gap_ms{0.0};
    // 相邻两行加表完全相同的比例。room_02 的这个值是 0.38，即零阶保持伪造出来的常量段。
    double accel_hold_ratio{0.0};
};

struct CaptureStats {
    std::uint64_t pairs_queued{0};
    std::uint64_t pairs_written{0};
    std::uint64_t pairs_dropped_queue_full{0};
    std::uint64_t pairs_dropped_not_paired{0};
    std::uint64_t depth_written{0};
    std::uint64_t pose_samples{0};
    double max_stereo_skew_ms{0.0};
    double last_pair_skew_ms{0.0};
};

// 录制会话与校验共享的运行参数。
struct SessionLimits {
    // unav_vio 回放侧使用的阈值；数值与 AssemblerConfig / StaticMotionProbeConfig 对齐。
    double max_imu_gap_ms{10.0};
    double max_frame_gap_ms{150.0};
    double max_stereo_sync_error_us{100.0};
    TimeNs imu_lead_ns{55'000'000};
    double probe_window_s{0.5};
    std::size_t probe_static_run{3};
    std::size_t probe_motion_run{5};
    double probe_max_static_rms{0.10};
    double probe_min_motion_rms{0.25};
    double probe_motion_lead_s{4.0};
    double probe_feed_lead_s{0.75};
    double anchor_offset_s{0.06};
};

} // namespace rh

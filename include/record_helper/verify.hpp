#pragma once

#include <optional>
#include <string>
#include <vector>

#include "record_helper/calibration.hpp"
#include "record_helper/motion_gate.hpp"
#include "record_helper/types.hpp"

namespace rh {

enum class CheckStatus { Pass, Warn, Fail };

struct CheckResult {
    CheckStatus status{CheckStatus::Pass};
    std::string id;
    std::string detail;
};

struct VerifyOptions {
    // 回放测试目前硬性要求 GT 文件存在（vio_mh01_replay_test.cpp 的加载断言）。
    // --no-gt 录制时把它降级为 warning，其余检查照旧。
    bool require_ground_truth{true};
    bool check_images{true};
    std::size_t image_sample{64};
    // vio_mh01_replay_test.cpp:499 硬要求 IMU 行数 > 30000，否则整条回放在加载阶段就判失败。
    std::size_t min_imu_rows{30001};
    SessionLimits limits;
    ProbeConfig probe;
    double expected_g{9.81};
    double static_accel_norm_tolerance{0.50};
};

struct VerificationReport {
    std::string dataset_root;
    std::vector<CheckResult> checks;

    std::size_t cam0_rows{0};
    std::size_t cam1_rows{0};
    std::size_t imu_rows{0};
    std::size_t gt_rows{0};
    std::size_t depth_rows{0};
    std::size_t synced_pairs{0};
    std::size_t dropped_pairs{0};
    std::size_t anchor_index{0};
    std::size_t available_frames{0};
    double max_imu_gap_ms{0.0};
    double mean_imu_gap_ms{0.0};
    double accel_hold_ratio{0.0};
    double gyro_hold_ratio{0.0};
    double max_frame_gap_ms{0.0};
    double static_accel_norm{0.0};
    TimeNs imu_begin_ns{0};
    TimeNs imu_end_ns{0};
    TimeNs first_pair_ns{0};
    TimeNs last_pair_ns{0};
    std::optional<StaticMotionBoundary> boundary;
    ImuNoise noise;
    BodyCalibration calibration;
    bool calibration_parsed{false};

    [[nodiscard]] std::size_t Count(CheckStatus status) const;
    [[nodiscard]] bool ok() const { return Count(CheckStatus::Fail) == 0; }
    void Add(CheckStatus status, std::string id, std::string detail);
};

// 直接按 unav_vio 回放侧的解析与同步规则重读一遍磁盘上的数据集：
// 通过即表示 integration.mh01_replay 的输入契约成立（不含估计精度）。
VerificationReport VerifyDataset(const std::string& dataset_root, const VerifyOptions& options);

void PrintReport(const VerificationReport& report, std::FILE* stream);

} // namespace rh

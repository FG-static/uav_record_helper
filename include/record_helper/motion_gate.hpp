#pragma once

#include <optional>
#include <string>
#include <vector>

#include "record_helper/types.hpp"

namespace rh {

// 与 unav_vio tests/integration/euroc_dataset.cpp 的 find_static_motion_boundary 同构：
// 0.5 s 窗口、按均值偏差求 RMS、≥3 个静止窗后 8 个窗内累计 5 个运动窗即构成边界，
// 并刻意保留「最后一个」边界（上游循环就是最后一次赋值胜出）。
struct ProbeConfig {
    double window_s{0.5};
    std::size_t static_run{3};
    std::size_t motion_run{5};
    double max_accel_rms{0.10};
    double max_gyro_rms{0.10};
    double min_motion_rms{0.25};
    double motion_lead_s{4.0};
    double feed_lead_s{0.75};
    std::size_t min_window_samples{4};
};

struct WindowStat {
    double accel_rms{0.0};
    double gyro_rms{0.0};
    std::size_t samples{0};
    bool static_ok{false};
    bool motion_ok{false};
};

struct StaticMotionBoundary {
    TimeNs motion_start_ns{0};
    TimeNs feed_start_ns{0};
    std::size_t static_end_window{0};
    double static_accel_rms{0.0};
    double static_gyro_rms{0.0};
};

struct ImuRowSample {
    TimeNs t_ns{0};
    Eigen::Vector3d a{Eigen::Vector3d::Zero()};
    Eigen::Vector3d w{Eigen::Vector3d::Zero()};
};

// 固定边界的 0.5 s 窗口累加器：窗口起点锚在第一条 IMU 样本，和上游 (t - t0) / step 一致。
// 录制过程中用它做「当前是否已静止/已在运动」的实时判定，避免事后重算全序列。
class WindowAccumulator {
  public:
    explicit WindowAccumulator(const ProbeConfig& config);

    void Push(TimeNs t_ns, const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro);
    // 结束时间已越过边界的窗口；未满 min_window_samples 的窗口 samples 偏小，判定视为不满足。
    std::vector<WindowStat> ClosedWindows() const;
    std::size_t closed_count() const { return closed_.size(); }
    const std::vector<WindowStat>& closed() const { return closed_; }
    TimeNs origin_ns() const { return origin_ns_; }

  private:
    struct Pending {
        std::size_t samples{0};
        Eigen::Vector3d accel_sum{Eigen::Vector3d::Zero()};
        Eigen::Vector3d gyro_sum{Eigen::Vector3d::Zero()};
        double accel_sq{0.0};
        double gyro_sq{0.0};
    };
    void CloseWindow();
    void SealPending();

    ProbeConfig config_;
    double step_ns_{0.0};
    TimeNs origin_ns_{0};
    bool started_{false};
    std::size_t current_index_{0};
    Pending pending_;
    std::vector<WindowStat> closed_;
};

// 用与录制端完全相同的语义在全序列上找最后一个静止→运动边界。
std::optional<StaticMotionBoundary> FindStaticMotionBoundary(const std::vector<ImuRowSample>& imu,
                                                             const ProbeConfig& config,
                                                             std::vector<WindowStat>* stats_out);

// 由静止段估计 IMU 噪声密度：白噪声用相邻差分（一阶差分对常值 bias 天然免疫），
// bias 随机游走用块均值的 Allan τ 分量。样本不足时 measured=false 并回退到保守默认值。
struct NoiseEstimate {
    ImuNoise noise;
    std::size_t blocks{0};
    double white_tau_ns{0.0};
    double random_walk_tau_s{0.0};
    std::string note;
};

NoiseEstimate EstimateImuNoise(const std::vector<GyroSample>& gyro,
                               const std::vector<AccelSample>& accel,
                               TimeNs begin_ns,
                               TimeNs end_ns);

// 录制引导用的实时门控状态。
struct LiveGateStatus {
    bool static_run_met{false};
    bool motion_run_met{false};
    bool too_aggressive{false};
    std::size_t static_windows{0};
    std::size_t motion_windows{0};
    double last_accel_rms{0.0};
    double last_gyro_rms{0.0};
    double accel_norm{0.0};
    std::string hint;
};

class LiveGate {
  public:
    explicit LiveGate(const ProbeConfig& config);
    void Push(TimeNs t_ns, const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro);
    // static_needed / motion_needed 为期望的连续窗口数（分别对应 --still 秒数 / 激励时长）。
    LiveGateStatus Assess(std::size_t static_needed, std::size_t motion_needed) const;
    std::size_t closed_windows() const { return accumulator_.closed_count(); }

  private:
    ProbeConfig config_;
    WindowAccumulator accumulator_;
    double last_accel_norm_{0.0};
};

} // namespace rh

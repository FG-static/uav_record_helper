#include "record_helper/motion_gate.hpp"

#include <algorithm>
#include <cmath>

namespace rh {

WindowAccumulator::WindowAccumulator(const ProbeConfig& config)
    : config_(config), step_ns_(config.window_s * 1e9) {}

void WindowAccumulator::Push(TimeNs t_ns,
                             const Eigen::Vector3d& accel,
                             const Eigen::Vector3d& gyro) {
    if (!started_) {
        // 窗口栅格锚在第一条 IMU 样本，等价于上游的 t0 = imu.front().time。
        started_ = true;
        origin_ns_ = t_ns;
    }
    const std::size_t index =
        static_cast<std::size_t>(static_cast<double>(t_ns - origin_ns_) / step_ns_);
    while (current_index_ < index) {
        CloseWindow();
    }
    pending_.samples += 1;
    pending_.accel_sum += accel;
    pending_.gyro_sum += gyro;
    pending_.accel_sq += accel.squaredNorm();
    pending_.gyro_sq += gyro.squaredNorm();
}

void WindowAccumulator::CloseWindow() {
    SealPending();
    current_index_ += 1;
    pending_ = Pending{};
}

void WindowAccumulator::SealPending() {
    WindowStat stat;
    stat.samples = pending_.samples;
    if (pending_.samples > 0) {
        const double n = static_cast<double>(pending_.samples);
        const Eigen::Vector3d accel_mean = pending_.accel_sum / n;
        const Eigen::Vector3d gyro_mean = pending_.gyro_sum / n;
        const double accel_var = pending_.accel_sq / n - accel_mean.squaredNorm();
        const double gyro_var = pending_.gyro_sq / n - gyro_mean.squaredNorm();
        // 与上游一致：RMS 是相对窗口均值的偏差，静止时 ‖a‖≈g 但偏差接近 0。
        stat.accel_rms = std::sqrt(std::max(0.0, accel_var));
        stat.gyro_rms = std::sqrt(std::max(0.0, gyro_var));
    }
    stat.static_ok = stat.samples >= config_.min_window_samples &&
                     stat.accel_rms <= config_.max_accel_rms &&
                     stat.gyro_rms <= config_.max_gyro_rms;
    stat.motion_ok = stat.accel_rms >= config_.min_motion_rms;
    closed_.push_back(stat);
}

std::vector<WindowStat> WindowAccumulator::ClosedWindows() const {
    return closed_;
}

std::optional<StaticMotionBoundary> FindStaticMotionBoundary(const std::vector<ImuRowSample>& imu,
                                                             const ProbeConfig& config,
                                                             std::vector<WindowStat>* stats_out) {
    if (imu.size() < 4) {
        return std::nullopt;
    }
    WindowAccumulator accumulator(config);
    for (const auto& sample : imu) {
        accumulator.Push(sample.t_ns, sample.a, sample.w);
    }
    std::vector<WindowStat> stats = accumulator.ClosedWindows();
    if (stats_out != nullptr) {
        *stats_out = stats;
    }
    if (stats.size() <= config.static_run) {
        return std::nullopt;
    }

    const TimeNs origin_ns = accumulator.origin_ns();
    const double step_ns = config.window_s * 1e9;
    std::optional<StaticMotionBoundary> found;
    const std::size_t lead_windows =
        static_cast<std::size_t>(config.motion_lead_s / config.window_s);
    for (std::size_t start = 0; start + config.static_run <= stats.size(); ++start) {
        bool static_run = true;
        for (std::size_t offset = 0; offset < config.static_run; ++offset) {
            if (!stats[start + offset].static_ok) {
                static_run = false;
                break;
            }
        }
        if (!static_run) {
            continue;
        }
        std::size_t static_end = start + config.static_run;
        while (static_end < stats.size() && stats[static_end].static_ok) {
            ++static_end;
        }
        const std::size_t motion_limit = std::min(stats.size(), static_end + lead_windows);
        std::size_t motion_count = 0;
        for (std::size_t index = static_end; index < motion_limit; ++index) {
            if (stats[index].motion_ok) {
                ++motion_count;
                if (motion_count >= config.motion_run) {
                    break;
                }
            } else if (stats[index].static_ok) {
                motion_count = 0;
            }
        }
        if (motion_count < config.motion_run) {
            continue;
        }
        StaticMotionBoundary boundary;
        boundary.motion_start_ns =
            origin_ns + static_cast<TimeNs>(static_cast<double>(static_end) * step_ns);
        boundary.feed_start_ns =
            boundary.motion_start_ns - static_cast<TimeNs>(config.feed_lead_s * 1e9);
        if (boundary.feed_start_ns < origin_ns) {
            boundary.feed_start_ns = origin_ns;
        }
        boundary.static_end_window = static_end;
        boundary.static_accel_rms = stats[start].accel_rms;
        boundary.static_gyro_rms = stats[start].gyro_rms;
        found = boundary;
    }
    return found;
}

namespace {

struct StreamAxis {
    std::vector<std::pair<TimeNs, double>> samples;
};

struct StreamAxes {
    StreamAxis axes[3];
    std::size_t samples{0};
};

template <typename Sample>
void CollectStream(StreamAxes* stream,
                   const std::vector<Sample>& samples,
                   TimeNs begin_ns,
                   TimeNs end_ns) {
    for (const auto& sample : samples) {
        if (sample.t_ns < begin_ns || sample.t_ns > end_ns) {
            continue;
        }
        if constexpr (std::is_same_v<Sample, GyroSample>) {
            for (int axis = 0; axis < 3; ++axis) {
                stream->axes[axis].samples.emplace_back(sample.t_ns, sample.w[axis]);
            }
        } else {
            for (int axis = 0; axis < 3; ++axis) {
                stream->axes[axis].samples.emplace_back(sample.t_ns, sample.a[axis]);
            }
        }
        stream->samples += 1;
    }
}

// 单轴的两延迟 Allan 分量：τ=dt 的一阶差分给白噪声密度，τ=block 的块均值方差给随机游走。
// Var(块均值) = M²·τ/3 + N²/(2τ)，因此先取 N 再反解 M；N²/(2τ) 大于观测方差时按 0 处理。
bool EstimateAxis(const std::vector<std::pair<TimeNs, double>>& samples,
                  double block_ns,
                  double* density,
                  double* random_walk,
                  std::size_t* blocks) {
    if (samples.size() < 20) {
        return false;
    }
    double delta_sq = 0.0;
    double dt_sum = 0.0;
    for (std::size_t index = 1; index < samples.size(); ++index) {
        const double delta = samples[index].second - samples[index - 1].second;
        delta_sq += delta * delta;
        dt_sum += static_cast<double>(samples[index].first - samples[index - 1].first);
    }
    // dt_sum 累加的是 TimeNs，必须换算成秒，否则噪声密度会被放大 1e9 倍。
    const double dt_s = dt_sum * 1e-9 / static_cast<double>(samples.size() - 1);
    const double mean_delta_sq = delta_sq / static_cast<double>(samples.size() - 1);
    if (dt_s <= 0.0 || mean_delta_sq <= 0.0) {
        return false;
    }
    const double white = std::sqrt(mean_delta_sq * dt_s);

    std::vector<double> means;
    std::size_t block = 0;
    double sum = 0.0;
    std::size_t count = 0;
    const TimeNs first_ns = samples.front().first;
    for (const auto& sample : samples) {
        const std::size_t index =
            static_cast<std::size_t>(static_cast<double>(sample.first - first_ns) / block_ns);
        if (index != block) {
            if (count > 0) {
                means.push_back(sum / static_cast<double>(count));
            }
            block = index;
            sum = 0.0;
            count = 0;
        }
        sum += sample.second;
        count += 1;
    }
    if (count > 0) {
        means.push_back(sum / static_cast<double>(count));
    }
    if (means.size() < 3) {
        return false;
    }
    double mean = 0.0;
    for (const double value : means) {
        mean += value / static_cast<double>(means.size());
    }
    double variance = 0.0;
    for (const double value : means) {
        variance += (value - mean) * (value - mean);
    }
    variance /= static_cast<double>(means.size() - 1);
    const double tau_s = block_ns * 1e-9;
    const double residual = variance - white * white / (2.0 * tau_s);
    *density = white;
    *random_walk = residual > 0.0 ? std::sqrt(3.0 * residual / tau_s) : 0.0;
    *blocks = means.size();
    return true;
}

bool EstimateStream(const StreamAxes& stream,
                    double block_ns,
                    double* density,
                    double* random_walk,
                    std::size_t* blocks) {
    if (stream.samples < 20) {
        return false;
    }
    double white_sum = 0.0;
    double walk_sum = 0.0;
    std::size_t smallest_blocks = 0;
    for (int axis = 0; axis < 3; ++axis) {
        double axis_density = 0.0;
        double axis_walk = 0.0;
        std::size_t axis_blocks = 0;
        if (!EstimateAxis(
                stream.axes[axis].samples, block_ns, &axis_density, &axis_walk, &axis_blocks)) {
            return false;
        }
        white_sum += axis_density;
        walk_sum += axis_walk;
        smallest_blocks =
            smallest_blocks == 0 ? axis_blocks : std::min(smallest_blocks, axis_blocks);
    }
    *density = white_sum / 3.0;
    *random_walk = walk_sum / 3.0;
    *blocks = smallest_blocks;
    return true;
}

} // namespace

NoiseEstimate EstimateImuNoise(const std::vector<GyroSample>& gyro,
                               const std::vector<AccelSample>& accel,
                               TimeNs begin_ns,
                               TimeNs end_ns) {
    NoiseEstimate estimate;
    // 回退值只保证协方差正定且量级合理，不代表器件真实噪声；真实值来自静止段实测。
    estimate.noise.sigma_g = 4.0e-4;
    estimate.noise.sigma_a = 1.0e-3;
    estimate.noise.sigma_bg = 4.0e-6;
    estimate.noise.sigma_ba = 5.0e-5;
    estimate.noise.g = 9.81;
    estimate.note = "fallback=conservative_default";

    const double span_s = static_cast<double>(end_ns - begin_ns) * 1e-9;
    if (span_s <= 1.0) {
        return estimate;
    }
    // 块长取静止段可用的较大 τ（≤2 s），对应 Allan 方差随机游走斜率刚开始抬升的位置。
    const double block_ns = (span_s >= 5.0 ? 2.0 : 1.0) * 1.0e9;

    StreamAxes gyro_stream;
    StreamAxes accel_stream;
    CollectStream(&gyro_stream, gyro, begin_ns, end_ns);
    CollectStream(&accel_stream, accel, begin_ns, end_ns);

    double sigma_g = 0.0;
    double sigma_bg = 0.0;
    double sigma_a = 0.0;
    double sigma_ba = 0.0;
    std::size_t gyro_blocks = 0;
    std::size_t accel_blocks = 0;
    const bool gyro_ok = EstimateStream(gyro_stream, block_ns, &sigma_g, &sigma_bg, &gyro_blocks);
    const bool accel_ok =
        EstimateStream(accel_stream, block_ns, &sigma_a, &sigma_ba, &accel_blocks);
    if (gyro_ok) {
        estimate.noise.sigma_g = sigma_g;
        estimate.noise.sigma_bg = sigma_bg;
    }
    if (accel_ok) {
        estimate.noise.sigma_a = sigma_a;
        estimate.noise.sigma_ba = sigma_ba;
    }
    estimate.blocks = std::min(gyro_ok ? gyro_blocks : 0, accel_ok ? accel_blocks : 0);
    estimate.random_walk_tau_s = block_ns * 1e-9;
    if (gyro_ok && accel_ok) {
        estimate.noise.measured = true;
        estimate.note = "measured=allan_two_lag";
    } else {
        estimate.note = "fallback=insufficient_static_samples";
    }
    return estimate;
}

LiveGate::LiveGate(const ProbeConfig& config) : config_(config), accumulator_(config) {}

void LiveGate::Push(TimeNs t_ns, const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro) {
    accumulator_.Push(t_ns, accel, gyro);
    last_accel_norm_ = accel.norm();
}

LiveGateStatus LiveGate::Assess(std::size_t static_needed, std::size_t motion_needed) const {
    LiveGateStatus status;
    const std::vector<WindowStat>& windows = accumulator_.closed();
    if (!windows.empty()) {
        status.last_accel_rms = windows.back().accel_rms;
        status.last_gyro_rms = windows.back().gyro_rms;
    }
    // 尾部连续静止/运动窗数，直接对应门控要等的「这一段是否已经够长」。
    std::size_t tail_static = 0;
    for (std::size_t index = windows.size(); index-- > 0;) {
        if (!windows[index].static_ok) {
            break;
        }
        ++tail_static;
    }
    std::size_t tail_motion = 0;
    for (std::size_t index = windows.size(); index-- > 0;) {
        if (!windows[index].motion_ok) {
            break;
        }
        ++tail_motion;
    }
    status.static_windows = tail_static;
    status.motion_windows = tail_motion;
    status.accel_norm = last_accel_norm_;
    status.static_run_met = tail_static >= static_needed;
    status.motion_run_met = tail_motion >= motion_needed;
    status.too_aggressive = status.last_accel_rms > 2.5 || status.last_gyro_rms > 1.5;
    if (status.too_aggressive) {
        status.hint = "运动过猛：请放慢平移与转动，避免丢点与运动模糊";
    } else if (!status.static_run_met) {
        status.hint = "把设备平放不动，让加速度计和陀螺的窗口抖动降到阈值以下";
    } else if (!status.motion_run_met) {
        status.hint = "缓慢平移 30~50 cm 并小幅偏航/俯仰，持续 3 s 以上";
    } else {
        status.hint = "激励充分，可以开始正常录制";
    }
    return status;
}

} // namespace rh

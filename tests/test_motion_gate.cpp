#include "check.hpp"
#include "record_helper/motion_gate.hpp"
#include "synthetic.hpp"

#include <cmath>
#include <vector>

using rh::AccelSample;
using rh::GyroSample;
using rh::ImuRowSample;
using rh::ProbeConfig;
using rh::TimeNs;

namespace {

std::vector<ImuRowSample> Rows(const std::vector<GyroSample>& gyro,
                               const std::vector<AccelSample>& accel) {
    std::vector<ImuRowSample> rows;
    for (const auto& sample : gyro) {
        ImuRowSample row;
        row.t_ns = sample.t_ns;
        row.w = sample.w;
        // 最近邻取加表即可，这里的用例只关心窗口 RMS 的形状。
        const AccelSample* best = &accel.front();
        for (const auto& candidate : accel) {
            if (std::abs(candidate.t_ns - sample.t_ns) < std::abs(best->t_ns - sample.t_ns)) {
                best = &candidate;
            }
        }
        row.a = best->a;
        rows.push_back(row);
    }
    return rows;
}

// 每个窗口放 20 个样本，偏差在三轴上对称分布，使窗口 RMS 精确等于给定值
// （上游的 accel_rms 是三轴偏差平方和开根号）。末尾多喂一个样本把最后一个窗口封口。
std::vector<ImuRowSample> MakePattern(const std::vector<double>& accel_rms,
                                      const std::vector<double>& gyro_rms,
                                      double window_s) {
    std::vector<ImuRowSample> rows;
    const TimeNs origin = 5'000'000'000;
    const int steps = 20;
    const int total_steps = static_cast<int>(accel_rms.size()) * steps + 1;
    for (int flat = 0; flat < total_steps; ++flat) {
        const double bucket = static_cast<double>(flat) / steps;
        const std::size_t index = std::min(static_cast<std::size_t>(bucket), accel_rms.size() - 1);
        ImuRowSample row;
        // 时间按未截断的桶号推进：多出的最后一个样本负责把最后一个窗口封口。
        row.t_ns = origin + static_cast<TimeNs>(bucket * window_s * 1e9);
        const double sign = flat % 2 == 0 ? 1.0 : -1.0;
        const double accel_axis = accel_rms[index] / std::sqrt(3.0);
        const double gyro_axis = gyro_rms[index] / std::sqrt(3.0);
        row.a = Eigen::Vector3d(sign * accel_axis, sign * accel_axis, 9.81 + sign * accel_axis);
        row.w = Eigen::Vector3d(sign * gyro_axis, -sign * gyro_axis, sign * gyro_axis);
        rows.push_back(row);
    }
    return rows;
}

} // namespace

RH_TEST(motion_gate_windows_match_offline_semantics) {
    const auto capture = rh::testing::MakeSynthetic(4.0, 3.0, 2.0, 1.0);
    const ProbeConfig config;
    rh::WindowAccumulator accumulator(config);
    for (const auto& sample : capture.accel) {
        const GyroSample* best = &capture.gyro.front();
        for (const auto& candidate : capture.gyro) {
            if (std::abs(candidate.t_ns - sample.t_ns) < std::abs(best->t_ns - sample.t_ns)) {
                best = &candidate;
            }
        }
        accumulator.Push(sample.t_ns, sample.a, best->w);
    }
    const auto windows = accumulator.ClosedWindows();
    CHECK(windows.size() >= 9); // 10 s / 0.5 s
    for (const auto& window : windows) {
        CHECK(window.samples >= config.min_window_samples);
        CHECK(std::isfinite(window.accel_rms));
    }
    // 前 8 个窗属于静止段：偏差 RMS 必须远低于阈值；运动段必须远高于。
    CHECK(windows[0].static_ok);
    CHECK(windows[6].static_ok);
    CHECK(windows[9].motion_ok);
    CHECK(windows.back().accel_rms > config.max_accel_rms);
}

RH_TEST(motion_gate_finds_last_static_motion_boundary) {
    // 静止 → 运动 → 又静止 → 又运动：上游取「最后一个」边界，我们必须在同一位置。
    // 布局：静止 0-3，运动 4-9，静止 10-13，运动 14-19。两段都满足「≥3 静止 + ≥5 运动」，
    // 上游循环最后一次赋值胜出，所以边界必须落在后一段。
    std::vector<double> accel;
    for (int index = 0; index < 20; ++index) {
        const bool quiet = index < 4 || (index >= 10 && index < 14);
        accel.push_back(quiet ? 0.02 : 0.60);
    }
    const std::vector<double> gyro(accel.size(), 0.01);
    const auto rows = MakePattern(accel, gyro, 0.5);
    const ProbeConfig config;
    std::vector<rh::WindowStat> stats;
    const auto boundary = rh::FindStaticMotionBoundary(rows, config, &stats);
    CHECK(boundary.has_value());
    CHECK(stats.size() == accel.size());
    CHECK(boundary->static_end_window == 14);
    const TimeNs origin = rows.front().t_ns;
    CHECK(boundary->motion_start_ns == origin + static_cast<TimeNs>(14 * 0.5 * 1e9));
    CHECK(boundary->feed_start_ns ==
          boundary->motion_start_ns - static_cast<TimeNs>(config.feed_lead_s * 1e9));
    CHECK(boundary->feed_start_ns > origin);
}

RH_TEST(motion_gate_requires_genuine_stillness) {
    const std::vector<double> accel(16, 0.30); // 一直在动，从未静止
    const std::vector<double> gyro(16, 0.01);
    const auto rows = MakePattern(accel, gyro, 0.5);
    const ProbeConfig config;
    CHECK(!rh::FindStaticMotionBoundary(rows, config, nullptr).has_value());

    const std::vector<double> quiet(16, 0.02); // 一直静止，没有激励
    const auto quiet_rows = MakePattern(quiet, gyro, 0.5);
    CHECK(!rh::FindStaticMotionBoundary(quiet_rows, config, nullptr).has_value());

    // 静止够长但运动窗不足 5 个：仍然不构成边界；补到 5 个就成立。
    std::vector<double> short_motion = {0.02, 0.02, 0.02, 0.6, 0.6, 0.6, 0.6};
    const auto short_rows = MakePattern(short_motion, gyro, 0.5);
    CHECK(!rh::FindStaticMotionBoundary(short_rows, config, nullptr).has_value());
    short_motion.push_back(0.6);
    CHECK(rh::FindStaticMotionBoundary(MakePattern(short_motion, gyro, 0.5), config, nullptr)
              .has_value());
}

RH_TEST(motion_gate_synthetic_capture_is_initializable) {
    const auto capture = rh::testing::MakeSynthetic(4.0, 3.0, 4.0, 1.0);
    const auto rows = Rows(capture.gyro, capture.accel);
    const ProbeConfig config;
    const auto boundary = rh::FindStaticMotionBoundary(rows, config, nullptr);
    CHECK(boundary.has_value());
    if (boundary.has_value()) {
        // 静止段 4 s ⇒ motion_start 约在 4 s 处，feed_start 比它早 0.75 s。
        const double motion_seconds = (boundary->motion_start_ns - rows.front().t_ns) * 1e-9;
        CHECK(motion_seconds > 3.4 && motion_seconds < 5.0);
    }
}

RH_TEST(motion_gate_noise_estimate_recovers_white_density) {
    // 白噪声：密度可由相邻差分精确反推，这里验证估计器与解析值一致。
    const double dt_s = 0.0025;
    const double amplitude = 2.0e-3; // Lcg::Uniform 是 ±amplitude 的均匀分布
    const double sigma = amplitude / std::sqrt(3.0);
    const double expected_density = sigma * std::sqrt(2.0 * dt_s);
    std::vector<GyroSample> gyro;
    std::vector<AccelSample> accel;
    rh::testing::Lcg lcg(0xC0FFEEULL);
    for (std::size_t index = 0; index < 400 * 12; ++index) {
        const TimeNs stamp = static_cast<TimeNs>(index * 2'500'000LL);
        GyroSample gyro_sample;
        gyro_sample.t_ns = stamp;
        gyro_sample.w =
            Eigen::Vector3d(lcg.Uniform(amplitude), lcg.Uniform(amplitude), lcg.Uniform(amplitude));
        gyro.push_back(gyro_sample);
        AccelSample accel_sample;
        accel_sample.t_ns = stamp;
        accel_sample.a =
            Eigen::Vector3d(lcg.Uniform(4.0e-3), lcg.Uniform(4.0e-3), 9.81 + lcg.Uniform(4.0e-3));
        accel.push_back(accel_sample);
    }
    const auto estimate = rh::EstimateImuNoise(gyro, accel, 0, gyro.back().t_ns);
    CHECK(estimate.noise.measured);
    CHECK(estimate.blocks >= 3);
    CHECK(estimate.noise.sigma_g > expected_density * 0.85);
    CHECK(estimate.noise.sigma_g < expected_density * 1.15);
    const double expected_accel = (4.0e-3 / std::sqrt(3.0)) * std::sqrt(2.0 * 0.0025);
    CHECK(estimate.noise.sigma_a > expected_accel * 0.85);
    CHECK(estimate.noise.sigma_a < expected_accel * 1.15);
    CHECK(estimate.noise.sigma_bg >= 0.0); // 纯白噪声下随机游走被钳到 0，不伪造负方差

    const auto short_window =
        rh::EstimateImuNoise(gyro, accel, 0, static_cast<TimeNs>(400'000'000));
    CHECK(!short_window.noise.measured);
    CHECK(short_window.noise.sigma_g > 0.0); // 回退值仍然可用，上游要求 > 0
}

RH_TEST(motion_gate_noise_estimate_sees_bias_random_walk) {
    // 在白噪声上叠加真正的随机游走 bias，估计器必须把非零的 random_walk 分辨出来。
    std::vector<GyroSample> gyro;
    std::vector<AccelSample> accel;
    rh::testing::Lcg lcg(0xBADC0DEULL);
    Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
    const double dt_s = 0.0025;
    const double walk_g = 3.0e-4; // rad/s per sqrt(s)
    const double walk_a = 2.0e-3; // m/s² per sqrt(s)
    for (std::size_t index = 0; index < 400 * 20; ++index) {
        const TimeNs stamp = static_cast<TimeNs>(index * 2'500'000LL);
        gyro_bias += Eigen::Vector3d(lcg.Uniform(1.0), lcg.Uniform(1.0), lcg.Uniform(1.0)) *
                     (walk_g * std::sqrt(dt_s));
        accel_bias += Eigen::Vector3d(lcg.Uniform(1.0), lcg.Uniform(1.0), lcg.Uniform(1.0)) *
                      (walk_a * std::sqrt(dt_s));
        GyroSample gyro_sample;
        gyro_sample.t_ns = stamp;
        gyro_sample.w =
            gyro_bias +
            Eigen::Vector3d(lcg.Uniform(2.0e-3), lcg.Uniform(2.0e-3), lcg.Uniform(2.0e-3));
        gyro.push_back(gyro_sample);
        AccelSample accel_sample;
        accel_sample.t_ns = stamp;
        accel_sample.a = Eigen::Vector3d(accel_bias.x(), accel_bias.y(), 9.81 + accel_bias.z());
        accel.push_back(accel_sample);
    }
    const auto estimate = rh::EstimateImuNoise(gyro, accel, 0, gyro.back().t_ns);
    CHECK(estimate.noise.measured);
    CHECK(estimate.noise.sigma_bg > 0.0);
    CHECK(estimate.noise.sigma_ba > 0.0);
    // 块均值方差主要由游走主导：量级应当接近注入值（保守允许 3 倍误差带）。
    CHECK(estimate.noise.sigma_bg > walk_g * 0.3);
    CHECK(estimate.noise.sigma_bg < walk_g * 3.0);
    CHECK(estimate.noise.sigma_ba > walk_a * 0.3);
    CHECK(estimate.noise.sigma_ba < walk_a * 3.0);
}

RH_TEST(motion_gate_live_gate_tracks_phases) {
    const auto capture = rh::testing::MakeSynthetic(4.0, 3.0, 2.0, 1.0);
    const ProbeConfig config;
    rh::LiveGate gate(config);
    std::size_t static_met_at = 0;
    std::size_t motion_met_at = 0;
    bool too_aggressive_seen = false;
    std::size_t index = 0;
    for (const auto& sample : capture.accel) {
        const GyroSample* best = &capture.gyro.front();
        for (const auto& candidate : capture.gyro) {
            if (std::abs(candidate.t_ns - sample.t_ns) < std::abs(best->t_ns - sample.t_ns)) {
                best = &candidate;
            }
        }
        gate.Push(sample.t_ns, sample.a, best->w);
        const auto status = gate.Assess(8, 6);
        too_aggressive_seen = too_aggressive_seen || status.too_aggressive;
        ++index;
        if (static_met_at == 0 && status.static_run_met) {
            static_met_at = index;
        }
        if (motion_met_at == 0 && static_met_at != 0 && status.motion_run_met) {
            motion_met_at = index;
        }
    }
    // 250 Hz 加表：静止 4 s ≈ 1000 个样本后满 8 个连续静止窗，再 3 s 后满 6 个运动窗。
    CHECK(static_met_at >= 950 && static_met_at <= 1100);
    CHECK(motion_met_at > static_met_at + 700);
    CHECK(motion_met_at <= static_met_at + 950);
    CHECK(!too_aggressive_seen); // 合成激励是温和的，不该被判为过猛
    const auto final_status = gate.Assess(8, 6);
    CHECK(final_status.accel_norm > 0.0);
    CHECK(!final_status.hint.empty());
}

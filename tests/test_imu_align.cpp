#include "check.hpp"
#include "record_helper/imu_align.hpp"
#include "synthetic.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

using rh::AccelSample;
using rh::AlignResult;
using rh::GyroSample;
using rh::ImuGridMode;
using rh::ImuSegment;
using rh::TimeNs;

namespace {

constexpr TimeNs kGyroDt = 2'500'000;  // 400 Hz
constexpr TimeNs kAccelDt = 4'000'000; // 250 Hz
constexpr TimeNs kMaxGap = 10'000'000;

std::vector<GyroSample> MakeGyro(std::size_t count, TimeNs origin = 1'000'000'000) {
    std::vector<GyroSample> samples;
    for (std::size_t index = 0; index < count; ++index) {
        GyroSample sample;
        sample.t_ns = origin + static_cast<TimeNs>(index) * kGyroDt;
        sample.w = Eigen::Vector3d(index * 0.001, -index * 0.002, index * 0.003);
        samples.push_back(sample);
    }
    return samples;
}

std::vector<AccelSample> MakeAccel(std::size_t count, TimeNs origin = 1'000'000'000) {
    std::vector<AccelSample> samples;
    for (std::size_t index = 0; index < count; ++index) {
        AccelSample sample;
        sample.t_ns = origin + static_cast<TimeNs>(index) * kAccelDt;
        sample.a = Eigen::Vector3d(0.0, 0.0, 9.81 + index * 0.01);
        samples.push_back(sample);
    }
    return samples;
}

} // namespace

RH_TEST(imu_align_uses_gyro_grid_and_interpolates_accel) {
    const auto gyro = MakeGyro(400);
    const auto accel = MakeAccel(250);
    AlignResult result;
    std::string error;
    CHECK(rh::AlignImuStreams(gyro, accel, kMaxGap, ImuGridMode::kGyroTimestamps, &result, &error));
    // 陀螺覆盖到 1.9975 s、加表只到 1.996 s：末端未覆盖的陀螺样本必须整行丢弃。
    CHECK(result.rows.size() == 399);
    CHECK(result.stats.gyro_raw == 400);
    CHECK(result.stats.accel_raw == 250);
    CHECK_NEAR(result.stats.grid_dt_ms, 2.5, 1e-9);
    CHECK(result.stats.max_gap_ms <= 2.6);
    // 加表是内插出来的：相邻两行几乎不可能出现完全相同的三元组（room_02 的 0.38 就是零阶保持）。
    CHECK(result.stats.accel_hold_ratio < 0.01);
    for (std::size_t index = 1; index < result.rows.size(); ++index) {
        CHECK(result.rows[index].t_ns > result.rows[index - 1].t_ns);
        CHECK(result.rows[index].t_ns == gyro[index].t_ns);
    }
    // 内插值的正确性：t=1.010 s 落在 accel 1.008(9.83) 与 1.012(9.84) 的正中。
    CHECK_NEAR(result.rows[4].a.z(), 9.835, 1e-9);
    // t=1.0125 s 落在 1.012(9.84) 与 1.016(9.85) 之间，α = 0.125。
    CHECK_NEAR(result.rows[5].a.z(), 9.84125, 1e-9);
    CHECK(result.segments.size() == 1);
}

RH_TEST(imu_align_drops_uncovered_and_splits_segments) {
    auto gyro = MakeGyro(400);
    auto accel = MakeAccel(100); // 只覆盖前 400 ms
    AlignResult result;
    std::string error;
    CHECK(rh::AlignImuStreams(gyro, accel, kMaxGap, ImuGridMode::kGyroTimestamps, &result, &error));
    // 加表末端之后的一律不输出，也绝不用零阶保持补到最后。
    CHECK(result.rows.size() <= 160);
    CHECK(result.dropped_grid_uncovered > 200);
    CHECK(result.rows.back().t_ns <= accel.back().t_ns);

    gyro = MakeGyro(400);
    accel = MakeAccel(400);
    // 中段挖掉 30 个陀螺样本（75 ms > 10 ms 上限）：必须切成两段，而不是造出假样本。
    gyro.erase(gyro.begin() + 100, gyro.begin() + 130);
    CHECK(rh::AlignImuStreams(gyro, accel, kMaxGap, ImuGridMode::kGyroTimestamps, &result, &error));
    CHECK(result.segments.size() == 2);
    CHECK(result.dropped_gap_too_large == 1);
    for (const auto& segment : result.segments) {
        CHECK(segment.row_end > segment.row_begin);
        for (std::size_t index = segment.row_begin + 1; index < segment.row_end; ++index) {
            CHECK(result.rows[index].t_ns - result.rows[index - 1].t_ns <= kMaxGap);
        }
    }
    CHECK(result.segments[0].row_end - result.segments[0].row_begin == 100);
}

RH_TEST(imu_align_normalizes_out_of_order_and_duplicates) {
    auto gyro = MakeGyro(50);
    auto accel = MakeAccel(80);
    std::swap(gyro[10], gyro[11]);
    gyro[20] = gyro[19]; // 重复时间戳
    auto broken = gyro[30];
    broken.w = Eigen::Vector3d(NAN, 0.0, 0.0);
    gyro.push_back(broken);
    AlignResult result;
    std::string error;
    CHECK(rh::AlignImuStreams(gyro, accel, kMaxGap, ImuGridMode::kGyroTimestamps, &result, &error));
    for (std::size_t index = 1; index < result.rows.size(); ++index) {
        CHECK(result.rows[index].t_ns > result.rows[index - 1].t_ns);
        CHECK(std::isfinite(result.rows[index].w.z()));
    }
    CHECK(result.stats.dropped_non_increasing >= 1);
}

RH_TEST(imu_align_rejects_empty_and_bad_config) {
    AlignResult result;
    std::string error;
    CHECK(!rh::AlignImuStreams({}, {}, kMaxGap, ImuGridMode::kGyroTimestamps, &result, &error));
    CHECK(!error.empty());
    error.clear();
    CHECK(!rh::AlignImuStreams(
        MakeGyro(10), MakeAccel(1), kMaxGap, ImuGridMode::kGyroTimestamps, &result, &error));
    CHECK(!rh::AlignImuStreams(
        MakeGyro(10), MakeAccel(10), 0, ImuGridMode::kGyroTimestamps, &result, &error));
    // 抽稀栅格反过来要求陀螺至少 2 条：加表当栅格时陀螺是被重采样那条。
    CHECK(!rh::AlignImuStreams(
        MakeGyro(1), MakeAccel(10), kMaxGap, ImuGridMode::kAccelTimestamps, &result, &error));
    CHECK(!error.empty());
}

RH_TEST(imu_align_synthetic_capture_is_contiguous) {
    const auto capture = rh::testing::MakeSynthetic(4.0, 3.0, 2.0, 1.0);
    AlignResult result;
    std::string error;
    CHECK(rh::AlignImuStreams(capture.gyro,
                              capture.accel,
                              kMaxGap,
                              ImuGridMode::kGyroTimestamps,
                              &result,
                              &error));
    CHECK(result.segments.size() == 1); // 合成数据本身连续，裁窗不该切出多段
    CHECK(result.stats.accel_hold_ratio < 0.01);
    CHECK(result.stats.max_gap_ms <= 2.6);
}

RH_TEST(imu_align_accel_grid_decimates_gyro_without_synthesis) {
    const auto gyro = MakeGyro(400);
    const auto accel = MakeAccel(250);
    AlignResult result;
    std::string error;
    CHECK(rh::AlignImuStreams(
        gyro, accel, kMaxGap, ImuGridMode::kAccelTimestamps, &result, &error));
    CHECK(result.stats.gyro_raw == 400);
    CHECK(result.stats.accel_raw == 250);
    // 栅格变成加表的 4 ms；加表末端 1.996 s 仍在陀螺 [1.000, 1.9975] 覆盖内，所以 250 行全留。
    CHECK_NEAR(result.stats.grid_dt_ms, 4.0, 1e-9);
    CHECK(result.rows.size() == 250);

    for (std::size_t index = 0; index < result.rows.size(); ++index) {
        const auto& row = result.rows[index];
        // 加表列必须逐字等于某个加表实测样本：这一列一个数都没造出来。
        const auto measured_accel = std::find_if(
            accel.begin(), accel.end(), [&](const AccelSample& s) { return s.t_ns == row.t_ns; });
        CHECK(measured_accel != accel.end());
        if (measured_accel != accel.end()) {
            CHECK(measured_accel->a == row.a);
        }
        // 陀螺列同样必须是实测样本之一，且就是时间最近的那一条。
        const auto measured_gyro = std::find_if(
            gyro.begin(), gyro.end(), [&](const GyroSample& s) { return s.w == row.w; });
        CHECK(measured_gyro != gyro.end());
        if (measured_gyro != gyro.end()) {
            CHECK(std::abs(measured_gyro->t_ns - row.t_ns) <= kGyroDt / 2);
            CHECK(measured_gyro ==
                  std::min_element(gyro.begin(),
                                   gyro.end(),
                                   [&](const GyroSample& left, const GyroSample& right) {
                                       return std::abs(left.t_ns - row.t_ns) <
                                              std::abs(right.t_ns - row.t_ns);
                                   }));
        }
        if (index > 0) {
            CHECK(row.t_ns > result.rows[index - 1].t_ns);
        }
    }
    // 4 ms 栅格配 2.5 ms 陀螺：每行的 w 最多比栅格时刻偏 1 ms，且绝不重复用同一条陀螺。
    CHECK_NEAR(result.stats.max_source_skew_ms, 1.0, 1e-9);
    CHECK(result.stats.accel_hold_ratio < 1e-12);
    CHECK(result.stats.gyro_hold_ratio < 1e-12);
}

RH_TEST(imu_align_accel_grid_drops_rows_outside_gyro_coverage) {
    const auto gyro = MakeGyro(160); // 只覆盖 1.000 ~ 1.3975 s
    const auto accel = MakeAccel(250);
    AlignResult result;
    std::string error;
    CHECK(rh::AlignImuStreams(
        gyro, accel, kMaxGap, ImuGridMode::kAccelTimestamps, &result, &error));
    // 加表时刻超出陀螺 [首, 尾] 的一律整行丢弃，绝不拿最后的陀螺值往外保持。
    CHECK(result.rows.size() == 100);
    CHECK(result.dropped_grid_uncovered == 150);
    CHECK(result.rows.back().t_ns <= gyro.back().t_ns);
}

RH_TEST(imu_align_accel_grid_splits_on_gyro_dropout) {
    auto gyro = MakeGyro(400);
    const auto accel = MakeAccel(250);
    // 陀螺中段断 20 条（50 ms）：落在断口中间、最近样本也差 10 ms 以上的加表时刻必须断开成两段。
    gyro.erase(gyro.begin() + 100, gyro.begin() + 120);
    AlignResult result;
    std::string error;
    CHECK(rh::AlignImuStreams(
        gyro, accel, kMaxGap, ImuGridMode::kAccelTimestamps, &result, &error));
    CHECK(result.segments.size() == 2);
    CHECK(result.dropped_grid_uncovered >= 1);
    // 栅格自身（加表）没有缺口，所以断开只能归因于陀螺配对不上，而不是造出了桥接样本。
    CHECK(result.dropped_gap_too_large == 0);
    for (std::size_t segment = 0; segment < result.segments.size(); ++segment) {
        const ImuSegment& range = result.segments[segment];
        for (std::size_t index = range.row_begin + 1; index < range.row_end; ++index) {
            CHECK(result.rows[index].t_ns - result.rows[index - 1].t_ns <= kAccelDt);
            CHECK(std::isfinite(result.rows[index].w.z()));
        }
    }
}

RH_TEST(imu_align_accel_grid_synthetic_capture_has_no_repeats) {
    const auto capture = rh::testing::MakeSynthetic(4.0, 3.0, 2.0, 1.0);
    AlignResult interpolated;
    AlignResult decimated;
    std::string error;
    CHECK(rh::AlignImuStreams(capture.gyro,
                              capture.accel,
                              kMaxGap,
                              ImuGridMode::kGyroTimestamps,
                              &interpolated,
                              &error));
    CHECK(rh::AlignImuStreams(capture.gyro,
                              capture.accel,
                              kMaxGap,
                              ImuGridMode::kAccelTimestamps,
                              &decimated,
                              &error));
    CHECK(decimated.rows.size() < interpolated.rows.size());
    CHECK(decimated.segments.size() == 1);
    CHECK(decimated.stats.accel_hold_ratio < 0.01);
    CHECK(decimated.stats.gyro_hold_ratio < 0.01);
    CHECK(decimated.stats.max_source_skew_ms <= 1.25);
    CHECK(interpolated.stats.max_source_skew_ms == 0.0); // 内插模式不引入取数偏移
}

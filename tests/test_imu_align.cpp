#include "check.hpp"
#include "record_helper/imu_align.hpp"
#include "synthetic.hpp"

#include <cmath>
#include <vector>

using rh::AccelSample;
using rh::AlignResult;
using rh::GyroSample;
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
    CHECK(rh::AlignImuStreams(gyro, accel, kMaxGap, &result, &error));
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
    CHECK(rh::AlignImuStreams(gyro, accel, kMaxGap, &result, &error));
    // 加表末端之后的一律不输出，也绝不用零阶保持补到最后。
    CHECK(result.rows.size() <= 160);
    CHECK(result.dropped_accel_uncovered > 200);
    CHECK(result.rows.back().t_ns <= accel.back().t_ns);

    gyro = MakeGyro(400);
    accel = MakeAccel(400);
    // 中段挖掉 30 个陀螺样本（75 ms > 10 ms 上限）：必须切成两段，而不是造出假样本。
    gyro.erase(gyro.begin() + 100, gyro.begin() + 130);
    CHECK(rh::AlignImuStreams(gyro, accel, kMaxGap, &result, &error));
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
    CHECK(rh::AlignImuStreams(gyro, accel, kMaxGap, &result, &error));
    for (std::size_t index = 1; index < result.rows.size(); ++index) {
        CHECK(result.rows[index].t_ns > result.rows[index - 1].t_ns);
        CHECK(std::isfinite(result.rows[index].w.z()));
    }
    CHECK(result.stats.dropped_non_increasing >= 1);
}

RH_TEST(imu_align_rejects_empty_and_bad_config) {
    AlignResult result;
    std::string error;
    CHECK(!rh::AlignImuStreams({}, {}, kMaxGap, &result, &error));
    CHECK(!error.empty());
    error.clear();
    CHECK(!rh::AlignImuStreams(MakeGyro(10), MakeAccel(1), kMaxGap, &result, &error));
    CHECK(!rh::AlignImuStreams(MakeGyro(10), MakeAccel(10), 0, &result, &error));
}

RH_TEST(imu_align_synthetic_capture_is_contiguous) {
    const auto capture = rh::testing::MakeSynthetic(4.0, 3.0, 2.0, 1.0);
    AlignResult result;
    std::string error;
    CHECK(rh::AlignImuStreams(capture.gyro, capture.accel, kMaxGap, &result, &error));
    CHECK(result.segments.size() == 1); // 合成数据本身连续，裁窗不该切出多段
    CHECK(result.stats.accel_hold_ratio < 0.01);
    CHECK(result.stats.max_gap_ms <= 2.6);
}

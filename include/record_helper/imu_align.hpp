#pragma once

#include <string>
#include <vector>

#include "record_helper/types.hpp"

namespace rh {

// 一段连续可用的 IMU 行（区间为 rows 的下标，左闭右开）。断开原因只有三种：
// 时间戳非严格递增、相邻间隔超过上限、加表数据未覆盖该时刻。
struct ImuSegment {
    std::size_t row_begin{0};
    std::size_t row_end{0};
};

struct AlignResult {
    std::vector<ImuRow> rows;
    std::vector<ImuSegment> segments;
    ImuAlignStats stats;
    // 因加表未覆盖或间隔超限而丢弃的陀螺样本数（这些时刻整行不输出）。
    std::size_t dropped_accel_uncovered{0};
    std::size_t dropped_gap_too_large{0};
};

// 以陀螺原始时间戳为唯一栅格，把加表线性内插到同一时刻。刻意不做零阶保持：
// room_02 的 data.csv 有 38% 的行在重复上一个加表三元组，那就是合并策略造成的假数据。
// 输出保证：时间戳严格递增、间隔 ≤ max_imu_gap_ns、gyro/accel 全部有限。
bool AlignImuStreams(const std::vector<GyroSample>& gyro,
                     const std::vector<AccelSample>& accel,
                     TimeNs max_imu_gap_ns,
                     AlignResult* result,
                     std::string* error);

} // namespace rh

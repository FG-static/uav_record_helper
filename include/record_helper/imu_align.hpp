#pragma once

#include <string>
#include <vector>

#include "record_helper/types.hpp"

namespace rh {

// 一段连续可用的 IMU 行（区间为 rows 的下标，左闭右开）。断开原因只有三种：
// 时间戳非严格递增、相邻间隔超过上限、另一条流的数据未覆盖该时刻。
struct ImuSegment {
    std::size_t row_begin{0};
    std::size_t row_end{0};
};

// imu0/data.csv 的一行只有一个时刻，而加表与陀螺各有各的节拍，必须挑一条流的时间戳当栅格。
// 两种模式的共同点：绝不零阶保持，栅格覆盖不到的时刻整行丢弃而不是补出来。
enum class ImuGridMode {
    // 陀螺原始时间戳当栅格（默认）。加表线性内插到陀螺时刻，陀螺保持全速率。
    kGyroTimestamps,
    // 加表原始时间戳当栅格。陀螺只从实测样本里取时间最近的那一条（抽稀），
    // 两条流的数值都是设备真测过的；代价是丢掉高出来的那部分陀螺速率。
    kAccelTimestamps,
};

struct AlignResult {
    std::vector<ImuRow> rows;
    std::vector<ImuSegment> segments;
    ImuAlignStats stats;
    // 栅格时刻整行丢弃的数量：另一条流没覆盖到、取不到有效样本、或数值非有限。
    std::size_t dropped_grid_uncovered{0};
    // 栅格自身出现超过 max_imu_gap_ns 的缺口而被断开的次数。
    std::size_t dropped_gap_too_large{0};
};

// 把两条流统一到同一栅格后写出 rows。
// kGyroTimestamps：栅格 = 陀螺时刻，加表线性内插（room_02 那 38% 的重复行就是没走这条路的后果）。
// kAccelTimestamps：栅格 = 加表时刻，陀螺取最近实测样本；最近样本与栅格时刻的偏移超过
// max_imu_gap_ns 时整行丢弃，实测最大偏移写进 stats.max_source_skew_ms。
// 输出保证：时间戳严格递增、间隔 ≤ max_imu_gap_ns、gyro/accel 全部有限。
bool AlignImuStreams(const std::vector<GyroSample>& gyro,
                     const std::vector<AccelSample>& accel,
                     TimeNs max_imu_gap_ns,
                     ImuGridMode mode,
                     AlignResult* result,
                     std::string* error);

// 稳定字符串，用于命令行取值与 record_summary.yaml 留档："gyro" / "accel"。
const char* ImuGridModeName(ImuGridMode mode);

// 解析命令行取值；无法识别时返回 false 且不写 mode。
bool ParseImuGridMode(const std::string& text, ImuGridMode* mode);

} // namespace rh

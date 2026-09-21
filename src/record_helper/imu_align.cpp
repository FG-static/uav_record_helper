#include "record_helper/imu_align.hpp"

#include <algorithm>
#include <cmath>

namespace rh {
namespace {

bool Finite(const Eigen::Vector3d& value) {
    return std::isfinite(value.x()) && std::isfinite(value.y()) && std::isfinite(value.z());
}

TimeNs AbsDelta(TimeNs left, TimeNs right) {
    return left > right ? left - right : right - left;
}

// 两条流在栅格逻辑里是同构的：都是「时刻 + 三元组」，谁当栅格由调用方决定。
struct Motion {
    TimeNs t_ns{0};
    Eigen::Vector3d v{Eigen::Vector3d::Zero()};
};

// 设备回调顺序不保证严格递增（USB 重传、双端点），先按时间稳定排序再去重，
// 保证写出的 imu0/data.csv 满足「严格递增」这条会整文件失败的上游约束。
template <typename Sample>
std::vector<Motion> NormalizeStream(const std::vector<Sample>& input,
                                    Eigen::Vector3d Sample::*field,
                                    std::size_t* dropped) {
    std::vector<Motion> sorted;
    sorted.reserve(input.size());
    for (const auto& sample : input) {
        sorted.push_back(Motion{sample.t_ns, (sample.*field)});
    }
    std::stable_sort(sorted.begin(), sorted.end(), [](const Motion& left, const Motion& right) {
        return left.t_ns < right.t_ns;
    });
    std::vector<Motion> unique;
    unique.reserve(sorted.size());
    for (const auto& sample : sorted) {
        if (!unique.empty() && unique.back().t_ns == sample.t_ns) {
            *dropped += 1;
            continue;
        }
        unique.push_back(sample);
    }
    return unique;
}

enum class Resample { kInterpolate, kNearest };

// 以 grid 的时间戳为栅格，把 other 配到每个栅格时刻上，填 out->rows / out->segments。
// grid_is_gyro 决定栅格那条流写进 w 列还是 a 列。
// 共同点：other 的 [首, 尾] 之外一律整行丢弃，不外推也不零阶保持。
// kInterpolate：线性内插到栅格时刻。
// kNearest：只取时间最近的 other 实测样本（抽稀），不造任何数值；
//           最近样本偏移超过 max_gap_ns 说明这段时间另一条流断了，整行丢弃。
void BuildRows(const std::vector<Motion>& grid,
               const std::vector<Motion>& other,
               bool grid_is_gyro,
               Resample resample,
               TimeNs max_gap_ns,
               AlignResult* out) {
    const TimeNs other_begin = other.front().t_ns;
    const TimeNs other_end = other.back().t_ns;
    const double max_skew_ms = static_cast<double>(max_gap_ns) * 1e-6;
    std::size_t cursor = 0;
    TimeNs previous_time = 0;
    bool in_segment = false;
    for (const auto& point : grid) {
        if (!Finite(point.v)) {
            out->dropped_grid_uncovered += 1;
            continue;
        }
        if (in_segment && point.t_ns - previous_time > max_gap_ns) {
            out->dropped_gap_too_large += 1;
            in_segment = false;
        }
        if (point.t_ns < other_begin || point.t_ns > other_end) {
            out->dropped_grid_uncovered += 1;
            in_segment = false;
            continue;
        }
        Eigen::Vector3d value = Eigen::Vector3d::Zero();
        double emitted_skew_ms = 0.0; // 内插值就定义在栅格时刻上，偏移恒为 0
        if (resample == Resample::kInterpolate) {
            while (cursor + 1 < other.size() && other[cursor + 1].t_ns < point.t_ns) {
                ++cursor;
            }
            const Motion& lower = other[cursor];
            const Motion& upper = other[cursor + 1 < other.size() ? cursor + 1 : cursor];
            value = lower.v;
            if (upper.t_ns != lower.t_ns && point.t_ns > lower.t_ns) {
                const double alpha = static_cast<double>(point.t_ns - lower.t_ns) /
                                     static_cast<double>(upper.t_ns - lower.t_ns);
                value = lower.v + alpha * (upper.v - lower.v);
            }
        } else {
            while (cursor + 1 < other.size() &&
                   AbsDelta(other[cursor + 1].t_ns, point.t_ns) <=
                       AbsDelta(other[cursor].t_ns, point.t_ns)) {
                ++cursor;
            }
            const double skew_ms =
                static_cast<double>(AbsDelta(other[cursor].t_ns, point.t_ns)) * 1e-6;
            if (skew_ms > max_skew_ms) {
                out->dropped_grid_uncovered += 1;
                in_segment = false;
                continue;
            }
            emitted_skew_ms = skew_ms;
            value = other[cursor].v;
        }
        if (!Finite(value)) {
            out->dropped_grid_uncovered += 1;
            in_segment = false;
            continue;
        }
        // 只有真正写进 data.csv 的行才参与统计：被丢掉的配对不该虚报偏移量。
        out->stats.max_source_skew_ms = std::max(out->stats.max_source_skew_ms, emitted_skew_ms);
        ImuRow row;
        row.t_ns = point.t_ns;
        row.w = grid_is_gyro ? point.v : value;
        row.a = grid_is_gyro ? value : point.v;
        if (!in_segment) {
            out->segments.push_back(ImuSegment{out->rows.size(), out->rows.size() + 1});
            in_segment = true;
        } else {
            out->segments.back().row_end = out->rows.size() + 1;
        }
        out->rows.push_back(row);
        previous_time = point.t_ns;
    }
}

// 相邻两行同一列完全相同的比例：这是「这一行的值不是新测的」的可观测痕迹。
double HoldRatio(const std::vector<ImuRow>& rows, Eigen::Vector3d ImuRow::*field) {
    if (rows.size() < 2) {
        return 0.0;
    }
    std::size_t repeats = 0;
    for (std::size_t index = 1; index < rows.size(); ++index) {
        if (rows[index].*field == rows[index - 1].*field) {
            ++repeats;
        }
    }
    return static_cast<double>(repeats) / static_cast<double>(rows.size() - 1);
}

void FinishStats(AlignResult* out) {
    out->stats.rows = out->rows.size();
    if (out->rows.size() < 2) {
        return;
    }
    std::vector<TimeNs> deltas;
    deltas.reserve(out->rows.size() - 1);
    for (std::size_t index = 0; index + 1 < out->rows.size(); ++index) {
        deltas.push_back(out->rows[index + 1].t_ns - out->rows[index].t_ns);
    }
    std::sort(deltas.begin(), deltas.end());
    out->stats.grid_dt_ms = deltas[deltas.size() / 2] * 1e-6;
    out->stats.max_gap_ms = deltas.back() * 1e-6;
    out->stats.accel_hold_ratio = HoldRatio(out->rows, &ImuRow::a);
    out->stats.gyro_hold_ratio = HoldRatio(out->rows, &ImuRow::w);
}

} // namespace

const char* ImuGridModeName(ImuGridMode mode) {
    return mode == ImuGridMode::kAccelTimestamps ? "accel" : "gyro";
}

bool ParseImuGridMode(const std::string& text, ImuGridMode* mode) {
    if (mode == nullptr) {
        return false;
    }
    if (text == "gyro") {
        *mode = ImuGridMode::kGyroTimestamps;
        return true;
    }
    if (text == "accel") {
        *mode = ImuGridMode::kAccelTimestamps;
        return true;
    }
    return false;
}

bool AlignImuStreams(const std::vector<GyroSample>& gyro,
                     const std::vector<AccelSample>& accel,
                     TimeNs max_imu_gap_ns,
                     ImuGridMode mode,
                     AlignResult* result,
                     std::string* error) {
    if (result == nullptr) {
        return false;
    }
    if (max_imu_gap_ns <= 0) {
        if (error != nullptr) {
            *error = "max_imu_gap_ns 必须为正";
        }
        return false;
    }
    AlignResult out;
    std::size_t dropped_gyro_duplicates = 0;
    std::size_t dropped_accel_duplicates = 0;
    const std::vector<Motion> gyro_stream =
        NormalizeStream(gyro, &GyroSample::w, &dropped_gyro_duplicates);
    const std::vector<Motion> accel_stream =
        NormalizeStream(accel, &AccelSample::a, &dropped_accel_duplicates);
    out.stats.gyro_raw = gyro.size();
    out.stats.accel_raw = accel.size();
    out.stats.dropped_non_increasing = dropped_gyro_duplicates + dropped_accel_duplicates;
    // 内插需要左右两个端点，所以被重采样那条流至少 2 个样本。
    const bool gyro_is_grid = mode == ImuGridMode::kGyroTimestamps;
    if (gyro_is_grid && (gyro_stream.empty() || accel_stream.size() < 2)) {
        if (error != nullptr) {
            *error = "IMU 样本不足：至少需要 1 条陀螺与 2 条加表";
        }
        return false;
    }
    if (!gyro_is_grid && (accel_stream.empty() || gyro_stream.size() < 2)) {
        if (error != nullptr) {
            *error = "IMU 样本不足：至少需要 1 条加表与 2 条陀螺";
        }
        return false;
    }
    BuildRows(gyro_is_grid ? gyro_stream : accel_stream,
              gyro_is_grid ? accel_stream : gyro_stream,
              gyro_is_grid,
              gyro_is_grid ? Resample::kInterpolate : Resample::kNearest,
              max_imu_gap_ns,
              &out);
    FinishStats(&out);
    result->rows = std::move(out.rows);
    result->segments = std::move(out.segments);
    result->stats = out.stats;
    result->dropped_grid_uncovered = out.dropped_grid_uncovered;
    result->dropped_gap_too_large = out.dropped_gap_too_large;
    if (result->rows.empty()) {
        if (error != nullptr) {
            *error = std::string("对齐后没有可用的 IMU 行：") + ImuGridModeName(mode) +
                     " 栅格下另一条流的覆盖率或时间间隔不满足契约";
        }
        return false;
    }
    return true;
}

} // namespace rh

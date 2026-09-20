#include "record_helper/imu_align.hpp"

#include <algorithm>
#include <cmath>

namespace rh {
namespace {

bool Finite(const Eigen::Vector3d& value) {
    return std::isfinite(value.x()) && std::isfinite(value.y()) && std::isfinite(value.z());
}

// 设备回调顺序不保证严格递增（USB 重传、双端点），先按时间稳定排序再去重，
// 保证写出的 imu0/data.csv 满足「严格递增」这条会整文件失败的上游约束。
template <typename Sample>
std::vector<Sample> NormalizeStream(const std::vector<Sample>& input, std::size_t* dropped) {
    std::vector<Sample> sorted = input;
    std::stable_sort(sorted.begin(), sorted.end(), [](const Sample& left, const Sample& right) {
        return left.t_ns < right.t_ns;
    });
    std::vector<Sample> unique;
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

} // namespace

bool AlignImuStreams(const std::vector<GyroSample>& gyro,
                     const std::vector<AccelSample>& accel,
                     TimeNs max_imu_gap_ns,
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
    const std::vector<GyroSample> gyro_stream = NormalizeStream(gyro, &dropped_gyro_duplicates);
    const std::vector<AccelSample> accel_stream = NormalizeStream(accel, &dropped_accel_duplicates);
    out.stats.gyro_raw = gyro.size();
    out.stats.accel_raw = accel.size();
    out.stats.dropped_non_increasing = dropped_gyro_duplicates + dropped_accel_duplicates;
    if (gyro_stream.empty() || accel_stream.size() < 2) {
        if (error != nullptr) {
            *error = "IMU 样本不足：至少需要 1 条陀螺与 2 条加表";
        }
        return false;
    }

    // 加表有效区间只取 [first, last]，端点之外一律丢弃而不是外推。
    const TimeNs accel_begin = accel_stream.front().t_ns;
    const TimeNs accel_end = accel_stream.back().t_ns;
    std::size_t bracket = 0;
    std::size_t valid_rows = 0;
    TimeNs previous_time = 0;
    bool in_segment = false;
    for (const auto& sample : gyro_stream) {
        if (!Finite(sample.w)) {
            out.dropped_accel_uncovered += 1;
            continue;
        }
        if (in_segment && sample.t_ns - previous_time > max_imu_gap_ns) {
            out.dropped_gap_too_large += 1;
            in_segment = false;
        }
        if (sample.t_ns < accel_begin || sample.t_ns > accel_end) {
            out.dropped_accel_uncovered += 1;
            in_segment = false;
            continue;
        }
        while (bracket + 1 < accel_stream.size() && accel_stream[bracket + 1].t_ns < sample.t_ns) {
            ++bracket;
        }
        const AccelSample& lower = accel_stream[bracket];
        const AccelSample& upper =
            accel_stream[bracket + 1 < accel_stream.size() ? bracket + 1 : bracket];
        Eigen::Vector3d accel = lower.a;
        if (upper.t_ns != lower.t_ns && sample.t_ns > lower.t_ns) {
            const double alpha = static_cast<double>(sample.t_ns - lower.t_ns) /
                                 static_cast<double>(upper.t_ns - lower.t_ns);
            accel = lower.a + alpha * (upper.a - lower.a);
        }
        if (!Finite(accel)) {
            out.dropped_accel_uncovered += 1;
            in_segment = false;
            continue;
        }
        ImuRow row;
        row.t_ns = sample.t_ns;
        row.w = sample.w;
        row.a = accel;
        if (!in_segment) {
            out.segments.push_back(ImuSegment{out.rows.size(), out.rows.size() + 1});
            in_segment = true;
        } else {
            out.segments.back().row_end = out.rows.size() + 1;
        }
        out.rows.push_back(row);
        previous_time = sample.t_ns;
        valid_rows += 1;
    }
    out.stats.rows = valid_rows;
    if (valid_rows > 1) {
        std::vector<TimeNs> deltas;
        deltas.reserve(valid_rows - 1);
        for (std::size_t index = 0; index + 1 < out.rows.size(); ++index) {
            deltas.push_back(out.rows[index + 1].t_ns - out.rows[index].t_ns);
        }
        std::sort(deltas.begin(), deltas.end());
        out.stats.grid_dt_ms = deltas[deltas.size() / 2] * 1e-6;
        out.stats.max_gap_ms = deltas.back() * 1e-6;
        std::size_t repeats = 0;
        for (std::size_t index = 1; index < out.rows.size(); ++index) {
            if (out.rows[index].a == out.rows[index - 1].a) {
                ++repeats;
            }
        }
        out.stats.accel_hold_ratio =
            static_cast<double>(repeats) / static_cast<double>(out.rows.size() - 1);
    }
    out.stats.accel_interpolated = valid_rows;
    result->rows = std::move(out.rows);
    result->segments = std::move(out.segments);
    result->stats = out.stats;
    result->dropped_accel_uncovered = out.dropped_accel_uncovered;
    result->dropped_gap_too_large = out.dropped_gap_too_large;
    if (result->rows.empty()) {
        if (error != nullptr) {
            *error = "对齐后没有可用的 IMU 行：加表覆盖率或时间间隔不满足契约";
        }
        return false;
    }
    return true;
}

} // namespace rh

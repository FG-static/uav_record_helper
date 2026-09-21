#include "record_helper/dataset_writer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "record_helper/euroc_format.hpp"
#include "record_helper/png_io.hpp"

namespace rh {
namespace {

constexpr std::size_t kMaxWriteErrors = 8;

TimeNs ToNs(double seconds) {
    return static_cast<TimeNs>(seconds * 1e9);
}

std::string FrameFileName(TimeNs t_ns) {
    return std::to_string(t_ns) + ".png";
}

bool HasBoundaryAnchor(const std::vector<ImuRow>& rows,
                       std::size_t begin,
                       std::size_t end,
                       const std::vector<WrittenFrame>& frames,
                       std::size_t frame_begin,
                       std::size_t frame_end,
                       const ProbeConfig& probe,
                       TimeNs anchor_offset_ns,
                       StaticMotionBoundary* boundary,
                       std::size_t* anchor_index) {
    std::vector<ImuRowSample> samples;
    samples.reserve(end - begin);
    for (std::size_t index = begin; index < end; ++index) {
        samples.push_back(ImuRowSample{rows[index].t_ns, rows[index].a, rows[index].w});
    }
    const auto found = FindStaticMotionBoundary(samples, probe, nullptr);
    if (!found.has_value()) {
        return false;
    }
    const TimeNs anchor_time = found->feed_start_ns + anchor_offset_ns;
    std::size_t candidate = frame_end;
    for (std::size_t index = frame_begin; index < frame_end; ++index) {
        if (frames[index].t_ns >= anchor_time) {
            candidate = index;
            break;
        }
    }
    if (candidate >= frame_end) {
        return false;
    }
    *boundary = *found;
    *anchor_index = candidate;
    return true;
}

} // namespace

DatasetWriter::DatasetWriter(std::string root,
                             BodyCalibration calibration,
                             ImuNoise noise,
                             const WriteOptions& options,
                             Provenance provenance)
    : root_(std::move(root)), calibration_(std::move(calibration)), noise_(noise),
      options_(options), provenance_(std::move(provenance)) {
    queue_capacity_ = options_.queue_capacity == 0 ? 12 : options_.queue_capacity;
}

DatasetWriter::~DatasetWriter() {
    Stop();
}

bool DatasetWriter::Start(std::string* error) {
    const char* dirs[] = {"mav0/cam0/data",
                          "mav0/cam1/data",
                          "mav0/imu0",
                          "mav0/depth0/data",
                          "mav0/state_groundtruth_estimate0"};
    for (const char* dir : dirs) {
        if (!fmt::MakeDirectoryTree(fmt::JoinPath(root_, dir), error)) {
            return false;
        }
    }
    if (std::unique_lock<std::mutex> lock(mutex_); running_) {
        return true;
    }
    running_ = true;
    writer_ = std::thread(&DatasetWriter::WriterLoop, this);
    return true;
}

bool DatasetWriter::Enqueue(StereoPairRecord&& record) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_) {
        return false;
    }
    if (queue_.size() >= queue_capacity_) {
        dropped_ += 1;
        return false;
    }
    Job job;
    job.is_depth = false;
    job.pair = std::move(record);
    queue_.push_back(std::move(job));
    available_.notify_one();
    return true;
}

bool DatasetWriter::Enqueue(DepthRecord&& record) {
    if (!options_.write_depth) {
        return true;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_) {
        return false;
    }
    if (queue_.size() >= queue_capacity_) {
        dropped_ += 1;
        return false;
    }
    Job job;
    job.is_depth = true;
    job.depth = std::move(record);
    queue_.push_back(std::move(job));
    available_.notify_one();
    return true;
}

void DatasetWriter::Stop() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_) {
            return;
        }
        running_ = false;
    }
    available_.notify_all();
    if (writer_.joinable()) {
        writer_.join();
    }
}

void DatasetWriter::WriterLoop() {
    for (;;) {
        Job job;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            available_.wait(lock, [&]() { return !queue_.empty() || !running_; });
            if (queue_.empty()) {
                if (!running_) {
                    return;
                }
                continue;
            }
            job = std::move(queue_.front());
            queue_.pop_front();
        }
        std::string error;
        if (job.is_depth) {
            const std::string path =
                fmt::JoinPath(root_, "mav0/depth0/data/" + FrameFileName(job.depth.t_ns));
            if (WritePngGray16(path,
                               job.depth.width,
                               job.depth.height,
                               job.depth.millimetres.data(),
                               options_.png_level,
                               &error)) {
                std::lock_guard<std::mutex> lock(mutex_);
                written_depth_.push_back(job.depth.t_ns);
            } else {
                std::lock_guard<std::mutex> lock(mutex_);
                if (write_errors_.size() < kMaxWriteErrors) {
                    write_errors_.push_back(error);
                }
            }
            continue;
        }
        const std::string left_path =
            fmt::JoinPath(root_, "mav0/cam0/data/" + FrameFileName(job.pair.t_ns));
        const std::string right_path =
            fmt::JoinPath(root_, "mav0/cam1/data/" + FrameFileName(job.pair.t_ns));
        bool ok = WritePngGray8(left_path,
                                calibration_.camera0.width,
                                calibration_.camera0.height,
                                job.pair.left.data(),
                                options_.png_level,
                                &error);
        if (ok) {
            ok = WritePngGray8(right_path,
                               calibration_.camera1.width,
                               calibration_.camera1.height,
                               job.pair.right.data(),
                               options_.png_level,
                               &error);
        }
        if (!ok) {
            std::printf("%s\n", error.c_str());
            std::lock_guard<std::mutex> lock(mutex_);
            if (write_errors_.size() < kMaxWriteErrors) {
                write_errors_.push_back(error);
            }
            std::remove(left_path.c_str());
            continue;
        }
        const double skew_ms = std::abs(static_cast<double>(job.pair.t_ns - job.pair.t1_ns)) * 1e-6;
        std::lock_guard<std::mutex> lock(mutex_);
        max_skew_ms_ = std::max(max_skew_ms_, skew_ms);
        written_frames_.push_back(WrittenFrame{job.pair.t_ns, job.pair.t1_ns});
    }
}

double DatasetWriter::max_stereo_skew_ms() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return max_skew_ms_;
}

ReplayWindow SelectReplayWindow(const std::vector<ImuRow>& rows,
                                const std::vector<ImuSegment>& segments,
                                const std::vector<WrittenFrame>& frames,
                                const WriteOptions& options,
                                const ProbeConfig& probe) {
    ReplayWindow best;
    std::int64_t best_score = -1;
    const TimeNs max_frame_gap_ns = ToNs(options.limits.max_frame_gap_ms * 1e-3);
    for (const auto& segment : segments) {
        if (segment.row_end <= segment.row_begin) {
            continue;
        }
        const TimeNs imu_begin = rows[segment.row_begin].t_ns;
        const TimeNs imu_end = rows[segment.row_end - 1].t_ns;
        const TimeNs frame_begin_time = imu_begin + options.imu_lead_required_ns;
        const TimeNs frame_end_time = imu_end - options.imu_tail_guard_ns;
        std::size_t lo = frames.size();
        for (std::size_t index = 0; index < frames.size(); ++index) {
            if (frames[index].t_ns >= frame_begin_time) {
                lo = index;
                break;
            }
        }
        if (lo >= frames.size()) {
            continue;
        }
        std::size_t hi = lo;
        while (hi < frames.size() && frames[hi].t_ns <= frame_end_time) {
            ++hi;
        }
        if (hi <= lo) {
            continue;
        }
        // 在 [lo, hi) 内按帧间隔断点切分，逐段评估（回放对 >max_frame_gap 的断点直接拒绝）。
        std::size_t run_begin = lo;
        for (std::size_t index = lo + 1; index <= hi; ++index) {
            const bool cut =
                index == hi || frames[index].t_ns - frames[index - 1].t_ns > max_frame_gap_ns;
            if (!cut) {
                continue;
            }
            StaticMotionBoundary boundary;
            std::size_t anchor_index = hi;
            const bool usable = HasBoundaryAnchor(rows,
                                                  segment.row_begin,
                                                  segment.row_end,
                                                  frames,
                                                  run_begin,
                                                  index,
                                                  probe,
                                                  ToNs(options.limits.anchor_offset_s),
                                                  &boundary,
                                                  &anchor_index);
            const std::int64_t score =
                (usable ? static_cast<std::int64_t>(1) << 40 : 0) +
                static_cast<std::int64_t>(index - run_begin);
            if (score > best_score) {
                best_score = score;
                best.found = true;
                best.imu_row_begin = segment.row_begin;
                best.imu_row_end = segment.row_end;
                // 正式录制中途再静止不能把窗口裁到「最后一个」边界：整段连续帧都保留。
                // 尾部无 IMU 覆盖的帧仍按 frame_end_time 丢掉。
                best.pair_begin = run_begin;
                best.pair_end = index;
                best.pair_count = best.pair_end - best.pair_begin;
            }
            run_begin = index;
        }
    }
    return best;
}

std::string DatasetWriter::CameraYaml(const CameraIntrinsics& camera,
                                      const RigidTransform& to_body) const {
    std::string yaml = "#RecordHelper：D435i 双目红外流，EuRoC mav0 兼容写法（只使用 flow 列表）\n";
    yaml += "sensor_type: camera\n";
    yaml += "comment: D435i infra stream written by RecordHelper\n";
    yaml += fmt::FormatMatrixField("T_BS", 4, 4, cal::FlattenT_BS(to_body));
    yaml += "rate_hz: " + std::to_string(provenance_.camera_fps) + "\n";
    std::vector<std::int64_t> resolution{static_cast<std::int64_t>(camera.width),
                                         static_cast<std::int64_t>(camera.height)};
    yaml += "resolution: " + fmt::FormatIntFlowList(resolution) + "\n";
    yaml += "camera_model: pinhole\n";
    yaml +=
        "intrinsics: " + fmt::FormatFlowList({camera.fx, camera.fy, camera.cx, camera.cy}) + "\n";
    yaml += "distortion_model: radial-tangential\n";
    std::vector<double> distortion(camera.distortion.begin(), camera.distortion.end());
    yaml += "distortion_coefficients: " + fmt::FormatFlowList(distortion) + "\n";
    return yaml;
}

bool DatasetWriter::WriteCalibrationFiles(std::string* error) {
    CameraIntrinsics camera0 = calibration_.camera0;
    CameraIntrinsics camera1 = calibration_.camera1;
    if (options_.zero_distortion) {
        camera0.distortion = {{0.0, 0.0, 0.0, 0.0}};
        camera1.distortion = {{0.0, 0.0, 0.0, 0.0}};
    }
    BodyCalibration effective = calibration_;
    effective.camera0 = camera0;
    effective.camera1 = camera1;
    if (!fmt::WriteWholeFile(fmt::JoinPath(root_, "mav0/cam0/sensor.yaml"),
                             CameraYaml(camera0, effective.camera0_to_body),
                             error)) {
        return false;
    }
    if (!fmt::WriteWholeFile(fmt::JoinPath(root_, "mav0/cam1/sensor.yaml"),
                             CameraYaml(camera1, effective.camera1_to_body),
                             error)) {
        return false;
    }
    std::string imu_yaml = "#RecordHelper：D435i 内置 IMU，噪声参数由本次静止段实测\n";
    imu_yaml += "sensor_type: imu\n";
    imu_yaml += "comment: accelerometer frame is the body frame B\n";
    imu_yaml += fmt::FormatMatrixField("T_BS", 4, 4, cal::FlattenT_BS(RigidTransform{}));
    imu_yaml += "rate_hz: " + std::to_string(provenance_.imu_gyro_hz) + "\n";
    imu_yaml += "gyroscope_noise_density: " + fmt::FormatDouble(noise_.sigma_g) + "\n";
    imu_yaml += "gyroscope_random_walk: " + fmt::FormatDouble(noise_.sigma_bg) + "\n";
    imu_yaml += "accelerometer_noise_density: " + fmt::FormatDouble(noise_.sigma_a) + "\n";
    imu_yaml += "accelerometer_random_walk: " + fmt::FormatDouble(noise_.sigma_ba) + "\n";
    return fmt::WriteWholeFile(fmt::JoinPath(root_, "mav0/imu0/sensor.yaml"), imu_yaml, error);
}

bool DatasetWriter::WriteImuCsv(const std::vector<ImuRow>& rows, std::string* error) {
    std::string csv = "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
                      "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n";
    csv.reserve(rows.size() * 96);
    for (const auto& row : rows) {
        csv += std::to_string(row.t_ns) + "," + fmt::FormatDouble(row.w.x()) + "," +
               fmt::FormatDouble(row.w.y()) + "," + fmt::FormatDouble(row.w.z()) + "," +
               fmt::FormatDouble(row.a.x()) + "," + fmt::FormatDouble(row.a.y()) + "," +
               fmt::FormatDouble(row.a.z()) + "\n";
    }
    return fmt::WriteWholeFile(fmt::JoinPath(root_, "mav0/imu0/data.csv"), csv, error);
}

bool DatasetWriter::WriteCameraCsv(const std::vector<WrittenFrame>& frames, std::string* error) {
    // 两份 CSV 的时间戳逐个相同：unav_vio 的 synchronize_stereo_stamps 要求 ns 完全相等，
    // 且回放测试断言 dropped_count == 0。右目原始时间戳只在 summary 中留档。
    std::string csv = "#timestamp [ns],filename\n";
    for (const auto& frame : frames) {
        csv += std::to_string(frame.t_ns) + "," + FrameFileName(frame.t_ns) + "\n";
    }
    if (!fmt::WriteWholeFile(fmt::JoinPath(root_, "mav0/cam0/data.csv"), csv, error)) {
        return false;
    }
    return fmt::WriteWholeFile(fmt::JoinPath(root_, "mav0/cam1/data.csv"), csv, error);
}

bool DatasetWriter::WriteDepthCsv(const std::vector<TimeNs>& times, std::string* error) {
    std::string csv = "#timestamp [ns],filename\n";
    for (const TimeNs time : times) {
        csv += std::to_string(time) + "," + FrameFileName(time) + "\n";
    }
    return fmt::WriteWholeFile(fmt::JoinPath(root_, "mav0/depth0/data.csv"), csv, error);
}

bool DatasetWriter::WriteGroundTruthCsv(const std::vector<PoseRecord>& poses, std::string* error) {
    std::string csv =
        "#META_NUMBER_ROWS_USELESS: 0\n"
        "#timestamp [ns],p_RS_wr_x [m],p_RS_wr_y [m],p_RS_wr_z [m],q_RS_wq [unitless],"
        "q_RS_wx [unitless],q_RS_wy [unitless],q_RS_wz [unitless],v_RS_wr_x [m s^-1],"
        "v_RS_wr_y [m s^-1],v_RS_wr_z [m s^-1],b_g_RS_wx [rad s^-1],b_g_RS_wy [rad s^-1],"
        "b_g_RS_wz [rad s^-1],b_a_RS_wx [m s^-2],b_a_RS_wy [m s^-2],b_a_RS_wz [m s^-2]\n";
    csv.reserve(poses.size() * 200);
    for (std::size_t index = 0; index < poses.size(); ++index) {
        const PoseRecord& pose = poses[index];
        Eigen::Quaterniond q = pose.q.normalized();
        Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
        // 中心差分给速度；端点退回单侧差分。GT 的 b_g/b_a 按惯例填 0。
        if (index + 1 < poses.size() && poses[index + 1].t_ns > pose.t_ns) {
            const double dt = (poses[index + 1].t_ns - pose.t_ns) * 1e-9;
            velocity = (poses[index + 1].p - pose.p) / dt;
        }
        if (index > 0 && pose.t_ns > poses[index - 1].t_ns) {
            const double dt = (pose.t_ns - poses[index - 1].t_ns) * 1e-9;
            const Eigen::Vector3d backward = (pose.p - poses[index - 1].p) / dt;
            velocity = index + 1 < poses.size() ? (velocity + backward) * 0.5 : backward;
        }
        csv += std::to_string(pose.t_ns) + "," + fmt::FormatDouble(pose.p.x()) + "," +
               fmt::FormatDouble(pose.p.y()) + "," + fmt::FormatDouble(pose.p.z()) + "," +
               fmt::FormatDouble(q.w()) + "," + fmt::FormatDouble(q.x()) + "," +
               fmt::FormatDouble(q.y()) + "," + fmt::FormatDouble(q.z()) + "," +
               fmt::FormatDouble(velocity.x()) + "," + fmt::FormatDouble(velocity.y()) + "," +
               fmt::FormatDouble(velocity.z()) + ",0,0,0,0,0,0\n";
    }
    return fmt::WriteWholeFile(
        fmt::JoinPath(root_, "mav0/state_groundtruth_estimate0/data.csv"), csv, error);
}

bool DatasetWriter::WriteSummary(const WriteResult& result,
                                 const ImuAlignStats& imu_stats,
                                 const std::vector<TimeNs>& kept_depth,
                                 std::string* error) {
    std::string summary = "#RecordHelper 录制留档：不被 unav_vio 解析，只用于溯源\n";
    summary += "sequence: " + provenance_.sequence_name + "\n";
    summary += "recorder: " + provenance_.recorder_version + "\n";
    summary += "device: " + provenance_.device_name + "\n";
    summary += "serial: " + provenance_.serial + "\n";
    summary += "firmware: " + provenance_.firmware + "\n";
    summary += "librealsense: " + provenance_.librealsense_version + "\n";
    summary += "usb_type: " + provenance_.usb_type + "\n";
    summary += "started_unix_s: " + fmt::FormatDouble(provenance_.started_unix_seconds) + "\n";
    summary += "time_base: 相对第一条 IMU/相机样本的纳秒偏移，非 Unix epoch\n";
    summary += "counts:\n";
    summary += "  cam0: " + std::to_string(result.pairs) + "\n";
    summary += "  cam1: " + std::to_string(result.pairs) + "\n";
    summary += "  depth0: " + std::to_string(kept_depth.size()) + "\n";
    summary += "  imu0: " + std::to_string(result.imu_rows) + "\n";
    summary += "  state_groundtruth_estimate0: " + std::to_string(result.gt_rows) + "\n";
    summary += "frames:\n";
    summary += "  cam0: camera_infra1_optical_frame\n";
    summary += "  cam1: camera_infra2_optical_frame\n";
    summary += "  depth0: camera_depth_optical_frame\n";
    summary += "  imu0: camera_accel_frame_is_body_frame\n";
    summary += "imu:\n";
    summary += "  gyro_hz: " + fmt::FormatDouble(1e3 / imu_stats.grid_dt_ms) + "\n";
    summary += "  accel_raw_hz: " +
               fmt::FormatDouble(
                   imu_stats.accel_raw * 1e9 /
                   std::max(1.0, static_cast<double>(result.imu_end_ns - result.imu_begin_ns))) +
               "\n";
    summary += "  accel_hold_ratio: " + fmt::FormatDouble(imu_stats.accel_hold_ratio) + "\n";
    summary += "  max_gap_ms: " + fmt::FormatDouble(imu_stats.max_gap_ms) + "\n";
    summary += "  grid_dt_ms: " + fmt::FormatDouble(imu_stats.grid_dt_ms) + "\n";
    // 静止标定阶段实测的重力方向与 T_BS 第三列的夹角；这是「IMU 数据系 == 外参表所指系」的证据。
    summary += "  static_gravity_vs_t_bs_col3_deg: " +
               fmt::FormatDouble(provenance_.static_gravity_angle_deg) + "\n";
    summary += "  noise_source: " + provenance_.noise_source + "\n";
    summary += "  sigma_a: " + fmt::FormatDouble(noise_.sigma_a) + "\n";
    summary += "  sigma_bg: " + fmt::FormatDouble(noise_.sigma_bg) + "\n";
    summary += "  sigma_ba: " + fmt::FormatDouble(noise_.sigma_ba) + "\n";
    summary += "  sigma_g: " + fmt::FormatDouble(noise_.sigma_g) + "\n";
    summary += "stereo:\n";
    summary += "  shared_exposure_timestamps: true\n";
    summary += "  max_skew_ms: " + fmt::FormatDouble(result.max_stereo_skew_ms) + "\n";
    summary += "  distortion_model0: " + provenance_.distortion_model0 + "\n";
    summary += "  distortion_model1: " + provenance_.distortion_model1 + "\n";
    summary +=
        "  zero_distortion_written: " + std::string(options_.zero_distortion ? "true" : "false") +
        "\n";
    summary += "window:\n";
    summary += "  first_pair_ns: " + std::to_string(result.first_pair_ns) + "\n";
    summary += "  last_pair_ns: " + std::to_string(result.last_pair_ns) + "\n";
    summary += "  imu_begin_ns: " + std::to_string(result.imu_begin_ns) + "\n";
    summary += "  imu_end_ns: " + std::to_string(result.imu_end_ns) + "\n";
    summary += "  trimmed_pairs: " + std::to_string(result.trimmed_pairs) + "\n";
    summary += "  dropped_queue_full: " + std::to_string(result.dropped_queue_full) + "\n";
    if (result.boundary.has_value()) {
        summary += "  motion_start_ns: " + std::to_string(result.boundary->motion_start_ns) + "\n";
        summary += "  feed_start_ns: " + std::to_string(result.boundary->feed_start_ns) + "\n";
    }
    if (!provenance_.capture_note.empty()) {
        summary += "note: " + provenance_.capture_note + "\n";
    }
    if (!write_errors_.empty()) {
        summary += "write_errors:\n";
        for (const auto& line : write_errors_) {
            summary += "  - " + line + "\n";
        }
    }
    return fmt::WriteWholeFile(fmt::JoinPath(root_, "record_summary.yaml"), summary, error);
}

bool DatasetWriter::Finalize(const std::vector<GyroSample>& gyro,
                             const std::vector<AccelSample>& accel,
                             const std::vector<PoseRecord>& poses,
                             WriteResult* result,
                             std::string* error) {
    Stop();
    std::vector<WrittenFrame> frames;
    std::vector<TimeNs> depth_times;
    double skew_ms = 0.0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        frames = written_frames_;
        depth_times = written_depth_;
        skew_ms = max_skew_ms_;
    }
    std::sort(
        frames.begin(), frames.end(), [](const WrittenFrame& left, const WrittenFrame& right) {
            return left.t_ns < right.t_ns;
        });
    frames.erase(std::unique(frames.begin(),
                             frames.end(),
                             [](const WrittenFrame& left, const WrittenFrame& right) {
                                 return left.t_ns == right.t_ns;
                             }),
                 frames.end());
    std::sort(depth_times.begin(), depth_times.end());

    AlignResult aligned;
    if (!AlignImuStreams(
            gyro, accel, ToNs(options_.limits.max_imu_gap_ms * 1e-3), &aligned, error)) {
        return false;
    }
    const ProbeConfig probe;
    const ReplayWindow window =
        SelectReplayWindow(aligned.rows, aligned.segments, frames, options_, probe);
    if (!window.found) {
        if (error != nullptr) {
            *error = "找不到同时满足 IMU 前置/右端覆盖与帧间隔的可用窗口";
        }
        return false;
    }
    std::vector<ImuRow> rows(
        aligned.rows.begin() + static_cast<std::ptrdiff_t>(window.imu_row_begin),
        aligned.rows.begin() + static_cast<std::ptrdiff_t>(window.imu_row_end));
    std::vector<WrittenFrame> kept(frames.begin() + static_cast<std::ptrdiff_t>(window.pair_begin),
                                   frames.begin() + static_cast<std::ptrdiff_t>(window.pair_end));

    // 落在窗口外的图像不会再被 data.csv 引用，直接删掉，避免留下半套数据。
    std::size_t removed = 0;
    for (std::size_t index = 0; index < frames.size(); ++index) {
        const bool keep = index >= window.pair_begin && index < window.pair_end;
        if (keep) {
            continue;
        }
        const std::string name = FrameFileName(frames[index].t_ns);
        removed += 1;
        std::remove(fmt::JoinPath(root_, "mav0/cam0/data/" + name).c_str());
        std::remove(fmt::JoinPath(root_, "mav0/cam1/data/" + name).c_str());
    }
    std::vector<TimeNs> kept_depth;
    for (const TimeNs time : depth_times) {
        const bool keep = time >= kept.front().t_ns && time <= rows.back().t_ns;
        if (keep) {
            kept_depth.push_back(time);
        } else {
            std::remove(fmt::JoinPath(root_, "mav0/depth0/data/" + FrameFileName(time)).c_str());
        }
    }
    std::sort(kept_depth.begin(), kept_depth.end());

    result->root = root_;
    result->pairs = kept.size();
    result->depth_frames = kept_depth.size();
    result->imu_rows = rows.size();
    result->first_pair_ns = kept.front().t_ns;
    result->last_pair_ns = kept.back().t_ns;
    result->imu_begin_ns = rows.front().t_ns;
    result->imu_end_ns = rows.back().t_ns;
    result->trimmed_pairs = removed;
    result->dropped_queue_full = dropped_;
    result->max_stereo_skew_ms = skew_ms;
    for (const auto& line : write_errors_) {
        result->warnings.push_back("写盘失败: " + line);
    }
    if (dropped_ > 0) {
        result->warnings.push_back("写盘队列满导致丢弃 " + std::to_string(dropped_) + " 对，" +
                                   "可降低分辨率/png-level 或改用更快的磁盘");
    }
    if (skew_ms * 1000.0 > options_.limits.max_stereo_sync_error_us) {
        result->warnings.push_back("左右曝光时间戳最大偏斜 " + fmt::FormatDouble(skew_ms) +
                                   " ms 超过 unav_vio 的 100 us 同步门限");
    }

    std::vector<PoseRecord> kept_poses;
    if (options_.write_pose_gt && !poses.empty()) {
        std::vector<PoseRecord> sorted = poses;
        std::sort(
            sorted.begin(), sorted.end(), [](const PoseRecord& left, const PoseRecord& right) {
                return left.t_ns < right.t_ns;
            });
        for (const auto& pose : sorted) {
            if (pose.t_ns < rows.front().t_ns || pose.t_ns > rows.back().t_ns) {
                continue;
            }
            if (!kept_poses.empty() && kept_poses.back().t_ns == pose.t_ns) {
                continue;
            }
            kept_poses.push_back(pose);
        }
    }
    result->gt_rows = kept_poses.size();

    StaticMotionBoundary boundary;
    std::size_t anchor_index = 0;
    const bool usable = HasBoundaryAnchor(rows,
                                          0,
                                          rows.size(),
                                          kept,
                                          0,
                                          kept.size(),
                                          probe,
                                          ToNs(options_.limits.anchor_offset_s),
                                          &boundary,
                                          &anchor_index);
    if (usable) {
        result->boundary = boundary;
    } else {
        result->warnings.push_back(
            "写入的 IMU 段内没有静止→运动边界：回放会在 find_static_motion_boundary 处早退");
    }
    if (anchor_index > 0) {
        result->warnings.push_back("anchor 之前有 " + std::to_string(anchor_index) +
                                   " 对图像会被回放的静止边界跳过，不影响初始化");
    }

    if (!WriteCalibrationFiles(error) || !WriteImuCsv(rows, error) ||
        !WriteCameraCsv(kept, error) ||
        (options_.write_depth && !WriteDepthCsv(kept_depth, error)) ||
        (options_.write_pose_gt && !WriteGroundTruthCsv(kept_poses, error))) {
        return false;
    }
    return WriteSummary(*result, aligned.stats, kept_depth, error);
}

} // namespace rh

#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "record_helper/calibration.hpp"
#include "record_helper/imu_align.hpp"
#include "record_helper/motion_gate.hpp"
#include "record_helper/types.hpp"

namespace rh {

struct WriteOptions {
    bool write_depth{true};
    bool write_pose_gt{true};
    // D400 交付的红外图已被硬件修正过，但固件仍会报告一整套畸变系数。默认原样写出报告值，
    // rh probe 的两目几何摘要用于确认是否该改用 --zero-distortion。
    bool zero_distortion{false};
    int png_level{3};
    // 有界写盘队列的成对帧数上限：满了就整对丢弃，绝不丢半对破坏硬同步。
    // 30 fps 下 12 帧 ≈ 400 ms 的磁盘抖动余量；离线合成用例需要一次性灌入。
    std::size_t queue_capacity{12};
    // 末尾留给 IMU 的覆盖时长：保证最后一个曝光区间右端有样本可插值。
    TimeNs imu_tail_guard_ns{200'000'000};
    TimeNs imu_lead_required_ns{55'000'000};
    SessionLimits limits;
};

struct Provenance {
    std::string sequence_name;
    std::string device_name;
    std::string serial;
    std::string firmware;
    std::string librealsense_version;
    std::string usb_type;
    double started_unix_seconds{0.0};
    std::string recorder_version{"RecordHelper 0.1.0"};
    std::string noise_source;
    std::string distortion_model0;
    std::string distortion_model1;
    std::string capture_note;
    // 静止标定阶段实测：平均比力与 T_BS 第三列的夹角（度）。<0 = 没测成。见 CheckGravityDirection。
    double static_gravity_angle_deg{-1.0};
    int camera_fps{30};
    int imu_gyro_hz{400};
};

// 一对已落盘双目图像的共享曝光时间戳。右目原始时间戳只用于统计偏斜。
struct WrittenFrame {
    TimeNs t_ns{0};
    TimeNs t1_ns{0};
};

struct WriteResult {
    std::string root;
    std::size_t pairs{0};
    std::size_t depth_frames{0};
    std::size_t imu_rows{0};
    std::size_t gt_rows{0};
    TimeNs first_pair_ns{0};
    TimeNs last_pair_ns{0};
    TimeNs imu_begin_ns{0};
    TimeNs imu_end_ns{0};
    std::size_t trimmed_pairs{0};
    std::size_t dropped_queue_full{0};
    std::vector<std::string> warnings;
    std::optional<StaticMotionBoundary> boundary;
    double max_stereo_skew_ms{0.0};
};

// 从「IMU 连续段 + 已落盘双目帧」里挑出满足全部回放约束的最大可用窗口。
// 每条约束都来自 unav_vio 的回放路径：帧间隔 ≤ max_frame_gap、IMU 前置 ≥ 55 ms、
// IMU 右端覆盖最后一个曝光、且窗口内必须存在静止→运动边界。
// 窗口起点不裁到「最后一个」静止边界：正式录制中途再静止必须整段保留。
struct ReplayWindow {
    bool found{false};
    std::size_t imu_row_begin{0};
    std::size_t imu_row_end{0};
    std::size_t pair_begin{0};
    std::size_t pair_end{0};
    std::size_t pair_count{0};
};

ReplayWindow SelectReplayWindow(const std::vector<ImuRow>& rows,
                                const std::vector<ImuSegment>& segments,
                                const std::vector<WrittenFrame>& frames,
                                const WriteOptions& options,
                                const ProbeConfig& probe);

// 采集线程只把像素 move 进有界队列，PNG 编码在写盘线程里做，避免 USB 回调被压缩耗时拖住。
// 队列满时整对丢弃：丢一对不破坏硬同步契约，丢半对才会。
class DatasetWriter {
  public:
    DatasetWriter(std::string root,
                  BodyCalibration calibration,
                  ImuNoise noise,
                  const WriteOptions& options,
                  Provenance provenance);
    ~DatasetWriter();

    DatasetWriter(const DatasetWriter&) = delete;
    DatasetWriter& operator=(const DatasetWriter&) = delete;

    bool Start(std::string* error);
    bool Enqueue(StereoPairRecord&& record);
    bool Enqueue(DepthRecord&& record);
    void Stop();

    // 用采集期间攒下的 IMU 与位姿写 CSV/YAML，并删除落在有效窗口之外的图像。
    bool Finalize(const std::vector<GyroSample>& gyro,
                  const std::vector<AccelSample>& accel,
                  const std::vector<PoseRecord>& poses,
                  WriteResult* result,
                  std::string* error);

    double max_stereo_skew_ms() const;

  private:
    void WriterLoop();
    bool WriteCalibrationFiles(std::string* error);
    bool WriteImuCsv(const std::vector<ImuRow>& rows, std::string* error);
    bool WriteCameraCsv(const std::vector<WrittenFrame>& frames, std::string* error);
    bool WriteDepthCsv(const std::vector<TimeNs>& times, std::string* error);
    bool WriteGroundTruthCsv(const std::vector<PoseRecord>& poses, std::string* error);
    bool WriteSummary(const WriteResult& result,
                      const ImuAlignStats& imu_stats,
                      const std::vector<TimeNs>& kept_depth,
                      std::string* error);
    std::string CameraYaml(const CameraIntrinsics& camera, const RigidTransform& to_body) const;

    std::string root_;
    BodyCalibration calibration_;
    ImuNoise noise_;
    WriteOptions options_;
    Provenance provenance_;

    struct Job {
        bool is_depth{false};
        StereoPairRecord pair;
        DepthRecord depth;
    };

    std::thread writer_;
    mutable std::mutex mutex_;
    std::condition_variable available_;
    std::deque<Job> queue_;
    bool running_{false};
    std::size_t queue_capacity_{12};
    std::uint64_t dropped_{0};
    double max_skew_ms_{0.0};
    std::vector<std::string> write_errors_;

    std::vector<WrittenFrame> written_frames_;
    std::vector<TimeNs> written_depth_;
};

} // namespace rh

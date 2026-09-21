#pragma once

#include <functional>
#include <string>
#include <vector>

#include "record_helper/calibration.hpp"
#include "record_helper/types.hpp"

namespace rs2 {
class frame;
}

namespace rh::capture {

struct StreamConfig {
    int width{848};
    int height{480};
    int fps{30};
    bool depth{true};
    bool pose{true};
    bool projector_on{true};
    // >0 时关闭两目自动曝光并固定曝光时间（微秒）：室内自动曝光容易放到 10 ms 以上，
    // 全局快门也会糊，是跟踪丢点的常见根因。
    int exposure_microseconds{0};
    std::string serial;
    int probe_frames{60};
};

struct DeviceSnapshot {
    std::string name;
    std::string serial;
    std::string firmware;
    std::string usb_type;
    std::string sdk_version;
};

struct CalibrationInfo {
    BodyCalibration calibration;
    cal::StereoGeometrySummary geometry;
    double gyro_hz{0.0};
    double accel_hz{0.0};
    double camera_fps{0.0};
    bool pose_available{false};
    std::string distortion_model0;
    std::string distortion_model1;
    std::string accel_frame_note;
};

struct Handlers {
    std::function<void(StereoPairRecord&&)> on_pair;
    std::function<void(DepthRecord&&)> on_depth;
    std::function<void(const GyroSample&)> on_gyro;
    std::function<void(const AccelSample&)> on_accel;
    std::function<void(const PoseRecord&)> on_pose;
};

struct SessionStats {
    std::uint64_t pairs{0};
    std::uint64_t unpaired{0};
    // 逐帧回调队列写满时被丢弃的帧数：调用方长时间不进 Step() 才会发生。
    std::uint64_t dropped{0};
    std::uint64_t gyro{0};
    std::uint64_t accel{0};
    std::uint64_t depth{0};
    std::uint64_t pose{0};
    double max_stereo_skew_ms{0.0};
    // 设备元数据 SENSOR_TIMESTAMP 与 frame.get_timestamp() 的最大偏差，用于确认时间戳域一致。
    double max_timestamp_agreement_ms{0.0};
    double projector_temp_c{0.0};
};

// 只负责「设备 → 内存中的记录」这一段：帧号配对、时间戳归一、外参读取。
// 落盘、门控、状态机都不在这里，方便无设备时单测其余部分。
class Session {
  public:
    Session();
    ~Session();

    Session(const Session&) = delete;
    Session& operator=(const Session&) = delete;

    // 枚举可用设备；serial_filter 为空时返回全部。
    static bool ListDevices(std::vector<DeviceSnapshot>* devices, std::string* error);

    bool Start(const StreamConfig& config,
               Handlers handlers,
               DeviceSnapshot* device,
               CalibrationInfo* info,
               std::string* error);
    // 阻塞取一帧并分派；超时不是错误，返回 true 表示「本次没有异常」。
    bool Step(std::string* error);
    void Stop();

    const SessionStats& stats() const { return stats_; }
    bool running() const { return running_; }

  private:
    struct Impl;
    Impl* impl_{nullptr};
    Handlers handlers_;
    SessionStats stats_;
    bool running_{false};

    // 逐帧回调只入队，派发都在调用方线程：左右目靠时间戳全等配对，所以先到的一方要能挂住等另一半。
    void DispatchFrame(const rs2::frame& frame);
    void HoldForPair(const rs2::frame& frame, TimeNs stamp, bool left);
    void EmitPair(const rs2::frame& left, const rs2::frame& right);
    void EmitDepth(const rs2::frame& depth);
};

// 把设备报告的 3x4 外参转成 RigidTransform，并做正交化修正（p_to = R·p_from + t）。
RigidTransform TransformFromRs2(const float rotation[9], const float translation[3]);

} // namespace rh::capture

#pragma once

#include <cmath>
#include <string>
#include <vector>

#include "record_helper/dataset_writer.hpp"
#include "record_helper/types.hpp"

namespace rh::testing {

// 确定性伪随机：不依赖 <random> 的跨实现分布差异，保证用例可复现。
class Lcg {
  public:
    explicit Lcg(std::uint64_t seed) : state_(seed) {}
    double Uniform(double scale) {
        state_ = state_ * 6364136223846793005ULL + 1442695040888963407ULL;
        const double unit = static_cast<double>(state_ >> 11) / 9007199254740992.0;
        return (unit - 0.5) * 2.0 * scale;
    }

  private:
    std::uint64_t state_;
};

struct SyntheticCapture {
    std::vector<GyroSample> gyro;
    std::vector<AccelSample> accel;
    std::vector<PoseRecord> poses;
    std::vector<StereoPairRecord> pairs;
    std::vector<DepthRecord> depth;
};

// 造一段结构上应当可回放的合成录制：静止 → 温和运动 → 匀速跟踪 → 尾部 IMU 覆盖。
// 采样率刻意与 D435i 一致（gyro 400 Hz、accel 250 Hz、双目 30 fps），
// 这样 writer 的内插与裁窗路径在单测里就是真实路径。
inline SyntheticCapture MakeSynthetic(double still_s,
                                      double motion_s,
                                      double track_s,
                                      double tail_s,
                                      TimeNs origin_ns = 8'640'000'000'000LL,
                                      double pause_begin_s = -1.0,
                                      double pause_end_s = -1.0) {
    SyntheticCapture capture;
    const double total = still_s + motion_s + track_s + tail_s;
    Lcg lcg(0x5eed1234ULL);
    const double gyro_dt = 2'500'000.0;  // 400 Hz
    const double accel_dt = 4'000'000.0; // 250 Hz
    const double g = 9.81;
    auto moving_at = [&](double seconds) {
        if (pause_begin_s >= 0.0 && seconds >= pause_begin_s && seconds < pause_end_s) {
            return false;
        }
        return seconds > still_s;
    };
    for (double seconds = 0.0; seconds < total; seconds += gyro_dt * 1e-9) {
        GyroSample sample;
        sample.t_ns = origin_ns + static_cast<TimeNs>(seconds * 1e9);
        const bool moving = moving_at(seconds);
        const double amplitude = moving ? 0.30 : 0.0;
        sample.w =
            Eigen::Vector3d(amplitude * std::sin(2.0 * M_PI * 0.8 * seconds) + lcg.Uniform(0.002),
                            lcg.Uniform(0.002),
                            moving ? 0.05 : lcg.Uniform(0.002));
        capture.gyro.push_back(sample);
    }
    for (double seconds = 0.0; seconds < total; seconds += accel_dt * 1e-9) {
        AccelSample sample;
        sample.t_ns = origin_ns + static_cast<TimeNs>(seconds * 1e9);
        const bool moving = moving_at(seconds);
        const double shape = moving ? 1.0 : 0.0;
        sample.a = Eigen::Vector3d(
            shape * 0.9 * std::sin(2.0 * M_PI * 1.3 * seconds) + lcg.Uniform(0.004),
            shape * 0.4 * std::sin(2.0 * M_PI * 0.6 * seconds + 1.0) + lcg.Uniform(0.004),
            g + shape * 0.5 * std::sin(2.0 * M_PI * 0.9 * seconds) + lcg.Uniform(0.004));
        capture.accel.push_back(sample);
    }
    const std::uint32_t width = 160;
    const std::uint32_t height = 120;
    const double frame_dt = 1e9 / 30.0;
    for (double seconds = still_s * 0.5; seconds < total - tail_s * 0.5;
         seconds += frame_dt * 1e-9) {
        StereoPairRecord record;
        record.t_ns = origin_ns + static_cast<TimeNs>(seconds * 1e9);
        record.t1_ns = record.t_ns;
        record.frame_counter0 = static_cast<std::uint64_t>(seconds / frame_dt);
        record.frame_counter1 = record.frame_counter0;
        record.left.resize(static_cast<std::size_t>(width) * height);
        record.right.resize(static_cast<std::size_t>(width) * height);
        for (std::uint32_t row = 0; row < height; ++row) {
            for (std::uint32_t column = 0; column < width; ++column) {
                const int value = static_cast<int>(column * 2 + row + seconds * 5.0) & 0xff;
                record.left[static_cast<std::size_t>(row) * width + column] =
                    static_cast<std::uint8_t>(value);
                record.right[static_cast<std::size_t>(row) * width + column] =
                    static_cast<std::uint8_t>((value + 13) & 0xff);
            }
        }
        capture.pairs.push_back(std::move(record));

        DepthRecord depth;
        depth.t_ns = record.t_ns + 1; // 深度与左目共享设备时钟，故意留 1 ns 差以覆盖独立时间线
        depth.width = width;
        depth.height = height;
        depth.millimetres.assign(static_cast<std::size_t>(width) * height, 1234);
        capture.depth.push_back(std::move(depth));

        PoseRecord pose;
        pose.t_ns = record.t_ns;
        pose.p = Eigen::Vector3d(0.25 * seconds, 0.0, 0.0);
        pose.q = Eigen::Quaterniond::Identity();
        pose.confidence = 3.0;
        capture.poses.push_back(pose);
    }
    return capture;
}

// 与 D435i 同量级的合成标定：光轴近似平行、50 mm 水平基线、光学系相对 IMU 系约 90° 置换。
inline BodyCalibration SyntheticCalibration(std::uint32_t width = 160, std::uint32_t height = 120) {
    BodyCalibration calibration;
    calibration.camera0.width = width;
    calibration.camera0.height = height;
    calibration.camera1.width = width;
    calibration.camera1.height = height;
    calibration.camera0.fx = calibration.camera0.fy = 110.0;
    calibration.camera1.fx = calibration.camera1.fy = 110.0;
    calibration.camera0.cx = width * 0.5 - 0.5;
    calibration.camera0.cy = height * 0.5 - 0.5;
    calibration.camera1.cx = width * 0.5 - 0.5;
    calibration.camera1.cy = height * 0.5 - 0.5;
    Eigen::Matrix3d rotation;
    // 光学系 (z 前、x 右、y 下) 到 IMU 系 (x 前、y 左、z 上) 的常用轴向置换。
    rotation << 0.0, 0.0, -1.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0;
    calibration.camera0_to_body.r = rotation;
    calibration.camera0_to_body.t = Eigen::Vector3d(-0.0055, 0.0210, 0.0117);
    calibration.camera1_to_body.r = rotation;
    // 真实 D435 的两目基线在相机 x 方向；先按光学系给定位移，再旋到 body 系。
    calibration.camera1_to_body.t =
        calibration.camera0_to_body.t + rotation * Eigen::Vector3d(-0.0500, 0.0, 0.0);
    calibration.device_serial = "SYNTH001";
    calibration.device_name = "Synthetic D435i";
    calibration.firmware = "test";
    return calibration;
}

inline ImuNoise SyntheticNoise() {
    ImuNoise noise;
    noise.sigma_a = 1.0e-3;
    noise.sigma_g = 4.0e-4;
    noise.sigma_ba = 5.0e-5;
    noise.sigma_bg = 4.0e-6;
    noise.g = 9.81;
    noise.measured = true;
    return noise;
}

inline WriteOptions SyntheticOptions(bool depth = true, bool pose_gt = true) {
    WriteOptions options;
    options.write_depth = depth;
    options.write_pose_gt = pose_gt;
    options.png_level = 1;
    // 合成用例是离线灌数据，不受 30 fps 节拍约束，因此把队列开到足够大，
    // 让「丢弃」只发生在真正会破坏契约的地方（由 overflow 用例单独验证）。
    options.queue_capacity = 4096;
    return options;
}

inline Provenance SyntheticProvenance(const std::string& sequence) {
    Provenance provenance;
    provenance.sequence_name = sequence;
    provenance.device_name = "Synthetic D435i";
    provenance.serial = "SYNTH001";
    provenance.firmware = "test";
    provenance.noise_source = "unit-test";
    return provenance;
}

} // namespace rh::testing

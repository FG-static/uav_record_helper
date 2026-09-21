#include "cli_common.hpp"

#include <cstdio>
#include <string>
#include <vector>

#include "record_helper/capture.hpp"

namespace rh::cli {
namespace {

void PrintUsage() {
    std::printf("用法: rh probe [--serial S] [--width 848] [--height 480] [--fps 30] [--no-depth]\n"
                "          [--pose-gt auto|on|off] [--probe-frames 60] [--projector on|off]\n"
                "          [--exposure-us 4000]\n"
                "\n"
                "  读取设备能力、内参、外参、IMU 速率与时间戳域一致性，不写任何文件。\n"
                "  输出里的「两目几何摘要」用来决定畸变系数该按报告值还是按零写出。\n");
}

const char* DistortionHint(const cal::StereoGeometrySummary& geometry) {
    if (geometry.relative_rotation_deg < 0.05 && geometry.off_axis_baseline_ratio < 0.01) {
        return "两目已平行且基线水平：红外图是硬件修正过的，建议 --zero-distortion";
    }
    return "两目存在非平凡相对旋转：建议保留设备报告的畸变系数（radtan k1,k2,p1,p2）";
}

} // namespace

int RunProbe(int argc, char** argv) {
    ArgumentParser parser(argc, argv, "probe");
    std::string error;
    // serial 可能是字母，pose-gt / projector 是关键字取值，必须声明成带字符串取值的选项。
    if (!parser.Parse({"serial", "pose-gt", "projector"})) {
        std::printf("%s\n", error.c_str());
        return 2;
    }
    std::vector<std::string> known{"serial",
                                   "width",
                                   "height",
                                   "fps",
                                   "no-depth",
                                   "pose-gt",
                                   "probe-frames",
                                   "projector",
                                   "help",
                                   "exposure-us"};
    if (!parser.RejectUnknown(known, &error)) {
        std::printf("%s\n", error.c_str());
        PrintUsage();
        return 2;
    }
    if (parser.Has("help")) {
        PrintUsage();
        return 0;
    }

    std::vector<capture::DeviceSnapshot> devices;
    if (!capture::Session::ListDevices(&devices, &error)) {
        std::printf("枚举设备失败: %s\n", error.c_str());
        return 1;
    }
    std::printf("可见设备 %zu 台:\n", devices.size());
    for (const auto& device : devices) {
        std::printf("  %-32s serial=%-16s fw=%-14s usb=%s sdk=%s\n",
                    device.name.c_str(),
                    device.serial.c_str(),
                    device.firmware.c_str(),
                    device.usb_type.c_str(),
                    device.sdk_version.c_str());
    }
    if (devices.empty()) {
        std::printf("没有可枚举的 RealSense 设备。\n");
        return 1;
    }

    capture::StreamConfig config;
    config.serial = parser.Value("serial");
    config.width = parser.Integer("width", 848, &error);
    config.height = parser.Integer("height", 480, &error);
    config.fps = parser.Integer("fps", 30, &error);
    config.probe_frames = parser.Integer("probe-frames", 60, &error);
    config.depth = !parser.Has("no-depth");
    const std::string pose_mode = parser.Value("pose-gt", "auto");
    config.pose = pose_mode != "off";
    config.projector_on = parser.Value("projector", "on") != "off";
    config.exposure_microseconds = parser.Integer("exposure-us", 0, &error);
    if (!error.empty()) {
        std::printf("%s\n", error.c_str());
        return 2;
    }

    capture::Session session;
    capture::DeviceSnapshot device;
    capture::CalibrationInfo info;
    std::vector<GyroSample> gyro;
    std::vector<AccelSample> accel;
    capture::Handlers handlers;
    handlers.on_gyro = [&](const GyroSample& sample) { gyro.push_back(sample); };
    handlers.on_accel = [&](const AccelSample& sample) { accel.push_back(sample); };
    // probe 只读能力，不落盘：on_pair 留空即只统计配对情况。
    handlers.on_pair = [](StereoPairRecord&& record) { static_cast<void>(record); };
    if (!session.Start(config, handlers, &device, &info, &error)) {
        std::printf("启动采集失败: %s\n", error.c_str());
        if (!info.calibration.device_serial.empty()) {
            std::printf("（外参已读到，仅流配置失败）\n");
        }
        return 1;
    }
    const double started = MonotonicSeconds();
    while (session.stats().pairs < static_cast<std::uint64_t>(config.probe_frames) &&
           MonotonicSeconds() - started < 8.0) {
        if (!session.Step(&error)) {
            std::printf("取帧失败: %s\n", error.c_str());
            break;
        }
    }
    const double sampled_seconds = MonotonicSeconds() - started;
    session.Stop();

    const auto& stats = session.stats();
    std::printf("\n== 实际启用的流 ==\n");
    std::printf("  红外/深度 %ux%u @ %.0f fps，gyro %.0f Hz，accel %.0f Hz，pose %s\n",
                info.calibration.camera0.width,
                info.calibration.camera0.height,
                info.camera_fps,
                info.gyro_hz,
                info.accel_hz,
                info.pose_available ? "可用" : "不可用");
    std::printf("  %s\n", info.accel_frame_note.c_str());
    if (!info.pose_available && pose_mode == "on") {
        std::printf("  警告: 请求了伪 GT，但设备的 pose 流不可用\n");
    }

    std::printf("\n== 内参（对应交付给主机的像素）==\n");
    for (int index = 0; index < 2; ++index) {
        const CameraIntrinsics& camera =
            index == 0 ? info.calibration.camera0 : info.calibration.camera1;
        std::printf("  cam%d %s: fx=%.3f fy=%.3f cx=%.3f cy=%.3f\n",
                    index,
                    camera.frame_id.c_str(),
                    camera.fx,
                    camera.fy,
                    camera.cx,
                    camera.cy);
        std::printf("       d=[%.6g, %.6g, %.6g, %.6g] k3=%.6g  model=%s\n",
                    camera.distortion[0],
                    camera.distortion[1],
                    camera.distortion[2],
                    camera.distortion[3],
                    camera.reported_k3,
                    index == 0 ? info.distortion_model0.c_str() : info.distortion_model1.c_str());
    }

    std::printf("\n== 两目几何摘要 ==\n");
    std::printf("  基线 %.2f mm，光轴相对角 %.4f°，非轴向基线比例 %.5f\n",
                info.geometry.baseline_m * 1000.0,
                info.geometry.relative_rotation_deg,
                info.geometry.off_axis_baseline_ratio);
    std::printf("  建议: %s\n", DistortionHint(info.geometry));

    std::printf("\n== 外参 T_BS（相机光学系 → IMU/加表系，写进 sensor.yaml 的行主序 4x4）==\n");
    for (int index = 0; index < 2; ++index) {
        const RigidTransform& transform =
            index == 0 ? info.calibration.camera0_to_body : info.calibration.camera1_to_body;
        std::printf(
            "  cam%d R=[%.6f %.6f %.6f; %.6f %.6f %.6f; %.6f %.6f %.6f] t=[%.5f %.5f %.5f]\n",
            index,
            transform.r(0, 0),
            transform.r(0, 1),
            transform.r(0, 2),
            transform.r(1, 0),
            transform.r(1, 1),
            transform.r(1, 2),
            transform.r(2, 0),
            transform.r(2, 1),
            transform.r(2, 2),
            transform.t.x(),
            transform.t.y(),
            transform.t.z());
    }

    std::printf("\n== 时间戳与同步 ==\n");
    std::printf("  pairs=%llu 未配对=%llu 队列丢弃=%llu 左右偏斜最大=%.4f ms\n",
                static_cast<unsigned long long>(stats.pairs),
                static_cast<unsigned long long>(stats.unpaired),
                static_cast<unsigned long long>(stats.dropped),
                stats.max_stereo_skew_ms);
    std::printf("  SENSOR_TIMESTAMP 元数据与 get_timestamp() 最大偏差=%.6f ms\n",
                stats.max_timestamp_agreement_ms);
    if (stats.pairs == 0) {
        std::printf("  没有采到成对红外帧：设备可能被占用，或所选分辨率下左右流未同时起流\n");
    }

    if (accel.size() > 4) {
        Eigen::Vector3d sum = Eigen::Vector3d::Zero();
        for (const auto& sample : accel) {
            sum += sample.a;
        }
        std::printf("  采样期间 ‖accel‖ 均值=%.4f m/s²（设备静止时应接近 9.81，偏差 >0.5 "
                    "会让静止初始化失败）\n",
                    sum.norm() / static_cast<double>(accel.size()));
    }
    // IMU 必须按标称速率到达。取帧接口用错（例如按 frameset 取，一个 frameset 每流最多一帧）
    // 会把它抽稀到相机帧率：imu0 栅格间隔立刻超上游的 10 ms 上限，而其余检查全都照过。
    const double gyro_hz =
        sampled_seconds > 0.0 ? static_cast<double>(stats.gyro) / sampled_seconds : 0.0;
    const double accel_hz =
        sampled_seconds > 0.0 ? static_cast<double>(stats.accel) / sampled_seconds : 0.0;
    std::printf("  实测采样率 gyro=%.0f Hz accel=%.0f Hz（请求 %.0f/%.0f Hz，样本 %llu/%llu，窗口 %.2f s）\n",
                gyro_hz,
                accel_hz,
                info.gyro_hz > 0.0 ? info.gyro_hz : 1.0,
                info.accel_hz > 0.0 ? info.accel_hz : 1.0,
                static_cast<unsigned long long>(stats.gyro),
                static_cast<unsigned long long>(stats.accel),
                sampled_seconds);
    int status = 0;
    const auto require_rate = [&](const char* name, double measured, double requested) {
        if (requested <= 0.0 || measured >= 0.8 * requested) {
            return;
        }
        std::printf("  [FAIL] %s 实测只有请求值的 %.0f%%：IMU 被抽稀，栅格间隔会超 10 ms 上限\n",
                    name,
                    100.0 * measured / requested);
        status = 1;
    };
    require_rate("gyro", gyro_hz, info.gyro_hz);
    require_rate("accel", accel_hz, info.accel_hz);
    return status;
}

} // namespace rh::cli

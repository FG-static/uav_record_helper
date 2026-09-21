#include <sys/stat.h>

#include <cctype>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "cli_common.hpp"
#include "record_helper/capture.hpp"
#include "record_helper/dataset_writer.hpp"
#include "record_helper/euroc_format.hpp"
#include "record_helper/verify.hpp"

namespace rh::cli {
namespace {

enum class Phase { Preflight, Still, Excite, Track, Tail, Finalizing, Aborted };

// 静止标定阶段允许设备摆得不那么水平；容差要宽到不误伤手摆，又窄到能抓住 90°/180° 的口径错。
constexpr double kGravityDirectionToleranceDeg = 25.0;

// 按「记录系 = 光学系」（x 右、y 下、z 朝镜头外）把静止段实测的世界上翻译成摆法，
// 好让中止信息说得出「你现在是怎么放的」，而不只是丢一个夹角。
const char* StillPoseHint(const Eigen::Vector3d& up) {
    if (up.z() > 0.8) {
        return "镜头竖直朝上（就是本阶段要求的摆法）";
    }
    if (up.z() < -0.8) {
        return "镜头朝下扣在桌上";
    }
    if (up.y() < -0.8) {
        return "镜头水平朝前、画面正立";
    }
    if (up.y() > 0.8) {
        return "镜头水平朝前、画面倒立";
    }
    if (up.x() < -0.8) {
        return "侧躺、画面里左边朝上";
    }
    if (up.x() > 0.8) {
        return "侧躺、画面里右边朝上";
    }
    return "斜放，没对准任何一根轴";
}

const char* PhaseName(Phase phase) {
    switch (phase) {
    case Phase::Preflight:
        return "预热";
    case Phase::Still:
        return "静止标定";
    case Phase::Excite:
        return "温和激励";
    case Phase::Track:
        return "正式录制";
    case Phase::Tail:
        return "尾部覆盖";
    case Phase::Finalizing:
        return "收尾写盘";
    case Phase::Aborted:
        return "已放弃";
    }
    return "?";
}

bool DirectoryExists(const std::string& path) {
    struct stat info{};
    return ::stat(path.c_str(), &info) == 0 && S_ISDIR(info.st_mode);
}

bool LooksLikeExistingDataset(const std::string& root) {
    return DirectoryExists(fmt::JoinPath(root, "mav0"));
}

std::string DefaultSequenceName() {
    char buffer[64];
    const std::time_t now = std::time(nullptr);
    std::tm broken{};
    localtime_r(&now, &broken);
    std::strftime(buffer, sizeof(buffer), "d435i_%Y%m%d_%H%M%S", &broken);
    return buffer;
}

bool ValidSequenceName(const std::string& name) {
    if (name.empty() || name.size() > 64) {
        return false;
    }
    for (const char character : name) {
        const bool ok = std::isalnum(static_cast<unsigned char>(character)) != 0 ||
                        character == '_' || character == '-' || character == '.';
        if (!ok || character == '/') {
            return false;
        }
    }
    return name != "." && name != "..";
}

void PrintRecordUsage() {
    std::printf(
        "用法: rh record [--out DIR] [--seq NAME] [--still 4] [--excite 3]\n"
        "                [--width 848] [--height 480] [--fps 30] [--serial S]\n"
        "                [--pose-gt auto|on|off] [--no-pose-gt] [--no-depth] [--zero-distortion]\n"
        "                [--projector on|off] [--exposure-us 3000] [--png-level 3] [--tail 0.2]\n"
        "                [--force] [--no-verify] [--duration SEC]\n"
        "\n"
        "  预热后自动等静止标定（--still）和温和激励（--excite）满足，然后进入正式录制。\n"
        "  正式录制不限时长、不看运动/静止：中途停住也不会改阶段。\n"
        "  只有手动结束才进尾部 IMU 覆盖：Enter 或 q 或 Ctrl-C 结束并出盘，x 放弃。\n"
        "  --still/--excite 是初始化前提，不能跳过。\n"
        "  --duration 只作状态行建议时长（unav_vio 要 IMU >30000 行，约 80 s），到点不会自动停。\n");
}

} // namespace

int RunRecord(int argc, char** argv) {
    InstallSignalHandlers();
    ArgumentParser parser(argc, argv, "record");
    std::string error;
    if (!parser.Parse()) {
        std::printf("%s\n", error.c_str());
        return 2;
    }
    if (parser.Has("help")) {
        PrintRecordUsage();
        return 0;
    }
    const std::vector<std::string> known{"out",
                                         "seq",
                                         "duration",
                                         "still",
                                         "excite",
                                         "width",
                                         "height",
                                         "fps",
                                         "serial",
                                         "pose-gt",
                                         "no-pose-gt",
                                         "no-depth",
                                         "zero-distortion",
                                         "projector",
                                         "png-level",
                                         "tail",
                                         "force",
                                         "no-verify",
                                         "exposure-us"};
    if (!parser.RejectUnknown(known, &error)) {
        std::printf("%s\n", error.c_str());
        PrintRecordUsage();
        return 2;
    }

    double duration = parser.Number("duration", 0.0, &error);
    double still_seconds = parser.Number("still", 4.0, &error);
    double excite_seconds = parser.Number("excite", 3.0, &error);
    double tail_seconds = parser.Number("tail", 0.2, &error);
    const int width = parser.Integer("width", 848, &error);
    const int height = parser.Integer("height", 480, &error);
    const int fps = parser.Integer("fps", 30, &error);
    const int png_level = parser.Integer("png-level", 3, &error);
    if (!error.empty()) {
        std::printf("%s\n", error.c_str());
        return 2;
    }
    if (duration < 0.0 || still_seconds < 1.5 || excite_seconds < 2.5 || tail_seconds < 0.1) {
        std::printf("--duration 不能为负；--still 至少 1.5 s（3 个 0.5 s 窗）；"
                    "--excite 至少 2.5 s（5 个 0.5 s 窗）；--tail 至少 0.1 s\n");
        return 2;
    }

    std::string out = parser.Value("out", ".");
    std::string seq = parser.Value("seq", DefaultSequenceName());
    if (!ValidSequenceName(seq)) {
        std::printf("非法的 --seq: %s（只允许字母数字、_、-、.）\n", seq.c_str());
        return 2;
    }
    const std::string root = fmt::JoinPath(out, seq);
    if (LooksLikeExistingDataset(root) && !parser.Has("force")) {
        std::printf("目标已存在数据集，拒绝覆盖: %s（要覆盖请加 --force）\n", root.c_str());
        return 1;
    }

    capture::StreamConfig stream;
    stream.width = width;
    stream.height = height;
    stream.fps = fps;
    stream.serial = parser.Value("serial");
    stream.depth = !parser.Has("no-depth");
    stream.pose = parser.Value("pose-gt", "auto") != "off" && !parser.Has("no-pose-gt");
    stream.projector_on = parser.Value("projector", "on") != "off";
    stream.exposure_microseconds = parser.Integer("exposure-us", 0, &error);

    WriteOptions options;
    options.write_depth = stream.depth;
    options.write_pose_gt = stream.pose;
    options.zero_distortion = parser.Has("zero-distortion");
    options.png_level = png_level;
    options.imu_tail_guard_ns = static_cast<TimeNs>(tail_seconds * 1e9);

    capture::Session session;
    capture::DeviceSnapshot device;
    capture::CalibrationInfo info;
    std::vector<GyroSample> gyro;
    std::vector<AccelSample> accel;
    std::vector<PoseRecord> poses;
    const double reserve_s = duration > 1.0 ? duration : 120.0;
    gyro.reserve(static_cast<std::size_t>(reserve_s * 600));
    accel.reserve(static_cast<std::size_t>(reserve_s * 400));

    Phase phase = Phase::Preflight;
    std::unique_ptr<DatasetWriter> writer;
    ImuNoise noise;
    NoiseEstimate noise_estimate;
    // 静止段实测重力方向与 T_BS 第三列的夹角；<0 表示没测成。写进 record_summary.yaml 留痕。
    double static_gravity_angle_deg = -1.0;
    TimeNs latest_imu_ns = 0;
    TimeNs gate_last_ns = 0;

    ProbeConfig probe;
    LiveGate gate(probe);
    const std::size_t still_windows =
        static_cast<std::size_t>(still_seconds / probe.window_s + 0.5);
    const std::size_t motion_windows =
        static_cast<std::size_t>(excite_seconds / probe.window_s + 0.5);

    auto open_writer = [&](std::string* failure) -> bool {
        Provenance provenance;
        provenance.sequence_name = seq;
        provenance.device_name = device.name;
        provenance.serial = device.serial;
        provenance.firmware = device.firmware;
        provenance.librealsense_version = device.sdk_version;
        provenance.usb_type = device.usb_type;
        provenance.started_unix_seconds = static_cast<double>(std::time(nullptr));
        provenance.noise_source = noise_estimate.note;
        provenance.distortion_model0 = info.distortion_model0;
        provenance.distortion_model1 = info.distortion_model1;
        provenance.camera_fps = static_cast<int>(info.camera_fps);
        provenance.imu_gyro_hz = static_cast<int>(info.gyro_hz);
        provenance.capture_note =
            "still=" + std::to_string(static_cast<int>(still_seconds * 10) / 10.0) +
            "s excite=" + std::to_string(static_cast<int>(excite_seconds * 10) / 10.0) + "s";
        provenance.static_gravity_angle_deg = static_gravity_angle_deg;
        auto created =
            std::make_unique<DatasetWriter>(root, info.calibration, noise, options, provenance);
        if (!created->Start(failure)) {
            return false;
        }
        writer = std::move(created);
        return true;
    };

    capture::Handlers handlers;
    handlers.on_accel = [&](const AccelSample& sample) {
        accel.push_back(sample);
        latest_imu_ns = sample.t_ns;
    };
    handlers.on_gyro = [&](const GyroSample& sample) { gyro.push_back(sample); };
    handlers.on_pose = [&](const PoseRecord& record) { poses.push_back(record); };
    handlers.on_pair = [&](StereoPairRecord&& record) {
        if (writer && phase != Phase::Preflight && phase != Phase::Aborted) {
            writer->Enqueue(std::move(record));
        }
    };

    if (!session.Start(stream, handlers, &device, &info, &error)) {
        std::printf("启动 D435i 失败: %s\n", error.c_str());
        return 1;
    }
    std::printf("设备 %s / 序列号 %s / 固件 %s / USB %s\n",
                device.name.c_str(),
                device.serial.c_str(),
                device.firmware.c_str(),
                device.usb_type.c_str());
    std::printf("流: 红外 %ux%u@%.0fHz, gyro %.0fHz, accel %.0fHz, depth %s, 位姿流 %s\n",
                info.calibration.camera0.width,
                info.calibration.camera0.height,
                info.camera_fps,
                info.gyro_hz,
                info.accel_hz,
                stream.depth ? "开" : "关",
                info.pose_available ? "可用" : "不可用");
    const auto geometry = info.geometry;
    std::printf("标定: 基线 %.1f mm，两目光轴相对角 %.4f°，畸变 model=%s/%s%s\n",
                geometry.baseline_m * 1000.0,
                geometry.relative_rotation_deg,
                info.distortion_model0.c_str(),
                info.distortion_model1.c_str(),
                options.zero_distortion ? "（写出零畸变）" : "");
    // 预热阶段噪声还没实测，用一组占位正值只校验内参/外参；真实值在静止段结束时写入。
    ImuNoise placeholder_noise;
    placeholder_noise.sigma_a = placeholder_noise.sigma_g = 1e-3;
    placeholder_noise.sigma_ba = placeholder_noise.sigma_bg = 1e-5;
    placeholder_noise.g = 9.81;
    placeholder_noise.measured = true;
    auto validation = cal::ValidateCalibration(info.calibration, placeholder_noise, 9.81);
    for (const auto& problem : validation.problems) {
        std::printf("  标定问题: %s\n", problem.c_str());
    }
    if (!validation.ok()) {
        std::printf("设备标定不可用，先修外参/内参再录。\n");
        session.Stop();
        return 1;
    }
    if (stream.pose && !info.pose_available) {
        std::printf("  提示: 设备没有位姿流，本次不会写伪 GT；"
                    "回放测试目前仍要求 GT 文件，验证时用 --no-verify 或改测试断言。\n");
        options.write_pose_gt = false;
    }

    KeyReader keys;
    const double started = MonotonicSeconds();
    double phase_started = started;
    double last_status = 0.0;
    char key = 0;
    bool stop_requested = false;

    std::printf("\n阶段 %s：等待出流…\n", PhaseName(phase));
    while (phase != Phase::Aborted) {
        if (interrupted != 0) {
            interrupted = 0;
            if (phase == Phase::Track || phase == Phase::Tail) {
                std::printf("\n收到中断信号，结束当前阶段\n");
                stop_requested = true;
            } else {
                std::printf("\n收到中断信号：静止标定或激励尚未完成，放弃本次录制\n");
                phase = Phase::Aborted;
            }
        }
        while (keys.Poll(&key)) {
            if (key == 'x' || key == 'X') {
                phase = Phase::Aborted;
                break;
            }
            if (key == 'q' || key == 'Q') {
                if (phase == Phase::Track || phase == Phase::Tail) {
                    stop_requested = true;
                } else {
                    phase = Phase::Aborted;
                }
                break;
            }
            if (key == '\n' || key == '\r') {
                // Enter：激励可提前结束；正式录制里这是唯一的「下一阶段」（收尾）。
                // 静止标定不能跳。正式录制中途的运动/静止不读键以外的任何门控。
                if (phase == Phase::Excite) {
                    std::printf("  跳过剩余激励等待\n");
                    phase = Phase::Track;
                    phase_started = MonotonicSeconds();
                    std::printf("\n阶段 %s：不限时长。停住也不会改阶段。按 Enter / q / Ctrl-C 结束\n",
                                PhaseName(phase));
                } else if (phase == Phase::Track || phase == Phase::Tail) {
                    std::printf("  手动结束录制\n");
                    stop_requested = true;
                }
                break;
            }
        }
        if (phase == Phase::Aborted) {
            break;
        }
        if (phase == Phase::Finalizing) {
            break;
        }
        if (!session.Step(&error)) {
            std::printf("取帧失败: %s\n", error.c_str());
            phase = Phase::Aborted;
            break;
        }
        // 用加表节拍喂门控（250 Hz > 每窗 4 样本下限），陀螺值取最近一条。
        if (!accel.empty() && !gyro.empty()) {
            const AccelSample& sample = accel.back();
            if (sample.t_ns != gate_last_ns) {
                gate.Push(sample.t_ns, sample.a, gyro.back().w);
                gate_last_ns = sample.t_ns;
            }
        }
        const LiveGateStatus gate_status = gate.Assess(still_windows, motion_windows);

        if (phase == Phase::Preflight) {
            if (session.stats().pairs > 3 && gyro.size() > 20 && accel.size() > 10) {
                phase = Phase::Still;
                phase_started = MonotonicSeconds();
                std::printf("\n阶段 %s：请把设备「镜头竖直朝上」平放桌上、完全不动约 %.0f s"
                            "（当前 accel_rms=%.3f gyro_rms=%.3f）\n",
                            PhaseName(phase),
                            still_seconds,
                            gate_status.last_accel_rms,
                            gate_status.last_gyro_rms);
            }
        } else if (phase == Phase::Still) {
            if (gate_status.static_run_met) {
                const TimeNs begin_ns = latest_imu_ns - static_cast<TimeNs>(still_seconds * 1e9);
                noise_estimate = EstimateImuNoise(gyro, accel, begin_ns, latest_imu_ns);
                noise = noise_estimate.noise;
                Eigen::Vector3d static_sum = Eigen::Vector3d::Zero();
                std::size_t static_samples = 0;
                for (const auto& sample : accel) {
                    if (sample.t_ns >= begin_ns && sample.t_ns <= latest_imu_ns) {
                        static_sum += sample.a;
                        ++static_samples;
                    }
                }
                const double mean_norm =
                    static_samples > 0 ? (static_sum / static_cast<double>(static_samples)).norm()
                                       : 0.0;
                std::printf("\n阶段 %s：静止段完成（%.1f s，‖a‖≈%.3f m/s²）\n",
                            PhaseName(phase),
                            still_seconds,
                            mean_norm);
                std::printf("  噪声%s: sigma_a=%.3e sigma_g=%.3e sigma_ba=%.3e sigma_bg=%.3e\n",
                            noise.measured ? " 实测" : " 用回退默认（静止段太短）",
                            noise.sigma_a,
                            noise.sigma_g,
                            noise.sigma_ba,
                            noise.sigma_bg);
                const auto gravity = cal::CheckGravity(mean_norm, 9.81, 0.5);
                if (!gravity.pass) {
                    std::printf(
                        "  警告: 静止 ‖a‖ 与 9.81 偏差 %.3f m/s²，零速假设可能被初始化拒绝\n",
                        gravity.error);
                }
                // 本阶段设备是平放、镜头朝上的，所以世界上 = 光学 +z：静止段平均比力必须沿
                // T_BS 第三列。模长好看而方向不对，就是 IMU 数据系与外参表口径不一致。
                Eigen::Vector3d static_mean = Eigen::Vector3d::Zero();
                if (static_samples > 0) {
                    static_mean = static_sum / static_cast<double>(static_samples);
                }
                const auto direction = cal::CheckGravityDirection(
                    static_mean, info.calibration.camera0_to_body.r, kGravityDirectionToleranceDeg);
                static_gravity_angle_deg = direction.measured_valid ? direction.angle_deg : -1.0;
                if (!direction.measured_valid) {
                    std::printf("  警告: 静止段样本不足，没能核对重力方向与 T_BS 第三列\n");
                } else {
                    const Eigen::Vector3d up = direction.measured.normalized();
                    std::printf("  重力方向核对: 静止均值 (%+.3f, %+.3f, %+.3f)，世界上 = 记录系 "
                                "(%+.2f, %+.2f, %+.2f)，与 T_BS 第三列 (%+.3f, %+.3f, %+.3f) "
                                "夹角 %.1f°（容差 %.0f°）\n",
                                static_mean.x(),
                                static_mean.y(),
                                static_mean.z(),
                                up.x(),
                                up.y(),
                                up.z(),
                                direction.expected.x(),
                                direction.expected.y(),
                                direction.expected.z(),
                                direction.angle_deg,
                                kGravityDirectionToleranceDeg);
                    if (!direction.pass) {
                        std::printf(
                            "  中止: 本阶段要求「镜头竖直朝上」，此时世界上应是 (0, 0, +1)；"
                            "实测是 (%+.2f, %+.2f, %+.2f) → %s。\n"
                            "        按提示重摆再跑即可（还没写盘，不占空间）。\n"
                            "        若确认已镜头朝上而夹角仍在 90°/180° 附近，那才是加表数据系与\n"
                            "        外参表口径不一致——换过 SDK 构建、后端或内核驱动会这样；此时照抄\n"
                            "        设备外参写出的 T_BS 是错的，要先修口径再录。\n",
                            up.x(),
                            up.y(),
                            up.z(),
                            StillPoseHint(up));
                        phase = Phase::Aborted;
                        break;
                    }
                }
                if (!open_writer(&error)) {
                    std::printf("无法写出到 %s: %s\n", root.c_str(), error.c_str());
                    phase = Phase::Aborted;
                    break;
                }
                phase = Phase::Excite;
                phase_started = MonotonicSeconds();
                std::printf("\n阶段 %s：%s\n", PhaseName(phase), gate_status.hint.c_str());
            }
        } else if (phase == Phase::Excite) {
            if (gate_status.motion_run_met) {
                phase = Phase::Track;
                phase_started = MonotonicSeconds();
                std::printf("\n阶段 %s：激励满足，开始正式录制（不限时长）。"
                            "中途停下不会改阶段。按 Enter / q / Ctrl-C 结束并进入收尾\n",
                            PhaseName(phase));
                if (duration > 0.0) {
                    std::printf("  建议至少 %.0f s（IMU 行数过 30000 大约要 80 s）\n", duration);
                }
            }
        } else if (phase == Phase::Track) {
            if (stop_requested) {
                stop_requested = false;
                phase = Phase::Tail;
                phase_started = MonotonicSeconds();
                std::printf("\n阶段 %s：再保持 %.0f s，让 IMU 覆盖最后一个曝光"
                            "（再按 Enter / q / Ctrl-C 可立刻出盘）\n",
                            PhaseName(phase),
                            tail_seconds + 0.4);
            }
        } else if (phase == Phase::Tail) {
            if (stop_requested || MonotonicSeconds() - phase_started >= tail_seconds + 0.4) {
                phase = Phase::Finalizing;
                break;
            }
        }

        if (MonotonicSeconds() - last_status > 0.5) {
            last_status = MonotonicSeconds();
            std::printf("\r[%-5s] 双目=%llu 未配对=%llu IMU=%llu/%llu 位姿=%zu "
                        "accel_rms=%.3f gyro_rms=%.3f 已录=%.0fs%s        ",
                        PhaseName(phase),
                        static_cast<unsigned long long>(session.stats().pairs),
                        static_cast<unsigned long long>(session.stats().unpaired),
                        static_cast<unsigned long long>(gyro.size()),
                        static_cast<unsigned long long>(accel.size()),
                        poses.size(),
                        gate_status.last_accel_rms,
                        gate_status.last_gyro_rms,
                        phase == Phase::Track ? MonotonicSeconds() - phase_started : 0.0,
                        phase == Phase::Track ? "  Enter/q结束" : "");
            std::fflush(stdout);
        }
    }
    std::printf("\n");
    session.Stop();

    if (phase == Phase::Aborted) {
        if (writer) {
            writer->Stop();
        }
        const std::string detail = writer ? ("图像已部分写入 " + root + "，需要时可手动删除")
                                          : std::string("尚未开始写盘");
        std::printf("已放弃本次录制：%s\n", detail.c_str());
        return 130;
    }
    if (!writer) {
        std::printf("没有进入写盘阶段：静止段从未满足，无法标定噪声。\n");
        return 1;
    }
    writer->Stop();
    WriteResult result;
    if (!writer->Finalize(gyro, accel, poses, &result, &error)) {
        std::printf("出盘失败: %s\n", error.c_str());
        return 1;
    }
    std::printf("写盘完成: %s/mav0\n", root.c_str());
    std::printf("  双目 %zu 对（窗口外裁掉 %zu 对，队列满丢 %zu 对），IMU %zu 行，深度 %zu 帧，"
                "伪 GT %zu 行\n",
                result.pairs,
                result.trimmed_pairs,
                result.dropped_queue_full,
                result.imu_rows,
                result.depth_frames,
                result.gt_rows);
    std::printf("  时长 %.1f s，左右曝光最大偏斜 %.4f ms\n",
                (result.last_pair_ns - result.first_pair_ns) * 1e-9,
                result.max_stereo_skew_ms);
    for (const auto& warning : result.warnings) {
        std::printf("  警告: %s\n", warning.c_str());
    }

    if (parser.Has("no-verify")) {
        return 0;
    }
    VerifyOptions verify_options;
    verify_options.require_ground_truth = options.write_pose_gt;
    verify_options.limits.max_imu_gap_ms = options.limits.max_imu_gap_ms;
    verify_options.limits.anchor_offset_s = options.limits.anchor_offset_s;
    verify_options.limits.imu_lead_ns = options.limits.imu_lead_ns;
    const auto report = VerifyDataset(root, verify_options);
    std::printf("\n== 回放就绪校验 ==\n");
    PrintReport(report, stdout);
    if (report.ok()) {
        std::printf("\n下一步（在 unav_vio 构建目录里）:\n  env UNAV_VIO_EUROC_MH01=%s "
                    "UNAV_VIO_MH01_MAX_FRAMES=300 ctest --test-dir <build> -R mh01_replay "
                    "--output-on-failure\n",
                    root.c_str());
    }
    return report.ok() ? 0 : 1;
}

} // namespace rh::cli

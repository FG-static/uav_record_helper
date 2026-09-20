#include "record_helper/capture.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>

#include <librealsense2/rs.hpp>

namespace rh::capture {
namespace {

TimeNs FrameNs(const rs2::frame& frame) {
    // get_timestamp() 统一是毫秒（设备单调时钟域），乘 1e6 归到纳秒。
    return static_cast<TimeNs>(std::llround(frame.get_timestamp() * 1e6));
}

double MetadataNs(const rs2::frame& frame, bool* available) {
    *available = false;
    if (!frame.supports_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP)) {
        return 0.0;
    }
    const rs2_metadata_type value = frame.get_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP);
    *available = true;
    return static_cast<double>(value) * 1000.0; // 元数据单位是 usec
}

std::string SdkVersion() {
    return std::to_string(RS2_API_MAJOR_VERSION) + "." + std::to_string(RS2_API_MINOR_VERSION) +
           "." + std::to_string(RS2_API_PATCH_VERSION);
}

const char* StreamName(rs2_stream stream) {
    switch (stream) {
    case RS2_STREAM_DEPTH:
        return "depth";
    case RS2_STREAM_INFRARED:
        return "infrared";
    case RS2_STREAM_GYRO:
        return "gyro";
    case RS2_STREAM_ACCEL:
        return "accel";
    case RS2_STREAM_POSE:
        return "pose";
    case RS2_STREAM_MOTION:
        return "motion";
    default:
        return "other";
    }
}

// 设备级没有一次性列出全部 profile 的接口，按 sensor 聚合即可覆盖红外/深度/IMU/位姿。
std::vector<rs2::stream_profile> DeviceProfiles(rs2::device& device, std::string* notes) {
    std::vector<rs2::stream_profile> profiles;
    for (const auto& sensor : device.query_sensors()) {
        std::string name = "unnamed-sensor";
        if (sensor.supports(RS2_CAMERA_INFO_NAME)) {
            name = sensor.get_info(RS2_CAMERA_INFO_NAME);
        }
        try {
            const auto stream_profiles = sensor.get_stream_profiles();
            profiles.insert(profiles.end(), stream_profiles.begin(), stream_profiles.end());
            if (notes != nullptr) {
                *notes += "  " + name + ": " + std::to_string(stream_profiles.size()) + " profiles";
                std::vector<std::string> unique;
                for (const auto& profile : stream_profiles) {
                    std::string item = std::string(StreamName(profile.stream_type())) +
                                       "#" + std::to_string(profile.stream_index()) + " " +
                                       rs2_format_to_string(profile.format()) + "@" +
                                       std::to_string(profile.fps());
                    if (std::find(unique.begin(), unique.end(), item) == unique.end()) {
                        unique.push_back(item);
                    }
                }
                for (const auto& item : unique) {
                    *notes += " | " + item;
                }
                *notes += "\n";
            }
        } catch (const rs2::error& e) {
            if (notes != nullptr) {
                *notes += "  " + name + " 读 profile 失败: " + e.what() + "\n";
            }
        }
    }
    return profiles;
}

std::string DescribeError(const rs2::error& error) {
    std::string message = error.what();
    if (message.find("power state") != std::string::npos ||
        message.find("ACCESS") != std::string::npos ||
        message.find("Permission") != std::string::npos) {
        message +=
            "\n  这已经不是「摄像头」开关的问题。macOS 把 D435i 当成普通 UVC 摄像头，"
            "\n  系统相机驱动占着 USB 接口 0，librealsense/libusb 因此 RS2_USB_STATUS_ACCESS。"
            "\n  点允许只授权了 AVFoundation，夺不走这个接口。按顺序试："
            "\n  1. 退出飞书/Lark、Zoom、Photo Booth、浏览器里所有用摄像头的标签"
            "\n  2. 拔掉相机，直插 Mac 雷电/USB-C（不要扩展坞），再插上"
            "\n  3. 在「终端.app」里用管理员权限跑（Intel 从 macOS 12 起的官方做法）："
            "\n       sudo ./build-release/rh probe"
            "\n       sudo ./build-release/rh record --out ~/datasets --seq room_03 --duration 80";
    }
    return message;
}

struct PickResult {
    rs2::stream_profile profile;
    bool found{false};
    std::string description;
};

// 优先精确匹配分辨率/帧率；失败时退回同流同索引的最近候选，并把实际选择写进摘要。
PickResult PickProfile(const std::vector<rs2::stream_profile>& profiles,
                       rs2_stream stream,
                       int index,
                       int width,
                       int height,
                       rs2_format format,
                       int fps) {
    PickResult result;
    double best_score = std::numeric_limits<double>::max();
    for (const auto& raw : profiles) {
        if (raw.stream_type() != stream || (index >= 0 && raw.stream_index() != index)) {
            continue;
        }
        if (format != RS2_FORMAT_ANY && raw.format() != format) {
            continue;
        }
        double score = 0.0;
        const bool video_stream = stream == RS2_STREAM_DEPTH || stream == RS2_STREAM_INFRARED ||
                                  stream == RS2_STREAM_COLOR;
        if (video_stream) {
            auto video = raw.as<rs2::video_stream_profile>();
            score = std::abs(static_cast<double>(video.width() - width)) +
                    std::abs(static_cast<double>(video.height() - height));
        }
        score += std::abs(static_cast<double>(raw.fps() - fps)) * 0.5;
        if (raw.fps() > 0 && fps > 0 && raw.fps() == fps) {
            score *= 0.5; // 帧率精确命中优先
        }
        if (score < best_score) {
            best_score = score;
            result.profile = raw;
            result.found = true;
            std::string text = StreamName(stream);
            text += " index=" + std::to_string(raw.stream_index());
            if (stream == RS2_STREAM_DEPTH || stream == RS2_STREAM_INFRARED ||
                stream == RS2_STREAM_COLOR) {
                auto video = raw.as<rs2::video_stream_profile>();
                text += " " + std::to_string(video.width()) + "x" + std::to_string(video.height());
            }
            text += " fmt=" + std::string(rs2_format_to_string(raw.format())) +
                    " hz=" + std::to_string(raw.fps());
            result.description = text;
        }
    }
    return result;
}

CameraIntrinsics MapIntrinsics(const rs2_intrinsics& raw, std::string* model) {
    CameraIntrinsics camera;
    camera.width = static_cast<std::uint32_t>(raw.width);
    camera.height = static_cast<std::uint32_t>(raw.height);
    camera.fx = raw.fx;
    camera.fy = raw.fy;
    camera.cx = raw.ppx;
    camera.cy = raw.ppy;
    const char* name = rs2_distortion_to_string(raw.model);
    if (model != nullptr) {
        *model = name == nullptr ? "unknown" : name;
    }
    // unav_vio 只有正向 radtan(k1,k2,p1,p2)：inverse 模型无法等价表达，按零畸变写出并告警。
    const bool inverse = raw.model == RS2_DISTORTION_INVERSE_BROWN_CONRADY;
    if (raw.model == RS2_DISTORTION_NONE || inverse) {
        for (double& value : camera.distortion) {
            value = 0.0;
        }
        camera.reported_k3 = 0.0;
        if (inverse && model != nullptr) {
            *model += " (written as zero distortion: 正向 radtan 无法表达 inverse)";
        }
        return camera;
    }
    camera.distortion = {raw.coeffs[0], raw.coeffs[1], raw.coeffs[2], raw.coeffs[3]};
    camera.reported_k3 = raw.coeffs[4];
    return camera;
}

} // namespace

RigidTransform TransformFromRs2(const float rotation[9], const float translation[3]) {
    RigidTransform transform;
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            transform.r(row, column) = rotation[row * 3 + column];
        }
    }
    transform.t = Eigen::Vector3d(translation[0], translation[1], translation[2]);
    cal::OrthonormalizeRotation(&transform.r);
    return transform;
}

struct Session::Impl {
    rs2::context context;
    rs2::device device;
    rs2::pipeline* pipeline{nullptr};
    rs2::pipeline_profile profile;
    rs2::stream_profile cam0_profile;
    rs2::stream_profile cam1_profile;
    rs2::stream_profile accel_profile;
    rs2::stream_profile gyro_profile;
    bool has_pose{false};
    bool has_depth{false};
    RigidTransform camera0_to_body;
    std::uint64_t last_pose_frame{0};
};

Session::Session() : impl_(new Impl()) {}

Session::~Session() {
    Stop();
    delete impl_->pipeline;
    delete impl_;
}

bool Session::ListDevices(std::vector<DeviceSnapshot>* devices, std::string* error) {
    if (devices == nullptr) {
        return false;
    }
    try {
        rs2::context context;
        for (auto&& info : context.query_devices()) {
            DeviceSnapshot snapshot;
            const auto assign = [&](rs2_camera_info key, std::string* target) {
                const char* value = info.get_info(key);
                if (value != nullptr) {
                    *target = value;
                }
            };
            assign(RS2_CAMERA_INFO_NAME, &snapshot.name);
            assign(RS2_CAMERA_INFO_SERIAL_NUMBER, &snapshot.serial);
            assign(RS2_CAMERA_INFO_FIRMWARE_VERSION, &snapshot.firmware);
            assign(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR, &snapshot.usb_type);
            if (snapshot.usb_type.empty()) {
                assign(RS2_CAMERA_INFO_CONNECTION_TYPE, &snapshot.usb_type);
            }
            snapshot.sdk_version = SdkVersion();
            devices->push_back(snapshot);
        }
        return true;
    } catch (const rs2::error& e) {
        if (error != nullptr) {
            *error = DescribeError(e);
        }
        return false;
    }
}

bool Session::Start(const StreamConfig& config,
                    Handlers handlers,
                    DeviceSnapshot* device,
                    CalibrationInfo* info,
                    std::string* error) {
    if (running_) {
        if (error != nullptr) {
            *error = "采集已经在运行";
        }
        return false;
    }
    handlers_ = std::move(handlers);
    try {
        bool found = false;
        for (auto&& candidate : impl_->context.query_devices()) {
            const std::string serial = candidate.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            const std::string name = candidate.get_info(RS2_CAMERA_INFO_NAME);
            if (!config.serial.empty() && config.serial != serial) {
                continue;
            }
            if (name.find("D4") == std::string::npos && serial.empty()) {
                continue;
            }
            impl_->device = candidate;
            found = true;
            break;
        }
        if (!found) {
            if (error != nullptr) {
                *error =
                    config.serial.empty()
                        ? "没有可用的 RealSense 设备（或被其它进程占用，例如 realsense-viewer）"
                        : "找不到序列号为 " + config.serial + " 的设备";
            }
            return false;
        }
        if (device != nullptr) {
            device->name = impl_->device.get_info(RS2_CAMERA_INFO_NAME);
            device->serial = impl_->device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            device->firmware = impl_->device.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION);
            if (impl_->device.supports(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR)) {
                device->usb_type = impl_->device.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR);
            } else if (impl_->device.supports(RS2_CAMERA_INFO_CONNECTION_TYPE)) {
                device->usb_type = impl_->device.get_info(RS2_CAMERA_INFO_CONNECTION_TYPE);
            }
            device->sdk_version = SdkVersion();
        }

        rs2::config pipeline_config;
        std::string sensor_notes;
        const std::vector<rs2::stream_profile> profiles =
            DeviceProfiles(impl_->device, &sensor_notes);
        auto stereo0 = PickProfile(profiles,
                                   RS2_STREAM_INFRARED,
                                   1,
                                   config.width,
                                   config.height,
                                   RS2_FORMAT_Y8,
                                   config.fps);
        auto stereo1 = PickProfile(profiles,
                                   RS2_STREAM_INFRARED,
                                   2,
                                   config.width,
                                   config.height,
                                   RS2_FORMAT_Y8,
                                   config.fps);
        // 陀螺取 400 Hz、加表取 250 Hz：两者都远高于回放的 10 ms 间隔上限，且加表只需内插。
        // 新批次 D435i 加表可能是 100 Hz 而不是 250，PickProfile 会按最接近帧率回退。
        auto gyro = PickProfile(profiles, RS2_STREAM_GYRO, -1, 0, 0, RS2_FORMAT_MOTION_XYZ32F, 400);
        auto accel =
            PickProfile(profiles, RS2_STREAM_ACCEL, -1, 0, 0, RS2_FORMAT_MOTION_XYZ32F, 250);
        if (!gyro.found) {
            gyro = PickProfile(profiles, RS2_STREAM_GYRO, -1, 0, 0, RS2_FORMAT_ANY, 400);
        }
        if (!accel.found) {
            accel = PickProfile(profiles, RS2_STREAM_ACCEL, -1, 0, 0, RS2_FORMAT_ANY, 250);
        }
        if (!stereo0.found || !stereo1.found || !gyro.found || !accel.found) {
            if (error != nullptr) {
                *error = std::string("设备未提供所需的红外或 IMU 流：") + stereo0.description +
                         " / " + stereo1.description + " / " +
                         (gyro.found ? gyro.description : std::string("gyro=无")) + " / " +
                         (accel.found ? accel.description : std::string("accel=无"));
                *error += "\n  设备上报的 sensor/profile：\n" +
                          (sensor_notes.empty() ? std::string("  （query_sensors 为空）\n")
                                                : sensor_notes);
#ifdef __APPLE__
                *error +=
                    "  固件里有 IMU（enumerate 会显示 Imu Type），但当前 Homebrew librealsense"
                    " 在 macOS 上不枚举 Motion Module（官方文档：Motion sensors are disabled）。"
                    " 红外/深度可以录，unav_vio 需要的 gyro/accel 在这台 Mac 上出不来。"
                    " 要录回放数据集请换 Linux。";
#else
                *error +=
                    "  Linux 上 IMU 走 hidraw。若红外有、gyro/accel 无：把 pack/99-realsense.rules"
                    " 拷到 /etc/udev/rules.d/，udevadm reload + 拔插相机后再试。"
                    " 也可用 Intel 官方 librealsense 包自带的 udev。";
#endif
            }
            return false;
        }
        const auto stereo0_video = stereo0.profile.as<rs2::video_stream_profile>();
        const auto stereo1_video = stereo1.profile.as<rs2::video_stream_profile>();
        pipeline_config.enable_stream(RS2_STREAM_INFRARED,
                                      1,
                                      stereo0_video.width(),
                                      stereo0_video.height(),
                                      RS2_FORMAT_Y8,
                                      stereo0_video.fps());
        pipeline_config.enable_stream(RS2_STREAM_INFRARED,
                                      2,
                                      stereo1_video.width(),
                                      stereo1_video.height(),
                                      RS2_FORMAT_Y8,
                                      stereo1_video.fps());
        pipeline_config.enable_stream(
            RS2_STREAM_GYRO, gyro.profile.format(), gyro.profile.fps());
        pipeline_config.enable_stream(
            RS2_STREAM_ACCEL, accel.profile.format(), accel.profile.fps());

        auto depth = PickProfile(profiles,
                                 RS2_STREAM_DEPTH,
                                 -1,
                                 config.width,
                                 config.height,
                                 RS2_FORMAT_Z16,
                                 config.fps);
        if (config.depth && depth.found) {
            const auto depth_video = depth.profile.as<rs2::video_stream_profile>();
            pipeline_config.enable_stream(RS2_STREAM_DEPTH,
                                          depth_video.width(),
                                          depth_video.height(),
                                          RS2_FORMAT_Z16,
                                          depth_video.fps());
            impl_->has_depth = true;
        }
        auto pose = PickProfile(profiles, RS2_STREAM_POSE, -1, 0, 0, RS2_FORMAT_6DOF, 200);
        if (config.pose && pose.found) {
            pipeline_config.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF, pose.profile.fps());
            impl_->has_pose = true;
        }

        delete impl_->pipeline;
        impl_->pipeline = new rs2::pipeline(impl_->context);
        try {
            impl_->profile = impl_->pipeline->start(pipeline_config);
        } catch (const rs2::error& e) {
            if (!impl_->has_pose) {
                throw;
            }
            // pose 流（设备端 HL-SLAM）在部分固件/系统上不可用：退回不带伪 GT 的配置。
            impl_->has_pose = false;
            impl_->has_depth = false;
            impl_->pipeline->stop();
            rs2::config fallback;
            fallback.enable_stream(RS2_STREAM_INFRARED,
                                   1,
                                   stereo0_video.width(),
                                   stereo0_video.height(),
                                   RS2_FORMAT_Y8,
                                   stereo0_video.fps());
            fallback.enable_stream(RS2_STREAM_INFRARED,
                                   2,
                                   stereo1_video.width(),
                                   stereo1_video.height(),
                                   RS2_FORMAT_Y8,
                                   stereo1_video.fps());
            fallback.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, gyro.profile.fps());
            fallback.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, accel.profile.fps());
            if (config.depth && depth.found) {
                const auto depth_video = depth.profile.as<rs2::video_stream_profile>();
                fallback.enable_stream(RS2_STREAM_DEPTH,
                                       depth_video.width(),
                                       depth_video.height(),
                                       RS2_FORMAT_Z16,
                                       depth_video.fps());
                impl_->has_depth = true;
            }
            impl_->profile = impl_->pipeline->start(fallback);
            if (error != nullptr) {
                *error = std::string("pose 流启动失败，已按无伪 GT 继续：") + DescribeError(e);
            }
        }

        impl_->cam0_profile = impl_->profile.get_stream(RS2_STREAM_INFRARED, 1);
        impl_->cam1_profile = impl_->profile.get_stream(RS2_STREAM_INFRARED, 2);
        impl_->accel_profile = impl_->profile.get_stream(RS2_STREAM_ACCEL);
        impl_->gyro_profile = impl_->profile.get_stream(RS2_STREAM_GYRO);

        const auto cam0_video = impl_->cam0_profile.as<rs2::video_stream_profile>();
        const auto cam1_video = impl_->cam1_profile.as<rs2::video_stream_profile>();

        const rs2_extrinsics cam0_to_accel =
            impl_->cam0_profile.get_extrinsics_to(impl_->profile.get_stream(RS2_STREAM_ACCEL));
        const rs2_extrinsics cam1_to_accel =
            impl_->cam1_profile.get_extrinsics_to(impl_->profile.get_stream(RS2_STREAM_ACCEL));
        const rs2_extrinsics gyro_to_accel =
            impl_->gyro_profile.get_extrinsics_to(impl_->profile.get_stream(RS2_STREAM_ACCEL));

        if (info != nullptr) {
            info->calibration.device_serial = impl_->device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            info->calibration.device_name = impl_->device.get_info(RS2_CAMERA_INFO_NAME);
            info->calibration.firmware = impl_->device.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION);
            info->calibration.camera0 =
                MapIntrinsics(cam0_video.get_intrinsics(), &info->distortion_model0);
            info->calibration.camera1 =
                MapIntrinsics(cam1_video.get_intrinsics(), &info->distortion_model1);
            info->calibration.camera0.frame_id = "camera_infra1_optical_frame";
            info->calibration.camera1.frame_id = "camera_infra2_optical_frame";
            info->calibration.camera0_to_body =
                TransformFromRs2(cam0_to_accel.rotation, cam0_to_accel.translation);
            info->calibration.camera1_to_body =
                TransformFromRs2(cam1_to_accel.rotation, cam1_to_accel.translation);
            info->calibration.gyro_to_body =
                TransformFromRs2(gyro_to_accel.rotation, gyro_to_accel.translation);
            const double gyro_offset = info->calibration.gyro_to_body.t.norm();
            const double gyro_cosine = std::max(
                -1.0, std::min(1.0, (info->calibration.gyro_to_body.r.trace() - 1.0) / 2.0));
            const double gyro_angle = std::acos(gyro_cosine) * 180.0 / M_PI;
            info->calibration.gyro_matches_accel_frame = gyro_angle < 0.1 && gyro_offset < 1e-4;
            info->accel_frame_note = "body B = accelerometer frame；gyro→accel 相对旋转 " +
                                     std::to_string(gyro_angle) + "°、平移 " +
                                     std::to_string(gyro_offset) + " m";
            info->camera_fps = cam0_video.fps();
            info->gyro_hz = impl_->gyro_profile.fps();
            info->accel_hz = impl_->accel_profile.fps();
            info->pose_available = impl_->has_pose;
            info->geometry = cal::SummarizeStereoGeometry(info->calibration);
        }

        for (const auto& sensor : impl_->device.query_sensors()) {
            if (sensor.supports(RS2_OPTION_LASER_POWER)) {
                sensor.set_option(RS2_OPTION_LASER_POWER, config.projector_on ? 1.0f : 0.0f);
            }
            if (config.exposure_microseconds > 0 &&
                sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
                sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0.0f);
                if (sensor.supports(RS2_OPTION_EXPOSURE)) {
                    sensor.set_option(RS2_OPTION_EXPOSURE,
                                      static_cast<float>(config.exposure_microseconds));
                }
            }
        }
        running_ = true;
        return true;
    } catch (const rs2::error& e) {
        Stop();
        if (error != nullptr) {
            *error = DescribeError(e);
        }
        return false;
    } catch (const std::exception& e) {
        Stop();
        if (error != nullptr) {
            *error = e.what();
        }
        return false;
    }
}

void Session::Stop() {
    if (impl_ == nullptr) {
        return;
    }
    if (impl_->pipeline != nullptr) {
        try {
            impl_->pipeline->stop();
        } catch (...) {
            // 停止阶段的异常不能覆盖已经落盘的数据集。
        }
    }
    running_ = false;
}

bool Session::Step(std::string* error) {
    if (!running_ || impl_->pipeline == nullptr) {
        if (error != nullptr) {
            *error = "采集未启动";
        }
        return false;
    }
    rs2::frameset frames;
    try {
        frames = impl_->pipeline->wait_for_frames(200);
    } catch (const rs2::error& e) {
        if (error != nullptr) {
            *error = DescribeError(e);
        }
        return false;
    }
    if (!frames) {
        return true; // 超时但没有错误：由调用方决定等待预算
    }
    ++stats_.framesets;

    rs2::frame left;
    rs2::frame right;
    rs2::frame depth;
    for (std::size_t index = 0; index < frames.size(); ++index) {
        const rs2::frame frame = frames[index];
        const rs2::stream_profile profile = frame.get_profile();
        bool metadata_available = false;
        const double metadata_ns = MetadataNs(frame, &metadata_available);
        const TimeNs stamp = FrameNs(frame);
        if (metadata_available) {
            stats_.max_timestamp_agreement_ms =
                std::max(stats_.max_timestamp_agreement_ms,
                         std::abs(metadata_ns - static_cast<double>(stamp)) * 1e-6);
        }
        switch (profile.stream_type()) {
        case RS2_STREAM_INFRARED:
            if (profile.stream_index() == 1) {
                left = frame;
            } else if (profile.stream_index() == 2) {
                right = frame;
            }
            break;
        case RS2_STREAM_DEPTH:
            depth = frame;
            break;
        case RS2_STREAM_GYRO: {
            const rs2_vector data = frame.as<rs2::motion_frame>().get_motion_data();
            GyroSample sample;
            sample.t_ns = stamp;
            sample.w = Eigen::Vector3d(data.x, data.y, data.z);
            ++stats_.gyro;
            if (handlers_.on_gyro) {
                handlers_.on_gyro(sample);
            }
            break;
        }
        case RS2_STREAM_ACCEL: {
            const rs2_vector data = frame.as<rs2::motion_frame>().get_motion_data();
            AccelSample sample;
            sample.t_ns = stamp;
            sample.a = Eigen::Vector3d(data.x, data.y, data.z);
            ++stats_.accel;
            if (handlers_.on_accel) {
                handlers_.on_accel(sample);
            }
            break;
        }
        case RS2_STREAM_POSE: {
            if (!handlers_.on_pose) {
                break;
            }
            const rs2_pose pose = frame.as<rs2::pose_frame>().get_pose_data();
            PoseRecord record;
            record.t_ns = stamp;
            // 位姿流给的是 world←camera0；再套一次 camera0→body 外参得到 world←body。
            const rs2::stream_profile pose_stream = impl_->profile.get_stream(RS2_STREAM_POSE);
            const rs2_extrinsics cam0_from_pose =
                impl_->cam0_profile.get_extrinsics_to(pose_stream);
            const RigidTransform pose_to_cam0 =
                cal::Invert(TransformFromRs2(cam0_from_pose.rotation, cam0_from_pose.translation));
            RigidTransform world_to_cam0;
            world_to_cam0.r =
                Eigen::Quaterniond(
                    pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z)
                    .normalized()
                    .toRotationMatrix();
            world_to_cam0.t =
                Eigen::Vector3d(pose.translation.x, pose.translation.y, pose.translation.z);
            const RigidTransform world_to_body = cal::Compose(pose_to_cam0, world_to_cam0);
            record.p = world_to_body.t;
            record.q = Eigen::Quaterniond(world_to_body.r).normalized();
            record.confidence = static_cast<double>(pose.tracker_confidence);
            ++stats_.pose;
            handlers_.on_pose(record);
            break;
        }
        default:
            break;
        }
    }

    if (left && right) {
        const rs2::video_frame left_video = left.as<rs2::video_frame>();
        const rs2::video_frame right_video = right.as<rs2::video_frame>();
        if (left_video.get_frame_number() != right_video.get_frame_number()) {
            ++stats_.unpaired;
        } else {
            StereoPairRecord record;
            record.frame_counter0 = left_video.get_frame_number();
            record.frame_counter1 = right_video.get_frame_number();
            const TimeNs left_ns = FrameNs(left);
            record.t_ns = left_ns; // Camera0 是共享曝光时间的权威时间戳
            record.t1_ns = FrameNs(right);
            const double skew_ms = std::abs(static_cast<double>(record.t_ns - record.t1_ns)) * 1e-6;
            stats_.max_stereo_skew_ms = std::max(stats_.max_stereo_skew_ms, skew_ms);
            const auto copy_gray = [](const rs2::video_frame& frame,
                                      std::vector<std::uint8_t>* out) {
                const int width = frame.get_width();
                const int height = frame.get_height();
                const int stride = frame.get_stride_in_bytes();
                const auto* data = static_cast<const std::uint8_t*>(frame.get_data());
                out->resize(static_cast<std::size_t>(width) * height);
                for (int row = 0; row < height; ++row) {
                    std::memcpy(out->data() + static_cast<std::size_t>(row) * width,
                                data + static_cast<std::size_t>(row) * stride,
                                static_cast<std::size_t>(width));
                }
            };
            copy_gray(left_video, &record.left);
            copy_gray(right_video, &record.right);
            ++stats_.pairs;
            if (handlers_.on_pair) {
                handlers_.on_pair(std::move(record));
            }
        }
    } else if (left || right) {
        ++stats_.unpaired;
    }

    if (depth && handlers_.on_depth) {
        const rs2::video_frame depth_video = depth.as<rs2::video_frame>();
        DepthRecord record;
        record.t_ns = FrameNs(depth);
        record.width = static_cast<std::uint32_t>(depth_video.get_width());
        record.height = static_cast<std::uint32_t>(depth_video.get_height());
        const std::size_t pixels = static_cast<std::size_t>(record.width) * record.height;
        record.millimetres.resize(pixels);
        const int stride = depth_video.get_stride_in_bytes();
        const auto* data = static_cast<const std::uint8_t*>(depth.get_data());
        for (std::uint32_t row = 0; row < record.height; ++row) {
            const auto* line = reinterpret_cast<const std::uint16_t*>(
                data + static_cast<std::size_t>(row) * stride);
            std::memcpy(record.millimetres.data() + static_cast<std::size_t>(row) * record.width,
                        line,
                        static_cast<std::size_t>(record.width) * sizeof(std::uint16_t));
        }
        ++stats_.depth;
        handlers_.on_depth(std::move(record));
    }
    return true;
}

} // namespace rh::capture

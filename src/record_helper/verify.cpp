#include "record_helper/verify.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>

#include "record_helper/euroc_format.hpp"
#include "record_helper/png_io.hpp"

namespace rh {
namespace {

constexpr const char* kSensorFiles[] = {"mav0/cam0/sensor.yaml",
                                        "mav0/cam1/sensor.yaml",
                                        "mav0/imu0/sensor.yaml",
                                        "mav0/cam0/data.csv",
                                        "mav0/cam1/data.csv",
                                        "mav0/imu0/data.csv"};

// data.csv 的一行：时间戳 + 文件名。文件名必须原样使用，回放是拿它拼路径喂 cv::imread 的。
struct CameraRow {
    TimeNs t_ns{0};
    std::string filename;
};

TimeNs ToNs(double seconds) {
    return static_cast<TimeNs>(seconds * 1e9);
}

const char* StatusName(CheckStatus status) {
    switch (status) {
    case CheckStatus::Pass:
        return "PASS";
    case CheckStatus::Warn:
        return "WARN";
    case CheckStatus::Fail:
        return "FAIL";
    }
    return "?";
}

std::string DetectBlockSequenceHint(const std::string& path) {
    std::string content;
    if (!fmt::ReadWholeFile(path, &content, nullptr)) {
        return {};
    }
    // 回放解析器只认 flow 列表；`- 640` 这种 block sequence 会让它在第一个键上就失败。
    return content.find("\n- ") != std::string::npos ? "（文件里是 YAML block sequence，"
                                                       "unav_vio 的最小解析器不接受）"
                                                     : "";
}

bool LoadCameraSensor(const std::string& path,
                      CameraIntrinsics* camera,
                      RigidTransform* to_body,
                      VerificationReport* report) {
    fmt::YamlBlock block;
    std::string error;
    bool ok = fmt::ReadYamlBlock(path, "resolution", &block, &error);
    if (!ok) {
        report->Add(CheckStatus::Fail,
                    "camera.sensor_yaml",
                    path + " resolution: " + error + DetectBlockSequenceHint(path));
        return false;
    }
    if (block.data.size() != 2) {
        report->Add(CheckStatus::Fail, "camera.sensor_yaml", path + " resolution 需要 2 个值");
        return false;
    }
    camera->width = static_cast<std::uint32_t>(block.data[0]);
    camera->height = static_cast<std::uint32_t>(block.data[1]);

    if (!ok || !fmt::ReadYamlBlock(path, "intrinsics", &block, &error) || block.data.size() != 4) {
        report->Add(CheckStatus::Fail,
                    "camera.sensor_yaml",
                    path +
                        " intrinsics: " + (error.empty() ? "需要恰好 4 个值 fx fy cx cy" : error));
        return false;
    }
    camera->fx = block.data[0];
    camera->fy = block.data[1];
    camera->cx = block.data[2];
    camera->cy = block.data[3];

    if (!fmt::ReadYamlBlock(path, "distortion_coefficients", &block, &error)) {
        report->Add(
            CheckStatus::Fail, "camera.sensor_yaml", path + " distortion_coefficients: " + error);
        return false;
    }
    // 上游允许 4 或 5 个，但第 5 个（k3）必须为 0：radtan 模型不支持 k3。
    if (block.data.size() == 5) {
        camera->reported_k3 = block.data[4];
        block.data.pop_back();
    }
    if (block.data.size() != 4) {
        report->Add(CheckStatus::Fail,
                    "camera.sensor_yaml",
                    path + " distortion_coefficients 只能是 4 个（或第 5 个为 0 的 5 个）");
        return false;
    }
    for (int index = 0; index < 4; ++index) {
        camera->distortion[index] = block.data[index];
    }
    if (std::abs(camera->reported_k3) > 1e-6) {
        report->Add(CheckStatus::Fail,
                    "camera.k3",
                    path + " 第 5 个畸变系数 k3=" + fmt::FormatDouble(camera->reported_k3) +
                        "，unav_vio 的 radtan 模型没有 k3，必须改写成无畸变或先做修正");
    }

    if (!fmt::ReadYamlBlock(path, "T_BS", &block, &error)) {
        report->Add(CheckStatus::Fail, "camera.sensor_yaml", path + " T_BS: " + error);
        return false;
    }
    if (block.cols != 4 || (block.rows != 3 && block.rows != 4) ||
        block.data.size() != static_cast<std::size_t>(block.cols) * block.rows) {
        report->Add(
            CheckStatus::Fail, "camera.sensor_yaml", path + " T_BS 需要 cols: 4 与 rows: 3 或 4");
        return false;
    }
    *to_body = cal::TransformFromT_BS(block.data);
    report->Add(CheckStatus::Pass, "camera.sensor_yaml", path + " 可按上游规则解析");
    return true;
}

bool LoadImuSensor(const std::string& path, ImuNoise* noise, VerificationReport* report) {
    struct Key {
        const char* yaml;
        double ImuNoise::* target;
        const char* label;
    };
    const Key keys[] = {
        {"gyroscope_noise_density", &ImuNoise::sigma_g, "sigma_g"},
        {"gyroscope_random_walk", &ImuNoise::sigma_bg, "sigma_bg"},
        {"accelerometer_noise_density", &ImuNoise::sigma_a, "sigma_a"},
        {"accelerometer_random_walk", &ImuNoise::sigma_ba, "sigma_ba"},
    };
    bool ok = true;
    for (const auto& key : keys) {
        double value = 0.0;
        std::string error;
        if (!fmt::ReadYamlScalar(path, key.yaml, &value, &error)) {
            report->Add(CheckStatus::Fail, "imu.noise_density", error);
            ok = false;
            continue;
        }
        noise->*(key.target) = value;
    }
    noise->g = 9.81; // 上游 euroc_dataset.cpp 硬编码，录制端无法通过文件改写
    if (ok) {
        report->Add(CheckStatus::Pass,
                    "imu.noise_density",
                    "sigma_a=" + fmt::FormatDouble(noise->sigma_a) +
                        " sigma_g=" + fmt::FormatDouble(noise->sigma_g) +
                        " sigma_ba=" + fmt::FormatDouble(noise->sigma_ba) +
                        " sigma_bg=" + fmt::FormatDouble(noise->sigma_bg));
    } else {
        report->Add(CheckStatus::Warn,
                    "imu.g",
                    "g 由 unav_vio 回放侧固定为 9.81，静止段 ‖a‖ 必须与之相符才能用零速假设");
    }
    return ok;
}

bool ParseImuCsv(const std::string& path,
                 std::vector<ImuRow>* rows,
                 VerificationReport* report,
                 const VerifyOptions& options) {
    std::string content;
    std::string error;
    if (!fmt::ReadWholeFile(path, &content, &error)) {
        report->Add(CheckStatus::Fail, "imu.data_csv", error);
        return false;
    }
    const auto csv = fmt::SplitCsvRows(content);
    TimeNs previous = -1;
    double max_gap = 0.0;
    double sum_gap = 0.0;
    std::size_t repeats = 0;
    std::size_t gyro_repeats = 0;
    std::size_t bad_rows = 0;
    for (std::size_t index = 0; index < csv.size(); ++index) {
        const auto& fields = csv[index];
        if (fields.size() != 7) {
            if (bad_rows == 0) {
                report->Add(CheckStatus::Fail,
                            "imu.data_csv",
                            path + " 第 " + std::to_string(index + 1) + " 行有 " +
                                std::to_string(fields.size()) + " 个字段，上游要求恰好 7 个");
            }
            ++bad_rows;
            continue;
        }
        ImuRow row;
        double values[6] = {0.0};
        if (!fmt::ParseInt64(fields[0], &row.t_ns) || row.t_ns < 0) {
            report->Add(CheckStatus::Fail,
                        "imu.data_csv",
                        path + " 第 " + std::to_string(index + 1) +
                            " 行时间戳必须是非负整数纳秒（不接受小数/科学计数法）");
            return false;
        }
        bool numbers_ok = true;
        for (int field = 0; field < 6; ++field) {
            if (!fmt::ParseDouble(fields[field + 1], &values[field])) {
                numbers_ok = false;
                break;
            }
        }
        if (!numbers_ok) {
            report->Add(CheckStatus::Fail,
                        "imu.data_csv",
                        path + " 第 " + std::to_string(index + 1) + " 行有非法数值");
            return false;
        }
        row.w = Eigen::Vector3d(values[0], values[1], values[2]);
        row.a = Eigen::Vector3d(values[3], values[4], values[5]);
        if (previous >= 0) {
            if (row.t_ns <= previous) {
                report->Add(CheckStatus::Fail,
                            "imu.monotonic",
                            path + " 第 " + std::to_string(index + 1) +
                                " 行时间戳未严格递增，上游会整文件失败");
                return false;
            }
            const double gap_ms = (row.t_ns - previous) * 1e-6;
            max_gap = std::max(max_gap, gap_ms);
            sum_gap += gap_ms;
            if (row.a == rows->back().a) {
                ++repeats;
            }
            if (row.w == rows->back().w) {
                ++gyro_repeats;
            }
        }
        previous = row.t_ns;
        rows->push_back(row);
    }
    if (bad_rows > 0) {
        return false;
    }
    if (rows->size() < 4) {
        report->Add(CheckStatus::Fail, "imu.data_csv", "IMU 行数不足，无法初始化");
        return false;
    }
    report->Add(max_gap <= options.limits.max_imu_gap_ms ? CheckStatus::Pass : CheckStatus::Fail,
                "imu.gap",
                "最大相邻间隔 " + fmt::FormatDouble(max_gap) + " ms，上限 " +
                    fmt::FormatDouble(options.limits.max_imu_gap_ms) + " ms");
    const double denominator = static_cast<double>(rows->size() - 1);
    const double hold = static_cast<double>(repeats) / denominator;
    report->Add(hold > 0.05 ? CheckStatus::Warn : CheckStatus::Pass,
                "imu.zero_order_hold",
                "相邻行加表完全相同的比例 " + fmt::FormatDouble(hold * 100.0) +
                    " %（room_02 是 38%，来自 ROS 的零阶保持合并）");
    // 抽稀栅格下陀螺才是可能被重复使用的那一列，所以两列都得看。
    const double gyro_hold = static_cast<double>(gyro_repeats) / denominator;
    report->Add(gyro_hold > 0.05 ? CheckStatus::Warn : CheckStatus::Pass,
                "imu.gyro_repeat",
                "相邻行陀螺完全相同的比例 " + fmt::FormatDouble(gyro_hold * 100.0) +
                    " %（加表栅格抽稀时非零属正常，接近 1 - 加表/陀螺 才说明退化成零阶保持）");
    report->mean_imu_gap_ms = sum_gap / denominator;
    report->max_imu_gap_ms = max_gap;
    report->accel_hold_ratio = hold;
    report->gyro_hold_ratio = gyro_hold;
    report->imu_begin_ns = rows->front().t_ns;
    report->imu_end_ns = rows->back().t_ns;
    report->imu_rows = rows->size();
    report->Add(CheckStatus::Pass,
                "imu.data_csv",
                std::to_string(rows->size()) + " 行，标称 " +
                    fmt::FormatDouble(1e3 / report->mean_imu_gap_ms) + " Hz");
    // 上游 integration.mh01_replay 在加载阶段就硬性要求 IMU 行数 > 30000，
    // 光满足「能解析」并不等于「能回放」，所以这条必须在录制端就报出来。
    const bool long_enough = rows->size() >= options.min_imu_rows;
    const double missing_seconds =
        (static_cast<double>(options.min_imu_rows) - static_cast<double>(rows->size())) *
        report->mean_imu_gap_ms / 1000.0;
    report->Add(long_enough ? CheckStatus::Pass : CheckStatus::Fail,
                "imu.length",
                long_enough ? std::to_string(rows->size()) + " 行 ≥ 回放测试要求的 " +
                                  std::to_string(options.min_imu_rows) + " 行"
                            : std::to_string(rows->size()) + " 行 < 回放测试要求的 " +
                                  std::to_string(options.min_imu_rows) +
                                  " 行（vio_mh01_replay_test.cpp:499），按当前速率还需再录约 " +
                                  fmt::FormatDouble(missing_seconds) + " s");
    return true;
}

bool ParseCameraCsv(const std::string& path,
                    std::vector<CameraRow>* rows_out,
                    VerificationReport* report,
                    bool required = true) {
    std::string content;
    std::string error;
    if (!fmt::ReadWholeFile(path, &content, &error)) {
        // 可选流（depth0）缺失不是契约违规：--no-depth 录出来的包本来就没有。
        if (required) {
            report->Add(CheckStatus::Fail, "camera.data_csv", error);
        }
        return false;
    }
    const auto csv = fmt::SplitCsvRows(content);
    TimeNs previous = -1;
    for (std::size_t index = 0; index < csv.size(); ++index) {
        const auto& fields = csv[index];
        if (fields.size() != 2 || fields[1].empty()) {
            report->Add(CheckStatus::Fail,
                        "camera.data_csv",
                        path + " 第 " + std::to_string(index + 1) +
                            " 行需要恰好 timestamp,filename 两列");
            return false;
        }
        CameraRow row;
        if (!fmt::ParseInt64(fields[0], &row.t_ns) || row.t_ns < 0 ||
            (previous >= 0 && row.t_ns <= previous)) {
            report->Add(CheckStatus::Fail,
                        "camera.data_csv",
                        path + " 第 " + std::to_string(index + 1) + " 行时间戳必须非负且严格递增");
            return false;
        }
        previous = row.t_ns;
        row.filename = fields[1];
        rows_out->push_back(row);
    }
    return true;
}

bool ParseGroundTruth(const std::string& path, std::size_t* rows, VerificationReport* report) {
    std::string content;
    std::string error;
    if (!fmt::ReadWholeFile(path, &content, &error)) {
        return false;
    }
    const auto csv = fmt::SplitCsvRows(content);
    TimeNs previous = -1;
    for (std::size_t index = 0; index < csv.size(); ++index) {
        const auto& fields = csv[index];
        if (fields.size() != 17) {
            report->Add(CheckStatus::Fail,
                        "ground_truth",
                        path + " 第 " + std::to_string(index + 1) + " 行需要 17 列，实际 " +
                            std::to_string(fields.size()));
            return false;
        }
        double values[16] = {};
        for (int field = 0; field < 16; ++field) {
            if (!fmt::ParseDouble(fields[field + 1], &values[field])) {
                report->Add(CheckStatus::Fail,
                            "ground_truth",
                            path + " 第 " + std::to_string(index + 1) + " 行有非法数值");
                return false;
            }
        }
        const double norm = std::sqrt(values[3] * values[3] + values[4] * values[4] +
                                      values[5] * values[5] + values[6] * values[6]);
        if (std::abs(norm - 1.0) > 1e-3) {
            report->Add(CheckStatus::Fail,
                        "ground_truth",
                        path + " 第 " + std::to_string(index + 1) + " 行四元数模长 " +
                            fmt::FormatDouble(norm) + " 偏离 1 超过 1e-3");
            return false;
        }
        std::int64_t time = 0;
        if (!fmt::ParseInt64(fields[0], &time) || (previous >= 0 && time <= previous)) {
            report->Add(CheckStatus::Fail, "ground_truth", path + " 时间戳必须严格递增");
            return false;
        }
        previous = time;
        ++*rows;
    }
    return true;
}

void CheckImages(const std::string& mav0,
                 const std::string& camera_dir,
                 const std::vector<CameraRow>& rows_out,
                 const CameraIntrinsics& camera,
                 VerificationReport* report,
                 const VerifyOptions& options) {
    std::size_t missing = 0;
    for (const auto& row : rows_out) {
        std::ifstream probe(fmt::JoinPath(mav0, camera_dir + "/data/" + row.filename));
        if (!probe.good()) {
            ++missing;
        }
    }
    report->Add(missing == 0 ? CheckStatus::Pass : CheckStatus::Fail,
                "camera.images",
                camera_dir + " 引用 " + std::to_string(rows_out.size()) + " 个文件，缺失 " +
                    std::to_string(missing));
    if (missing > 0 || !options.check_images || rows_out.empty()) {
        return;
    }
    std::vector<std::size_t> sample;
    const std::size_t count = std::min(options.image_sample, rows_out.size());
    for (std::size_t index = 0; index < count; ++index) {
        sample.push_back(index * (rows_out.size() - 1) / (count > 1 ? count - 1 : 1));
    }
    std::size_t bad = 0;
    std::string first_error;
    for (const std::size_t index : sample) {
        const std::string path =
            fmt::JoinPath(mav0, camera_dir + "/data/" + rows_out[index].filename);
        PngHeaderInfo info;
        std::string error;
        if (!InspectPng(path, &info, &error) || info.color_type != 0 || info.bit_depth != 8 ||
            info.interlace != 0 || !info.terminated || info.width != camera.width ||
            info.height != camera.height) {
            ++bad;
            if (first_error.empty()) {
                first_error =
                    path + " 不是完整收尾的 8-bit 单通道 " + std::to_string(camera.width) + "x" +
                    std::to_string(camera.height) + " PNG（实际 " + std::to_string(info.width) +
                    "x" + std::to_string(info.height) + " depth=" + std::to_string(info.bit_depth) +
                    " color=" + std::to_string(info.color_type) + "）";
            }
        }
    }
    report->Add(bad == 0 ? CheckStatus::Pass : CheckStatus::Fail,
                "camera.pixel_format",
                camera_dir + " 抽样 " + std::to_string(sample.size()) + " 张，异常 " +
                    std::to_string(bad) + (bad > 0 ? "：" + first_error : ""));
}

} // namespace

std::size_t VerificationReport::Count(CheckStatus status) const {
    std::size_t total = 0;
    for (const auto& check : checks) {
        total += check.status == status ? 1 : 0;
    }
    return total;
}

void VerificationReport::Add(CheckStatus status, std::string id, std::string detail) {
    checks.push_back(CheckResult{status, std::move(id), std::move(detail)});
}

VerificationReport VerifyDataset(const std::string& dataset_root, const VerifyOptions& options) {
    VerificationReport report;
    report.dataset_root = dataset_root;
    const std::string mav0 = fmt::JoinPath(dataset_root, "mav0");

    std::vector<std::string> missing;
    for (const char* file : kSensorFiles) {
        std::ifstream probe(fmt::JoinPath(dataset_root, file));
        if (!probe.good()) {
            missing.emplace_back(file);
        }
    }
    if (!missing.empty()) {
        std::string detail;
        for (const auto& item : missing) {
            detail += (detail.empty() ? "" : ", ") + item;
        }
        report.Add(CheckStatus::Fail, "layout", "缺少必需文件: " + detail);
        return report;
    }
    report.Add(CheckStatus::Pass, "layout", "mav0 目录结构完整");

    CameraIntrinsics camera0;
    CameraIntrinsics camera1;
    RigidTransform to_body0;
    RigidTransform to_body1;
    const bool sensors_ok =
        LoadCameraSensor(fmt::JoinPath(mav0, "cam0/sensor.yaml"), &camera0, &to_body0, &report) &&
        LoadCameraSensor(fmt::JoinPath(mav0, "cam1/sensor.yaml"), &camera1, &to_body1, &report);
    LoadImuSensor(fmt::JoinPath(mav0, "imu0/sensor.yaml"), &report.noise, &report);
    if (sensors_ok) {
        report.calibration.camera0 = camera0;
        report.calibration.camera1 = camera1;
        report.calibration.camera0_to_body = to_body0;
        report.calibration.camera1_to_body = to_body1;
        report.calibration_parsed = true;
        auto validation =
            cal::ValidateCalibration(report.calibration, report.noise, options.expected_g);
        for (const auto& problem : validation.problems) {
            report.Add(CheckStatus::Fail, "calibration", problem);
        }
        for (const auto& warning : validation.warnings) {
            report.Add(CheckStatus::Warn, "calibration", warning);
        }
        if (validation.ok()) {
            const auto geometry = cal::SummarizeStereoGeometry(report.calibration);
            report.Add(
                geometry.relative_rotation_deg > 1.0 ? CheckStatus::Warn : CheckStatus::Pass,
                "calibration.geometry",
                "基线 " + fmt::FormatDouble(geometry.baseline_m * 1000.0) + " mm，两目光轴相对角 " +
                    fmt::FormatDouble(geometry.relative_rotation_deg) + "°，非轴向基线比例 " +
                    fmt::FormatDouble(geometry.off_axis_baseline_ratio) + "；" +
                    geometry.interpretation);
        }
        // 单位阵「数值上合法但可能整体错」，值得提醒；但它是哪一边错，数据集本身说不清：
        // 取决于该路径把加表数据交在哪个系里。本机 SDK 直读实测过，记录系与外参表同口径、
        // 单位阵成立（见 record_summary.yaml 的 static_gravity_vs_t_bs_col3_deg）。
        // 所以这里只提示，不判死——判死会把好数据也拒掉。
        const bool identity_like =
            to_body0.r(0, 0) > 0.999 && to_body0.r(1, 1) > 0.999 && to_body0.r(2, 2) > 0.999;
        if (identity_like) {
            report.Add(CheckStatus::Warn,
                       "calibration.identity_extrinsic",
                       "T_BS 旋转是单位阵：只有当 imu0 的数据系与外参表同口径时才成立。"
                       "录制端用静止段实测核对（rh record 打印夹角并写进 record_summary.yaml）；"
                       "外来数据集无法据此判定时，以回放初始化结果为准");
        }
    }

    std::vector<ImuRow> imu_rows;
    const bool imu_ok =
        ParseImuCsv(fmt::JoinPath(mav0, "imu0/data.csv"), &imu_rows, &report, options);
    if (!imu_ok) {
        report.imu_rows = imu_rows.size();
    }

    std::vector<CameraRow> frames0;
    std::vector<CameraRow> frames1;
    std::vector<CameraRow> paired_frames; // 参与配对、且文件名与图像对得上的双目序列
    const bool cam0_ok = ParseCameraCsv(fmt::JoinPath(mav0, "cam0/data.csv"), &frames0, &report);
    const bool cam1_ok = ParseCameraCsv(fmt::JoinPath(mav0, "cam1/data.csv"), &frames1, &report);
    report.cam0_rows = frames0.size();
    report.cam1_rows = frames1.size();

    std::vector<CameraRow> synced0;
    std::vector<CameraRow> synced1;
    if (cam0_ok && cam1_ok) {
        // 复刻 synchronize_stereo_stamps：只在 ns 完全相等时配对，其余一律计入 dropped_count。
        std::size_t left = 0;
        std::size_t right = 0;
        while (left < frames0.size() && right < frames1.size()) {
            if (frames0[left].t_ns == frames1[right].t_ns) {
                synced0.push_back(frames0[left]);
                synced1.push_back(frames1[right]); // 两目文件名可能不同，各自留一份
                ++left;
                ++right;
                continue;
            }
            ++report.dropped_pairs;
            if (frames0[left].t_ns < frames1[right].t_ns) {
                ++left;
            } else {
                ++right;
            }
        }
        report.dropped_pairs += (frames0.size() - left) + (frames1.size() - right);
        report.synced_pairs = synced0.size();
        report.Add(report.dropped_pairs == 0 ? CheckStatus::Pass : CheckStatus::Fail,
                   "stereo.hard_sync",
                   "可配对 " + std::to_string(report.synced_pairs) + " 对，dropped_count=" +
                       std::to_string(report.dropped_pairs) + "（回放断言要求 0；room_02 是 41）");
        double max_gap_ms = 0.0;
        for (std::size_t index = 1; index < synced0.size(); ++index) {
            max_gap_ms =
                std::max(max_gap_ms, (synced0[index].t_ns - synced0[index - 1].t_ns) * 1e-6);
        }
        report.max_frame_gap_ms = max_gap_ms;
        report.Add(max_gap_ms <= options.limits.max_frame_gap_ms ? CheckStatus::Pass
                                                                 : CheckStatus::Fail,
                   "camera.frame_gap",
                   "最大帧间隔 " + fmt::FormatDouble(max_gap_ms) + " ms，上限 " +
                       fmt::FormatDouble(options.limits.max_frame_gap_ms) + " ms");
        if (report.calibration_parsed) {
            CheckImages(mav0, "cam0", synced0, camera0, &report, options);
            CheckImages(mav0, "cam1", synced1, camera1, &report, options);
        }
        if (!synced0.empty()) {
            report.first_pair_ns = synced0.front().t_ns;
            report.last_pair_ns = synced0.back().t_ns;
        }
        paired_frames = synced0;
    }

    std::size_t gt_rows = 0;
    const std::string gt_path = fmt::JoinPath(mav0, "state_groundtruth_estimate0/data.csv");
    const bool gt_present = ParseGroundTruth(gt_path, &gt_rows, &report);
    report.gt_rows = gt_rows;
    if (!gt_present) {
        report.Add(options.require_ground_truth ? CheckStatus::Fail : CheckStatus::Warn,
                   "ground_truth",
                   "缺少 " + gt_path +
                       "：当前 integration.mh01_replay 会在这里早退（伪 GT 或放宽该断言二选一）");
    } else {
        const bool overlaps = gt_rows >= 2;
        report.Add(overlaps ? CheckStatus::Pass : CheckStatus::Warn,
                   "ground_truth",
                   "GT " + std::to_string(gt_rows) +
                       " 行；伪 GT 只用于给出相对偏差，不构成绝对精度声明");
    }

    std::vector<CameraRow> depth_frames;
    if (ParseCameraCsv(fmt::JoinPath(mav0, "depth0/data.csv"), &depth_frames, &report, false)) {
        report.depth_rows = depth_frames.size();
        report.Add(CheckStatus::Pass,
                   "depth",
                   "depth0 " + std::to_string(depth_frames.size()) +
                       " 帧（unav_vio 不读取，仅作留档）");
    } else {
        report.Add(CheckStatus::Warn, "depth", "没有 depth0：估计链不需要，仅影响后续可视化");
    }

    if (imu_ok && !paired_frames.empty()) {
        std::vector<ImuRowSample> samples;
        samples.reserve(imu_rows.size());
        for (const auto& row : imu_rows) {
            samples.push_back(ImuRowSample{row.t_ns, row.a, row.w});
        }
        std::vector<WindowStat> windows;
        const auto boundary = FindStaticMotionBoundary(samples, options.probe, &windows);
        if (!boundary.has_value()) {
            double best_accel = 1e9;
            double best_gyro = 1e9;
            for (const auto& window : windows) {
                if (window.samples >= options.probe.min_window_samples) {
                    best_accel = std::min(best_accel, window.accel_rms);
                    best_gyro = std::min(best_gyro, window.gyro_rms);
                }
            }
            report.Add(
                CheckStatus::Fail,
                "initialization.static_window",
                "没有静止→运动边界：全序列最静窗口 accel_rms=" + fmt::FormatDouble(best_accel) +
                    " gyro_rms=" + fmt::FormatDouble(best_gyro) + "，阈值 " +
                    fmt::FormatDouble(options.probe.max_accel_rms) + "/" +
                    fmt::FormatDouble(options.probe.max_gyro_rms) + "。开头请平放 " +
                    fmt::FormatDouble(options.probe.window_s * options.probe.static_run) +
                    " s 以上，随后 4 s 内开始缓慢运动");
        } else {
            report.boundary = boundary;
            const TimeNs anchor_time =
                boundary->feed_start_ns + ToNs(options.limits.anchor_offset_s);
            std::size_t anchor = paired_frames.size();
            std::size_t source_row = 0;
            for (std::size_t index = 0; index < paired_frames.size(); ++index) {
                if (paired_frames[index].t_ns >= anchor_time) {
                    anchor = index;
                    source_row = index + 1;
                    break;
                }
            }
            report.anchor_index = anchor;
            report.available_frames =
                anchor < paired_frames.size() ? paired_frames.size() - anchor : 0;
            const bool anchor_found = anchor < paired_frames.size();
            report.Add(anchor_found ? CheckStatus::Pass : CheckStatus::Fail,
                       "initialization.anchor",
                       anchor_found
                           ? "回放将从源数据行 " + std::to_string(source_row) + " 起喂入，可用 " +
                                 std::to_string(report.available_frames) + " 对双目"
                           : "静止边界之后没有相机帧");
            const TimeNs imu_feed_start =
                anchor_found ? paired_frames[anchor].t_ns - options.limits.imu_lead_ns : 0;
            report.Add(anchor_found && imu_feed_start >= boundary->feed_start_ns
                           ? CheckStatus::Pass
                           : CheckStatus::Fail,
                       "initialization.imu_lead",
                       "anchor 前 " + fmt::FormatDouble(options.limits.imu_lead_ns * 1e-6) +
                           " ms 处的 IMU 必须仍落在静止窗内（feed_start=" +
                           std::to_string(boundary->feed_start_ns) +
                           ", imu_feed_start=" + std::to_string(imu_feed_start) + "）");
            report.Add(anchor_found && report.available_frames >= 8 ? CheckStatus::Pass
                                                                    : CheckStatus::Fail,
                       "initialization.bootstrap",
                       "双目米制 bootstrap 需要 " + std::to_string(report.available_frames) +
                           " 对可用帧（至少 8 对才有 7 个相邻区间）");
            const bool tail_covered = imu_rows.back().t_ns >= report.last_pair_ns;
            report.Add(tail_covered ? CheckStatus::Pass : CheckStatus::Fail,
                       "imu.tail_coverage",
                       tail_covered
                           ? "最后一个曝光 " + std::to_string(report.last_pair_ns) +
                                 " 已被 IMU 末尾 " + std::to_string(imu_rows.back().t_ns) + " 覆盖"
                           : "IMU 未覆盖最后一个曝光区间，该帧会被丢弃");
            // 静止段重力模长：决定零速假设与初始化门控能否成立。
            Eigen::Vector3d sum = Eigen::Vector3d::Zero();
            std::size_t samples_used = 0;
            for (const auto& row : imu_rows) {
                if (row.t_ns >= boundary->feed_start_ns &&
                    row.t_ns <= boundary->feed_start_ns + ToNs(0.25)) {
                    sum += row.a;
                    ++samples_used;
                }
            }
            if (samples_used > 0) {
                report.static_accel_norm = (sum / static_cast<double>(samples_used)).norm();
                const auto gravity = cal::CheckGravity(report.static_accel_norm,
                                                       options.expected_g,
                                                       options.static_accel_norm_tolerance);
                report.Add(gravity.pass ? CheckStatus::Pass : CheckStatus::Fail,
                           "initialization.gravity",
                           "静止段 ‖a‖=" + fmt::FormatDouble(gravity.mean_norm) +
                               "，与回放侧固定的 g=9.81 偏差 " + fmt::FormatDouble(gravity.error) +
                               " m/s²，容差 " +
                               fmt::FormatDouble(options.static_accel_norm_tolerance));
            }
        }
    }
    return report;
}

void PrintReport(const VerificationReport& report, std::FILE* stream) {
    std::fprintf(stream, "数据集: %s\n", report.dataset_root.c_str());
    for (const auto& check : report.checks) {
        std::fprintf(stream,
                     "[%s] %-28s %s\n",
                     StatusName(check.status),
                     check.id.c_str(),
                     check.detail.c_str());
    }
    std::fprintf(stream,
                 "汇总: 双目 %zu/%zu 行（配对 %zu，丢弃 %zu），IMU %zu 行（最大间隔 %.3f ms，"
                 "加表重复 %.1f%%，陀螺重复 %.1f%%），GT %zu 行\n",
                 report.cam0_rows,
                 report.cam1_rows,
                 report.synced_pairs,
                 report.dropped_pairs,
                 report.imu_rows,
                 report.max_imu_gap_ms,
                 report.accel_hold_ratio * 100.0,
                 report.gyro_hold_ratio * 100.0,
                 report.gt_rows);
    if (report.boundary.has_value()) {
        std::fprintf(stream,
                     "静止→运动边界: motion_start_ns=%lld feed_start_ns=%lld anchor 行=%zu\n",
                     static_cast<long long>(report.boundary->motion_start_ns),
                     static_cast<long long>(report.boundary->feed_start_ns),
                     report.anchor_index + 1);
    }
    std::fprintf(stream,
                 "结论: %s（fail=%zu warn=%zu）\n",
                 report.ok() ? "可按 unav_vio 回放契约加载" : "不满足回放契约",
                 report.Count(CheckStatus::Fail),
                 report.Count(CheckStatus::Warn));
}

} // namespace rh

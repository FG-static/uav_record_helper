#include "check.hpp"
#include "record_helper/dataset_writer.hpp"
#include "record_helper/euroc_format.hpp"
#include "record_helper/verify.hpp"
#include "synthetic.hpp"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

bool HasCheck(const rh::VerificationReport& report, const std::string& id, rh::CheckStatus status) {
    for (const auto& check : report.checks) {
        if (check.id == id && check.status == status) {
            return true;
        }
    }
    return false;
}

bool HasFailing(const rh::VerificationReport& report, const std::string& id) {
    return HasCheck(report, id, rh::CheckStatus::Fail);
}

std::string FirstFailure(const rh::VerificationReport& report) {
    for (const auto& check : report.checks) {
        if (check.status == rh::CheckStatus::Fail) {
            return check.id + ": " + check.detail;
        }
    }
    return {};
}

// 变异用例共用一份合法数据集：先构造一次，再整份复制后破坏单点。
class DatasetFixture {
  public:
    DatasetFixture() {
        std::string error;
        valid_root_ = rh::fmt::JoinPath(base_.path(), "valid");
        auto capture = rh::testing::MakeSynthetic(4.0, 3.0, 4.0, 1.0);
        rh::DatasetWriter writer(valid_root_,
                                 rh::testing::SyntheticCalibration(),
                                 rh::testing::SyntheticNoise(),
                                 rh::testing::SyntheticOptions(),
                                 rh::testing::SyntheticProvenance("fixture"));
        if (!writer.Start(&error)) {
            return;
        }
        for (auto& record : capture.pairs) {
            writer.Enqueue(std::move(record));
        }
        for (auto& record : capture.depth) {
            writer.Enqueue(std::move(record));
        }
        writer.Stop();
        writer.Finalize(capture.gyro, capture.accel, capture.poses, &result_, &error);
        built_ = result_.pairs > 0;
    }

    [[nodiscard]] bool built() const { return built_; }
    const rh::WriteResult& result() const { return result_; }

    // 返回一份可安全改写的副本目录（进程退出时随基目录一起清理）。
    std::string Copy(const std::string& name) const {
        const std::string destination = rh::fmt::JoinPath(base_.path(), name);
        const std::string command = "cp -R '" + valid_root_ + "' '" + destination + "'";
        if (std::system(command.c_str()) != 0) {
            return {};
        }
        return destination;
    }

    const std::string& valid_root() const { return valid_root_; }

  private:
    rh::testing::TempDirectory base_{"rh-verify-fixture"};
    std::string valid_root_;
    rh::WriteResult result_;
    bool built_{false};
};

DatasetFixture& Fixture() {
    static DatasetFixture fixture;
    return fixture;
}

bool ReplaceInFile(const std::string& path, const std::string& from, const std::string& to) {
    std::string content;
    std::string error;
    if (!rh::fmt::ReadWholeFile(path, &content, &error)) {
        return false;
    }
    const std::size_t position = content.find(from);
    if (position == std::string::npos) {
        return false;
    }
    content.replace(position, from.size(), to);
    return rh::fmt::WriteWholeFile(path, content, &error);
}

std::vector<std::string> ReadLines(const std::string& path) {
    std::vector<std::string> lines;
    std::string content;
    std::string error;
    if (!rh::fmt::ReadWholeFile(path, &content, &error)) {
        return lines;
    }
    std::istringstream stream(content);
    std::string line;
    while (std::getline(stream, line)) {
        lines.push_back(line);
    }
    return lines;
}

bool WriteLines(const std::string& path, const std::vector<std::string>& lines) {
    std::string content;
    std::string error;
    for (const auto& line : lines) {
        content += line + "\n";
    }
    return rh::fmt::WriteWholeFile(path, content, &error);
}

} // namespace

// 合成夹具只有 12 s，达不到上游 >30000 行 IMU 的门槛，所以这里显式放宽该单项。
rh::VerifyOptions FixtureOptions() {
    rh::VerifyOptions options;
    options.min_imu_rows = 0;
    return options;
}

RH_TEST(verify_accepts_generated_dataset) {
    CHECK(Fixture().built());
    const auto report = rh::VerifyDataset(Fixture().valid_root(), FixtureOptions());
    if (!report.ok()) {
        std::printf("    意外失败 %s\n", FirstFailure(report).c_str());
    }
    CHECK(report.ok());
    CHECK(HasCheck(report, "stereo.hard_sync", rh::CheckStatus::Pass));
    CHECK(!HasFailing(report, "initialization.static_window"));
    CHECK(!HasFailing(report, "initialization.anchor"));
    CHECK(!HasFailing(report, "initialization.imu_lead"));
    CHECK(!HasFailing(report, "imu.tail_coverage"));
    CHECK(report.calibration_parsed);
    CHECK(report.noise.sigma_g > 0.0);
    CHECK(report.static_accel_norm > 9.0 && report.static_accel_norm < 10.5);
    CHECK(report.synced_pairs == Fixture().result().pairs);
    CHECK(report.available_frames > 100);
    CHECK(report.max_imu_gap_ms <= 10.0);
    CHECK(report.accel_hold_ratio < 0.05);
}

RH_TEST(verify_rejects_block_sequence_yaml) {
    const std::string root = Fixture().Copy("block-seq");
    CHECK(!root.empty());
    const std::string path = rh::fmt::JoinPath(root, "mav0/cam0/sensor.yaml");
    CHECK(ReplaceInFile(path, "resolution: [160, 120]", "resolution:\n- 160\n- 120"));
    const auto report = rh::VerifyDataset(root, FixtureOptions());
    CHECK(HasFailing(report, "camera.sensor_yaml"));
    CHECK(!report.ok());
}

RH_TEST(verify_rejects_missing_imu_noise) {
    const std::string root = Fixture().Copy("no-noise");
    CHECK(ReplaceInFile(rh::fmt::JoinPath(root, "mav0/imu0/sensor.yaml"),
                        "gyroscope_noise_density:",
                        "#gyroscope_noise_density:"));
    const auto report = rh::VerifyDataset(root, FixtureOptions());
    CHECK(HasFailing(report, "imu.noise_density"));
    CHECK(HasFailing(report, "calibration"));
}

RH_TEST(verify_rejects_unpaired_camera_timestamps) {
    const std::string root = Fixture().Copy("skew");
    const std::string path = rh::fmt::JoinPath(root, "mav0/cam1/data.csv");
    auto lines = ReadLines(path);
    std::vector<std::string> shifted;
    for (const auto& line : lines) {
        if (line.empty() || line.front() == '#') {
            shifted.push_back(line);
            continue;
        }
        std::int64_t time = 0;
        CHECK(rh::fmt::ParseInt64(line.substr(0, line.find(',')), &time));
        const std::int64_t moved = time + 1'000'000; // 右目整体平移 1 ms
        shifted.push_back(std::to_string(moved) + "," + std::to_string(moved) + ".png");
    }
    CHECK(WriteLines(path, shifted));
    const auto report = rh::VerifyDataset(root, FixtureOptions());
    CHECK(HasFailing(report, "stereo.hard_sync"));
    CHECK(report.dropped_pairs > 0);
}

RH_TEST(verify_warns_on_identity_extrinsics) {
    // 单位阵「数值上合法但口径可疑」，值得提醒；但数据集自己说不清哪边错：本机 SDK 直读实测过，
    // 记录系与外参表同口径时单位阵就是对的。所以它只能是 WARN，不能把好数据判死。
    const std::string root = Fixture().Copy("identity-tbs");
    const std::string path = rh::fmt::JoinPath(root, "mav0/cam0/sensor.yaml");
    std::string content;
    std::string error;
    CHECK(rh::fmt::ReadWholeFile(path, &content, &error));
    const std::size_t begin = content.find("T_BS:");
    const std::size_t line_end = content.find('\n', content.find("data:", begin));
    CHECK(begin != std::string::npos);
    const std::string identity =
        "T_BS:\n  cols: 4\n  rows: 4\n  data: [1.0, 0.0, 0.0, -0.0055, 0.0, 1.0, 0.0, 0.021, "
        "0.0, 0.0, 1.0, 0.0117, 0.0, 0.0, 0.0, 1.0]\n";
    content = content.substr(0, begin) + identity + content.substr(line_end + 1);
    CHECK(rh::fmt::WriteWholeFile(path, content, &error));
    const auto report = rh::VerifyDataset(root, FixtureOptions());
    CHECK(HasCheck(report, "calibration.identity_extrinsic", rh::CheckStatus::Warn));
    CHECK(!HasFailing(report, "calibration.identity_extrinsic"));
}

RH_TEST(verify_rejects_broken_imu) {
    {
        // 时间戳重复：模拟 ROS 合并出的同一时刻两条样本。
        const std::string root = Fixture().Copy("dup-time");
        const std::string path = rh::fmt::JoinPath(root, "mav0/imu0/data.csv");
        auto lines = ReadLines(path);
        std::vector<std::string> rewritten;
        std::size_t data_rows = 0;
        for (auto& line : lines) {
            if (!line.empty() && line.front() != '#') {
                ++data_rows;
                if (data_rows == 6) {
                    const auto previous = rewritten.back();
                    rewritten.push_back(previous.substr(0, previous.find(',')) +
                                        line.substr(line.find(',')));
                    continue;
                }
            }
            rewritten.push_back(line);
        }
        CHECK(WriteLines(path, rewritten));
        const auto report = rh::VerifyDataset(root, FixtureOptions());
        CHECK(HasFailing(report, "imu.monotonic"));
    }
    {
        // 中段缺 100 行（>10 ms 间隔）：整文件仍然可解析，但回放会在 TimeGap 上拒绝。
        const std::string root = Fixture().Copy("gap");
        const std::string path = rh::fmt::JoinPath(root, "mav0/imu0/data.csv");
        auto lines = ReadLines(path);
        std::vector<std::string> rewritten;
        std::size_t data_rows = 0;
        for (const auto& line : lines) {
            if (line.empty() || line.front() == '#') {
                rewritten.push_back(line);
                continue;
            }
            ++data_rows;
            if (data_rows > 150 && data_rows <= 250) {
                continue;
            }
            rewritten.push_back(line);
        }
        CHECK(WriteLines(path, rewritten));
        const auto report = rh::VerifyDataset(root, FixtureOptions());
        CHECK(HasFailing(report, "imu.gap"));
    }
}

RH_TEST(verify_rejects_frame_and_image_defects) {
    {
        const std::string root = Fixture().Copy("bad-size");
        CHECK(ReplaceInFile(rh::fmt::JoinPath(root, "mav0/cam0/sensor.yaml"),
                            "resolution: [160, 120]",
                            "resolution: [320, 240]"));
        const auto report = rh::VerifyDataset(root, FixtureOptions());
        CHECK(HasFailing(report, "camera.pixel_format"));
    }
    {
        const std::string root = Fixture().Copy("missing-image");
        const std::string path = rh::fmt::JoinPath(root, "mav0/cam0/data.csv");
        const auto lines = ReadLines(path);
        const std::string last = lines.back();
        const std::string name = last.substr(last.find(',') + 1);
        // 只在磁盘上删掉一张被 CSV 引用的图：契约要求引用与文件严格一致。
        CHECK(std::remove(rh::fmt::JoinPath(root, "mav0/cam0/data/" + name).c_str()) == 0);
        const auto report = rh::VerifyDataset(root, FixtureOptions());
        CHECK(HasFailing(report, "camera.images"));
    }
    {
        // 大段空洞导致帧间隔 > 150 ms：回放的 max_frame_gap 会拒。
        const std::string root = Fixture().Copy("frame-gap");
        for (const char* camera : {"cam0", "cam1"}) {
            const std::string path =
                rh::fmt::JoinPath(root, std::string("mav0/") + camera + "/data.csv");
            auto lines = ReadLines(path);
            std::vector<std::string> kept;
            std::size_t index = 0;
            for (const auto& line : lines) {
                if (line.empty() || line.front() == '#') {
                    kept.push_back(line);
                    continue;
                }
                ++index;
                if (index > 20 && index <= 60) {
                    continue; // 从两份 CSV 同时抽掉 40 帧，制造 >150 ms 的曝光空洞
                }
                kept.push_back(line);
            }
            CHECK(WriteLines(path, kept));
        }
        const auto report = rh::VerifyDataset(root, FixtureOptions());
        CHECK(HasFailing(report, "camera.frame_gap"));
    }
}

RH_TEST(verify_reports_missing_ground_truth) {
    const std::string root = Fixture().Copy("no-gt");
    std::remove(rh::fmt::JoinPath(root, "mav0/state_groundtruth_estimate0/data.csv").c_str());
    rh::VerifyOptions strict = FixtureOptions();
    const auto strict_report = rh::VerifyDataset(root, strict);
    CHECK(HasFailing(strict_report, "ground_truth"));
    CHECK(!strict_report.ok());
    rh::VerifyOptions lenient = FixtureOptions();
    lenient.require_ground_truth = false;
    const auto relaxed = rh::VerifyDataset(root, lenient);
    CHECK(HasCheck(relaxed, "ground_truth", rh::CheckStatus::Warn));
    CHECK(!HasFailing(relaxed, "ground_truth"));
}

RH_TEST(verify_flags_dataset_too_short_for_replay) {
    // 上游 integration.mh01_replay 要求 IMU 行数 > 30000；12 s 的夹具必须被这条挡下，
    // 并且给出「还差多少秒」这种能直接指导重录的结论。
    const auto report = rh::VerifyDataset(Fixture().valid_root(), rh::VerifyOptions{});
    CHECK(HasFailing(report, "imu.length"));
    CHECK(!report.ok());
    bool mentions_seconds = false;
    for (const auto& check : report.checks) {
        if (check.id == "imu.length") {
            mentions_seconds = check.detail.find("还需再录") != std::string::npos;
        }
    }
    CHECK(mentions_seconds);
}

RH_TEST(verify_rejects_missing_layout) {
    const rh::testing::TempDirectory temp("rh-verify-empty");
    const auto report = rh::VerifyDataset(temp.path(), FixtureOptions());
    CHECK(HasFailing(report, "layout"));
    CHECK(!report.ok());
    const auto missing = rh::VerifyDataset("/tmp/rh-definitely-not-a-dataset", FixtureOptions());
    CHECK(HasFailing(missing, "layout"));
}

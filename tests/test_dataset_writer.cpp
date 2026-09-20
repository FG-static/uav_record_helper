#include "check.hpp"
#include "record_helper/dataset_writer.hpp"
#include "record_helper/euroc_format.hpp"
#include "record_helper/verify.hpp"
#include "synthetic.hpp"

#include <chrono>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

using rh::DatasetWriter;
using rh::TimeNs;
using rh::WriteResult;

namespace {

int CountLines(const std::string& path) {
    std::ifstream file(path);
    int lines = 0;
    std::string line;
    while (std::getline(file, line)) {
        if (!line.empty() && line.front() != '#') {
            ++lines;
        }
    }
    return file.good() || file.eof() ? lines : -1;
}

bool FileExists(const std::string& path) {
    std::ifstream probe(path);
    return probe.good();
}

std::string ReadFile(const std::string& path) {
    std::string content;
    std::string error;
    static_cast<void>(rh::fmt::ReadWholeFile(path, &content, &error));
    return content;
}

void EnqueueAll(DatasetWriter& writer, std::vector<rh::StereoPairRecord>& pairs) {
    for (auto& record : pairs) {
        CHECK(writer.Enqueue(std::move(record))); // 夹具队列足够大，不该丢
    }
}

// 走完「合成采集 → 落盘 → 出 CSV/YAML」全流程，返回写盘结果供各用例复用。
WriteResult WriteSyntheticDataset(const std::string& root,
                                  rh::WriteOptions options = rh::testing::SyntheticOptions()) {
    auto capture = rh::testing::MakeSynthetic(4.0, 3.0, 4.0, 1.0);
    DatasetWriter writer(root,
                         rh::testing::SyntheticCalibration(),
                         rh::testing::SyntheticNoise(),
                         options,
                         rh::testing::SyntheticProvenance("synthetic_case"));
    std::string error;
    if (!writer.Start(&error)) {
        std::printf("  Start 失败: %s\n", error.c_str());
        return WriteResult{};
    }
    EnqueueAll(writer, capture.pairs);
    for (auto& record : capture.depth) {
        writer.Enqueue(std::move(record));
    }
    writer.Stop();
    WriteResult result;
    if (!writer.Finalize(capture.gyro, capture.accel, capture.poses, &result, &error)) {
        std::printf("  Finalize 失败: %s\n", error.c_str());
        return WriteResult{};
    }
    return result;
}

} // namespace

RH_TEST(dataset_writer_produces_replayable_tree) {
    const rh::testing::TempDirectory temp("rh-write");
    CHECK(temp.valid());
    const std::string root = rh::fmt::JoinPath(temp.path(), "room_synth");
    const WriteResult result = WriteSyntheticDataset(root);
    CHECK(result.pairs > 100);
    CHECK(result.dropped_queue_full == 0);
    CHECK(result.imu_rows > 1000);
    CHECK(result.gt_rows > 100);
    CHECK(result.boundary.has_value());

    const std::string mav0 = rh::fmt::JoinPath(root, "mav0");
    CHECK(CountLines(rh::fmt::JoinPath(mav0, "cam0/data.csv")) == static_cast<int>(result.pairs));
    CHECK(CountLines(rh::fmt::JoinPath(mav0, "cam1/data.csv")) == static_cast<int>(result.pairs));
    CHECK(CountLines(rh::fmt::JoinPath(mav0, "imu0/data.csv")) ==
          static_cast<int>(result.imu_rows));
    CHECK(CountLines(rh::fmt::JoinPath(mav0, "depth0/data.csv")) ==
          static_cast<int>(result.depth_frames));
    CHECK(CountLines(rh::fmt::JoinPath(mav0, "state_groundtruth_estimate0/data.csv")) ==
          static_cast<int>(result.gt_rows));

    // 两份相机 CSV 必须逐行相同：这是 synchronize_stereo_stamps 的 dropped_count==0 前提。
    CHECK(ReadFile(rh::fmt::JoinPath(mav0, "cam0/data.csv")) ==
          ReadFile(rh::fmt::JoinPath(mav0, "cam1/data.csv")));

    const std::string imu = ReadFile(rh::fmt::JoinPath(mav0, "imu0/data.csv"));
    CHECK(imu.rfind("#timestamp [ns],w_RS_S_x", 0) == 0);
    const auto rows = rh::fmt::SplitCsvRows(imu);
    CHECK(rows.size() == result.imu_rows);
    for (const auto& row : rows) {
        CHECK(row.size() == 7);
    }
    // 时间戳既不是 0 基也不是 Unix epoch：合成数据用设备单调时钟量级，回放只关心差值。
    std::int64_t first_time = 0;
    CHECK(rh::fmt::ParseInt64(rows.front()[0], &first_time));
    CHECK(first_time > 1'000'000'000'000);

    const std::string cam0_yaml = ReadFile(rh::fmt::JoinPath(mav0, "cam0/sensor.yaml"));
    CHECK(cam0_yaml.find("resolution: [160, 120]") != std::string::npos);
    CHECK(cam0_yaml.find("intrinsics: [") != std::string::npos);
    CHECK(cam0_yaml.find("distortion_coefficients: [") != std::string::npos);
    CHECK(cam0_yaml.find("\n- ") == std::string::npos); // 绝不写 block sequence
    const std::string imu_yaml = ReadFile(rh::fmt::JoinPath(mav0, "imu0/sensor.yaml"));
    CHECK(imu_yaml.find("gyroscope_noise_density:") != std::string::npos);
    CHECK(imu_yaml.find("accelerometer_random_walk:") != std::string::npos);
    CHECK(FileExists(rh::fmt::JoinPath(root, "record_summary.yaml")));
    CHECK(FileExists(
        rh::fmt::JoinPath(mav0, "cam0/data/" + std::to_string(result.first_pair_ns) + ".png")));

    // 端到端：合成录制必须直接通过回放契约校验。
    rh::VerifyOptions verify_options;
    verify_options.min_imu_rows = 0;
    const auto report = rh::VerifyDataset(root, verify_options);
    if (!report.ok()) {
        rh::PrintReport(report, stdout);
    }
    CHECK(report.ok());
    CHECK(report.synced_pairs == result.pairs);
    CHECK(report.dropped_pairs == 0);
    CHECK(report.available_frames > 100);
}

RH_TEST(dataset_writer_trims_uncovered_tail) {
    // 加表比陀螺早停 1 s：尾部若干双目帧没有 IMU 右端覆盖，必须由写盘阶段裁掉而不是补值。
    const rh::testing::TempDirectory temp("rh-trim");
    auto capture = rh::testing::MakeSynthetic(4.0, 3.0, 4.0, 1.0);
    const TimeNs cutoff = capture.accel.back().t_ns - static_cast<TimeNs>(1.0e9);
    std::vector<rh::AccelSample> trimmed;
    for (const auto& sample : capture.accel) {
        if (sample.t_ns <= cutoff) {
            trimmed.push_back(sample);
        }
    }
    capture.accel = trimmed;
    const std::string root = rh::fmt::JoinPath(temp.path(), "trim");
    DatasetWriter writer(root,
                         rh::testing::SyntheticCalibration(),
                         rh::testing::SyntheticNoise(),
                         rh::testing::SyntheticOptions(),
                         rh::testing::SyntheticProvenance("trim"));
    std::string error;
    CHECK(writer.Start(&error));
    EnqueueAll(writer, capture.pairs);
    writer.Stop();
    WriteResult result;
    CHECK(writer.Finalize(capture.gyro, capture.accel, capture.poses, &result, &error));
    CHECK(result.last_pair_ns <= result.imu_end_ns);
    CHECK(result.last_pair_ns <= cutoff);
    CHECK(result.trimmed_pairs > 0);
    CHECK(capture.pairs.back().t_ns > result.last_pair_ns);
    CHECK(!FileExists(rh::fmt::JoinPath(
        root, "mav0/cam0/data/" + std::to_string(capture.pairs.back().t_ns) + ".png")));
    rh::VerifyOptions verify_options;
    verify_options.min_imu_rows = 0; // 合成夹具只有 12 s
    const auto report = rh::VerifyDataset(root, verify_options);
    if (!report.ok()) {
        rh::PrintReport(report, stdout);
    }
    CHECK(report.ok());
    CHECK(report.last_pair_ns <= report.imu_end_ns);
    // 裁掉的帧对应的 PNG 不该留下孤儿文件。
    CHECK(!FileExists(rh::fmt::JoinPath(
              root, "mav0/cam0/data/" + std::to_string(capture.pairs.front().t_ns) + ".png")) ||
          result.first_pair_ns == capture.pairs.front().t_ns);
}

RH_TEST(dataset_writer_keeps_frames_across_midtrack_pause) {
    // 正式录制中途静止 2.5 s 再起步：窗口必须保留暂停之前的帧，不能裁到最后一个静止边界。
    const rh::testing::TempDirectory temp("rh-pause");
    const rh::TimeNs origin = 8'640'000'000'000LL;
    auto capture = rh::testing::MakeSynthetic(4.0, 3.0, 8.0, 1.0, origin, 8.0, 10.5);
    const std::string root = rh::fmt::JoinPath(temp.path(), "pause");
    DatasetWriter writer(root,
                         rh::testing::SyntheticCalibration(),
                         rh::testing::SyntheticNoise(),
                         rh::testing::SyntheticOptions(),
                         rh::testing::SyntheticProvenance("pause"));
    std::string error;
    CHECK(writer.Start(&error));
    EnqueueAll(writer, capture.pairs);
    writer.Stop();
    WriteResult result;
    CHECK(writer.Finalize(capture.gyro, capture.accel, capture.poses, &result, &error));
    CHECK(result.boundary.has_value());
    CHECK(result.boundary->motion_start_ns > origin + 8'000'000'000LL);
    CHECK(result.first_pair_ns < origin + 5'000'000'000LL);
    CHECK(result.last_pair_ns > origin + 10'500'000'000LL);
    rh::VerifyOptions verify_options;
    verify_options.min_imu_rows = 0;
    const auto report = rh::VerifyDataset(root, verify_options);
    if (!report.ok()) {
        rh::PrintReport(report, stdout);
    }
    CHECK(report.ok());
}

RH_TEST(dataset_writer_queue_overflow_keeps_pairs_complete) {
    const rh::testing::TempDirectory temp("rh-overflow");
    auto capture = rh::testing::MakeSynthetic(4.0, 3.0, 3.0, 1.0);
    rh::WriteOptions options = rh::testing::SyntheticOptions();
    options.write_depth = false;
    const std::string root = rh::fmt::JoinPath(temp.path(), "overflow");
    DatasetWriter writer(root,
                         rh::testing::SyntheticCalibration(),
                         rh::testing::SyntheticNoise(),
                         options,
                         rh::testing::SyntheticProvenance("overflow"));
    std::string error;
    CHECK(writer.Start(&error));
    // 一次性灌入全部帧：队列必然溢出，但丢弃只能是整对。
    std::size_t submitted = 0;
    for (auto& record : capture.pairs) {
        ++submitted;
        writer.Enqueue(std::move(record));
    }
    writer.Stop();
    WriteResult result;
    CHECK(writer.Finalize(capture.gyro, capture.accel, capture.poses, &result, &error));
    CHECK(result.pairs <= submitted);
    CHECK(ReadFile(rh::fmt::JoinPath(root, "mav0/cam0/data.csv")) ==
          ReadFile(rh::fmt::JoinPath(root, "mav0/cam1/data.csv")));
    rh::VerifyOptions overflow_options;
    overflow_options.min_imu_rows = 0;
    const auto report = rh::VerifyDataset(root, overflow_options);
    CHECK(report.dropped_pairs == 0);
}

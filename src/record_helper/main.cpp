#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "cli_common.hpp"
#include "record_helper/verify.hpp"
#ifdef __APPLE__
#include "macos_privacy.hpp"
#endif

namespace {

void PrintUsage(const char* program) {
    std::printf("RecordHelper —— D435i 一键录制，输出 unav_vio 可直接回放的数据集\n"
                "\n"
                "用法:\n"
                "  %s probe   [选项]     探测设备能力、内参、外参与时间戳域，不写文件\n"
                "  %s record  [选项]     分阶段引导录制：静止标定 → 温和激励 → 手动停 → 出盘 → 自检\n"
                "  %s verify  DIR [--no-gt-required] [--deep]\n"
                "                          按 unav_vio 回放解析规则审查任意 mav0 数据集\n"
                "\n"
                "录制产物目录结构（与 EuRoC 一致，且刻意避开了 room_02 的 7 处不可回放问题）:\n"
                "  <out>/<seq>/mav0/cam0/{sensor.yaml,data.csv,data/}\n"
                "  <out>/<seq>/mav0/cam1/{sensor.yaml,data.csv,data/}\n"
                "  <out>/<seq>/mav0/imu0/{sensor.yaml,data.csv}\n"
                "  <out>/<seq>/mav0/depth0/{data.csv,data/}      unav_vio 不读，仅留档\n"
                "  <out>/<seq>/mav0/state_groundtruth_estimate0/data.csv  设备端位姿流伪 GT\n"
                "  <out>/<seq>/record_summary.yaml               录制溯源，不被解析\n"
                "\n"
                "回放:\n"
                "  env UNAV_VIO_EUROC_MH01=<out>/<seq> ctest --test-dir <unav_vio build> "
                "-R mh01_replay --output-on-failure\n"
                "\n"
                "子命令的具体选项用 `%s <子命令> --help` 查看。\n",
                program,
                program,
                program,
                program);
}

int RunVerify(int argc, char** argv) {
    rh::cli::ArgumentParser parser(argc, argv, "verify");
    std::string error;
    if (!parser.Parse()) {
        std::printf("%s\n", error.c_str());
        return 2;
    }
    const std::vector<std::string> known{"no-gt-required", "deep", "help", "samples"};
    if (!parser.RejectUnknown(known, &error)) {
        std::printf("%s\n", error.c_str());
        return 2;
    }
    if (parser.Positional().empty()) {
        std::printf("verify 需要数据集目录（含 mav0/）\n");
        return 2;
    }
    const std::string root = parser.Positional()[0];
    rh::VerifyOptions options;
    options.require_ground_truth = !parser.Has("no-gt-required");
    options.check_images = true;
    options.image_sample = parser.Integer("samples", parser.Has("deep") ? 512 : 64, &error);
    if (!error.empty()) {
        std::printf("%s\n", error.c_str());
        return 2;
    }
    const auto report = rh::VerifyDataset(root, options);
    rh::PrintReport(report, stdout);
    return report.ok() ? 0 : 1;
}

} // namespace

namespace rh::cli {
int RunProbe(int argc, char** argv);
int RunRecord(int argc, char** argv);
} // namespace rh::cli

int main(int argc, char** argv) {
    if (argc < 2) {
        PrintUsage(argv[0]);
        return 2;
    }
    const std::string command = argv[1];
    if (command == "record" || command == "probe") {
#ifdef __APPLE__
        std::string privacy_error;
        if (!rh::macos::EnsureCameraAccess(&privacy_error)) {
            std::fprintf(stderr, "%s\n", privacy_error.c_str());
            return 1;
        }
#endif
        if (command == "record") {
            return rh::cli::RunRecord(argc - 1, argv + 1);
        }
        return rh::cli::RunProbe(argc - 1, argv + 1);
    }
    if (command == "verify") {
        return RunVerify(argc - 1, argv + 1);
    }
    if (command == "help" || command == "--help" || command == "-h") {
        PrintUsage(argv[0]);
        return 0;
    }
    std::printf("未知子命令: %s\n\n", command.c_str());
    PrintUsage(argv[0]);
    return 2;
}

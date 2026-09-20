#include "check.hpp"
#include "record_helper/verify.hpp"

#include <cstdlib>
#include <fstream>
#include <string>

namespace {

bool HasFailing(const rh::VerificationReport& report, const std::string& id) {
    for (const auto& check : report.checks) {
        if (check.id == id && check.status == rh::CheckStatus::Fail) {
            return true;
        }
    }
    return false;
}

} // namespace

// 这不是「我们的数据集」用例，而是把 room_02 为什么不能直接回放钉成可执行文档：
// 任何一条曾经失败的检查重新通过，说明上游解析器变了，需要重新评估录制契约。
RH_TEST(verify_reports_room02_blockers) {
    const char* from_env = std::getenv("RH_ROOM02");
    const std::string root =
        from_env != nullptr && *from_env != '\0' ? from_env : "/Users/mac/datasets/room_02";
    std::ifstream probe(root + "/mav0/imu0/data.csv");
    if (!probe.good()) {
        std::printf("    跳过：找不到 room_02（%s），可用 RH_ROOM02=... 指定\n", root.c_str());
        return;
    }
    const auto report = rh::VerifyDataset(root, rh::VerifyOptions{});
    std::printf("    room_02 检查结论:\n");
    for (const auto& check : report.checks) {
        if (check.status != rh::CheckStatus::Pass) {
            std::printf("      [%s] %s %s\n",
                        check.status == rh::CheckStatus::Fail ? "FAIL" : "WARN",
                        check.id.c_str(),
                        check.detail.c_str());
        }
    }
    CHECK(!report.ok());
    CHECK(HasFailing(report, "camera.sensor_yaml"));           // block sequence
    CHECK(HasFailing(report, "imu.noise_density"));            // 四个噪声键缺失
    CHECK(HasFailing(report, "stereo.hard_sync"));             // 左右时间戳集合不全等
    CHECK(!report.calibration_parsed);                         // block sequence 让外参根本读不进来
    CHECK(HasFailing(report, "initialization.static_window")); // 开头没有静止段
    CHECK(HasFailing(report, "ground_truth"));                 // 没有 state_groundtruth_estimate0
    CHECK(report.dropped_pairs > 0);
    CHECK(report.accel_hold_ratio > 0.1); // ROS unite_imu_method 的零阶保持痕迹
}

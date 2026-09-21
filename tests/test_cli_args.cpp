#include "check.hpp"
#include "cli_common.hpp"
#include "record_helper/imu_align.hpp"

#include <deque>
#include <string>
#include <vector>

namespace {

// record 自己声明的那批带字符串取值的选项；解析器只认这些，其余仍按数字取值处理。
const std::vector<std::string>& RecordFlags() {
    static const std::vector<std::string> flags{
        "out", "seq", "serial", "imu-grid", "pose-gt", "projector"};
    return flags;
}

// argv 必须比构造函数用得久，所以每个取值都存进稳定的 deque。
rh::cli::ArgumentParser MakeParser(const std::vector<std::string>& args) {
    static std::deque<std::string> storage;
    std::vector<char*> argv;
    argv.push_back(nullptr);
    for (const auto& argument : args) {
        storage.push_back(argument);
        argv.push_back(const_cast<char*>(storage.back().c_str()));
    }
    return rh::cli::ArgumentParser(static_cast<int>(argv.size()), argv.data(), "record");
}

} // namespace

RH_TEST(cli_args_consumes_keyword_values) {
    auto parser = MakeParser({"--imu-grid",
                              "accel",
                              "--seq",
                              "room_04",
                              "--pose-gt",
                              "off",
                              "--projector",
                              "on",
                              "--still",
                              "5"});
    CHECK(parser.Parse(RecordFlags()));
    CHECK(parser.Value("imu-grid", "gyro") == "accel");
    CHECK(parser.Value("seq", "fallback") == "room_04");
    CHECK(parser.Value("pose-gt", "auto") == "off");
    CHECK(parser.Value("projector", "on") == "on");
    std::string error;
    CHECK(parser.Number("still", 4.0, &error) == 5.0);
    CHECK(error.empty());
    // 关键：这些词不该漏成游离的位置参数，否则选项等于被静默忽略。
    CHECK(parser.Positional().empty());

    rh::ImuGridMode mode = rh::ImuGridMode::kGyroTimestamps;
    CHECK(rh::ParseImuGridMode(parser.Value("imu-grid", "gyro"), &mode));
    CHECK(mode == rh::ImuGridMode::kAccelTimestamps);
}

RH_TEST(cli_args_accepts_equals_form_and_default) {
    auto equals = MakeParser({"--imu-grid=accel"});
    CHECK(equals.Parse(RecordFlags()));
    CHECK(equals.Value("imu-grid", "gyro") == "accel");

    auto unset = MakeParser({"--still", "5"});
    CHECK(unset.Parse(RecordFlags()));
    CHECK(unset.Value("imu-grid", "gyro") == "gyro");

    // 只写了选项名没给值：走默认值，也不能把后面的数字吞进来当取值。
    auto bare = MakeParser({"--imu-grid", "--still", "5"});
    CHECK(bare.Parse(RecordFlags()));
    CHECK(bare.Value("imu-grid", "gyro") == "gyro");
    std::string error;
    CHECK(bare.Number("still", 4.0, &error) == 5.0);
    CHECK(error.empty());
}

RH_TEST(cli_args_keeps_numeric_flags_working) {
    auto parser =
        MakeParser({"--serial", "939622074967", "--imu-grid", "accel", "--png-level", "6"});
    CHECK(parser.Parse(RecordFlags()));
    CHECK(parser.Value("serial") == "939622074967");
    CHECK(parser.Value("imu-grid", "gyro") == "accel");
    std::string error;
    CHECK(parser.Integer("png-level", 3, &error) == 6);
    CHECK(error.empty());
}

RH_TEST(cli_args_misspelled_flag_does_not_swallow_value) {
    // 拼错的 --imu-grod 没被声明成字符串取值选项，所以不该吃掉紧跟的 accel；
    // 而 RejectUnknown 必须当场报出来，不能让录制带着默认栅格静默启动设备。
    auto parser = MakeParser({"--imu-grod", "accel"});
    CHECK(parser.Parse(RecordFlags()));
    CHECK(parser.Value("imu-grod") == "1"); // 空值退回哨兵 "1"，说明真的没拿到取值
    CHECK(parser.Positional().size() == 1);
    std::string error;
    const std::vector<std::string> known{"imu-grid", "still"};
    CHECK(!parser.RejectUnknown(known, &error));
    CHECK(!error.empty());
}

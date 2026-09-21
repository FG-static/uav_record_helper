#include "check.hpp"
#include "record_helper/euroc_format.hpp"

#include <cstdint>
#include <string>
#include <vector>

RH_TEST(euroc_format_number_parsing) {
    double value = 0.0;
    CHECK(rh::fmt::ParseDouble("  -0.25 ", &value));
    CHECK_NEAR(value, -0.25, 0.0);
    CHECK(rh::fmt::ParseDouble("1.76187114e-05", &value));
    CHECK_NEAR(value, 1.76187114e-05, 1e-15);
    CHECK(rh::fmt::ParseDouble("+3.5", &value));
    CHECK(!rh::fmt::ParseDouble("1.5junk", &value));
    CHECK(!rh::fmt::ParseDouble("NaN", &value));
    CHECK(!rh::fmt::ParseDouble("inf", &value));
    CHECK(!rh::fmt::ParseDouble("0x10", &value));
    CHECK(!rh::fmt::ParseDouble("", &value));
    CHECK(!rh::fmt::ParseDouble("++2", &value));

    std::int64_t time = 0;
    CHECK(rh::fmt::ParseInt64("1789887830574471436", &time));
    CHECK(time == 1789887830574471436LL);
    // 上游用 std::from_chars<int64_t>：小数与科学计数法都必须拒绝。
    CHECK(!rh::fmt::ParseInt64("1.5", &time));
    CHECK(!rh::fmt::ParseInt64("1e3", &time));
    CHECK(!rh::fmt::ParseInt64("0012", &time) || time == 12);
    CHECK(!rh::fmt::ParseInt64("-5x", &time));
}

RH_TEST(euroc_format_double_round_trip) {
    const std::vector<double> samples = {0.0,
                                         -0.0,
                                         1.0,
                                         384.4474792480469,
                                         1.0947644252537633e-47,
                                         -0.28340811,
                                         1.76187114e-05,
                                         1789887830574471436.0,
                                         4.09e-05,
                                         9.81};
    for (const double sample : samples) {
        const std::string text = rh::fmt::FormatDouble(sample);
        CHECK(!text.empty());
        // 科学计数法的 e+18 是合法写法，但绝不允许出现 +0.25 这种前导加号。
        CHECK(text.front() != '+');
        CHECK(text.find("nan") == std::string::npos);
        double parsed = 0.0;
        CHECK(rh::fmt::ParseDouble(text, &parsed));
        CHECK(parsed == sample);
    }
    CHECK(rh::fmt::FormatDouble(std::numeric_limits<double>::quiet_NaN()).empty());
}

RH_TEST(euroc_format_csv_rows) {
    const std::string content = "#timestamp [ns],filename\r\n"
                                "1789887830574471436,1789887830574471436.png\r\n"
                                "\r\n"
                                "  1789887830607811279 , 1789887830607811279.png \r\n"
                                "trailing,\r\n";
    const auto rows = rh::fmt::SplitCsvRows(content);
    CHECK(rows.size() == 3);
    CHECK(rows[0].size() == 2);
    CHECK(rows[0][0] == "1789887830574471436");
    CHECK(rows[1].size() == 2);
    CHECK(rows[1][1] == "1789887830607811279.png");
    CHECK(rows[2].size() == 2);
    CHECK(rows[2][1].empty()); // 尾随逗号产生空字段，和上游一致
}

RH_TEST(euroc_format_yaml_flow_and_matrix) {
    const rh::testing::TempDirectory temp("rh-yaml");
    const std::string path = rh::fmt::JoinPath(temp.path(), "sensor.yaml");
    std::string error;
    std::string yaml = "#General sensor definitions.\n";
    yaml += "sensor_type: camera\n";
    yaml += "resolution: [848, 480]\n";
    yaml += "intrinsics: [437.5, 437.5, 425.5, 240.5]  # 注释必须在解析前剥掉\n";
    yaml += "distortion_coefficients: [0.0, 0.0, 0.0, 0.0]\n";
    yaml += rh::fmt::FormatMatrixField(
        "T_BS", 4, 4, {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16});
    yaml += "rate_hz: 30\n";
    CHECK(rh::fmt::WriteWholeFile(path, yaml, &error));

    rh::fmt::YamlBlock block;
    CHECK(rh::fmt::ReadYamlBlock(path, "resolution", &block, &error));
    CHECK(block.data.size() == 2 && block.data[0] == 848 && block.data[1] == 480);
    CHECK(rh::fmt::ReadYamlBlock(path, "intrinsics", &block, &error));
    CHECK(block.data.size() == 4);
    CHECK(rh::fmt::ReadYamlBlock(path, "T_BS", &block, &error));
    CHECK(block.rows == 4);
    CHECK(block.cols == 4);
    CHECK(block.data.size() == 16);
    CHECK_NEAR(block.data[3], 4.0, 0.0); // 行主序，平移在第 3/7/11 个
    double scalar = 0.0;
    CHECK(rh::fmt::ReadYamlScalar(path, "rate_hz", &scalar, &error));
    CHECK_NEAR(scalar, 30.0, 0.0);
    CHECK(!rh::fmt::ReadYamlScalar(path, "missing_key", &scalar, &error));
}

RH_TEST(euroc_format_reads_block_sequence) {
    // 上游 read_yaml_block 已接受 block sequence（含 PyYAML 默认的「与 key 同缩进」写法）。
    // 对偶实现必须跟上，否则「rh verify 通过 ⟺ 上游能解析」这个前提就断了。
    const rh::testing::TempDirectory temp("rh-yaml-blockseq");
    const std::string path = rh::fmt::JoinPath(temp.path(), "sensor.yaml");
    std::string error;
    const std::string yaml = "sensor_type: camera\n"
                             "resolution:\n"
                             "- 640\n"
                             "- 480\n"
                             "T_BS:\n"
                             "  rows: 4\n"
                             "  cols: 4\n"
                             "  data:\n"
                             "  - 1.0\n"
                             "  - 0.0\n"
                             "  - 0.0\n"
                             "  - 0.0\n"
                             "  - 0.0\n"
                             "  - 1.0\n"
                             "  - 0.0\n"
                             "  - 0.0\n"
                             "  - 0.0\n"
                             "  - 0.0\n"
                             "  - 1.0\n"
                             "  - 0.0\n"
                             "  - 0.0\n"
                             "  - 0.0\n"
                             "  - 0.0\n"
                             "  - 1.0\n"
                             "rate_hz: 30\n";
    CHECK(rh::fmt::WriteWholeFile(path, yaml, &error));
    rh::fmt::YamlBlock block;
    CHECK(rh::fmt::ReadYamlBlock(path, "resolution", &block, &error));
    CHECK(block.data.size() == 2 && block.cols == 2 && block.rows == 1 && block.data[0] == 640.0 &&
          block.data[1] == 480.0);
    error.clear();
    CHECK(rh::fmt::ReadYamlBlock(path, "T_BS", &block, &error));
    CHECK(block.data.size() == 16 && block.cols == 4 && block.rows == 4);
    // 块之后的顶层键不能被吃掉。
    double rate = 0.0;
    CHECK(rh::fmt::ReadYamlScalar(path, "rate_hz", &rate, &error));
    CHECK(rate == 30.0);

    // 仍然要拒的：元素数与 rows/cols 不符（理由必须是数量不匹配，不是语法不认识）、
    // 以及无空格负数混进块里（那不是合法项标记）。
    CHECK(rh::fmt::WriteWholeFile(path, "T_BS:\n  cols: 4\n  rows: 4\n  data:\n  - 1.0\n  - 2.0\n",
                                  &error));
    CHECK(!rh::fmt::ReadYamlBlock(path, "T_BS", &block, &error));
    CHECK(rh::fmt::WriteWholeFile(
        path, "T_BS:\n  cols: 1\n  rows: 2\n  data:\n  - 1.0\n-0.5\n", &error));
    CHECK(!rh::fmt::ReadYamlBlock(path, "T_BS", &block, &error));
}

RH_TEST(euroc_format_rejects_malformed_lists) {
    const rh::testing::TempDirectory temp("rh-yaml-broken");
    const std::string path = rh::fmt::JoinPath(temp.path(), "sensor.yaml");
    std::string error;
    const std::vector<std::string> bad = {
        "intrinsics: []\n",
        "intrinsics: [1, 2, 3,]\n",
        "intrinsics: [1, 2, , 4]\n",
        "intrinsics: [1, 2, 3, 4] junk\n",
        "intrinsics: 1, 2, 3, 4\n",
    };
    for (const auto& line : bad) {
        CHECK(rh::fmt::WriteWholeFile(path, "sensor_type: camera\n" + line, &error));
        rh::fmt::YamlBlock block;
        CHECK(!rh::fmt::ReadYamlBlock(path, "intrinsics", &block, &error));
        error.clear();
    }
    // 5 值列表本身合法（k3 由上层判 0），但 rows/cols 与元素数不匹配必须失败。
    CHECK(
        rh::fmt::WriteWholeFile(path, "T_BS:\n  cols: 4\n  rows: 4\n  data: [1, 2, 3]\n", &error));
    rh::fmt::YamlBlock block;
    CHECK(!rh::fmt::ReadYamlBlock(path, "T_BS", &block, &error));
}

RH_TEST(euroc_format_paths) {
    const rh::testing::TempDirectory temp("rh-paths");
    const std::string deep = rh::fmt::JoinPath(rh::fmt::JoinPath(temp.path(), "mav0"), "cam0/data");
    std::string error;
    CHECK(rh::fmt::MakeDirectoryTree(deep, &error));
    CHECK(rh::fmt::WriteWholeFile(rh::fmt::JoinPath(deep, "x.csv"), "a,b\n", &error));
    std::string content;
    CHECK(rh::fmt::ReadWholeFile(rh::fmt::JoinPath(deep, "x.csv"), &content, &error));
    CHECK(content == "a,b\n");
    CHECK(!rh::fmt::ReadWholeFile(rh::fmt::JoinPath(deep, "missing.csv"), &content, &error));
    CHECK(rh::fmt::JoinPath("/tmp/", "a") == "/tmp/a");
    CHECK(rh::fmt::JoinPath("/tmp", "") == "/tmp");
}

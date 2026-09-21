#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace rh::fmt {

// 本模块是 unav_vio 回放解析器的对偶实现：写出的每个 token 都必须能被
// tests/integration/euroc_dataset.cpp 的 read_yaml_block / read_csv_rows / parse_number 接受，
// 校验器也复用同一套读取语义，因此「rh verify 通过」等价于「回放侧能解析」。

[[nodiscard]] std::string Trim(std::string_view text);
// 与 yaml_value() 一致：'#' 之后一律视为注释，然后去空白。
[[nodiscard]] std::string StripComment(std::string_view line);

// from_chars 语义：必须消费整个 token，拒绝 NaN/Inf/十六进制/尾随垃圾；允许单个前导 '+'。
[[nodiscard]] bool ParseDouble(std::string_view text, double* value);
[[nodiscard]] bool ParseInt64(std::string_view text, std::int64_t* value);

// 最短可往返且保证 from_chars 可解析的十进制写法；非有限数返回空串。
[[nodiscard]] std::string FormatDouble(double value);
[[nodiscard]] std::string FormatFlowList(const std::vector<double>& values);
[[nodiscard]] std::string FormatIntFlowList(const std::vector<std::int64_t>& values);

// EuRoC 的矩阵字段：cols/rows/data 三行，data 用单行 flow 列表，避免跨行续接带来的歧义。
[[nodiscard]] std::string
FormatMatrixField(const std::string& key, int rows, int cols, const std::vector<double>& data);

struct YamlBlock {
    std::vector<double> data;
    int rows{0};
    int cols{0};
};

// 接受三种形态：`key: [a, b, c]`、`key:` + 缩进的 cols/rows/data 块，以及 YAML block sequence
// （`- 1.0` 逐行，元素可与 key 同缩进——PyYAML 默认就这么写）。最后这种是 2026-09-21 跟着上游
// read_yaml_block 一起放开的：录制端仍然只写 flow 列表，放开只是为了让「verify 通过」继续等价于
// 「上游能解析」，不把第三方导出包误判成不可用。
[[nodiscard]] bool
ReadYamlBlock(const std::string& path, const std::string& key, YamlBlock* out, std::string* error);
[[nodiscard]] bool
ReadYamlScalar(const std::string& path, const std::string& key, double* out, std::string* error);

// 与 read_csv_rows 一致：跳过空行与 '#' 开头的表头，逗号切分并逐字段去空白。
[[nodiscard]] std::vector<std::vector<std::string>> SplitCsvRows(const std::string& content);

[[nodiscard]] bool ReadWholeFile(const std::string& path, std::string* out, std::string* error);
[[nodiscard]] bool
WriteWholeFile(const std::string& path, const std::string& content, std::string* error);
[[nodiscard]] bool MakeDirectoryTree(const std::string& path, std::string* error);
[[nodiscard]] std::string JoinPath(const std::string& left, const std::string& right);

} // namespace rh::fmt

#include "record_helper/euroc_format.hpp"

#include <cerrno>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <sstream>
#include <system_error>

#include <sys/stat.h>
#include <unistd.h>

namespace rh::fmt {
namespace {

constexpr const char* kWhitespace = " \t\r\n";

bool EndsWith(const std::string& text, const std::string& suffix) {
    return text.size() >= suffix.size() &&
           text.compare(text.size() - suffix.size(), suffix.size(), suffix) == 0;
}

} // namespace

std::string Trim(std::string_view text) {
    const std::size_t begin = text.find_first_not_of(kWhitespace);
    if (begin == std::string_view::npos) {
        return {};
    }
    const std::size_t end = text.find_last_not_of(kWhitespace);
    return std::string(text.substr(begin, end - begin + 1));
}

std::string StripComment(std::string_view line) {
    const std::size_t hash = line.find('#');
    if (hash != std::string_view::npos) {
        line = line.substr(0, hash);
    }
    return Trim(line);
}

bool ParseDouble(std::string_view text, double* value) {
    std::string token(Trim(text));
    if (!token.empty() && token.front() == '+') {
        token.erase(token.begin());
        if (token.empty() || token.front() == '-' || token.front() == '+') {
            return false;
        }
    }
    if (token.empty() || value == nullptr) {
        return false;
    }
    const auto parsed = std::from_chars(token.data(), token.data() + token.size(), *value);
    if (parsed.ec != std::errc{} || parsed.ptr != token.data() + token.size()) {
        return false;
    }
    return std::isfinite(*value);
}

bool ParseInt64(std::string_view text, std::int64_t* value) {
    std::string token(Trim(text));
    if (!token.empty() && token.front() == '+') {
        token.erase(token.begin());
        if (token.empty() || token.front() == '-' || token.front() == '+') {
            return false;
        }
    }
    if (token.empty() || value == nullptr) {
        return false;
    }
    const auto parsed = std::from_chars(token.data(), token.data() + token.size(), *value);
    return parsed.ec == std::errc{} && parsed.ptr == token.data() + token.size();
}

std::string FormatDouble(double value) {
    if (!std::isfinite(value)) {
        return {};
    }
    char buffer[64];
    for (int precision = 6; precision <= 17; ++precision) {
        std::snprintf(buffer, sizeof(buffer), "%.*g", precision, value);
        double round_trip = 0.0;
        if (ParseDouble(buffer, &round_trip) && round_trip == value) {
            return buffer;
        }
    }
    std::snprintf(buffer, sizeof(buffer), "%.17g", value);
    return buffer;
}

std::string FormatFlowList(const std::vector<double>& values) {
    std::string out = "[";
    for (std::size_t index = 0; index < values.size(); ++index) {
        if (index != 0) {
            out += ", ";
        }
        out += FormatDouble(values[index]);
    }
    out += "]";
    return out;
}

std::string FormatIntFlowList(const std::vector<std::int64_t>& values) {
    std::string out = "[";
    for (std::size_t index = 0; index < values.size(); ++index) {
        if (index != 0) {
            out += ", ";
        }
        out += std::to_string(values[index]);
    }
    out += "]";
    return out;
}

std::string
FormatMatrixField(const std::string& key, int rows, int cols, const std::vector<double>& data) {
    std::ostringstream stream;
    stream << key << ":\n"
           << "  cols: " << cols << "\n"
           << "  rows: " << rows << "\n"
           << "  data: " << FormatFlowList(data) << "\n";
    return stream.str();
}

bool ParseNumberList(std::string_view text, std::vector<double>* values) {
    const std::string storage = Trim(text);
    std::string_view list = storage;
    if (list.size() < 2 || list.front() != '[' || list.back() != ']') {
        return false;
    }
    list.remove_prefix(1);
    list.remove_suffix(1);
    std::vector<double> parsed;
    std::size_t begin = 0;
    while (begin <= list.size()) {
        const std::size_t comma = list.find(',', begin);
        const std::string_view token = list.substr(
            begin, comma == std::string_view::npos ? std::string_view::npos : comma - begin);
        // 空元素（含尾随逗号造成的末尾空 token）视为破损输入，和上游解析器一致。
        if (Trim(token).empty()) {
            return false;
        }
        double value = 0.0;
        if (!ParseDouble(token, &value)) {
            return false;
        }
        parsed.push_back(value);
        if (comma == std::string_view::npos) {
            break;
        }
        begin = comma + 1;
    }
    if (parsed.empty()) {
        return false;
    }
    if (values != nullptr) {
        *values = std::move(parsed);
    }
    return true;
}

bool ReadYamlBlock(const std::string& path,
                   const std::string& key,
                   YamlBlock* out,
                   std::string* error) {
    auto fail = [&](const std::string& reason) {
        if (error != nullptr) {
            *error = path + " key " + key + ": " + reason;
        }
        return false;
    };
    std::string content;
    if (!ReadWholeFile(path, &content, error)) {
        return false;
    }
    const std::string top_key = key + ":";
    std::istringstream lines(content);
    std::string line;
    while (std::getline(lines, line)) {
        const std::string trimmed = StripComment(line);
        if (trimmed.rfind(top_key, 0) != 0) {
            continue;
        }
        const std::size_t indent = line.find_first_not_of(" \t");
        const std::string inline_part = Trim(trimmed.substr(top_key.size()));
        YamlBlock block;
        if (!inline_part.empty()) {
            if (!ParseNumberList(inline_part, &block.data)) {
                return fail("malformed inline list");
            }
            block.cols = static_cast<int>(block.data.size());
            block.rows = 1;
            if (out != nullptr) {
                *out = block;
            }
            return true;
        }
        bool have_data = false;
        while (std::getline(lines, line)) {
            const std::string field = StripComment(line);
            if (field.empty()) {
                continue;
            }
            if (line.find_first_not_of(" \t") <= indent) {
                break;
            }
            auto parse_shape = [&](const std::string& token, int* target, const char* field_name) {
                std::int64_t parsed = 0;
                if (!ParseInt64(token, &parsed) || parsed < 0 || parsed > 1'000'000) {
                    return fail(std::string("malformed ") + field_name);
                }
                *target = static_cast<int>(parsed);
                return true;
            };
            if (field.rfind("cols:", 0) == 0) {
                if (!parse_shape(field.substr(5), &block.cols, "cols")) {
                    return false;
                }
            } else if (field.rfind("rows:", 0) == 0) {
                if (!parse_shape(field.substr(5), &block.rows, "rows")) {
                    return false;
                }
            } else if (field.rfind("data:", 0) == 0) {
                if (have_data) {
                    return fail("duplicate data");
                }
                std::string data_line(field.substr(5));
                while (data_line.find(']') == std::string::npos && std::getline(lines, line)) {
                    if (!StripComment(line).empty() && line.find_first_not_of(" \t") <= indent) {
                        return fail("unterminated data list");
                    }
                    data_line += " " + StripComment(line);
                }
                if (!ParseNumberList(data_line, &block.data)) {
                    return fail("malformed numeric data");
                }
                have_data = true;
            } else {
                return fail("unknown matrix field");
            }
        }
        if (!have_data || block.cols <= 0 || block.rows <= 0 ||
            static_cast<std::uint64_t>(block.cols) * static_cast<std::uint64_t>(block.rows) !=
                block.data.size()) {
            return fail("missing data or mismatched rows/cols");
        }
        if (out != nullptr) {
            *out = block;
        }
        return true;
    }
    return fail("key not found");
}

bool ReadYamlScalar(const std::string& path,
                    const std::string& key,
                    double* out,
                    std::string* error) {
    std::string content;
    if (!ReadWholeFile(path, &content, error)) {
        return false;
    }
    const std::string top_key = key + ":";
    std::istringstream lines(content);
    std::string line;
    while (std::getline(lines, line)) {
        const std::string trimmed = StripComment(line);
        if (trimmed.rfind(top_key, 0) != 0) {
            continue;
        }
        const std::string value = Trim(trimmed.substr(top_key.size()));
        double parsed = 0.0;
        if (!ParseDouble(value, &parsed)) {
            if (error != nullptr) {
                *error = path + " key " + key + ": malformed scalar";
            }
            return false;
        }
        if (out != nullptr) {
            *out = parsed;
        }
        return true;
    }
    if (error != nullptr) {
        *error = path + " key " + key + ": key not found";
    }
    return false;
}

std::vector<std::vector<std::string>> SplitCsvRows(const std::string& content) {
    std::vector<std::vector<std::string>> rows;
    std::istringstream lines(content);
    std::string line;
    while (std::getline(lines, line)) {
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        const std::string trimmed = Trim(line);
        if (trimmed.empty() || trimmed.front() == '#') {
            continue;
        }
        std::vector<std::string> fields;
        std::istringstream row(line);
        std::string field;
        while (std::getline(row, field, ',')) {
            fields.emplace_back(Trim(field));
        }
        if (!line.empty() && line.back() == ',') {
            fields.emplace_back();
        }
        if (!fields.empty()) {
            rows.push_back(std::move(fields));
        }
    }
    return rows;
}

bool ReadWholeFile(const std::string& path, std::string* out, std::string* error) {
    std::FILE* handle = std::fopen(path.c_str(), "rb");
    if (handle == nullptr) {
        if (error != nullptr) {
            *error = "无法读取文件: " + path;
        }
        return false;
    }
    std::string content;
    char buffer[8192];
    std::size_t read = 0;
    while ((read = std::fread(buffer, 1, sizeof(buffer), handle)) > 0) {
        content.append(buffer, read);
    }
    const bool failed = std::ferror(handle) != 0;
    std::fclose(handle);
    if (failed) {
        if (error != nullptr) {
            *error = "读取中断: " + path;
        }
        return false;
    }
    if (out != nullptr) {
        *out = std::move(content);
    }
    return true;
}

bool WriteWholeFile(const std::string& path, const std::string& content, std::string* error) {
    std::FILE* handle = std::fopen(path.c_str(), "wb");
    if (handle == nullptr) {
        if (error != nullptr) {
            *error = "无法写入文件: " + path;
        }
        return false;
    }
    const std::size_t written = std::fwrite(content.data(), 1, content.size(), handle);
    const int flush = std::fflush(handle);
    const int close = std::fclose(handle);
    if (written != content.size() || flush != 0 || close != 0) {
        if (error != nullptr) {
            *error = "写入不完整: " + path;
        }
        return false;
    }
    return true;
}

bool MakeDirectoryTree(const std::string& path, std::string* error) {
    if (path.empty()) {
        return true;
    }
    std::string built;
    std::size_t cursor = 0;
    if (path.front() == '/') {
        built = "/";
        cursor = 1;
    }
    while (cursor <= path.size()) {
        const std::size_t slash = path.find('/', cursor);
        const std::size_t stop = slash == std::string::npos ? path.size() : slash;
        const std::string piece = path.substr(cursor, stop - cursor);
        if (!piece.empty()) {
            if (built.empty()) {
                built = piece;
            } else if (built.back() == '/') {
                built += piece;
            } else {
                built += "/" + piece;
            }
            if (mkdir(built.c_str(), 0755) != 0 && errno != EEXIST) {
                if (error != nullptr) {
                    *error = "无法创建目录: " + built;
                }
                return false;
            }
        }
        if (slash == std::string::npos) {
            break;
        }
        cursor = slash + 1;
    }
    return true;
}

std::string JoinPath(const std::string& left, const std::string& right) {
    if (left.empty()) {
        return right;
    }
    if (right.empty()) {
        return left;
    }
    if (left.back() == '/' || EndsWith(left, "/")) {
        return left + right;
    }
    return left + "/" + right;
}

} // namespace rh::fmt

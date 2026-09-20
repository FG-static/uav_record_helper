#pragma once

#include <chrono>
#include <csignal>
#include <cstdint>
#include <string>
#include <vector>

namespace rh::cli {

struct Flag {
    std::string name;
    std::string value;
    bool present{false};
};

// 极简命令行解析：--key value / --key=value / --flag。值统一按字符串取，类型转换在调用处做，
// 这样非法输入能给出「哪个选项不对」的明确错误，而不是 std::stoi 抛异常。
class ArgumentParser {
  public:
    ArgumentParser(int argc, char** argv, std::string usage);

    bool Parse();
    // 每个子命令显式声明自己的合法选项，拼错的选项必须立刻失败而不是被忽略。
    bool RejectUnknown(const std::vector<std::string>& known, std::string* error) const;
    bool Has(const std::string& name) const;
    std::string Value(const std::string& name, const std::string& fallback = {}) const;
    bool Bool(const std::string& name, bool fallback) const;
    int Integer(const std::string& name, int fallback, std::string* error) const;
    double Number(const std::string& name, double fallback, std::string* error) const;
    const std::vector<std::string>& Positional() const { return positional_; }
    const std::string& usage() const { return usage_; }

  private:
    std::vector<std::string> args_;
    std::vector<std::string> positional_;
    std::vector<Flag> flags_;
    std::string usage_;
};

// 终端非阻塞单键读取：录制引导靠它推进阶段，不需要 GUI。
class KeyReader {
  public:
    KeyReader();
    ~KeyReader();
    bool Poll(char* key);
    void Restore();

  private:
    bool enabled_{false};
    int fd_{-1};
    struct Impl;
    Impl* impl_{nullptr};
};

void InstallSignalHandlers();

double MonotonicSeconds();
std::string FormatBytes(std::uint64_t bytes);

extern volatile std::sig_atomic_t interrupted;

} // namespace rh::cli

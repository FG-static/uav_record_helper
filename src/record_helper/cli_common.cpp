#include "cli_common.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <termios.h>
#include <unistd.h>

namespace rh::cli {
namespace {

bool ValueToken(const std::string& token) {
    return !token.empty() && token[0] != '-';
}

bool IsNumeric(const std::string& token) {
    for (const char character : token) {
        if (std::isdigit(static_cast<unsigned char>(character)) == 0 && character != '.' &&
            character != '-') {
            return false;
        }
    }
    return true;
}

void OnInterrupt(int) {
    interrupted = 1;
}

} // namespace

volatile std::sig_atomic_t interrupted = 0;

ArgumentParser::ArgumentParser(int argc, char** argv, std::string usage)
    : usage_(std::move(usage)) {
    for (int index = 1; index < argc; ++index) {
        args_.emplace_back(argv[index]);
    }
}

bool ArgumentParser::Parse(const std::vector<std::string>& string_valued) {
    for (std::size_t index = 0; index < args_.size(); ++index) {
        const std::string& token = args_[index];
        if (token.size() > 2 && token[0] == '-' && token[1] == '-') {
            std::string name = token.substr(2);
            std::string value;
            const std::size_t equals = name.find('=');
            if (equals != std::string::npos) {
                value = name.substr(equals + 1);
                name = name.substr(0, equals);
            } else if (index + 1 < args_.size() && ValueToken(args_[index + 1])) {
                // 只认数字取值是不够的：--seq room_04 / --imu-grid accel / --pose-gt off
                // 会被退回默认值，而那个词变成游离的位置参数，选项等于被静默忽略。
                const bool wants_value =
                    std::find(string_valued.begin(), string_valued.end(), name) !=
                    string_valued.end();
                if (wants_value || IsNumeric(args_[index + 1])) {
                    value = args_[++index];
                }
            }
            flags_.push_back(Flag{name, value, true});
            continue;
        }
        positional_.push_back(token);
    }
    return true;
}

bool ArgumentParser::RejectUnknown(const std::vector<std::string>& known,
                                   std::string* error) const {
    for (const auto& flag : flags_) {
        bool ok = false;
        for (const auto& name : known) {
            ok = ok || name == flag.name;
        }
        if (!ok) {
            if (error != nullptr) {
                *error = "不认识的选项 --" + flag.name;
            }
            return false;
        }
    }
    return true;
}

bool ArgumentParser::Has(const std::string& name) const {
    for (const auto& flag : flags_) {
        if (flag.name == name) {
            return true;
        }
    }
    return false;
}

std::string ArgumentParser::Value(const std::string& name, const std::string& fallback) const {
    for (const auto& flag : flags_) {
        if (flag.name == name) {
            return flag.value.empty() ? (fallback.empty() ? "1" : fallback) : flag.value;
        }
    }
    return fallback;
}

bool ArgumentParser::Bool(const std::string& name, bool fallback) const {
    const std::string value = Value(name);
    if (value.empty()) {
        return fallback;
    }
    return value == "1" || value == "true" || value == "on" || value == "yes";
}

int ArgumentParser::Integer(const std::string& name, int fallback, std::string* error) const {
    const std::string value = Value(name);
    if (value.empty()) {
        return fallback;
    }
    char* end = nullptr;
    const long parsed = std::strtol(value.c_str(), &end, 10);
    if (end == value.c_str() || *end != '\0') {
        if (error != nullptr) {
            *error = "--" + name + " 需要整数，实际是 " + value;
        }
        return fallback;
    }
    return static_cast<int>(parsed);
}

double ArgumentParser::Number(const std::string& name, double fallback, std::string* error) const {
    const std::string value = Value(name);
    if (value.empty()) {
        return fallback;
    }
    char* end = nullptr;
    const double parsed = std::strtod(value.c_str(), &end);
    if (end == value.c_str() || *end != '\0') {
        if (error != nullptr) {
            *error = "--" + name + " 需要数字，实际是 " + value;
        }
        return fallback;
    }
    return parsed;
}

struct KeyReader::Impl {
    struct termios original{};
};

KeyReader::KeyReader() : impl_(new Impl()) {
    fd_ = ::fileno(stdin);
    if (!::isatty(fd_)) {
        return;
    }
    if (tcgetattr(fd_, &impl_->original) != 0) {
        return;
    }
    struct termios raw = impl_->original;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(fd_, TCSANOW, &raw) == 0) {
        enabled_ = true;
    }
}

KeyReader::~KeyReader() {
    Restore();
}

void KeyReader::Restore() {
    if (enabled_) {
        tcsetattr(fd_, TCSANOW, &impl_->original);
        enabled_ = false;
    }
    delete impl_;
    impl_ = nullptr;
}

bool KeyReader::Poll(char* key) {
    if (!enabled_) {
        return false;
    }
    unsigned char byte = 0;
    const ssize_t read = ::read(fd_, &byte, 1);
    if (read != 1) {
        return false;
    }
    *key = static_cast<char>(byte);
    return true;
}

double MonotonicSeconds() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return std::chrono::duration<double>(now).count();
}

std::string FormatBytes(std::uint64_t bytes) {
    double value = static_cast<double>(bytes);
    const char* units[] = {"B", "KiB", "MiB", "GiB"};
    int unit = 0;
    while (value >= 1024.0 && unit < 3) {
        value /= 1024.0;
        ++unit;
    }
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%.1f %s", value, units[unit]);
    return buffer;
}

namespace {
struct InterruptInstaller {
    InterruptInstaller() {
        struct sigaction action{};
        action.sa_handler = OnInterrupt;
        sigemptyset(&action.sa_mask);
        action.sa_flags = 0;
        sigaction(SIGINT, &action, nullptr);
        sigaction(SIGTERM, &action, nullptr);
    }
};
} // namespace

void InstallSignalHandlers() {
    static InterruptInstaller installer;
}

} // namespace rh::cli

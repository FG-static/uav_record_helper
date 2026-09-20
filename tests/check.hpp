#pragma once

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include <unistd.h>

namespace rh::testing {

using TestFn = void (*)();

void Register(const char* name, TestFn fn);
int RunAll();
void Expect(bool ok, const char* expression, const char* file, int line);
void ExpectNear(double actual,
                double expected,
                double tolerance,
                const char* expression,
                const char* file,
                int line);

struct Registrar {
    Registrar(const char* name, TestFn function) { Register(name, function); }
};

// mkdtemp 隔离的临时目录，析构时删除；所有落盘用例都靠它避免污染仓库。
class TempDirectory {
  public:
    explicit TempDirectory(const std::string& prefix = "rh-test") {
        char pattern[256];
        std::snprintf(pattern, sizeof(pattern), "/tmp/%s.XXXXXX", prefix.c_str());
        const char* created = mkdtemp(pattern);
        if (created != nullptr) {
            path_ = created;
        }
    }
    ~TempDirectory() {
        if (path_.empty()) {
            return;
        }
        const std::string command = "rm -rf '" + path_ + "'";
        if (std::system(command.c_str()) != 0) {
            std::printf("  警告: 临时目录清理失败 %s\n", path_.c_str());
        }
    }
    TempDirectory(const TempDirectory&) = delete;
    TempDirectory& operator=(const TempDirectory&) = delete;
    const std::string& path() const { return path_; }
    [[nodiscard]] bool valid() const { return !path_.empty(); }

  private:
    std::string path_;
};

} // namespace rh::testing

#define RH_TEST(name)                                                                              \
    static void name();                                                                            \
    static ::rh::testing::Registrar registrar_##name(#name, &name);                                \
    static void name()

#define CHECK(condition) ::rh::testing::Expect((condition), #condition, __FILE__, __LINE__)
#define CHECK_NEAR(actual, expected, tolerance)                                                    \
    ::rh::testing::ExpectNear((actual), (expected), (tolerance), #actual, __FILE__, __LINE__)

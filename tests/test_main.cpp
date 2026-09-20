#include "check.hpp"

#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace rh::testing {
namespace {

struct Entry {
    const char* name;
    TestFn function;
};

std::vector<Entry>& Registry() {
    static std::vector<Entry> registry;
    return registry;
}

int failures = 0;
int checks = 0;
const char* current = "";

} // namespace

void Register(const char* name, TestFn function) {
    Registry().push_back(Entry{name, function});
}

void Expect(bool ok, const char* expression, const char* file, int line) {
    ++checks;
    if (ok) {
        return;
    }
    ++failures;
    std::printf("  FAIL %s: %s (%s:%d)\n", current, expression, file, line);
}

void ExpectNear(double actual,
                double expected,
                double tolerance,
                const char* expression,
                const char* file,
                int line) {
    ++checks;
    if (std::isfinite(actual) && std::abs(actual - expected) <= tolerance) {
        return;
    }
    ++failures;
    std::printf("  FAIL %s: %s 期望 %g 实际 %g (%s:%d)\n",
                current,
                expression,
                expected,
                actual,
                file,
                line);
}

int RunAll() {
    int local_failures = 0;
    for (const auto& entry : Registry()) {
        current = entry.name;
        const int before = failures;
        entry.function();
        const bool ok = failures == before;
        std::printf("%s %s\n", ok ? "[ ok ]" : "[FAIL]", entry.name);
        if (!ok) {
            ++local_failures;
        }
    }
    std::printf("共 %d 个用例，%d 个断言，失败 %d 个\n",
                static_cast<int>(Registry().size()),
                checks,
                failures);
    return local_failures == 0 ? 0 : 1;
}

} // namespace rh::testing

namespace {
int main_impl() {
    return rh::testing::RunAll();
}
} // namespace

int main() {
    return main_impl();
}

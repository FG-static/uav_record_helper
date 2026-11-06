#include "csi_dispatch.hpp"
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <unistd.h>
#include <vector>

namespace rh::cli {
std::optional<int> DispatchCsi(int argc, char** argv) {
    const std::string command = argc > 1 ? argv[1] : "";
    bool csi = command == "export-csi";
    if (command == "record" || command == "probe") {
        for (int i = 2; i < argc; ++i) {
            const std::string token = argv[i];
            if (token == "--backend=csi-mavlink" || (token == "--backend" && i + 1 < argc &&
                                                     std::string(argv[i + 1]) == "csi-mavlink")) {
                csi = true;
            }
        }
    }
    if (!csi)
        return std::nullopt;
    const char* python = std::getenv("RH_CSI_PYTHON");
    if (!python || !*python)
        python = "python3";
    std::vector<char*> args{const_cast<char*>(python), const_cast<char*>(RH_CSI_HELPER_PATH)};
    for (int i = 1; i < argc; ++i)
        args.push_back(argv[i]);
    args.push_back(nullptr);
    // 直接传 argv，路径和选项不经过 shell。
    execvp(python, args.data());
    std::fprintf(stderr, "CSI Python 启动失败: %s\n", std::strerror(errno));
    return 1;
}
} // namespace rh::cli

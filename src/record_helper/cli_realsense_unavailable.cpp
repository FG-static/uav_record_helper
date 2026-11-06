#include <cstdio>
namespace rh::cli {
namespace {
int Unavailable() {
    std::fprintf(stderr,
                 "本构建未启用 RealSense；CSI 使用 --backend csi-mavlink --config FILE；"
                 "RealSense 请以 -DRH_WITH_REALSENSE=ON 重建。\n");
    return 2;
}
} // namespace
int RunRecord(int, char**) {
    return Unavailable();
}
int RunProbe(int, char**) {
    return Unavailable();
}
} // namespace rh::cli

#pragma once
#include <optional>
namespace rh::cli {
// CSI 使用独立 Python/GStreamer 后端；RealSense 仍走原生录制流程。
std::optional<int> DispatchCsi(int argc, char** argv);
} // namespace rh::cli

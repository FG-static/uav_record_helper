#pragma once

#include <string>

namespace rh::macos {

// USB 相机接口受 TCC 摄像头权限管控。librealsense/libusb 直接 claim 接口时，
// macOS 26 经常直接返回 ACCESS、不弹窗。必须先走 AVFoundation 申请，并泵一下
// 主线程 runloop，系统弹窗才出得来。
bool EnsureCameraAccess(std::string* error);

} // namespace rh::macos

#include "macos_privacy.hpp"

#import <AVFoundation/AVFoundation.h>
#import <AppKit/AppKit.h>
#import <Foundation/Foundation.h>

#include <libproc.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>
#include <string>

namespace rh::macos {
namespace {

std::string ProcessName(pid_t pid) {
    char path[PROC_PIDPATHINFO_MAXSIZE] = {0};
    if (proc_pidpath(pid, path, sizeof(path)) <= 0) {
        return "?";
    }
    const char* slash = std::strrchr(path, '/');
    return slash == nullptr ? path : slash + 1;
}

std::string HostChain() {
    std::string chain = ProcessName(getpid());
    pid_t pid = getppid();
    for (int depth = 0; depth < 4 && pid > 1; ++depth) {
        chain += " ← ";
        chain += ProcessName(pid);
        struct proc_bsdinfo info {};
        if (proc_pidinfo(pid, PROC_PIDTBSDINFO, 0, &info, sizeof(info)) <= 0) {
            break;
        }
        pid = static_cast<pid_t>(info.pbi_ppid);
    }
    return chain;
}

bool LooksLikeIdeHelper(const std::string& chain) {
    return chain.find("Cursor") != std::string::npos || chain.find("Code Helper") != std::string::npos ||
           chain.find("Electron") != std::string::npos;
}

std::string DeniedMessage() {
    std::string text =
        "摄像头权限未授予。D435i 的 USB 接口被算进「摄像头」隐私项；当前进程链：\n  ";
    text += HostChain();
    text +=
        "\n\n从 Cursor / IDE 内置终端申请时，弹窗经常记到 Helper 上并被吞掉。"
        "\n请用系统自带的「终端.app」（不要用 Cursor 内置终端）跑："
        "\n  open -a Terminal \"$PWD\""
        "\n  ./build-release/rh probe"
        "\n第一次会问「终端」要不要用摄像头，点允许。"
        "\n\n若曾经点过不允许，或列表里没有终端："
        "\n  tccutil reset Camera"
        "\n  系统设置 → 隐私与安全性 → 摄像头"
        "\n  open \"x-apple.systemsettings:com.apple.settings.PrivacySecurity.extension?Privacy_Camera\"";
    return text;
}

void PumpUntil(dispatch_semaphore_t sem, NSTimeInterval seconds) {
    const NSDate* deadline = [NSDate dateWithTimeIntervalSinceNow:seconds];
    while (dispatch_semaphore_wait(sem, DISPATCH_TIME_NOW) != 0) {
        if ([deadline timeIntervalSinceNow] < 0.0) {
            break;
        }
        [[NSRunLoop currentRunLoop] runMode:NSDefaultRunLoopMode
                                 beforeDate:[NSDate dateWithTimeIntervalSinceNow:0.05]];
    }
}

} // namespace

bool EnsureCameraAccess(std::string* error) {
    @autoreleasepool {
        const AVAuthorizationStatus status =
            [AVCaptureDevice authorizationStatusForMediaType:AVMediaTypeVideo];
        if (status == AVAuthorizationStatusAuthorized) {
            return true;
        }
        if (status == AVAuthorizationStatusDenied || status == AVAuthorizationStatusRestricted) {
            if (error != nullptr) {
                *error = DeniedMessage();
            }
            return false;
        }

        [NSApplication sharedApplication];
        [NSApp setActivationPolicy:NSApplicationActivationPolicyAccessory];
        [NSApp activateIgnoringOtherApps:YES];

        std::fprintf(stderr, "正在请求摄像头权限，请看屏幕中央的系统弹窗（最多等 90 秒）…\n");
        if (LooksLikeIdeHelper(HostChain())) {
            std::fprintf(stderr,
                         "提示：当前是从 IDE 终端启动的，弹窗可能被吞。"
                         "若 几秒内没有窗口，改用「终端.app」跑同一条命令。\n");
        }

        dispatch_semaphore_t sem = dispatch_semaphore_create(0);
        __block BOOL granted = NO;
        [AVCaptureDevice requestAccessForMediaType:AVMediaTypeVideo
                                 completionHandler:^(BOOL ok) {
                                     granted = ok;
                                     dispatch_semaphore_signal(sem);
                                 }];
        PumpUntil(sem, 90.0);

        if (granted) {
            return true;
        }
        if (error != nullptr) {
            *error = DeniedMessage();
        }
        return false;
    }
}

} // namespace rh::macos

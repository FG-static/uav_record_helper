cmake_minimum_required(VERSION 3.22)

project(RecordHelper LANGUAGES CXX VERSION 0.1.0)

# RecordHelper：D435i 一键录制，输出 unav_vio 回放侧可直接消费的 EuRoC 风格数据集。
# 依赖只有 librealsense（设备访问）、Eigen（刚体/四元数数学，与 unav_vio 同族）和 zlib（PNG）。
# 不引入 ROS、OpenCV、Ceres：录制端不做估计，也不做图像算法。

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

if(APPLE)
    enable_language(OBJCXX)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(Eigen3 CONFIG REQUIRED)
find_package(ZLIB REQUIRED)
find_package(Threads REQUIRED)
find_package(realsense2 CONFIG REQUIRED)

add_library(record_helper_format STATIC
    src/record_helper/png_io.cpp
    src/record_helper/euroc_format.cpp
    src/record_helper/calibration.cpp
    src/record_helper/imu_align.cpp
    src/record_helper/motion_gate.cpp
    src/record_helper/dataset_writer.cpp
    src/record_helper/verify.cpp)
target_include_directories(record_helper_format PUBLIC include)
target_link_libraries(record_helper_format PUBLIC Eigen3::Eigen ZLIB::ZLIB)

# 设备访问单独成层：无 SDK/无权限的环境下也能构建 format 层与校验器。
add_library(record_helper_capture STATIC
    src/record_helper/capture.cpp)
target_include_directories(record_helper_capture PUBLIC include)
target_link_libraries(record_helper_capture PUBLIC record_helper_format realsense2::realsense2 Threads::Threads)

add_executable(rh
    src/record_helper/main.cpp
    src/record_helper/cli_common.cpp
    src/record_helper/cli_probe.cpp
    src/record_helper/cli_record.cpp)
target_link_libraries(rh PRIVATE record_helper_capture)
set_target_properties(rh PROPERTIES OUTPUT_NAME "rh")
# Linux 就是这个 rh。macOS 下面再包成 .app。录制请用 Linux：当前 SDK 在 macOS 上不枚举 IMU。

# macOS 把 USB 相机接口纳入 TCC 摄像头授权：从终端直跑时授权归属会落到父 App，
# 弹窗可能被静默抑制。把可执行文件本身做成 .app bundle（自带 NSCameraUsageDescription），
# 启动时走 AVFoundation 申请权限并泵 runloop；再 adhoc 签名把 Info.plist 绑进 bundle。
if(APPLE)
    target_sources(rh PRIVATE src/record_helper/macos_privacy.mm)
    set_source_files_properties(src/record_helper/macos_privacy.mm PROPERTIES
        COMPILE_FLAGS "-fobjc-arc")
    target_link_libraries(rh PRIVATE "-framework AVFoundation" "-framework AppKit" "-framework Foundation")
    set_target_properties(rh PROPERTIES
        MACOSX_BUNDLE TRUE
        MACOSX_BUNDLE_INFO_PLIST "${CMAKE_CURRENT_SOURCE_DIR}/pack/Info.plist.in"
        MACOSX_BUNDLE_BUNDLE_NAME "RecordHelper"
        MACOSX_BUNDLE_GUI_IDENTIFIER "com.unav.recordhelper"
        MACOSX_BUNDLE_BUNDLE_VERSION "${PROJECT_VERSION}"
        OUTPUT_NAME "rh")
    add_custom_command(TARGET rh POST_BUILD
        COMMAND codesign --force --sign - --identifier com.unav.recordhelper --timestamp=none
                "$<TARGET_BUNDLE_DIR:rh>"
        COMMAND ${CMAKE_COMMAND} -E create_symlink
                "${CMAKE_CURRENT_BINARY_DIR}/rh.app/Contents/MacOS/rh"
                "${CMAKE_CURRENT_BINARY_DIR}/rh"
        COMMENT "签名 rh.app 并链接短路径 rh -> Contents/MacOS/rh")
endif()

enable_testing()

add_executable(rh_format_tests
    tests/test_png_io.cpp
    tests/test_euroc_format.cpp
    tests/test_calibration.cpp
    tests/test_imu_align.cpp
    tests/test_motion_gate.cpp
    tests/test_dataset_writer.cpp
    tests/test_verify.cpp
    tests/test_verify_room02.cpp
    tests/test_main.cpp)
target_include_directories(rh_format_tests PRIVATE tests)
target_link_libraries(rh_format_tests PRIVATE record_helper_format)
add_test(NAME format.contract COMMAND rh_format_tests)

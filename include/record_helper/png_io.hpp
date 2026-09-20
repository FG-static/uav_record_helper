#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace rh {

struct PngHeaderInfo {
    std::uint32_t width{0};
    std::uint32_t height{0};
    int bit_depth{0};
    int color_type{-1};
    int interlace{-1};
    bool terminated{false};
    std::size_t file_bytes{0};
};

// 写单通道 PNG（color type 0）：8-bit 给双目红外，16-bit 给深度（毫米，大端样本）。
// 只用 zlib deflate + 零滤波，不引入图像库；unav_vio 侧用 cv::imread(IMREAD_GRAYSCALE) 读取。
bool WritePngGray8(const std::string& path,
                   std::uint32_t width,
                   std::uint32_t height,
                   const std::uint8_t* pixels,
                   int compression_level,
                   std::string* error);

bool WritePngGray16(const std::string& path,
                    std::uint32_t width,
                    std::uint32_t height,
                    const std::uint16_t* samples,
                    int compression_level,
                    std::string* error);

// 只解析签名与 chunk 结构，不做像素解码：用于校验分辨率/位深/通道数和文件是否完整收尾。
bool InspectPng(const std::string& path, PngHeaderInfo* info, std::string* error);

} // namespace rh

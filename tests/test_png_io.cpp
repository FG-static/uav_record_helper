#include "check.hpp"
#include "record_helper/euroc_format.hpp"
#include "record_helper/png_io.hpp"

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include <zlib.h>

namespace {

// 自己解一遍 PNG：确认写出的确实是「zlib deflate + 每行 filter 0」的可解码数据，
// 而不是只把字节写进文件、要等 cv::imread 才发现是坏图。
bool DecodePng(const std::string& path, std::vector<std::uint8_t>* pixels, int* bit_depth) {
    std::FILE* handle = std::fopen(path.c_str(), "rb");
    if (handle == nullptr) {
        return false;
    }
    std::vector<std::uint8_t> bytes;
    unsigned char buffer[4096];
    std::size_t read = 0;
    while ((read = std::fread(buffer, 1, sizeof(buffer), handle)) > 0) {
        bytes.insert(bytes.end(), buffer, buffer + read);
    }
    std::fclose(handle);
    if (bytes.size() < 8) {
        return false;
    }
    std::vector<std::uint8_t> idat;
    std::size_t offset = 8;
    *bit_depth = 0;
    std::size_t width = 0;
    std::size_t height = 0;
    while (offset + 8 <= bytes.size()) {
        const auto length =
            static_cast<std::size_t>((bytes[offset] << 24) | (bytes[offset + 1] << 16) |
                                     (bytes[offset + 2] << 8) | bytes[offset + 3]);
        const std::string type(reinterpret_cast<const char*>(bytes.data() + offset + 4), 4);
        const std::size_t payload = offset + 8;
        if (type == "IHDR") {
            width = (static_cast<std::size_t>(bytes[payload]) << 24) |
                    (static_cast<std::size_t>(bytes[payload + 1]) << 16) |
                    (static_cast<std::size_t>(bytes[payload + 2]) << 8) |
                    static_cast<std::size_t>(bytes[payload + 3]);
            height = (static_cast<std::size_t>(bytes[payload + 4]) << 24) |
                     (static_cast<std::size_t>(bytes[payload + 5]) << 16) |
                     (static_cast<std::size_t>(bytes[payload + 6]) << 8) |
                     static_cast<std::size_t>(bytes[payload + 7]);
            *bit_depth = bytes[payload + 8];
        }
        if (type == "IDAT") {
            idat.insert(idat.end(), bytes.begin() + payload, bytes.begin() + payload + length);
        }
        if (type == "IEND") {
            break;
        }
        offset = payload + length + 4;
    }
    const std::size_t channels = *bit_depth == 16 ? 2 : 1;
    const std::size_t raw_size = height * (1 + width * channels);
    std::vector<std::uint8_t> raw(raw_size);
    uLongf destination = static_cast<uLongf>(raw.size());
    if (uncompress(raw.data(), &destination, idat.data(), static_cast<uLong>(idat.size())) !=
            Z_OK ||
        destination != raw.size()) {
        return false;
    }
    pixels->resize(width * height * channels);
    for (std::size_t row = 0; row < height; ++row) {
        if (raw[row * (1 + width * channels)] != 0) {
            return false; // 写出端只用 filter 0
        }
        std::memcpy(pixels->data() + row * width * channels,
                    raw.data() + row * (1 + width * channels) + 1,
                    width * channels);
    }
    return true;
}

} // namespace

RH_TEST(png_io_writes_decodable_gray8) {
    const int width = 37;
    const int height = 11;
    std::vector<std::uint8_t> pixels(static_cast<std::size_t>(width) * height);
    for (int row = 0; row < height; ++row) {
        for (int column = 0; column < width; ++column) {
            pixels[static_cast<std::size_t>(row) * width + column] =
                static_cast<std::uint8_t>((row * 31 + column * 7) & 0xff);
        }
    }
    const rh::testing::TempDirectory temp("rh-png");
    CHECK(temp.valid());
    const std::string path = rh::fmt::JoinPath(temp.path(), "a.png");
    std::string error;
    CHECK(rh::WritePngGray8(path, width, height, pixels.data(), 3, &error));
    rh::PngHeaderInfo info;
    CHECK(rh::InspectPng(path, &info, &error));
    CHECK(info.width == static_cast<std::uint32_t>(width));
    CHECK(info.height == static_cast<std::uint32_t>(height));
    CHECK(info.bit_depth == 8);
    CHECK(info.color_type == 0);
    CHECK(info.interlace == 0);
    CHECK(info.terminated);
    std::vector<std::uint8_t> decoded;
    int bit_depth = 0;
    CHECK(DecodePng(path, &decoded, &bit_depth));
    CHECK(bit_depth == 8);
    CHECK(decoded == pixels);
}

RH_TEST(png_io_writes_gray16_big_endian) {
    const int width = 9;
    const int height = 4;
    std::vector<std::uint16_t> samples(static_cast<std::size_t>(width) * height);
    for (std::size_t index = 0; index < samples.size(); ++index) {
        samples[index] = static_cast<std::uint16_t>(1000 + index * 37);
    }
    const rh::testing::TempDirectory temp("rh-png16");
    const std::string path = rh::fmt::JoinPath(temp.path(), "d.png");
    std::string error;
    CHECK(rh::WritePngGray16(path, width, height, samples.data(), 1, &error));
    rh::PngHeaderInfo info;
    CHECK(rh::InspectPng(path, &info, &error));
    CHECK(info.bit_depth == 16);
    CHECK(info.color_type == 0);
    std::vector<std::uint8_t> decoded;
    int bit_depth = 0;
    CHECK(DecodePng(path, &decoded, &bit_depth));
    for (std::size_t index = 0; index < samples.size(); ++index) {
        const int value = (decoded[index * 2] << 8) | decoded[index * 2 + 1];
        CHECK(value == static_cast<int>(samples[index]));
    }
}

RH_TEST(png_io_rejects_bad_input) {
    std::string error;
    CHECK(!rh::WritePngGray8("/tmp/rh-should-not-exist.png", 0, 4, nullptr, 1, &error));
    CHECK(!error.empty());
    error.clear();
    CHECK(!rh::InspectPng("/tmp/rh-definitely-missing.png", nullptr, &error));
}

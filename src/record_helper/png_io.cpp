#include "record_helper/png_io.hpp"

#include <cstdio>
#include <cstring>

#include <zlib.h>

namespace rh {
namespace {

constexpr std::uint8_t kSignature[8] = {0x89, 'P', 'N', 'G', 0x0d, 0x0a, 0x1a, 0x0a};

void AppendBE32(std::vector<std::uint8_t>* out, std::uint32_t value) {
    out->push_back(static_cast<std::uint8_t>(value >> 24));
    out->push_back(static_cast<std::uint8_t>(value >> 16));
    out->push_back(static_cast<std::uint8_t>(value >> 8));
    out->push_back(static_cast<std::uint8_t>(value));
}

void AppendChunk(std::vector<std::uint8_t>* out,
                 const char type[4],
                 const std::vector<std::uint8_t>& payload) {
    AppendBE32(out, static_cast<std::uint32_t>(payload.size()));
    const std::size_t type_begin = out->size();
    out->insert(out->end(), type, type + 4);
    out->insert(out->end(), payload.begin(), payload.end());
    const uLong crc = crc32(0L, Z_NULL, 0);
    const std::uint32_t checksum = static_cast<std::uint32_t>(
        crc32(crc, out->data() + type_begin, static_cast<uInt>(out->size() - type_begin)));
    AppendBE32(out, checksum);
}

bool WriteWholeFile(const std::string& path,
                    const std::vector<std::uint8_t>& bytes,
                    std::string* error) {
    std::FILE* handle = std::fopen(path.c_str(), "wb");
    if (handle == nullptr) {
        if (error != nullptr) {
            *error = "无法打开写出文件: " + path;
        }
        return false;
    }
    const std::size_t written = std::fwrite(bytes.data(), 1, bytes.size(), handle);
    const int flush = std::fflush(handle);
    const int close = std::fclose(handle);
    if (written != bytes.size() || flush != 0 || close != 0) {
        if (error != nullptr) {
            *error = "PNG 写入不完整: " + path;
        }
        return false;
    }
    return true;
}

bool EncodeGray(const std::string& path,
                std::uint32_t width,
                std::uint32_t height,
                int bit_depth,
                const std::uint8_t* pixels,
                int compression_level,
                std::string* error) {
    if (width == 0 || height == 0 || pixels == nullptr) {
        if (error != nullptr) {
            *error = "PNG 参数非法（空图像或空指针）";
        }
        return false;
    }
    const int channels = bit_depth == 16 ? 2 : 1;
    const std::size_t row_bytes = static_cast<std::size_t>(width) * channels;
    std::vector<std::uint8_t> raw;
    raw.reserve(static_cast<std::size_t>(height) * (row_bytes + 1));
    for (std::uint32_t row = 0; row < height; ++row) {
        raw.push_back(0); // filter type 0 (None)
        const std::uint8_t* source = pixels + static_cast<std::size_t>(row) * row_bytes;
        raw.insert(raw.end(), source, source + row_bytes);
    }

    std::vector<std::uint8_t> compressed;
    compressed.resize(compressBound(static_cast<uLong>(raw.size())));
    uLongf destination_size = static_cast<uLongf>(compressed.size());
    const int level = compression_level < 0 ? 3 : compression_level;
    if (compress2(compressed.data(),
                  &destination_size,
                  raw.data(),
                  static_cast<uLong>(raw.size()),
                  level) != Z_OK) {
        if (error != nullptr) {
            *error = "zlib deflate 失败: " + path;
        }
        return false;
    }
    compressed.resize(destination_size);

    std::vector<std::uint8_t> out;
    out.insert(out.end(), kSignature, kSignature + 8);
    std::vector<std::uint8_t> ihdr;
    AppendBE32(&ihdr, width);
    AppendBE32(&ihdr, height);
    ihdr.push_back(static_cast<std::uint8_t>(bit_depth));
    ihdr.push_back(0); // color type: grayscale
    ihdr.push_back(0); // compression method: deflate
    ihdr.push_back(0); // filter method: adaptive
    ihdr.push_back(0); // interlace: none
    AppendChunk(&out, "IHDR", ihdr);
    AppendChunk(&out, "IDAT", compressed);
    AppendChunk(&out, "IEND", {});
    return WriteWholeFile(path, out, error);
}

} // namespace

bool WritePngGray8(const std::string& path,
                   std::uint32_t width,
                   std::uint32_t height,
                   const std::uint8_t* pixels,
                   int compression_level,
                   std::string* error) {
    return EncodeGray(path, width, height, 8, pixels, compression_level, error);
}

bool WritePngGray16(const std::string& path,
                    std::uint32_t width,
                    std::uint32_t height,
                    const std::uint16_t* samples,
                    int compression_level,
                    std::string* error) {
    const std::size_t count = static_cast<std::size_t>(width) * height;
    // PNG 的 16-bit 样本按大端存放；主机通常是小端，因此逐样本拆字节。
    std::vector<std::uint8_t> bytes(count * 2);
    for (std::size_t index = 0; index < count; ++index) {
        bytes[index * 2] = static_cast<std::uint8_t>(samples[index] >> 8);
        bytes[index * 2 + 1] = static_cast<std::uint8_t>(samples[index] & 0xff);
    }
    return EncodeGray(path, width, height, 16, bytes.data(), compression_level, error);
}

bool InspectPng(const std::string& path, PngHeaderInfo* info, std::string* error) {
    if (info == nullptr) {
        return false;
    }
    std::FILE* handle = std::fopen(path.c_str(), "rb");
    if (handle == nullptr) {
        if (error != nullptr) {
            *error = "无法打开图像文件: " + path;
        }
        return false;
    }
    std::uint8_t signature[8];
    bool ok =
        std::fread(signature, 1, 8, handle) == 8 && std::memcmp(signature, kSignature, 8) == 0;
    bool saw_ihdr = false;
    bool saw_iend = false;
    std::size_t consumed = 8;
    while (ok) {
        std::uint8_t prefix[8];
        if (std::fread(prefix, 1, 8, handle) != 8) {
            break;
        }
        consumed += 8;
        std::uint32_t length = (static_cast<std::uint32_t>(prefix[0]) << 24) |
                               (static_cast<std::uint32_t>(prefix[1]) << 16) |
                               (static_cast<std::uint32_t>(prefix[2]) << 8) |
                               static_cast<std::uint32_t>(prefix[3]);
        const char* type = reinterpret_cast<const char*>(prefix + 4);
        std::vector<std::uint8_t> payload;
        if (length > (1u << 24)) {
            ok = false;
            break;
        }
        payload.resize(length);
        if (length != 0 && std::fread(payload.data(), 1, length, handle) != length) {
            ok = false;
            break;
        }
        consumed += length + 4; // 含 CRC
        if (std::fseek(handle, 4, SEEK_CUR) != 0) {
            ok = false;
            break;
        }
        if (std::strncmp(type, "IHDR", 4) == 0 && length == 13) {
            saw_ihdr = true;
            info->width = (static_cast<std::uint32_t>(payload[0]) << 24) |
                          (static_cast<std::uint32_t>(payload[1]) << 16) |
                          (static_cast<std::uint32_t>(payload[2]) << 8) |
                          static_cast<std::uint32_t>(payload[3]);
            info->height = (static_cast<std::uint32_t>(payload[4]) << 24) |
                           (static_cast<std::uint32_t>(payload[5]) << 16) |
                           (static_cast<std::uint32_t>(payload[6]) << 8) |
                           static_cast<std::uint32_t>(payload[7]);
            info->bit_depth = payload[8];
            info->color_type = payload[9];
            info->interlace = payload[12];
        }
        if (std::strncmp(type, "IEND", 4) == 0) {
            saw_iend = true;
            break;
        }
    }
    long position = std::ftell(handle);
    std::fseek(handle, 0, SEEK_END);
    long end = std::ftell(handle);
    info->file_bytes =
        end > position ? static_cast<std::size_t>(end) : static_cast<std::size_t>(consumed);
    std::fclose(handle);
    if (!ok || !saw_ihdr) {
        if (error != nullptr) {
            *error = "PNG 结构损坏或缺少 IHDR: " + path;
        }
        return false;
    }
    info->terminated = saw_iend;
    return true;
}

} // namespace rh

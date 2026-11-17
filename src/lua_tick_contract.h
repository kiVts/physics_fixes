#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace lua_tick {
inline constexpr uint32_t build = 0x6ae31989;
inline constexpr size_t callback_rva = 0x38ba70, window_offset = 0xfe, window_size = 42;
inline constexpr size_t literal_rva = 0xb62074;
struct Binding { size_t rva, size; uint64_t hash; };
inline constexpr Binding bindings[] = {
    {0x38ba70, 481, 0x66015eb93d5854cdull},
    {0x17ca10, 103, 0x975bba9132d7e1a2ull},
    {0x98120, 153, 0xd33c228245fc39e7ull},
    {0xb5640, 197, 0xee356b8f996f7eeeull},
    {0xb5580, 54, 0xcb676831ae056fabull},
    {0xb53f0, 388, 0x2a30dca6d4e28eb0ull},
    {0x1e6070, 696, 0x13c7620701538449ull},
};
inline uint64_t hash(const uint8_t* bytes, size_t size) {
    uint64_t value = 14695981039346656037ull;
    for (size_t i = 0; i < size; ++i) value = (value ^ bytes[i]) * 1099511628211ull;
    return value;
}
inline bool verified(const uint8_t* image, size_t image_size, uint32_t timestamp) {
    if (!image || timestamp != build || literal_rva > image_size || 7 > image_size - literal_rva ||
        std::memcmp(image + literal_rva, "onTick", 7)) return false;
    for (const auto& binding : bindings) {
        if (binding.rva > image_size || binding.size > image_size - binding.rva ||
            hash(image + binding.rva, binding.size) != binding.hash) return false;
    }
    return true;
}
inline std::array<uint8_t, window_size> replacement() {
    std::array<uint8_t, window_size> bytes;
    bytes.fill(0x90);
    const uint8_t instructions[] = {0x48, 0x8d, 0x05, 0, 0, 0, 0, 0x48, 0x89, 0x44, 0x24, 0x20, 0x31, 0xff};
    std::memcpy(bytes.data(), instructions, sizeof instructions);
    const int32_t displacement = int32_t(literal_rva - (callback_rva + window_offset + 7));
    std::memcpy(bytes.data() + 3, &displacement, sizeof displacement);
    return bytes;
}
void install();
}

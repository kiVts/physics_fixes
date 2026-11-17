#pragma once
// Exact-build contract of the onTick name borrow ([vehicle_logic] lua_borrow_tick_name). One row per audited game
// build: the caller's 42-byte window, the "onTick" literal and the hashed code the patch depends on. New builds' rows
// are only added for masked-identical code; any other build keeps the original allocation.
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace lua_tick {
inline constexpr size_t window_size = 42;
struct Binding { size_t rva, size; uint64_t hash; };
struct Build {
    uint32_t build;
    size_t callback_rva, window_offset, literal_rva;
    Binding bindings[7]; // caller, name allocator, and the lookup / ownership code it reaches
};
inline constexpr Build builds[] = {
    {0x6ae31989, 0x38ba70, 0xfe, 0xb62074, {
        {0x38ba70, 481, 0x66015eb93d5854cdull},
        {0x17ca10, 103, 0x975bba9132d7e1a2ull},
        {0x98120, 153, 0xd33c228245fc39e7ull},
        {0xb5640, 197, 0xee356b8f996f7eeeull},
        {0xb5580, 54, 0xcb676831ae056fabull},
        {0xb53f0, 388, 0x2a30dca6d4e28eb0ull},
        {0x1e6070, 696, 0x13c7620701538449ull}}},
    {0x6afb81d6, 0x38ba70, 0xfe, 0xb62ff4, {
        {0x38ba70, 481, 0x54061793597ca6e8ull},
        {0x17ca10, 103, 0xa684635cb1150223ull},
        {0x98120, 153, 0xd33c228245fc39e7ull},
        {0xb5640, 197, 0xee356b8f996f7eeeull},
        {0xb5580, 54, 0xcb676831ae056fabull},
        {0xb53f0, 388, 0x636e08bbf274b750ull},
        {0x1e6070, 696, 0x03cf3950b5fc15f9ull}}},
    // new build rows go above this line
};
inline uint64_t hash(const uint8_t* bytes, size_t size) {
    uint64_t value = 14695981039346656037ull;
    for (size_t i = 0; i < size; ++i) value = (value ^ bytes[i]) * 1099511628211ull;
    return value;
}
inline const Build* build_for(uint32_t timestamp) {
    for (const Build& b : builds)
        if (b.build == timestamp) return &b;
    return nullptr;
}
// the running build's row if its literal and every binding hash match, else nullptr
inline const Build* verified(const uint8_t* image, size_t image_size, uint32_t timestamp) {
    const Build* b = build_for(timestamp);
    if (!image || !b || b->literal_rva > image_size || 7 > image_size - b->literal_rva ||
        std::memcmp(image + b->literal_rva, "onTick", 7)) return nullptr;
    if (b->callback_rva + b->window_offset + window_size > b->bindings[0].rva + b->bindings[0].size ||
        b->bindings[0].rva != b->callback_rva) return nullptr; // the window lies inside the hashed caller
    for (const auto& binding : b->bindings) {
        if (binding.rva > image_size || binding.size > image_size - binding.rva ||
            hash(image + binding.rva, binding.size) != binding.hash) return nullptr;
    }
    return b;
}
inline std::array<uint8_t, window_size> replacement(const Build& b) {
    std::array<uint8_t, window_size> bytes;
    bytes.fill(0x90);
    const uint8_t instructions[] = {0x48, 0x8d, 0x05, 0, 0, 0, 0, 0x48, 0x89, 0x44, 0x24, 0x20, 0x31, 0xff};
    std::memcpy(bytes.data(), instructions, sizeof instructions);
    const int32_t displacement = int32_t(b.literal_rva - (b.callback_rva + b.window_offset + 7));
    std::memcpy(bytes.data() + 3, &displacement, sizeof displacement);
    return bytes;
}
void install();
// where install() wrote the window and the bytes it wrote (false: not installed); lua_gate::recheck allows them
bool installed_window(uintptr_t* at, const uint8_t** bytes, size_t* size);
}

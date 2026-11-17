#pragma once
#include <cstddef>
#include <cstdint>

namespace radio_phase {
struct Audited { uint32_t build; uint64_t tail, mixer, antenna; };
inline constexpr Audited audited[] = {
    {0x6ae31989, 0x2fd988f4c08c59c5ull, 0x45308935ff4b933cull, 0x17d8186f9e08b425ull},
    {0x6afb81d6, 0xe4bff04baa460a0dull, 0x534c8351f95b68fbull, 0x4755f1bd43d993cdull},
};
inline constexpr uint32_t build = 0x6ae31989; // first audited build (kept for existing users)
inline const Audited* audited_for(uint32_t game_build) {
    for (const Audited& a : audited) if (a.build == game_build) return &a;
    return nullptr;
}
inline constexpr size_t call_offset = 0xbf7, tail_offset = 0xbe0, tail_size = 214;
inline constexpr size_t mixer_size = 2344, antenna_size = 1900;
inline uint64_t hash(const uint8_t* bytes, size_t size) {
    uint64_t value = 14695981039346656037ull;
    for (size_t i = 0; i < size; ++i) value = (value ^ bytes[i]) * 1099511628211ull;
    return value;
}
inline bool matches(const Audited& a, const uint8_t* tail, const uint8_t* mixer, const uint8_t* antenna) {
    return hash(tail, tail_size) == a.tail && hash(mixer, mixer_size) == a.mixer && hash(antenna, antenna_size) == a.antenna;
}
inline bool matches(const uint8_t* tail, const uint8_t* mixer, const uint8_t* antenna) {
    for (const Audited& a : audited) if (matches(a, tail, mixer, antenna)) return true;
    return false;
}
}

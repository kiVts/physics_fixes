#pragma once
#include <cstdint>
namespace sound_profiler::native {
enum class Kind { assign, update, check };
struct Binding { uint32_t build, rva, size; uint64_t hash; Kind kind; };
inline constexpr Binding bindings[] = {
    {0x6a9a9e20, 0x8e8270, 0xb0, 0xcaa6cf9a63e87c7full, Kind::assign},
    {0x6a9a9e20, 0x28a7d0, 0x433, 0x652eca1039d3e7eaull, Kind::update},
    {0x6a9a9e20, 0x28ac10, 0x15f, 0xac85abef1c3f9bccull, Kind::check},
    {0x6a9a9e20, 0x286c50, 0x6c, 0x74b99ae787e157e5ull, Kind::check},
    {0x6a9a9e20, 0x289250, 0x801, 0xc3a59cf1c3ef15bdull, Kind::check},
    {0x6a9a9e20, 0x289e40, 0x1d1, 0x79061a0b4b57166dull, Kind::check},
    {0x6a9a9e20, 0x288e30, 0x61, 0x6636dbe0438cea73ull, Kind::check},
    {0x6a9a9e20, 0x288ea0, 0xce, 0xa473de609ba8ec65ull, Kind::check},
    {0x6a9a9e20, 0x8cd430, 0xc2, 0x2b8edee507911fbbull, Kind::check},
    {0x6a9a9e20, 0x8cd500, 0x82, 0x652e575bb323033aull, Kind::check},
    {0x6a9a9e20, 0x28a120, 0x14, 0xea436ce4fea34f10ull, Kind::check},
    {0x6a9a9e20, 0x28a420, 0x8b, 0x529975d2b526ef57ull, Kind::check},
    {0x6ae31989, 0x8ec0b0, 0xb0, 0x88a4aef3e06eb4c6ull, Kind::assign},
    {0x6ae31989, 0x28cb40, 0x433, 0xca6449ee8a592672ull, Kind::update},
    {0x6ae31989, 0x28cf80, 0x15f, 0x25f6e19d252c60dbull, Kind::check},
    {0x6ae31989, 0x288fc0, 0x6c, 0x974865efa84f9bdaull, Kind::check},
    {0x6ae31989, 0x28b5c0, 0x801, 0x776e621134659039ull, Kind::check},
    {0x6ae31989, 0x28c1b0, 0x1d1, 0xf84613d6f7e96eb4ull, Kind::check},
    {0x6ae31989, 0x28b1a0, 0x61, 0x21522b5dd8594289ull, Kind::check},
    {0x6ae31989, 0x28b210, 0xce, 0x3ad15b2881772360ull, Kind::check},
    {0x6ae31989, 0x8d1a60, 0xc2, 0x9901030d9d871a17ull, Kind::check},
    {0x6ae31989, 0x8d1b30, 0x82, 0x2c8cddc896abd5eeull, Kind::check},
    {0x6ae31989, 0x28c490, 0x14, 0x3aa51a684bf639e3ull, Kind::check},
    {0x6ae31989, 0x28c790, 0x8b, 0x00ab5ab5646f33d0ull, Kind::check},
};
}

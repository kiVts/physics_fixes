#pragma once
#include <cstdint>
namespace seabed_query::native {
enum class Kind { query, cleanup, check };
struct Binding { uint32_t build, rva, size; uint64_t hash; Kind kind; };
inline constexpr Binding bindings[] = {
    {0x6a9a9e20, 0x2c23e0, 0x2c8, 0x35ea3cde2a93e115ull, Kind::query},
    {0x6a9a9e20, 0x2c40c0, 0x41, 0x584cf5a918851388ull, Kind::cleanup},
    {0x6a9a9e20, 0x2c3fa0, 0x119, 0x90c45ed8a6224982ull, Kind::check},
    {0x6a9a9e20, 0x2c1d10, 0x6c2, 0x5d9def21a8134a71ull, Kind::check},
    {0x6a9a9e20, 0x2c26b0, 0x607, 0x340bc8e6947ccf36ull, Kind::check},
    {0x6a9a9e20, 0x8cbca0, 0xdc, 0x53048c034c67717cull, Kind::check},
    {0x6a9a9e20, 0x8d6140, 0xf7, 0xabab9556a7cad7a8ull, Kind::check},
    {0x6a9a9e20, 0x19d460, 0xb3, 0xd9000af704f265bdull, Kind::check},
    {0x6a9a9e20, 0x19af20, 0xc0, 0xe128fba8e916bcb5ull, Kind::check},
    {0x6a9a9e20, 0x19cfc0, 0x49c, 0x8eb38c489d810feaull, Kind::check},
    {0x6ae31989, 0x2c48f0, 0x2c8, 0x055284a3a475ae42ull, Kind::query},
    {0x6ae31989, 0x2c65d0, 0x41, 0x80a7bcbeb2fd694dull, Kind::cleanup},
    {0x6ae31989, 0x2c64b0, 0x119, 0x90c45ed8a6224982ull, Kind::check},
    {0x6ae31989, 0x2c4220, 0x6c2, 0x2f4639e17c3d5febull, Kind::check},
    {0x6ae31989, 0x2c4bc0, 0x607, 0xa462ea86c3a6bbb3ull, Kind::check},
    {0x6ae31989, 0x8d02d0, 0xdc, 0x2a346767aacfe30dull, Kind::check},
    {0x6ae31989, 0x8da2c0, 0xf7, 0x5da7940321426002ull, Kind::check},
    {0x6ae31989, 0x19d310, 0xb3, 0x62ca5f32e80811c1ull, Kind::check},
    {0x6ae31989, 0x19add0, 0xc0, 0xf0996c022814451eull, Kind::check},
    {0x6ae31989, 0x19ce70, 0x49c, 0xe79069051bf3c5ecull, Kind::check},
};
}

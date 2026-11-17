#pragma once
// lua_gate's start-up check and patch bytes, free of game state so the standalone tests run the same code.
#include "lua_gate_contract.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iterator>

namespace lua_gate {
inline constexpr size_t kNumAnchors = std::size(kAnchors), kNumUnits = std::size(kUnits);

inline uint64_t fnv(const uint8_t* p, size_t n)
{
    uint64_t h = 14695981039346656037ull;
    for (size_t i = 0; i < n; ++i) h = (h ^ p[i]) * 1099511628211ull;
    return h;
}

// exactly one match in [beg, end), or 0
inline uintptr_t find_unique(uintptr_t beg, uintptr_t end, const uint8_t* pat, size_t len)
{
    if (end - beg < len) return 0;
    uintptr_t found = 0;
    const uint8_t* p = reinterpret_cast<const uint8_t*>(beg);
    const uint8_t* stop = reinterpret_cast<const uint8_t*>(end) - len;
    while (p <= stop) {
        p = static_cast<const uint8_t*>(memchr(p, pat[0], size_t(stop - p) + 1));
        if (!p) break;
        if (!memcmp(p, pat, len)) {
            if (found) return 0;
            found = reinterpret_cast<uintptr_t>(p);
        }
        ++p;
    }
    return found;
}

// .text of the build, mapped: finds the anchors and checks every unit and the patch site.
// nullptr = verified (anchors / units filled), else the reason.
inline const char* locate(uintptr_t text_beg, uintptr_t text_end, uint32_t build, uintptr_t* anchors, uintptr_t* units)
{
    if (build != kBuild) return "not the audited build";
    for (size_t i = 0; i < kNumAnchors; ++i)
        if (!(anchors[i] = find_unique(text_beg, text_end, kAnchors[i].bytes, kAnchors[i].len))) return "an anchor isn't found exactly once";
    for (size_t i = 0; i < kNumUnits; ++i) {
        const Unit& u = kUnits[i];
        const uintptr_t at = anchors[u.anchor] + intptr_t(u.delta);
        if (at < text_beg || at + u.size > text_end || fnv(reinterpret_cast<const uint8_t*>(at), u.size) != u.hash)
            return "an audited code unit differs";
        units[i] = at;
    }
    if (memcmp(reinterpret_cast<const void*>(units[kPrecallUnit] + kPrecallSiteOffset), kSiteBytes, sizeof kSiteBytes))
        return "the Lua runtime's C-function call isn't the expected instruction";
    return nullptr;
}

inline std::array<uint8_t, 47> stub_code(uintptr_t active, uintptr_t check)
{
    std::array<uint8_t, 47> c = {
        0x48, 0xB8, 0, 0, 0, 0, 0, 0, 0, 0, // mov rax, active
        0x83, 0x38, 0x00,                   // cmp dword ptr [rax], 0
        0x74, 0x1A,                         // je fast (15 + 26 = 41)
        0x48, 0x83, 0xEC, 0x28,             // sub rsp, 0x28
        0x48, 0x8B, 0xCB,                   // mov rcx, rbx
        0x4C, 0x89, 0xF2,
        0x48, 0xB8, 0, 0, 0, 0, 0, 0, 0, 0, // mov rax, check
        0xFF, 0xD0,                         // call rax
        0x48, 0x83, 0xC4, 0x28,             // add rsp, 0x28
        0x48, 0x8B, 0xCB,                   // fast: mov rcx, rbx
        0x41, 0xFF, 0xE6,                   // jmp r14
    };
    memcpy(c.data() + 2, &active, 8);
    memcpy(c.data() + 27, &check, 8);
    return c;
}

// call qword ptr [rip + disp] at the site; the slot holds the stub's address
inline std::array<uint8_t, 6> site_code(int32_t disp_to_slot)
{
    std::array<uint8_t, 6> c = {0xFF, 0x15};
    memcpy(c.data() + 2, &disp_to_slot, 4);
    return c;
}
}

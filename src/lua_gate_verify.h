#pragma once
// lua_gate's start-up check and patch bytes, free of game state so the standalone tests run the same code.
#include "lua_gate_contract.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iterator>

namespace lua_gate {
// one contract per audited build (lua_gate_contract_<build>.h); nullptr for any other build
inline const Contract* contract_for(uint32_t build)
{
    for (const Contract* c : kContracts)
        if (c->build == build) return c;
    return nullptr;
}
constexpr size_t max_over_contracts(size_t Contract::*field)
{
    size_t m = 0;
    for (const Contract* c : kContracts) m = c->*field > m ? c->*field : m;
    return m;
}
inline constexpr size_t kMaxAnchors = max_over_contracts(&Contract::num_anchors);
inline constexpr size_t kMaxUnits = max_over_contracts(&Contract::num_units);

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

// .text of the build, mapped: checks the contract's anchors, every unit and the patch site.
// nullptr = verified (anchors[] / units[] filled; arrays of kMaxAnchors / kMaxUnits), else the reason.
inline const char* locate(uintptr_t text_beg, uintptr_t text_end, const Contract* c, uintptr_t* anchors, uintptr_t* units)
{
    if (!c) return "not an audited build";
    for (size_t i = 0; i < c->num_anchors; ++i)
        if (!(anchors[i] = find_unique(text_beg, text_end, c->anchors[i].bytes, c->anchors[i].len))) return "an anchor isn't found exactly once";
    for (size_t i = 0; i < c->num_units; ++i) {
        const Unit& u = c->units[i];
        if (u.anchor >= c->num_anchors) return "a code unit names a missing anchor";
        const uintptr_t at = anchors[u.anchor] + intptr_t(u.delta);
        if (at < text_beg || at + u.size > text_end || fnv(reinterpret_cast<const uint8_t*>(at), u.size) != u.hash)
            return "an audited code unit differs";
        units[i] = at;
    }
    for (uint16_t k : {c->precall_unit, c->timeout_hook_unit, c->alloc_unit, c->caller_unit, c->http_tail_unit})
        if (k >= c->num_units) return "a named unit is out of range";
    for (size_t i = 0; i < c->num_named; ++i)
        if (c->named[i].unit >= c->num_units) return "a named unit is out of range";
    if (c->precall_site_offset + sizeof kSiteBytes > c->units[c->precall_unit].size ||
        memcmp(reinterpret_cast<const void*>(units[c->precall_unit] + c->precall_site_offset), kSiteBytes, sizeof kSiteBytes))
        return "the Lua runtime's C-function call isn't the expected instruction";
    return nullptr;
}

// The running build's contract (*out, nullptr if none) and its check.
inline const char* locate(uintptr_t text_beg, uintptr_t text_end, uint32_t build, const Contract** out, uintptr_t* anchors, uintptr_t* units)
{
    *out = contract_for(build);
    return locate(text_beg, text_end, *out, anchors, units);
}

// The final detour behind a hook jump at p (no fault handling: callers guard), or 0. MinHook writes `jmp rel32` at
// the target (or, hot-patched, `jmp short -7` to a jmp rel32 in the 5 bytes above) into its x64 relay `jmp [rip+0];
// dq detour`; the SWML core's detour there is its chain stub `jmp [rip+2]; int3; int3; dq newest`.
inline uintptr_t follow_hook(const uint8_t* p)
{
    uintptr_t j;
    int32_t rel;
    if (p[0] == 0xE9) {
        memcpy(&rel, p + 1, 4);
        j = reinterpret_cast<uintptr_t>(p) + 5 + intptr_t(rel);
    } else if (p[0] == 0xEB && p[1] == 0xF9 && p[-5] == 0xE9) {
        memcpy(&rel, p - 4, 4);
        j = reinterpret_cast<uintptr_t>(p) + intptr_t(rel);
    } else {
        return 0;
    }
    for (int hop = 0; hop < 2; ++hop) {
        const uint8_t* r = reinterpret_cast<const uint8_t*>(j);
        int32_t d;
        memcpy(&d, r + 2, 4);
        if (r[0] != 0xFF || r[1] != 0x25 || !(d == 0 || (d == 2 && r[6] == 0xCC && r[7] == 0xCC))) break;
        memcpy(&j, r + 6 + d, 8);
    }
    return j;
}

// Address ranges whose current bytes are expected to differ from the copy (each already checked to hold its
// expected bytes by the caller).
struct Range { uintptr_t lo, hi; };
// The first byte of a unit (now / copy `was`, n bytes, at address `base`) that differs and isn't explained by the
// first `hooked` bytes or one of the ranges; n when there is none.
inline size_t first_unexplained(const uint8_t* now, const uint8_t* was, size_t n, uintptr_t base, size_t hooked, const Range* ok, size_t nok)
{
    for (size_t k = 0; k < n; ++k) {
        if (now[k] == was[k] || k < hooked) continue;
        bool inside = false;
        for (size_t r = 0; r < nok && !inside; ++r) inside = base + k >= ok[r].lo && base + k < ok[r].hi;
        if (!inside) return k;
    }
    return n;
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

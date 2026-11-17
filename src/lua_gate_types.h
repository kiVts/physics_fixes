#pragma once
// Types of the generated per-build contracts (lua_gate_contract_<build>.h).
#include <cstddef>
#include <cstdint>
#include <iterator>

namespace lua_gate {
struct Anchor { uint8_t len; uint8_t bytes[64]; };
struct Unit { uint8_t anchor; int32_t delta; uint32_t size; uint64_t hash; };
struct Named { const char* name; uint16_t unit; uint8_t allowed; };
struct LockedName { const char* name; uint8_t anchor; int32_t delta; };
struct Contract {
    uint32_t build;
    const Anchor* anchors;
    size_t num_anchors;
    const Unit* units;
    size_t num_units;
    const Named* named;
    size_t num_named;
    const LockedName* locked;
    size_t num_locked;
    uint16_t precall_unit, timeout_hook_unit, alloc_unit, caller_unit, http_tail_unit;
    uint32_t precall_site_offset;
    uint16_t clock_unit; // the game's ms clock: the SWML core hooks it (update-loop fix)
    uint16_t timer_unit; // the game's other clock: the SDK's flight-pass timer detour (native_draw.h) may sit on it
};
// mov rcx, rbx; call r14: the Lua runtime's one C-function call (luaD_precall), the same in every audited build
inline constexpr uint8_t kSiteBytes[6] = {0x48, 0x8b, 0xcb, 0x41, 0xff, 0xd6};
}

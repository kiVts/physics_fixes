#pragma once
// Detours on audited code units that unlocked Lua accepts besides this mod's sink wrappers: this mod's own
// _aligned_free detour when nothing else is chained behind it (own_alone), and the one other mod's detour: the SDK's flight-pass timer detour
// (sdk/native_draw.h flight_timer_detour, installed by on_flight(), e.g. Realistic Aero's VLM overlay) on the game
// timer. The SWML core's own clock hook is accepted separately (vehicle_mt.cpp lua_hook_ok).
//
// On every call the detour calls *next (what it hooked), then compares its own return address with *key and only on a
// match runs its callback (the render thread's flight pass). Unlocked Lua reaches the timer only through the game
// clock's two calls, which lie inside audited code in every audited build: when *key is outside
// the audited code, the callback can't run on a Lua call and the detour only passes the call on.
//
// check() accepts a chain only when every hop is exactly this code (as MSVC /O2 builds the SDK header; only its five
// RIP and TLS fields may differ), each hop's *key is outside the audited code, and the last hop's *next is MinHook's
// trampoline (not in a loaded image) jumping back into the hooked unit. Anything else: unlocked Lua goes off.
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace lua_clock_hook {
// flight_timer_detour: push rbx; sub rsp,30h; call [next]; mov rcx,[rsp+38h] (its return address); mov rbx,rax;
// cmp rcx,[key]; jne out; (TLS renderer; callback; call rax); out: mov rax,rbx; add rsp,30h; pop rbx; ret
inline constexpr uint8_t kFlightTimer[] = {
    0x40, 0x53, 0x48, 0x83, 0xEC, 0x30,                         // push rbx; sub rsp, 0x30
    0xFF, 0x15, 0, 0, 0, 0,                                     // 06 call [rip+next]
    0x48, 0x8B, 0x4C, 0x24, 0x38,
    0x48, 0x8B, 0xD8,                                           // 11 mov rbx, rax
    0x48, 0x3B, 0x0D, 0, 0, 0, 0,                               // 14 cmp rcx, [rip+key]
    0x75, 0x45,                                                 // 1b jne 0x62
    0x8B, 0x15, 0, 0, 0, 0,                                     // 1d mov edx, [rip+_tls_index]
    0x65, 0x48, 0x8B, 0x0C, 0x25, 0x58, 0x00, 0x00, 0x00,       // 23 mov rcx, gs:[0x58]
    0xB8, 0, 0, 0, 0,                                           // 2c mov eax, secrel(renderer)
    0x48, 0x8B, 0x0C, 0xD1,
    0x48, 0x8B, 0x14, 0x08,
    0x48, 0x85, 0xD2, 0x74, 0x21,
    0x48, 0x8B, 0x05, 0, 0, 0, 0,                               // 3e mov rax, [rip+callback]
    0x48, 0x85, 0xC0, 0x74, 0x15,                               // 45 test rax, rax; je 0x5f
    0x48, 0x8D, 0x4C, 0x24, 0x20,
    0x48, 0x89, 0x54, 0x24, 0x20,
    0x48, 0xC7, 0x44, 0x24, 0x28, 0x00, 0x00, 0x00, 0x00,
    0xFF, 0xD0,                                                 // 5d call rax
    0x48, 0x8B, 0xC3, 0x48, 0x83, 0xC4, 0x30, 0x5B, 0xC3,       // 5f mov rax, rbx; add rsp, 0x30; pop rbx; ret
};
inline constexpr size_t kFieldAt[] = {0x08, 0x17, 0x1F, 0x2D, 0x41}; // next, key, _tls_index, secrel, callback (4 bytes each)

// p: code at its own address. true = the flight-timer detour; *next_slot / *key_slot: where it reads its next hop and
// its key (no fault handling: callers guard)
inline bool flight_timer(const uint8_t* p, uintptr_t* next_slot, uintptr_t* key_slot)
{
    size_t from = 0;
    for (size_t f : kFieldAt) {
        if (memcmp(p + from, kFlightTimer + from, f - from)) return false;
        from = f + 4;
    }
    if (memcmp(p + from, kFlightTimer + from, sizeof kFlightTimer - from)) return false;
    int32_t d;
    memcpy(&d, p + 0x08, 4);
    *next_slot = reinterpret_cast<uintptr_t>(p) + 0x0C + intptr_t(d);
    memcpy(&d, p + 0x17, 4);
    *key_slot = reinterpret_cast<uintptr_t>(p) + 0x1B + intptr_t(d);
    return true;
}

// MinHook's trampoline for a hook on `unit`: the unit's first instructions (5 bytes or more, copied), then
// jmp [rip+0]; dq back, with back just past them
inline bool trampoline_into(const uint8_t* t, uintptr_t unit)
{
    static constexpr uint8_t kJmp[6] = {0xFF, 0x25, 0, 0, 0, 0};
    for (size_t i = 5; i <= 32; ++i)
        if (!memcmp(t + i, kJmp, sizeof kJmp)) {
            uintptr_t back;
            memcpy(&back, t + i + 6, 8);
            return back >= unit + 5 && back <= unit + 16;
        }
    return false;
}

using Pred = bool (*)(uintptr_t);

// This mod's own detour on an audited unit (its _aligned_free detour, render_scratch.cpp / render_mt.cpp: thread-safe
// from any thread, only its own leaf SRW locks, no world lock, no VM token, no game state): accepted when it is the
// newest detour on `unit` and what it calls on (`next`, its original) is MinHook's trampoline back into the unit, i.e.
// no other mod's detour runs behind it. nullptr = accepted, else why not. No fault handling: callers guard.
inline const char* own_alone(uintptr_t detour, uintptr_t unit, uintptr_t own, uintptr_t next, Pred in_image)
{
    if (!own || detour != own) return "not this mod's own detour";
    if (!next || in_image(next)) return "another detour runs behind this mod's own one";
    return trampoline_into(reinterpret_cast<const uint8_t*>(next), unit) ? nullptr : "this mod's own detour calls on into unknown code";
}

// detour: the newest detour on `unit`. nullptr = accepted, else why not. in_image: the address is in a loaded image
// (MinHook's trampolines aren't); audited: the address is inside the audited Lua code. No fault handling: callers guard.
inline const char* check(uintptr_t detour, uintptr_t unit, Pred in_image, Pred audited)
{
    uintptr_t at = detour;
    for (int hop = 0; hop < 4; ++hop) {
        uintptr_t next_slot, key_slot, next, key;
        if (!flight_timer(reinterpret_cast<const uint8_t*>(at), &next_slot, &key_slot)) return "not the SDK flight-pass timer detour";
        memcpy(&key, reinterpret_cast<const void*>(key_slot), 8);
        if (audited(key)) return "the flight-pass timer detour's call-site key is inside the audited Lua code";
        memcpy(&next, reinterpret_cast<const void*>(next_slot), 8);
        if (!next) return "the flight-pass timer detour has no next hop";
        if (!in_image(next))
            return trampoline_into(reinterpret_cast<const uint8_t*>(next), unit) ? nullptr
                                                                                : "the flight-pass timer detour calls on into unknown code";
        at = next; // another flight-pass timer detour (a second mod using on_flight)
    }
    return "the flight-pass timer chain is too long";
}
} // namespace lua_clock_hook

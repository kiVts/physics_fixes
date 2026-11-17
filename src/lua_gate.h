#pragma once
// Microcontroller Lua without the vehicle-logic world lock ([vehicle_logic] lua_unlocked=1, off by default).
//
// A Lua block's onTick only touches its own VM and component, except through C functions. Every C function a
// script reaches goes through one call in the Lua runtime. That call is routed through a check: on an
// unlocked onTick thread, a C function that isn't on the audited allowlist (or a call on another VM) takes the
// world lock for the rest of the component tick. Not audited = locked; nothing has to be remembered by hand.
//
// verify() runs first at start-up, before this mod patches or hooks anything: it finds the running build's contract
// (lua_gate_contract_<build>.h), its anchors by byte pattern, hashes every code unit an unlocked onTick can run and
// keeps a copy of their bytes. Any difference, or a build without a contract, and the feature stays off.
// Hooks are only switched on after every mod is loaded, so recheck() compares the units with that copy again once
// the game runs (each tick): only this mod's own wrappers, the SWML core's clock hook, the SDK's flight-pass timer
// detour (lua_clock_hook.h), the precall patch and the onTick name window may differ.
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace lua_gate {
using Check = void(__fastcall*)(uintptr_t L, uintptr_t f);

// Start-up: build, anchors and all unit hashes. false = off (logged with the reason).
bool verify();
// After verify(), at mod init: route the Lua runtime's C-function call through `check` while `*active` != 0.
bool patch(Check check, const std::atomic<int>* active);

// The final detour behind a hook at `at` (MinHook's jump and relay, then the SWML core's chain stub: its newest
// detour), or 0 when `at` doesn't start with a hook jump.
uintptr_t hook_target(uintptr_t at);

// Every unit against its bytes at verify(). A difference is allowed only in: the first bytes of a unit hooked by a
// detour `hook_ok(unit, detour, which)` accepts (which: the unit is the game clock / the game timer / neither); the
// precall patch this mod wrote; [window, window + size) when it holds exactly `window_bytes`. nullptr = everything
// accounted for, else the reason (*at = the address).
enum class Clock : uint8_t { none, clock, timer };
using HookOk = bool (*)(uintptr_t unit, uintptr_t detour, Clock which);
// note: when set, every unexplained unit is passed to it (the address, the detour followed or 0, why), not just the
// first; the result and *at are still the first one's.
using Note = void (*)(uintptr_t at, uintptr_t detour, const char* why);
const char* recheck(HookOk hook_ok, uintptr_t window, const uint8_t* window_bytes, size_t size, uintptr_t* at,
                    Note note = nullptr);

bool audited(uintptr_t a);          // a lies inside a verified code unit
size_t unit_count();                // code units verify() hashed
bool allowed(uintptr_t f);          // audited C function (sorted lookup)
const char* name_of(uintptr_t f);   // log name, or nullptr
uintptr_t timeout_hook();           // the script time limit hook
uintptr_t alloc_fn();               // the Lua allocator
uintptr_t caller();
uintptr_t http_tail();              // the HTTP tail after the onTick call
}

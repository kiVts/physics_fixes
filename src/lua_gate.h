#pragma once
// Microcontroller Lua without the vehicle-logic world lock ([vehicle_logic] lua_unlocked=1, off by default).
//
// A Lua block's onTick only touches its own VM and component, except through C functions. Every C function a
// script reaches goes through one call in the Lua runtime. That call is routed through a check: on an
// unlocked onTick thread, a C function that isn't on the audited allowlist (or a call on another VM) takes the
// world lock for the rest of the component tick. Not audited = locked; nothing has to be remembered by hand.
//
// verify() runs first at start-up, before this mod patches or hooks anything: it finds the contract's anchors by
// byte pattern and hashes every code unit an unlocked onTick can run (lua_gate_contract.h). Any difference, or
// another build, and the feature stays off.
#include <atomic>
#include <cstdint>

namespace lua_gate {
using Check = void(__fastcall*)(uintptr_t L, uintptr_t f);

// Start-up: build, anchors and all unit hashes. false = off (logged with the reason).
bool verify();
// After verify(), at mod init: route the Lua runtime's C-function call through `check` while `*active` != 0.
bool patch(Check check, const std::atomic<int>* active);

size_t unit_count();                // code units verify() hashed
bool allowed(uintptr_t f);          // audited C function (sorted lookup)
const char* name_of(uintptr_t f);   // log name, or nullptr
uintptr_t timeout_hook();           // the script time limit hook
uintptr_t alloc_fn();               // the Lua allocator
uintptr_t caller();
uintptr_t http_tail();              // the HTTP tail after the onTick call
}

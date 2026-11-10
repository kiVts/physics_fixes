#pragma once
#include <cstdint>
// Optional mod-to-mod observer ABI. Keep the copy in physics_fixes identical.
// Borrowed pointers are used only during the call. All callbacks are read-only.
struct LogicTraceBridge {
    uint32_t size, version;
    void(__cdecl* batch)(uintptr_t list, uint32_t mode, uint32_t reason, uint32_t vehicles, uint32_t units);
    void(__cdecl* component)(uintptr_t component, uint32_t stage, uint32_t unit, uint32_t order);
    void(__cdecl* body)(uintptr_t body, uint32_t stage);
};
using GetLogicTraceBridge = const LogicTraceBridge*(__cdecl*)();
// mode: 0 native, 1 serial, 2 grouped, 3 parallel.
// reason: 0 selected, 1 mode off, 2 frame invalid, 3 collect failed,
//         4 below minimum, 5 grouping failed. stage: 1 before, 2 after.

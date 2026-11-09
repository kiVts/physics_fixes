#pragma once
#include <windows.h>
#include <cstdint>

inline bool sensor_source_ok(uintptr_t source)
{
    __try {
        auto owner = *reinterpret_cast<const int* const*>(source + 0xc0);
        if (!owner || owner[0] != 2) return true;
        auto character = *reinterpret_cast<const uint8_t* const*>(owner + 6);
        if (!character) return true;
        auto holder = *reinterpret_cast<const uint8_t* const* const*>(character + 0x90);
        if (!holder) return true;
        const uint8_t* object = *holder;
        if (object) (void)*reinterpret_cast<const volatile uintptr_t*>(object + 0x2100);
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

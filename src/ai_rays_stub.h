#pragma once
// The call stub of ai_rays.cpp: called in place of a 5-argument function, it calls `target` with the same five
// arguments plus the caller's rbx as a sixth, and returns its rax. Its own 0x38-byte frame keeps the call 16-aligned;
// unwind data is registered so stack walks get through it. mem: 128 bytes, executable. Tested in ai_rays_stub_test.
#include <windows.h>
#include <cstdint>
#include <cstring>
#include <initializer_list>

inline bool make_rbx_stub(uint8_t* mem, const void* target)
{
    uint8_t* p = mem;
    auto emit = [&p](std::initializer_list<uint8_t> b) { for (uint8_t x : b) *p++ = x; };
    emit({0x48, 0x83, 0xEC, 0x38});       // sub rsp, 0x38
    emit({0x48, 0x8B, 0x44, 0x24, 0x60});
    emit({0x48, 0x89, 0x44, 0x24, 0x20});
    emit({0x48, 0x89, 0x5C, 0x24, 0x28});
    emit({0x49, 0xBB});                   // mov r11, target
    uint64_t t = reinterpret_cast<uint64_t>(target);
    memcpy(p, &t, 8), p += 8;
    emit({0x41, 0xFF, 0xD3});       // call r11
    emit({0x48, 0x83, 0xC4, 0x38}); // add rsp, 0x38
    emit({0xC3});                   // ret
    DWORD code_size = DWORD(p - mem);
    const uint8_t info[] = {0x01, 0x04, 0x01, 0x00,  // version 1, prolog 4 bytes, 1 code, no frame register
                            0x04, 0x62, 0x00, 0x00}; // at +4: UWOP_ALLOC_SMALL, (6 + 1) * 8 = 0x38
    memcpy(mem + 64, info, sizeof info);
    auto* rf = reinterpret_cast<RUNTIME_FUNCTION*>(mem + 96);
    rf->BeginAddress = 0;
    rf->EndAddress = code_size;
    rf->UnwindData = 64;
    FlushInstructionCache(GetCurrentProcess(), mem, code_size);
    return RtlAddFunctionTable(rf, 1, reinterpret_cast<DWORD64>(mem)) != FALSE;
}

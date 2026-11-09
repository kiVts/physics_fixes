#pragma once
#include <cstdint>
#include <cstring>
#include <initializer_list>


namespace call_stub {

struct Emit {
    uint8_t* p;
    void b(std::initializer_list<uint8_t> v)
    {
        for (uint8_t x : v) *p++ = x;
    }
    void d32(uint32_t v)
    {
        memcpy(p, &v, 4);
        p += 4;
    }
    void d64(uint64_t v)
    {
        memcpy(p, &v, 8);
        p += 8;
    }
};

constexpr uint32_t kFrame = 0x170, kArgSlots = 32;
constexpr uint8_t kPrologSize = 4; // push rbp; mov rbp, rsp

inline size_t make_call_stub(uint8_t* at, uint32_t id, void** tramp_slot, void* enter, void* leave)
{
    Emit e{at};
    e.b({0x55});                   // push rbp
    e.b({0x48, 0x89, 0xE5});       // mov rbp, rsp
    e.b({0x48, 0x81, 0xEC});       // sub rsp, kFrame
    e.d32(kFrame);
    e.b({0x48, 0x89, 0x4D, 0x10});
    e.b({0x48, 0x89, 0x55, 0x18});
    e.b({0x4C, 0x89, 0x45, 0x20});
    e.b({0x4C, 0x89, 0x4D, 0x28});
    const uint8_t xmm_modrm[4] = {0x84, 0x8C, 0x94, 0x9C};
    for (int x = 0; x < 4; x++) {
        e.b({0xF3, 0x0F, 0x7F, xmm_modrm[x], 0x24});
        e.d32(0x120 + 16 * x);
    }
    e.b({0xB9}); // mov ecx, id
    e.d32(id);
    e.b({0x48, 0x8D, 0x55, 0x10});
    e.b({0x48, 0xB8});             // mov rax, enter
    e.d64(reinterpret_cast<uint64_t>(enter));
    e.b({0xFF, 0xD0}); // call rax
    e.b({0x31, 0xC0});                   // xor eax, eax
    e.b({0x4C, 0x8B, 0x54, 0xC5, 0x30});
    e.b({0x4C, 0x89, 0x54, 0xC4, 0x20});
    e.b({0xFF, 0xC0});                   // inc eax
    e.b({0x83, 0xF8, static_cast<uint8_t>(kArgSlots)}); // cmp eax, 32
    e.b({0x72, 0xEF});                   // jb (back 17 bytes, to the first mov)
    e.b({0x48, 0x8B, 0x4D, 0x10});
    e.b({0x48, 0x8B, 0x55, 0x18});
    e.b({0x4C, 0x8B, 0x45, 0x20});
    e.b({0x4C, 0x8B, 0x4D, 0x28});
    for (int x = 0; x < 4; x++) {
        e.b({0xF3, 0x0F, 0x6F, xmm_modrm[x], 0x24});
        e.d32(0x120 + 16 * x);
    }
    e.b({0x48, 0xB8}); // mov rax, tramp_slot
    e.d64(reinterpret_cast<uint64_t>(tramp_slot));
    e.b({0xFF, 0x10});                         // call [rax]
    e.b({0x48, 0x89, 0x84, 0x24, 0x60, 0x01, 0x00, 0x00});
    e.b({0xF3, 0x0F, 0x7F, 0x84, 0x24});
    e.d32(0x120);
    e.b({0xB9}); // mov ecx, id
    e.d32(id);
    e.b({0x48, 0xB8}); // mov rax, leave
    e.d64(reinterpret_cast<uint64_t>(leave));
    e.b({0xFF, 0xD0});                         // call rax
    e.b({0x48, 0x8B, 0x84, 0x24, 0x60, 0x01, 0x00, 0x00});
    e.b({0xF3, 0x0F, 0x6F, 0x84, 0x24});
    e.d32(0x120);
    e.b({0x48, 0x8D, 0x65, 0x00}); // lea rsp, [rbp]
    e.b({0x5D});                   // pop rbp
    e.b({0xC3});                   // ret
    return static_cast<size_t>(e.p - at);
}

// One UNWIND_INFO (8 bytes, DWORD aligned) shared by every stub: frame register rbp (offset 0), set at offset 4; rbp pushed at offset 1.
inline void make_call_stub_unwind(uint8_t* at)
{
    const uint8_t ui[8] = {
        0x01,             // version 1, no flags
        kPrologSize,      // prolog size
        2,                // unwind codes
        0x05,             // frame register rbp, frame offset 0
        0x04, 0x03,       // offset 4: UWOP_SET_FPREG
        0x01, 0x50,       // offset 1: UWOP_PUSH_NONVOL rbp
    };
    memcpy(at, ui, sizeof ui);
}

} // namespace call_stub

#include "lua_gate.h"
#include "lua_gate_verify.h"
#include "game.h"
#include "log.h"
#include "native_patch.h"
#include <windows.h>
#include <algorithm>
#include <cstring>
#include <vector>

namespace lua_gate {
namespace {
uintptr_t g_anchor[kNumAnchors] = {};
uintptr_t g_unit[kNumUnits] = {};
std::vector<uintptr_t> g_allowed; // sorted
struct NameAt {
    uintptr_t at;
    const char* name;
};
std::vector<NameAt> g_names; // sorted by address
bool g_verified = false;

bool text_range(uintptr_t& beg, uintptr_t& end)
{
    __try {
        auto* image = reinterpret_cast<const uint8_t*>(game_base());
        auto* dos = reinterpret_cast<const IMAGE_DOS_HEADER*>(image);
        if (dos->e_magic != IMAGE_DOS_SIGNATURE) return false;
        auto* nt = reinterpret_cast<const IMAGE_NT_HEADERS64*>(image + dos->e_lfanew);
        if (nt->Signature != IMAGE_NT_SIGNATURE) return false;
        auto* sec = IMAGE_FIRST_SECTION(nt);
        for (unsigned i = 0; i < nt->FileHeader.NumberOfSections; ++i)
            if (!memcmp(sec[i].Name, ".text", 6)) {
                beg = reinterpret_cast<uintptr_t>(image) + sec[i].VirtualAddress;
                end = beg + sec[i].Misc.VirtualSize;
                return true;
            }
    } __except (EXCEPTION_EXECUTE_HANDLER) {
    }
    return false;
}

const char* locate_guarded(uintptr_t tb, uintptr_t te)
{
    __try {
        return locate(tb, te, game_build(), g_anchor, g_unit);
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return "reading the game's code faulted";
    }
}
}

bool verify()
{
    uintptr_t tb = 0, te = 0;
    const char* why = text_range(tb, te) ? locate_guarded(tb, te) : "no .text section";
    if (why) {
        swlog("Lua unlocked: %s (build %08x, audited %08x); microcontroller Lua stays under the lock", why, game_build(), kBuild);
        return false;
    }
    g_allowed.clear();
    g_names.clear();
    for (const Named& n : kNamed) {
        if (n.allowed) g_allowed.push_back(g_unit[n.unit]);
        g_names.push_back({g_unit[n.unit], n.name});
    }
    for (const LockedName& n : kLockedNames) g_names.push_back({g_anchor[n.anchor] + intptr_t(n.delta), n.name});
    std::sort(g_allowed.begin(), g_allowed.end());
    std::sort(g_names.begin(), g_names.end(), [](const NameAt& a, const NameAt& b) { return a.at < b.at; });
    g_verified = true;
    return true;
}

size_t unit_count() { return kNumUnits; }

bool allowed(uintptr_t f) { return std::binary_search(g_allowed.begin(), g_allowed.end(), f); }

const char* name_of(uintptr_t f)
{
    auto it = std::lower_bound(g_names.begin(), g_names.end(), f, [](const NameAt& a, uintptr_t v) { return a.at < v; });
    return it != g_names.end() && it->at == f ? it->name : nullptr;
}

uintptr_t timeout_hook() { return g_verified ? g_unit[kTimeoutHookUnit] : 0; }
uintptr_t alloc_fn() { return g_verified ? g_unit[kAllocUnit] : 0; }
uintptr_t caller() { return g_verified ? g_unit[kCallerUnit] : 0; }
uintptr_t http_tail() { return g_verified ? g_unit[kHttpTailUnit] : 0; }

// The 6-byte C-function call becomes a 6-byte call through the stub's pointer slot (same return address). The C
// function returns straight to the Lua runtime, so nothing of ours is on the stack while it runs and a Lua error out
// of it skips nothing.
bool patch(Check check, const std::atomic<int>* active)
{
    static_assert(sizeof(std::atomic<int>) == 4, "the stub compares a dword");
    if (!g_verified || !check || !active) return false;
    uint8_t* site = reinterpret_cast<uint8_t*>(g_unit[kPrecallUnit] + kPrecallSiteOffset);
    if (memcmp(site, kSiteBytes, sizeof kSiteBytes)) return false;
    uint8_t* mem = swml4::patch::allocate_near(reinterpret_cast<uintptr_t>(site), 4096);
    if (!mem) return false;
    const auto stub = stub_code(reinterpret_cast<uintptr_t>(active), reinterpret_cast<uintptr_t>(check));
    memcpy(mem + 16, stub.data(), stub.size());
    const uintptr_t stub_at = reinterpret_cast<uintptr_t>(mem + 16);
    memcpy(mem, &stub_at, 8); // the call's pointer slot
    FlushInstructionCache(GetCurrentProcess(), mem, 16 + stub.size());
    const int64_t disp = int64_t(reinterpret_cast<intptr_t>(mem)) - int64_t(reinterpret_cast<intptr_t>(site) + 6);
    if (disp < INT32_MIN || disp > INT32_MAX) return false;
    const auto call = site_code(int32_t(disp));
    return swml4::patch::write_code(site, call.data(), call.size()); // mod init: no Lua runs yet (6 bytes, not atomic)
}
}

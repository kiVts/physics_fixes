#include "lua_gate.h"
#include "lua_gate_verify.h"
#include "game.h"
#include "log.h"
#include "native_patch.h"
#include <windows.h>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <span>
#include <vector>

namespace lua_gate {
namespace {
uintptr_t g_anchor[kMaxAnchors] = {};
uintptr_t g_unit[kMaxUnits] = {};
const Contract* g_contract = nullptr;
std::vector<uintptr_t> g_allowed; // sorted
struct NameAt {
    uintptr_t at;
    const char* name;
};
std::vector<NameAt> g_names; // sorted by address
bool g_verified = false;
std::vector<uint8_t> g_copy;      // every unit's bytes at verify(), back to back
std::vector<size_t> g_copy_at;    // where unit i starts in g_copy
uint8_t g_site_now[6] = {};       // the precall call this mod wrote (patch())
bool g_patched = false;

bool copy_units()
{
    __try {
        const Contract& c = *g_contract;
        size_t at = 0;
        for (size_t i = 0; i < c.num_units; ++i) {
            g_copy_at[i] = at;
            memcpy(g_copy.data() + at, reinterpret_cast<const void*>(g_unit[i]), c.units[i].size);
            at += c.units[i].size;
        }
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

const char* recheck_guarded(HookOk hook_ok, Range* ok, size_t nok, uintptr_t* at, Note note)
{
    const Contract& c = *g_contract;
    const char* first = nullptr;
    __try {
        for (size_t i = 0; i < c.num_units; ++i) {
            const uint8_t* now = reinterpret_cast<const uint8_t*>(g_unit[i]);
            const uint8_t* was = g_copy.data() + g_copy_at[i];
            const size_t n = c.units[i].size;
            if (!memcmp(now, was, n)) continue;
            const uintptr_t detour = follow_hook(now);
            const Clock which = i == c.clock_unit ? Clock::clock : i == c.timer_unit ? Clock::timer : Clock::none;
            const bool accepted = detour && hook_ok(g_unit[i], detour, which);
            const size_t hooked = accepted ? (now[0] == 0xE9 ? 5 : 2) : 0;
            const size_t k = first_unexplained(now, was, n, g_unit[i], hooked, ok, nok);
            if (k == n) continue;
            const char* why = detour && !accepted && k < 5
                                  ? "an audited function is hooked by code that isn't this mod's own accepted detour, the SWML clock or the SDK flight-pass timer"
                                  : "an audited code unit changed after start-up";
            if (!first) {
                first = why;
                *at = g_unit[i] + k;
            }
            if (!note) return first;
            note(g_unit[i] + k, detour, why); // every unit, for the log
        }
        return first;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return "reading the game's code faulted";
    }
}

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

const char* audited_builds()
{
    static char text[16 * std::size(kContracts) + 1];
    size_t at = 0;
    for (const Contract* c : kContracts) at += snprintf(text + at, sizeof text - at, at ? ", %08x" : "%08x", c->build);
    return text;
}

const char* locate_guarded(uintptr_t tb, uintptr_t te)
{
    __try {
        return locate(tb, te, game_build(), &g_contract, g_anchor, g_unit);
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
        swlog("Lua unlocked: %s (build %08x; audited builds %s); microcontroller Lua stays under the lock", why, game_build(), audited_builds());
        return false;
    }
    g_allowed.clear();
    g_names.clear();
    const Contract& c = *g_contract;
    for (const Named& n : std::span(c.named, c.num_named)) {
        if (n.allowed) g_allowed.push_back(g_unit[n.unit]);
        g_names.push_back({g_unit[n.unit], n.name});
    }
    for (const LockedName& n : std::span(c.locked, c.num_locked)) g_names.push_back({g_anchor[n.anchor] + intptr_t(n.delta), n.name});
    std::sort(g_allowed.begin(), g_allowed.end());
    std::sort(g_names.begin(), g_names.end(), [](const NameAt& a, const NameAt& b) { return a.at < b.at; });
    size_t total = 0;
    for (size_t i = 0; i < c.num_units; ++i) total += c.units[i].size;
    g_copy.assign(total, 0);
    g_copy_at.assign(c.num_units, 0);
    if (!copy_units()) {
        swlog("Lua unlocked: copying the audited code faulted; microcontroller Lua stays under the lock");
        return false;
    }
    g_verified = true;
    return true;
}

uintptr_t hook_target(uintptr_t at)
{
    __try {
        return follow_hook(reinterpret_cast<const uint8_t*>(at));
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return 0;
    }
}

const char* recheck(HookOk hook_ok, uintptr_t window, const uint8_t* window_bytes, size_t size, uintptr_t* at, Note note)
{
    *at = 0;
    if (!g_verified) return "not verified";
    Range ok[2];
    size_t nok = 0;
    __try {
        const uintptr_t site = g_unit[g_contract->precall_unit] + g_contract->precall_site_offset;
        if (g_patched && !memcmp(reinterpret_cast<const void*>(site), g_site_now, sizeof g_site_now)) ok[nok++] = {site, site + sizeof g_site_now};
        if (window && window_bytes && !memcmp(reinterpret_cast<const void*>(window), window_bytes, size)) ok[nok++] = {window, window + size};
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return "reading the game's code faulted";
    }
    return recheck_guarded(hook_ok, ok, nok, at, note);
}

size_t unit_count() { return g_verified ? g_contract->num_units : 0; }

bool audited(uintptr_t a)
{
    if (!g_verified) return false;
    for (size_t i = 0; i < g_contract->num_units; ++i)
        if (a >= g_unit[i] && a < g_unit[i] + g_contract->units[i].size) return true;
    return false;
}

bool allowed(uintptr_t f) { return std::binary_search(g_allowed.begin(), g_allowed.end(), f); }

const char* name_of(uintptr_t f)
{
    auto it = std::lower_bound(g_names.begin(), g_names.end(), f, [](const NameAt& a, uintptr_t v) { return a.at < v; });
    return it != g_names.end() && it->at == f ? it->name : nullptr;
}

uintptr_t timeout_hook() { return g_verified ? g_unit[g_contract->timeout_hook_unit] : 0; }
uintptr_t alloc_fn() { return g_verified ? g_unit[g_contract->alloc_unit] : 0; }
uintptr_t caller() { return g_verified ? g_unit[g_contract->caller_unit] : 0; }
uintptr_t http_tail() { return g_verified ? g_unit[g_contract->http_tail_unit] : 0; }

// The 6-byte C-function call becomes a 6-byte call through the stub's pointer slot (same return address). The C
// function returns straight to the Lua runtime, so nothing of ours is on the stack while it runs and a Lua error out
// of it skips nothing.
bool patch(Check check, const std::atomic<int>* active)
{
    static_assert(sizeof(std::atomic<int>) == 4, "the stub compares a dword");
    if (!g_verified || !check || !active) return false;
    uint8_t* site = reinterpret_cast<uint8_t*>(g_unit[g_contract->precall_unit] + g_contract->precall_site_offset);
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
    if (!swml4::patch::write_code(site, call.data(), call.size())) return false; // mod init: no Lua runs yet (6 bytes, not atomic)
    memcpy(g_site_now, call.data(), sizeof g_site_now);
    g_patched = true;
    return true;
}
}

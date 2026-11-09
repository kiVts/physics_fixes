
#include "game.h"
#include "hooks.h"
#include "log.h"
#include "sdk.h"

#include <windows.h>
#include <atomic>
#include <cstring>
#include <vector>

namespace {

constexpr uintptr_t kCallback = 0x1405ed160, kDispatcherVt = 0x140ae0a30;
constexpr int kSlotNum = 8, kSlotGet = 9;
constexpr size_t kManifoldPoints = 0x5e0;

using Num_t = int(__fastcall*)(void*);
using Get_t = void*(__fastcall*)(void*, int);
Num_t g_num = nullptr;
Get_t g_get = nullptr;
uintptr_t g_cb_lo = 0, g_cb_hi = 0;
std::atomic<bool> g_on{false};

struct Snap {
    void* disp = nullptr;
    int real_n = 0;
    std::vector<void*> list;
};
thread_local Snap tl_snap;
std::atomic<uint64_t> g_calls{0}, g_total{0}, g_kept{0};

bool from_callback(void* ra)
{
    uintptr_t a = reinterpret_cast<uintptr_t>(ra);
    return a >= g_cb_lo && a < g_cb_hi;
}

int __fastcall num_detour(void* d)
{
    if (!g_on.load(std::memory_order_relaxed) || !from_callback(_ReturnAddress())) return g_num(d);
    Snap& s = tl_snap;
    s.disp = d;
    s.real_n = g_num(d);
    s.list.clear();
    for (int i = 0; i < s.real_n; i++) {
        void* m = g_get(d, i);
        int points;
        memcpy(&points, static_cast<uint8_t*>(m) + kManifoldPoints, 4);
        if (points > 0) s.list.push_back(m);
    }
    g_calls.fetch_add(1, std::memory_order_relaxed);
    g_total.fetch_add(s.real_n, std::memory_order_relaxed);
    g_kept.fetch_add(s.list.size(), std::memory_order_relaxed);
    return static_cast<int>(s.list.size());
}

void* __fastcall get_detour(void* d, int i)
{
    if (!g_on.load(std::memory_order_relaxed) || !from_callback(_ReturnAddress())) return g_get(d, i);
    Snap& s = tl_snap;
    if (s.disp != d || i < 0 || i >= static_cast<int>(s.list.size()) || g_num(d) != s.real_n) {
        if (g_on.exchange(false))
            swlog("contact skip: the manifolds changed during the callback (count %d -> %d); the full list from now on",
                  s.real_n, g_num(d));
        return g_get(d, i); // unreachable in practice: the callback reads the count once
    }
    return s.list[i];
}

DWORD WINAPI log_loop(void*)
{
    uint64_t lc = 0, lt = 0, lk = 0;
    for (;;) {
        Sleep(5000);
        uint64_t c = g_calls, t = g_total, k = g_kept;
        if (c == lc || !g_on) continue;
        swlog("contact skip 5s: %llu callbacks, %.0f manifolds each, %.0f with contacts walked (%.1f%% skipped)",
              static_cast<unsigned long long>(c - lc), double(t - lt) / double(c - lc), double(k - lk) / double(c - lc),
              t > lt ? 100.0 * double((t - lt) - (k - lk)) / double(t - lt) : 0.0);
        lc = c, lt = t, lk = k;
    }
}

bool install()
{
    if (!GetPrivateProfileIntW(L"contact", L"skip_empty", 1, settings_ini())) {
        swlog("contact skip: off in settings.ini");
        return true;
    }
    uintptr_t cb = game_live(kCallback), vt = game_live(kDispatcherVt);
    if (!cb || !vt) return false;
    DWORD64 image = 0;
    PRUNTIME_FUNCTION rf = RtlLookupFunctionEntry(cb, &image, nullptr);
    if (!rf || image + rf->BeginAddress != cb) {
        swlog("contact skip: the callback isn't where expected; off");
        return true;
    }
    g_cb_lo = cb;
    g_cb_hi = static_cast<uintptr_t>(image + rf->EndAddress);
    auto slots = reinterpret_cast<void**>(vt);
    // the slots must still be Bullet's two accessors
    const uint8_t kNum[] = {0x8B, 0x41, 0x14, 0xC3};
    const uint8_t kGet[] = {0x48, 0x8B, 0x41, 0x20, 0x48, 0x63, 0xD2};
    if (memcmp(slots[kSlotNum], kNum, sizeof kNum) || memcmp(slots[kSlotGet], kGet, sizeof kGet)) {
        swlog("contact skip: the dispatcher's manifold accessors aren't the expected code; off");
        return true;
    }
    g_num = reinterpret_cast<Num_t>(slots[kSlotNum]);
    g_get = reinterpret_cast<Get_t>(slots[kSlotGet]);
    DWORD old;
    if (!VirtualProtect(&slots[kSlotNum], 16, PAGE_READWRITE, &old)) return false;
    g_on = true;
    slots[kSlotNum] = reinterpret_cast<void*>(&num_detour);
    slots[kSlotGet] = reinterpret_cast<void*>(&get_detour);
    VirtualProtect(&slots[kSlotNum], 16, old, &old);
    CloseHandle(CreateThread(nullptr, 0, log_loop, nullptr, 0, nullptr));
    swlog("contact skip: on (the contact callback walks only manifolds with contacts; \"contact skip 5s\" lines)");
    return true;
}

HookRegistrar reg{{"contact skip", /*needs_known_build=*/true, &install}};

} // namespace

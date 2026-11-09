
#include "game.h"
#include "hooks.h"
#include "log.h"
#include "sdk.h"

#include <windows.h>
#include <tlhelp32.h>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

// render_mt.cpp: blocks from the parallel render prep's per-thread heaps (see there)
bool rp_arena_free(void* p);
void rp_free_hook_live();

namespace {

constexpr uintptr_t kAlignedMalloc = 0x1409b2d6c, kAlignedFree = 0x1409b2d50;
constexpr uintptr_t kMallocSites[] = {0x1402fc9b9, 0x1402fce96};
constexpr uintptr_t kFreeSites[] = {0x1402fce49, 0x1402fd291};

uintptr_t live(uintptr_t ghidra_address) { return game_live(ghidra_address); }

using Malloc_t = void*(__cdecl*)(size_t size, size_t align);
using Free_t = void(__cdecl*)(void* p);
Malloc_t g_malloc = nullptr;
Free_t g_free = nullptr; // the real _aligned_free (MinHook trampoline)

struct Pool {
    size_t size;
    uintptr_t lo = 0, hi = 0;
    std::vector<void*> free_slots;
    SRWLOCK lock = SRWLOCK_INIT;
};
Pool g_pools[2] = {{0x4800}, {0xc000}};
uint8_t* g_region = nullptr;
uintptr_t g_region_lo = 0, g_region_hi = 0;
std::atomic<uint64_t> g_from_pool{0}, g_heap{0}, g_back_here{0}, g_back_render{0};
std::atomic<bool> g_hook_live{false};

// Pool slots are handed out only once the _aligned_free hook is in place (its jump written at the
// function start): a slot reaching the real _aligned_free would corrupt the heap.
bool hook_live()
{
    if (g_hook_live.load(std::memory_order_relaxed)) return true;
    if (*reinterpret_cast<const uint8_t*>(live(kAlignedFree)) != 0xE9) return false;
    g_hook_live = true;
    return true;
}

void* __cdecl scratch_malloc(size_t size, size_t align)
{
    if (align <= 16 && hook_live())
        for (Pool& p : g_pools) {
            if (p.size != size) continue;
            void* slot = nullptr;
            AcquireSRWLockExclusive(&p.lock);
            if (!p.free_slots.empty()) {
                slot = p.free_slots.back();
                p.free_slots.pop_back();
            }
            ReleaseSRWLockExclusive(&p.lock);
            if (slot) {
                g_from_pool++;
                return slot;
            }
        }
    g_heap++;
    return g_malloc(size, align);
}

bool give_back(void* ptr)
{
    uintptr_t a = reinterpret_cast<uintptr_t>(ptr);
    if (a < g_region_lo || a >= g_region_hi) return false;
    for (Pool& p : g_pools)
        if (a >= p.lo && a < p.hi) {
            AcquireSRWLockExclusive(&p.lock);
            p.free_slots.push_back(ptr);
            ReleaseSRWLockExclusive(&p.lock);
            return true;
        }
    return true; // inside the region but not a slot start: never handed out, never freed by the heap
}

void __cdecl scratch_free(void* p)
{
    if (give_back(p)) g_back_here++;
    else if (!rp_arena_free(p)) g_free(p);
}

void __cdecl aligned_free_detour(void* p) // every _aligned_free in the game (the render side's too)
{
    if (give_back(p)) g_back_render++;
    else if (!rp_arena_free(p)) g_free(p);
}

// --- patching E8 call sites -------------------------------------------------------------------------------
uint8_t* g_stubs = nullptr;

uint8_t* alloc_near(uintptr_t target)
{
    SYSTEM_INFO si;
    GetSystemInfo(&si);
    uintptr_t gran = si.dwAllocationGranularity;
    for (uintptr_t d = gran; d < 0x70000000; d += gran) {
        for (uintptr_t a : {target - d, target + d}) {
            a &= ~(gran - 1);
            if (void* p = VirtualAlloc(reinterpret_cast<void*>(a), 4096, MEM_COMMIT | MEM_RESERVE, PAGE_EXECUTE_READWRITE))
                return static_cast<uint8_t*>(p);
        }
    }
    return nullptr;
}

// jmp qword ptr [rip+0]; dq target
uint8_t* write_stub(uint8_t* at, void* target)
{
    const uint8_t jmp[6] = {0xFF, 0x25, 0, 0, 0, 0};
    memcpy(at, jmp, 6);
    uintptr_t t = reinterpret_cast<uintptr_t>(target);
    memcpy(at + 6, &t, 8);
    return at;
}

bool site_ok(uintptr_t site, uintptr_t expected_target)
{
    const uint8_t* p = reinterpret_cast<const uint8_t*>(site);
    if (p[0] != 0xE8) return false;
    int32_t rel;
    memcpy(&rel, p + 1, 4);
    return site + 5 + rel == expected_target;
}

bool patch_site(uintptr_t site, const uint8_t* stub)
{
    int64_t rel = reinterpret_cast<intptr_t>(stub) - static_cast<intptr_t>(site + 5);
    if (rel < INT32_MIN || rel > INT32_MAX) return false;
    int32_t r = static_cast<int32_t>(rel);
    DWORD old;
    if (!VirtualProtect(reinterpret_cast<void*>(site), 5, PAGE_EXECUTE_READWRITE, &old)) return false;
    memcpy(reinterpret_cast<uint8_t*>(site) + 1, &r, 4);
    VirtualProtect(reinterpret_cast<void*>(site), 5, old, &old);
    FlushInstructionCache(GetCurrentProcess(), reinterpret_cast<void*>(site), 5);
    return true;
}

// Suspend every other thread of the process while the call sites are rewritten.
template <typename F>
bool with_threads_suspended(F&& fn)
{
    DWORD self = GetCurrentThreadId(), pid = GetCurrentProcessId();
    HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPTHREAD, 0);
    if (snap == INVALID_HANDLE_VALUE) return false;
    HANDLE suspended[512];
    int n = 0;
    THREADENTRY32 te{sizeof te};
    for (BOOL ok = Thread32First(snap, &te); ok && n < 512; ok = Thread32Next(snap, &te)) {
        if (te.th32OwnerProcessID != pid || te.th32ThreadID == self) continue;
        HANDLE h = OpenThread(THREAD_SUSPEND_RESUME, FALSE, te.th32ThreadID);
        if (!h) continue;
        if (SuspendThread(h) == static_cast<DWORD>(-1)) {
            CloseHandle(h);
            continue;
        }
        suspended[n++] = h;
    }
    CloseHandle(snap);
    bool r = fn();
    for (int i = 0; i < n; i++) {
        ResumeThread(suspended[i]);
        CloseHandle(suspended[i]);
    }
    return r;
}

DWORD WINAPI logger_thread(void*)
{
    for (;;) {
        Sleep(5000);
        uint64_t f = g_from_pool.exchange(0), h = g_heap.exchange(0), b = g_back_here.exchange(0), r = g_back_render.exchange(0);
        size_t free0, free1;
        AcquireSRWLockShared(&g_pools[0].lock);
        free0 = g_pools[0].free_slots.size();
        ReleaseSRWLockShared(&g_pools[0].lock);
        AcquireSRWLockShared(&g_pools[1].lock);
        free1 = g_pools[1].free_slots.size();
        ReleaseSRWLockShared(&g_pools[1].lock);
        if (f + h)
            swlog("render scratch 5s: %.0f buffers/s from the pool, %.0f/s from the heap (pool empty), %.0f/s back right away, %.0f/s back"
                  " from the render side; free slots %zu (18 KB) / %zu (48 KB)",
                  f / 5.0, h / 5.0, b / 5.0, r / 5.0, free0, free1);
    }
}

bool install()
{
    if (!GetPrivateProfileIntW(L"render", L"scratch_buffers", 1, settings_ini())) {
        swlog("render scratch: off in settings.ini");
        return true;
    }
    if (!game_build_matches()) return false;
    for (uintptr_t s : kMallocSites)
        if (!site_ok(live(s), live(kAlignedMalloc))) {
            swlog("render scratch: call site %llx isn't the expected call; off", static_cast<unsigned long long>(s));
            return true;
        }
    for (uintptr_t s : kFreeSites)
        if (!site_ok(live(s), live(kAlignedFree))) {
            swlog("render scratch: call site %llx isn't the expected call; off", static_cast<unsigned long long>(s));
            return true;
        }
    g_malloc = game_ptr<Malloc_t>(kAlignedMalloc);
    int slots = static_cast<int>(GetPrivateProfileIntW(L"render", L"pool_slots", 512, settings_ini()));
    if (slots < 16) slots = 16;
    size_t total = 0;
    for (Pool& p : g_pools) total += p.size * slots;
    g_region = static_cast<uint8_t*>(VirtualAlloc(nullptr, total, MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE));
    if (!g_region) {
        swlog("render scratch: no memory for the pool; off");
        return true;
    }
    uint8_t* at = g_region;
    for (Pool& p : g_pools) {
        p.lo = reinterpret_cast<uintptr_t>(at);
        p.free_slots.reserve(slots);
        for (int i = slots - 1; i >= 0; i--) p.free_slots.push_back(at + p.size * i);
        at += p.size * slots;
        p.hi = reinterpret_cast<uintptr_t>(at);
    }
    g_region_lo = reinterpret_cast<uintptr_t>(g_region);
    g_region_hi = reinterpret_cast<uintptr_t>(at);
    if (!hook_game(kAlignedFree, reinterpret_cast<void*>(&aligned_free_detour), reinterpret_cast<void**>(&g_free))) {
        swlog("render scratch: hooking _aligned_free FAILED; off");
        return true;
    }
    g_stubs = alloc_near(game_base());
    if (!g_stubs) {
        swlog("render scratch: no memory within 2 GB of the game; off");
        return true;
    }
    uint8_t* malloc_stub = write_stub(g_stubs, reinterpret_cast<void*>(&scratch_malloc));
    uint8_t* free_stub = write_stub(g_stubs + 16, reinterpret_cast<void*>(&scratch_free));
    FlushInstructionCache(GetCurrentProcess(), g_stubs, 32);
    bool ok = with_threads_suspended([&] {
        bool all = true;
        for (uintptr_t s : kMallocSites) all &= patch_site(live(s), malloc_stub);
        for (uintptr_t s : kFreeSites) all &= patch_site(live(s), free_stub);
        return all;
    });
    if (!ok) {
        swlog("render scratch: patching FAILED");
        return false;
    }
    if (HANDLE h = CreateThread(nullptr, 0, logger_thread, nullptr, 0, nullptr)) CloseHandle(h);
    swlog("render scratch: on (pool of %d x 18 KB + %d x 48 KB; 4 call sites in FUN_1402fc8f0 and _aligned_free; \"render scratch 5s\" lines)",
          slots, slots);
    return true;
}

HookRegistrar reg{{"render_scratch", /*needs_known_build=*/true, &install}};

} // namespace

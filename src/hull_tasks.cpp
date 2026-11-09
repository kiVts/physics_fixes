
#include "hull_rays.h"
#include "game.h"
#include "hooks.h"
#include "log.h"
#include "sdk.h"

#include <windows.h>
#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

namespace {

constexpr uintptr_t kQueueLoop = 0x14017d800;    // mm job queue worker (queue*)
constexpr uintptr_t kTaskStart = 0x1404bab00;    // (vehicle, queue, 12 more): queues the vehicle's hull tasks
constexpr uintptr_t kMtxLock = 0x14095ab30;      // _Mtx_lock wrapper (mtx) -> 0 ok
constexpr uintptr_t kMtxUnlock = 0x14095ab54;    // _Mtx_unlock (mtx)
constexpr uintptr_t kCndWait = 0x14095ae98;      // _Cnd_wait (cnd, mtx)
constexpr uintptr_t kCndBroadcast = 0x14095ae24; // _Cnd_broadcast (cnd)
constexpr uintptr_t kSetThreadName = 0x14017cca0;
constexpr uintptr_t kFree = 0x1409b2d50;         // the game's _aligned_free
constexpr uintptr_t kResize = 0x1408de0b0;       // ring resize (ring, new capacity): copies, resets head
constexpr uintptr_t kHullTaskVftable = 0x140b3ff28;
constexpr uintptr_t kLockSite = 0x14048c99d, kLockResume = 0x14048c9a4;
constexpr uintptr_t kUnlockSite = 0x14048caa1, kUnlockResume = 0x14048caaf;
constexpr uint8_t kLockBytes[] = {0x8B, 0x4B, 0x10, 0x44, 0x8B, 0x43, 0x08};
constexpr uint8_t kUnlockBytes[] = {0xF2, 0x41, 0x0F, 0x11, 0x40, 0x18, 0xEB, 0x06};
constexpr uintptr_t kRingFreeJump = 0x14048a817;
constexpr uint8_t kRingFreeBytes[] = {0x74, 0x1A};

using Loop_t = void (*)(uint8_t*);
using Start_t = void (*)(void*, void*, uint64_t, uint64_t, uint64_t, uint64_t, uint64_t, uint64_t, uint64_t, uint64_t, uint64_t,
                         uint64_t, uint64_t, uint64_t);
using Mtx_t = int (*)(void*);
using Unlock_t = void (*)(void*);
using CndWait_t = int (*)(void*, void*);
using Cnd_t = void (*)(void*);
using Name_t = void (*)(const char*);
using Free_t = void (*)(void*);
using Resize_t = void (*)(void*, uint32_t);

Loop_t g_loop_orig;
Start_t g_start_orig;
Mtx_t g_lock;
Unlock_t g_unlock;
CndWait_t g_wait;
Cnd_t g_broadcast;
Name_t g_set_name;
Free_t g_free;
Resize_t g_resize;
uintptr_t g_hull_vftable;
int g_threads = 4;
bool g_parallel = true;

uintptr_t live(uintptr_t a) { return game_live(a); }

void run_task(uint8_t* task)
{
    auto vt = *reinterpret_cast<void (***)(uint8_t*)>(task);
    hull_rays_join(); // the body's water rays may still be on hull_rays' pool (cheap when none are)
    vt[1](task);
    task[0xb0] = 0;
    g_lock(task + 0x10);
    InterlockedExchange(reinterpret_cast<volatile LONG*>(task + 8), 1);
    g_broadcast(task + 0x68);
    g_unlock(task + 0x10);
}

std::mutex g_owner_mx;
std::unordered_map<uintptr_t, uintptr_t> g_owner; // body -> vehicle scene object

constexpr uint32_t kMaxBodies = 1024;

uint32_t read_bodies(uintptr_t ring_addr, uintptr_t* out)
{
    __try {
        auto* ring = reinterpret_cast<const uint8_t*>(ring_addr);
        auto* data = *reinterpret_cast<const uintptr_t* const*>(ring);
        uint32_t cap = *reinterpret_cast<const uint32_t*>(ring + 8), head = *reinterpret_cast<const uint32_t*>(ring + 0xc),
                 count = *reinterpret_cast<const uint32_t*>(ring + 0x10);
        if (!data || !cap || count > cap || count > kMaxBodies) return 0;
        for (uint32_t i = 0; i < count; i++) out[i] = data[(head + i) % cap];
        return count;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return 0;
    }
}

void note_bodies(uintptr_t vehicle) // keyed by the vehicle's body ring: one per vehicle
{
    uintptr_t bodies[kMaxBodies];
    uint32_t n = read_bodies(vehicle, bodies);
    std::lock_guard<std::mutex> l(g_owner_mx);
    if (g_owner.size() > 65536) g_owner.clear(); // bodies come and go; the next tick fills it again
    for (uint32_t i = 0; i < n; i++) g_owner[bodies[i]] = vehicle;
}

void start_detour(void* vehicle, void* queue, uint64_t a3, uint64_t a4, uint64_t a5, uint64_t a6, uint64_t a7, uint64_t a8, uint64_t a9,
                  uint64_t a10, uint64_t a11, uint64_t a12, uint64_t a13, uint64_t a14)
{
    if (g_parallel && vehicle) note_bodies(reinterpret_cast<uintptr_t>(vehicle));
    g_start_orig(vehicle, queue, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14);
}

// --- pool --------------------------------------------------------------------------------------------------------
// One group per vehicle with tasks queued or running; a group runs on one thread at a time, its tasks in order.
struct Group {
    std::deque<uint8_t*> tasks;
    bool running = false;
};
std::mutex g_pm;
std::condition_variable g_pcv, g_idle;
std::unordered_map<uintptr_t, Group> g_groups;
std::deque<uintptr_t> g_ready; // groups with tasks and no thread
int g_pending = 0, g_started = 0, g_busy_threads = 0;

// stats (under g_pm)
uint64_t g_n_hull = 0, g_n_other = 0, g_n_untagged = 0, g_busy_ticks = 0, g_peak = 0;
int64_t g_window = 0;
LARGE_INTEGER g_freq{};

int64_t qpc()
{
    LARGE_INTEGER t;
    QueryPerformanceCounter(&t);
    return t.QuadPart;
}

void pool_thread(int index)
{
    using SetDesc_t = HRESULT(WINAPI*)(HANDLE, PCWSTR);
    if (auto set = reinterpret_cast<SetDesc_t>(GetProcAddress(GetModuleHandleW(L"kernel32.dll"), "SetThreadDescription"))) {
        wchar_t name[32];
        swprintf(name, 32, L"hull_task_%d", index);
        set(GetCurrentThread(), name);
    }
    std::unique_lock<std::mutex> l(g_pm);
    for (;;) {
        g_pcv.wait(l, [] { return !g_ready.empty(); });
        uintptr_t key = g_ready.front();
        g_ready.pop_front();
        Group& g = g_groups[key];
        g.running = true;
        g_busy_threads++;
        g_peak = std::max<uint64_t>(g_peak, g_busy_threads);
        while (!g.tasks.empty()) {
            uint8_t* task = g.tasks.front();
            g.tasks.pop_front();
            l.unlock();
            int64_t t0 = qpc();
            run_task(task); // task is gone after this
            int64_t dt = qpc() - t0;
            l.lock();
            g_busy_ticks += uint64_t(dt);
            g_pending--;
        }
        g_busy_threads--;
        g_groups.erase(key); // g is gone too
        if (g_pending == 0) g_idle.notify_all();
    }
}

void pool_submit(uintptr_t key, uint8_t* task)
{
    std::lock_guard<std::mutex> l(g_pm);
    while (g_started < g_threads) std::thread(pool_thread, g_started++).detach();
    Group& g = g_groups[key];
    bool was_empty = g.tasks.empty();
    g.tasks.push_back(task);
    g_pending++;
    if (was_empty && !g.running) {
        g_ready.push_back(key);
        g_pcv.notify_one();
    }
}

void pool_wait_idle()
{
    std::unique_lock<std::mutex> l(g_pm);
    g_idle.wait(l, [] { return g_pending == 0; });
}

void maybe_log_locked()
{
    int64_t now = qpc();
    if (!g_window) g_window = now;
    double secs = double(now - g_window) / double(g_freq.QuadPart);
    if (secs < 5) return;
    swlog("hull tasks 5s: %.0f/s on %d threads (%.0f/s untagged ran alone, %.0f/s other tasks alone), %.1f ms/s of task time, "
          "up to %llu threads at once",
          g_n_hull / secs, g_threads, g_n_untagged / secs, g_n_other / secs, double(g_busy_ticks) * 1000.0 / double(g_freq.QuadPart) / secs,
          static_cast<unsigned long long>(g_peak));
    g_n_hull = g_n_other = g_n_untagged = g_busy_ticks = g_peak = 0;
    g_window = now;
}

// --- the vehicle_tick queue's worker --------------------------------------------------------------------------
void tick_queue_loop(uint8_t* q)
{
    void* mtx = q + 0x38;
    void* cnd = q + 0x90;
    auto& data = *reinterpret_cast<uint8_t***>(q + 0x18);
    auto& cap = *reinterpret_cast<uint32_t*>(q + 0x20);
    auto& head = *reinterpret_cast<uint32_t*>(q + 0x24);
    auto& count = *reinterpret_cast<uint32_t*>(q + 0x28);
    volatile uint8_t& stop = q[0xd8];

    if (g_lock(mtx) != 0) return;
    for (;;) {
        while (count == 0 && !stop) g_wait(cnd, mtx);
        if (stop) break;
        uint8_t* task = data[head % cap];
        uint32_t old_cap = cap;
        head = head == cap - 1 ? 0 : head + 1;
        if (--count == 0) { // as the game's loop: an empty ring gives its buffer back (head reset: fast_spawn 1.7.1)
            if (old_cap) {
                g_free(data);
                data = nullptr;
                cap = 0;
            }
            head = 0;
        } else if (count < old_cap / 2 && old_cap > 32) {
            g_resize(q + 0x18, old_cap / 2);
        }
        *reinterpret_cast<uint8_t**>(q + 0x30) = task;
        g_unlock(mtx);

        uintptr_t vehicle = 0;
        bool hull = *reinterpret_cast<const uintptr_t*>(task) == g_hull_vftable;
        if (hull && g_parallel) {
            uintptr_t body = *reinterpret_cast<const uintptr_t*>(task + 0xb8);
            std::lock_guard<std::mutex> l(g_owner_mx);
            auto it = g_owner.find(body);
            if (it != g_owner.end()) vehicle = it->second;
        }
        if (vehicle) {
            pool_submit(vehicle, task);
            std::lock_guard<std::mutex> l(g_pm);
            g_n_hull++;
            maybe_log_locked();
        } else { // not a hull task, a body we don't know, or serial mode: after everything before it, alone
            pool_wait_idle();
            int64_t t0 = qpc();
            run_task(task);
            std::lock_guard<std::mutex> l(g_pm);
            g_busy_ticks += uint64_t(qpc() - t0);
            (hull ? (g_parallel ? g_n_untagged : g_n_hull) : g_n_other)++;
            maybe_log_locked();
        }
        *reinterpret_cast<uint8_t**>(q + 0x30) = nullptr;
        if (g_lock(mtx) != 0) return;
    }
    g_unlock(mtx);
    pool_wait_idle(); // the owner joins this thread and then frees the queue and its tasks
}

void loop_detour(uint8_t* q)
{
    bool ours = false;
    __try {
        const char* name = *reinterpret_cast<const char* const*>(q + 0xe0);
        ours = name && strcmp(name, "vehicle_tick") == 0;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        ours = false;
    }
    if (!ours) return g_loop_orig(q);
    g_set_name("vehicle_tick");
    swlog("hull tasks: queue %p (\"vehicle_tick\") worker started, %s", q,
          g_parallel ? "hull tasks on the pool, grouped by vehicle" : "serial (this loop, one task at a time)");
    tick_queue_loop(q);
}

uint8_t* alloc_near(uintptr_t target, size_t size)
{
    SYSTEM_INFO si;
    GetSystemInfo(&si);
    uintptr_t gran = si.dwAllocationGranularity;
    for (uintptr_t d = gran; d < 0x70000000; d += gran)
        for (uintptr_t a : {target - d, target + d}) {
            a &= ~(gran - 1);
            if (void* p = VirtualAlloc(reinterpret_cast<void*>(a), size, MEM_COMMIT | MEM_RESERVE, PAGE_EXECUTE_READWRITE))
                return static_cast<uint8_t*>(p);
        }
    return nullptr;
}

void put32(uint8_t* at, int32_t v) { memcpy(at, &v, 4); }
void put64(uint8_t* at, uint64_t v) { memcpy(at, &v, 8); }

// rel32 from the end of the instruction at `ins` (length `len`) to `target`
int32_t rip_rel(const uint8_t* ins, size_t len, const void* target)
{
    return static_cast<int32_t>(reinterpret_cast<intptr_t>(target) - reinterpret_cast<intptr_t>(ins + len));
}

bool patch_jump(uintptr_t site, const uint8_t* stub, size_t len)
{
    int64_t rel = reinterpret_cast<intptr_t>(stub) - static_cast<intptr_t>(site + 5);
    if (rel < INT32_MIN || rel > INT32_MAX) return false;
    uint8_t code[8];
    memset(code, 0x90, sizeof code);
    code[0] = 0xE9;
    put32(code + 1, static_cast<int32_t>(rel));
    DWORD old;
    if (!VirtualProtect(reinterpret_cast<void*>(site), len, PAGE_EXECUTE_READWRITE, &old)) return false;
    memcpy(reinterpret_cast<void*>(site), code, len); // mod init runs before the game loop starts
    VirtualProtect(reinterpret_cast<void*>(site), len, old, &old);
    FlushInstructionCache(GetCurrentProcess(), reinterpret_cast<void*>(site), len);
    return true;
}

// Lock byte at mem + 0; lock stub at mem + 64, unlock stub at mem + 128. No calls, no registers or flags changed
// beyond what the replaced code and the instruction after it set (the next instruction at both sites writes the flags).
bool patch_ring_lock()
{
    uintptr_t lock_site = live(kLockSite), unlock_site = live(kUnlockSite);
    if (memcmp(reinterpret_cast<const void*>(lock_site), kLockBytes, sizeof kLockBytes) != 0 ||
        memcmp(reinterpret_cast<const void*>(unlock_site), kUnlockBytes, sizeof kUnlockBytes) != 0) {
        swlog("hull tasks: FUN_14048ad30 differs from the expected build at the ring push; off");
        return false;
    }
    uint8_t* mem = alloc_near(lock_site, 4096);
    if (!mem) {
        swlog("hull tasks: no memory near the game for the ring lock; off");
        return false;
    }
    uint8_t* lockb = mem;
    uint8_t* a = mem + 64;
    {
        uint8_t c[] = {
            0x50,                               // push rax
            0xB0, 0x01,                         // spin: mov al, 1
            0x86, 0x05, 0, 0, 0, 0,             // xchg [rip + lock], al
            0x84, 0xC0,                         // test al, al
            0x74, 0x04,                         // jz got
            0xF3, 0x90,                         // pause
            0xEB, 0xF2,                         // jmp spin
            0x58,                               // got: pop rax
            0x8B, 0x4B, 0x10,                   // mov ecx, [rbx + 0x10]   (replaced)
            0x44, 0x8B, 0x43, 0x08,             // mov r8d, [rbx + 8]      (replaced)
            0xFF, 0x25, 0, 0, 0, 0,             // jmp [rip]
            0, 0, 0, 0, 0, 0, 0, 0,
        };
        memcpy(a, c, sizeof c);
        put32(a + 5, rip_rel(a + 3, 6, lockb));
        put64(a + 31, live(kLockResume));
    }
    uint8_t* b = mem + 128;
    {
        uint8_t c[] = {
            0xF2, 0x41, 0x0F, 0x11, 0x40, 0x18,
            0xC6, 0x05, 0, 0, 0, 0, 0x00,       // mov byte [rip + lock], 0
            0xFF, 0x25, 0, 0, 0, 0,             // jmp [rip]
            0, 0, 0, 0, 0, 0, 0, 0,
        };
        memcpy(b, c, sizeof c);
        put32(b + 8, rip_rel(b + 6, 7, lockb));
        put64(b + 19, live(kUnlockResume));
    }
    FlushInstructionCache(GetCurrentProcess(), mem, 4096);
    // unlock first: it only clears a byte nobody has set yet
    if (!patch_jump(unlock_site, b, sizeof kUnlockBytes) || !patch_jump(lock_site, a, sizeof kLockBytes)) {
        swlog("hull tasks: ring lock patch FAILED");
        return false;
    }
    return true;
}

bool patch_keep_ring()
{
    uintptr_t site = live(kRingFreeJump);
    if (memcmp(reinterpret_cast<const void*>(site), kRingFreeBytes, sizeof kRingFreeBytes) != 0) {
        swlog("hull tasks: FUN_14048a610 differs from the expected build at the ring free; rings freed every tick as before");
        return false;
    }
    DWORD old;
    if (!VirtualProtect(reinterpret_cast<void*>(site), 1, PAGE_EXECUTE_READWRITE, &old)) return false;
    *reinterpret_cast<uint8_t*>(site) = 0xEB;
    VirtualProtect(reinterpret_cast<void*>(site), 1, old, &old);
    FlushInstructionCache(GetCurrentProcess(), reinterpret_cast<void*>(site), 1);
    return true;
}

bool install()
{
    const wchar_t* ini = settings_ini();
    if (GetPrivateProfileIntW(L"hull_tasks", L"keep_ring", 1, ini) && patch_keep_ring())
        swlog("hull tasks: bodies keep their hull force ring's buffer between ticks (keep_ring=1)");
    wchar_t mode[32] = {};
    GetPrivateProfileStringW(L"hull_tasks", L"mode", L"parallel", mode, 32, ini);
    if (_wcsicmp(mode, L"off") == 0) {
        swlog("hull tasks: mode=off, the game's one worker thread");
        return true;
    }
    g_parallel = _wcsicmp(mode, L"serial") != 0;
    int threads = static_cast<int>(GetPrivateProfileIntW(L"hull_tasks", L"threads", 0, ini));
    g_threads = std::clamp(threads > 0 ? threads : 4, 1, 16);
    QueryPerformanceFrequency(&g_freq);
    g_lock = game_ptr<Mtx_t>(kMtxLock);
    g_unlock = game_ptr<Unlock_t>(kMtxUnlock);
    g_wait = game_ptr<CndWait_t>(kCndWait);
    g_broadcast = game_ptr<Cnd_t>(kCndBroadcast);
    g_set_name = game_ptr<Name_t>(kSetThreadName);
    g_free = game_ptr<Free_t>(kFree);
    g_resize = game_ptr<Resize_t>(kResize);
    g_hull_vftable = reinterpret_cast<uintptr_t>(game_ptr<void*>(kHullTaskVftable));
    if (g_parallel && !patch_ring_lock()) g_parallel = false; // without the lock only the serial loop is safe
    if (!hook_game(kTaskStart, reinterpret_cast<void*>(&start_detour), reinterpret_cast<void**>(&g_start_orig)) ||
        !hook_game(kQueueLoop, reinterpret_cast<void*>(&loop_detour), reinterpret_cast<void**>(&g_loop_orig)))
        return false;
    hull_rays_task_join_ready(); // every hull task now starts in run_task
    swlog("hull tasks: %s (\"hull tasks 5s\" lines)",
          g_parallel ? "vehicle bodies' air / water tasks on a pool, one vehicle per thread at a time" : "serial check mode");
    if (g_parallel) swlog("hull tasks: %d threads; the shared water-flow ring push in FUN_14048ad30 is locked", g_threads);
    return true;
}

HookRegistrar reg{{"hull tasks", /*needs_known_build=*/true, &install}};

} // namespace

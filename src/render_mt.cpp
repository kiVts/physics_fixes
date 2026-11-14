
#include "microopt_ab.h"

#include "game.h"
#include "join_wait.h"
#include "hooks.h"
#include "log.h"
#include "sdk.h"

#include <windows.h>
#include <intrin.h>
#include <malloc.h>
#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <new>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

bool rp_install_mesh_refs();

namespace {

constexpr uintptr_t kLoopEntry = 0x1406c797c, kLoopEntryNext = 0x1406c7983;
constexpr uintptr_t kLoopExit = 0x1406c97a8, kLoopExitNext = 0x1406c97af;
constexpr uint8_t kEntryBytes[7] = {0x48, 0x8D, 0x95, 0x50, 0x03, 0x00, 0x00};
constexpr uint8_t kExitBytes[7] = {0x49, 0x8D, 0x8F, 0x78, 0x07, 0x00, 0x00};
constexpr uintptr_t kConstXmm10 = 0x140baf8f0, kConstXmm11 = 0x140bb0984;
constexpr uintptr_t kAlignedMalloc = 0x1409b2d6c, kAlignedRealloc = 0x1409b2dec;
constexpr uintptr_t kRenderPrep = 0x1406bb400, kGroupCheck = 0x140546af0, kGameFree = 0x1409b2d50, kEventResize = 0x1408de0b0;
constexpr size_t kRbpFromRsp = 0x120;                            // lea rbp,[rsp-0x8838] before sub rsp,0x8958
constexpr size_t kFrameSize = 0x8958 + 0x40 + 8 + 15 * 8;          // locals, 8 pushes, return address, 15 arguments
constexpr size_t kListBytes = 0x988;
constexpr size_t kEventSlot = 0x38;
constexpr size_t kCallRoom = 256 * 1024;                           // stack below the copied frame for the loop's calls
constexpr uint32_t kMaxGroups = 1 << 15;

struct QueueDef {
    uint32_t off, rec;
    uintptr_t resize;
};
const QueueDef kQueues[] = {
    {0x60, 0x88, 0x1408e0a40},  {0x78, 0x88, 0x1408e0a40},  {0x90, 0x88, 0x1408e0a40},  {0xa8, 0x88, 0x1408e0a40},
    {0x1b0, 0xe8, 0x1408e0b50}, {0x1c8, 0xe8, 0x1408e0b50}, {0x1e0, 0x118, 0x1408e0610}, {0x1f8, 0x118, 0x1408e0610},
    {0x210, 0xe8, 0x1408e0b50}, {0x228, 0xe8, 0x1408e0b50}, {0x240, 0x118, 0x1408e0610}, {0x258, 0xe8, 0x1408e0b50},
    {0x318, 200, 0x1408e03f0},  {0x360, 200, 0x1408e03f0},  {0x378, 0xe0, 0x1408e02e0},  {0x3a8, 200, 0x1408e03f0},
    {0x3c0, 200, 0x1408e03f0},  {0x3d8, 200, 0x1408e03f0},  {0x468, 0xd8, 0x1408dffc0},  {0x510, 0x120, 0x1408e3c40},
    {0x558, 0x3c, 0x1408dfdb0}, {0x570, 0x3b8, 0x1408dfca0}, {0x5b8, 0x180, 0x1408e7510},
};
constexpr int kNQ = sizeof kQueues / sizeof kQueues[0];
constexpr int kEv = kNQ; // index of the event queue in snapshots

uintptr_t live(uintptr_t ghidra_address) { return game_live(ghidra_address); }

// the game's ring queue header {T* data, u32 capacity, u32 head, u32 count}
struct Ring {
    uintptr_t data;
    uint32_t cap, head, count, pad;
};
static_assert(sizeof(Ring) == 0x18, "ring header");

// --- registers / participants ---------------------------------------------------------------------------------
struct Ctx {
    uint64_t gpr[16];
    uint64_t xmm[32]; // xmm0..15, 16 bytes each
};
static_assert(offsetof(Ctx, xmm) == 0x80, "ctx layout");
enum { RAX, RCX, RDX, RBX, RSP, RBP, RSI, RDI, R8, R9, R10, R11, R12, R13, R14, R15 };

struct Seg {
    uint32_t group;
    uintptr_t entry;      // the object-list entry (vehicle) drawn
    uint64_t t0, cycles;  // time spent drawing it (next tick's assignment)
    uint32_t start[kNQ + 1], end[kNQ + 1];
};
struct Participant {
    uintptr_t host_rsp; // +0  (machine code)
    Ctx* clone;         // +8  (machine code)
    int index;
    alignas(16) uint8_t list[kListBytes];
    Ring ev;
    uint32_t group_counter;
    std::vector<Seg> segs;
    bool open;
    uint64_t cycles;
    uint64_t start_delay; // from the update thread's go to this participant starting
};
static_assert(offsetof(Participant, host_rsp) == 0 && offsetof(Participant, clone) == 8, "participant layout");

using Resize_t = void(__fastcall*)(Ring* q, uint32_t capacity);
using Free_t = void(__cdecl*)(void* p);
using GroupCheck_t = bool(__fastcall*)(uintptr_t entry);
using SwitchIn_t = void(__fastcall*)(Participant* p);
Free_t g_free = nullptr;
Resize_t g_resize[kNQ], g_ev_resize = nullptr;
GroupCheck_t g_group_orig = nullptr;
SwitchIn_t g_switch_in = nullptr;
uint8_t* g_switch_out = nullptr;

enum Mode { kOff, kSingle, kParallel };
Mode g_mode = kOff;
std::atomic<bool> g_disabled{false};
int g_threads = 1; // participants, the update thread included

thread_local Participant* tl_p = nullptr;
std::vector<Participant*> g_parts;
std::atomic<uint32_t> g_claim[kMaxGroups];
uint32_t g_stamp = 0;

// this tick's inputs (written by the update thread before the participants start)
Ctx g_main;
uintptr_t g_lo = 0, g_list = 0, g_evq = 0;
bool g_main_participates = true;

std::unordered_map<uintptr_t, double> g_cost;  // entry -> smoothed cycles
std::unordered_map<uintptr_t, int> g_owner;    // entry -> participant, this run
std::atomic<uint64_t> g_go_tsc{0};

// --- private heaps ------------------------------------------------------------------------------------------------
struct ArenaHdr {
    void* raw;
    HANDLE heap;
    size_t size;
    uint64_t tag; // kArenaTag ^ user pointer; at user-8
};
static_assert(sizeof(ArenaHdr) == 32, "arena header");
constexpr uint64_t kArenaTag = 0x53574D4C4152454Eull;
using AMalloc_t = void*(__cdecl*)(size_t size, size_t align);
using ARealloc_t = void*(__cdecl*)(void* p, size_t size, size_t align);
AMalloc_t g_amalloc_orig = nullptr;
ARealloc_t g_arealloc_orig = nullptr;
Free_t g_afree_orig = nullptr; // only when render_mt hooks _aligned_free itself (scratch_buffers=0)
HANDLE g_heaps[64];
int g_nheaps = 0;
bool g_heaps_wanted = false;   // the participants' heaps (parallel render prep)
HANDLE g_ser_heap = nullptr;   // the serializer's heap (serialize.cpp), also in g_heaps
std::atomic<bool> g_free_hook_live{false};
thread_local HANDLE tl_heap = nullptr; // set while a participant draws (parallel mode, free hook live)
std::atomic<uint64_t> g_arena_allocs{0};
bool g_upd_pool = false;
thread_local bool tl_upd = false; // set on the update thread by the loop callback once the free hook is live
std::atomic<uint64_t> g_upd_allocs{0}, g_upd_fallback{0};

void* arena_alloc(HANDLE h, size_t size, size_t align)
{
    if (align < 16) align = 16;
    if (align & (align - 1)) return nullptr;
    uint8_t* raw = static_cast<uint8_t*>(HeapAlloc(h, 0, size + align + sizeof(ArenaHdr)));
    if (!raw) return nullptr;
    uintptr_t user = (reinterpret_cast<uintptr_t>(raw) + sizeof(ArenaHdr) + align - 1) & ~(align - 1);
    ArenaHdr* hd = reinterpret_cast<ArenaHdr*>(user - sizeof(ArenaHdr));
    hd->raw = raw, hd->heap = h, hd->size = size, hd->tag = kArenaTag ^ user;
    g_arena_allocs.fetch_add(1, std::memory_order_relaxed);
    return reinterpret_cast<void*>(user);
}

ArenaHdr* arena_hdr(const void* p)
{
    uintptr_t u = reinterpret_cast<uintptr_t>(p);
    if (!u || (u & 15) || !g_nheaps) return nullptr;
    if (*reinterpret_cast<const uint64_t*>(u - 8) != (kArenaTag ^ u)) return nullptr; // the CRT's own pointer slot otherwise
    ArenaHdr* hd = reinterpret_cast<ArenaHdr*>(u - sizeof(ArenaHdr));
    for (int i = 0; i < g_nheaps; i++)
        if (hd->heap == g_heaps[i]) return hd;
    return nullptr;
}

constexpr size_t kSlabSize = 64 * 1024; // = VirtualAlloc's granularity, so a block's slab is its address & ~0xffff
constexpr uint32_t kSlabHdr = 64;
constexpr uint32_t kClassSize[] = {48,  64,  80,  96,  112, 128,  160,  192,  224,  256,  320,  384,
                                   448, 512, 640, 768, 1024, 1280, 1536, 2048, 2560, 3072, 4096};
constexpr int kClasses = sizeof(kClassSize) / sizeof(kClassSize[0]);
constexpr int kPools = 16; // threads beyond that share (still correct: every pool has its lock)
struct Pool;
struct Slab {
    Slab *next, *prev; // the class's partial list
    void* free;        // freed blocks
    Pool* pool;
    uint32_t cls, live, used, cap; // used: blocks handed out from the never-used end
    bool listed;
};
static_assert(sizeof(Slab) <= kSlabHdr, "slab header");
struct Pool {
    SRWLOCK lock = SRWLOCK_INIT;
    Slab* partial[kClasses] = {};
    uint32_t slabs = 0;
    uint64_t blocks = 0;
};
Pool g_pools[kPools];
std::atomic<int> g_next_pool{0};
thread_local Pool* tl_pool = nullptr;
const HANDLE kPoolMarker = reinterpret_cast<HANDLE>(&g_pools);

void slab_unlink(Slab* s)
{
    if (s->prev) s->prev->next = s->next;
    else s->pool->partial[s->cls] = s->next;
    if (s->next) s->next->prev = s->prev;
    s->next = s->prev = nullptr;
    s->listed = false;
}

void slab_link(Slab* s)
{
    Slab*& head = s->pool->partial[s->cls];
    s->prev = nullptr;
    s->next = head;
    if (head) head->prev = s;
    head = s;
    s->listed = true;
}

void* pool_alloc(size_t size, size_t align)
{
    if (align > 16) return nullptr;
    size_t need = size + sizeof(ArenaHdr);
    int c = 0;
    while (c < kClasses && kClassSize[c] < need) c++;
    if (c == kClasses) return nullptr;
    Pool* pl = tl_pool;
    if (!pl) pl = tl_pool = &g_pools[g_next_pool.fetch_add(1) % kPools];
    AcquireSRWLockExclusive(&pl->lock);
    Slab* s = pl->partial[c];
    if (!s) {
        s = static_cast<Slab*>(VirtualAlloc(nullptr, kSlabSize, MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE));
        if (!s) {
            ReleaseSRWLockExclusive(&pl->lock);
            return nullptr;
        }
        s->free = nullptr;
        s->pool = pl;
        s->cls = c, s->live = 0, s->used = 0, s->cap = (kSlabSize - kSlabHdr) / kClassSize[c];
        slab_link(s);
        pl->slabs++;
    }
    uint8_t* block;
    if (s->free) {
        block = static_cast<uint8_t*>(s->free);
        s->free = *reinterpret_cast<void**>(block);
    } else {
        block = reinterpret_cast<uint8_t*>(s) + kSlabHdr + static_cast<size_t>(s->used++) * kClassSize[c];
    }
    s->live++;
    pl->blocks++;
    if (!s->free && s->used == s->cap) slab_unlink(s); // full
    ReleaseSRWLockExclusive(&pl->lock);
    uintptr_t user = reinterpret_cast<uintptr_t>(block) + sizeof(ArenaHdr);
    ArenaHdr* hd = reinterpret_cast<ArenaHdr*>(block);
    hd->raw = block, hd->heap = kPoolMarker, hd->size = size, hd->tag = kArenaTag ^ user;
    return reinterpret_cast<void*>(user);
}

void pool_free(void* block)
{
    Slab* s = reinterpret_cast<Slab*>(reinterpret_cast<uintptr_t>(block) & ~(kSlabSize - 1));
    Pool* pl = s->pool; // set once when the slab was made; the slab stays until its pool frees it under this lock
    void* release = nullptr;
    AcquireSRWLockExclusive(&pl->lock);
    *static_cast<void**>(block) = s->free;
    s->free = block;
    s->live--;
    pl->blocks--;
    if (!s->listed) slab_link(s); // was full
    if (s->live == 0 && (s->next || s->prev)) { // empty, and not the class's last partial slab
        slab_unlink(s);
        pl->slabs--;
        release = s;
    }
    ReleaseSRWLockExclusive(&pl->lock);
    if (release) VirtualFree(release, 0, MEM_RELEASE);
}

void arena_release(ArenaHdr* hd)
{
    HANDLE h = hd->heap;
    void* raw = hd->raw;
    hd->tag = 0;
    if (h == kPoolMarker) pool_free(raw);
    else HeapFree(h, 0, raw);
}

void* __cdecl amalloc_detour(size_t size, size_t align)
{
    if (HANDLE h = tl_heap) {
        if (h == g_ser_heap)
            if (void* p = pool_alloc(size, align)) return p;
        if (void* p = arena_alloc(h, size, align)) return p;
    } else if (tl_upd && microopt_on()) {
        if (void* p = pool_alloc(size, align)) {
            g_upd_allocs.fetch_add(1, std::memory_order_relaxed);
            return p;
        }
        g_upd_fallback.fetch_add(1, std::memory_order_relaxed);
    }
    return g_amalloc_orig(size, align);
}

void* __cdecl arealloc_detour(void* p, size_t size, size_t align)
{
    ArenaHdr* hd = arena_hdr(p);
    if (!hd) return p || !tl_heap ? g_arealloc_orig(p, size, align) : amalloc_detour(size, align);
    if (!size) {
        arena_release(hd);
        return nullptr;
    }
    void* n = amalloc_detour(size, align);
    if (!n) return nullptr;
    memcpy(n, p, std::min(size, hd->size));
    arena_release(hd);
    return n;
}

void __cdecl afree_detour(void* p) // only when render_scratch doesn't hook _aligned_free
{
    if (ArenaHdr* hd = arena_hdr(p)) arena_release(hd);
    else g_afree_orig(p);
}

} // namespace

// render_scratch.cpp's _aligned_free hook asks first; true = it was an arena block (freed)
bool rp_arena_free(void* p)
{
    if (ArenaHdr* hd = arena_hdr(p)) {
        arena_release(hd);
        return true;
    }
    return false;
}
void rp_free_hook_live() { g_free_hook_live.store(true); }

namespace {
// _aligned_free hooked (by render_scratch.cpp or here)? Hooks go live after every mod installed: look for the jump.
bool free_hook_live()
{
    if (g_free_hook_live.load(std::memory_order_relaxed)) return true;
    if (*reinterpret_cast<const uint8_t*>(live(kGameFree)) != 0xE9) return false;
    g_free_hook_live.store(true);
    return true;
}
} // namespace

// serialize.cpp: while the vehicle / world serializer runs, this thread's _aligned_malloc comes from the serializer's
// heap (its blocks are freed from any thread by the _aligned_free hook). Returns the previous heap for the leave call.
void* rp_serialize_heap_enter()
{
    void* prev = tl_heap;
    if (g_ser_heap && free_hook_live()) tl_heap = g_ser_heap;
    return prev;
}
void rp_serialize_heap_leave(void* prev) { tl_heap = static_cast<HANDLE>(prev); }
bool rp_serialize_heap_ready() { return g_ser_heap && free_hook_live(); }
void rp_serialize_pool_stats(uint32_t* slabs, uint64_t* blocks)
{
    *slabs = 0;
    *blocks = 0;
    for (Pool& pl : g_pools) {
        AcquireSRWLockShared(&pl.lock);
        *slabs += pl.slabs;
        *blocks += pl.blocks;
        ReleaseSRWLockShared(&pl.lock);
    }
}

namespace {

// --- claiming vehicles ------------------------------------------------------------------------------------------
void snapshot(const Participant* p, uint32_t* out)
{
    for (int q = 0; q < kNQ; q++) out[q] = reinterpret_cast<const Ring*>(p->list + kQueues[q].off)->count;
    out[kEv] = p->ev.count;
}

void close_segment(Participant* p)
{
    if (!p->open) return;
    Seg& s = p->segs.back();
    snapshot(p, s.end);
    s.cycles = __rdtsc() - s.t0;
    p->open = false;
}

bool __fastcall group_detour(uintptr_t entry)
{
    bool r = g_group_orig(entry);
    Participant* p = tl_p;
    if (!p) return r;
    uint32_t g = p->group_counter++;
    if (!r) return false;
    bool mine;
    auto own = g_owner.find(entry);
    if (own != g_owner.end()) {
        mine = own->second == p->index;
        if (mine && g < kMaxGroups) g_claim[g].store(g_stamp, std::memory_order_relaxed);
    } else if (g < kMaxGroups) {
        uint32_t old = g_claim[g].load(std::memory_order_relaxed);
        mine = old != g_stamp && g_claim[g].compare_exchange_strong(old, g_stamp, std::memory_order_acq_rel);
    } else {
        mine = p->index == 0;
    }
    if (!mine) return false;
    close_segment(p);
    p->segs.emplace_back();
    Seg& s = p->segs.back();
    s.group = g;
    s.entry = entry;
    s.t0 = __rdtsc();
    snapshot(p, s.start);
    p->open = true;
    return true;
}

// called by the exit stub: the participant reaching the loop's end, or null (normal path)
extern "C" Participant* __fastcall rp_exit_participant() { return tl_p; }

// --- running one participant ------------------------------------------------------------------------------------
uint64_t rebase(uint64_t v, intptr_t delta, const Participant* p)
{
    if (v - g_lo < kFrameSize) return v + delta;
    if (v - g_list < kListBytes) return reinterpret_cast<uintptr_t>(p->list) + (v - g_list);
    if (v == g_evq) return reinterpret_cast<uintptr_t>(&p->ev);
    return v;
}

void prepare(Participant* p)
{
    // the shadow queues keep their arrays from run to run (no regrowing from empty, no heap traffic)
    Ring keep[kNQ];
    for (int q = 0; q < kNQ; q++) keep[q] = *reinterpret_cast<const Ring*>(p->list + kQueues[q].off);
    memcpy(p->list, reinterpret_cast<const void*>(g_list), kListBytes);
    for (int q = 0; q < kNQ; q++) {
        Ring* r = reinterpret_cast<Ring*>(p->list + kQueues[q].off);
        *r = Ring{keep[q].data, keep[q].cap, 0, 0, 0};
    }
    p->ev.head = p->ev.count = 0;
    p->group_counter = 0;
    p->segs.clear();
    p->open = false;
}

__declspec(noinline) void run_participant(Participant* p)
{
    uint64_t t0 = __rdtsc();
    p->start_delay = t0 - g_go_tsc.load(std::memory_order_relaxed);
    uint8_t* raw = static_cast<uint8_t*>(_alloca(kCallRoom + kFrameSize + 64));
    uintptr_t clo = (reinterpret_cast<uintptr_t>(raw) + kCallRoom + 15) & ~uintptr_t(15);
    intptr_t delta = static_cast<intptr_t>(clo) - static_cast<intptr_t>(g_lo);
    memcpy(reinterpret_cast<void*>(clo), reinterpret_cast<const void*>(g_lo), kFrameSize);
    uint64_t* q = reinterpret_cast<uint64_t*>(clo);
    for (size_t i = 0; i < kFrameSize / 8; i++) q[i] = rebase(q[i], delta, p);
    Ctx c = g_main;
    for (int r = 0; r < 16; r++) c.gpr[r] = rebase(g_main.gpr[r], delta, p);
    p->clone = &c;
    tl_p = p;
    if (g_mode == kParallel && g_heaps_wanted && free_hook_live() && p->index < g_nheaps)
        tl_heap = g_heaps[p->index];
    g_switch_in(p); // runs the game's loop on the copied frame; returns when it reaches the loop's end
    tl_heap = nullptr;
    tl_p = nullptr;
    close_segment(p);
    p->cycles = __rdtsc() - t0;
}

std::mutex g_mu;
std::condition_variable g_cv;
std::atomic<uint64_t> g_gen{0}, g_wake{0};
std::atomic<int> g_done{0};
std::vector<HANDLE> g_workers;
uint64_t g_tsc_per_ms = 3000000;
constexpr uint64_t kSpinMs = 6;

DWORD WINAPI worker_main(void* arg)
{
    Participant* p = static_cast<Participant*>(arg);
    uint64_t seen = 0, seen_wake = 0;
    for (;;) {
        {
            std::unique_lock<std::mutex> lock(g_mu);
            g_cv.wait(lock, [&] { return g_gen.load() != seen || g_wake.load() != seen_wake; });
            seen_wake = g_wake.load();
        }
        // Waiting for the loop: pure spin for 50 us, then give the core to any ready thread (the physics pool runs at
        // the same time) between checks instead of holding it for up to kSpinMs.
        uint64_t start = __rdtsc(), deadline = start + kSpinMs * g_tsc_per_ms, hot = start + g_tsc_per_ms / 20;
        while (g_gen.load(std::memory_order_acquire) == seen) {
            uint64_t now = __rdtsc();
            if (now >= deadline) break;
            if (now < hot) _mm_pause();
            else SwitchToThread();
        }
        if (g_gen.load(std::memory_order_acquire) == seen) continue; // no loop this time: sleep again
        seen = g_gen.load();
        run_participant(p);
        g_done.fetch_add(1, std::memory_order_acq_rel);
    }
}

extern "C" void __fastcall rp_prep_begins()
{
    if (g_mode != kParallel || g_disabled.load(std::memory_order_relaxed)) return;
    {
        std::lock_guard<std::mutex> lock(g_mu);
        g_wake.fetch_add(1);
    }
    g_cv.notify_all();
}

void assign_owners(int n)
{
    g_owner.clear();
    if (n <= 1 || g_cost.empty()) return;
    std::vector<std::pair<double, uintptr_t>> v;
    v.reserve(g_cost.size());
    for (const auto& c : g_cost) v.emplace_back(c.second, c.first);
    std::sort(v.begin(), v.end(), [](const auto& a, const auto& b) { return a.first > b.first; });
    double load[64] = {};
    for (const auto& c : v) {
        int k = 0;
        for (int i = 1; i < n; i++)
            if (load[i] < load[k]) k = i;
        g_owner[c.second] = k;
        load[k] += c.first;
    }
}

// --- merge ----------------------------------------------------------------------------------------------------------
struct SegRef {
    uint32_t group;
    const Participant* p;
    const Seg* s;
};
std::vector<SegRef> g_order;

void update_costs()
{
    std::unordered_map<uintptr_t, double> next;
    next.reserve(g_order.size() * 2);
    for (const auto& r : g_order) {
        auto it = g_cost.find(r.s->entry);
        double c = double(r.s->cycles);
        next[r.s->entry] = it == g_cost.end() ? c : 0.5 * it->second + 0.5 * c;
    }
    g_cost.swap(next);
}

void move_records(Ring* dst, Resize_t resize, uint32_t rec, int qi, bool events)
{
    uint64_t total = 0;
    for (const SegRef& r : g_order) total += r.s->end[qi] - r.s->start[qi];
    if (!total) return;
    if (dst->count + total > dst->cap) {
        uint64_t want = std::max<uint64_t>({uint64_t(dst->cap) * 2, dst->count + total, 0x20});
        resize(dst, static_cast<uint32_t>(want));
    }
    for (const SegRef& r : g_order) {
        const Ring* src = events ? &r.p->ev : reinterpret_cast<const Ring*>(r.p->list + kQueues[qi].off);
        for (uint32_t i = r.s->start[qi]; i < r.s->end[qi]; i++) {
            const uint8_t* from = reinterpret_cast<const uint8_t*>(src->data) + size_t((src->head + i) % src->cap) * rec;
            uint8_t* to = reinterpret_cast<uint8_t*>(dst->data) + size_t((dst->head + dst->count) % dst->cap) * rec;
            memcpy(to, from, rec);
            dst->count++;
        }
    }
}

// anything in a shadow outside the queue headers that differs from the real list = a write we didn't expect
bool shadow_clean(const Participant* p, size_t* where)
{
    const uint8_t* real = reinterpret_cast<const uint8_t*>(g_list);
    for (size_t i = 0; i < kListBytes; i++) {
        if (p->list[i] == real[i]) continue;
        bool header = false;
        for (const QueueDef& q : kQueues) header |= i >= q.off && i < q.off + sizeof(Ring);
        if (!header) {
            *where = i;
            return false;
        }
    }
    return true;
}

void reset_shadows(Participant* p)
{
    // records were moved (memcpy, ownership of their sub-arrays included): keep the arrays for the next run, empty
    for (const QueueDef& q : kQueues) {
        Ring* r = reinterpret_cast<Ring*>(p->list + q.off);
        r->head = r->count = 0;
    }
    p->ev.head = p->ev.count = 0;
}

// --- stats ------------------------------------------------------------------------------------------------------------
struct Stats {
    uint64_t runs = 0, original = 0, cycles = 0, merge_cycles = 0, groups = 0, records = 0, events = 0, busiest = 0;
    uint64_t work = 0, delay = 0, assigned = 0, claimed = 0;
} g_st;
ULONGLONG g_window = 0;
uint64_t g_window_tsc = 0;

void maybe_log()
{
    ULONGLONG now = GetTickCount64();
    if (!g_window) {
        g_window = now;
        g_window_tsc = __rdtsc();
        return;
    }
    if (now - g_window < 5000) return;
    double secs = (now - g_window) / 1000.0, cyc_ms = double(__rdtsc() - g_window_tsc) / (secs * 1000.0);
    uint64_t n = g_st.runs ? g_st.runs : 1;
    swlog("render prep loop 5s: %s on %d threads, %.0f runs/s (%.0f/s original loop), %.3f ms/run (merge %.3f ms, busiest thread %.3f ms), "
          "%.0f vehicles, %.0f records, %.0f events per run",
          g_mode == kParallel ? "parallel" : "single", g_mode == kParallel ? g_threads : 1, g_st.runs / secs, g_st.original / secs,
          g_st.cycles / cyc_ms / n, g_st.merge_cycles / cyc_ms / n, g_st.busiest / cyc_ms / n, double(g_st.groups) / n, double(g_st.records) / n,
          double(g_st.events) / n);
    if (g_mode == kParallel)
        swlog("  balance: busiest thread %.0f%% of the average, slowest start %.3f ms after go; %.0f vehicles assigned from last tick's times, %.0f "
              "claimed first-come",
              g_st.work ? 100.0 * g_st.busiest * g_threads / g_st.work : 0.0, g_st.delay / cyc_ms / n, double(g_st.assigned) / n, double(g_st.claimed) / n);
    if (g_mode == kParallel)
        swlog("  heaps: %s, %.0f allocations/run from the threads' own heaps", g_heaps_wanted ? (free_hook_live() ? "own heap per thread" : "WAITING for the _aligned_free hook") : "off (prep_heaps=0)",
              double(g_arena_allocs.exchange(0)) / n);
    g_st = Stats{};
    g_window = now;
    g_window_tsc = __rdtsc();
}

// --- the loop -------------------------------------------------------------------------------------------------------
bool g_frame_bad_logged = false;

extern "C" int __fastcall rp_run(Ctx* c)
{
    if (g_mode == kOff || g_disabled.load(std::memory_order_relaxed)) return 0;
    uint64_t t0 = __rdtsc();
    g_main = *c;
    g_lo = c->gpr[RSP];
    g_list = c->gpr[R13];
    bool ok = c->gpr[RBP] == g_lo + kRbpFromRsp;
    if (ok) g_evq = *reinterpret_cast<const uint64_t*>(c->gpr[RBP] + kEventSlot);
    ok = ok && g_list && g_evq && !(g_evq - g_lo < kFrameSize);
    if (!ok) {
        if (!g_frame_bad_logged) {
            g_frame_bad_logged = true;
            swlog("render prep loop: the frame isn't laid out as expected (rbp %llx rsp %llx list %llx events %llx); running the original loop",
                  static_cast<unsigned long long>(c->gpr[RBP]), static_cast<unsigned long long>(g_lo), static_cast<unsigned long long>(g_list),
                  static_cast<unsigned long long>(g_evq));
        }
        g_st.original++;
        maybe_log();
        return 0;
    }
    // the update thread runs participant 0 on its own stack: the copied frame plus room for the loop's calls
    ULONG_PTR stack_lo = 0, stack_hi = 0;
    GetCurrentThreadStackLimits(&stack_lo, &stack_hi);
    if (reinterpret_cast<uintptr_t>(_AddressOfReturnAddress()) - stack_lo < kCallRoom + kFrameSize + 128 * 1024) {
        if (!g_frame_bad_logged) {
            g_frame_bad_logged = true;
            swlog("render prep loop: only %llu KB of stack left on the update thread; running the original loop",
                  static_cast<unsigned long long>((reinterpret_cast<uintptr_t>(_AddressOfReturnAddress()) - stack_lo) / 1024));
        }
        g_st.original++;
        maybe_log();
        return 0;
    }
    g_stamp++;
    if (g_stamp == 0) g_stamp = 1; // 0 = never claimed
    int n = g_mode == kParallel ? g_threads : 1;
    for (int i = 0; i < n; i++) prepare(g_parts[i]);
    assign_owners(n);

    g_go_tsc.store(__rdtsc(), std::memory_order_relaxed);
    if (n > 1) {
        g_done.store(0);
        {
            std::lock_guard<std::mutex> lock(g_mu);
            g_gen.fetch_add(1, std::memory_order_release); // spinning workers start at once; sleeping ones are woken
        }
        g_cv.notify_all();
    }
    run_participant(g_parts[0]); // the update thread is participant 0
    if (n > 1)
        wait_for_workers("render prep", [n] { return g_done.load(std::memory_order_acquire) == n - 1; });

    uint64_t tm = __rdtsc();
    g_order.clear();
    uint64_t busiest = 0, work = 0, delay = 0;
    for (int i = 0; i < n; i++) {
        busiest = std::max(busiest, g_parts[i]->cycles);
        work += g_parts[i]->cycles;
        if (i) delay = std::max(delay, g_parts[i]->start_delay);
        for (const Seg& s : g_parts[i]->segs) g_order.push_back(SegRef{s.group, g_parts[i], &s});
    }
    std::sort(g_order.begin(), g_order.end(), [](const SegRef& a, const SegRef& b) { return a.group < b.group; });
    for (int i = 0; i < n; i++) {
        size_t where = 0;
        if (!shadow_clean(g_parts[i], &where) && !g_disabled.exchange(true))
            swlog("render prep loop: participant %d changed the render list at +0x%zx outside the known queues; switching off after this tick", i, where);
    }
    uint64_t records = 0;
    for (int q = 0; q < kNQ; q++) {
        Ring* dst = reinterpret_cast<Ring*>(g_list + kQueues[q].off);
        uint32_t before = dst->count;
        move_records(dst, g_resize[q], kQueues[q].rec, q, false);
        records += dst->count - before;
    }
    Ring* evq = reinterpret_cast<Ring*>(g_evq);
    uint32_t ev_before = evq->count;
    move_records(evq, g_ev_resize, 8, kEv, true);
    for (int i = 0; i < n; i++) reset_shadows(g_parts[i]);
    uint64_t assigned = 0;
    for (const SegRef& r : g_order) assigned += g_owner.count(r.s->entry);
    update_costs();

    g_st.runs++;
    g_st.cycles += __rdtsc() - t0;
    g_st.merge_cycles += __rdtsc() - tm;
    g_st.busiest += busiest;
    g_st.work += work;
    g_st.delay += delay;
    g_st.assigned += assigned;
    g_st.claimed += g_order.size() - assigned;
    g_st.groups += g_order.size();
    g_st.records += records;
    g_st.events += evq->count - ev_before;
    maybe_log();
    return 1;
}

// --- machine code ------------------------------------------------------------------------------------------------------
struct Asm {
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
    // modrm for [base + disp32] with reg field r (SIB when the base is rsp / r12)
    void mem(int r, int base, int32_t disp)
    {
        *p++ = uint8_t(0x80 | ((r & 7) << 3) | (base & 7));
        if ((base & 7) == 4) *p++ = 0x24;
        d32(static_cast<uint32_t>(disp));
    }
    void mov_store(int base, int32_t disp, int reg) // mov [base+disp], reg64
    {
        *p++ = uint8_t(0x48 | (reg >= 8 ? 4 : 0) | (base >= 8 ? 1 : 0));
        *p++ = 0x89;
        mem(reg, base, disp);
    }
    void mov_load(int reg, int base, int32_t disp) // mov reg64, [base+disp]
    {
        *p++ = uint8_t(0x48 | (reg >= 8 ? 4 : 0) | (base >= 8 ? 1 : 0));
        *p++ = 0x8B;
        mem(reg, base, disp);
    }
    void movdqu(bool store, int x, int base, int32_t disp) // movdqu [base+disp], xmm / xmm, [base+disp]
    {
        *p++ = 0xF3;
        if (x >= 8 || base >= 8) *p++ = uint8_t(0x40 | (x >= 8 ? 4 : 0) | (base >= 8 ? 1 : 0));
        *p++ = 0x0F;
        *p++ = store ? 0x7F : 0x6F;
        mem(x, base, disp);
    }
    void mov_rax(uint64_t v) { b({0x48, 0xB8}), d64(v); }
    void jmp_abs(uint64_t v) { b({0xFF, 0x25, 0, 0, 0, 0}), d64(v); } // jmp [rip+0]
    uint8_t* align()
    {
        while (reinterpret_cast<uintptr_t>(p) & 15) *p++ = 0xCC;
        return p;
    }
};

constexpr int32_t kCtxAt = 0x20, kStubFrame = 0x200;

uint8_t* make_entry_stub(Asm& a)
{
    uint8_t* start = a.align();
    a.b({0x48, 0x81, 0xEC}), a.d32(kStubFrame); // sub rsp, 0x200
    for (int r = 0; r < 16; r++)
        if (r != RSP) a.mov_store(RSP, kCtxAt + 8 * r, r);
    a.b({0x48, 0x8D, 0x84, 0x24}), a.d32(kStubFrame);
    a.mov_store(RSP, kCtxAt + 8 * RSP, RAX);
    for (int x = 0; x < 16; x++) a.movdqu(true, x, RSP, kCtxAt + 0x80 + 16 * x);
    a.b({0x48, 0x8D, 0x8C, 0x24}), a.d32(kCtxAt);
    a.mov_rax(reinterpret_cast<uint64_t>(&rp_run));
    a.b({0xFF, 0xD0});                                // call rax
    a.b({0x85, 0xC0});                                // test eax, eax
    a.b({0x0F, 0x85}); uint8_t* jdone = a.p; a.d32(0); // jnz done
    // original loop: every register back, replay the patched instruction
    for (int x = 0; x < 16; x++) a.movdqu(false, x, RSP, kCtxAt + 0x80 + 16 * x);
    for (int r = 1; r < 16; r++)
        if (r != RSP) a.mov_load(r, RSP, kCtxAt + 8 * r);
    a.mov_load(RAX, RSP, kCtxAt);
    a.b({0x48, 0x81, 0xC4}), a.d32(kStubFrame);       // add rsp, 0x200
    a.b({0x48, 0x8D, 0x95, 0x50, 0x03, 0x00, 0x00});
    a.jmp_abs(live(kLoopEntryNext));
    // done: callee-saved registers are still the entry's (rp_run keeps them); reload the loop's constants
    uint8_t* done = a.p;
    int32_t rel = static_cast<int32_t>(done - (jdone + 4));
    memcpy(jdone, &rel, 4);
    a.b({0x48, 0x81, 0xC4}), a.d32(kStubFrame);       // add rsp, 0x200
    a.mov_rax(live(kConstXmm10));
    a.b({0xF3, 0x44, 0x0F, 0x10, 0x10});              // movss xmm10, [rax]
    a.mov_rax(live(kConstXmm11));
    a.b({0xF3, 0x44, 0x0F, 0x10, 0x18});              // movss xmm11, [rax]
    a.b({0x49, 0x8D, 0x8F, 0x78, 0x07, 0x00, 0x00});
    a.jmp_abs(live(kLoopExitNext));
    return start;
}

uint8_t* make_exit_stub(Asm& a)
{
    uint8_t* start = a.align();
    a.b({0x48, 0x83, 0xEC, 0x20});                    // sub rsp, 0x20 (the loop's rsp is 16-aligned)
    a.mov_rax(reinterpret_cast<uint64_t>(&rp_exit_participant));
    a.b({0xFF, 0xD0});                                // call rax
    a.b({0x48, 0x83, 0xC4, 0x20});                    // add rsp, 0x20
    a.b({0x48, 0x85, 0xC0});                          // test rax, rax
    a.b({0x74, 0x0F});                                // jz normal (+15)
    a.b({0x48, 0x89, 0xC1});                          // mov rcx, rax
    a.mov_rax(reinterpret_cast<uint64_t>(g_switch_out)); // 10 bytes
    a.b({0xFF, 0xE0});                                // jmp rax
    // normal: live here are only callee-saved registers and rsp (tools/live_in.py), so the call above is harmless
    a.b({0x49, 0x8D, 0x8F, 0x78, 0x07, 0x00, 0x00});
    a.jmp_abs(live(kLoopExitNext));
    return start;
}

// switch_in(Participant* p): save the host's callee-saved registers and rsp in p, load p->clone, enter the loop
uint8_t* make_switch_in(Asm& a)
{
    uint8_t* start = a.align();
    a.b({0x53, 0x55, 0x56, 0x57, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57}); // push rbx rbp rsi rdi r12-r15
    a.b({0x48, 0x81, 0xEC}), a.d32(0xA8);             // sub rsp, 0xa8 (16-aligned, room for xmm6-15)
    for (int x = 6; x < 16; x++) a.movdqu(true, x, RSP, 16 * (x - 6));
    a.mov_store(RCX, 0, RSP);                         // p->host_rsp = rsp
    a.mov_load(RAX, RCX, 8);                          // rax = p->clone
    for (int x = 0; x < 16; x++) a.movdqu(false, x, RAX, 0x80 + 16 * x);
    for (int r = 1; r < 16; r++)
        if (r != RSP) a.mov_load(r, RAX, 8 * r);
    a.mov_load(RSP, RAX, 8 * RSP);
    a.mov_load(RAX, RAX, 0);
    a.b({0x48, 0x8D, 0x95, 0x50, 0x03, 0x00, 0x00});
    a.jmp_abs(live(kLoopEntryNext));
    return start;
}

// switch_out (jumped to with rcx = participant at the loop's end): back to switch_in's caller
uint8_t* make_switch_out(Asm& a)
{
    uint8_t* start = a.align();
    a.mov_load(RSP, RCX, 0);                          // rsp = p->host_rsp
    for (int x = 6; x < 16; x++) a.movdqu(false, x, RSP, 16 * (x - 6));
    a.b({0x48, 0x81, 0xC4}), a.d32(0xA8);             // add rsp, 0xa8
    a.b({0x41, 0x5F, 0x41, 0x5E, 0x41, 0x5D, 0x41, 0x5C, 0x5F, 0x5E, 0x5D, 0x5B}); // pop r15-r12 rdi rsi rbp rbx
    a.b({0xC3});                                      // ret
    return start;
}

void* g_prep_tramp = nullptr;
uint8_t* make_prep_stub(Asm& a)
{
    uint8_t* start = a.align();
    a.b({0x51, 0x52, 0x41, 0x50, 0x41, 0x51});
    a.b({0x48, 0x83, 0xEC, 0x68});
    for (int x = 0; x < 4; x++) a.movdqu(true, x, RSP, 0x20 + 16 * x);
    a.mov_rax(reinterpret_cast<uint64_t>(&rp_prep_begins));
    a.b({0xFF, 0xD0});                                // call rax
    for (int x = 0; x < 4; x++) a.movdqu(false, x, RSP, 0x20 + 16 * x);
    a.b({0x48, 0x83, 0xC4, 0x68});                    // add rsp, 0x68
    a.b({0x41, 0x59, 0x41, 0x58, 0x5A, 0x59});
    a.mov_rax(reinterpret_cast<uint64_t>(&g_prep_tramp));
    a.b({0xFF, 0x20});                                // jmp [rax]
    return start;
}

uint8_t* alloc_near(uintptr_t target, size_t size)
{
    SYSTEM_INFO si;
    GetSystemInfo(&si);
    uintptr_t gran = si.dwAllocationGranularity;
    for (uintptr_t d = gran; d < 0x70000000; d += gran)
        for (uintptr_t a : {target - d, target + d}) {
            a &= ~(gran - 1);
            if (void* p = VirtualAlloc(reinterpret_cast<void*>(a), size, MEM_COMMIT | MEM_RESERVE, PAGE_EXECUTE_READWRITE)) return static_cast<uint8_t*>(p);
        }
    return nullptr;
}

bool patch_jump(uintptr_t at, const uint8_t* expect, uint8_t* to)
{
    uint8_t* site = reinterpret_cast<uint8_t*>(live(at));
    int64_t rel = reinterpret_cast<intptr_t>(to) - static_cast<intptr_t>(live(at) + 5);
    if (memcmp(site, expect, 7) != 0 || rel < INT32_MIN || rel > INT32_MAX) return false;
    uint8_t code[7] = {0xE9, 0, 0, 0, 0, 0x90, 0x90};
    int32_t r = static_cast<int32_t>(rel);
    memcpy(code + 1, &r, 4);
    DWORD old;
    if (!VirtualProtect(site, 7, PAGE_EXECUTE_READWRITE, &old)) return false;
    memcpy(site, code, 7); // mod init: before the game's loop runs
    VirtualProtect(site, 7, old, &old);
    FlushInstructionCache(GetCurrentProcess(), site, 7);
    return true;
}

// The private heaps and the _aligned_malloc / _aligned_realloc (/ _aligned_free) hooks: one heap per render prep
// participant (prep = parallel render prep with prep_heaps=1), plus one for the vehicle / world serializer
// (serialize.cpp, [serialize] private_heap=1), which runs whatever the render prep mode.
bool setup_heaps(const std::wstring& ini, bool prep)
{
    bool ser = GetPrivateProfileIntW(L"serialize", L"private_heap", 1, ini.c_str()) != 0;
    g_upd_pool = GetPrivateProfileIntW(L"heap", L"update_pool", 0, ini.c_str()) != 0;
    if (!prep && !ser && !g_upd_pool) return false;
    if (prep)
        for (int i = 0; i < g_threads && i < 62; i++)
            if (HANDLE h = HeapCreate(0, 1 << 20, 0)) g_heaps[g_nheaps++] = h;
    if (prep && g_nheaps != g_threads) return false;
    if (ser)
        if ((g_ser_heap = HeapCreate(0, 1 << 20, 0))) {
            g_heaps[g_nheaps++] = g_ser_heap; // blocks too big for the pool
            g_heaps[g_nheaps++] = kPoolMarker;
        }
    if (g_upd_pool && (!g_nheaps || g_heaps[g_nheaps - 1] != kPoolMarker)) g_heaps[g_nheaps++] = kPoolMarker;
    bool ok = hook_game(kAlignedMalloc, reinterpret_cast<void*>(&amalloc_detour), reinterpret_cast<void**>(&g_amalloc_orig)) &&
              hook_game(kAlignedRealloc, reinterpret_cast<void*>(&arealloc_detour), reinterpret_cast<void**>(&g_arealloc_orig));
    if (ok && !GetPrivateProfileIntW(L"render", L"scratch_buffers", 1, ini.c_str())) {
        // render_scratch.cpp doesn't hook _aligned_free: do it here
        ok = hook_game(kGameFree, reinterpret_cast<void*>(&afree_detour), reinterpret_cast<void**>(&g_afree_orig));
        if (ok) rp_free_hook_live();
    }
    if (!ok) {
        swlog("private heaps: hooking _aligned_malloc / _aligned_realloc FAILED; shared heap");
        g_ser_heap = nullptr;
        g_upd_pool = false;
        return false;
    }
    if (g_upd_pool) {
        // the update thread's flag: set once the _aligned_free hook (render_scratch's or ours) is in place
        bool loop_ok = swml()->on_loop(swml_self(), [](void*) {
                if (!tl_upd && free_hook_live()) tl_upd = true;
                static uint64_t last = 0;
                uint64_t now = GetTickCount64();
                if (!last) last = now;
                if (now - last < 30000) return;
                last = now;
                uint32_t slabs;
                uint64_t blocks;
                rp_serialize_pool_stats(&slabs, &blocks);
                swlog("update pool 30s: %llu blocks from the pool (%.0f/s), %llu too big (to the CRT); pools now %u slabs, %llu live blocks",
                      (unsigned long long)g_upd_allocs.load(), g_upd_allocs.load() / 30.0,
                      (unsigned long long)g_upd_fallback.load(), slabs, (unsigned long long)blocks);
                g_upd_allocs.store(0);
                g_upd_fallback.store(0);
            }, nullptr, nullptr, nullptr);
        if (!loop_ok) g_upd_pool = false;
        swlog(g_upd_pool ? "update pool: the update thread's small _aligned_malloc blocks from its slab pool (\"update pool 30s\" lines)"
                         : "update pool: no game loop events; off");
    }
    return prep;
}

bool install()
{
    std::wstring ini = settings_ini();
    wchar_t mode[32];
    GetPrivateProfileStringW(L"render", L"parallel_prep", L"parallel", mode, 32, ini.c_str());
    g_mode = !_wcsicmp(mode, L"single") ? kSingle : !_wcsicmp(mode, L"parallel") ? kParallel : kOff;
    if (g_mode == kOff) {
        setup_heaps(ini, false);
        swlog("render prep loop: off in settings.ini ([render] parallel_prep=parallel to draw vehicles on several threads)");
        return true;
    }
    int threads = static_cast<int>(GetPrivateProfileIntW(L"render", L"prep_threads", 0, ini.c_str()));
    if (threads <= 0) threads = std::min(6, std::max(1, static_cast<int>(std::thread::hardware_concurrency()) - 2));
    g_threads = g_mode == kParallel ? std::max(1, threads) : 1;

    // Install before patching the loop or starting any worker. Shared mesh ownership is not
    // covered by vehicle claims or the shadow-list check, even with prep_heaps disabled.
    if (g_mode == kParallel && !rp_install_mesh_refs()) {
        g_mode = kOff;
        return true;
    }

    g_free = reinterpret_cast<Free_t>(live(kGameFree));
    g_ev_resize = reinterpret_cast<Resize_t>(live(kEventResize));
    for (int q = 0; q < kNQ; q++) g_resize[q] = reinterpret_cast<Resize_t>(live(kQueues[q].resize));

    uint8_t* mem = alloc_near(game_base(), 4096);
    if (!mem) {
        swlog("render prep loop: no memory within 2 GB of the game; off");
        return true;
    }
    Asm a{mem};
    g_switch_out = make_switch_out(a);
    g_switch_in = reinterpret_cast<SwitchIn_t>(make_switch_in(a));
    uint8_t* exit_stub = make_exit_stub(a);
    uint8_t* entry_stub = make_entry_stub(a);
    uint8_t* prep_stub = make_prep_stub(a);
    FlushInstructionCache(GetCurrentProcess(), mem, a.p - mem);

    for (int i = 0; i < g_threads; i++) {
        auto* p = static_cast<Participant*>(_aligned_malloc(sizeof(Participant), 64));
        new (p) Participant();
        p->index = i;
        p->segs.reserve(512);
        g_parts.push_back(p);
    }
    if (!hook_game(kGroupCheck, reinterpret_cast<void*>(&group_detour), reinterpret_cast<void**>(&g_group_orig))) {
        swlog("render prep loop: hooking FUN_140546af0 FAILED; off");
        return false;
    }
    bool prep_heaps = g_mode == kParallel && GetPrivateProfileIntW(L"render", L"prep_heaps", 1, ini.c_str()) != 0;
    g_heaps_wanted = setup_heaps(ini, prep_heaps);
    if (prep_heaps && !g_heaps_wanted) swlog("render prep loop: per-thread heaps FAILED to set up; shared heap");
    if (g_mode == kParallel && !hook_game(kRenderPrep, prep_stub, &g_prep_tramp))
        swlog("render prep loop: hooking FUN_1406bb400 (early worker wake-up) FAILED; workers wake at the loop");
    {
        // cycle counter rate, for the workers' spin limit
        LARGE_INTEGER f, q0, q1;
        QueryPerformanceFrequency(&f);
        QueryPerformanceCounter(&q0);
        uint64_t t0 = __rdtsc();
        Sleep(20);
        QueryPerformanceCounter(&q1);
        double ms = double(q1.QuadPart - q0.QuadPart) * 1000.0 / double(f.QuadPart);
        if (ms > 0) g_tsc_per_ms = static_cast<uint64_t>(double(__rdtsc() - t0) / ms);
    }
    // exit first: once the entry jump exists, a participant must find its way out
    if (!patch_jump(kLoopExit, kExitBytes, exit_stub) || !patch_jump(kLoopEntry, kEntryBytes, entry_stub)) {
        swlog("render prep loop: the loop isn't the expected code; off");
        g_mode = kOff;
        return true;
    }
    for (int i = 1; i < g_threads; i++) {
        HANDLE h = CreateThread(nullptr, 8 << 20, worker_main, g_parts[i], STACK_SIZE_PARAM_IS_A_RESERVATION, nullptr);
        if (!h) {
            swlog("render prep loop: worker thread FAILED; %d threads", i);
            g_threads = i;
            break;
        }
        g_workers.push_back(h);
    }
    swlog("render prep loop: %s, %d threads (\"render prep loop 5s\" lines)", g_mode == kParallel ? "parallel" : "single (machinery check, one thread)",
          g_mode == kParallel ? g_threads : 1);
    return true;
}

HookRegistrar reg{{"render_prep_loop", /*needs_known_build=*/true, &install}};

} // namespace

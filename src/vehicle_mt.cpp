
#include "hull_rays.h"
#include "game.h"
#include "join_wait.h"
#include "hooks.h"
#include "log.h"
#include "sdk.h"
#include "vehicle_aero.h"
#include "sensor_sources.h"
#include "world.h"
#include "far_logic.h"

#include <windows.h>
#include <intrin.h>
#include <xmmintrin.h>
#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <array>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace {

constexpr uintptr_t kLoopPatch = 0x1405496e9, kLoopBody = 0x1405496ee, kLoopDone = 0x14054991a;
constexpr uint8_t kPatchBytes[5] = {0x33, 0xC9, 0x89, 0x4D, 0x84}; // xor ecx,ecx; mov [rbp-0x7c],ecx
constexpr uintptr_t kBodyUpdate = 0x14049d7b0;
constexpr uintptr_t kVtConnector = 0x140b50988, kVtDoorDock = 0x140af90e8, kVtRopeHook = 0x140b600f0;
constexpr size_t kConnectorPartner = 0x950, kDockPartner = 0x300, kRopePartnerA = 0xa98, kRopePartnerB = 0xaa0;
constexpr uintptr_t kRayStackLoad = 0x1400ccbbc;
constexpr uint8_t kRayStackBytes[7] = {0x48, 0x8B, 0xB1, 0xF8, 0x00, 0x00, 0x00};
constexpr uintptr_t kRaySinks[] = {0x1401a4300, 0x1401a4710, 0x1400de610};
bool g_ray_patched = false;

enum SinkKind {
    kAlways, // always locked, for the call
    kQueue,  // push helper: locked when rcx points into the world object, until the current tick returns (it may
             // hand its caller a slot to fill)
    kResize, // resize: locked when rcx points into the world object; until the tick returns only when called from
             // unlocked code (a locked caller fills the slot before it returns itself)
    kWriter,
    kReader, // only reads world state: shared lock for the call when rcx..r9 points into the world object;
             // exclusive when argument out_arg (the one it writes) points into the world
};
struct Sink {
    uintptr_t addr;
    const char* name;
    SinkKind kind = kAlways;
    int out_arg = 0; // kReader: 1-based argument it writes through, 0 = none
    bool new_build = false; // addr is the signatures' build's own address (vehicle_sinks_ported.inc), found by game_live_native
};
const Sink kSinks[] = {
#include "vehicle_sinks.inc"
#include "vehicle_queue_sinks.inc"
#include "vehicle_sinks_ported.inc"
};
constexpr int kNumSinks = sizeof kSinks / sizeof kSinks[0];

uintptr_t live(uintptr_t ghidra_address) { return game_live(ghidra_address); }

enum Mode { kOff, kSerial, kGrouped, kParallel };
Mode g_mode = kOff;
int g_threads = 0, g_min_vehicles = 8, g_grouped_first_s = 0;
int g_lod_logic = 0, g_lod_n = 3; // [lod] logic (0 off, 1 on, 2 compare), logic_n
double g_lod_far2 = 1200.0 * 1200.0, g_lod_near2 = 1000.0 * 1000.0;
std::atomic<VehicleAeroObserver> g_aero_observer{nullptr};
std::atomic<bool> g_aero_observer_ready{false};
ULONGLONG g_parallel_from = 0; // GetTickCount64 when parallel mode may start (after the grouped warm-up)

// --- the game's calls -----------------------------------------------------------------------------------------
using BodyUpdate_t = void(__fastcall*)(uintptr_t body, uintptr_t a2, uintptr_t a3, uintptr_t a4, uintptr_t a5, uintptr_t a6, uintptr_t a7);
using Tick_t = void(__fastcall*)(uintptr_t comp, uintptr_t rdx, uintptr_t r8, uintptr_t r9, uintptr_t s20, uintptr_t s28, float s30,
                                 uintptr_t s38, uintptr_t s40, uintptr_t s48, uintptr_t s50, uintptr_t s58, uint32_t s60, uint32_t s68,
                                 uintptr_t s70, uintptr_t s78, uintptr_t s80, uintptr_t s88, uintptr_t s90, uintptr_t s98, uint8_t sa0,
                                 uint8_t sa8, uintptr_t sb0, uintptr_t sb8, uintptr_t sc0);
BodyUpdate_t g_body_update = nullptr;
uintptr_t g_vt_conn = 0, g_vt_dock = 0, g_vt_rope = 0; // live vftable addresses (set in install)

struct Frame {
    uintptr_t list;
    uintptr_t p2, p3, p4, p5, p6, p7, p8;
    uintptr_t p9, p11, p12, p13, p14, p15, p19;
};
Frame g_f;

template <typename T>
T rd(uintptr_t a)
{
    return *reinterpret_cast<const T*>(a);
}
uintptr_t q_at(uintptr_t q, uint32_t i, size_t elem = 8) { return rd<uintptr_t>(q) + size_t((rd<uint32_t>(q + 0xc) + i) % rd<uint32_t>(q + 8)) * elem; }

inline void release_sticky();

// One vehicle, exactly as the original loop does it (re-reads counts the same way).
void run_vehicle(uintptr_t entry, uintptr_t veh)
{
    VehicleAeroObserver observer = g_aero_observer.load(std::memory_order_acquire);
    if (observer) observer(veh, true);
    const Frame& f = g_f;
    uint8_t sa8 = rd<uint8_t>(entry + 0x48);
    uint8_t sa0 = rd<uint8_t>(veh + 0x29c);
    uint32_t bi = 0;
    do {
        uintptr_t body = rd<uintptr_t>(q_at(veh, bi));
        float s30 = rd<float>(q_at(veh + 0x30, bi, 4));
        g_body_update(body, f.p12, f.p6, f.p7, f.p11, f.p8, f.list);
        release_sticky();
        if (rd<uintptr_t>(body + 0x718)) {
            uint32_t n = rd<uint32_t>(body + 0x460);
            for (uint32_t ci = 0; ci != n; ci++) {
                uintptr_t comp = rd<uintptr_t>(q_at(body + 0x450, ci));
                auto tick = rd<Tick_t>(rd<uintptr_t>(comp) + 0x358);
                tick(comp, f.p2, f.p4, f.p6, rd<uintptr_t>(body + 0x718), body + 0x720, s30, f.p5, f.p7, body + 0x14d0, f.p8, body + 0x5d0,
                     rd<uint32_t>(body + 0x264), rd<uint32_t>(body + 0x147c), f.p11, f.p12, f.p13, f.p14, f.p15, f.p9, sa0, sa8, f.p3, f.list, f.p19);
                release_sticky();
            }
        }
        bi++;
    } while (bi < rd<uint32_t>(veh + 0x10));
    if (observer) {
        observer(veh, false);
        // Aero can post its collected forces here through our usual queue wrappers.
        release_sticky();
    }
}

constexpr uintptr_t kZ33Patch = 0x14054993a, kZ33Body = 0x140549941, kZ33Done = 0x140549aed;
constexpr uint8_t kZ33Bytes[7] = {0x45, 0x33, 0xF6, 0x4C, 0x89, 0x75, 0xA0}; // xor r14d,r14d; mov [rbp-0x60],r14
constexpr uintptr_t kBodyPass = 0x1404baf80, kWaterSample = 0x1404cb790, kRingGrow32 = 0x1408dd1f0;
using U64 = uintptr_t;
using BodyPass_t = void(__fastcall*)(U64, U64, U64, U64, U64, U64, U64, U64, U64, U64, U64, U64, U64, U64, U64, U64, U64, U64, U64, U64);
using Water_t = void(__fastcall*)(U64, U64, U64, U64, U64, U64, U64, U64);
using Grow_t = void(__fastcall*)(void* ring);
using Vcall_t = void(__fastcall*)(U64);
BodyPass_t g_body_pass = nullptr;
Water_t g_water = nullptr;
Grow_t g_grow32 = nullptr;
struct Ring32 {
    uint8_t* data;
    uint32_t cap, head, count, pad;
};
std::vector<Ring32> g_acc; // per vehicle index (g_veh), reused across ticks
struct Z33Frame {
    U64 list, p2, p3, p4, p5, p7, p8, p9, p10, p12, p13, p19, p20, q18;
};
Z33Frame g_z;
bool g_z33_defer_vcalls = false;
int g_job = 0;                   // 0x33 while run_unit runs zone 0x33's loop
constexpr int kZ33Vts = 32;
std::atomic<uintptr_t> g_z33_vt[kZ33Vts];

void z33_note_vt(uintptr_t vt)
{
    for (int i = 0; i < kZ33Vts; i++) {
        uintptr_t cur = g_z33_vt[i].load(std::memory_order_relaxed);
        if (cur == vt) return;
        if (!cur) {
            uintptr_t expected = 0;
            if (g_z33_vt[i].compare_exchange_strong(expected, vt) || expected == vt) return;
        }
    }
}

void z33_vcalls(uintptr_t veh)
{
    for (uint32_t i = 0; i < rd<uint32_t>(veh + 0x60); i++) {
        uintptr_t obj = rd<uintptr_t>(rd<uintptr_t>(veh + 0x50) + size_t((rd<uint32_t>(veh + 0x5c) + i) % rd<uint32_t>(veh + 0x58)) * 0x30 + 0x18);
        if (!obj) continue;
        uintptr_t vt = rd<uintptr_t>(obj);
        z33_note_vt(vt);
        reinterpret_cast<Vcall_t>(rd<uintptr_t>(vt + 0x28))(obj);
        release_sticky();
    }
}

void run_vehicle33(uintptr_t entry, uintptr_t veh, Ring32* acc)
{
    const Z33Frame& f = g_z;
    g_body_pass(veh, f.p2, f.p3, f.p4, f.p5, veh + 0x190, f.p7, f.p8, f.p9, f.p10, veh + 0x30, f.p12, f.p13, rd<uint8_t>(veh + 0x29c),
                rd<uint8_t>(entry + 0x48), f.list + 0x70, reinterpret_cast<U64>(acc), rd<uint32_t>(veh + 0x178), f.p19, f.p20);
    release_sticky();
    for (int k = 0; k < 4; k++) *reinterpret_cast<uint64_t*>(veh + 0x140 + 8 * k) = 0;
    g_water(veh, f.p12, f.p7, f.p5, f.q18, f.p10, f.p7, f.list);
    release_sticky();
    if (!g_z33_defer_vcalls) z33_vcalls(veh);
}

bool eligible(uintptr_t entry, uintptr_t* veh)
{
    if (rd<int>(entry + 0x10) != 5) return false;
    uintptr_t v = rd<uintptr_t>(entry + 8);
    if (!v || rd<uint32_t>(v + 0x10) == 0 || rd<uint8_t>(entry) != 0) return false;
    *veh = v;
    return true;
}

// --- sink wrappers ------------------------------------------------------------------------------------------
// Default: one critical section, every caller exclusive (spins 4000 times before sleeping: most holds are short).
// [vehicle_logic] reader_lock=1: an SRW lock that readers (kReader) share. SRW locks aren't recursive and can't be
// upgraded, so each thread keeps its own mode and count; a reader that calls something needing the exclusive
// lock gives up its shared hold and waits for the exclusive one (counted: "upgrades"). Test 2026-10-20 15:46:
// 0 upgrades but 3.8-3.9 ms/tick vs 2.95 with the critical section (SRW didn't spin then; it does now).
CRITICAL_SECTION g_world_cs;
SRWLOCK g_world_lock = SRWLOCK_INIT;
bool g_reader_lock = false;
constexpr int kLockSpin = 4000;
enum LockMode { kUnlocked, kShared, kExclusive };
thread_local LockMode tl_mode = kUnlocked;
thread_local int tl_count = 0;
thread_local bool tl_par = false; // this thread runs vehicle units of a grouped / parallel phase
thread_local int tl_vehicle = -1; // index of the vehicle this thread is running
struct WrapFrame {
    uintptr_t ret;
    int sink;
    bool sticky;
    uint64_t t0;
};
constexpr int kMaxDepth = 64;
thread_local WrapFrame tl_frames[kMaxDepth];
thread_local int tl_depth = 0;
thread_local bool tl_sticky = false; // this thread keeps one lock count until its current tick returns
thread_local uint64_t tl_sticky_t0 = 0;
thread_local int tl_sticky_sink = 0;
uint8_t* g_post_stub = nullptr;
void* g_tramps[kNumSinks];
uintptr_t g_world_lo = 0, g_world_hi = 0; // the world object (set per tick in read_frame)
constexpr uintptr_t kWorldFromList = 0x16a68, kWorldSize = 845799; // tracer run 2026-10-20 13:45

std::atomic<uint64_t> g_sink_calls[kNumSinks], g_sink_cycles[kNumSinks], g_sticky_by_sink[kNumSinks], g_sticky_n_by_sink[kNumSinks], g_upgrades_by_sink[kNumSinks];
std::atomic<uint64_t> g_wait_cycles{0}, g_overflow{0}, g_sticky_holds{0}, g_sticky_cycles{0}, g_cond_free{0}, g_shared_calls{0}, g_upgrades{0};

// Known shared queues: count before / after each outermost locked call, attributed to the running vehicle.
struct Queue {
    const char* name;
    uintptr_t header; // 0 = unknown this tick
    uint32_t before;  // count at the start of the loop
    uint64_t tracked; // appended under the lock this tick
    uint32_t at_enter;
};
enum { kQPhysics, kQEngine, kQWeapon, kQProjectile, kQWeapon98, kQWeapon28, kQAudio, kQEffect20, kQEffect38, kQEffect68, kQEffect80, kQEffectC8, kNumQueues };
Queue g_queues[kNumQueues] = {{"physics events"}, {"engine (arg11+0x1178)"}, {"weapon (arg11+0x11f0)"}, {"projectile (arg14)"},
                              {"weapon (arg14+0x18)"}, {"weapon (arg14+0x30)"}, {"audio (arg10+0x10)"}, {"effect (arg12+0x20)"},
                              {"effect (arg12+0x38)"}, {"effect (arg12+0x68)"}, {"effect (arg12+0x80)"}, {"effect (arg12+0xc8)"}};
struct Posted {
    int vehicle;
    uint32_t n;
};
std::vector<Posted> g_posted; // physics events appended per outermost locked call, in lock order

uint32_t q_count(const Queue& q) { return q.header ? rd<uint32_t>(q.header + 0x10) : 0; }

bool in_world(uintptr_t a) { return a - g_world_lo < g_world_hi - g_world_lo; }

uintptr_t sink_arg(const uintptr_t* ret_slot, int n);

constexpr uintptr_t kSensorScan = 0x14090a290;
struct Snap {
    uintptr_t src = 0;
    struct {
        const uintptr_t* data;
        uint32_t cap, head, count, pad;
    } hdr{};
    std::vector<uintptr_t> buf, good; // good: buf without dead sources, only when there are any
};
bool g_snap_on = true;
constexpr int kMaxSnaps = 32;
Snap g_snaps[kMaxSnaps];
int g_nsnaps = 0;
std::mutex g_snap_mx;
std::vector<uintptr_t> g_snap_known; // list addresses seen by the scan (guarded by g_snap_mx)
std::atomic<uint64_t> g_snap_hits{0}, g_snap_misses{0};
uint64_t g_snap_ticks = 0, g_snap_grown = 0, g_snap_changed = 0, g_snap_logged = 0, g_snap_dead = 0, g_snap_dead_logged = 0;
int g_sensor_sink = -1;

bool copy_ring(uintptr_t src, Snap& sn)
{
    __try {
        auto data = rd<const uintptr_t*>(src);
        uint32_t cap = rd<uint32_t>(src + 8), head = rd<uint32_t>(src + 0xc), count = rd<uint32_t>(src + 0x10);
        if (count > cap || cap > (1u << 24) || (count && !data)) return false;
        sn.buf.resize(count);
        for (uint32_t i = 0; i < count; i++) sn.buf[i] = data[(head + i) % cap];
        sn.src = src;
        sn.hdr = {sn.buf.data(), std::max(1u, count), 0, count, 0};
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

void drop_dead_sources(Snap& sn)
{
    size_t dead = 0;
    for (uintptr_t source : sn.buf) dead += !sensor_source_ok(source);
    if (!dead) return;
    sn.good.clear();
    for (uintptr_t source : sn.buf) {
        if (sensor_source_ok(source)) {
            sn.good.push_back(source);
        } else if (g_snap_dead_logged++ < 20) {
            swlog("vehicle logic: sensor source %llx in list %llx has a dead character owner; left out of this tick's scan",
                  static_cast<unsigned long long>(source), static_cast<unsigned long long>(sn.src));
        }
    }
    sn.hdr = {sn.good.data(), std::max(1u, static_cast<uint32_t>(sn.good.size())), 0, static_cast<uint32_t>(sn.good.size()), 0};
    g_snap_dead += dead;
}

// Loop thread, before the workers start.
void snapshot_sensor_lists()
{
    g_nsnaps = 0;
    if (!g_snap_on) return;
    std::lock_guard<std::mutex> lk(g_snap_mx);
    for (uintptr_t a : g_snap_known)
        if (g_nsnaps < kMaxSnaps && in_world(a) && copy_ring(a, g_snaps[g_nsnaps])) drop_dead_sources(g_snaps[g_nsnaps++]);
    g_snap_ticks++;
}

// Loop thread, after the workers finished: did any copied list change while the copies were in use?
void check_sensor_lists()
{
    for (int i = 0; i < g_nsnaps; i++) {
        const Snap& sn = g_snaps[i];
        Snap live;
        if (!copy_ring(sn.src, live)) continue;
        if (live.buf == sn.buf) continue;
        bool prefix = live.buf.size() >= sn.buf.size() && std::equal(sn.buf.begin(), sn.buf.end(), live.buf.begin());
        (prefix ? g_snap_grown : g_snap_changed)++;
        if (!prefix && g_snap_logged++ < 10)
            swlog("vehicle logic: sensor source list %llx changed during the loop (%zu -> %zu entries, not just appended)",
                  static_cast<unsigned long long>(sn.src), sn.buf.size(), live.buf.size());
    }
}

// vl_sink_enter for the sensor scan: true = handled (runs on a copy, unlocked).
bool sensor_scan_unlocked(uintptr_t* ret_slot)
{
    uintptr_t list = ret_slot[-3];
    if (in_world(sink_arg(ret_slot, 7))) return false; // writes into the world: the normal exclusive path
    for (int i = 0; i < g_nsnaps; i++)
        if (g_snaps[i].src == list) {
            ret_slot[-3] = reinterpret_cast<uintptr_t>(&g_snaps[i].hdr);
            g_snap_hits.fetch_add(1, std::memory_order_relaxed);
            return true;
        }
    g_snap_misses.fetch_add(1, std::memory_order_relaxed);
    if (in_world(list)) {
        std::lock_guard<std::mutex> lk(g_snap_mx);
        if (g_snap_known.size() < kMaxSnaps && std::find(g_snap_known.begin(), g_snap_known.end(), list) == g_snap_known.end())
            g_snap_known.push_back(list);
    }
    return false;
}

// Appends to the known queues since the lock was taken (called right before its last release).
void count_appends()
{
    for (int i = 0; i < kNumQueues; i++) {
        Queue& q = g_queues[i];
        uint32_t now = q_count(q);
        if (now > q.at_enter) {
            q.tracked += now - q.at_enter;
            if (i == kQPhysics) g_posted.push_back(Posted{tl_vehicle, now - q.at_enter});
        }
    }
}

// SRW locks go to sleep almost at once; spin first like the critical section does.
template <typename Try>
bool spin_try(Try try_fn)
{
    for (int i = 0; i < kLockSpin; i++) {
        if (try_fn()) return true;
        _mm_pause();
    }
    return false;
}

void acquire_exclusive_raw()
{
    uint64_t t0 = __rdtsc();
    if (!g_reader_lock) {
        if (!TryEnterCriticalSection(&g_world_cs)) {
            EnterCriticalSection(&g_world_cs);
            g_wait_cycles += __rdtsc() - t0;
        }
    } else if (!TryAcquireSRWLockExclusive(&g_world_lock)) {
        if (!spin_try([] { return TryAcquireSRWLockExclusive(&g_world_lock) != 0; })) AcquireSRWLockExclusive(&g_world_lock);
        g_wait_cycles += __rdtsc() - t0;
    }
    for (Queue& q : g_queues) q.at_enter = q_count(q); // only exclusive holders append
}

void lock_acquire(bool exclusive, int sink)
{
    exclusive = exclusive || !g_reader_lock;
    if (tl_mode == kUnlocked) {
        if (exclusive) {
            acquire_exclusive_raw();
            tl_mode = kExclusive;
        } else {
            uint64_t t0 = __rdtsc();
            if (!TryAcquireSRWLockShared(&g_world_lock)) {
                if (!spin_try([] { return TryAcquireSRWLockShared(&g_world_lock) != 0; })) AcquireSRWLockShared(&g_world_lock);
                g_wait_cycles += __rdtsc() - t0;
            }
            tl_mode = kShared;
        }
        tl_count = 1;
        return;
    }
    if (exclusive && tl_mode == kShared) {
        // no in-place upgrade: drop the shared hold (a reader appended nothing) and wait for the exclusive one;
        // the outer reader frames then run on under the exclusive lock until the count drops to 0
        g_upgrades++;
        g_upgrades_by_sink[sink].fetch_add(1, std::memory_order_relaxed);
        ReleaseSRWLockShared(&g_world_lock);
        acquire_exclusive_raw();
        tl_mode = kExclusive;
    }
    tl_count++;
}

void lock_release()
{
    if (--tl_count) return;
    if (tl_mode == kExclusive) {
        count_appends();
        if (g_reader_lock) ReleaseSRWLockExclusive(&g_world_lock);
        else LeaveCriticalSection(&g_world_cs);
    } else {
        ReleaseSRWLockShared(&g_world_lock);
    }
    tl_mode = kUnlocked;
}

uintptr_t sink_arg(const uintptr_t* ret_slot, int n) { return n <= 4 ? ret_slot[-n] : ret_slot[n]; }

extern "C" void __fastcall vl_sink_enter(uintptr_t* ret_slot, int sink)
{
    if (!tl_par || tl_sticky) return; // tl_sticky: the exclusive lock is held until the tick returns already
    const Sink& sk = kSinks[sink];
    if (sink == g_sensor_sink && tl_mode == kUnlocked && sensor_scan_unlocked(ret_slot)) return;
    SinkKind kind = sk.kind;
    bool exclusive = true;
    if (kind != kAlways) {
        bool world = in_world(ret_slot[-1]);
        if (kind == kWriter || kind == kReader) world = world || in_world(ret_slot[-2]) || in_world(ret_slot[-3]) || in_world(ret_slot[-4]);
        if (!world) {
            g_cond_free.fetch_add(1, std::memory_order_relaxed);
            return; // this vehicle's own queue
        }
        if (kind == kReader) {
            exclusive = sk.out_arg && in_world(sink_arg(ret_slot, sk.out_arg));
            if (!exclusive) {
                if (tl_mode != kUnlocked) return; // already covered (shared or exclusive)
                if (g_reader_lock) g_shared_calls.fetch_add(1, std::memory_order_relaxed);
            }
        }
    }
    // keep the lock to the end of the tick: a push helper may hand out a slot; a resize called from unlocked code
    // is followed by its caller's unlocked fill
    bool sticky = kind == kQueue || (kind == kResize && tl_depth == 0);
    if (tl_depth >= kMaxDepth) {
        // no frame left: take the exclusive lock and keep it for the rest of the tick
        g_overflow++;
        lock_acquire(true, sink);
        tl_sticky = true;
        tl_sticky_t0 = __rdtsc();
        tl_sticky_sink = sink;
        g_sticky_holds++;
        return;
    }
    lock_acquire(exclusive, sink);
    tl_frames[tl_depth++] = WrapFrame{*ret_slot, sink, sticky, __rdtsc()};
    *ret_slot = reinterpret_cast<uintptr_t>(g_post_stub);
}

extern "C" uintptr_t __fastcall vl_sink_leave()
{
    WrapFrame fr = tl_frames[--tl_depth];
    if (tl_depth == 0) {
        g_sink_calls[fr.sink].fetch_add(1, std::memory_order_relaxed);
        g_sink_cycles[fr.sink].fetch_add(__rdtsc() - fr.t0, std::memory_order_relaxed);
    }
    if (fr.sticky && !tl_sticky) {
        // a slot handed out by a world queue may still be filled by the caller: keep this lock count
        tl_sticky = true;
        tl_sticky_t0 = __rdtsc();
        tl_sticky_sink = fr.sink;
        g_sticky_holds++;
        return fr.ret;
    }
    lock_release();
    return fr.ret;
}

// After every body update / component tick: drops the lock a world-queue call kept.
inline void release_sticky()
{
    if (!tl_sticky) return;
    tl_sticky = false;
    uint64_t held = __rdtsc() - tl_sticky_t0;
    g_sticky_cycles.fetch_add(held, std::memory_order_relaxed);
    g_sticky_by_sink[tl_sticky_sink].fetch_add(held, std::memory_order_relaxed);
    g_sticky_n_by_sink[tl_sticky_sink].fetch_add(1, std::memory_order_relaxed);
    lock_release();
}

// Entry stub per sink: saves the argument registers, calls vl_sink_enter(&return address, sink), restores
// them and jumps to the MinHook trampoline.
uint8_t* make_sink_stub(uint8_t* mem, int sink, void** tramp_slot)
{
    uint8_t code[] = {
        0x51, 0x52, 0x41, 0x50, 0x41, 0x51,
        0x48, 0x83, 0xEC, 0x68,                   // sub rsp, 0x68
        0xF3, 0x0F, 0x7F, 0x44, 0x24, 0x20,
        0xF3, 0x0F, 0x7F, 0x4C, 0x24, 0x30,
        0xF3, 0x0F, 0x7F, 0x54, 0x24, 0x40,
        0xF3, 0x0F, 0x7F, 0x5C, 0x24, 0x50,
        0x48, 0x8D, 0x8C, 0x24, 0x88, 0, 0, 0,
        0xBA, 0, 0, 0, 0,                         // mov edx, sink
        0x48, 0xB8, 0, 0, 0, 0, 0, 0, 0, 0,       // mov rax, vl_sink_enter
        0xFF, 0xD0,                               // call rax
        0xF3, 0x0F, 0x6F, 0x44, 0x24, 0x20,
        0xF3, 0x0F, 0x6F, 0x4C, 0x24, 0x30,
        0xF3, 0x0F, 0x6F, 0x54, 0x24, 0x40,
        0xF3, 0x0F, 0x6F, 0x5C, 0x24, 0x50,
        0x48, 0x83, 0xC4, 0x68,                   // add rsp, 0x68
        0x41, 0x59, 0x41, 0x58, 0x5A, 0x59,
        0x48, 0xB8, 0, 0, 0, 0, 0, 0, 0, 0,       // mov rax, tramp_slot
        0xFF, 0x20,                               // jmp qword ptr [rax]
    };
    int32_t s = sink;
    memcpy(&code[43], &s, 4);
    uintptr_t enter = reinterpret_cast<uintptr_t>(&vl_sink_enter), slot = reinterpret_cast<uintptr_t>(tramp_slot);
    memcpy(&code[49], &enter, 8);
    memcpy(&code[sizeof code - 10], &slot, 8);
    memcpy(mem, code, sizeof code);
    return mem + ((sizeof code + 15) & ~size_t(15));
}

uint8_t* make_post_stub(uint8_t* mem)
{
    uint8_t code[] = {
        0x48, 0x83, 0xEC, 0x08,                   // sub rsp, 8        (slot for the real return address)
        0x50, 0x52,
        0x48, 0x83, 0xEC, 0x48,                   // sub rsp, 0x48     (rsp 16-aligned for the call)
        0xF3, 0x0F, 0x7F, 0x44, 0x24, 0x20,
        0xF3, 0x0F, 0x7F, 0x4C, 0x24, 0x30,
        0x48, 0xB8, 0, 0, 0, 0, 0, 0, 0, 0,       // mov rax, vl_sink_leave
        0xFF, 0xD0,                               // call rax          -> rax = real return address
        0xF3, 0x0F, 0x6F, 0x44, 0x24, 0x20,
        0xF3, 0x0F, 0x6F, 0x4C, 0x24, 0x30,
        0x48, 0x83, 0xC4, 0x48,                   // add rsp, 0x48
        0x48, 0x89, 0x44, 0x24, 0x10,
        0x5A, 0x58,
        0xC3,                                     // ret
    };
    uintptr_t leave = reinterpret_cast<uintptr_t>(&vl_sink_leave);
    memcpy(&code[24], &leave, 8);
    memcpy(mem, code, sizeof code);
    return mem + ((sizeof code + 15) & ~size_t(15));
}

// A sink must start a real function: the begin of a .pdata entry that isn't a chained fragment.
bool is_function_start(uintptr_t a)
{
    DWORD64 base = 0;
    PRUNTIME_FUNCTION rf = RtlLookupFunctionEntry(a, &base, nullptr);
    if (!rf || base + rf->BeginAddress != a) return false;
    const uint8_t* unwind = reinterpret_cast<const uint8_t*>(base + rf->UnwindData);
    return ((unwind[0] >> 3) & 0x4) == 0; // UNW_FLAG_CHAININFO
}

struct alignas(16) RayStack {
    uint8_t bytes[0x20];
};
thread_local RayStack tl_ray_stack = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}};
extern "C" void* __fastcall vl_ray_stack() { return &tl_ray_stack; }

// Called from the patched load: returns this thread's stack in rsi, every other register kept.
uint8_t* make_ray_stub(uint8_t* mem)
{
    uint8_t code[] = {
        0x50, 0x51, 0x52, 0x41, 0x50, 0x41, 0x51, 0x41, 0x52, 0x41, 0x53,
        0x48, 0x83, 0xEC, 0x20,                                           // sub rsp, 0x20 (16-aligned: call + 7 pushes = 64)
        0x48, 0xB8, 0, 0, 0, 0, 0, 0, 0, 0,                               // mov rax, vl_ray_stack
        0xFF, 0xD0,                                                       // call rax
        0x48, 0x83, 0xC4, 0x20,                                           // add rsp, 0x20
        0x48, 0x89, 0xC6,                                                 // mov rsi, rax
        0x41, 0x5B, 0x41, 0x5A, 0x41, 0x59, 0x41, 0x58, 0x5A, 0x59, 0x58,
        0xC3,                                                             // ret
    };
    uintptr_t fn = reinterpret_cast<uintptr_t>(&vl_ray_stack);
    memcpy(&code[17], &fn, 8);
    memcpy(mem, code, sizeof code);
    return mem + ((sizeof code + 15) & ~size_t(15));
}

// --- units ------------------------------------------------------------------------------------------------------
struct Veh {
    uintptr_t entry, veh;
    int cost;    // components
    uint32_t id; // the game's vehicle id (object list entry +4): the same on every PC
};
std::vector<Veh> g_veh;
std::vector<int> g_parent;
struct Link {
    uintptr_t comp, partner;
    int vehicle;
    int kind; // 0 connector, 1 door dock, 2 rope hook
};
uint64_t g_link_unions[3], g_rope_unresolved; // this tick
std::vector<Link> g_links;
struct Unit {
    std::vector<int> vehicles; // ascending
    int cost;
    uint64_t cycles;
    int lod = 0;       // distant AI logic: 0 not eligible, 1 decimated, 2 control (compare mode)
    bool skip = false; // decimated and not its tick: not run, its impulses are replayed
};
std::vector<Unit> g_units;
std::vector<int> g_order; // units, costliest first

int find(int a)
{
    while (g_parent[a] != a) a = g_parent[a] = g_parent[g_parent[a]];
    return a;
}
void unite(int a, int b)
{
    a = find(a), b = find(b);
    if (a != b) g_parent[std::max(a, b)] = std::min(a, b);
}

void build_units()
{
    const uintptr_t vt_conn = g_vt_conn, vt_dock = g_vt_dock, vt_rope = g_vt_rope;
    g_links.clear();
    g_parent.resize(g_veh.size());
    int rope_first = -1;
    for (int v = 0; v < static_cast<int>(g_veh.size()); v++) {
        g_parent[v] = v;
        uintptr_t veh = g_veh[v].veh;
        int cost = 0;
        uint32_t nb = rd<uint32_t>(veh + 0x10);
        for (uint32_t bi = 0; bi < nb; bi++) {
            uintptr_t body = rd<uintptr_t>(q_at(veh, bi));
            uint32_t nc = rd<uint32_t>(body + 0x460);
            cost += 1 + nc;
            for (uint32_t ci = 0; ci < nc; ci++) {
                uintptr_t comp = rd<uintptr_t>(q_at(body + 0x450, ci));
                uintptr_t vt = rd<uintptr_t>(comp);
                if (vt == vt_conn) g_links.push_back(Link{comp, rd<uintptr_t>(comp + kConnectorPartner), v, 0});
                else if (vt == vt_dock) g_links.push_back(Link{comp, rd<uintptr_t>(comp + kDockPartner), v, 1});
                else if (vt == vt_rope) {
                    g_links.push_back(Link{comp, rd<uintptr_t>(comp + kRopePartnerA), v, 2});
                    g_links.push_back(Link{comp, rd<uintptr_t>(comp + kRopePartnerB), v, 2});
                }
            }
        }
        g_veh[v].cost = cost;
    }
    std::sort(g_links.begin(), g_links.end(), [](const Link& a, const Link& b) { return a.comp < b.comp; });
    g_link_unions[0] = g_link_unions[1] = g_link_unions[2] = g_rope_unresolved = 0;
    for (const Link& l : g_links) {
        if (!l.partner) continue;
        auto it = std::lower_bound(g_links.begin(), g_links.end(), l.partner, [](const Link& a, uintptr_t c) { return a.comp < c; });
        if (it != g_links.end() && it->comp == l.partner) {
            if (find(l.vehicle) != find(it->vehicle)) g_link_unions[l.kind]++;
            unite(l.vehicle, it->vehicle);
        } else if (l.kind == 2) {
            // a rope hook attached to something that isn't a rope hook of a ticking vehicle: keep all of
            // those together (the old rule), since the partner's owner isn't known
            g_rope_unresolved++;
            if (rope_first < 0) rope_first = l.vehicle;
            else unite(rope_first, l.vehicle);
        }
        // a connector / dock whose partner isn't on a ticking vehicle: nothing else runs that partner's logic
    }
    for (Unit& u : g_units) u.vehicles.clear();
    std::vector<int> unit_of(g_veh.size(), -1);
    int nu = 0;
    for (int v = 0; v < static_cast<int>(g_veh.size()); v++) {
        int r = find(v);
        if (unit_of[r] < 0) {
            unit_of[r] = nu++;
            if (static_cast<int>(g_units.size()) < nu) g_units.emplace_back();
            g_units[nu - 1].vehicles.clear();
            g_units[nu - 1].cost = 0;
        }
        Unit& u = g_units[unit_of[r]];
        u.vehicles.push_back(v);
        u.cost += g_veh[v].cost;
    }
    g_units.resize(nu);
    g_order.resize(nu);
    for (int i = 0; i < nu; i++) g_order[i] = i;
    std::sort(g_order.begin(), g_order.end(), [](int a, int b) { return g_units[a].cost > g_units[b].cost; });
}

bool build_units_raw()
{
    __try {
        build_units();
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

void run_unit(Unit& u)
{
    if (u.skip && g_job != 0x33) { // distant AI logic decimates zone 0x32 only
        u.cycles = 0;
        return;
    }
    uint64_t t0 = __rdtsc();
    for (int v : u.vehicles) {
        tl_vehicle = v;
        if (g_job == 0x33) run_vehicle33(g_veh[v].entry, g_veh[v].veh, &g_acc[v]);
        else run_vehicle(g_veh[v].entry, g_veh[v].veh);
    }
    release_sticky();
    tl_vehicle = -1;
    u.cycles = __rdtsc() - t0;
}

// --- worker pool ------------------------------------------------------------------------------------------------
std::mutex g_mu;
std::condition_variable g_cv;
uint64_t g_gen = 0;
std::atomic<int> g_next{0}, g_done{0};
unsigned g_mxcsr = 0;
std::vector<HANDLE> g_workers;

void run_units()
{
    for (;;) {
        int i = g_next.fetch_add(1, std::memory_order_relaxed);
        if (i >= static_cast<int>(g_order.size())) return;
        run_unit(g_units[g_order[i]]);
    }
}

DWORD WINAPI worker_main(void*)
{
    tl_par = true;
    uint64_t seen = 0;
    for (;;) {
        {
            std::unique_lock<std::mutex> lock(g_mu);
            g_cv.wait(lock, [&] { return g_gen != seen; });
            seen = g_gen;
        }
        _mm_setcsr(g_mxcsr);
        run_units();
        g_done.fetch_add(1, std::memory_order_acq_rel);
    }
}

// --- stats ------------------------------------------------------------------------------------------------------
struct Stats {
    uint64_t ticks = 0, original = 0, cycles = 0, unit_cycles = 0, vehicles = 0, units = 0, largest = 0, group_cycles = 0;
    uint64_t reordered = 0, reorder_ok = 0, reorder_skipped = 0, list_changed = 0;
    uint64_t unions[3] = {}, rope_unresolved = 0;
    uint64_t unprotected[kNumQueues] = {};
    // by distance from your character (local_player.cpp): work and vehicles per ring, in ticks where it was known
    uint64_t ring_cycles[kDistRings] = {}, ring_vehicles[kDistRings] = {}, ring_ticks = 0, ring_unplaced = 0;
    // distant AI logic ([lod] logic)
    uint64_t lod_ticks = 0, ai_vehicles = 0, group_vehicles = 0, unsafe_vehicles = 0, tainted_vehicles = 0, crew_seats = 0, seat_hits = 0, ai_scans = 0, units_decimated = 0, units_control = 0;
    uint64_t units_skipped = 0, vehicles_skipped = 0, captured = 0, replayed = 0, stash_dropped = 0, capture_failed = 0, faults = 0;
    uint64_t lod_cycles = 0;
    uint64_t no_tick = 0, host_blocked = 0, net_reports = 0, net_connected = 0, net_capable = 0;
    uint64_t client_no_set = 0, client_sets = 0, client_set_ids = 0, client_set_lag = 0;
    double speed_dec = 0, speed_ctl = 0;
    uint64_t speed_dec_n = 0, speed_ctl_n = 0;
    Mode mode = kOff;
} g_st;

int nearest_ring(const double p[3], const double (*players)[3], int n)
{
    double best = INFINITY;
    for (int i = 0; i < n; i++) {
        double dx = p[0] - players[i][0], dy = p[1] - players[i][1], dz = p[2] - players[i][2];
        best = std::min(best, dx * dx + dy * dy + dz * dz);
    }
    double d = std::sqrt(best);
    return d < 500.0 ? 0 : d < 1000.0 ? 1 : d < 2000.0 ? 2 : 3;
}

int unit_ring_raw(const Unit& u, const double (*players)[3], int np)
{
    __try {
        for (int v : u.vehicles) {
            uintptr_t veh = g_veh[v].veh;
            uint32_t nb = rd<uint32_t>(veh + 0x10);
            for (uint32_t bi = 0; bi < nb; bi++) {
                uintptr_t so = rd<uintptr_t>(rd<uintptr_t>(q_at(veh, bi)) + 0x718);
                if (!so) continue;
                const double* p = reinterpret_cast<const double*>(so + 0x78);
                if (!std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2])) continue;
                return nearest_ring(p, players, np);
            }
        }
    } __except (EXCEPTION_EXECUTE_HANDLER) {
    }
    return -1;
}

void count_rings()
{
    double players[kMaxPlayers][3];
    int np = players_snapshot(players, kMaxPlayers);
    if (np <= 0) return;
    g_st.ring_ticks++;
    for (const Unit& u : g_units) {
        int k = unit_ring_raw(u, players, np);
        if (k < 0) {
            g_st.ring_unplaced += u.cycles;
            continue;
        }
        g_st.ring_cycles[k] += u.cycles;
        g_st.ring_vehicles[k] += u.vehicles.size();
    }
}
ULONGLONG g_window = 0;
uint64_t g_window_tsc = 0;

const char* mode_name(Mode m) { return m == kSerial ? "serial" : m == kGrouped ? "grouped" : m == kParallel ? "parallel" : "off"; }

void log_lod(uint64_t t, double cyc_ms);
void count_far_work();
void log_far_work(double t, double cyc_ms);

void maybe_log(Mode m)
{
    ULONGLONG now = GetTickCount64();
    if (!g_window) {
        g_window = now;
        g_window_tsc = __rdtsc();
        g_st.mode = m;
        return;
    }
    if (now - g_window < 5000 && m == g_st.mode) return;
    double secs = (now - g_window) / 1000.0;
    double cyc_ms = double(__rdtsc() - g_window_tsc) / (secs * 1000.0);
    uint64_t t = g_st.ticks ? g_st.ticks : 1;
    if ((g_st.ticks || g_st.original) && secs > 0.5) {
        swlog("vehicle logic 5s: mode %s on %d threads, %.0f ticks/s (%.0f/s ran the original loop), loop %.2f ms/tick (work %.2f ms/tick,"
              " grouping %.3f ms), %.0f vehicles in %.0f units (largest %.0f vehicles)",
              mode_name(g_st.mode), g_st.mode == kParallel ? g_threads + 1 : 1, g_st.ticks / secs, g_st.original / secs, g_st.cycles / cyc_ms / t,
              g_st.unit_cycles / cyc_ms / t, g_st.group_cycles / cyc_ms / t, double(g_st.vehicles) / t, double(g_st.units) / t, double(g_st.largest) / t);
        if (g_st.mode >= kGrouped)
            swlog("  groups formed by: %.1f connector links, %.1f dock links, %.1f rope links, %.1f rope hooks with an unknown partner (kept"
                  " together); raycasts %s",
                  double(g_st.unions[0]) / t, double(g_st.unions[1]) / t, double(g_st.unions[2]) / t, double(g_st.rope_unresolved) / t,
                  g_ray_patched ? "in parallel (per-thread ray stack)" : "under the lock");
        if (g_st.mode >= kGrouped) {
            uint64_t calls = 0, cyc = 0;
            for (int i = 0; i < kNumSinks; i++) calls += g_sink_calls[i].load(), cyc += g_sink_cycles[i].load();
            swlog("  shared systems: %.0f locked calls/tick, %.2f ms/tick under the lock, %.2f ms/tick waiting for it; physics events put back in"
                  " order: %.0f/tick (%llu ticks ok, %llu skipped)%s",
                  double(calls) / t, cyc / cyc_ms / t, g_wait_cycles.load() / cyc_ms / t, double(g_st.reordered) / t,
                  static_cast<unsigned long long>(g_st.reorder_ok), static_cast<unsigned long long>(g_st.reorder_skipped),
                  g_overflow.load() ? " (wrapper nesting overflow!)" : "");
            swlog("  world-queue calls: %.1f/tick kept the lock to the end of their tick (%.3f ms/tick held that way), %.0f/tick on a"
                  " vehicle's own queues ran unlocked; %.0f/tick read-only calls shared the lock (%.1f/tick had to switch to exclusive)",
                  double(g_sticky_holds.load()) / t, g_sticky_cycles.load() / cyc_ms / t, double(g_cond_free.load()) / t,
                  double(g_shared_calls.load()) / t, double(g_upgrades.load()) / t);
            if (g_snap_on)
                swlog("  sensor lists: %.0f/tick scans ran unlocked on per-tick copies, %.0f/tick locked (list not copied yet); copies"
                      " checked after the loop: %llu grew (seen next tick), %llu changed otherwise; %llu dead sources skipped",
                      double(g_snap_hits.exchange(0)) / t, double(g_snap_misses.exchange(0)) / t,
                      static_cast<unsigned long long>(g_snap_grown), static_cast<unsigned long long>(g_snap_changed), static_cast<unsigned long long>(g_snap_dead));
            g_snap_grown = g_snap_changed = 0;
            std::vector<int> top;
            for (int i = 0; i < kNumSinks; i++)
                if (g_sink_cycles[i].load()) top.push_back(i);
            size_t nt = std::min<size_t>(8, top.size());
            std::partial_sort(top.begin(), top.begin() + nt, top.end(), [](int a, int b) { return g_sink_cycles[a].load() > g_sink_cycles[b].load(); });
            for (size_t k = 0; k < nt; k++)
                swlog("    %6.3f ms/tick  %7.1f calls/tick  %llx %s", g_sink_cycles[top[k]].load() / cyc_ms / t, double(g_sink_calls[top[k]].load()) / t,
                      static_cast<unsigned long long>(kSinks[top[k]].addr), kSinks[top[k]].name);
            std::vector<int> st;
            for (int i = 0; i < kNumSinks; i++)
                if (g_sticky_by_sink[i].load()) st.push_back(i);
            size_t ns = std::min<size_t>(5, st.size());
            std::partial_sort(st.begin(), st.begin() + ns, st.end(), [](int a, int b) { return g_sticky_by_sink[a].load() > g_sticky_by_sink[b].load(); });
            for (size_t k = 0; k < ns; k++)
                swlog("    held to the end of the tick after: %6.3f ms/tick  %6.1f/tick  %llx %s", g_sticky_by_sink[st[k]].load() / cyc_ms / t,
                      double(g_sticky_n_by_sink[st[k]].load()) / t, static_cast<unsigned long long>(kSinks[st[k]].addr), kSinks[st[k]].name);
            for (int i = 0; i < kNumSinks; i++)
                if (g_upgrades_by_sink[i].load())
                    swlog("    switched a reader to exclusive: %6.1f/tick  %llx %s", double(g_upgrades_by_sink[i].load()) / t,
                          static_cast<unsigned long long>(kSinks[i].addr), kSinks[i].name);
            for (int i = 0; i < kNumSinks; i++) g_sticky_by_sink[i] = 0, g_sticky_n_by_sink[i] = 0, g_upgrades_by_sink[i] = 0;
            if (g_st.list_changed)
                swlog("  note: the object list changed during the loop in %llu ticks", static_cast<unsigned long long>(g_st.list_changed));
            for (int i = 0; i < kNumQueues; i++)
                if (g_st.unprotected[i])
                    swlog("  WARNING: %llu appends to %s happened outside the lock (a function missing from vehicle_sinks.inc?)",
                          static_cast<unsigned long long>(g_st.unprotected[i]), g_queues[i].name);
            for (int i = 0; i < kNumSinks; i++) g_sink_calls[i] = 0, g_sink_cycles[i] = 0;
            g_wait_cycles = 0, g_sticky_holds = 0, g_sticky_cycles = 0, g_cond_free = 0, g_shared_calls = 0, g_upgrades = 0;
            if (g_st.ring_ticks) {
                double rt = double(g_st.ring_ticks);
                std::string line;
                for (int k = 0; k < kDistRings; k++) {
                    char buf[96];
                    snprintf(buf, sizeof buf, "%s%s %.0f vehicles %.2f ms", k ? " | " : "", kDistRingNames[k], g_st.ring_vehicles[k] / rt,
                             g_st.ring_cycles[k] / cyc_ms / rt);
                    line += buf;
                }
                swlog("  work by distance from the nearest player (per tick, %.0f%% of ticks placed): %s; no position %.2f ms", 100.0 * rt / t,
                      line.c_str(), g_st.ring_unplaced / cyc_ms / rt);
            } else {
                swlog("  work by distance: no player position known (not spawned, or the player scan isn't finding them)");
            }
            log_lod(t, cyc_ms);
            log_far_work(double(t), cyc_ms);
        }
    }
    g_st = Stats{};
    g_st.mode = m;
    g_window = now;
    g_window_tsc = __rdtsc();
}

// --- physics event order ---------------------------------------------------------------------------------------
std::vector<std::pair<int, uintptr_t>> g_events;

bool reorder_physics_events()
{
    Queue& q = g_queues[kQPhysics];
    if (!q.header) return false;
    uint32_t now = q_count(q);
    uint64_t sum = 0;
    for (const Posted& p : g_posted) sum += p.n;
    if (now < q.before || now - q.before != sum) return false;
    g_events.clear();
    uint32_t i = q.before;
    for (const Posted& p : g_posted)
        for (uint32_t k = 0; k < p.n; k++, i++) g_events.emplace_back(p.vehicle, rd<uintptr_t>(q_at(q.header, i)));
    std::stable_sort(g_events.begin(), g_events.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
    i = q.before;
    for (const auto& e : g_events) *reinterpret_cast<uintptr_t*>(q_at(q.header, i++)) = e.second;
    g_st.reordered += sum;
    return true;
}

bool reorder_raw()
{
    __try {
        return reorder_physics_events();
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

constexpr uintptr_t kGameNew = 0x140958c28, kRingPush = 0x1408f06c0;
using GameNew_t = void*(__cdecl*)(size_t);
using RingPush_t = void(__fastcall*)(uintptr_t ring, void* value);
struct FarVeh {
    bool is_far = false, crew = false, player = false, stash_ok = false;
    uint64_t ai_tick = 0, seen = 0;
    uint64_t unsafe_until = 0; // posted an event that can't be repeated (or a force on another vehicle): full logic until then
    std::vector<uint8_t> stash;
};
constexpr uint64_t kUnsafeTicks = 600;
// A/B (logic=3): T2 on for 30 s, off for 30 s, all session; running totals per state (0 off, 1 on) to measure the gain
// in one world: tick period (ticks/s), vehicle-logic loop wall time and its thread time.
constexpr uint64_t kAbPeriod = 1800;

// Multiplayer. Every phase, compare window and A/B window runs on the host tick and units are keyed by their smallest
// vehicle id, so the same ship runs on the same ticks on every PC. The host decides which vehicles are eligible and
// publishes that set (network_fixes sends it to every peer that runs this, ~10/s); a client decimates exactly the host's
// set, and nothing without a fresh one. The host decimates only while every connected peer runs it (network_fixes'
// report), or, without a report, while no remote player is in the world.
constexpr uintptr_t kIsHost = 0x140ceb6ff; // byte: 1 host / single player, 0 client
std::atomic<int> g_net_connected{0}, g_net_capable{0};
std::atomic<uint32_t> g_net_report_at{0};
std::mutex g_set_mu;
far_logic::FarSet g_pub, g_recv; // host: published; client: last received
uint32_t g_pub_ms = 0, g_recv_ms = 0;
std::vector<uint32_t> g_unit_ids;
bool g_ids_ok = true, g_ids_logged = false;
std::vector<uint32_t> g_ids_scratch;

bool ids_unique()
{
    g_ids_scratch.clear();
    for (const Veh& v : g_veh) g_ids_scratch.push_back(v.id);
    std::sort(g_ids_scratch.begin(), g_ids_scratch.end());
    bool ok = std::adjacent_find(g_ids_scratch.begin(), g_ids_scratch.end()) == g_ids_scratch.end() &&
              (g_ids_scratch.empty() || g_ids_scratch.front() != 0);
    if (!g_ids_logged && !g_ids_scratch.empty()) {
        g_ids_logged = true;
        swlog("vehicle logic: vehicle ids %s (%zu vehicles, smallest %u, largest %u)", ok ? "unique" : "NOT unique: multiplayer"
              " distant AI logic stays off", g_ids_scratch.size(), g_ids_scratch.front(), g_ids_scratch.back());
    }
    return ok;
}
struct AbState {
    uint64_t ticks = 0, period = 0, loop = 0, work = 0, skipped = 0;
};
AbState g_ab[2];
int g_ab_now = -1, g_ab_prev = -1; // state of this tick / the last counted one (-1: not counted, e.g. no player)
uint64_t g_ab_last = 0, g_last_work = 0, g_ab_skipped = 0;
std::unordered_map<uintptr_t, uint64_t> g_unsafe_hist;
constexpr double kGroupM = 80.0;         // an uncrewed vehicle this close to an AI-crewed one is part of its group
constexpr uint64_t kComparePeriod = 1800; // compare mode: each unit switches between decimated and full every 30 s
// per vehicle index this tick
struct VehNow {
    FarVeh* f;
    bool low, ai, tainted;
    double anchor[3];
    bool anchored;
};
std::vector<VehNow> g_vnow;
std::vector<std::array<double, 3>> g_ai_anchors;
std::unordered_map<uintptr_t, uint64_t> g_ev_hist;
std::unordered_map<uintptr_t, FarVeh> g_farveh;
uint64_t g_lod_tick = 0;
std::vector<const void*> g_crew_seats, g_player_seats;
std::vector<uintptr_t> g_comps, g_scenes;
std::vector<int8_t> g_lod_capture; // per vehicle index this tick: 1 = a decimated unit's vehicle that ran (copy its impulses)

void vehicle_seats(uintptr_t veh, FarVeh& f)
{
    g_comps.clear();
    uint32_t nb = rd<uint32_t>(veh + 0x10);
    for (uint32_t bi = 0; bi < nb; bi++) {
        uintptr_t body = rd<uintptr_t>(q_at(veh, bi));
        uint32_t nc = rd<uint32_t>(body + 0x460);
        for (uint32_t ci = 0; ci < nc; ci++) g_comps.push_back(rd<uintptr_t>(q_at(body + 0x450, ci)));
    }
    int hits = 0;
    bool player = false;
    f.crew = far_logic::ai_crewed(g_comps, g_crew_seats, g_player_seats, &hits, &player);
    f.player = player;
    g_st.ai_scans++;
    g_st.seat_hits += hits;
}

void body_scenes(uintptr_t veh, std::vector<uintptr_t>& out)
{
    out.clear();
    uint32_t nb = rd<uint32_t>(veh + 0x10);
    for (uint32_t bi = 0; bi < nb; bi++)
        if (uintptr_t so = rd<uintptr_t>(rd<uintptr_t>(q_at(veh, bi)) + 0x718)) out.push_back(so);
    std::sort(out.begin(), out.end());
}

// Nearest squared distance of the vehicle's bodies to any player; low = every body at sea level; anchor = first body.
double vehicle_d2(uintptr_t veh, const double (*players)[3], int np, bool& low, double anchor[3], bool& anchored)
{
    double best = INFINITY;
    anchored = false;
    uint32_t nb = rd<uint32_t>(veh + 0x10);
    for (uint32_t bi = 0; bi < nb; bi++) {
        uintptr_t so = rd<uintptr_t>(rd<uintptr_t>(q_at(veh, bi)) + 0x718);
        if (!so) continue;
        const double* p = reinterpret_cast<const double*>(so + 0x78);
        if (!std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2])) {
            low = false;
            continue;
        }
        if (p[1] < -60.0 || p[1] > 40.0) low = false;
        if (!anchored) {
            for (int k = 0; k < 3; k++) anchor[k] = p[k];
            anchored = true;
        }
        for (int i = 0; i < np; i++) {
            double dx = p[0] - players[i][0], dy = p[1] - players[i][1], dz = p[2] - players[i][2];
            best = std::min(best, dx * dx + dy * dy + dz * dz);
        }
    }
    return best;
}

// The eligible-unit rule on the host: AI (crewed, or the rest of a crewed group), every body at sea level, far, not kept at
// full logic. Fills g_unit_ids with the eligible vehicles' ids (at most kSetMaxIds) and sets u.lod = 1 on eligible units.
// Every vehicle, on the host and on clients: seats (every 30 ticks), the far rule, sea level, anchor, and the taint rule:
// a vehicle that ever had a player in a seat, or ever came within kTaintM of a player, is never decimated and never
// held by far physics again this session (by vehicle id: it survives unloading). Players' own vehicles stay out.
constexpr double kTaintM = 300.0;
std::unordered_set<uint32_t> g_tainted;

void lod_scan_vehicles(const double (*players)[3], int np)
{
    if (g_lod_tick % 30 == 1) lod_seats(g_crew_seats, g_player_seats);
    g_st.crew_seats += g_crew_seats.size();
    g_vnow.resize(g_veh.size());
    g_ai_anchors.clear();
    for (size_t v = 0; v < g_veh.size(); v++) {
        uintptr_t veh = g_veh[v].veh;
        FarVeh& f = g_farveh[veh];
        f.seen = g_lod_tick;
        if (!f.ai_tick || g_lod_tick - f.ai_tick >= 30) {
            vehicle_seats(veh, f);
            f.ai_tick = g_lod_tick;
        }
        VehNow& n = g_vnow[v];
        n.f = &f;
        n.low = true;
        double d2 = vehicle_d2(veh, players, np, n.low, n.anchor, n.anchored);
        f.is_far = far_logic::far_rule(f.is_far, d2, g_lod_far2, g_lod_near2);
        if (f.player || !(d2 > kTaintM * kTaintM)) g_tainted.insert(g_veh[v].id);
        n.tainted = g_tainted.count(g_veh[v].id) != 0;
        if (f.crew && n.anchored) g_ai_anchors.push_back({n.anchor[0], n.anchor[1], n.anchor[2]});
    }
}

void lod_host_eligible(bool allowed)
{
    for (VehNow& n : g_vnow) {
        n.ai = !n.tainted && (n.f->crew || (!n.f->player && n.anchored && far_logic::near_any(n.anchor, g_ai_anchors, kGroupM)));
        if (n.tainted) g_st.tainted_vehicles++;
        if (n.f->crew) g_st.ai_vehicles++;
        else if (n.ai) g_st.group_vehicles++;
        if (n.ai && g_lod_tick < n.f->unsafe_until) g_st.unsafe_vehicles++;
    }
    g_unit_ids.clear();
    if (!allowed) return;
    for (Unit& u : g_units) {
        bool eligible = !u.vehicles.empty() && g_unit_ids.size() + u.vehicles.size() <= far_logic::kSetMaxIds;
        for (int v : u.vehicles) {
            const VehNow& n = g_vnow[v];
            if (!n.ai || !n.low || !n.f->is_far || g_lod_tick < n.f->unsafe_until) eligible = false;
        }
        if (!eligible) continue;
        u.lod = 1;
        for (int v : u.vehicles) g_unit_ids.push_back(g_veh[v].id);
    }
    std::sort(g_unit_ids.begin(), g_unit_ids.end());
}

// Client: eligible = every vehicle of the unit is in the host's set (and not kept at full logic here).
void lod_client_eligible(const std::vector<uint32_t>& ids)
{
    for (Unit& u : g_units) {
        bool eligible = !u.vehicles.empty();
        for (int v : u.vehicles) {
            FarVeh& f = g_farveh[g_veh[v].veh];
            f.seen = g_lod_tick;
            if (!std::binary_search(ids.begin(), ids.end(), g_veh[v].id) || g_lod_tick < f.unsafe_until || g_tainted.count(g_veh[v].id))
                eligible = false;
        }
        if (eligible) u.lod = 1;
    }
}

void lod_decide()
{
    for (Unit& u : g_units) u.lod = 0, u.skip = false;
    if (!g_lod_logic) return;
    uint32_t ht = 0;
    if (!host_tick(ht)) {
        g_st.no_tick++;
        return;
    }
    double players[kMaxPlayers][3];
    int remote = 0;
    int np = players_snapshot(players, kMaxPlayers, &remote);
    if (np <= 0) return;
    g_lod_tick++;
    g_st.lod_ticks++;
    if (g_lod_tick % 60 == 1) g_ids_ok = ids_unique();
    bool host = *game_ptr<const volatile uint8_t*>(kIsHost) != 0;
    int mode = g_lod_logic, n = g_lod_n;
    uint32_t now = GetTickCount();
    if (host) {
        uint32_t at = g_net_report_at.load(std::memory_order_relaxed);
        int connected = g_net_connected.load(std::memory_order_relaxed), capable = g_net_capable.load(std::memory_order_relaxed);
        bool fresh = at && now - at < 3000;
        bool allowed = far_logic::host_may_decimate(fresh, connected, capable, remote);
        bool multiplayer = remote > 0 || (fresh && connected > 0);
        if (multiplayer && !g_ids_ok) allowed = false;
        if (!allowed) g_st.host_blocked++;
        if (fresh) g_st.net_connected += connected, g_st.net_capable += capable, g_st.net_reports++;
        lod_scan_vehicles(players, np);
        lod_host_eligible(allowed);
        std::lock_guard<std::mutex> lock(g_set_mu);
        g_pub.tick = ht;
        g_pub.mode = uint8_t(mode);
        g_pub.n = uint8_t(n);
        g_pub.ids = g_unit_ids;
        g_pub_ms = now;
    } else {
        far_logic::FarSet set;
        bool ok;
        {
            std::lock_guard<std::mutex> lock(g_set_mu);
            ok = far_logic::set_fresh(now, g_recv_ms, ht, g_recv.tick, 2000, 600);
            if (ok) set = g_recv;
        }
        if (!ok || !g_ids_ok) {
            g_st.client_no_set++;
            for (auto& [veh, f] : g_farveh) f.stash_ok = false;
            return;
        }
        mode = set.mode;
        n = set.n;
        lod_scan_vehicles(players, np); // the taint rule and far physics need it here too
        g_st.client_sets++;
        g_st.client_set_ids += set.ids.size();
        g_st.client_set_lag += ht > set.tick ? ht - set.tick : set.tick - ht;
        lod_client_eligible(set.ids);
    }
    if (mode == 3) {
        g_ab_now = ((ht / kAbPeriod) & 1) == 0 ? 1 : 0;
        if (!g_ab_now) {
            for (Unit& u : g_units) u.lod = 0;
            for (auto& [veh, f] : g_farveh) f.stash_ok = false; // copies from the last on period are stale by the next one
            return;
        }
    }
    for (Unit& u : g_units) {
        if (u.lod != 1) {
            for (int v : u.vehicles) g_farveh[g_veh[v].veh].stash_ok = false;
            continue;
        }
        uint32_t min_id = UINT32_MAX;
        for (int v : u.vehicles) min_id = std::min(min_id, g_veh[v].id);
        uint64_t key = far_logic::unit_key(min_id);
        if (mode == 2 && far_logic::control_now(ht, key, kComparePeriod)) {
            u.lod = 2;
            g_st.units_control++;
            for (int v : u.vehicles) g_farveh[g_veh[v].veh].stash_ok = false; // no copy from before its control period
            continue;
        }
        g_st.units_decimated++;
        if (far_logic::runs_this_tick(ht, key, n)) continue;
        bool stashed = true;
        for (int v : u.vehicles) stashed = stashed && g_farveh[g_veh[v].veh].stash_ok;
        if (!stashed) continue; // no copy of its forces yet: run it this tick
        u.skip = true;
        g_st.units_skipped++;
        g_st.vehicles_skipped += u.vehicles.size();
        g_ab_skipped += u.vehicles.size();
    }
}

// Far physics candidates (far_physics.cpp): every body (compound scene object) of every vehicle in a decimated unit this
// tick that has no player seat and is untainted, with its vehicle id and whether an NPC crews it. Far physics holds
// crewed ones, and uncrewed ones only as parts joined to a crewed one since loading. Published every tick (empty too).
std::vector<far_logic::Cand> g_far_cands;
// Why vehicles of decimated units don't become candidates (counters only; the 2026-10-28 diagnosis that used them is done).
struct PubStats {
    uint64_t calls = 0, size_mismatch = 0, units = 0, vehicles = 0, no_rec = 0, player = 0, tainted = 0, no_body = 0,
             bodies = 0, faults = 0;
} g_pubst;
void lod_publish_physics_raw()
{
    g_far_cands.clear();
    g_pubst.calls++;
    __try {
        if (g_vnow.size() != g_veh.size()) g_pubst.size_mismatch++;
        else
            for (const Unit& u : g_units) {
                if (u.lod != 1) continue;
                g_pubst.units++;
                for (int v : u.vehicles) {
                    const VehNow& n = g_vnow[v];
                    g_pubst.vehicles++;
                    if (!n.f) { g_pubst.no_rec++; continue; }
                    if (n.f->player) { g_pubst.player++; continue; }
                    if (n.tainted) { g_pubst.tainted++; continue; }
                    uintptr_t veh = g_veh[v].veh;
                    uint32_t nb = rd<uint32_t>(veh + 0x10);
                    for (uint32_t bi = 0; bi < nb; bi++)
                        if (uintptr_t so = rd<uintptr_t>(rd<uintptr_t>(q_at(veh, bi)) + 0x718)) {
                            g_far_cands.push_back(far_logic::Cand{so, g_veh[v].id, n.f->crew});
                            g_pubst.bodies++;
                        } else g_pubst.no_body++;
                }
            }
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        g_far_cands.clear();
        g_pubst.faults++;
    }
}
void lod_publish_physics()
{
    if (!far_physics_on()) return;
    lod_publish_physics_raw();
    far_physics_candidates(g_far_cands);
}

void lod_after(bool reordered)
{
    if (!g_lod_logic || !g_lod_tick) return;
    uint64_t t0 = __rdtsc();
    // capture: decimated units that ran this tick
    g_lod_capture.assign(g_veh.size(), 0);
    for (const Unit& u : g_units) {
        if (u.lod != 1 || u.skip) continue;
        for (int v : u.vehicles) {
            FarVeh& f = g_farveh[g_veh[v].veh];
            f.stash.clear();
            f.stash_ok = reordered;
            if (reordered) g_lod_capture[v] = 1;
            else g_st.capture_failed++;
        }
    }
    if (reordered) {
        // g_events is sorted by vehicle: each vehicle's body scene objects are read once
        int scenes_of = -1;
        for (const auto& [v, ev] : g_events) {
            if (v < 0 || v >= static_cast<int>(g_veh.size()) || !g_lod_capture[v]) continue;
            FarVeh& f = g_farveh[g_veh[v].veh];
            uintptr_t evt = rd<uintptr_t>(ev);
            g_ev_hist[evt]++;
            size_t size = 0;
            far_logic::EventKind kind = far_logic::event_kind(game_ghidra(evt), &size);
            if (kind == far_logic::kHolds) continue;
            bool unsafe = kind == far_logic::kUnsafe;
            uintptr_t why = evt;
            if (!unsafe) {
                if (scenes_of != v) body_scenes(g_veh[v].veh, g_scenes), scenes_of = v;
                uintptr_t target = rd<uintptr_t>(ev + far_logic::kEventTarget);
                if (!std::binary_search(g_scenes.begin(), g_scenes.end(), target)) unsafe = true, why = 0; // a force on another vehicle
            }
            if (unsafe) {
                if (g_lod_tick >= f.unsafe_until) g_unsafe_hist[why]++;
                f.unsafe_until = g_lod_tick + kUnsafeTicks;
                f.stash.clear();
                f.stash_ok = false;
                continue;
            }
            if (!f.stash_ok) continue;
            far_logic::stash_add(f.stash, reinterpret_cast<const void*>(ev), static_cast<uint32_t>(size));
            g_st.captured++;
        }
    }
    // replay: skipped units post their copied forces again
    uintptr_t ring = g_queues[kQPhysics].header;
    auto game_new = game_ptr<GameNew_t>(kGameNew);
    auto push = game_ptr<RingPush_t>(kRingPush);
    for (const Unit& u : g_units) {
        if (!u.skip) continue;
        for (int v : u.vehicles) {
            FarVeh& f = g_farveh[g_veh[v].veh];
            body_scenes(g_veh[v].veh, g_scenes);
            if (!ring || !far_logic::stash_valid(f.stash, g_scenes)) {
                f.stash.clear();
                f.stash_ok = false;
                g_st.stash_dropped++;
                continue;
            }
            far_logic::stash_each(f.stash, [&](const uint8_t* bytes, uint32_t size) {
                void* ev = game_new(size);
                if (!ev) return;
                memcpy(ev, bytes, size);
                push(ring, ev);
                g_st.replayed++;
            });
        }
    }
    for (const Unit& u : g_units) {
        if (!u.lod) continue;
        uintptr_t so = rd<uintptr_t>(rd<uintptr_t>(q_at(g_veh[u.vehicles[0]].veh, 0)) + 0x718);
        if (!so) continue;
        const float* vel = reinterpret_cast<const float*>(so + 0x98);
        double sp = std::sqrt(double(vel[0]) * vel[0] + double(vel[1]) * vel[1] + double(vel[2]) * vel[2]);
        if (!std::isfinite(sp)) continue;
        (u.lod == 1 ? g_st.speed_dec : g_st.speed_ctl) += sp;
        (u.lod == 1 ? g_st.speed_dec_n : g_st.speed_ctl_n)++;
    }
    if (g_lod_tick % 600 == 0)
        for (auto it = g_farveh.begin(); it != g_farveh.end();) it = g_lod_tick - it->second.seen > 600 ? g_farveh.erase(it) : ++it;
    g_st.lod_cycles += __rdtsc() - t0;
}

// Any fault: no unit is skipped this tick and every stash is dropped (all units run fully until they have new copies).
void lod_reset()
{
    for (Unit& u : g_units) u.lod = 0, u.skip = false;
    for (auto& [veh, f] : g_farveh) f.stash_ok = false, f.stash.clear();
    g_st.faults++;
}

void lod_decide_raw()
{
    __try {
        lod_decide();
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        lod_reset();
    }
}

void lod_after_raw(bool reordered)
{
    __try {
        lod_after(reordered);
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        lod_reset();
    }
}

const char* event_rtti_raw(uintptr_t vt)
{
    static uintptr_t image = reinterpret_cast<uintptr_t>(GetModuleHandleW(nullptr));
    __try {
        auto col = reinterpret_cast<const uint32_t*>(rd<uintptr_t>(vt - 8));
        if (!col || col[0] != 1 || reinterpret_cast<uintptr_t>(col) - col[5] != image) return nullptr;
        auto name = reinterpret_cast<const char*>(image + col[3] + 0x10);
        return name[0] == '.' && name[1] == '?' ? name : nullptr;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return nullptr;
    }
}

std::string event_class(uintptr_t vt)
{
    const char* raw = event_rtti_raw(vt);
    if (!raw) {
        char buf[24];
        snprintf(buf, sizeof buf, "%llx", static_cast<unsigned long long>(vt));
        return buf;
    }
    std::string n(raw + 4);
    size_t at = n.find('@');
    return at == std::string::npos ? n : n.substr(0, at);
}

// Far vehicle work probe (2026-10-28, scoping a far-vehicle logic LOD beyond AI crews): every vehicle 0.5 km or more
// from the nearest player, by why it isn't decimated (first rule it fails), with the logic work it cost this tick (its
// unit's time split evenly over the unit's vehicles), and for the ones outside AI groups whether they looked idle:
// first body's speed under 0.2 m/s and no physics events posted this tick. "far vehicle work 5s" lines.
enum FarClass { kFcDecimatedRun, kFcSkipped, kFcControl, kFcTainted, kFcPlayer, kFcNotAi, kFcHigh, kFcNear, kFcUnsafe, kFcOther, kFcN };
const char* const kFcNames[kFcN] = {"decimated (ran)", "decimated (skipped)", "compare control", "tainted (player was near)",
                                    "player seat", "no AI crew / group", "not at sea level", "inside far_m", "kept full (event)", "other"};
struct FarWork {
    uint64_t ticks = 0, n[kFcN] = {}, cyc[kFcN] = {}, idle_n[kFcN] = {}, idle_cyc[kFcN] = {}, near_n = 0, near_cyc = 0;
} g_fw;
std::vector<uint32_t> g_fw_events;

void count_far_work_raw()
{
    if (g_vnow.size() != g_veh.size()) return;
    double players[kMaxPlayers][3];
    int np = players_snapshot(players, kMaxPlayers);
    if (np <= 0) return;
    g_fw.ticks++;
    g_fw_events.assign(g_veh.size(), 0);
    for (const Posted& p : g_posted)
        if (p.vehicle >= 0 && size_t(p.vehicle) < g_fw_events.size()) g_fw_events[p.vehicle] += p.n;
    for (const Unit& u : g_units) {
        if (u.vehicles.empty()) continue;
        uint64_t per = u.cycles / u.vehicles.size();
        for (int v : u.vehicles) {
            const VehNow& n = g_vnow[v];
            bool low = true, anchored = false;
            double anchor[3];
            double d2 = vehicle_d2(g_veh[v].veh, players, np, low, anchor, anchored);
            if (!(d2 >= 500.0 * 500.0)) {
                g_fw.near_n++, g_fw.near_cyc += per;
                continue;
            }
            int c;
            if (u.lod == 1) c = u.skip ? kFcSkipped : kFcDecimatedRun;
            else if (u.lod == 2) c = kFcControl;
            else if (!n.f) c = kFcOther;
            else if (n.f->player) c = kFcPlayer;
            else if (n.tainted) c = kFcTainted;
            else if (!n.ai) c = kFcNotAi;
            else if (!n.low) c = kFcHigh;
            else if (!n.f->is_far) c = kFcNear;
            else if (g_lod_tick < n.f->unsafe_until) c = kFcUnsafe;
            else c = kFcOther;
            g_fw.n[c]++, g_fw.cyc[c] += per;
            bool idle = g_fw_events[v] == 0;
            if (idle) {
                uintptr_t so = rd<uintptr_t>(rd<uintptr_t>(q_at(g_veh[v].veh, 0)) + 0x718);
                if (so) {
                    const float* vel = reinterpret_cast<const float*>(so + 0x98);
                    double sp = std::sqrt(double(vel[0]) * vel[0] + double(vel[1]) * vel[1] + double(vel[2]) * vel[2]);
                    idle = std::isfinite(sp) && sp < 0.2;
                }
            }
            if (idle) g_fw.idle_n[c]++, g_fw.idle_cyc[c] += per;
        }
    }
}
void count_far_work()
{
    __try {
        count_far_work_raw();
    } __except (EXCEPTION_EXECUTE_HANDLER) {
    }
}
void log_far_work(double, double cyc_ms)
{
    if (!g_fw.ticks) return;
    double t = double(g_fw.ticks);
    std::string line;
    double total = 0, idle = 0;
    for (int c = 0; c < kFcN; c++) {
        if (!g_fw.n[c]) continue;
        char buf[160];
        snprintf(buf, sizeof buf, "%s%s %.1f veh %.2f ms (idle %.1f veh %.2f ms)", line.empty() ? "" : "; ", kFcNames[c], g_fw.n[c] / t,
                 g_fw.cyc[c] / cyc_ms / t, g_fw.idle_n[c] / t, g_fw.idle_cyc[c] / cyc_ms / t);
        line += buf;
        total += g_fw.cyc[c] / cyc_ms / t, idle += g_fw.idle_cyc[c] / cyc_ms / t;
    }
    swlog("  far vehicle work 5s (>= 0.5 km, per tick): %.2f ms, of it idle (still, no physics events) %.2f ms; under 0.5 km %.1f "
          "veh %.2f ms; by why not decimated: %s", total, idle, g_fw.near_n / t, g_fw.near_cyc / cyc_ms / t, line.c_str());
    g_fw = FarWork{};
}

void log_lod(uint64_t t, double cyc_ms)
{
    if (!g_lod_logic) return;
    if (g_lod_logic == 3 && (g_ab[0].ticks || g_ab[1].ticks)) {
        auto part = [&](const AbState& a, char* out, size_t n) {
            double secs = a.period / cyc_ms / 1000.0;
            double k = a.ticks ? double(a.ticks) : 1.0;
            snprintf(out, n, "%llu ticks, %.1f ticks/s (%.2f ms per tick), loop %.3f ms, work %.2f ms per tick, %.1f vehicles skipped",
                     static_cast<unsigned long long>(a.ticks), secs > 0 ? a.ticks / secs : 0.0, secs > 0 ? secs * 1000.0 / k : 0.0,
                     a.loop / cyc_ms / k, a.work / cyc_ms / k, a.skipped / k);
        };
        char on[256], off[256];
        part(g_ab[1], on, sizeof on);
        part(g_ab[0], off, sizeof off);
        swlog("  distant AI logic A/B since start (30 s each way): ON %s | OFF %s", on, off);
    }
    if (!g_st.lod_ticks) {
        swlog("  distant AI logic: no player position known (nothing decimated)");
        return;
    }
    double lt = double(g_st.lod_ticks);
    swlog("  distant AI logic%s: %.0f AI-crewed vehicles + %.0f in their groups (crew seats %.0f, found on vehicles %.2f per scan); units per tick: %.1f decimated,"
          " %.1f control, %.1f skipped (%.1f vehicles); impulses %.0f copied, %.0f replayed per tick; %llu stashes dropped, %llu"
          " captures failed, %llu faults; %.3f ms/tick%s",
          g_lod_logic == 2 ? " (compare)" : "", g_st.ai_vehicles / lt, g_st.group_vehicles / lt, g_st.crew_seats / lt,
          g_st.ai_scans ? double(g_st.seat_hits) / g_st.ai_scans : 0.0, g_st.units_decimated / lt, g_st.units_control / lt,
          g_st.units_skipped / lt, g_st.vehicles_skipped / lt, g_st.captured / lt, g_st.replayed / lt,
          static_cast<unsigned long long>(g_st.stash_dropped), static_cast<unsigned long long>(g_st.capture_failed),
          static_cast<unsigned long long>(g_st.faults), g_st.lod_cycles / cyc_ms / double(t),
          g_st.seat_hits == 0 && g_st.ai_scans ? " [no crew seat found on any vehicle: character +0x2100 isn't the seat component?]" : "");
    if (g_lod_logic == 2 && (g_st.speed_dec_n || g_st.speed_ctl_n))
        swlog("  distant AI logic compare (each unit alternates every 30 s): mean speed decimated %.2f m/s (%llu samples), full %.2f m/s"
              " (%llu samples)",
              g_st.speed_dec_n ? g_st.speed_dec / g_st.speed_dec_n : 0.0, static_cast<unsigned long long>(g_st.speed_dec_n),
              g_st.speed_ctl_n ? g_st.speed_ctl / g_st.speed_ctl_n : 0.0, static_cast<unsigned long long>(g_st.speed_ctl_n));
    if (!g_ev_hist.empty()) {
        std::vector<std::pair<uint64_t, uintptr_t>> top;
        uint64_t all = 0;
        for (auto& [vt, n] : g_ev_hist) top.push_back({n, vt}), all += n;
        std::sort(top.begin(), top.end(), [](auto& a, auto& b) { return a.first > b.first; });
        std::string line;
        for (size_t k = 0; k < top.size() && k < 8; k++) {
            char buf[128];
            snprintf(buf, sizeof buf, "%s%s %.1f%%", k ? ", " : "", event_class(top[k].second).c_str(), 100.0 * top[k].first / all);
            line += buf;
        }
        swlog("  distant AI logic events of decimated units on their run ticks (%.0f per tick): %s", all / lt, line.c_str());
        g_ev_hist.clear();
    }
    if (g_st.unsafe_vehicles || !g_unsafe_hist.empty()) {
        std::string line;
        for (auto& [vt, n] : g_unsafe_hist) {
            char buf[128];
            snprintf(buf, sizeof buf, "%s%s %llu", line.empty() ? "" : ", ", vt ? event_class(vt).c_str() : "force on another vehicle",
                     static_cast<unsigned long long>(n));
            line += buf;
        }
        swlog("  distant AI logic kept at full logic: %.1f vehicles per tick (10 s after an event that can't be repeated); new this"
              " window: %s", g_st.unsafe_vehicles / lt, line.empty() ? "none" : line.c_str());
        g_unsafe_hist.clear();
    }
    if (g_st.tainted_vehicles)
        swlog("  distant AI logic: %.1f vehicles per tick never decimated or held (a player sat in them or came within %.0f m this"
              " session; %zu so far)", g_st.tainted_vehicles / lt, kTaintM, g_tainted.size());
    if (g_st.no_tick)
        swlog("  distant AI logic: the host tick isn't known yet in %llu ticks (nothing decimated then)",
              static_cast<unsigned long long>(g_st.no_tick));
    if (*game_ptr<const volatile uint8_t*>(kIsHost)) {
        if (g_st.net_connected || g_st.host_blocked) // nothing to say in single player
            swlog("  distant AI logic multiplayer (host): %.1f peers connected, %.1f run it; blocked in %llu of %llu ticks%s",
                  g_st.net_reports ? double(g_st.net_connected) / g_st.net_reports : 0.0,
                  g_st.net_reports ? double(g_st.net_capable) / g_st.net_reports : 0.0, static_cast<unsigned long long>(g_st.host_blocked),
                  static_cast<unsigned long long>(g_st.lod_ticks),
                  g_st.host_blocked ? " (a peer without it, or remote players and no network_fixes report: nothing decimated)" : "");
    } else {
        swlog("  distant AI logic multiplayer (client): followed the host's set in %llu ticks (%.0f vehicles, %.0f ticks from the"
              " host's), no fresh set in %llu ticks (nothing decimated then)",
              static_cast<unsigned long long>(g_st.client_sets), g_st.client_sets ? double(g_st.client_set_ids) / g_st.client_sets : 0.0,
              g_st.client_sets ? double(g_st.client_set_lag) / g_st.client_sets : 0.0, static_cast<unsigned long long>(g_st.client_no_set));
    }
}

// --- the loop ---------------------------------------------------------------------------------------------------
bool g_frame_bad_logged = false;

bool read_frame(uintptr_t rbp, uintptr_t r13, uintptr_t rdi, uintptr_t r14)
{
    Frame& f = g_f;
    f.list = r13;
    f.p2 = rd<uintptr_t>(rbp + 0x38);
    f.p3 = rd<uintptr_t>(rbp - 0x38);
    f.p4 = rd<uintptr_t>(rbp - 0x10);
    f.p5 = rd<uintptr_t>(rbp - 0x30);
    f.p6 = rd<uintptr_t>(rbp - 0x68);
    f.p7 = rd<uintptr_t>(rbp - 0x28);
    f.p8 = rd<uintptr_t>(rbp - 0x18);
    f.p9 = rd<uintptr_t>(rbp + 0x18);
    f.p11 = rd<uintptr_t>(rbp - 0x20);
    f.p12 = rd<uintptr_t>(rbp - 0x48);
    f.p13 = rd<uintptr_t>(rbp + 0x20);
    f.p14 = rd<uintptr_t>(rbp + 0x30);
    f.p15 = rd<uintptr_t>(rbp + 0x10);
    f.p19 = rd<uintptr_t>(rbp + 0x28);
    const uintptr_t arg12_off = per_build<uintptr_t>(0xb6568, 0xb6560), arg10_off = per_build<uintptr_t>(0x3668, 0x3670);
    bool ok = rdi == f.p12 && r14 == f.p6 && f.p5 == r13 + 0x1b40 && f.p11 == r13 + 0x358 && f.p12 == r13 + arg12_off && f.p14 == r13 + 0x1668;
    if (!ok && !g_frame_bad_logged) {
        g_frame_bad_logged = true;
        swlog("vehicle logic: the loop's frame isn't laid out as expected (rdi %d r14 %d scene %d arg11 %d arg12 %d arg14 %d); running the original loop",
              rdi == f.p12, r14 == f.p6, f.p5 == r13 + 0x1b40, f.p11 == r13 + 0x358, f.p12 == r13 + arg12_off, f.p14 == r13 + 0x1668);
    }
    if (!ok) return false;
    g_world_lo = r13 - per_build<uintptr_t>(kWorldFromList, kWorldFromList + 8);
    g_world_hi = g_world_lo + kWorldSize;
    uintptr_t arg10 = r13 - arg10_off, arg11 = f.p11, arg12 = f.p12, arg14 = f.p14;
    const uintptr_t headers[kNumQueues] = {f.p5 + 0x30,  arg11 + 0x1178, arg11 + 0x11f0, arg14,         arg14 + 0x18, arg14 + 0x30,
                                           arg10 + 0x10, arg12 + 0x20,   arg12 + 0x38,   arg12 + 0x68, arg12 + 0x80, arg12 + 0xc8};
    for (int i = 0; i < kNumQueues; i++) {
        g_queues[i].header = headers[i];
        g_queues[i].tracked = 0;
        g_queues[i].before = rd<uint32_t>(headers[i] + 0x10);
    }
    return true;
}

bool read_frame_raw(uintptr_t rbp, uintptr_t r13, uintptr_t rdi, uintptr_t r14)
{
    __try {
        return read_frame(rbp, r13, rdi, r14);
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

bool collect_raw(uintptr_t list)
{
    __try {
        g_veh.clear();
        uint32_t n = rd<uint32_t>(list + 0x330);
        for (uint32_t i = 0; i < n; i++) {
            uintptr_t entry = rd<uintptr_t>(q_at(list + 0x320, i)), v;
            if (eligible(entry, &v)) g_veh.push_back(Veh{entry, v, 0, rd<uint32_t>(entry + 4)});
        }
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

// zone 0x33 reuses this tick's units: valid when loop 0x32 ran them and the list is unchanged
bool g_units_ready = false;
uint32_t g_units_list_count = 0, g_units_list_head = 0;

extern "C" int __fastcall vl_run(uintptr_t rbp, uintptr_t r13, uintptr_t rdi, uintptr_t r14)
{
    g_units_ready = false;
    hull_rays_join(); // the hull water rays cast on the pool (hull_rays.cpp) are read in this loop's zone 0x33
    if (g_lod_logic == 3) {
        uint64_t now = __rdtsc();
        if (g_ab_prev >= 0 && g_ab_last) g_ab[g_ab_prev].period += now - g_ab_last;
        g_ab_last = now;
        g_ab_prev = -1;
        g_ab_now = -1;
        g_ab_skipped = 0;
        g_last_work = 0;
    }
    Mode mode = g_mode;
    if (mode == kParallel) {
        if (!g_parallel_from) g_parallel_from = GetTickCount64() + ULONGLONG(std::max(0, g_grouped_first_s)) * 1000; // first world tick
        if (GetTickCount64() < g_parallel_from) mode = kGrouped;
    }
    if (mode == kOff || !read_frame_raw(rbp, r13, rdi, r14) || !collect_raw(r13)) {
        g_st.original++;
        maybe_log(mode);
        return 0;
    }
    if (static_cast<int>(g_veh.size()) < g_min_vehicles) {
        if (!g_aero_observer.load(std::memory_order_acquire)) {
            g_st.original++;
            maybe_log(mode);
            return 0;
        }
        // Keep a batch boundary for a single player aircraft, without waking workers.
        mode = kSerial;
    }
    uint64_t t0 = __rdtsc();
    uint32_t list_count = rd<uint32_t>(r13 + 0x330), list_head = rd<uint32_t>(r13 + 0x32c);
    if (mode == kSerial) {
        for (const Veh& v : g_veh) run_vehicle(v.entry, v.veh);
        g_st.vehicles += g_veh.size();
    } else {
        uint64_t tg = __rdtsc();
        if (!build_units_raw()) {
            g_st.original++;
            return 0;
        }
        g_st.group_cycles += __rdtsc() - tg;
        lod_decide_raw();
        lod_publish_physics();
        g_posted.clear();
        snapshot_sensor_lists();
        tl_par = true;
        if (mode == kGrouped) {
            for (int u : g_order) run_unit(g_units[u]);
        } else {
            g_mxcsr = _mm_getcsr();
            g_next.store(0);
            g_done.store(0);
            {
                std::lock_guard<std::mutex> lock(g_mu);
                g_gen++;
            }
            g_cv.notify_all();
            run_units();
            wait_for_workers("vehicle logic", [] { return g_done.load(std::memory_order_acquire) == g_threads; });
        }
        tl_par = false;
        check_sensor_lists();
        g_nsnaps = 0;
        for (int i = 0; i < kNumQueues; i++) {
            Queue& q = g_queues[i];
            uint32_t now = q_count(q);
            if (now > q.before && now - q.before > q.tracked) g_st.unprotected[i] += now - q.before - q.tracked;
        }
        bool reordered = reorder_raw();
        if (reordered) g_st.reorder_ok++;
        else g_st.reorder_skipped++;
        lod_after_raw(reordered);
        uint64_t work = 0;
        size_t largest = 0;
        for (const Unit& u : g_units) work += u.cycles, largest = std::max(largest, u.vehicles.size());
        g_st.unit_cycles += work;
        g_last_work = work;
        count_rings();
        count_far_work();
        g_st.vehicles += g_veh.size();
        g_st.units += g_units.size();
        g_st.largest += largest;
        for (int k = 0; k < 3; k++) g_st.unions[k] += g_link_unions[k];
        g_st.rope_unresolved += g_rope_unresolved;
        g_units_ready = true;
        g_units_list_count = rd<uint32_t>(r13 + 0x330), g_units_list_head = rd<uint32_t>(r13 + 0x32c);
    }
    if (rd<uint32_t>(r13 + 0x330) != list_count || rd<uint32_t>(r13 + 0x32c) != list_head) g_st.list_changed++;
    uint64_t loop = __rdtsc() - t0;
    g_st.cycles += loop;
    g_st.ticks++;
    if (g_lod_logic == 3 && g_ab_now >= 0) {
        AbState& a = g_ab[g_ab_now];
        a.ticks++;
        a.loop += loop;
        a.work += g_last_work;
        a.skipped += g_ab_skipped;
        g_ab_prev = g_ab_now;
    }
    maybe_log(mode);
    return 1;
}

// --- zone 0x33 runner ---------------------------------------------------------------------------------------
Mode g_z33_mode = kOff;
bool g_z33_frame_bad_logged = false;
struct Z33Stats {
    uint64_t ticks = 0, original = 0, cycles = 0, entries = 0, unprotected = 0, reorder_skipped = 0;
};
Z33Stats g_z33;
ULONGLONG g_z33_last_log = 0, g_z33_last_classes = 0;
double g_cyc_per_ms = 0;

bool z33_read_frame(uintptr_t rbp, uintptr_t r13)
{
    Z33Frame& z = g_z;
    z.list = r13;
    z.p2 = rd<U64>(rbp - 0x10), z.p3 = rd<U64>(rbp - 0x30), z.p4 = rd<U64>(rbp), z.p5 = rd<U64>(rbp - 0x68);
    z.p7 = rd<U64>(rbp - 0x28), z.p8 = rd<U64>(rbp + 0x48), z.p9 = rd<U64>(rbp + 0x18), z.p10 = rd<U64>(rbp - 0x20);
    z.p12 = rd<U64>(rbp - 0x48), z.p13 = rd<U64>(rbp - 0x38), z.p19 = rd<U64>(rbp + 0x40), z.p20 = rd<U64>(rbp + 0x10);
    z.q18 = rd<U64>(rbp - 0x18);
    // the same frame slots loop 0x32 read (read_frame): they must not have changed
    const Frame& f = g_f;
    bool ok = z.list == f.list && z.p2 == f.p4 && z.p3 == f.p5 && z.p5 == f.p6 && z.p7 == f.p7 && z.p9 == f.p9 && z.p10 == f.p11 &&
              z.p12 == f.p12 && z.p13 == f.p3 && z.p20 == f.p15 && z.q18 == f.p8;
    if (!ok && !g_z33_frame_bad_logged) {
        g_z33_frame_bad_logged = true;
        swlog("zone 0x33: the frame doesn't match loop 0x32's reads; running the original loop");
    }
    return ok;
}

bool z33_read_frame_raw(uintptr_t rbp, uintptr_t r13)
{
    __try {
        return z33_read_frame(rbp, r13);
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

void z33_merge(Ring32* dst)
{
    for (size_t v = 0; v < g_veh.size(); v++) {
        Ring32& s = g_acc[v];
        for (uint32_t k = 0; k < s.count; k++) {
            const uint8_t* src = s.data + size_t((s.head + k) % s.cap) * 0x20;
            if (dst->count == dst->cap) g_grow32(dst);
            memcpy(dst->data + size_t((dst->head + dst->count) % dst->cap) * 0x20, src, 0x20);
            dst->count++;
        }
        g_z33.entries += s.count;
    }
}

void z33_log()
{
    ULONGLONG now = GetTickCount64();
    if (!g_z33_last_log) g_z33_last_log = now;
    if (now - g_z33_last_log < 5000) return;
    g_z33_last_log = now;
    double t = g_z33.ticks ? double(g_z33.ticks) : 1.0;
    swlog("zone 0x33 5s: %s, %llu ticks taken over, %llu original; loop %.3f ms per tick, %.1f queue entries per tick, %llu unprotected "
          "appends, %llu reorders skipped",
          mode_name(g_z33_mode), static_cast<unsigned long long>(g_z33.ticks), static_cast<unsigned long long>(g_z33.original),
          g_z33.cycles / g_cyc_per_ms / t, g_z33.entries / t, static_cast<unsigned long long>(g_z33.unprotected),
          static_cast<unsigned long long>(g_z33.reorder_skipped));
    g_z33 = Z33Stats{};
    if (now - g_z33_last_classes >= 60000) {
        g_z33_last_classes = now;
        std::string s;
        for (int i = 0; i < kZ33Vts; i++) {
            uintptr_t vt = g_z33_vt[i].load(std::memory_order_relaxed);
            if (!vt) break;
            s += ' ';
            s += event_class(vt);
        }
        swlog("zone 0x33: classes in the vehicles' +0x50 lists:%s", s.empty() ? " none yet" : s.c_str());
    }
}

extern "C" int __fastcall vl_run33(uintptr_t rbp, uintptr_t r13)
{
    if (g_z33_mode == kOff || !g_units_ready || rd<uint32_t>(r13 + 0x330) != g_units_list_count ||
        rd<uint32_t>(r13 + 0x32c) != g_units_list_head || !z33_read_frame_raw(rbp, r13)) {
        g_z33.original++;
        z33_log();
        return 0;
    }
    uint64_t t0 = __rdtsc();
    auto* acc = reinterpret_cast<Ring32*>(rbp - 0x60); // what the original loop's start zeroes
    *reinterpret_cast<uint64_t*>(rbp - 0x60) = 0;
    *reinterpret_cast<uint64_t*>(rbp - 0x58) = 0;
    *reinterpret_cast<uint32_t*>(rbp - 0x50) = 0;
    if (g_acc.size() < g_veh.size()) g_acc.resize(g_veh.size(), Ring32{});
    for (size_t v = 0; v < g_veh.size(); v++) g_acc[v].head = g_acc[v].count = 0;
    for (int i = 0; i < kNumQueues; i++) {
        g_queues[i].tracked = 0;
        g_queues[i].before = q_count(g_queues[i]);
    }
    g_posted.clear();
    snapshot_sensor_lists();
    g_z33_defer_vcalls = g_z33_mode == kParallel;
    g_job = 0x33;
    tl_par = true;
    if (g_z33_mode == kGrouped) {
        for (int u : g_order) run_unit(g_units[u]);
    } else {
        g_mxcsr = _mm_getcsr();
        g_next.store(0);
        g_done.store(0);
        {
            std::lock_guard<std::mutex> lock(g_mu);
            g_gen++;
        }
        g_cv.notify_all();
        run_units();
        wait_for_workers("vehicle zone 0x33", [] { return g_done.load(std::memory_order_acquire) == g_threads; });
    }
    tl_par = false;
    g_job = 0;
    check_sensor_lists();
    g_nsnaps = 0;
    for (int i = 0; i < kNumQueues; i++) {
        Queue& q = g_queues[i];
        uint32_t now = q_count(q);
        if (now > q.before && now - q.before > q.tracked) g_z33.unprotected += now - q.before - q.tracked;
    }
    if (!reorder_raw()) g_z33.reorder_skipped++;
    if (g_z33_defer_vcalls)
        for (const Veh& v : g_veh) z33_vcalls(v.veh);
    z33_merge(acc);
    g_z33.cycles += __rdtsc() - t0;
    g_z33.ticks++;
    z33_log();
    return 1;
}

uint8_t* make_z33_stub(uint8_t* mem, uintptr_t done, uintptr_t body)
{
    uint8_t code[] = {
        0x48, 0x89, 0xE9,                         // mov rcx, rbp
        0x4C, 0x89, 0xEA,
        0x48, 0x83, 0xEC, 0x20,                   // sub rsp, 0x20
        0x48, 0xB8, 0, 0, 0, 0, 0, 0, 0, 0,       // mov rax, vl_run33
        0xFF, 0xD0,                               // call rax
        0x48, 0x83, 0xC4, 0x20,                   // add rsp, 0x20
        0x85, 0xC0,                               // test eax, eax
        0x74, 0x14,                               // jz original
        0x41, 0x8B, 0x85, 0x30, 0x03, 0x00, 0x00,
        0x49, 0xBA, 0, 0, 0, 0, 0, 0, 0, 0,
        0x41, 0xFF, 0xE2,                         // jmp r10
        // original: the 7 bytes the jump replaced, then the loop
        0x45, 0x33, 0xF6,                         // xor r14d, r14d
        0x4C, 0x89, 0x75, 0xA0,                   // mov [rbp-0x60], r14
        0x48, 0xB8, 0, 0, 0, 0, 0, 0, 0, 0,
        0xFF, 0xE0,                               // jmp rax
    };
    uintptr_t run = reinterpret_cast<uintptr_t>(&vl_run33);
    memcpy(&code[12], &run, 8);
    memcpy(&code[39], &done, 8);
    memcpy(&code[59], &body, 8);
    memcpy(mem, code, sizeof code);
    return mem + ((sizeof code + 15) & ~size_t(15));
}

uint8_t* make_loop_stub(uint8_t* mem, uintptr_t done, uintptr_t body)
{
    uint8_t code[] = {
        0x48, 0x89, 0xE9,                         // mov rcx, rbp
        0x4C, 0x89, 0xEA,
        0x49, 0x89, 0xF8,
        0x4D, 0x89, 0xF1,                         // mov r9, r14
        0x48, 0x83, 0xEC, 0x20,                   // sub rsp, 0x20
        0x48, 0xB8, 0, 0, 0, 0, 0, 0, 0, 0,       // mov rax, vl_run
        0xFF, 0xD0,                               // call rax
        0x48, 0x83, 0xC4, 0x20,                   // add rsp, 0x20
        0x85, 0xC0,                               // test eax, eax
        0x74, 0x0C,                               // jz original
        0x48, 0xB8, 0, 0, 0, 0, 0, 0, 0, 0,
        0xFF, 0xE0,                               // jmp rax
        // original: the 5 bytes the jump replaced, then the loop
        0x33, 0xC9,                               // xor ecx, ecx
        0x89, 0x4D, 0x84,                         // mov [rbp-0x7c], ecx
        0x48, 0xB8, 0, 0, 0, 0, 0, 0, 0, 0,
        0xFF, 0xE0,                               // jmp rax
    };
    uintptr_t run = reinterpret_cast<uintptr_t>(&vl_run);
    memcpy(&code[18], &run, 8);
    memcpy(&code[38], &done, 8);
    memcpy(&code[55], &body, 8);
    memcpy(mem, code, sizeof code);
    return mem + ((sizeof code + 15) & ~size_t(15));
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

bool install()
{
    std::wstring ini = settings_ini();
    wchar_t mode[32];
    GetPrivateProfileStringW(L"vehicle_logic", L"mode", L"parallel", mode, 32, ini.c_str());
    g_mode = !_wcsicmp(mode, L"serial") ? kSerial : !_wcsicmp(mode, L"grouped") ? kGrouped : !_wcsicmp(mode, L"parallel") ? kParallel : kOff;
    if (g_mode == kOff) {
        swlog("vehicle logic: off in settings.ini ([vehicle_logic] mode=parallel to run vehicles on several threads)");
        return true;
    }
    g_min_vehicles = static_cast<int>(GetPrivateProfileIntW(L"vehicle_logic", L"min_vehicles", 8, ini.c_str()));
    g_reader_lock = GetPrivateProfileIntW(L"vehicle_logic", L"reader_lock", 0, ini.c_str()) != 0;
    g_snap_on = GetPrivateProfileIntW(L"vehicle_logic", L"snapshot_sensors", 1, ini.c_str()) != 0;
    for (int i = 0; i < kNumSinks; i++)
        if (kSinks[i].addr == kSensorScan) g_sensor_sink = i;
    InitializeCriticalSectionAndSpinCount(&g_world_cs, kLockSpin);
    g_grouped_first_s = static_cast<int>(GetPrivateProfileIntW(L"vehicle_logic", L"grouped_first_s", 0, ini.c_str()));
    g_lod_logic = std::clamp(static_cast<int>(GetPrivateProfileIntW(L"lod", L"logic", 0, ini.c_str())), 0, 3);
    g_lod_n = std::clamp(static_cast<int>(GetPrivateProfileIntW(L"lod", L"logic_n", 3, ini.c_str())), 2, 8);
    double far_m = std::max(100, static_cast<int>(GetPrivateProfileIntW(L"lod", L"far_m", 1200, ini.c_str())));
    double near_m = std::clamp(static_cast<int>(GetPrivateProfileIntW(L"lod", L"near_m", 1000, ini.c_str())), 50, int(far_m));
    g_lod_far2 = far_m * far_m;
    g_lod_near2 = near_m * near_m;
    if (g_lod_logic)
        swlog("vehicle logic: distant AI logic %s, every %d ticks for AI-crewed vehicles at sea level farther than %.0f m from every"
              " player (near again at %.0f m); their impulses are replayed on the other ticks",
              g_lod_logic == 2   ? "in compare mode (each unit alternates between decimated and full every 30 s)"
              : g_lod_logic == 3 ? "in A/B mode (on 30 s, off 30 s, running totals per state)"
                                 : "on",
              g_lod_n, far_m, near_m);
    int threads = static_cast<int>(GetPrivateProfileIntW(L"vehicle_logic", L"threads", 0, ini.c_str()));
    if (threads <= 0) threads = std::min(6, std::max(1, static_cast<int>(std::thread::hardware_concurrency()) - 2));
    g_threads = std::max(0, threads - 1); // the update thread takes part
    if (g_mode == kParallel && g_threads == 0) g_mode = kGrouped;

    const uint8_t* site = reinterpret_cast<const uint8_t*>(live(kLoopPatch));
    if (memcmp(site, kPatchBytes, sizeof kPatchBytes) != 0) {
        swlog("vehicle logic: the loop isn't the expected code at %llx; off", static_cast<unsigned long long>(kLoopPatch));
        return true;
    }
    g_body_update = reinterpret_cast<BodyUpdate_t>(live(kBodyUpdate));
    g_vt_conn = live(kVtConnector), g_vt_dock = live(kVtDoorDock), g_vt_rope = live(kVtRopeHook);
    g_posted.reserve(8192);

    uint8_t* mem = alloc_near(game_base(), 256 * 1024); // ~670 sink stubs of 112 bytes
    if (!mem) {
        swlog("vehicle logic: no memory within 2 GB of the game; off");
        return true;
    }
    uint8_t* p = make_loop_stub(mem, live(kLoopDone), live(kLoopBody));
    uint8_t* loop_stub = mem;

    if (g_mode >= kGrouped) {
        const uint8_t* ray = reinterpret_cast<const uint8_t*>(live(kRayStackLoad));
        if (GetPrivateProfileIntW(L"vehicle_logic", L"parallel_raycasts", 1, ini.c_str()) && memcmp(ray, kRayStackBytes, 7) == 0) {
            uint8_t* stub = p;
            p = make_ray_stub(p);
            int64_t rr = reinterpret_cast<intptr_t>(stub) - static_cast<intptr_t>(live(kRayStackLoad) + 5);
            if (rr >= INT32_MIN && rr <= INT32_MAX) {
                uint8_t call[7] = {0xE8, 0, 0, 0, 0, 0x90, 0x90}; // call stub; nop; nop
                int32_t r32 = static_cast<int32_t>(rr);
                memcpy(call + 1, &r32, 4);
                DWORD old;
                if (VirtualProtect(reinterpret_cast<void*>(live(kRayStackLoad)), 7, PAGE_EXECUTE_READWRITE, &old)) {
                    memcpy(reinterpret_cast<void*>(live(kRayStackLoad)), call, 7); // mod init: before any world exists
                    VirtualProtect(reinterpret_cast<void*>(live(kRayStackLoad)), 7, old, &old);
                    FlushInstructionCache(GetCurrentProcess(), reinterpret_cast<void*>(live(kRayStackLoad)), 7);
                    g_ray_patched = true;
                }
            }
        }
        if (!g_ray_patched)
            swlog("vehicle logic: raycasts stay under the lock (parallel_raycasts=0 or unexpected code at %llx)", static_cast<unsigned long long>(kRayStackLoad));
        g_post_stub = p;
        p = make_post_stub(p);
        int hooked = 0, skipped = 0, gone = 0;
        const bool ported = game_build_ported();
        for (int i = 0; i < kNumSinks; i++) {
            const Sink& s = kSinks[i];
            if (s.new_build && !ported) continue;
            // original-build sinks that a ported build no longer has are replaced in vehicle_sinks_ported.inc
            uintptr_t a = s.new_build ? game_live_native(s.addr) : game_live_if_mapped(s.addr);
            if (!a) {
                gone++;
                continue;
            }
            if (g_ray_patched && std::find(std::begin(kRaySinks), std::end(kRaySinks), s.addr) != std::end(kRaySinks)) continue;
            if (!is_function_start(a)) {
                skipped++;
                swlog("vehicle logic: %s%llx %s isn't a function start; not wrapped", s.new_build ? "new:" : "",
                      static_cast<unsigned long long>(s.addr), s.name);
                continue;
            }
            uint8_t* stub = p;
            p = make_sink_stub(p, i, &g_tramps[i]);
            if (!hook_address(a, stub, &g_tramps[i])) {
                swlog("vehicle logic: wrapping %llx %s FAILED; parallel mode off", static_cast<unsigned long long>(kSinks[i].addr), kSinks[i].name);
                g_mode = kSerial;
                break;
            }
            hooked++;
        }
        FlushInstructionCache(GetCurrentProcess(), mem, p - mem);
        swlog("vehicle logic: %d shared-system functions wrapped, %d skipped, %d original-build ones gone in this build "
              "(vehicle_sinks_ported.inc stands in for them)", hooked, skipped, gone);
    }

    // jmp rel32 at the loop start
    int64_t rel = reinterpret_cast<intptr_t>(loop_stub) - static_cast<intptr_t>(live(kLoopPatch) + 5);
    if (rel < INT32_MIN || rel > INT32_MAX) {
        swlog("vehicle logic: stub out of jump range; off");
        return true;
    }
    uint8_t jmp[5] = {0xE9};
    int32_t r = static_cast<int32_t>(rel);
    memcpy(jmp + 1, &r, 4);
    DWORD old;
    if (!VirtualProtect(reinterpret_cast<void*>(live(kLoopPatch)), 5, PAGE_EXECUTE_READWRITE, &old)) return false;
    memcpy(reinterpret_cast<void*>(live(kLoopPatch)), jmp, 5); // mod init runs before the game loop starts
    VirtualProtect(reinterpret_cast<void*>(live(kLoopPatch)), 5, old, &old);
    FlushInstructionCache(GetCurrentProcess(), reinterpret_cast<void*>(live(kLoopPatch)), 5);
    hull_rays_loop_join_ready(); // vl_run now runs at every vehicle loop's start

    // zone 0x33's loop 1 on the same units ([vehicle_logic] zone33=off | grouped | parallel)
    wchar_t z33[32];
    GetPrivateProfileStringW(L"vehicle_logic", L"zone33", L"off", z33, 32, ini.c_str());
    g_z33_mode = !_wcsicmp(z33, L"grouped") ? kGrouped : !_wcsicmp(z33, L"parallel") ? kParallel : kOff;
    if (g_z33_mode == kParallel && g_mode != kParallel) g_z33_mode = kGrouped;
    if (g_mode < kGrouped) g_z33_mode = kOff;
    if (g_z33_mode != kOff) {
        const uint8_t* z33_site = reinterpret_cast<const uint8_t*>(live(kZ33Patch));
        g_body_pass = reinterpret_cast<BodyPass_t>(live(kBodyPass));
        g_water = reinterpret_cast<Water_t>(live(kWaterSample));
        g_grow32 = reinterpret_cast<Grow_t>(live(kRingGrow32));
        LARGE_INTEGER fq, q0, q1;
        QueryPerformanceFrequency(&fq);
        QueryPerformanceCounter(&q0);
        uint64_t c0 = __rdtsc();
        Sleep(20);
        QueryPerformanceCounter(&q1);
        g_cyc_per_ms = double(__rdtsc() - c0) / (double(q1.QuadPart - q0.QuadPart) * 1000.0 / double(fq.QuadPart));
        uint8_t* z33_stub = p;
        int64_t zr = reinterpret_cast<intptr_t>(z33_stub) - static_cast<intptr_t>(live(kZ33Patch) + 5);
        if (!z33_site || memcmp(z33_site, kZ33Bytes, sizeof kZ33Bytes) != 0 || !g_body_pass || !g_water || !g_grow32 || zr < INT32_MIN ||
            zr > INT32_MAX) {
            swlog("zone 0x33: not the expected code (or out of jump range); its loop stays serial");
            g_z33_mode = kOff;
        } else {
            p = make_z33_stub(p, live(kZ33Done), live(kZ33Body));
            FlushInstructionCache(GetCurrentProcess(), z33_stub, p - z33_stub);
            uint8_t zj[7] = {0xE9, 0, 0, 0, 0, 0x90, 0x90}; // jmp stub; nop; nop
            int32_t z32 = static_cast<int32_t>(zr);
            memcpy(zj + 1, &z32, 4);
            DWORD zold;
            if (VirtualProtect(reinterpret_cast<void*>(live(kZ33Patch)), 7, PAGE_EXECUTE_READWRITE, &zold)) {
                memcpy(reinterpret_cast<void*>(live(kZ33Patch)), zj, 7);
                VirtualProtect(reinterpret_cast<void*>(live(kZ33Patch)), 7, zold, &zold);
                FlushInstructionCache(GetCurrentProcess(), reinterpret_cast<void*>(live(kZ33Patch)), 7);
                swlog("zone 0x33: loop 1 %s on vehicle logic's units (\"zone 0x33 5s\" lines)",
                      g_z33_mode == kParallel ? "parallel (the +0x50 list calls run serially after)" : "grouped (one thread, wrappers on)");
            } else {
                g_z33_mode = kOff;
            }
        }
    }

    if (g_mode == kParallel) {
        int prio = GetThreadPriority(GetCurrentThread());
        for (int i = 0; i < g_threads; i++) {
            HANDLE h = CreateThread(nullptr, 8 << 20, worker_main, nullptr, STACK_SIZE_PARAM_IS_A_RESERVATION, nullptr);
            if (!h) {
                swlog("vehicle logic: worker thread FAILED; grouped mode");
                g_mode = kGrouped;
                break;
            }
            SetThreadPriority(h, prio == THREAD_PRIORITY_ERROR_RETURN ? THREAD_PRIORITY_NORMAL : prio);
            g_workers.push_back(h);
        }
        g_threads = static_cast<int>(g_workers.size());
    }
    g_aero_observer_ready.store(true, std::memory_order_release);
    swlog("vehicle logic: mode %s%s, %d threads, min %d vehicles, %s (\"vehicle logic 5s\" lines)", mode_name(g_mode),
          g_mode == kParallel ? " (grouped for the first grouped_first_s seconds)" : "", g_mode == kParallel ? g_threads + 1 : 1, g_min_vehicles,
          g_reader_lock ? "reader / writer lock (reader_lock=1)" : "one exclusive lock");
    return true;
}

HookRegistrar reg{{"parallel vehicle logic", /*needs_known_build=*/true, &install}};

} // namespace

// Distant AI logic in multiplayer: network_fixes (redundancy.cpp lod_poll, game network thread) calls these.
extern "C" __declspec(dllexport) uint32_t __cdecl swml_lod_net_version() { return g_lod_logic ? 1u : 0u; }

extern "C" __declspec(dllexport) void __cdecl swml_lod_net_peers(int connected, int capable)
{
    g_net_connected.store(connected, std::memory_order_relaxed);
    g_net_capable.store(capable, std::memory_order_relaxed);
    g_net_report_at.store(GetTickCount() | 1u, std::memory_order_relaxed);
}

extern "C" __declspec(dllexport) uint32_t __cdecl swml_lod_net_host_payload(uint8_t* out, uint32_t cap)
{
    if (!g_lod_logic || !out || !*game_ptr<const volatile uint8_t*>(kIsHost)) return 0;
    std::lock_guard<std::mutex> lock(g_set_mu);
    if (!g_pub_ms || GetTickCount() - g_pub_ms > 1000 || !g_pub.mode) return 0;
    return far_logic::set_encode(g_pub, out, cap);
}

extern "C" __declspec(dllexport) void __cdecl swml_lod_net_client_payload(const uint8_t* data, uint32_t size)
{
    if (!g_lod_logic || !data || *game_ptr<const volatile uint8_t*>(kIsHost)) return;
    far_logic::FarSet set;
    if (!far_logic::set_decode(data, size, set)) return;
    std::lock_guard<std::mutex> lock(g_set_mu);
    g_recv = std::move(set);
    g_recv_ms = GetTickCount() | 1u;
}

extern "C" __declspec(dllexport) bool __cdecl swml_register_vehicle_aero_observer(uint32_t version, VehicleAeroObserver observer)
{
    if (version != kVehicleAeroObserverVersion || !g_aero_observer_ready.load(std::memory_order_acquire)) return false;
    if (!observer) { g_aero_observer.store(nullptr, std::memory_order_release); return true; }
    VehicleAeroObserver expected = nullptr;
    return g_aero_observer.compare_exchange_strong(expected, observer, std::memory_order_acq_rel) || expected == observer;
}

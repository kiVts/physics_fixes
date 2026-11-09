
#include "game.h"
#include "hooks.h"
#include "log.h"
#include "sdk.h"
#include "mt_dispatch.h"
#include "world.h"

#include <windows.h>
#include <intrin.h>
#include <algorithm>
#include <vector>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace {

constexpr uintptr_t kNearCallback = 0x1400d84b0;

constexpr size_t kObjOrigin = 0x70, kObjShape = 0x188, kObjFlags = 0x1a0, kObjActivation = 0x1b0,
                 kObjInternalType = 0x1f0, kObjUser = 0x1f8;
constexpr size_t kProxyGroup = 8, kProxyMask = 0xc;
constexpr size_t kCompoundTree = 0x80, kDbvtLeaves = 0x14; // btCompoundShape child btDbvt*, btDbvt::m_leaves
constexpr int kCoGhost = 4, kCfStatic = 1, kShapeCompound = 31;
constexpr size_t kPairAlgorithm = 0x10, kAlgGetManifolds = 3, kManifoldPoints = 0x5e0;
constexpr uintptr_t kAlignedFreeFn = 0x140cbf720; // btAlignedFree function pointer (variable)

struct ManifoldArray {
    uint8_t allocator;
    int size, capacity;
    void** data;
    bool owns;
};
void* g_manifold_buf[512];

// Contact points the pair's manifolds hold after the step, or -1 if unknown.
int count_contacts_raw(const void* pair)
{
    ManifoldArray arr{0, 0, 512, g_manifold_buf, false};
    int total = -1;
    __try {
        auto* alg = *reinterpret_cast<void* const*>(static_cast<const uint8_t*>(pair) + kPairAlgorithm);
        if (!alg) return 0;
        auto fn = reinterpret_cast<void(__fastcall*)(void*, ManifoldArray*)>((*reinterpret_cast<void***>(alg))[kAlgGetManifolds]);
        fn(alg, &arr);
        total = 0;
        for (int i = 0; i < arr.size; i++)
            if (arr.data[i]) total += *reinterpret_cast<const int*>(static_cast<const uint8_t*>(arr.data[i]) + kManifoldPoints);
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        total = -1;
    }
    if (arr.owns && arr.data) { // grew past our buffer: the game allocated it, the game frees it
        auto free_fn = *game_ptr<void(__cdecl**)(void*)>(kAlignedFreeFn);
        if (free_fn) free_fn(arr.data);
    }
    return total;
}

constexpr uintptr_t kClearManifold = 0x1400d8140;
using ClearManifold_t = void(__fastcall*)(void* dispatcher, void* manifold);
bool clear_contacts(const void* pair, void* dispatcher)
{
    void* buf[64];
    ManifoldArray arr{0, 0, 64, buf, false};
    bool any = false;
    __try {
        auto* alg = *reinterpret_cast<void* const*>(static_cast<const uint8_t*>(pair) + kPairAlgorithm);
        if (!alg) return false;
        auto fn = reinterpret_cast<void(__fastcall*)(void*, ManifoldArray*)>((*reinterpret_cast<void***>(alg))[kAlgGetManifolds]);
        fn(alg, &arr);
        for (int i = 0; i < arr.size; i++) {
            if (!arr.data[i] || *reinterpret_cast<const int*>(static_cast<const uint8_t*>(arr.data[i]) + kManifoldPoints) == 0) continue;
            game_ptr<ClearManifold_t>(kClearManifold)(dispatcher, arr.data[i]);
            any = true;
        }
    } __except (EXCEPTION_EXECUTE_HANDLER) {
    }
    if (arr.owns && arr.data) {
        auto free_fn = *game_ptr<void(__cdecl**)(void*)>(kAlignedFreeFn);
        if (free_fn) free_fn(arr.data);
    }
    return any;
}

using Near_t = void(__fastcall*)(void* pair, void* dispatcher, const void* info);
Near_t g_orig = nullptr;

bool g_measure = true;
int g_skip_ghosts = 0;      // 0 off, 1 every ghost (breaks characters), 2 only the big 1.26 m sensor ghosts
bool g_skip_seated = false; // ghosts of seated characters
bool g_puppets = false;     // [puppets] cull: client puppet characters (puppet_cull.cpp)

// Object kinds in the log:
//   D dynamic body, S static, O other
//   J dynamic vehicle body paired with another body of the same vehicle (joined by constraints)
//   B  a character's big r 1.26 m sensor ghost (filter group 0x1220; seated or not isn't known)
//   G / g  a character's small sensor ghosts (group 0x220), walking / seated
//   X  a ghost that belongs to no character (e.g. the 740 group 0x820 ones)
constexpr int kBigGhostGroupBit = 0x1000;

uint8_t kind_of(const uint8_t* obj, const uint8_t* proxy)
{
    int type = *reinterpret_cast<const int*>(obj + kObjInternalType);
    if (type == kCoGhost) {
        // The big sensor is known by its filter group alone (its scene object isn't in the character's
        // ghost slots, so its owner can't be resolved); the small ones by their owner.
        if (*reinterpret_cast<const int*>(proxy + kProxyGroup) & kBigGhostGroupBit) return 'B';
        GhostOwner owner = ghost_owner(obj);
        if (owner == kGhostNotCharacter) return 'X';
        return owner == kGhostCharacterSeated ? 'g' : 'G';
    }
    if (*reinterpret_cast<const int*>(obj + kObjFlags) & kCfStatic) return 'S';
    return type == 2 ? 'D' : 'O'; // 2 = CO_RIGID_BODY
}

uint8_t shape_of(const uint8_t* obj)
{
    auto* shape = *reinterpret_cast<const uint8_t* const*>(obj + kObjShape);
    return shape ? static_cast<uint8_t>(*reinterpret_cast<const int*>(shape + 8)) : 0xff;
}

const char* shape_name(uint8_t t)
{
    switch (t) {
    case 0: return "box";
    case 4: return "hull";
    case 5: return "cloud";
    case 8: return "sphere";
    case 10: return "capsule";
    case 13: return "cylinder";
    case 21: return "trimesh";
    case 28: return "plane";
    case 31: return "compound";
    default: return nullptr;
    }
}

void side_name(uint16_t side, char* s, size_t n)
{
    const char* name = shape_name(uint8_t(side));
    if (name) snprintf(s, n, "%c:%s", char(side >> 8), name);
    else snprintf(s, n, "%c:%u", char(side >> 8), unsigned(side & 0xff));
}

void describe(uint32_t key, char* out, size_t n)
{
    char a[16], b[16];
    side_name(uint16_t(key >> 16), a, sizeof a);
    side_name(uint16_t(key), b, sizeof b);
    snprintf(out, n, "%s-%s", a, b);
}

uintptr_t g_image = 0;

const char* rtti_name_raw(const void* obj)
{
    __try {
        auto vtable = *reinterpret_cast<const uintptr_t* const*>(obj);
        if (!vtable) return nullptr;
        auto col = reinterpret_cast<const uint32_t*>(vtable[-1]); // RTTICompleteObjectLocator
        if (!col || col[0] != 1) return nullptr;                  // x64 signature
        uintptr_t base = reinterpret_cast<uintptr_t>(col) - col[5]; // pSelf RVA -> image base
        if (base != g_image) return nullptr;
        auto name = reinterpret_cast<const char*>(base + col[3] + 0x10); // TypeDescriptor::name
        if (name[0] != '.' || name[1] != '?' || name[2] != 'A') return nullptr;
        return name;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return nullptr;
    }
}

// ".?AVmm_physics_body@@" -> "mm_physics_body"
void class_name(const char* raw, char* out, size_t n)
{
    if (!raw) {
        snprintf(out, n, "-");
        return;
    }
    const char* s = raw + 4;
    size_t len = 0;
    while (s[len] && s[len] != '@' && len + 1 < n) len++;
    memcpy(out, s, len);
    out[len] = 0;
}

// --- per-object stats (one 5 s window) ------------------------------------------------------------
struct Partner {
    uint16_t side; // kind << 8 | shape
    uint64_t cycles;
};

struct Obj {
    const void* ptr; // nullptr = free slot
    uint64_t cycles, pairs, dry_cycles;
    Partner partners[4];
    // copied when first seen in the window (the object may be gone by the time we log)
    uint16_t side;
    int children, group, mask, activation;
    double pos[3];
    const char* obj_class;
    const char* user_class;
};

constexpr int kObjSlots = 8192;
Obj g_objs[kObjSlots];
int g_obj_count = 0;
uint64_t g_obj_overflow = 0;

void snapshot_raw(Obj* o, const uint8_t* obj, const uint8_t* proxy)
{
    __try {
        o->group = *reinterpret_cast<const int*>(proxy + kProxyGroup);
        o->mask = *reinterpret_cast<const int*>(proxy + kProxyMask);
        o->activation = *reinterpret_cast<const int*>(obj + kObjActivation);
        for (int i = 0; i < 3; i++) o->pos[i] = reinterpret_cast<const double*>(obj + kObjOrigin)[i];
        o->children = -1;
        auto* shape = *reinterpret_cast<const uint8_t* const*>(obj + kObjShape);
        if (shape && *reinterpret_cast<const int*>(shape + 8) == kShapeCompound) {
            auto* tree = *reinterpret_cast<const uint8_t* const*>(shape + kCompoundTree);
            if (tree) o->children = *reinterpret_cast<const int*>(tree + kDbvtLeaves);
        }
    } __except (EXCEPTION_EXECUTE_HANDLER) {
    }
}

Obj* obj_slot(const uint8_t* obj, const uint8_t* proxy, uint16_t side)
{
    uintptr_t h = (reinterpret_cast<uintptr_t>(obj) >> 4) * 0x9E3779B97F4A7C15ull;
    for (int i = 0; i < 64; i++) {
        Obj& o = g_objs[(h + i) & (kObjSlots - 1)];
        if (o.ptr == obj) return &o;
        if (!o.ptr) {
            if (g_obj_count >= kObjSlots * 3 / 4) break;
            o = Obj{};
            o.ptr = obj;
            o.side = side;
            snapshot_raw(&o, obj, proxy);
            o.obj_class = rtti_name_raw(obj);
            void* user = *reinterpret_cast<void* const*>(obj + kObjUser);
            o.user_class = user ? rtti_name_raw(user) : nullptr;
            g_obj_count++;
            return &o;
        }
    }
    g_obj_overflow++;
    return nullptr;
}

void obj_add(Obj* o, uint16_t partner, uint64_t cycles, bool dry)
{
    if (!o) return;
    o->cycles += cycles;
    o->pairs++;
    if (dry) o->dry_cycles += cycles;
    Partner* low = &o->partners[0];
    for (auto& p : o->partners) {
        if (p.side == partner && p.cycles) {
            p.cycles += cycles;
            return;
        }
        if (p.cycles < low->cycles) low = &p;
    }
    if (!low->cycles || cycles > low->cycles) *low = Partner{partner, cycles}; // keep the 4 biggest (approx.)
}

// --- per-class stats -----------------------------------------------------------------------------
struct Class {
    uint32_t key; // 0 = free
    uint64_t calls, cycles, dry_cycles;
};
Class g_classes[64]{};
uint64_t g_total_cycles = 0, g_total_dry = 0, g_total_pairs = 0, g_skipped = 0, g_other_thread = 0;
DWORD g_thread = 0;
ULONGLONG g_thread_seen = 0; // last call on g_thread (GetTickCount64)
ULONGLONG g_window_start = 0;
uint64_t g_window_tsc = 0;

void record_class(uint32_t key, uint64_t cycles, bool dry)
{
    for (auto& c : g_classes) {
        if (c.key == key || c.key == 0) {
            c.key = key;
            c.calls++;
            c.cycles += cycles;
            if (dry) c.dry_cycles += cycles;
            return;
        }
    }
}

// --- per object-pair stats -----------------------------------------------------------------------
struct PairStat {
    const void* a; // nullptr = free slot
    const void* b;
    const Obj* oa;
    const Obj* ob;
    uint64_t cycles, dry_cycles, calls, contacts;
};
constexpr int kPairSlots = 16384;
PairStat g_pairs[kPairSlots];
int g_pair_count = 0;

void pair_add(const void* a, const void* b, const Obj* oa, const Obj* ob, uint64_t cycles, int contacts)
{
    if (a > b) {
        std::swap(a, b);
        std::swap(oa, ob);
    }
    uintptr_t h = ((reinterpret_cast<uintptr_t>(a) >> 4) ^ (reinterpret_cast<uintptr_t>(b) * 31)) * 0x9E3779B97F4A7C15ull;
    for (int i = 0; i < 64; i++) {
        PairStat& p = g_pairs[(h >> 20) + i & (kPairSlots - 1)];
        if (p.a == a && p.b == b) {
            p.cycles += cycles;
            p.calls++;
            if (contacts == 0) p.dry_cycles += cycles;
            if (contacts > 0) p.contacts += contacts;
            return;
        }
        if (!p.a) {
            if (g_pair_count >= kPairSlots * 3 / 4) return;
            p = PairStat{a, b, oa, ob, cycles, contacts == 0 ? cycles : 0, 1, uint64_t(std::max(contacts, 0))};
            g_pair_count++;
            return;
        }
    }
}

void obj_label(const Obj* o, char* out, size_t n)
{
    if (!o) {
        snprintf(out, n, "?");
        return;
    }
    char side[16];
    side_name(o->side, side, sizeof side);
    if (o->children >= 0) snprintf(out, n, "%s(%d) at (%.0f, %.0f, %.0f)", side, o->children, o->pos[0], o->pos[1], o->pos[2]);
    else snprintf(out, n, "%s at (%.0f, %.0f, %.0f)", side, o->pos[0], o->pos[1], o->pos[2]);
}

void log_pairs(double cyc_per_ms, double secs)
{
    const PairStat* top[12] = {};
    for (const PairStat& p : g_pairs) {
        if (!p.a) continue;
        for (int i = 0; i < 12; i++) {
            if (!top[i] || p.cycles > top[i]->cycles) {
                for (int j = 11; j > i; j--) top[j] = top[j - 1];
                top[i] = &p;
                break;
            }
        }
    }
    swlog("  %d object pairs this window", g_pair_count);
    for (int i = 0; i < 12 && top[i]; i++) {
        const PairStat& p = *top[i];
        char la[80], lb[80];
        obj_label(p.oa, la, sizeof la);
        obj_label(p.ob, lb, sizeof lb);
        double dist = -1;
        if (p.oa && p.ob) {
            double dx = p.oa->pos[0] - p.ob->pos[0], dy = p.oa->pos[1] - p.ob->pos[1], dz = p.oa->pos[2] - p.ob->pos[2];
            dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        }
        swlog("  pair #%-2d %6.2f ms/s  %4.1f us/call  dry %3.0f%%  %5.1f contacts/call | %s  <->  %s | %.1f m apart", i + 1,
              p.cycles / cyc_per_ms / secs, p.cycles / cyc_per_ms * 1000.0 / p.calls,
              100.0 * p.dry_cycles / std::max<uint64_t>(1, p.cycles), double(p.contacts) / p.calls, la, lb, dist);
    }
}

void log_objects(double cyc_per_ms, double secs)
{
    // Pick the top 12 without sorting the whole table.
    const Obj* top[12] = {};
    for (const Obj& o : g_objs) {
        if (!o.ptr) continue;
        for (int i = 0; i < 12; i++) {
            if (!top[i] || o.cycles > top[i]->cycles) {
                for (int j = 11; j > i; j--) top[j] = top[j - 1];
                top[i] = &o;
                break;
            }
        }
    }
    swlog("  %d objects in pairs this window%s (a pair's time counts for both objects)", g_obj_count,
          g_obj_overflow ? " (table full, some not tracked)" : "");
    for (int i = 0; i < 12 && top[i]; i++) {
        const Obj& o = *top[i];
        char side[16], oc[48], uc[48], partner[16] = "-";
        side_name(o.side, side, sizeof side);
        class_name(o.obj_class, oc, sizeof oc);
        class_name(o.user_class, uc, sizeof uc);
        const Partner* best = &o.partners[0];
        for (auto& p : o.partners) if (p.cycles > best->cycles) best = &p;
        if (best->cycles) side_name(best->side, partner, sizeof partner);
        char children[16] = "";
        if (o.children >= 0) snprintf(children, sizeof children, " %d children", o.children);
        swlog("  obj #%-2d %-11s%s at (%.0f, %.0f, %.0f) %6.2f ms/s %7.0f pairs/s dry %3.0f%% | group 0x%x mask 0x%x act %d | %s / owner %s"
              " | most vs %s (%.0f%%)",
              i + 1, side, children, o.pos[0], o.pos[1], o.pos[2], o.cycles / cyc_per_ms / secs, o.pairs / secs,
              100.0 * o.dry_cycles / std::max<uint64_t>(1, o.cycles), o.group, o.mask, o.activation, oc, uc, partner,
              100.0 * best->cycles / std::max<uint64_t>(1, o.cycles));
    }
}

// Clears the window's stats; g_window_start = 0 makes the next maybe_log() start a new window.
void reset_window()
{
    for (auto& c : g_classes) c = Class{};
    memset(g_objs, 0, sizeof g_objs);
    memset(g_pairs, 0, sizeof g_pairs);
    g_obj_count = 0;
    g_pair_count = 0;
    g_obj_overflow = 0;
    g_total_cycles = g_total_dry = g_total_pairs = g_skipped = g_other_thread = 0;
    g_window_start = 0;
}

void maybe_log()
{
    ULONGLONG now = GetTickCount64();
    uint64_t tsc = __rdtsc();
    if (!g_window_start) {
        g_window_start = now;
        g_window_tsc = tsc;
        return;
    }
    if (now - g_window_start < 5000) return;
    double secs = (now - g_window_start) / 1000.0;
    double cyc_per_ms = double(tsc - g_window_tsc) / (secs * 1000.0);
    std::sort(std::begin(g_classes), std::end(g_classes), [](const Class& x, const Class& y) { return x.cycles > y.cycles; });
    swlog("narrowphase 5s: %.1f ms/s in near callback (dry %.0f%%), %.0f pairs/s, %llu ghost pairs skipped, %llu calls on other threads",
          g_total_cycles / cyc_per_ms / secs, 100.0 * g_total_dry / std::max<uint64_t>(1, g_total_cycles), g_total_pairs / secs,
          static_cast<unsigned long long>(g_skipped), static_cast<unsigned long long>(g_other_thread));
    for (int i = 0; i < 10 && g_classes[i].key; i++) {
        char name[40];
        describe(g_classes[i].key, name, sizeof name);
        swlog("  %-24s %5.1f%%  %7.2f ms/s  %8.0f pairs/s  %.1f us/pair  dry %3.0f%%", name,
              100.0 * g_classes[i].cycles / std::max<uint64_t>(1, g_total_cycles),
              g_classes[i].cycles / cyc_per_ms / secs, g_classes[i].calls / secs,
              g_classes[i].cycles / cyc_per_ms * 1000.0 / g_classes[i].calls,
              100.0 * g_classes[i].dry_cycles / std::max<uint64_t>(1, g_classes[i].cycles));
    }
    log_objects(cyc_per_ms, secs);
    log_pairs(cyc_per_ms, secs);
    reset_window();
    g_window_start = now;
    g_window_tsc = tsc;
}

// Parallel pairs: each thread times its pairs into its own buffer; the physics thread folds them into the tables
// right after the dispatch (mt_set_after_parallel), while every object is still alive.
struct Rec {
    const uint8_t *a, *b, *pa, *pb;
    uint16_t sa, sb;
    uint32_t key;
    uint64_t cycles;
    int contacts;
};
constexpr int kRecBufs = 32;
std::vector<Rec> g_recs[kRecBufs];
std::atomic<int> g_rec_next{0};
std::atomic<uint64_t> g_skipped_par{0};
thread_local int tl_rec = -1;

void account(const Rec& r)
{
    bool dry = r.contacts == 0;
    g_total_cycles += r.cycles;
    g_total_pairs++;
    if (dry) g_total_dry += r.cycles;
    record_class(r.key, r.cycles, dry);
    Obj* oa = obj_slot(r.a, r.pa, r.sa);
    Obj* ob = obj_slot(r.b, r.pb, r.sb);
    obj_add(oa, r.sb, r.cycles, dry);
    obj_add(ob, r.sa, r.cycles, dry);
    pair_add(r.a, r.b, oa, ob, r.cycles, r.contacts);
}

void after_parallel()
{
    if (!g_measure) return;
    g_thread = GetCurrentThreadId();
    g_thread_seen = GetTickCount64();
    for (auto& buf : g_recs) {
        for (const Rec& r : buf) account(r);
        buf.clear();
    }
    g_skipped += g_skipped_par.exchange(0);
    maybe_log();
}

void __fastcall near_detour(void* pair, void* dispatcher, const void* info)
{
    auto** proxies = static_cast<uint8_t**>(pair); // btBroadphasePair: proxy0, proxy1; proxy->m_clientObject first
    const uint8_t* pa = proxies[0];
    const uint8_t* pb = proxies[1];
    const uint8_t* a = *reinterpret_cast<const uint8_t* const*>(pa);
    const uint8_t* b = *reinterpret_cast<const uint8_t* const*>(pb);
    uint8_t ka = kind_of(a, pa), kb = kind_of(b, pb);
    if (ka == 'D' && kb == 'D') {
        int ga = body_group(a);
        if (ga >= 0 && ga == body_group(b)) ka = kb = 'J'; // two bodies of one vehicle
    }

    auto big = [](uint8_t k) { return k == 'B' || k == 'b'; };
    auto any_char_ghost = [](uint8_t k) { return k == 'B' || k == 'b' || k == 'G' || k == 'g'; };
    bool skip = (g_skip_ghosts == 1 && (any_char_ghost(ka) || any_char_ghost(kb) || ka == 'X' || kb == 'X')) ||
                (g_skip_ghosts == 2 && (big(ka) || big(kb))) ||
                (g_skip_seated && (ka == 'g' || kb == 'g' || ka == 'b' || kb == 'b'));
    bool parallel = mt_dispatch_active(); // tables are single-threaded: parallel pairs go through per-thread buffers
    if (!skip && g_puppets && puppet_skip_pair(a, b)) {
        // A client's puppet character (puppet_cull.cpp): drop what the pair touched before, then never run it.
        if (clear_contacts(pair, dispatcher)) puppet_note_cleared();
        return;
    }
    if (!skip && lod_skip_pair(a, b)) {
        // Seated crew far from every player (lod.cpp): the same, until they stand up or a player comes near.
        if (clear_contacts(pair, dispatcher)) lod_note_cleared();
        return;
    }
    if (!skip && far_physics_pair(a, b)) return; // far physics: held bodies stand still on this sub-step (far_physics.cpp)
    if (skip) {
        // No algorithm, no manifold for this pair.
        if (parallel) g_skipped_par.fetch_add(1, std::memory_order_relaxed);
        else {
            g_skipped++;
            if (g_measure && GetCurrentThreadId() == g_thread) maybe_log();
        }
        return;
    }
    if (!g_measure) {
        g_orig(pair, dispatcher, info);
        return;
    }
    if (parallel) {
        if (tl_rec < 0) tl_rec = g_rec_next.fetch_add(1) % kRecBufs; // a buffer per thread (pool threads are fixed)
        uint64_t t0 = __rdtsc();
        g_orig(pair, dispatcher, info);
        uint64_t cycles = __rdtsc() - t0;
        uint16_t sa = uint16_t(ka) << 8 | shape_of(a), sb = uint16_t(kb) << 8 | shape_of(b);
        uint32_t key = sa < sb ? (uint32_t(sa) << 16 | sb) : (uint32_t(sb) << 16 | sa);
        g_recs[tl_rec].push_back(Rec{a, b, pa, pb, sa, sb, key, cycles, count_contacts_raw(pair)});
        return;
    }
    // Measure one thread at a time: the scene that runs physics changes (main menu scene, then the
    // world on another thread), so follow another thread once the measured one has gone quiet for
    // 1 s, or when other threads make far more calls than it (two scenes stepping at once).
    DWORD tid = GetCurrentThreadId();
    ULONGLONG now = GetTickCount64();
    if (tid != g_thread) {
        bool busier = g_other_thread > 4 * g_total_pairs + 20000;
        if (g_thread && now - g_thread_seen < 1000 && !busier) {
            g_other_thread++;
            g_orig(pair, dispatcher, info);
            return;
        }
        if (g_thread) swlog("narrowphase: now measuring thread %lu (was %lu)", tid, g_thread);
        g_thread = tid;
        reset_window();
    }
    g_thread_seen = now;
    uint64_t t0 = __rdtsc();
    g_orig(pair, dispatcher, info);
    uint64_t cycles = __rdtsc() - t0;

    uint16_t sa = uint16_t(ka) << 8 | shape_of(a), sb = uint16_t(kb) << 8 | shape_of(b);
    uint32_t key = sa < sb ? (uint32_t(sa) << 16 | sb) : (uint32_t(sb) << 16 | sa);
    int contacts = count_contacts_raw(pair); // after the timed call: not part of the pair's cost
    bool dry = contacts == 0;
    g_total_cycles += cycles;
    g_total_pairs++;
    if (dry) g_total_dry += cycles;
    record_class(key, cycles, dry);
    Obj* oa = obj_slot(a, pa, sa);
    Obj* ob = obj_slot(b, pb, sb);
    obj_add(oa, sb, cycles, dry);
    obj_add(ob, sa, cycles, dry);
    pair_add(a, b, oa, ob, cycles, contacts);
    maybe_log();
}

bool install()
{
    g_image = reinterpret_cast<uintptr_t>(GetModuleHandleW(nullptr));
    g_measure = GetPrivateProfileIntW(L"narrowphase", L"measure", 0, settings_ini()) != 0;
    g_skip_ghosts = static_cast<int>(GetPrivateProfileIntW(L"narrowphase", L"skip_ghost_pairs", 2, settings_ini()));
    if (g_skip_ghosts < 0 || g_skip_ghosts > 2) g_skip_ghosts = 0;
    g_skip_seated = GetPrivateProfileIntW(L"narrowphase", L"skip_seated_ghosts", 1, settings_ini()) != 0;
    g_puppets = GetPrivateProfileIntW(L"puppets", L"cull", 1, settings_ini()) != 0;
    if (!g_measure && !g_skip_ghosts && !g_skip_seated && !g_puppets) {
        swlog("narrowphase: off in settings.ini");
        return true;
    }
    if (!hook_game(kNearCallback, reinterpret_cast<void*>(&near_detour), reinterpret_cast<void**>(&g_orig))) return false;
    if (g_measure) mt_set_after_parallel(&after_parallel);
    swlog("narrowphase: measure=%d skip_ghost_pairs=%d skip_seated_ghosts=%d puppet_cull=%d", g_measure, g_skip_ghosts, g_skip_seated,
          g_puppets);
    return true;
}

HookRegistrar reg{{"narrowphase", /*needs_known_build=*/true, &install}};

} // namespace

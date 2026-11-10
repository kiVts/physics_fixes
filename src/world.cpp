// Shared physics-world knowledge (see world.h). Everything runs on the physics thread.

#include "world.h"

#include "log.h"
#include "mt_dispatch.h"

#include <windows.h>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace {

constexpr size_t kWorldConstraintCount = 0x1a4, kWorldConstraintData = 0x1b0;
constexpr size_t kConstraintBodyA = 0x28, kConstraintBodyB = 0x30;         // btTypedConstraint m_rbA / m_rbB
constexpr size_t kObjInternalType = 0x1f0, kObjUser = 0x1f8;
constexpr size_t kSceneObjOwner = 0xc0;                                      // scene object -> owner + offset
constexpr size_t kCharOwnerOffset = 0x5b0, kCharGhost0 = 0x490, kCharGhost1 = 0x498, kCharGhost2 = 0x4a0, kCharSeat = 0x2100;
constexpr size_t kGhostSceneBt = 0x148; // mm_physics_ghost_object -> btPairCachingGhostObject (event_create_ghost)

std::unordered_map<const void*, int> g_index;
std::vector<int> g_parent;
bool g_valid = true;
DWORD g_owner_thread = 0; // the physics thread whose world the groups describe
uint64_t g_step_id = 0;
thread_local bool tl_reader = false; // a worker allowed to read the groups during a step
int g_checks = 0, g_bad = 0;

int find(int i)
{
    while (g_parent[i] != i) {
        g_parent[i] = g_parent[g_parent[i]];
        i = g_parent[i];
    }
    return i;
}

int index_of(const void* body)
{
    auto [it, added] = g_index.emplace(body, static_cast<int>(g_parent.size()));
    if (added) g_parent.push_back(it->second);
    return it->second;
}

bool read_constraint(const uint8_t* const* data, int i, const void*& a, const void*& b)
{
    __try {
        const uint8_t* c = data[i];
        if (!c) return false;
        a = *reinterpret_cast<const void* const*>(c + kConstraintBodyA);
        b = *reinterpret_cast<const void* const*>(c + kConstraintBodyB);
        // both must look like rigid bodies (internal type 2)
        return a && b && *reinterpret_cast<const int*>(static_cast<const uint8_t*>(a) + kObjInternalType) == 2 &&
               *reinterpret_cast<const int*>(static_cast<const uint8_t*>(b) + kObjInternalType) == 2;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

bool read_constraints(const uint8_t* world, int& n, const uint8_t* const*& data)
{
    __try {
        n = *reinterpret_cast<const int*>(world + kWorldConstraintCount);
        data = *reinterpret_cast<const uint8_t* const* const*>(world + kWorldConstraintData);
        return n >= 0 && n < 1000000 && (n == 0 || data);
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

GhostOwner ghost_owner_raw(const uint8_t* ghost)
{
    __try {
        const uint8_t* scene = *reinterpret_cast<const uint8_t* const*>(ghost + kObjUser);
        if (!scene) return kGhostNotCharacter;
        const uint8_t* back = *reinterpret_cast<const uint8_t* const*>(scene + kSceneObjOwner);
        if (!back) return kGhostNotCharacter;
        const uint8_t* ch = back - kCharOwnerOffset;
        // the character must list this ghost as one of its own (scene object, or its Bullet ghost)
        bool mine = false;
        for (size_t slot : {kCharGhost0, kCharGhost1, kCharGhost2}) {
            const uint8_t* s = *reinterpret_cast<const uint8_t* const*>(ch + slot);
            if (s == scene || (s && *reinterpret_cast<const uint8_t* const*>(s + kGhostSceneBt) == ghost)) mine = true;
        }
        if (!mine) return kGhostNotCharacter;
        return *reinterpret_cast<const void* const*>(ch + kCharSeat) ? kGhostCharacterSeated : kGhostCharacterWalking;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return kGhostNotCharacter;
    }
}

} // namespace

// Called by census.cpp at the start of every stepSimulation.
void world_step_begin(const void* world_ptr)
{
    g_step_id++;
    g_owner_thread = GetCurrentThreadId();
    if (!g_valid) return;
    g_index.clear();
    g_parent.clear();
    int n = 0;
    const uint8_t* const* data = nullptr;
    if (!read_constraints(static_cast<const uint8_t*>(world_ptr), n, data)) return;
    for (int i = 0; i < n; i++) {
        const void *a, *b;
        g_checks++;
        if (!read_constraint(data, i, a, b)) {
            g_bad++;
            continue;
        }
        int ia = find(index_of(a)), ib = find(index_of(b));
        if (ia != ib) g_parent[ia] = ib;
    }
    for (int i = 0; i < static_cast<int>(g_parent.size()); ++i) find(i); // compress before publishing to workers
    if (g_checks > 2000 && g_bad * 10 > g_checks) {
        g_valid = false;
        swlog("vehicle groups: constraint layout didn't check out (%d of %d unreadable); same-vehicle split off", g_bad, g_checks);
    }
}

int body_group(const void* obj)
{
    // read-only during a step: the physics thread and its parallel narrowphase workers may read it
    if (GetCurrentThreadId() != g_owner_thread && !mt_is_worker() && !tl_reader) return -1;
    auto it = g_index.find(obj);
    if (it == g_index.end()) return -1;
    // Only the build phase may compress paths. Narrowphase/solver workers share
    // this snapshot: even identical stores from find() are a C++ data race.
    int root = it->second;
    while (g_parent[root] != root) root = g_parent[root];
    return root;
}

bool body_groups_valid() { return g_valid; }

void world_mark_reader_thread() { tl_reader = true; }

GhostOwner ghost_owner(const void* ghost) { return ghost_owner_raw(static_cast<const uint8_t*>(ghost)); }

uint64_t world_step_id() { return g_step_id; }
unsigned long world_physics_thread() { return g_owner_thread; }

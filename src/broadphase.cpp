
#include "game.h"
#include "hooks.h"
#include "log.h"
#include "sdk.h"
#include "world.h"

#include <windows.h>
#include <cstdint>

namespace {

constexpr uintptr_t kDbvtSetAabb = 0x1400ccce0;
constexpr size_t kWorldBroadphase = 0x78, kWorldForceUpdateAll = 0x88, kBpDeferredCollide = 0xe5;

int g_deferred = 0, g_active_only = 0;
const void* g_seen_world = nullptr;
bool g_bad_layout = false;

bool is_dbvt(const uint8_t* bp)
{
    __try {
        auto vtable = *reinterpret_cast<const uintptr_t* const*>(bp);
        return vtable && vtable[3] == reinterpret_cast<uintptr_t>(game_ptr(kDbvtSetAabb));
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

bool install()
{
    g_deferred = GetPrivateProfileIntW(L"broadphase", L"deferred_pairs", 0, settings_ini());
    g_active_only = GetPrivateProfileIntW(L"broadphase", L"active_aabbs_only", 0, settings_ini());
    swlog("broadphase: deferred_pairs=%d active_aabbs_only=%d (applied at each physics step)",
          g_deferred, g_active_only);
    return true;
}

HookRegistrar reg{{"broadphase", /*needs_known_build=*/true, &install}};

} // namespace

// Called by census.cpp at the start of every stepSimulation (physics thread, world not being changed).
void broadphase_step_begin(void* world_ptr)
{
    if ((!g_deferred && !g_active_only) || g_bad_layout) return;
    auto world = static_cast<uint8_t*>(world_ptr);
    auto bp = *reinterpret_cast<uint8_t**>(world + kWorldBroadphase);
    if (world_ptr != g_seen_world) {
        g_seen_world = world_ptr;
        if (!bp || !is_dbvt(bp)) {
            g_bad_layout = true;
            swlog("broadphase: world+0x78 is not a btDbvtBroadphase, switches off");
            return;
        }
        swlog("broadphase: new world %p, game had deferred collide %d, force update all AABBs %d", world_ptr,
              bp[kBpDeferredCollide], world[kWorldForceUpdateAll]);
    }
    if (g_deferred) bp[kBpDeferredCollide] = 1;
    if (g_active_only) world[kWorldForceUpdateAll] = 0;
}

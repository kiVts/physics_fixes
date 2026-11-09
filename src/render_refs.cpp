#include "hooks.h"
#include "log.h"

#include <mutex>

namespace {
constexpr uintptr_t kMeshAssign = 0x1408e84b0;
using MeshAssign_t = void(__fastcall*)(void** destination, void* source);
MeshAssign_t g_mesh_assign = nullptr;
std::mutex g_mesh_refs_mutex;

void __fastcall mesh_assign_detour(void** destination, void* source)
{
    std::lock_guard<std::mutex> lock(g_mesh_refs_mutex);
    g_mesh_assign(destination, source);
}
} // namespace

bool rp_install_mesh_refs()
{
    if (!hook_game(kMeshAssign, reinterpret_cast<void*>(&mesh_assign_detour),
                   reinterpret_cast<void**>(&g_mesh_assign))) {
        swlog("render mesh refs: hooking shared mesh assign/release FAILED; parallel prep off");
        return false;
    }
    swlog("render mesh refs: shared mesh assign/release serialized (inventory render refcount race fix)");
    return true;
}

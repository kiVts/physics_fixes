// The multiplayer parts that run inside this mod's physics hooks live in network_fixes (touch_census.cpp there): the
// touch census, the client's push substeps and contact force, and their traces. This calls them at the same points
// it always did; without network_fixes the step and the solver are left as they are.

#include "physics_callbacks.h"
#include "world.h"

#include <windows.h>
#include <atomic>

namespace {

std::atomic<const SwmlPhysicsCallbacks*> g_callbacks{nullptr};
DWORD g_looked = 0;

const SwmlPhysicsCallbacks* callbacks() { return g_callbacks.load(std::memory_order_acquire); }

// network_fixes starts before this mod and may be absent, so the lookup is retried from the physics thread.
void look_up()
{
    DWORD now = GetTickCount();
    if (callbacks() || now - g_looked < 1000) return;
    g_looked = now;
    HMODULE net = GetModuleHandleW(L"network_fixes.dll");
    auto get = net ? reinterpret_cast<SwmlPhysicsCallbacksFn>(GetProcAddress(net, "swml_physics_callbacks")) : nullptr;
    const SwmlPhysicsCallbacks* table = get ? get() : nullptr;
    if (table && table->size >= sizeof(SwmlPhysicsCallbacks)) g_callbacks.store(table, std::memory_order_release);
}

} // namespace

void touch_step_begin(const void* world)
{
    look_up();
    if (auto c = callbacks()) c->step_begin(world);
}

double network_fixed_step(double elapsed, int max_substeps, double native_fixed)
{
    auto c = callbacks();
    return c ? c->fixed_step(elapsed, max_substeps, native_fixed, puppet_local_state() == 2) : native_fixed;
}

void character_launch_begin(const void* world, double elapsed, double fixed)
{
    if (auto c = callbacks()) c->launch_begin(world, elapsed, fixed);
}

void character_launch_end(const void* world)
{
    if (auto c = callbacks()) c->launch_end(world);
}

void network_step_end(const void* world, double fixed, int steps)
{
    if (auto c = callbacks()) c->step_end(world, fixed, steps);
}

void touch_solver_begin(const void* world)
{
    if (auto c = callbacks()) c->solver_begin(world);
}

bool touch_solver_native()
{
    auto c = callbacks();
    return c && c->solver_native();
}

void touch_solver_setup(void* solver, void** bodies, int nb, void** manifolds, int nm)
{
    if (auto c = callbacks()) c->solver_setup(solver, bodies, nb, manifolds, nm);
}

void touch_solver_finish(void* solver)
{
    if (auto c = callbacks()) c->solver_finish(solver);
}

void touch_solver_end()
{
    if (auto c = callbacks()) c->solver_end();
}

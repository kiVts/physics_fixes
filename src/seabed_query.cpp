#include "game.h"
#include "hooks.h"
#include "log.h"
#include "seabed_query_bindings.h"
#include <windows.h>
#include <cstdint>

namespace seabed_query {
using Ptr = uintptr_t;
using Query = float(*)(double, double, Ptr);
using Cleanup = void(*)(Ptr);
using Destroy = void*(*)(Ptr, unsigned);
Query original_query = nullptr;
Cleanup original_cleanup = nullptr;
thread_local unsigned query_depth = 0;

template<class T> T& field(Ptr p, size_t off = 0) {
    return *reinterpret_cast<T*>(p + off);
}

float query_hook(double x, double z, Ptr modifiers) {
    ++query_depth;
    float result;
    __try { result = original_query(x, z, modifiers); }
    __finally { --query_depth; }
    return result;
}

void cleanup_hook(Ptr tile) {
    if (query_depth) {
        const auto data = field<Ptr>(tile);
        const auto count = field<uint32_t>(tile, 8);
        for (uint32_t i = 0; i < count; ++i) {
            auto& object = field<Ptr>(data + size_t(i) * 0x48, 0x40);
            if (object) {
                const auto owned = object;
                object = 0;
                const auto destroy = field<Destroy>(field<Ptr>(owned));
                destroy(owned, 1);
            }
        }
    }
    original_cleanup(tile);
}

bool verify(uint32_t build, Ptr base) {
    bool found = false;
    for (const auto& binding : native::bindings) {
        if (binding.build != build) continue;
        found = true;
        const auto* bytes = reinterpret_cast<const uint8_t*>(base + binding.rva);
        uint64_t hash = 14695981039346656037ull;
        for (uint32_t i = 0; i < binding.size; ++i) { hash ^= bytes[i]; hash *= 1099511628211ull; }
        if (hash != binding.hash) return false;
    }
    return found;
}

bool install_hooks(uint32_t build, Ptr base) {
    Ptr query = 0, cleanup = 0;
    for (const auto& binding : native::bindings) if (binding.build == build) {
        if (binding.kind == native::Kind::query) query = base + binding.rva;
        if (binding.kind == native::Kind::cleanup) cleanup = base + binding.rva;
    }
    // The cleanup hook is inert until a query enters its thread-local scope.
    return query && cleanup &&
        hook_address(cleanup, reinterpret_cast<void*>(&cleanup_hook), reinterpret_cast<void**>(&original_cleanup)) &&
        hook_address(query, reinterpret_cast<void*>(&query_hook), reinterpret_cast<void**>(&original_query));
}

bool install() {
    if (!verify(game_build(), game_base())) {
        swlog("seabed query: unavailable, native ownership bindings differ");
        return false;
    }
    if (!install_hooks(game_build(), game_base())) return false;
    swlog("seabed query: temporary collision meshes released after height sampling");
    return true;
}
HookRegistrar registration{{"seabed query memory", false, &install}};
}

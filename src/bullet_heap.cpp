
#include "game.h"
#include "hooks.h"
#include "log.h"
#include "sdk.h"
#include "thread_arenas.h"

#include <windows.h>
#include <algorithm>
#include <atomic>
#include <cstring>
#include <thread>

namespace {

constexpr uintptr_t kGameAlloc = 0x140190700; // (size, alignment) -> _aligned_malloc(size, 16)
constexpr uintptr_t kGameFree = 0x140190710;  // (p) -> _aligned_free(p)
constexpr int kArenas = 32;

using Alloc_t = void* (*)(size_t, int);
using Free_t = void (*)(void*);

ThreadArenas g_heap;
Alloc_t g_game_alloc;
Free_t g_game_free;
std::atomic<int> g_next_arena{0};
std::atomic<unsigned long long> g_fallbacks{0};
thread_local int t_arena = -1;

void* alloc_fn(size_t n, int alignment)
{
    int a = t_arena;
    if (a < 0) a = t_arena = g_next_arena.fetch_add(1, std::memory_order_relaxed) % kArenas;
    if (void* p = g_heap.alloc(a, n)) return p; // 16-byte aligned, as the game's thunk (which ignores `alignment`)
    g_fallbacks.fetch_add(1, std::memory_order_relaxed);
    return g_game_alloc(n, alignment);
}

void free_fn(void* p)
{
    if (g_heap.owns(p)) return g_heap.free(p);
    g_game_free(p);
}

void log_loop()
{
    unsigned long long last_allocs = 0, last_fallbacks = 0;
    for (;;) {
        Sleep(60000);
        size_t in_use, committed;
        unsigned long long allocs;
        g_heap.stats(in_use, committed, allocs);
        unsigned long long fallbacks = g_fallbacks.load(std::memory_order_relaxed);
        if (allocs == last_allocs && fallbacks == last_fallbacks) continue;
        swlog("bullet heap 60s: %llu allocations from %d thread arenas, %llu fell back to the game's heap; %.1f MB in "
              "use of %.1f MB committed",
              allocs - last_allocs, std::min(g_next_arena.load(), kArenas), fallbacks - last_fallbacks,
              in_use / 1048576.0, committed / 1048576.0);
        last_allocs = allocs;
        last_fallbacks = fallbacks;
    }
}

bool install()
{
    int mode = int(GetPrivateProfileIntW(L"bullet", L"heap", 1, settings_ini()));
    bool wine = GetProcAddress(GetModuleHandleW(L"ntdll.dll"), "wine_get_version") != nullptr;
    if (mode == 0 || (mode == 1 && !wine)) {
        swlog("bullet heap: off (%s)", mode == 0 ? "heap=0" : "not Wine; Windows' heap doesn't serialize the threads");
        return true;
    }
    if (!g_heap.reserve(kArenas)) {
        swlog("bullet heap: couldn't reserve address space for the arenas, off");
        return true;
    }
    // Hooks go live together after every mod installed; a block from an arena can only exist after both are live.
    if (!hook_game(kGameFree, reinterpret_cast<void*>(&free_fn), reinterpret_cast<void**>(&g_game_free)) ||
        !hook_game(kGameAlloc, reinterpret_cast<void*>(&alloc_fn), reinterpret_cast<void**>(&g_game_alloc)))
        return false;
    std::thread(log_loop).detach();
    swlog("bullet heap: on (%s): Bullet allocates from per-thread arenas (%d x %zu MB reserved at %p)",
          wine ? "Wine" : "heap=2", kArenas, g_heap.slice() >> 20, g_heap.base());
    return true;
}

HookRegistrar reg{{"bullet heap", /*needs_known_build=*/true, &install}};

} // namespace

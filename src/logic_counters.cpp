
#include "game.h"
#include "hooks.h"
#include "log.h"
#include "sdk.h"

#include <windows.h>
#include <intrin.h>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <string>

namespace {

constexpr uintptr_t kCountdown = 0x14032abc0;

constexpr int kLen = 69;
const char* kPattern =
    "45 33 c0 44 39 81 ?? ?? ?? ?? 76 38 0f 1f 40 00 8b 81 ?? ?? ?? ?? 33 d2 41 03 c0 f7 b1 ?? ?? ?? ?? "
    "48 8b 81 ?? ?? ?? ?? 48 8b 14 d0 8b 42 ?? 85 c0 74 05 ff c8 89 42 ?? 41 ff c0 44 3b 81 ?? ?? ?? ?? 72 cc c3";

uint32_t g_count = 0, g_first = 0, g_cap = 0, g_array = 0, g_ctr = 0;
using Fn = void(__fastcall*)(uint8_t*);
Fn g_orig = nullptr;
int g_mode = 1;

// stats (update thread mostly; the pass can run on 2 threads, so relaxed atomics)
std::atomic<uint64_t> g_cyc[2], g_calls[2], g_nodes[2], g_slow{0};
std::atomic<ULONGLONG> g_last_log{0};

bool ab_new_now()
{
    if (g_mode != 2) return g_mode == 1;
    return (GetTickCount64() / 30000) % 2 == 0;
}

template <typename T>
T at(uint8_t* c, uint32_t off) { return *reinterpret_cast<T*>(c + off); }

void countdown_fast(uint8_t* c, uint32_t n)
{
    uint32_t cap = at<uint32_t>(c, g_cap), first = at<uint32_t>(c, g_first);
    if (cap == 0 || first >= cap || first + n < first) {  // not exact without the division: the game's code
        g_slow.fetch_add(1, std::memory_order_relaxed);
        g_orig(c);
        return;
    }
    uint8_t** ring = at<uint8_t**>(c, g_array);
    uint32_t idx = first;
    constexpr uint32_t kBatch = 16;
    uint8_t* nodes[kBatch];
    for (uint32_t done = 0; done < n;) {
        uint32_t m = n - done < kBatch ? n - done : kBatch;
        for (uint32_t k = 0; k < m; k++) {
            nodes[k] = ring[idx];
            _mm_prefetch(reinterpret_cast<const char*>(nodes[k] + g_ctr), _MM_HINT_T0);
            if (++idx == cap) idx = 0;
        }
        for (uint32_t k = 0; k < m; k++) {
            int32_t* ctr = reinterpret_cast<int32_t*>(nodes[k] + g_ctr);
            if (*ctr != 0) *ctr = *ctr - 1;
        }
        done += m;
    }
}

void log_maybe()
{
    ULONGLONG now = GetTickCount64(), last = g_last_log.load(std::memory_order_relaxed);
    if (now - last < 5000 || !g_last_log.compare_exchange_strong(last, now)) return;
    if (!last) return;
    uint64_t c[2], k[2], nn[2];
    for (int i = 0; i < 2; i++) c[i] = g_cyc[i].exchange(0), k[i] = g_calls[i].exchange(0), nn[i] = g_nodes[i].exchange(0);
    auto per = [](uint64_t a, uint64_t b) { return b ? double(a) / double(b) : 0.0; };
    swlog("logic counters 5s: new %llu calls %.1f nodes/call %.0f cycles/call | game %llu calls %.1f nodes/call %.0f cycles/call | slow path %llu",
          k[1], per(nn[1], k[1]), per(c[1], k[1]), k[0], per(nn[0], k[0]), per(c[0], k[0]), g_slow.exchange(0));
}

void __fastcall countdown_detour(uint8_t* c)
{
    uint32_t n = at<uint32_t>(c, g_count);
    bool fresh = ab_new_now();
    if (g_mode != 2) {
        if (n) fresh ? countdown_fast(c, n) : g_orig(c);
        return;
    }
    uint64_t t0 = __rdtsc();
    if (n) fresh ? countdown_fast(c, n) : g_orig(c);
    uint64_t t1 = __rdtsc();
    g_cyc[fresh].fetch_add(t1 - t0, std::memory_order_relaxed);
    g_calls[fresh].fetch_add(1, std::memory_order_relaxed);
    g_nodes[fresh].fetch_add(n, std::memory_order_relaxed);
    log_maybe();
}

bool match(const uint8_t* p)
{
    const char* s = kPattern;
    for (int i = 0; i < kLen; i++) {
        while (*s == ' ') s++;
        if (s[0] != '?' && static_cast<uint8_t>(strtoul(std::string(s, 2).c_str(), nullptr, 16)) != p[i]) return false;
        s += 2;
    }
    return true;
}

bool install()
{
    g_mode = static_cast<int>(GetPrivateProfileIntW(L"logic_counters", L"mode", 0, settings_ini()));
    if (g_mode == 0) {
        swlog("logic counters: off in settings.ini");
        return true;
    }
    auto* p = game_ptr<const uint8_t*>(kCountdown);
    if (!p || !match(p)) {
        swlog("logic counters: unexpected code at %llx; off", static_cast<unsigned long long>(kCountdown));
        return false;
    }
    auto d32 = [&](int off) { return *reinterpret_cast<const uint32_t*>(p + off); };
    g_count = d32(6), g_first = d32(18), g_cap = d32(29), g_array = d32(36), g_ctr = p[46];
    if (d32(62) != g_count || p[55] != g_ctr) {
        swlog("logic counters: inconsistent offsets in the game's code; off");
        return false;
    }
    if (!hook_game(kCountdown, reinterpret_cast<void*>(&countdown_detour), reinterpret_cast<void**>(&g_orig))) return false;
    swlog("logic counters: %s (ring +0x%x cap +0x%x first +0x%x count +0x%x, node counter +0x%x)",
          g_mode == 2 ? "A/B, 30 s each way" : "on", g_array, g_cap, g_first, g_count, g_ctr);
    return true;
}

HookRegistrar reg{{"logic counters", /*needs_known_build=*/true, &install}};

} // namespace

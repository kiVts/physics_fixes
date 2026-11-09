
#include "microopt_ab.h"

#include "hooks.h"
#include "log.h"
#include "sdk.h"

#include <windows.h>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <memory>

namespace {

constexpr uintptr_t kPose = 0x140297cf0;
constexpr size_t kSkelBones = 0x08, kSkelBoneCount = 0x10, kSkelAnims = 0x28, kSkelAnimCount = 0x30;
constexpr size_t kBoneStride = 0x188, kBoneRest = 0x10;
constexpr size_t kAnimStride = 0x20, kAnimTracks = 0x10, kAnimTrackCount = 0x18, kTrackStride = 0x28;
constexpr uint32_t kMaxBones = 64;
constexpr size_t kMat = 64;
constexpr int kEntries = 2048; // 8 KB each

using Pose_t = void(__fastcall*)(void* skel, uint32_t a, uint32_t b, float blend, uint16_t fa, uint16_t fb, float* out);
Pose_t g_orig = nullptr;
int g_mode = 2;

struct Key {
    uintptr_t skel, bones, anims;
    uint32_t count, a, b, blend, frames, pad = 0;
    bool operator==(const Key& o) const { return memcmp(this, &o, sizeof *this) == 0; }
};
struct Entry {
    SRWLOCK lock = SRWLOCK_INIT;
    bool valid = false;
    Key key{};
    uint8_t rest[kMaxBones * kMat]; // the rest matrices the pose was made from
    uint8_t out[kMaxBones * kMat];
};
std::unique_ptr<Entry[]> g_cache;

std::atomic<uint64_t> g_calls{0}, g_hits{0}, g_uncacheable{0}, g_checked{0}, g_diff{0};
std::atomic<int> g_diff_logged{0};
std::atomic<uint64_t> g_last_log{0};
struct Counts {
    uint32_t calls = 0, hits = 0, uncacheable = 0, checked = 0, diff = 0;
};
thread_local Counts tl;
thread_local uint32_t tl_hit_seq = 0; // not reset by flush: picks the hits to check

template <typename T> T rd(uintptr_t p, size_t off) { return *reinterpret_cast<const T*>(p + off); }

uint64_t mix(uint64_t h, uint64_t v)
{
    h ^= v + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
    return h;
}

// Fills the key and the rest matrices; false = don't cache this call.
bool describe(uintptr_t skel, uint32_t a, uint32_t b, float blend, uint16_t fa, uint16_t fb, Key& k, uint8_t* rest)
{
    __try {
        k.skel = skel;
        k.bones = rd<uintptr_t>(skel, kSkelBones);
        k.count = rd<uint32_t>(skel, kSkelBoneCount);
        k.anims = rd<uintptr_t>(skel, kSkelAnims);
        k.a = a, k.b = b;
        memcpy(&k.blend, &blend, 4);
        k.frames = uint32_t(fa) | uint32_t(fb) << 16;
        if (k.count == 0 || k.count > kMaxBones || !k.bones) return false;
        for (uint32_t i = 0; i < k.count; i++) memcpy(rest + i * kMat, reinterpret_cast<const void*>(k.bones + i * kBoneStride + kBoneRest), kMat);
        uint32_t n_anims = rd<uint32_t>(skel, kSkelAnimCount);
        if (a < n_anims && b < n_anims) {
            for (uint32_t which : {a, b}) {
                uintptr_t anim = k.anims + which * kAnimStride;
                uintptr_t tracks = rd<uintptr_t>(anim, kAnimTracks);
                uint32_t n = rd<uint32_t>(anim, kAnimTrackCount);
                for (uint32_t t = 0; t < n; t++)
                    if (rd<uint32_t>(tracks, t * kTrackStride) >= k.count) return false;
            }
        }
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

size_t slot(const Key& k)
{
    uint64_t h = mix(mix(mix(mix(k.skel, k.bones), k.anims), uint64_t(k.count) << 32 | k.a), uint64_t(k.b) << 32 | k.blend);
    h = mix(h, k.frames);
    return size_t(h ^ (h >> 29)) & (kEntries - 1);
}

void flush()
{
    g_calls.fetch_add(tl.calls, std::memory_order_relaxed);
    g_hits.fetch_add(tl.hits, std::memory_order_relaxed);
    g_uncacheable.fetch_add(tl.uncacheable, std::memory_order_relaxed);
    g_checked.fetch_add(tl.checked, std::memory_order_relaxed);
    g_diff.fetch_add(tl.diff, std::memory_order_relaxed);
    tl = Counts{};
    uint64_t now = GetTickCount64(), last = g_last_log.load(std::memory_order_relaxed);
    if (!last) g_last_log.compare_exchange_strong(last, now);
    else if (now - last >= 30000 && g_last_log.compare_exchange_strong(last, now)) {
        uint64_t c = g_calls.exchange(0), h = g_hits.exchange(0), u = g_uncacheable.exchange(0), k = g_checked.exchange(0),
                 d = g_diff.exchange(0);
        swlog("pose cache 30s: %llu calls (%.0f/s), %llu hits (%.1f%%), %llu not cacheable, %llu hits checked, %llu different",
              (unsigned long long)c, c / 30.0, (unsigned long long)h, c ? 100.0 * h / c : 0.0, (unsigned long long)u,
              (unsigned long long)k, (unsigned long long)d);
    }
}

void __fastcall detour(void* skel, uint32_t a, uint32_t b, float blend, uint16_t fa, uint16_t fb, float* out)
{
    if (!microopt_on()) return g_orig(skel, a, b, blend, fa, fb, out);
    tl.calls++;
    Key k{};
    thread_local uint8_t rest[kMaxBones * kMat];
    if (!describe(reinterpret_cast<uintptr_t>(skel), a, b, blend, fa, fb, k, rest)) {
        tl.uncacheable++;
        g_orig(skel, a, b, blend, fa, fb, out);
        if (tl.calls >= 1024) flush();
        return;
    }
    size_t bytes = size_t(k.count) * kMat;
    Entry& e = g_cache[slot(k)];
    bool hit = false;
    AcquireSRWLockShared(&e.lock);
    if (e.valid && e.key == k && memcmp(e.rest, rest, bytes) == 0) {
        memcpy(out, e.out, bytes);
        hit = true;
    }
    ReleaseSRWLockShared(&e.lock);
    if (hit) {
        tl.hits++;
        if (g_mode == 1 || (++tl_hit_seq & 1023) == 0) {
            thread_local float mine[kMaxBones * 16];
            memcpy(mine, out, bytes);
            g_orig(skel, a, b, blend, fa, fb, out);
            tl.checked++;
            if (memcmp(mine, out, bytes) != 0) {
                tl.diff++;
                if (g_diff_logged.fetch_add(1) < 5)
                    swlog("pose cache: hit differs from the game's pose (skeleton %p, %u bones, anims %u / %u, frames %u / %u)",
                          skel, k.count, a, b, fa, fb);
            }
        }
    } else {
        g_orig(skel, a, b, blend, fa, fb, out);
        AcquireSRWLockExclusive(&e.lock);
        e.key = k;
        memcpy(e.rest, rest, bytes);
        memcpy(e.out, out, bytes);
        e.valid = true;
        ReleaseSRWLockExclusive(&e.lock);
    }
    if (tl.calls >= 1024) flush();
}

bool install()
{
    g_mode = static_cast<int>(GetPrivateProfileIntW(L"pose_cache", L"mode", 0, settings_ini()));
    if (g_mode <= 0) {
        swlog("pose cache: off in settings.ini");
        return true;
    }
    g_cache.reset(new (std::nothrow) Entry[kEntries]);
    if (!g_cache) return false;
    if (!hook_game(kPose, reinterpret_cast<void*>(&detour), reinterpret_cast<void**>(&g_orig))) return false;
    swlog("pose cache: %s (skeleton poses by their inputs, %d entries, \"pose cache 30s\" lines)",
          g_mode == 1 ? "check (every hit recomputed, the game's pose kept)" : "on", kEntries);
    return true;
}

HookRegistrar reg{{"pose cache", /*needs_known_build=*/true, &install}};

} // namespace

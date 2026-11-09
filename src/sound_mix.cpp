
#include "hooks.h"
#include "log.h"
#include "sdk.h"

#include <windows.h>
#include <intrin.h>
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <vector>

namespace {

constexpr uintptr_t kSoundVolume = 0x140289c60;
constexpr size_t kMixSlots = 0x10, kMixCap = 0x18, kMixFirst = 0x1c, kMixCount = 0x20, kMixListener = 0xa8;
constexpr size_t kSndName = 0x00, kSndNameLen = 0x08, kSndPos = 0x18, kSndRadius = 0x34, kSndGain = 0x40, kSndVolume = 0x54;

using Volume_t = uint64_t(__fastcall*)(uint8_t* mixer, uint8_t* slot);
Volume_t o_volume = nullptr;

bool g_on = true, g_disabled = false;
int g_verify_every = 64;

struct Entry {
    uint8_t* sound;
    const uint8_t* name;
    uint32_t len;
    float loud;
    int index;
};

// Batch state (one thread: the sound update runs on the update thread).
uint8_t* g_mixer = nullptr;
uint8_t* g_slots = nullptr;
uint32_t g_first = 0, g_count = 0, g_next = 0;
enum class State { kIdle, kServe, kSkip } g_state = State::kIdle; // kServe: write ours; kSkip: the game's are written
std::vector<float> g_volume;
std::vector<Entry> g_entries;
uint64_t g_batches = 0, g_sounds = 0, g_singles = 0, g_cycles = 0, g_verified = 0;
ULONGLONG g_window = 0;
uint64_t g_window_tsc = 0;

uint8_t* sound_of(uint8_t* slot)
{
    auto p = *reinterpret_cast<uint8_t**>(slot);
    return p ? *reinterpret_cast<uint8_t**>(p) : nullptr;
}

float loudness(const uint8_t* mixer, const uint8_t* s)
{
    const double* l = reinterpret_cast<const double*>(mixer + kMixListener);
    const double* p = reinterpret_cast<const double*>(s + kSndPos);
    double dx = l[0] - p[0];
    double dy = l[1] - p[1];
    double dz = l[2] - p[2];
    double d = dy * dy;
    d += dx * dx;
    d += dz * dz;
    float r = *reinterpret_cast<const float*>(s + kSndRadius);
    float r2 = r * r;
    float t = r2 - static_cast<float>(d);
    float m = 0.0f > t ? 0.0f : t; // maxss(0, t): t when unordered or equal
    return (m / r2) * *reinterpret_cast<const float*>(s + kSndGain);
}

bool name_less(const Entry& a, const Entry& b)
{
    if (a.len != b.len) return a.len < b.len;
    int c = memcmp(a.name, b.name, a.len);
    return c < 0;
}

void compute_batch(uint8_t* mixer)
{
    g_slots = *reinterpret_cast<uint8_t**>(mixer + kMixSlots);
    uint32_t cap = *reinterpret_cast<uint32_t*>(mixer + kMixCap);
    g_first = *reinterpret_cast<uint32_t*>(mixer + kMixFirst);
    g_count = *reinterpret_cast<uint32_t*>(mixer + kMixCount);
    g_entries.resize(g_count);
    g_volume.assign(g_count, 0.0f);
    for (uint32_t i = 0; i < g_count; i++) {
        uint8_t* s = sound_of(g_slots + size_t((g_first + i) % cap) * 0x10);
        Entry& e = g_entries[i];
        e.sound = s;
        e.name = *reinterpret_cast<const uint8_t* const*>(s + kSndName);
        e.len = *reinterpret_cast<const uint32_t*>(s + kSndNameLen);
        e.loud = loudness(mixer, s);
        e.index = static_cast<int>(i);
    }
    // by name, then loudest first (NaN last: never louder than anything, and nothing is louder than it)
    std::sort(g_entries.begin(), g_entries.end(), [](const Entry& a, const Entry& b) {
        if (name_less(a, b)) return true;
        if (name_less(b, a)) return false;
        bool an = a.loud != a.loud, bn = b.loud != b.loud;
        if (an != bn) return bn;
        return a.loud > b.loud;
    });
    size_t g = 0;
    while (g < g_entries.size()) {
        size_t end = g + 1;
        while (end < g_entries.size() && !name_less(g_entries[g], g_entries[end])) end++;
        size_t first_equal = g; // first entry of the current run of equal loudness
        for (size_t k = g; k < end; k++) {
            const Entry& e = g_entries[k];
            if (k == g || g_entries[k - 1].loud != e.loud) first_equal = k; // NaN never equals: own run
            int louder = e.loud != e.loud ? 0 : static_cast<int>(first_equal - g);
            float factor = 1.0f;
            for (int c = 0; c < louder && c < 200; c++) factor *= 0.5f;
            g_volume[e.index] = factor * e.loud;
        }
        g = end;
    }
}

// Runs the game's function on every sound of the batch and compares with ours. Leaves the game's values.
bool verify_batch(uint8_t* mixer)
{
    uint32_t cap = *reinterpret_cast<uint32_t*>(mixer + kMixCap);
    for (uint32_t i = 0; i < g_count; i++) {
        uint8_t* slot = g_slots + size_t((g_first + i) % cap) * 0x10;
        o_volume(mixer, slot);
        float game = *reinterpret_cast<float*>(sound_of(slot) + kSndVolume);
        if (memcmp(&game, &g_volume[i], 4) != 0) {
            swlog("sound mix: volume of sound %u of %u differs (game %.9g, ours %.9g); fast mix off", i, g_count, game, g_volume[i]);
            return false;
        }
    }
    return true;
}

void maybe_log()
{
    ULONGLONG now = GetTickCount64();
    if (!g_window) {
        g_window = now;
        g_window_tsc = __rdtsc();
        return;
    }
    if (now - g_window < 5000) return;
    double secs = (now - g_window) / 1000.0;
    double cyc_per_ms = double(__rdtsc() - g_window_tsc) / (secs * 1000.0);
    swlog("sound mix 5s: %.1f batches/s, %.0f sounds per batch, %.3f ms per batch (fast), %.0f single calls/s (original), %llu verified",
          g_batches / secs, g_batches ? double(g_sounds) / g_batches : 0.0, g_batches ? g_cycles / cyc_per_ms / g_batches : 0.0,
          g_singles / secs, static_cast<unsigned long long>(g_verified));
    g_batches = g_sounds = g_singles = g_cycles = 0;
    g_window = now;
    g_window_tsc = __rdtsc();
}

uint64_t __fastcall volume_detour(uint8_t* mixer, uint8_t* slot)
{
    if (g_disabled) return o_volume(mixer, slot);
    uint8_t* slots = *reinterpret_cast<uint8_t**>(mixer + kMixSlots);
    uint32_t cap = *reinterpret_cast<uint32_t*>(mixer + kMixCap);
    uint32_t first = *reinterpret_cast<uint32_t*>(mixer + kMixFirst);
    uint32_t count = *reinterpret_cast<uint32_t*>(mixer + kMixCount);
    if (!slots || !cap || slot < slots || (slot - slots) % 0x10) {
        g_singles++;
        return o_volume(mixer, slot);
    }
    uint32_t phys = static_cast<uint32_t>((slot - slots) / 0x10);
    uint32_t index = (phys + cap - first % cap) % cap;
    if (index == 0 && count > 0) {
        uint64_t t0 = __rdtsc();
        g_mixer = mixer;
        compute_batch(mixer);
        g_batches++;
        g_sounds += count;
        g_next = 1;
        g_state = State::kServe;
        if (g_verify_every > 0 && (g_batches == 1 || g_batches % g_verify_every == 0)) {
            g_verified++;
            if (!verify_batch(mixer)) g_disabled = true;
            g_state = State::kSkip; // the game's function has written every sound of the batch
        } else {
            *reinterpret_cast<float*>(sound_of(slot) + kSndVolume) = g_volume[0];
        }
        if (g_next == count || g_disabled) g_state = State::kIdle;
        g_cycles += __rdtsc() - t0;
        maybe_log();
        return 0;
    }
    if (g_state != State::kIdle && mixer == g_mixer && index == g_next && slots == g_slots && first == g_first && count == g_count) {
        if (g_state == State::kServe) *reinterpret_cast<float*>(sound_of(slot) + kSndVolume) = g_volume[index];
        if (++g_next == g_count) g_state = State::kIdle;
        return 0;
    }
    g_state = State::kIdle;
    g_singles++;
    return o_volume(mixer, slot);
}

bool install()
{
    g_on = GetPrivateProfileIntW(L"sound", L"fast_mix", 1, settings_ini()) != 0;
    g_verify_every = static_cast<int>(GetPrivateProfileIntW(L"sound", L"verify_every", 64, settings_ini()));
    if (!g_on) {
        swlog("sound mix: off in settings.ini");
        return true;
    }
    if (!hook_game(kSoundVolume, reinterpret_cast<void*>(&volume_detour), reinterpret_cast<void**>(&o_volume))) return false;
    swlog("sound mix: one-pass volume ducking (verify every %d batches; \"sound mix 5s\" lines)", g_verify_every);
    return true;
}

HookRegistrar reg{{"sound_mix", /*needs_known_build=*/true, &install}};

} // namespace

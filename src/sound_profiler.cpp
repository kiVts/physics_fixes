#include "game.h"
#include "hooks.h"
#include "log.h"
#include "sound_profiler_bindings.h"
#include <windows.h>
#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>

namespace sound_profiler {
using Ptr = uintptr_t;
using Assign = void(*)(Ptr*, Ptr);
using Update = void(*)(Ptr, Ptr, Ptr, Ptr);
Assign original_assign = nullptr;
Update original_update = nullptr;
std::recursive_mutex references;
std::atomic<unsigned> rejected{0};
constexpr size_t channels_offset = 0xe0;
constexpr size_t mixer_channels_offset = 0x28;
constexpr size_t channel_count = 16;
constexpr uint32_t max_name = 4096;

template<class T> T& field(Ptr p, size_t offset = 0) {
    return *reinterpret_cast<T*>(p + offset);
}

void assign_hook(Ptr* destination, Ptr source) {
    std::lock_guard<std::recursive_mutex> lock(references);
    original_assign(destination, source);
}

bool valid_name(Ptr reference) {
    __try {
        const Ptr sound = field<Ptr>(reference);
        if (!sound) return false;
        const auto size = field<uint32_t>(sound, 8), capacity = field<uint32_t>(sound, 12);
        const Ptr data = field<Ptr>(sound);
        return size <= max_name && capacity > size && data && field<char>(data, size) == '\0';
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

struct ChannelSnapshot {
    std::array<uint8_t, mixer_channels_offset> prefix{};
    std::array<Ptr, channel_count> pinned{};
};
static_assert(offsetof(ChannelSnapshot, pinned) == mixer_channels_offset);
struct Frame {
    Ptr ui;
    Ptr source;
    ChannelSnapshot snapshot;
};

// Retain the sounds before releasing the channel lock. Native UI work then
// reads its own references, even if simulation replaces every live channel.
void pin(Frame& frame) {
    std::lock_guard<std::recursive_mutex> lock(references);
    if (!frame.source) return;
    auto* source = reinterpret_cast<const Ptr*>(frame.source + mixer_channels_offset);
    for (size_t i = 0; i < frame.snapshot.pinned.size(); ++i) {
        original_assign(&frame.snapshot.pinned[i], source[i]);
        if (frame.snapshot.pinned[i] && !valid_name(frame.snapshot.pinned[i])) {
            original_assign(&frame.snapshot.pinned[i], 0);
            if (rejected.fetch_add(1, std::memory_order_relaxed) < 4)
                swlog("sound profiler: invalid channel name omitted");
        }
    }
}

void release(Frame& frame) {
    field<Ptr>(frame.ui, channels_offset) = frame.source;
    std::lock_guard<std::recursive_mutex> lock(references);
    for (auto& reference : frame.snapshot.pinned) original_assign(&reference, 0);
}

void update_hook(Ptr ui, Ptr a, Ptr b, Ptr c) {
    Frame frame{ui, field<Ptr>(ui, channels_offset), {}};
    __try {
        pin(frame);
        field<Ptr>(ui, channels_offset) = reinterpret_cast<Ptr>(&frame.snapshot);
        original_update(ui, a, b, c);
    } __finally {
        release(frame);
    }
}

bool verify(uint32_t build, Ptr base) {
    bool found = false;
    for (auto& entry : native::bindings) {
        if (entry.build != build) continue;
        found = true;
        auto* bytes = reinterpret_cast<const uint8_t*>(base + entry.rva);
        uint64_t hash = 14695981039346656037ull;
        for (uint32_t i = 0; i < entry.size; ++i) { hash ^= bytes[i]; hash *= 1099511628211ull; }
        if (hash != entry.hash) return false;
    }
    return found;
}

bool install() {
    const auto build = game_build();
    const auto base = game_base();
    if (!verify(build, base)) {
        swlog("sound profiler: unavailable, native sound lifetime bindings differ");
        return false;
    }
    Ptr assign = 0, update = 0;
    for (auto& entry : native::bindings) if (entry.build == build) {
        if (entry.kind == native::Kind::assign) assign = base + entry.rva;
        if (entry.kind == native::Kind::update) update = base + entry.rva;
    }
    if (!assign || !update ||
        !hook_address(assign, reinterpret_cast<void*>(&assign_hook), reinterpret_cast<void**>(&original_assign)) ||
        !hook_address(update, reinterpret_cast<void*>(&update_hook), reinterpret_cast<void**>(&original_update))) return false;
    swlog("sound profiler: channel references pinned while drawing names; sound releases serialized");
    return true;
}
HookRegistrar registration{{"sound profiler lifetime", false, &install}};
}

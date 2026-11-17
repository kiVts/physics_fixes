#include "lua_tick_contract.h"
#include "game.h"
#include "log.h"
#include <windows.h>

namespace lua_tick {
namespace {
uintptr_t g_window_at = 0;
std::array<uint8_t, window_size> g_window{};
const Build* verified_live() {
    __try {
        const auto* image = reinterpret_cast<const uint8_t*>(game_base());
        const auto* dos = reinterpret_cast<const IMAGE_DOS_HEADER*>(image);
        if (dos->e_magic != IMAGE_DOS_SIGNATURE) return nullptr;
        const auto* nt = reinterpret_cast<const IMAGE_NT_HEADERS64*>(image + dos->e_lfanew);
        return nt->Signature == IMAGE_NT_SIGNATURE ? verified(image, nt->OptionalHeader.SizeOfImage, game_build()) : nullptr;
    } __except (EXCEPTION_EXECUTE_HANDLER) { return nullptr; }
}
}

// Called before vehicle_mt wraps the callback entry. The game's own call sequence is kept as it is; the
// borrowed string it passes lives as long as the module, so the existing cleanup is told not to free it.
void install() {
    if (!GetPrivateProfileIntW(L"vehicle_logic", L"lua_borrow_tick_name", 0, settings_ini())) return;
    const Build* b = verified_live();
    if (!b) {
        swlog("Lua tick name: build %08x not audited, or its native caller or lookup differs; original allocation retained", game_build());
        return;
    }
    auto* site = reinterpret_cast<uint8_t*>(game_base() + b->callback_rva + b->window_offset);
    const auto bytes = replacement(*b);
    DWORD protection;
    if (!VirtualProtect(site, bytes.size(), PAGE_EXECUTE_READWRITE, &protection)) {
        swlog("Lua tick name: could not make patch writable; original allocation retained");
        return;
    }
    std::memcpy(site, bytes.data(), bytes.size());
    DWORD unused;
    VirtualProtect(site, bytes.size(), protection, &unused);
    FlushInstructionCache(GetCurrentProcess(), site, bytes.size());
    g_window = bytes;
    g_window_at = reinterpret_cast<uintptr_t>(site);
    swlog("Lua tick name: verified; onTick borrows the native literal, removing one 32-byte allocation/free pair per eligible callback; fresh lookup and Lua locking retained");
}
bool installed_window(uintptr_t* at, const uint8_t** bytes, size_t* size) {
    if (!g_window_at) return false;
    *at = g_window_at;
    *bytes = g_window.data();
    *size = g_window.size();
    return true;
}
}

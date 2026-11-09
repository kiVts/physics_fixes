
#include "game.h"
#include "hooks.h"
#include "log.h"
#include "sdk.h"

#include <windows.h>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>

namespace {

constexpr uintptr_t kBoundingSphere = 0x140187c90;
constexpr uintptr_t kSceneDraw = 0x1406d2030;
constexpr double kFarM = 1e9;

using Sphere_t = void(__fastcall*)(const double* corners, void* a2, uint8_t* out);
using SceneDraw_t = void(__fastcall*)(void*, void*, void*, void*, void*, void*, void*, void*);
Sphere_t g_orig_sphere = nullptr;
SceneDraw_t g_orig_draw = nullptr;
thread_local const uint8_t* t_draw_arg = nullptr;
std::atomic<uint64_t> g_caught{0};
std::atomic<int> g_logged{0};
uintptr_t g_base = 0, g_size = 0;

bool sane(const double* c)
{
    __try {
        for (int i = 0; i < 24; i++)
            if (!std::isfinite(c[i]) || std::fabs(c[i]) > kFarM) return false;
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return true; // unreadable: leave it to the game
    }
}

void rtti(const uint8_t* obj, char* name, size_t size)
{
    name[0] = 0;
    __try {
        auto vt = *reinterpret_cast<const uintptr_t*>(obj);
        if (vt < g_base || vt >= g_base + g_size) return;
        auto col = *reinterpret_cast<const uint8_t* const*>(vt - 8);
        if (reinterpret_cast<uintptr_t>(col) < g_base || reinterpret_cast<uintptr_t>(col) >= g_base + g_size) return;
        uint32_t td = *reinterpret_cast<const uint32_t*>(col + 12);
        strncpy_s(name, size, reinterpret_cast<const char*>(g_base + td + 16), _TRUNCATE);
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        name[0] = 0;
    }
}

bool read3(const uint8_t* p, double* out)
{
    __try {
        memcpy(out, p, 3 * sizeof(double));
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

void report(const double* c)
{
    uint64_t n = g_caught.fetch_add(1) + 1;
    if (g_logged.fetch_add(1) >= 20) return;
    std::string around;
    const uint8_t* arg = t_draw_arg;
    if (arg) {
        double pos[3] = {};
        bool ok = read3(arg, pos);
        char buf[200];
        snprintf(buf, sizeof(buf), " scene-draw arg %p position %s(%.6g %.6g %.6g);", arg, ok ? "" : "unreadable ", pos[0], pos[1], pos[2]);
        around += buf;
        for (int back = 0; back <= 0x400; back += 8) {
            char cls[128];
            rtti(arg - back, cls, sizeof(cls));
            if (cls[0]) {
                snprintf(buf, sizeof(buf), " object at -0x%x: %s;", back, cls);
                around += buf;
                break;
            }
        }
    }
    void* frames[16];
    USHORT nf = RtlCaptureStackBackTrace(1, 16, frames, nullptr);
    std::string stack;
    for (USHORT i = 0; i < nf; i++) {
        auto a = reinterpret_cast<uintptr_t>(frames[i]);
        char buf[40];
        if (a >= g_base && a < g_base + g_size) snprintf(buf, sizeof(buf), " %llx", static_cast<unsigned long long>(a - g_base + 0x140000000));
        else snprintf(buf, sizeof(buf), " ?%llx", static_cast<unsigned long long>(a & 0xffffff));
        stack += buf;
    }
    swlog("render guard: object at an impossible position (%.6g %.6g %.6g), drawn with radius 0 instead of hanging (#%llu, "
          "thread %lu);%s stack:%s",
          c[0], c[1], c[2], static_cast<unsigned long long>(n), GetCurrentThreadId(), around.c_str(), stack.c_str());
}

void __fastcall sphere_detour(const double* corners, void* a2, uint8_t* out)
{
    if (sane(corners)) return g_orig_sphere(corners, a2, out);
    report(corners);
    __try {
        memcpy(out, corners, 3 * sizeof(double));
        *reinterpret_cast<float*>(out + 0x18) = 0.0f;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
    }
}

void __fastcall draw_detour(void* a1, void* a2, void* a3, void* a4, void* a5, void* a6, void* a7, void* a8)
{
    const uint8_t* prev = t_draw_arg;
    t_draw_arg = static_cast<const uint8_t*>(a3);
    g_orig_draw(a1, a2, a3, a4, a5, a6, a7, a8);
    t_draw_arg = prev;
}

void log_loop()
{
    uint64_t last = 0;
    for (;;) {
        Sleep(10000);
        uint64_t n = g_caught.load();
        if (n != last) swlog("render guard 10s: %llu impossible-position objects caught (%llu in total)", static_cast<unsigned long long>(n - last), static_cast<unsigned long long>(n));
        last = n;
        g_logged.store(std::min(g_logged.load(), 15)); // a few new detailed lines per window
    }
}

bool install()
{
    if (!GetPrivateProfileIntW(L"render", L"bounds_guard", 1, settings_ini())) {
        swlog("render guard: off in settings.ini");
        return true;
    }
    g_base = game_base();
    auto dos = reinterpret_cast<const IMAGE_DOS_HEADER*>(g_base);
    g_size = reinterpret_cast<const IMAGE_NT_HEADERS64*>(g_base + dos->e_lfanew)->OptionalHeader.SizeOfImage;
    if (!hook_game(kBoundingSphere, reinterpret_cast<void*>(&sphere_detour), reinterpret_cast<void**>(&g_orig_sphere)) ||
        !hook_game(kSceneDraw, reinterpret_cast<void*>(&draw_detour), reinterpret_cast<void**>(&g_orig_draw)))
        return false;
    CloseHandle(CreateThread(nullptr, 0, [](void*) -> DWORD { log_loop(); return 0; }, nullptr, 0, nullptr));
    swlog("render guard: on (objects beyond %.0e m or not finite get a zero bounding sphere instead of hanging the render "
          "thread; \"render guard\" lines)", kFarM);
    return true;
}

HookRegistrar reg{{"render guard", /*needs_known_build=*/true, &install}};

} // namespace

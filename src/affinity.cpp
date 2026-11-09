
#include "hooks.h"
#include "log.h"
#include "sdk.h"

#include <windows.h>
#include <tlhelp32.h>
#include <atomic>
#include <cstdint>
#include <vector>

namespace {

int g_mode = 0;
std::atomic<DWORD> g_update_tid{0};
KAFFINITY g_core = 0, g_all = 0; // the update thread's core; every logical processor of the process

bool find_core()
{
    DWORD len = 0;
    GetLogicalProcessorInformationEx(RelationProcessorCore, nullptr, &len);
    std::vector<uint8_t> buf(len);
    auto* info = reinterpret_cast<SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX*>(buf.data());
    if (!GetLogicalProcessorInformationEx(RelationProcessorCore, info, &len)) return false;
    DWORD_PTR proc = 0, sys = 0;
    GetProcessAffinityMask(GetCurrentProcess(), &proc, &sys);
    g_all = proc;
    for (DWORD off = 0; off < len;) {
        auto* e = reinterpret_cast<SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX*>(buf.data() + off);
        if (e->Relationship == RelationProcessorCore && e->Processor.GroupMask[0].Group == 0) {
            KAFFINITY m = e->Processor.GroupMask[0].Mask & proc;
            if (m && m != proc) g_core = m; // the last core wins
        }
        off += e->Size;
    }
    return g_core != 0;
}

void apply(bool on)
{
    DWORD self = GetCurrentThreadId(), pid = GetCurrentProcessId(), upd = g_update_tid.load();
    HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPTHREAD, 0);
    if (snap == INVALID_HANDLE_VALUE) return;
    THREADENTRY32 te{sizeof te};
    for (BOOL ok = Thread32First(snap, &te); ok; ok = Thread32Next(snap, &te)) {
        if (te.th32OwnerProcessID != pid || te.th32ThreadID == self) continue;
        HANDLE t = OpenThread(THREAD_SET_INFORMATION | THREAD_QUERY_INFORMATION, FALSE, te.th32ThreadID);
        if (!t) continue;
        KAFFINITY mask = !on ? g_all : te.th32ThreadID == upd ? g_core : g_all & ~g_core;
        SetThreadAffinityMask(t, mask);
        CloseHandle(t);
    }
    CloseHandle(snap);
}

DWORD WINAPI loop(void*)
{
    while (!g_update_tid.load()) Sleep(500);
    bool on = g_mode == 1 || g_mode == 2;
    ULONGLONG switched = GetTickCount64();
    swlog("isolate update: %s window (update thread %lu, core mask %#llx of %#llx)", on ? "ON" : "OFF",
          g_update_tid.load(), (unsigned long long)g_core, (unsigned long long)g_all);
    for (;;) {
        apply(on);
        Sleep(2000);
        if (g_mode == 2 && GetTickCount64() - switched >= 30000) {
            on = !on;
            switched = GetTickCount64();
            swlog("isolate update: %s window", on ? "ON" : "OFF");
        }
    }
}

bool install()
{
    g_mode = static_cast<int>(GetPrivateProfileIntW(L"parallel", L"isolate_update", 1, settings_ini()));
    if (g_mode <= 0) return true;
    if (!find_core()) {
        swlog("isolate update: no separate physical core found; off");
        return true;
    }
    if (!swml()->on_loop(swml_self(), [](void*) {
            if (!g_update_tid.load(std::memory_order_relaxed)) g_update_tid.store(GetCurrentThreadId());
        }, nullptr, nullptr, nullptr)) {
        swlog("isolate update: no game loop events; off");
        return true;
    }
    CloseHandle(CreateThread(nullptr, 0, loop, nullptr, 0, nullptr));
    swlog("isolate update: %s", g_mode == 2 ? "A/B every 30 s" : "on");
    return true;
}

HookRegistrar reg{{"isolate update", /*needs_known_build=*/false, &install}};

} // namespace

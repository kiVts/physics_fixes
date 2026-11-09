
#include "hooks.h"
#include "log.h"
#include "sdk.h"

#include <windows.h>
#include <intrin.h>
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace {

constexpr uintptr_t kStartProfile = 0x140163440, kStopProfile = 0x140163360;

using Start_t = void(__fastcall*)(const char* name);
using Stop_t = void(__fastcall*)();
Start_t g_start = nullptr;
Stop_t g_stop = nullptr;

// --- per-thread scope tree ------------------------------------------------------------------------
constexpr int kNodes = 1024, kHash = 4096, kDepth = 64, kTables = 64;

struct Node {
    const char* name;
    int parent;
    volatile uint64_t calls, cycles; // written by the owning thread only; read (racy, 64-bit atomic) by the logger
};

struct Table {
    DWORD tid;
    volatile int count;
    int depth, lost;
    int stack[kDepth];
    uint64_t t0[kDepth];
    int16_t hash[kHash]; // node index + 1
    Node nodes[kNodes];
};

Table* g_tables[kTables];
std::atomic<int> g_ntables{0};
thread_local Table* tl_table = nullptr;

Table* table()
{
    if (tl_table) return tl_table;
    int i = g_ntables.fetch_add(1);
    if (i >= kTables) return nullptr;
    auto* t = static_cast<Table*>(VirtualAlloc(nullptr, sizeof(Table), MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE));
    if (!t) return nullptr;
    t->tid = GetCurrentThreadId();
    g_tables[i] = t;
    tl_table = t;
    return t;
}

int node_index(Table* t, int parent, const char* name)
{
    uintptr_t h = ((reinterpret_cast<uintptr_t>(name) >> 3) * 0x9E3779B1u + uint32_t(parent + 1) * 0x85EBCA6Bu) & (kHash - 1);
    for (int probe = 0; probe < kHash; probe++) {
        int16_t& slot = t->hash[(h + probe) & (kHash - 1)];
        if (slot == 0) {
            if (t->count >= kNodes) return -1;
            int idx = t->count;
            t->nodes[idx].name = name;
            t->nodes[idx].parent = parent;
            t->nodes[idx].calls = 0;
            t->nodes[idx].cycles = 0;
            slot = static_cast<int16_t>(idx + 1);
            t->count = idx + 1; // publish after the node is filled in
            return idx;
        }
        const Node& n = t->nodes[slot - 1];
        if (n.name == name && n.parent == parent) return slot - 1;
    }
    return -1;
}

void __fastcall start_detour(const char* name)
{
    uint64_t now = __rdtsc();
    Table* t = table();
    if (!t) return;
    if (t->depth >= kDepth) {
        t->lost++;
        return;
    }
    int parent = t->depth ? t->stack[t->depth - 1] : -1;
    int idx = parent == -2 ? -2 : node_index(t, parent, name);
    t->stack[t->depth] = idx < 0 ? -2 : idx;
    t->t0[t->depth] = now;
    t->depth++;
}

void __fastcall stop_detour()
{
    uint64_t now = __rdtsc();
    Table* t = tl_table;
    if (!t) return;
    if (t->lost) {
        t->lost--;
        return;
    }
    if (!t->depth) return; // a Stop whose Start was inlined in the game (stepSimulation's own scope)
    t->depth--;
    int idx = t->stack[t->depth];
    if (idx < 0) return;
    Node& n = t->nodes[idx];
    n.calls = n.calls + 1;
    n.cycles = n.cycles + (now - t->t0[t->depth]);
}

void __fastcall empty_start(const char*) {}
void __fastcall empty_stop() {}

// --- logger ----------------------------------------------------------------------------------------
struct Prev {
    uint64_t calls, cycles;
};
std::vector<Prev> g_prev[kTables];

void print_children(const Table* t, const std::vector<Prev>& d, int parent, int depth, double cyc_per_ms, double secs,
                    double parent_ms)
{
    std::vector<int> kids;
    for (int i = 0; i < static_cast<int>(d.size()); i++)
        if (t->nodes[i].parent == parent && d[i].cycles) kids.push_back(i);
    std::sort(kids.begin(), kids.end(), [&](int a, int b) { return d[a].cycles > d[b].cycles; });
    for (int i : kids) {
        double ms = d[i].cycles / cyc_per_ms / secs;
        if (ms < 0.02) continue;
        char indent[40] = "";
        for (int k = 0; k < depth && k < 18; k++) strcat_s(indent, "  ");
        swlog("  %s%-*s %8.2f ms/s %9.0f calls/s %8.3f ms/call %5.1f%% of parent", indent, std::max(1, 46 - 2 * depth),
              t->nodes[i].name, ms, d[i].calls / secs, d[i].cycles / cyc_per_ms / std::max<uint64_t>(1, d[i].calls),
              parent_ms > 0 ? 100.0 * ms / parent_ms : 100.0);
        if (depth < 12) print_children(t, d, i, depth + 1, cyc_per_ms, secs, ms);
    }
}

DWORD WINAPI logger_thread(void*)
{
    LARGE_INTEGER freq, q0, q1;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&q0);
    uint64_t r0 = __rdtsc();
    for (;;) {
        Sleep(5000);
        QueryPerformanceCounter(&q1);
        uint64_t r1 = __rdtsc();
        double secs = double(q1.QuadPart - q0.QuadPart) / double(freq.QuadPart);
        double cyc_per_ms = double(r1 - r0) / (secs * 1000.0);
        q0 = q1;
        r0 = r1;
        int nt = std::min(g_ntables.load(), kTables);
        for (int ti = 0; ti < nt; ti++) {
            const Table* t = g_tables[ti];
            if (!t) continue;
            int count = t->count;
            std::vector<Prev>& prev = g_prev[ti];
            prev.resize(count, Prev{0, 0});
            std::vector<Prev> d(count);
            double total = 0;
            for (int i = 0; i < count; i++) {
                uint64_t c = t->nodes[i].calls, y = t->nodes[i].cycles;
                d[i] = Prev{c - prev[i].calls, y - prev[i].cycles};
                prev[i] = Prev{c, y};
                if (t->nodes[i].parent == -1) total += d[i].cycles / cyc_per_ms / secs;
            }
            if (total < 0.2) continue;
            swlog("bullet 5s [tid %lu]: %.2f ms/s in top-level Bullet scopes", t->tid, total);
            print_children(t, d, -1, 0, cyc_per_ms, secs, total);
        }
    }
}

bool install()
{
    std::wstring ini = settings_ini();
    bool profile = GetPrivateProfileIntW(L"bullet", L"profile", 0, ini.c_str()) != 0;
    bool off = GetPrivateProfileIntW(L"bullet", L"profiler_off", 1, ini.c_str()) != 0;
    if (!profile && !off) {
        swlog("bullet profiler: left as the game has it (settings.ini)");
        return true;
    }
    void* start = profile ? reinterpret_cast<void*>(&start_detour) : reinterpret_cast<void*>(&empty_start);
    void* stop = profile ? reinterpret_cast<void*>(&stop_detour) : reinterpret_cast<void*>(&empty_stop);
    if (!hook_game(kStartProfile, start, reinterpret_cast<void**>(&g_start))) return false;
    if (!hook_game(kStopProfile, stop, reinterpret_cast<void**>(&g_stop))) return false;
    if (profile) {
        if (HANDLE h = CreateThread(nullptr, 0, logger_thread, nullptr, 0, nullptr)) CloseHandle(h);
        swlog("bullet profiler: replaced by scope timing (\"bullet 5s\" lines every 5 s)");
    } else {
        swlog("bullet profiler: off");
    }
    return true;
}

HookRegistrar reg{{"bullet_profiler", /*needs_known_build=*/true, &install}};

} // namespace

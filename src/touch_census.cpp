
#include "game.h"
#include "hooks.h"
#include "log.h"
#include "world.h"
#include "vehicle_push.h"

#include <windows.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include "mt_dispatch.h"

namespace {

constexpr uintptr_t kIsHost = 0x140ceb6ff;
constexpr size_t kWorldObjCount = 0x0c, kWorldObjData = 0x18;
constexpr size_t kObjOrigin = 0x70, kObjProxy = 0x180, kObjFlags = 0x1a0, kObjInternalType = 0x1f0;
constexpr size_t kProxyMin = 0x20, kProxyMax = 0x40, kSceneRigid = 0xb8;
constexpr size_t kCharSeat=0x2100;
constexpr size_t kCharBodies[3] = {0x478, 0x480, 0x488};
constexpr int kCoRigid = 2, kEvery = 8, kMax = 32;
constexpr double kMargin = 0.05, kNearM = 10.0;
constexpr ULONGLONG kFreshMs = 1000;

bool g_on = false;
const volatile uint8_t* g_is_host = nullptr;
unsigned g_calls = 0;
SRWLOCK g_lock = SRWLOCK_INIT;
const void* g_list[kMax];
int g_n = 0;
ULONGLONG g_at = 0;

template <typename T> T rd(const void* p, size_t off) { return *reinterpret_cast<const T*>(static_cast<const uint8_t*>(p) + off); }

struct Box {
    double mn[3], mx[3];
};

bool box_of(const uint8_t* body, Box& b)
{
    const uint8_t* proxy = rd<const uint8_t*>(body, kObjProxy);
    if (!proxy) return false;
    memcpy(b.mn, proxy + kProxyMin, 24);
    memcpy(b.mx, proxy + kProxyMax, 24);
    for (int i = 0; i < 3; i++)
        if (!std::isfinite(b.mn[i]) || !std::isfinite(b.mx[i])) return false;
    return true;
}

bool touch(const Box& a, const Box& b)
{
    for (int i = 0; i < 3; i++)
        if (a.mn[i] > b.mx[i] + kMargin || b.mn[i] > a.mx[i] + kMargin) return false;
    return true;
}

int scan(const uint8_t* world, const uint8_t* ch, const void** out)
{
    const uint8_t* mine[3] = {};
    Box me[3];
    int nme = 0;
    double c[3] = {};
    for (size_t off : kCharBodies) {
        const uint8_t* so = rd<const uint8_t*>(ch, off);
        const uint8_t* body = so ? rd<const uint8_t*>(so, kSceneRigid) : nullptr;
        if (body && box_of(body, me[nme])) {
            mine[nme] = body;
            for (int i = 0; i < 3; i++) c[i] = (me[nme].mn[i] + me[nme].mx[i]) / 2;
            nme++;
        }
    }
    if (!nme) return 0;
    int n = rd<int>(world, kWorldObjCount);
    const uint8_t* const* objs = rd<const uint8_t* const*>(world, kWorldObjData);
    if (n <= 0 || n > 1000000 || !objs) return 0;
    int found = 0;
    for (int i = 0; i < n && found < kMax; i++) {
        const uint8_t* o = objs[i];
        if (!o || o == mine[0] || o == mine[1] || o == mine[2]) continue;
        const double* p = reinterpret_cast<const double*>(o + kObjOrigin);
        double dx = p[0] - c[0], dy = p[1] - c[1], dz = p[2] - c[2];
        if (!(dx * dx + dy * dy + dz * dz < kNearM * kNearM)) continue;
        if (rd<int>(o, kObjInternalType) != kCoRigid || (rd<int>(o, kObjFlags) & 1)) continue;
        Box b;
        if (!box_of(o, b)) continue;
        for (int j = 0; j < nme; j++)
            if (touch(me[j], b)) {
                out[found++] = o;
                break;
            }
    }
    return found;
}

int scan_safe(const uint8_t* world, const uint8_t* ch, const void** out)
{
    __try {
        return scan(world, ch, out);
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return -1;
    }
}

// For the push investigation: current nearby dynamic bodies and their actual supporting contacts.
// Read between steps, never from a worker. Revalidate watched bodies through the world's current object array.
struct SupportWatch { const uint8_t* body = nullptr; ULONGLONG until = 0; };
SupportWatch g_support[16];
ULONGLONG g_support_log = 0;
bool g_support_on = true;

void audit_support(const uint8_t* world, const void** touched, int nt)
{
    ULONGLONG now = GetTickCount64();
    int no = rd<int>(world, kWorldObjCount);
    auto objects = rd<const uint8_t* const*>(world,kWorldObjData);
    if (no < 0 || no > 1000000 || (no && !objects)) return;
    for (int j = 0; j < nt; j++) {
        auto b = static_cast<const uint8_t*>(touched[j]);
        auto so = rd<const uint8_t*>(b,0x1f8);
        auto owner = so ? rd<const uint8_t*>(so,0xc0) : nullptr;
        int kind = owner ? rd<int>(owner,0) : -1;
        if (kind == 2) continue; // character
        int slot = -1;
        for (int i = 0; i < 16; i++) if (g_support[i].body == b) { slot = i; break; }
        if (slot < 0) for (int i = 0; i < 16; i++) if (now > g_support[i].until) {slot = i; break;}
        if (slot >= 0) g_support[slot] = {b,now+8000};
    }
    if (now-g_support_log < 500) return;
    g_support_log = now;
    auto dispatcher = rd<const uint8_t*>(world,0x28);
    int nm = dispatcher ? rd<int>(dispatcher,0x14) : 0;
    auto manifolds = dispatcher ? rd<const uint8_t* const*>(dispatcher,0x20) : nullptr;
    if (nm < 0 || nm > 1000000 || (nm && !manifolds)) return;
    for (auto& w : g_support) {
        if (!w.body || now > w.until) continue;
        bool alive = false;
        for (int i = 0; i < no; i++) if (objects[i] == w.body) {alive=true;break;}
        if (!alive) {w={};continue;}
        auto b=w.body;
        auto proxy=rd<const uint8_t*>(b,kObjProxy);
        auto so=rd<const uint8_t*>(b,0x1f8);
        auto owner=so?rd<const uint8_t*>(so,0xc0):nullptr;
        int kind=owner?rd<int>(owner,0):-1;
        int movable=kind==3?rd<int>(owner,0x40):-1; // movable owner at +220, kind +260
        int contacts=0,support=0,static_contacts=0;
        double deepest=0,up=0,physical_depth=0,physical_ny=0; int physical=0;
        for(int i=0;i<nm;i++) {
            auto m=manifolds[i]; if(!m)continue;
            auto a=rd<const uint8_t*>(m,0x5d0), other=rd<const uint8_t*>(m,0x5d8);
            bool first=a==b;
            if(!first && other!=b)continue;
            if(!first)other=a;
            int n=rd<int>(m,0x5e0); if(n<0||n>4)continue;
            int of=rd<int>(other,kObjFlags);
            for(int j=0;j<n;j++) {
                auto pt=m+0x10+j*0x170;
                double d=rd<double>(pt,0xa0);
                double ny=rd<double>(pt,0x88)*(first?1:-1);
                if(d>.02)continue;
                contacts++;
                if(of&1)static_contacts++;
                if(!(of&4) && ny>.5) support++;
                if(d<deepest){deepest=d;up=ny;}
                if(!(of&4) && !(rd<int>(b,kObjFlags)&4) && rd<int>(other,kObjInternalType)==2) {
                    physical++;
                    if(d<physical_depth){physical_depth=d;physical_ny=ny;}
                }
            }
        }
        Box box{};box_of(b,box);
        swlog("push support (%s): body %p owner %d movable 0x%x mass %.3g flags 0x%x filter 0x%x/0x%x y %.3f low %.3f vy %.3f; contacts %d static %d support %d deepest %.4f normalY %.3f; response %d deepest %.4f normalY %.3f",
              *g_is_host?"host":"client",b,kind,movable,1.0/rd<double>(b,0x3d0),rd<int>(b,kObjFlags),
              proxy?rd<int>(proxy,8):0,proxy?rd<int>(proxy,0xc):0,rd<double>(b,0x78),box.mn[1],rd<double>(b,0x398),
              contacts,static_contacts,support,deepest,up,physical,physical_depth,physical_ny);
    }
}
#include "support_trace.inc"
#include "vehicle_reaction.inc"
#include "character_launch_trace.inc"

void audit_support_safe(const uint8_t* world, const void** touched, int nt)
{
    __try {audit_support(world,touched,nt);}
    __except(EXCEPTION_EXECUTE_HANDLER) {g_support_on=false;swlog("push support: audit read failed, disabled");}
}

bool install()
{
    g_solver_ab = GetPrivateProfileIntW(L"touch",L"solver_ab",0,settings_ini()) != 0;
    g_support_on = GetPrivateProfileIntW(L"touch",L"support_audit",1,settings_ini()) != 0;
    g_on = GetPrivateProfileIntW(L"touch", L"census", 1, settings_ini()) != 0;
    g_is_host = game_ptr<const volatile uint8_t*>(kIsHost);
    if (!g_is_host) g_on = false;
    swlog("touch census: %s", g_on ? "on (client: the bodies your character touches, for network_fixes' object push log)"
                                   : "off");
    return true;
}

HookRegistrar reg{{"touch census", /*needs_known_build=*/true, &install}};

} // namespace

void touch_step_begin(const void* world)
{
    if (!g_on || !world || ++g_calls % kEvery) return;
    const void* own = nullptr;
    double pos[3];
    if (!local_player_pos(pos, &own) || !own) return;
    const void* found[kMax];
    int n = scan_safe(static_cast<const uint8_t*>(world), static_cast<const uint8_t*>(own), found);
    if (n < 0) return;
    if(g_support_on) audit_support_safe(static_cast<const uint8_t*>(world),found,n);
    AcquireSRWLockExclusive(&g_lock);
    memcpy(g_list, found, sizeof(found[0]) * size_t(n));
    g_n = n;
    g_at = GetTickCount64();
    ReleaseSRWLockExclusive(&g_lock);
}

// network_fixes object_push.cpp (update thread): the Bullet bodies the own character touched at the last scan (within
// the last second), at most max. The bodies may be gone by the time they are read: callers guard their reads.
extern "C" __declspec(dllexport) int __cdecl swml_touched_bodies(const void** out, int max)
{
    AcquireSRWLockShared(&g_lock);
    int n = GetTickCount64() - g_at <= kFreshMs ? (g_n < max ? g_n : max) : 0;
    memcpy(out, g_list, sizeof(out[0]) * size_t(n));
    ReleaseSRWLockShared(&g_lock);
    return n;
}

// Diagnostic entry points. Faults disable this audit without changing game state.
void touch_solver_begin(const void* world) {
    __try {solver_trace_begin(static_cast<const uint8_t*>(world));}
    __except(EXCEPTION_EXECUTE_HANDLER) {g_solver_watch_n=0;g_support_on=false;}
}
bool touch_solver_native() {return g_solver_native;}
void touch_solver_setup(void* solver, void** bodies, int nb, void** mans, int nm) {
    __try {solver_trace_setup(solver,bodies,nb,mans,nm);}
    __except(EXCEPTION_EXECUTE_HANDLER) {}
}
void touch_solver_finish(void* solver) {
    __try {solver_trace_finish(solver);}
    __except(EXCEPTION_EXECUTE_HANDLER) {}
}
void touch_solver_end() {
    __try {solver_trace_end();}
    __except(EXCEPTION_EXECUTE_HANDLER) {g_solver_watch_n=0;g_support_on=false;}
}

void vehicle_push_step_end(const void* world,double fixed) {
    SwmlVehiclePushSample samples[16]{};int n=0;
    __try {sample_vehicle_reaction(static_cast<const uint8_t*>(world),fixed,samples,n);}
    __except(EXCEPTION_EXECUTE_HANDLER) {n=0;}
    AcquireSRWLockExclusive(&g_reaction_lock);
    memcpy(g_reaction_samples,samples,size_t(n)*sizeof(samples[0]));g_reaction_n=n;
    ++g_reaction_seq;g_reaction_at=GetTickCount64();ReleaseSRWLockExclusive(&g_reaction_lock);
}
extern "C" __declspec(dllexport) int __cdecl swml_vehicle_push_samples(SwmlVehiclePushSample* out,int max,uint32_t* seq) {
    if(!out || max<0 || !seq)return 0;
    AcquireSRWLockShared(&g_reaction_lock);*seq=g_reaction_seq;
    int n=GetTickCount64()-g_reaction_at<100?std::min(max,g_reaction_n):0;
    memcpy(out,g_reaction_samples,size_t(n)*sizeof(out[0]));ReleaseSRWLockShared(&g_reaction_lock);return n;
}

void character_launch_begin(const void* world,double elapsed,double fixed) {
    __try { launch_begin(static_cast<const uint8_t*>(world),elapsed,fixed); }
    __except(EXCEPTION_EXECUTE_HANDLER) {g_launch_pending=false;}
}
void character_launch_end(const void* world) {
    __try {launch_end(static_cast<const uint8_t*>(world));}
    __except(EXCEPTION_EXECUTE_HANDLER) {g_launch_pending=false;}
}

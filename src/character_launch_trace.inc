// Read-only rolling history of the local walking character around a launch.
// This records native Bullet fields; it never clamps velocities or moves bodies.
struct LaunchBody {const uint8_t* body=nullptr;int flags=0;double p[3]{},v[3]{},w[3]{},force[3]{};};
struct LaunchContact {const void* other=nullptr;int slot=0,flags=0;double depth=0,impulse=0,lateral[2]{},normal[3]{},v[3]{},point[3]{};};
struct LaunchFrame {uint64_t tick=0;DWORD at=0;const void* ch=nullptr;const void* held=nullptr;float yaw=0,pitch=0;double elapsed=0,fixed=0;LaunchBody pre[3]{},post[3]{};LaunchContact contacts[8]{};int nc=0;};
LaunchFrame g_launch_frames[32]{};
unsigned g_launch_head=0,g_launch_count=0;
uint64_t g_launch_tick=0;
DWORD g_launch_logged=0;
bool g_launch_trace=true,g_launch_pending=false;
void launch_read_bodies(const uint8_t* ch,LaunchBody out[3]) {
    for(int j=0;j<3;j++) {
        auto so=rd<const uint8_t*>(ch,kCharBodies[j]);auto b=so?rd<const uint8_t*>(so,kSceneRigid):nullptr;
        if(!b)continue;out[j].body=b;out[j].flags=rd<int>(b,kObjFlags);
        memcpy(out[j].p,b+kObjOrigin,24);memcpy(out[j].v,b+0x390,24);memcpy(out[j].w,b+0x3b0,24);memcpy(out[j].force,b+0x460,24);
    }
}
void launch_begin(const uint8_t* world,double elapsed,double fixed) {
    g_launch_pending=false;
    if(!g_launch_trace || !world || !g_is_host || *g_is_host)return;
    const void* own=nullptr;double pos[3];
    if(!local_player_pos(pos,&own) || !own || rd<const void*>(own,kCharSeat)) {g_launch_count=0;return;}
    auto ch=static_cast<const uint8_t*>(own);auto& f=g_launch_frames[g_launch_head];f={};
    f.tick=++g_launch_tick;f.at=GetTickCount();f.ch=ch;f.held=rd<const void*>(ch,0x2110);
    f.yaw=rd<float>(ch,0x2400);f.pitch=rd<float>(ch,0x2404);f.elapsed=elapsed;f.fixed=fixed;
    launch_read_bodies(ch,f.pre);g_launch_pending=true;
}
double launch_speed(const double v[3]) {return std::hypot(v[0],v[1],v[2]);}
void launch_end(const uint8_t* world) {
    if(!g_launch_pending)return;g_launch_pending=false;
    auto& f=g_launch_frames[g_launch_head];launch_read_bodies(static_cast<const uint8_t*>(f.ch),f.post);
    auto dispatcher=rd<const uint8_t*>(world,0x28);int nm=dispatcher?rd<int>(dispatcher,0x14):0;
    auto mans=dispatcher?rd<const uint8_t* const*>(dispatcher,0x20):nullptr;
    if(nm>=0 && nm<=1000000 && (!nm || mans))for(int i=0;i<nm;i++) {
        auto m=mans[i];if(!m)continue;auto a=rd<const uint8_t*>(m,0x5d0),b=rd<const uint8_t*>(m,0x5d8);
        for(int j=0;j<3;j++) {
            bool first=a && a==f.post[j].body;if(!first && (!b || b!=f.post[j].body))continue;
            auto other=first?b:a;if(!other)continue;int np=rd<int>(m,0x5e0);if(np<0 || np>4)continue;
            for(int k=0;k<np;k++) {
                auto p=m+0x10+k*0x170;LaunchContact c{};c.other=other;c.slot=j;c.flags=rd<int>(other,kObjFlags);
                c.depth=rd<double>(p,0xa0);c.impulse=rd<double>(p,0xe8);c.lateral[0]=rd<double>(p,0xf0);c.lateral[1]=rd<double>(p,0xf8);
                for(int axis=0;axis<3;axis++){c.normal[axis]=rd<double>(p,0x80+axis*8)*(first?1:-1);c.point[axis]=rd<double>(p,0x60+axis*8);}
                if(rd<int>(other,kObjInternalType)==2)memcpy(c.v,other+0x390,24);
                int dest=f.nc<8?f.nc++:0;
                if(f.nc==8)for(int t=1;t<8;t++)if(std::fabs(f.contacts[t].impulse)<std::fabs(f.contacts[dest].impulse))dest=t;
                if(!f.contacts[dest].other || std::fabs(c.impulse)>=std::fabs(f.contacts[dest].impulse))f.contacts[dest]=c;
            }
        }
    }
    bool trigger=false;
    for(int j=0;j<3;j++)if(f.post[j].body && !(f.post[j].flags&7)) {
        double speed=launch_speed(f.post[j].v),old=launch_speed(f.pre[j].v);
        trigger=trigger || !std::isfinite(speed) || speed>80 || (speed>35 && speed-old>20);
        double dp[3];for(int a=0;a<3;a++)dp[a]=f.post[j].p[a]-f.pre[j].p[a];
        trigger=trigger || launch_speed(dp)>5;
        if(g_launch_count) {
            const auto& previous=g_launch_frames[(g_launch_head+31)%32];
            if(previous.ch==f.ch && previous.post[j].body==f.pre[j].body) {
                for(int a=0;a<3;a++)dp[a]=f.pre[j].p[a]-previous.post[j].p[a];
                trigger=trigger || launch_speed(dp)>5;
            }
        }

    }
    g_launch_head=(g_launch_head+1)%32;g_launch_count=std::min(32u,g_launch_count+1);
    if(!trigger || (g_launch_logged && f.at-g_launch_logged<10000))return;
    g_launch_logged=f.at;
    swlog("character launch trace: %u completed physics calls, triggered tick %llu; read-only history (pre/post velocities m/s, contact impulses N.s)",g_launch_count,(unsigned long long)f.tick);
    for(unsigned i=0;i<g_launch_count;i++) {
        const auto& h=g_launch_frames[(g_launch_head+32-g_launch_count+i)%32];
        swlog("character launch frame: tick %llu at %lu ch %p held %p yaw %.6g pitch %.6g elapsed %.6g fixed %.6g",(unsigned long long)h.tick,h.at,h.ch,h.held,h.yaw,h.pitch,h.elapsed,h.fixed);
        for(int j=0;j<3;j++)if(h.post[j].body) {
            const auto& a=h.pre[j];const auto& b=h.post[j];
            swlog("character launch body: slot %d body %p flags %x -> %x p (%.9g %.9g %.9g) -> (%.9g %.9g %.9g) v (%.9g %.9g %.9g) -> (%.9g %.9g %.9g) w (%.6g %.6g %.6g) pre-force (%.9g %.9g %.9g)",j,b.body,a.flags,b.flags,a.p[0],a.p[1],a.p[2],b.p[0],b.p[1],b.p[2],a.v[0],a.v[1],a.v[2],b.v[0],b.v[1],b.v[2],b.w[0],b.w[1],b.w[2],a.force[0],a.force[1],a.force[2]);
        }
        for(int j=0;j<h.nc;j++) {const auto& c=h.contacts[j];swlog("character launch contact: slot %d other %p flags %x depth %.9g impulse %.9g friction (%.9g %.9g) normal (%.6g %.6g %.6g) other-v (%.6g %.6g %.6g) point (%.9g %.9g %.9g)",c.slot,c.other,c.flags,c.depth,c.impulse,c.lateral[0],c.lateral[1],c.normal[0],c.normal[1],c.normal[2],c.v[0],c.v[1],c.v[2],c.point[0],c.point[1],c.point[2]);}
    }
}

// Last solved substep gives the steady contact force I/dt against replay-driven vehicle bodies.
// Transfer the reciprocal horizontal force to the host, where the large vehicle remains simulated.
SRWLOCK g_reaction_lock=SRWLOCK_INIT;
SwmlVehiclePushSample g_reaction_samples[16]{};
int g_reaction_n=0;
uint32_t g_reaction_seq=0;
ULONGLONG g_reaction_at=0;
void sample_vehicle_reaction(const uint8_t* world,double dt,SwmlVehiclePushSample out[16],int& count)
{
    count=0;
    if(!g_is_host || *g_is_host || !(dt>0) || dt>.1)return;
    const void* own=nullptr;double pos[3];
    if(!local_player_pos(pos,&own) || !own || rd<const void*>(own,kCharSeat))return;
    const uint8_t* mine[3]{};
    for(int j=0;j<3;j++){auto so=rd<const uint8_t*>(own,kCharBodies[j]);mine[j]=so?rd<const uint8_t*>(so,kSceneRigid):nullptr;}
    auto dispatcher=rd<const uint8_t*>(world,0x28);if(!dispatcher)return;
    int nm=rd<int>(dispatcher,0x14);auto mans=rd<const uint8_t* const*>(dispatcher,0x20);
    if(nm<0 || nm>1000000 || (nm && !mans))return;
    double weights[16]{},points[16][3]{},forces[16][3]{};
    for(int i=0;i<nm;i++) {
        auto m=mans[i];auto a=rd<const uint8_t*>(m,0x5d0),b=rd<const uint8_t*>(m,0x5d8);
        bool am=a && (a==mine[0] || a==mine[1] || a==mine[2]);
        bool bm=b && (b==mine[0] || b==mine[1] || b==mine[2]);
        if(am==bm)continue;
        auto me=am?a:b;auto vehicle=am?b:a;
        if((rd<int>(me,kObjFlags)&4) || !vehicle || rd<int>(vehicle,kObjInternalType)!=2)continue;
        int flags=rd<int>(vehicle,kObjFlags);if(!(flags&0x800) || (flags&7))continue;
        auto so=rd<const uint8_t*>(vehicle,0x1f8);auto owner=so?rd<const uint8_t*>(so,0xc0):nullptr;
        if(!owner || rd<int>(owner,0)!=1)continue;
        int np=rd<int>(m,0x5e0);if(np<0 || np>4)continue;
        int slot=-1;for(int j=0;j<count;j++)if(out[j].body==vehicle)slot=j;
        if(slot<0){if(count==16)continue;slot=count++;out[slot]={};out[slot].body=vehicle;}
        for(int j=0;j<np;j++) {
            auto p=m+0x10+j*0x170;if(rd<double>(p,0xa0)>.02)continue;
            double normal=rd<double>(p,0xe8),lat1=rd<double>(p,0xf0),lat2=rd<double>(p,0xf8);
            if(!std::isfinite(normal) || !std::isfinite(lat1) || !std::isfinite(lat2) || normal<0)continue;
            double impulse[3],offset[3];
            for(int k=0;k<3;k++) {
                impulse[k]=(am?-1:1)*(normal*rd<double>(p,0x80+8*k)+lat1*rd<double>(p,0x130+8*k)+lat2*rd<double>(p,0x150+8*k));
                offset[k]=rd<double>(p,(am?0x40:0x60)+8*k)-rd<double>(vehicle,kObjOrigin+8*k);
            }
            impulse[1]=0;double weight=std::hypot(impulse[0],impulse[2]);
            if(!(weight>1e-8) || !std::isfinite(weight))continue;
            weights[slot]+=weight;
            for(int k=0;k<3;k++) {
                double lf=0,lp=0;
                for(int axis=0;axis<3;axis++) {
                    double r=rd<double>(vehicle,0x10+axis*0x20+k*8);
                    lf+=r*impulse[axis]/dt;lp+=r*offset[axis];
                }
                forces[slot][k]+=lf;points[slot][k]+=lp*weight;
            }
        }
    }
    int n=0;double total=0;
    for(int j=0;j<count;j++)if(weights[j]>0) {
        double mag=std::sqrt(forces[j][0]*forces[j][0]+forces[j][1]*forces[j][1]+forces[j][2]*forces[j][2]);
        if(!std::isfinite(mag) || mag<.01)continue;
        out[n].body=out[j].body;
        for(int k=0;k<3;k++){out[n].point[k]=float(points[j][k]/weights[j]);out[n].force[k]=float(forces[j][k]);}
        total+=mag;++n;
    }
    count=n;
    if(total>1500)for(int j=0;j<count;j++)for(float& f:out[j].force)f=float(f*1500/total);
}

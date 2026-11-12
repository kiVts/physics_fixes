#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <initializer_list>

namespace early_contract {
inline uint32_t u32(const uint8_t* p) { uint32_t v; std::memcpy(&v,p,4); return v; }
inline size_t count(const uint8_t* code,size_t size,std::initializer_list<uint8_t> pattern) {
    size_t hits=0;
    for(size_t i=0;i+pattern.size()<=size;++i)
        if(!std::memcmp(code+i,pattern.begin(),pattern.size()))++hits;
    return hits;
}
inline bool field(const uint8_t* code,size_t size,std::initializer_list<uint8_t> prefix,uint32_t offset) {
    size_t hits=0;
    for(size_t i=0;i+prefix.size()+4<=size;++i)
        if(!std::memcmp(code+i,prefix.begin(),prefix.size()) && u32(code+i+prefix.size())==offset)++hits;
    return hits==1;
}
struct Layout { uint32_t scene=0; };
inline bool async_mode(const uint8_t* code,size_t size) {
    const uint8_t expected[]={0x41,0x80,0xbf,0x68,0x0a,0,0,0};
    return size>=sizeof expected && !std::memcmp(code,expected,sizeof expected);
}
// Exact native instruction forms, not an entry signature standing in for field
// validation. The caller supplies the mapped gate-return and submit-return
// boundaries and the resolved submit target. Reject unfamiliar compilers/layouts.
inline bool decode(const uint8_t* code,size_t size,uintptr_t base,uintptr_t submit,Layout& result) {
    if(size<128 || size>2048)return false;
    if(std::memcmp(code,"\x84\xc0\x0f\x84",4))return false;
    if(std::memcmp(code+0x1a,"\x48\x81\xc3",3))return false;
    const uint32_t scene=u32(code+0x1d);
    if(scene<0x100 || scene>0x100000 || (scene&7))return false;
    if(code[size-5]!=0xe8)return false;
    int32_t relative=0;std::memcpy(&relative,code+size-4,4);
    if(base+size+relative!=submit)return false;
    // Queue header, executor ABI and queue-clear address preparation.
    if(count(code,size,{0x39,0x43,0x40})!=1 || count(code,size,{0x8b,0x43,0x3c})!=1 ||
       count(code,size,{0xf7,0x73,0x38})!=1 || count(code,size,{0x48,0x8b,0x43,0x30})!=1 ||
       count(code,size,{0x4c,0x8b,0x40,0x08})!=1 || count(code,size,{0x48,0x8d,0x4b,0x30})!=1)return false;
    if(!field(code,size,{0x41,0x8b,0x95},0x33c) ||
       !field(code,size,{0xf3,0x0f,0x10,0x80},0x90) ||
       !field(code,size,{0x41,0x83,0x87},0xca4) ||
       !field(code,size,{0x41,0x8b,0x9f},0xc98) ||
       !field(code,size,{0x41,0x8b,0xb7},0xc94))return false;
    // The unrolled and scalar loops each load these two fields.
    if(count(code,size,{0x45,0x8b,0x97,0x90,0x0c,0,0})!=2 ||
       count(code,size,{0x4d,0x8b,0x9f,0x88,0x0c,0,0})!=2 ||
       count(code,size,{0xf3,0x41,0x0f,0x10,0x87,0xa0,0x0c,0,0})!=2)return false;
    // Every descriptor write used by launch_early is checked independently.
    if(!field(code,size,{0xc6,0x82},scene+0x3b8) ||
       !field(code,size,{0xf3,0x0f,0x11,0x82},scene+0x2a8) ||
       !field(code,size,{0x48,0x8d,0x82},scene) ||
       !field(code,size,{0x48,0x89,0x82},scene+0x2b0) ||
       !field(code,size,{0x48,0x89,0x82},scene+0x2b8) ||
       !field(code,size,{0x89,0x82},scene+0x2c0) ||
       !field(code,size,{0x48,0x8d,0x8a},scene+0x2c8) ||
       !field(code,size,{0x48,0x81,0xc2},scene+0x1f0))return false;
    // Verify literal 1, not merely the location of the flag writes.
    bool flag=false;
    for(size_t i=0;i+7<=size;++i)
        if(code[i]==0xc6 && code[i+1]==0x82 && u32(code+i+2)==scene+0x3b8 && code[i+6]==1)flag=true;
    if(!flag || count(code,size,{0xb8,1,0,0,0})!=1)return false;
    result.scene=scene;return true;
}
}

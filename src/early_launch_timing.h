#pragma once
#include <cstdint>
#include <limits>

// Identity values are compared only, never dereferenced after an invocation.
// Carry only the wait moved out of the next native join, not total step time
// (which would count genuinely overlapped computation as update-thread waiting).
struct EarlyLaunchTiming {
    uintptr_t sim=0, frame=0;
    uint64_t wait_ns=0;
    static uint64_t add(uint64_t a,uint64_t b) {
        return b>std::numeric_limits<uint64_t>::max()-a?std::numeric_limits<uint64_t>::max():a+b;
    }
    void publish(uintptr_t s,uintptr_t f,uint64_t ns) {sim=s;frame=f;wait_ns=ns;}
    uint64_t take(uintptr_t s,uintptr_t f) {
        const auto value=s==sim && f==frame?wait_ns:0;
        sim=frame=0;wait_ns=0;return value;
    }
    static bool apply(uint64_t* samples,uint32_t cap,uint32_t head,uint32_t count,uint64_t paid) {
        if(!samples || !count || count>120 || count>cap || cap>1048576 || head>=cap)return false;
        auto& latest=samples[(uint64_t(head)+count-1)%cap];
        latest=add(latest,paid);return true;
    }
};

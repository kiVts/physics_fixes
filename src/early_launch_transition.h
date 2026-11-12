#pragma once
#include <cstdint>
// An early task leaves native post-physics copying to the following async join.
// A request to switch to sync is applied AFTER that join/post-copy boundary,
// never by suppressing the native synchronous branch and forgetting the copy.
struct EarlyLaunchTransition {
    uintptr_t sim=0, frame=0;
    bool deferred=false;
    void defer(uintptr_t s,uintptr_t f){sim=s;frame=f;deferred=true;}
    bool matches(uintptr_t s,uintptr_t f)const{return deferred && sim==s && frame==f;}
    void discard_other(uintptr_t s,uintptr_t f){if(!matches(s,f))*this={};}
    bool complete(uintptr_t s,uintptr_t f){if(!matches(s,f))return false;*this={};return true;}
};

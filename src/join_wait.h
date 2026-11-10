#pragma once
#include "log.h"
#include "sdk.h"
#include <windows.h>
#include <cstdint>

// A timed-out join cannot safely return: workers still own native pointers and
// caller stack data. Yield under contention and surface a prolonged wait instead
// of burning a core or pretending that the outstanding work completed.
template<class Done> void wait_for_workers(const char* phase, Done done) {
    const ULONGLONG start = GetTickCount64();
    bool reported = false;
    for (uint64_t spins = 0; !done(); ++spins) {
        if (spins < 2048) YieldProcessor();
        else if (reported) Sleep(1);
        else SwitchToThread();
        if (!reported && !(spins & 0x3fff) && GetTickCount64() - start >= 2000) {
            reported = true;
            swlog("%s: worker join exceeded 2 s; waiting to preserve native object lifetime", phase);
            swml_report_fault(swml(), swml_self());
        }
    }
}

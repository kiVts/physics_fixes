#pragma once
// The player-facing switches in settings.default.ini [multithreading]. Each one turns a whole area off on top of that
// area's own detailed settings, which are no longer in settings.default.ini but are still read (with the same
// defaults) when written into settings.overrides.ini by hand.
#include "log.h"
#include <windows.h>

// [multithreading] <area>; `fallback` when the key is missing (an install without the current settings.default.ini)
inline bool mt_area(const wchar_t* area, int fallback = 1)
{
    return GetPrivateProfileIntW(L"multithreading", area, fallback, settings_ini()) != 0;
}

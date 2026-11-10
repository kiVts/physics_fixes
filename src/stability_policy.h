#pragma once

// Audit 2026-11-10: contact callbacks can mutate the tree/particle registries and
// the scene event queue while logic runs. The contact-ring lock is insufficient;
// cylinder teardown also depends on a return address lost through hook chaining.
// Keep the native physics/logic order until those ownership paths are proven.
// This gate intentionally applies to existing early_launch=1 installations too.
inline constexpr bool kEarlyLaunchAudited = false;

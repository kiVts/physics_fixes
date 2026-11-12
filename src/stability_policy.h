#pragma once

// Audit 2026-11-10: contact callbacks can mutate the tree/particle registries and
// the scene event queue while logic runs. The ring-only guard was insufficient.
// Pass two replaces it with a scoped whole-overlap contact exclusion and explicit
// release-before-join fences. Pass three adds field validation, wait accounting,
// delayed sync toggles and a join before vehicle logic. Physics-side game-owner
// reads during zone 14 and uncertain task publication remain unresolved: retain
// native order in releases. Passing the local fixtures is not an enable gate.
// This gate intentionally applies to existing early_launch=1 installations too.
inline constexpr bool kEarlyLaunchAudited = false;

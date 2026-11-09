0.25.11: early launch: the contact callback and the logic's contact ring drain (zone 0x37) now share a lock. Together they could leave a ring with count != 0 and capacity 0: a divide-by-zero crash on the host (2026-11-08). If the drain isn't found as expected the early launch stays off. "contact rings 30s" lines.

0.25.10: held movable objects are excluded from touch ownership, host steering and pallet snapshots so native pickup has sole control. Physics Fixes includes a read-only 32-call local-character launch history (velocity, forces, facing, held item and contacts), dumped on abnormal walking speeds. This is diagnostic coverage for the reported roller launch, not a verified launch fix.

0.25.9: large vehicles retain host authority. The client relays its solved horizontal contact reaction (normal plus friction) for unowned vehicle bodies; the host applies a bounded force at the contact point. No vehicle-size ownership limit is raised. Forces expire after 150 ms, total at most 1500 N per player, and require a nearby remote character and a point within the host body AABB. [push] large_vehicles=0 disables the relay in Network Fixes.

0.25.8: compatible multiplayer pushing enables host-like 180 Hz collision substeps while the client walks. Native client physics otherwise hard-codes 60 Hz even when the host and UI use 180 Hz. Elapsed time is unchanged. Physics setting [push] client_physics_hz=0 disables refinement. Original GJK and native solver A/B tests did not prevent sinking; analytic contacts and parallel solver are restored.

0.25.7 diagnostic: traces actual pallet contact rows and vertical impulses. On a client, if a watched pallet sinks 15 cm, subsequent steps use the untouched native solver for a within-session A/B test. [touch] solver_ab=0 disables the automatic switch.

0.25.6: movable prop boxes use Bullet's native GJK contact witnesses/manifold handling. Oracle logs compare
analytic depth, normal and witness points. Support diagnostics now separate response contacts from sensor overlaps.

0.25.5: [touch] support_audit=1 logs actual contact counts, upward support, penetration depth, velocity and filters
for recently touched dynamic bodies on both PCs. Diagnostic only; no forces or contact points are changed.

0.25.3: client touch census (`[touch] census=1`): about 8 times a second the bodies your character touches, exported as
`swml_touched_bodies` for network_fixes 1.9.2's object push log. Log only.

0.25.2: host: remote players' characters skip collisions with dynamic bodies (`[puppets] host_cull=1`). The host
teleports each remote character as a driven body, an immovable wall to Bullet: it shoved pallets and small vehicles away
and into the ground the moment a client touched them, and fought the steering of objects a client pushes itself. Ground,
sensors and the host's own character still collide with it.

0.25.1: exports `swml_local_character` (this PC's own character, for network_fixes' object push).

0.22.22: far-physics diagnosis log lines removed (the funnel / candidates lines); multiplayer host line only when peers are connected.

0.22.21: "far vehicle work 5s" probe: logic work of vehicles 0.5 km+ away by why they aren't decimated, and how much of it is idle vehicles.
0.22.20: contain walk: one grid per body ring and thread, flat sorted cells (the single grid was rebuilt on nearly every search: ~1.2 ms per tick); "contain walk 5s" shows grid builds and search time.
0.22.19: solver: own island walk (counting sort of the manifolds by island instead of the game's quicksort; checked against the game's walk every 64 sub-steps; [solver] own_islands=1, "own island walk" line).
0.22.18: integrateTransforms on the physics pool when no body can take Bullet's CCD path ([lod] parallel_integrate=1, "integrate 5s" lines).
0.22.17: far physics funnel: groups that passed the hold rule, pair cache unreadable, all dropped for overlaps, nothing to hold (0.22.16: ~10 crewed groups per tick passed the rule, nothing refused, nothing held).
0.22.16: far physics diagnosis: "far candidates 5s" (vehicle_mt: why decimated vehicles are not published) and a "far physics funnel" line (where a tick with candidates stops); broadphase: prefetch proxies ahead of the ordered setAabb calls.
0.22.15: hull rays: follow the update thread's id every loop iteration (0.22.14's test: the first loop event came from
another thread and no cast was deferred all session).

0.22.13: [lod] parallel_predict: Bullet's predictUnconstraintMotion (damping + predicted transform per body, ~0.19 ms per
tick serial) runs on the physics pool; per-body work only, same result. Needs far physics on ([lod] physics), which hooks it.

0.22.12: sphere cache off by default ([contain] sphere_cache=1 turns it on): correct, but no gain measured (the containment
test's cost is its call count on bodies already in the CPU cache).

0.22.11: sphere cache: the self-check read the containment test's whole rax as its answer; the test returns a bool in al
(the miss path leaves rax's upper bits set). Every 'cache off' so far was that, not a wrong rejection.

0.22.10: sphere cache self-check: a hit only counts against the cache on a body that holds still over the check (sphere,
the game's test, sphere again); 0.22.9 still switched off on a body moved 1.4 km right after a world load.

0.22.9: sphere cache: a self-check hit is judged against the body as it is now (a body placed / moved by another thread
between the decision and the check, 19.3 km right after a world load twice, no longer switches the cache off).

0.22.7: sphere cache: switched itself off in the first test (a body 19 km from its cached sphere within one tick).
Each use now checks the body's translation and re-reads the sphere when it changed; rotation is covered by adding twice the
sphere's local offset to the limit (margin default 1 m). Contact skip measured: 0.70 -> 0.34 ms per tick.

0.22.5: fixes 0.22.4 failing to start on the 2026-10-28 build (radar msgs read a jump-table entry whose address was computed before
translation; the whole mod stopped, logic fell to ~10 fps). Vehicle logic's shared-system wrappers on that build: the
original lists map through the port table, plus vehicle_sinks_ported.inc (tools/port_sinks.py, 19 functions in the new build's
addresses: event constructors that were rebuilt, the new events such as ocean wave params, the weapon tick and its writer).
The SDK now runs each part's install guarded: an unmapped address or a crash switches that part off, not the mod.

0.22.1: far physics after its first run (2026-10-28 23:57): nothing was held, every crewed AI ship group was refused as
"joined to something else" (an AI ship is several vehicles joined by physics joints). Uncrewed vehicles now count as
joined parts when they are in the distant-AI-logic set, untainted, and have been joined to a crewed one every time they
were seen loaded (after a 2 s grace); seen alone once, never again this session (by vehicle id). A new line names what
refused groups are joined to. Self-checks stayed at 0 in that run.

0.20.3: `[lod] logic=3` A/B mode to measure the gain in one session: distant AI logic on for 30 s, off for 30 s, and a
`distant AI logic A/B since start` line with running totals per state: ticks/s (tick period), the vehicle-logic loop's
wall time and its thread time per tick, vehicles skipped. Copies are dropped while off.

0.20.2: distant AI logic, after the second compare run (22:44, run_logs/20261027_solo_0201_t2_groups): AI groups worked
(28 crewed + ~60 group vehicles, up to 25 skipped per tick; far ring ~18% less work per vehicle), but the same ships were
~15% slower while decimated (3.9 vs 4.6 m/s over the session), and decimated units also posted event_set_impulse_threshold
(~16%), event_set_transform and event_set_constraint_hinge_set_frames (~7% each) that were not repeated. Now both
impulse kinds are copied and replayed (0x30 / 0x38 bytes, variable-size stash); motor power, collision flags and damping
hold by themselves; any other event type, or a force on another vehicle's body, keeps that vehicle at full logic for 10 s
(`kept at full logic` line, with the reasons).

0.20.1: distant AI logic, after the first compare run (2026-10-27 22:38, run_logs/20261027_solo_0200_t2_compare): 0 faults,
0 dropped copies, ~140 impulses replayed per tick, but only 23 of 90 vehicles were AI-crewed (~5 skipped per tick of 67
at 1-2 km), and the decimated half averaged 6.5 m/s against 11 m/s for the control half (different ships, so not
conclusive). Now: an uncrewed vehicle without a player seat within 80 m of an AI-crewed one counts as part of its AI
group; compare mode alternates each unit between decimated and full every 30 s, so both speeds come from the same ships;
a new line lists the physics event types decimated units post on their run ticks (to see whether thrust also uses
events other than event_set_impulse).

0.18.2: two more for the AI-crew world. `[broadphase] parallel_aabbs=1`: the ~4100 objects' boxes in updateAabbs (0.42 ms
per physics sub-step, all on the physics thread) are computed on the physics pool and handed to the broadphase in the
game's order, the same calls with the same values; every 64th sub-step they are computed again on the physics thread
and must match bit for bit, else that part goes back to the game's loop. `[hull_tasks] keep_ring=1`: a body's hull
force list keeps its memory between ticks instead of being freed by the update thread (~1.3% of it in RtlFreeHeap) and
grown again by the task every tick. The 2026-10-27 18:23 run with 8 pool threads instead of 6 was no faster: each
thread got slower and the vehicle logic's lock waits doubled (2.6 -> 5.0 ms per tick of thread time).

0.17.2: `parallel 5s:` gets a second line splitting a parallel narrowphase dispatch: sorting out soft-body pairs (serial),
the pool run, the time each thread actually spends on pairs, the busiest thread.

0.15.0: catch-up physics skip (`[catchup]`, joining players only). When your game is far behind the host (more
than 60 ticks queued, e.g. right after joining), it replays the queued ticks with the physics step skipped for 2 of
every 3: everything except your own character follows the host's positions anyway, and events and vehicle logic still
run for every tick, so it catches up faster. Only while your character is seated or not spawned yet (a walking
character on a moving deck could be left behind). `catch-up physics 10s:` in physics_fixes.log. `skip_physics=0` turns it off.

0.14.1: a puppet's sensor ghosts now hold no contacts instead of their last ones. Before, a ghost whose contact list
was ever filled (any step the cull couldn't run) kept it for good, including pointers to the objects it touched, which
could be gone later. With the cull on from the start the list was empty anyway, so nothing visible changes.

0.13.7: advanced wheels keep 0.9999 of their spin per tick instead of the game's 0.99 (`[wheels]`). The game's loss
acted as a drag proportional to speed (45 kN at 31 m/s on a 10 t aircraft); it is now 1 % of that. Braking is
unchanged. Moved here from Realistic Aero 0.5.1 (aircraft only there), now for every vehicle.

# Physics Fixes (prototype)

0.13.0 adds an optional vehicle begin/end observer for Realistic Aero's VLM batching. With no observer,
the usual minimum-vehicle fallback is unchanged. With an observer, small scenes use the serial vehicle
loop so even one aircraft gets a complete batch; additional workers are not woken for those scenes.

0.12.1: fixes the shared inventory-mesh reference-count race exposed by parallel render preparation.
The complete mesh assign/release operation is serialized across callers. Parallel prep stays off if
that hook cannot be installed. Offline concurrency checks pass; in-game crash validation is pending.

- `measure=1` (default): every 5 s, `physics_fixes.log` gets a `narrowphase 5s:` block with the time Bullet
  spends per pair class, e.g. `G:capsule-D:compound` = character sensor ghost vs vehicle body.
- `skip_ghost_pairs=1`: the near callback returns early for pairs that involve a ghost object.
  Ghosts are `CF_NO_CONTACT_RESPONSE`, so the solver never used those contacts; the ghost's own
  overlap list is maintained by the broadphase and is unaffected.

Check with `skip_ghost_pairs=1` before turning it on for good: sitting in / leaving seats, ladders,
interact prompts near vehicles, swimming and entering water. Fixed address: build 0x6a9a9e20 only.
`multiplayer=local`: it doesn't change the simulation, each PC can run it or not.

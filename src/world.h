#pragma once
#include <cstdint>
#include <vector>

// Shared physics-world knowledge for the physics_fixes parts (physics thread only).

int body_group(const void* collision_object);
bool body_groups_valid(); // false if the constraint layout didn't check out
// Lets this thread call body_group() while the physics thread is inside a step (solver workers).
void world_mark_reader_thread();

enum GhostOwner { kGhostNotCharacter = 0, kGhostCharacterWalking = 1, kGhostCharacterSeated = 2 };
GhostOwner ghost_owner(const void* ghost_collision_object);

// Rebuilds the vehicle groups; census.cpp calls it at the start of every stepSimulation.
void world_step_begin(const void* world);

// Bullet broadphase switches ([broadphase] in settings.ini); census.cpp calls it at every stepSimulation.
void broadphase_step_begin(void* world);

// Closed-form box vs capsule / sphere contacts (box_contact.cpp, [narrowphase] box_contacts); census.cpp calls it at
// every stepSimulation.
void box_contacts_step_begin(const void* world);

// A stepSimulation is running right now (census.cpp's step hook; any scene). serialize.cpp waits for it to end
// before writing vehicles on the physics pool: the game's step N runs until tick N+1's zone 0x11, past the host's
// request zone.
bool physics_step_running();

// Increments at every stepSimulation (one game tick); the physics thread that owns the groups.
uint64_t world_step_id();
unsigned long world_physics_thread();

// Client puppet cull (puppet_cull.cpp): census.cpp calls puppet_step_begin at every stepSimulation; narrowphase.cpp asks
// about each pair (any thread during the step), heat_cache.cpp about each ghost contact update.
void puppet_step_begin(const void* world);
void puppet_step_end(); // after the step: each character's capsule origin, for the next step's "moved by the game" test
bool puppet_skip_pair(const void* a, const void* b);
void puppet_note_cleared();
bool puppet_skip_ghost(const void* scene_ghost);
bool puppet_ghosts_on();
// The local (non-puppet) characters seen by the last scan: -1 unknown (cull off, host, scan failed), 0 none,
// 1 all seated, 2 at least one not seated.
int puppet_local_state();

// The multiplayer work inside the physics step, in network_fixes (network_hooks.cpp here calls it).
void touch_step_begin(const void* world);
double network_fixed_step(double elapsed, int max_substeps, double native_fixed);
void network_step_end(const void* world, double fixed, int steps);

// Client catch-up physics skip (catchup_skip.cpp): census.cpp asks before every stepSimulation; true = don't step.
bool catchup_skip_step(const void* world);

// World sanity watch (sanity.cpp); census.cpp calls it after every stepSimulation.
void sanity_after_step(const void* world);

// This PC's own player character (local_player.cpp), refreshed every tick on the update thread. False while there is
// none (menu, not spawned) or it wasn't seen for 2 s. character (optional) gets the character object. Any thread.
bool local_player_pos(double out[3], const void** character = nullptr);
// The host tick: the host's own tick on the host, the tick being replayed on a client (local_player.cpp). Same number on
// every PC for the same simulated tick. False until the game's tick object has been seen.
bool host_tick(uint32_t& tick);
// Rings of the "by distance" log lines: 0 under 0.5 km, 1 0.5-1 km, 2 1-2 km, 3 2 km and more.
constexpr int kDistRings = 4;
extern const char* const kDistRingNames[kDistRings];
int distance_ring(const double pos[3], const double player[3]);

// Every player (lod.cpp): this PC's character and every remote player's, on host and clients alike. Rebuilt at each
// stepSimulation (census.cpp calls lod_step_begin before the step). Any thread.
constexpr int kMaxPlayers = 64;
int players_snapshot(double out[][3], int max, int* remote = nullptr); // count; -1 = none known
int player_ring(const double pos[3]); // ring of the nearest player, -1 = no player known
void lod_step_begin(const void* world);
// Narrowphase (any thread during the step): a pair with a far seated crew body ([lod] crew=1).
bool lod_skip_pair(const void* a, const void* b);
void lod_note_cleared();
void lod_seats(std::vector<const void*>& crew, std::vector<const void*>& players);
// Physics thread, after lod_step_begin: the Bullet bodies of seated non-player characters and of player characters
// (yours and remote players'), sorted.
void lod_parts(std::vector<const void*>& crew, std::vector<const void*>& players);

namespace far_logic { struct Cand; }
void far_physics_candidates(std::vector<far_logic::Cand>& cands); // update thread: vehicle bodies allowed this tick
void far_physics_step_begin(void* world, double time_step, int max_sub, double fixed); // census.cpp, before the step
void far_physics_step_end(void* world, int steps, uint64_t cycles);                    // census.cpp, after it
int far_physics_pair(const void* a, const void* b); // narrowphase (any thread): 1 = skip the pair
int far_physics_island(void* const* bodies, int n, bool (*is_dynamic)(const void*)); // solver: 0 none, 1 held, 2 mixed
bool far_physics_hold_substep();   // a sub-step on which held units are skipped
int far_physics_long_scale();      // n on the sub-step that steps held units with n x dt, else 0
void far_physics_note_units(int skipped, int long_units);
void far_physics_solver_fallback(); // the solver took the game's own path in this sub-step
bool far_physics_on();
bool solver_units_mode();           // solver_log.cpp: islands are solved as this mod's units (parallel or serial)

// Temporary pallet contact-to-solver diagnostic (network_fixes).
void touch_solver_begin(const void* world);
bool touch_solver_native();
void touch_solver_setup(void* solver, void** bodies, int nb, void** manifolds, int nm);
void touch_solver_finish(void* solver);
void touch_solver_end();

void character_launch_begin(const void* world,double elapsed,double fixed);
void character_launch_end(const void* world);

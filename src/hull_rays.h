#pragma once
// Hull water rays on a pool (hull_rays.cpp). The two places that read the rays' results call the join first.

// where a join runs (the "hull rays 5s" line splits the waiting by it, and by update thread / pool thread)
enum class HullJoin : int { loop, task, tick, full, count };
bool hull_rays_enabled();         // settings: [hull_rays] defer and [multithreading] physics (the casts go on the pool)
void hull_rays_join(HullJoin at); // every queued ray cast done (the caller helps); cheap when nothing is queued
void hull_rays_loop_join_ready(); // vehicle_mt: its loop stub calls hull_rays_join() at the vehicle loop's start
void hull_rays_task_join_ready(); // hull_tasks: its pool calls hull_rays_join() before every hull task

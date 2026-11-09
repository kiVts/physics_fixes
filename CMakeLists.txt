cmake_minimum_required(VERSION 3.24)
project(physics_fixes LANGUAGES C CXX)

# Needs the SWML SDK repo: cmake -B build -A x64 -DSWML_SDK_DIR=path/to/swml_sdk -DSWML_GAME_DATA_DIR=path/to/generated/headers
set(SWML_SDK_DIR "" CACHE PATH "SWML SDK folder (the one with SwmlMod.cmake)")
if(NOT EXISTS "${SWML_SDK_DIR}/SwmlMod.cmake")
  message(FATAL_ERROR "Set SWML_SDK_DIR to the SWML SDK folder")
endif()
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
add_subdirectory("${SWML_SDK_DIR}" swml_sdk)

include(FetchContent)
set(LIBDEFLATE_BUILD_SHARED_LIB OFF CACHE BOOL "" FORCE)
set(LIBDEFLATE_BUILD_GZIP OFF CACHE BOOL "" FORCE)
set(LIBDEFLATE_BUILD_TESTS OFF CACHE BOOL "" FORCE)
FetchContent_Declare(libdeflate GIT_REPOSITORY https://github.com/ebiggers/libdeflate.git GIT_TAG v1.24)
FetchContent_MakeAvailable(libdeflate)

swml_add_mod(physics_fixes SOURCES
  src/network_hooks.cpp
  src/catchup_skip.cpp
  src/body_contain.cpp
  src/hull_tasks.cpp
  src/particles_mt.cpp
  src/tri_empty.cpp
  src/sanity.cpp
  src/render_guard.cpp
  src/bullet_heap.cpp
  src/narrowphase.cpp
  src/profiler.cpp
  src/census.cpp
  src/broadphase.cpp
  src/solver_log.cpp
  src/sound_mix.cpp
  src/radar_msg.cpp
  src/broadphase_mt.cpp
  src/world.cpp
  src/compound_cache.cpp
  src/mt_dispatch.cpp
  src/render_scratch.cpp
  src/vehicle_mt.cpp
  src/render_bodies.cpp
  src/render_mt.cpp
  src/serialize.cpp
  src/render_refs.cpp
  src/rain_ray.cpp
  src/phys_pool.cpp
  src/tri_cull.cpp
  src/heat_cache.cpp
  src/puppet_cull.cpp
  src/local_player.cpp
  src/lod.cpp
  src/far_physics.cpp
  src/cut_probes.cpp
  src/setup.cpp
  src/sphere_cache.cpp
  src/box_contact.cpp
  src/hull_count.cpp
  src/contact_skip.cpp
  src/hull_rays.cpp
  src/contain_walk.cpp
  src/logic_counters.cpp
  src/overlap_physics.cpp
  src/overlap_query.cpp
  src/early_launch.cpp
  src/parallel_guard.cpp
  src/affinity.cpp
  src/box_aabb.cpp
  src/pose_cache.cpp
  src/event_guard.cpp
  src/ai_rays.cpp
)
target_include_directories(physics_fixes PRIVATE src)
target_link_libraries(physics_fixes PRIVATE libdeflate_static)

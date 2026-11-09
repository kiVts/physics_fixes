# Stormworks Physics Performance Mod

Multi-threads Stormworks' physics and game logic, and speeds up other slow parts of the game. This cuts tick time enormously in heavy worlds, easily reaching 4 times the speed of vanilla.

Vanilla does nearly all of its heavy work on about 4 threads: update, physics, render and one worker. In a busy world the update thread becomes the limit, while most of your CPU sits idle. This mod spreads a single tick across about 27 worker threads, with up to 6 for each major part:

- **Physics:** broadphase, collision checks and the constraint solver
- **Vehicle logic:** each vehicle's components and microcontrollers
- **Render preparation:** building the frame's draw data
- **Hull and particles:** air and water forces, water rays and particle emitters

It also lets physics run alongside game logic instead of waiting its turn, and replaces some of the game's most expensive loops with faster versions, including the sound mixer, radar messages and vehicle saving.

This will likely trigger your antivirus, obviously there is no viruses in here. 

To use, install [SWML](https://github.com/kiVts/swml) and add the zip from releases here. I also recommend you also install [No Crash Uploads](https://github.com/kiVts/no_crashlog_upload) so you don't upload a crash-report with an injected DLL to Geometa. consider other mods as well!

[Rotation Keybinds](https://github.com/kiVts/editor_rotation_rings)
[Composite Tracer](https://github.com/kiVts/composite_tracer)

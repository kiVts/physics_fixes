# Stormworks Physics Performance Mod

Multi-threads Stormworks' physics and game logic, and speeds up other slow parts of the game. This cuts tick time enormously in heavy worlds, easily reaching 4 times the speed of vanilla.

Vanilla does nearly all of its heavy work on about 4 threads: update, physics, render and one worker. In a busy world the update thread becomes the limit, while most of your CPU sits idle. This mod spreads a single tick across about 27 worker threads, with up to 6 for each major part:

- **Physics:** broadphase, collision checks and the constraint solver
- **Vehicle logic:** each vehicle's components and microcontrollers
- **Render preparation:** building the frame's draw data
- **Hull and particles:** air and water forces, water rays and particle emitters

This mod also makes physics and logic run concurrently, but was disabled after a few checks and tests proved it to be unstable. If you wish to enable do so at your own risk. It might get enabled later.

The stability update fixes hull-ray batch reuse and concurrent body-group writes, synchronizes pool setup, and reports prolonged worker joins through SWML. Worker jobs must still finish before their objects are released. Regression tests pass; live multiplayer soak testing remains pending.

Since SWML 4.5 effort been put to change the way mods are loaded, and should no longer trigger anti-virus by simply existing. 
This might trigger your antivirus, obviously there is no viruses in here. If a game crashes feel free to make an issue here and [join the discord](https://discord.gg/SsT4Y6Y3Zm) so the process to fixing them is quicker.

To use, install [SWML](https://github.com/kiVts/swml) and add the zip from releases here. I also recommend you install [No Crash Uploads](https://github.com/kiVts/no_crashlog_upload) so you don't upload a crash-report with an injected DLL to Geometa ( And a crash reason overwhelmingly likely to be due to one of these mods). consider other mods as well!

[Rotation Keybinds](https://github.com/kiVts/editor_rotation_rings)
[Composite Tracer](https://github.com/kiVts/composite_tracer)

Like what I do?

<a href="https://buymeacoffee.com/kivts">
  <img src="https://cdn.buymeacoffee.com/buttons/v2/default-yellow.png" width="120" alt="Buy Me a Coffee">
</a>

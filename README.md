# Stormworks Physics Performance Mod

Multi-threads Stormworks' physics and game logic, and speeds up other slow parts of the game. This cuts tick time enormously in heavy worlds, easily reaching 4 times the speed of vanilla.

Vanilla does nearly all of its heavy work on about 4 threads: update, physics, render and one worker. In a busy world the update thread becomes the limit, while most of your CPU sits idle. This mod spreads a single tick across about 27 worker threads, with up to 6 for each major part:

- **Physics:** broadphase, collision checks and the constraint solver
- **Vehicle logic:** each vehicle's components and microcontrollers
- **Render preparation:** building the frame's draw data
- **Hull and particles:** air and water forces, water rays and particle emitters

Early physics launch is disabled after a stability audit, including when existing settings enable it. Physics and game logic retain their normal ordering; the other parallel phases remain available. This can reduce throughput in heavy worlds. The mod also speeds up the sound mixer, radar messages and vehicle saving.

The stability update fixes hull-ray batch reuse and concurrent body-group writes, synchronizes pool setup, and reports prolonged worker joins through SWML. Worker jobs must still finish before their objects are released. Regression tests pass; live multiplayer soak testing remains pending.

This will likely trigger your antivirus, obviously there is no viruses in here. If a game crashes feel free to make an issue here and [join the discord](https://discord.gg/SsT4Y6Y3Zm) so the process to fixing them is quicker.

To use, install [SWML](https://github.com/kiVts/swml) and add the zip from releases here. I also recommend you also install [No Crash Uploads](https://github.com/kiVts/no_crashlog_upload) so you don't upload a crash-report with an injected DLL to Geometa ( And a crash reason overwhelmingly likely to be due to one of these mods). consider other mods as well!

[Rotation Keybinds](https://github.com/kiVts/editor_rotation_rings)
[Composite Tracer](https://github.com/kiVts/composite_tracer)

Like what I do?

<a href="https://buymeacoffee.com/kivts">
  <img src="https://cdn.buymeacoffee.com/buttons/v2/default-yellow.png" width="120" alt="Buy Me a Coffee">
</a>

## Control-loop diagnostics

For train ECU/logic oscillation, this version supplies read-only scheduling and
component-update observations to Network Fixes' opt-in control-loop recorder.
Configure `[logic_trace] enabled=1` in Network Fixes, on host and observers, then
restart. Use the settings that exhibit the issue; this adapter does not change
the selected logic mode. Collect `logs/logic_trace_*.jsonl` together with both
mods' logs. This adds diagnostics; the reported jitter is not yet claimed fixed.

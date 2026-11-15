# Stormworks Physics Performance Mod

[![Join Discord](https://swml.kivts.net/brand/badges/dark/discord.svg)](https://discord.gg/bX4RzunsxC)
[![Requires SWML](https://swml.kivts.net/brand/badges/dark/requires-swml.svg)](https://github.com/kiVts/swml/releases/latest)
[![SWML website](https://swml.kivts.net/brand/badges/dark/website.svg)](https://swml.kivts.net/)
[![GitHub releases](https://swml.kivts.net/brand/badges/dark/github.svg)](https://github.com/kiVts/physics_fixes/releases)
[![Support on Buy Me a Coffee](https://swml.kivts.net/brand/badges/dark/coffee.svg)](https://buymeacoffee.com/kivts)

Multi-threads Stormworks' physics and game logic, and speeds up other slow parts of the game. This cuts tick time enormously in heavy worlds, easily reaching 4 times the speed of vanilla.

Vanilla does nearly all of its heavy work on about 4 threads: update, physics, render and one worker. In a busy world the update thread becomes the limit, while most of your CPU sits idle. This mod spreads a single tick across about 27 worker threads, with up to 6 for each major part:

- **Physics:** broadphase, collision checks and the constraint solver
- **Vehicle logic:** each vehicle's components and microcontrollers
- **Render preparation:** building the frame's draw data
- **Hull and particles:** air and water forces, water rays and particle emitters

This mod also makes physics and logic run concurrently, but was disabled after a few checks and tests proved it to be unstable. If you wish to enable do so at your own risk. It might get enabled later.

The stability update fixes hull-ray batch reuse and concurrent body-group writes, synchronizes pool setup, and reports prolonged worker joins through SWML. Worker jobs must still finish before their objects are released. Regression tests pass; live multiplayer soak testing remains pending.

Since SWML 4.5 effort been put to change the way mods are loaded, and should no longer trigger anti-virus by simply existing
This still might trigger your defender. Obviously there is no viruses in here. If a game crashes feel free to make an issue here and [join the discord](https://discord.gg/SsT4Y6Y3Zm) so the process to fixing them is quicker.

This mod is made in compliance of Geometa EULA. Under Section 3 Paragraph 4, `You must not modify, merge, distribute, translate, reverse engineer, decompile, or try to obtain or use source code for the Geometa Services, except for mods as per section 5 or where permitted by applicable law.` In section 5, `You may create and share mods and other content for our games, provided they comply with this Agreement. You own the content you create, including the content created in game, mods created with Geometa tool kits, community websites, etc.`

To use, install [SWML](https://github.com/kiVts/swml) and add the zip from releases here. I also recommend you install [No Crash Uploads](https://github.com/kiVts/no_crashlog_upload) so you don't upload a crash-report with an injected DLL to Geometa ( And a crash reason overwhelmingly likely to be due to one of these mods). consider other mods as well!

[Rotation Keybinds](https://github.com/kiVts/editor_rotation_rings)
[Composite Tracer](https://github.com/kiVts/composite_tracer)

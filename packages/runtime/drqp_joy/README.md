# drqp_joy

`drqp_joy` consumes SDL3 through the workspace `sdl3_vendor` package instead of
embedding the third-party build in its own CMake logic. Ubuntu 24.04 in this
workspace does not provide a usable `libsdl3-dev` package for the target
environment, so `sdl3_vendor` can build `libsdl-org/SDL` at `release-3.4.4`
and expose `SDL3::SDL3` to downstream packages.

The vendored SDL build is configured with `SDL_UNIX_CONSOLE_BUILD=ON` and with
desktop windowing subsystems disabled. `drqp_joy` only needs SDL's gamepad and
haptic support, so this avoids hard dependencies on X11 and Wayland development
packages that are not required for the node's runtime behavior.

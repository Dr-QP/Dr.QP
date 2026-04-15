# drqp_joy

`drqp_joy` builds SDL3 from source via CMake `FetchContent` instead of relying on
an Ubuntu package. Ubuntu 24.04 in this workspace does not provide a usable
`libsdl3-dev` package for the target environment, so the package fetches
`libsdl-org/SDL` at `release-3.4.4` and links against `SDL3::SDL3` directly.

The SDL build is configured with `SDL_UNIX_CONSOLE_BUILD=ON` and with desktop
windowing subsystems disabled. `drqp_joy` only needs SDL's gamepad and haptic
support, so this avoids hard dependencies on X11 and Wayland development
packages that are not required for the node's runtime behavior.

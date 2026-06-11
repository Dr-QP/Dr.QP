# sdl3_vendor

Vendor package for providing SDL3 to the workspace.

This package follows the ROS joystick driver `sdl2_vendor` layout: it first
tries to locate an existing SDL3 installation through SDL3's native CMake
package, and if that is unavailable it builds SDL3 from source and installs it
into the workspace.

The vendored build keeps the console-only configuration used by `drqp_joy` and
disables desktop-oriented SDL subsystems that are not needed for gamepad and
haptic support on the target environment.

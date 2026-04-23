# DualSense audio and haptics over Bluetooth

Presently there is only single closed source solution that is claiming to have full support for audio and haptics over Bluetooth for DualSense controllers https://store.steampowered.com/app/1812620/DSX/

Other tools have not attempted building this support still.

DualShock 4 userland driver has a 10 years old open PR claiming to have added audio support https://github.com/chrippa/ds4drv/pull/80/changes#diff-a9c1342d4d6304e9c286f9ffd9d795494ba1aa7dd01b3266db8e5aa23dbed17b

[SAxense](https://github.com/egormanga/SAxense) is POC for audio-haptics, but it raw and heavily relies on external tools for processing. See [SAxense feasibility](./saxense-feasibility.md) for more details.

## Resources

Here are a couple of useful links that have more information for the possible build paths:

 1. DualSense Audio Support (closed issue) https://github.com/bluez/bluez/issues/892
 2.

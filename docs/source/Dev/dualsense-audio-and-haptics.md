# DualSense audio and haptics over Bluetooth

Presently there is only single closed source solution that is claiming to have full support for audio and haptics over Bluetooth for DualSense controllers <https://store.steampowered.com/app/1812620/DSX/>

Other tools have not attempted building this support still.

DualShock 4 userland driver has a 10 years old open PR claiming to have added audio support <https://github.com/chrippa/ds4drv/pull/80/changes#diff-a9c1342d4d6304e9c286f9ffd9d795494ba1aa7dd01b3266db8e5aa23dbed17b>

Another DualShock 4 audio playback repo, just someone's hack to stream sbc audio https://github.com/sensepost/dual-pod-shock/blob/master/dual_pod_shock_POC.c (SBC (Low-Complexity Subband Codec) is the mandatory, default Bluetooth audio codec for Advanced Audio Distribution Profile (A2DP), ensuring universal compatibility across all Bluetooth devices. It offers decent, reliable sound quality for everyday use—like podcasts or calls—but features lower fidelity, higher latency (~200-300ms), and less detail compared to premium codecs like aptX or LDAC).
`gst-launch-1.0 -q filesrc location=audiofilename.mp3 ! decodebin ! audioconvert ! audiosample ! sbcenc ! "audio/x-sbc,rate=32000,channels=2,channel-mode=dual,blocks=16,subbands=8,allocation-method=snr,bitpool=25" ! queque ! filesink location=audiofilename.sbc sync=false` command that was used to convert audio to SBC

[SAxense](https://github.com/egormanga/SAxense) is POC for audio-haptics, but it raw and heavily relies on external tools for processing. See [SAxense feasibility](./saxense-feasibility.md) for more details.

## Resources

Here are a couple of useful links that have more information for the possible build paths:

 1. DualSense Audio Support (closed issue) <https://github.com/bluez/bluez/issues/892>
 2. LGPL DualSense-ts "Fully featured DualSense support via WebHID in the browser or node-hid in Node.js."  <https://nsfm.github.io/dualsense-ts>
    - audio <https://nsfm.github.io/dualsense-ts/outputs/audio>
    - HID reports documented <https://nsfm.github.io/dualsense-ts/hid-reports>
 3. CC-BY-SA Controllers Fandom data grabs and reverse engineering of structures [Sony DualSense](https://controllers.fandom.com/wiki/Sony_DualSense)

## HID Reports and descriptions

From https://controllers.fandom.com/wiki/Sony_DualSense#Bluetooth

The following HID report and descriptor data provide some hints to figuring out the format for audio output.
[SAxense](https://github.com/egormanga/SAxense) uses 0x32 for haptics. Audio Data is 3KHz 2 channel SignedInt8, 64 samples per report

| ReportID [Hex, Dec] | Size | Type | Note |
| ------- | --- | --- | ---- |
| 0x32, 50 | 141 | Output | Set Controller State and/or Audio (unconfirmed) - Used by SAxense |
| 0x33, 51 | 205 | Output | Set Controller State and/or Audio (unconfirmed) |
| 0x34, 52 | 269 | Output | Set Controller State and/or Audio (unconfirmed) |
| 0x35, 53 | 333 | Output | Set Controller State and/or Audio (unconfirmed) |
| 0x36, 54 | 397 | Output | Set Controller State and/or Audio (unconfirmed) |
| 0x37, 55 | 461 | Output | Set Controller State and/or Audio (unconfirmed) |
| 0x38, 56 | 525 | Output | Set Controller State and/or Audio (unconfirmed) |
| 0x39, 57 | 546 | Output | Set Controller State and/or Audio (unconfirmed) |

```text
0x85, 0x32,        //   Report ID (50)
0x09, 0x32,        //   Usage (0x32)
0x95, 0x8D,        //   Report Count (141)
0x91, 0x02,        //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)


0x85, 0x33,        //   Report ID (51)
0x09, 0x33,        //   Usage (0x33)
0x95, 0xCD,        //   Report Count (205)
0x91, 0x02,        //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)


0x85, 0x34,        //   Report ID (52)
0x09, 0x34,        //   Usage (0x34)
0x96, 0x0D, 0x01,  //   Report Count (269)
0x91, 0x02,        //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)


0x85, 0x35,        //   Report ID (53)
0x09, 0x35,        //   Usage (0x35)
0x96, 0x4D, 0x01,  //   Report Count (333)
0x91, 0x02,        //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)


0x85, 0x36,        //   Report ID (54)
0x09, 0x36,        //   Usage (0x36)
0x96, 0x8D, 0x01,  //   Report Count (397)
0x91, 0x02,        //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)


0x85, 0x37,        //   Report ID (55)
0x09, 0x37,        //   Usage (0x37)
0x96, 0xCD, 0x01,  //   Report Count (461)
0x91, 0x02,        //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)


0x85, 0x38,        //   Report ID (56)
0x09, 0x38,        //   Usage (0x38)
0x96, 0x0D, 0x02,  //   Report Count (525)
0x91, 0x02,        //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)


0x85, 0x39,        //   Report ID (57)
0x09, 0x39,        //   Usage (0x39)
0x96, 0x22, 0x02,  //   Report Count (546)
0x91, 0x02,        //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
0x06, 0x80, 0xFF,  //   Usage Page (Vendor Defined 0xFF80)
```

from https://github.com/raspberrypi/linux/tree/rpi-5.15.y/arch/arm/boot/dts/overlays


Name:   sc16is752-i2c
Info:   Overlay for the NXP SC16IS752 dual UART with I2C Interface
        Enables the chip on I2C1 at 0x48 (or the "addr" parameter value). To
        select another address, please refer to table 10 in reference manual.
Load:   dtoverlay=sc16is752-i2c,<param>=<val>
Params: int_pin                 GPIO used for IRQ (default 24)
        addr                    Address (default 0x48)
        xtal                    On-board crystal frequency (default 14745600)


Name:   sc16is752-spi0
Info:   Overlay for the NXP SC16IS752 Dual UART with SPI Interface
        Enables the chip on SPI0.
Load:   dtoverlay=sc16is752-spi0,<param>=<val>
Params: int_pin                 GPIO used for IRQ (default 24)
        xtal                    On-board crystal frequency (default 14745600)


Name:   sc16is752-spi1
Info:   Overlay for the NXP SC16IS752 Dual UART with SPI Interface
        Enables the chip on SPI1.
        N.B.: spi1 is only accessible on devices with a 40pin header, eg:
              A+, B+, Zero and PI2 B; as well as the Compute Module.

Load:   dtoverlay=sc16is752-spi1,<param>=<val>
Params: int_pin                 GPIO used for IRQ (default 24)
        xtal                    On-board crystal frequency (default 14745600)





README
Introduction
============

This directory contains Device Tree overlays. Device Tree makes it possible
to support many hardware configurations with a single kernel and without the
need to explicitly load or blacklist kernel modules. Note that this isn't a
"pure" Device Tree configuration (c.f. MACH_BCM2835) - some on-board devices
are still configured by the board support code, but the intention is to
eventually reach that goal.

On Raspberry Pi, Device Tree usage is controlled from /boot/config.txt. By
default, the Raspberry Pi kernel boots with device tree enabled. You can
completely disable DT usage (for now) by adding:

    device_tree=

to your config.txt, which should cause your Pi to revert to the old way of
doing things after a reboot.

In /boot you will find a .dtb for each base platform. This describes the
hardware that is part of the Raspberry Pi board. The loader (start.elf and its
siblings) selects the .dtb file appropriate for the platform by name, and reads
it into memory. At this point, all of the optional interfaces (i2c, i2s, spi)
are disabled, but they can be enabled using Device Tree parameters:

    dtparam=i2c=on,i2s=on,spi=on

However, this shouldn't be necessary in many use cases because loading an
overlay that requires one of those interfaces will cause it to be enabled
automatically, and it is advisable to only enable interfaces if they are
needed.

Configuring additional, optional hardware is done using Device Tree overlays
(see below).

GPIO numbering uses the hardware pin numbering scheme (aka BCM scheme) and
not the physical pin numbers.

raspi-config
============

The Advanced Options section of the raspi-config utility can enable and disable
Device Tree use, as well as toggling the I2C and SPI interfaces. Note that it
is possible to both enable an interface and blacklist the driver, if for some
reason you should want to defer the loading.

Modules
=======

As well as describing the hardware, Device Tree also gives enough information
to allow suitable driver modules to be located and loaded, with the corollary
that unneeded modules are not loaded. As a result it should be possible to
remove lines from /etc/modules, and /etc/modprobe.d/raspi-blacklist.conf can
have its contents deleted (or commented out).

Using Overlays
==============

Overlays are loaded using the "dtoverlay" config.txt setting. As an example,
consider I2C Real Time Clock drivers. In the pre-DT world these would be loaded
by writing a magic string comprising a device identifier and an I2C address to
a special file in /sys/class/i2c-adapter, having first loaded the driver for
the I2C interface and the RTC device - something like this:

    modprobe i2c-bcm2835
    modprobe rtc-ds1307
    echo ds1307 0x68 > /sys/class/i2c-adapter/i2c-1/new_device

With DT enabled, this becomes a line in config.txt:

    dtoverlay=i2c-rtc,ds1307

This causes the file /boot/overlays/i2c-rtc.dtbo to be loaded and a "node"
describing the DS1307 I2C device to be added to the Device Tree for the Pi. By
default it usees address 0x68, but this can be modified with an additional DT
parameter:

    dtoverlay=i2c-rtc,ds1307,addr=0x68

Parameters usually have default values, although certain parameters are
mandatory. See the list of overlays below for a description of the parameters
and their defaults.

Making new Overlays based on existing Overlays
==============================================

Recent overlays have been designed in a more general way, so that they can be
adapted to hardware by changing their parameters. When you have additional
hardware with more than one device of a kind, you end up using the same overlay
multiple times with other parameters, e.g.

    # 2 CAN FD interfaces on spi but with different pins
    dtoverlay=mcp251xfd,spi0-0,interrupt=25
    dtoverlay=mcp251xfd,spi0-1,interrupt=24

    # a realtime clock on i2c
    dtoverlay=i2c-rtc,pcf85063

While this approach does work, it requires knowledge about the hardware design.
It is more feasible to simplify things for the end user by providing a single
overlay as it is done the traditional way.

A new overlay can be generated by using ovmerge utility.
https://github.com/raspberrypi/utils/blob/master/ovmerge/ovmerge

To generate an overlay for the above configuration we pass the configuration
to ovmerge and add the -c flag.

    ovmerge -c mcp251xfd-overlay.dts,spi0-0,interrupt=25 \
               mcp251xfd-overlay.dts,spi0-1,interrupt=24 \
               i2c-rtc-overlay.dts,pcf85063 \
    >> merged-overlay.dts

The -c option writes the command above as a comment into the overlay as
a marker that this overlay is generated and how it was generated.
After compiling the overlay it can be loaded in a single line.

    dtoverlay=merged

It does the same as the original configuration but without parameters.

The Overlay and Parameter Reference
===================================

N.B. When editing this file, please preserve the indentation levels to make it
simple to parse programmatically. NO HARD TABS.


Name:   <The base DTB>
Info:   Configures the base Raspberry Pi hardware
Load:   <loaded automatically>
Params:
        ant1                    Select antenna 1 (default). CM4 only.

        ant2                    Select antenna 2. CM4 only.

        noant                   Disable both antennas. CM4 only.

        audio                   Set to "on" to enable the onboard ALSA audio
                                interface (default "off")

        axiperf                 Set to "on" to enable the AXI bus performance
                                monitors.
                                See /sys/kernel/debug/raspberrypi_axi_monitor
                                for the results.

        cam0_reg                Enables CAM 0 regulator. CM1 & 3 only.

        cam0_reg_gpio           Set GPIO for CAM 0 regulator. Default 30.
                                CM1 & 3 only.

        cam1_reg                Enables CAM 1 regulator. CM1 & 3 only.

        cam1_reg_gpio           Set GPIO for CAM 1 regulator. Default 2.
                                CM1 & 3 only.

        eee                     Enable Energy Efficient Ethernet support for
                                compatible devices (default "on"). See also
                                "tx_lpi_timer". Pi3B+ only.

        eth_downshift_after     Set the number of auto-negotiation failures
                                after which the 1000Mbps modes are disabled.
                                Legal values are 2, 3, 4, 5 and 0, where
                                0 means never downshift (default 2). Pi3B+ only.

        eth_led0                Set mode of LED0 - amber on Pi3B+ (default "1"),
                                green on Pi4 (default "0").
                                The legal values are:

                                Pi3B+

                                0=link/activity          1=link1000/activity
                                2=link100/activity       3=link10/activity
                                4=link100/1000/activity  5=link10/1000/activity
                                6=link10/100/activity    14=off    15=on

                                Pi4

                                0=Speed/Activity         1=Speed
                                2=Flash activity         3=FDX
                                4=Off                    5=On
                                6=Alt                    7=Speed/Flash
                                8=Link                   9=Activity

        eth_led1                Set mode of LED1 - green on Pi3B+ (default "6"),
                                amber on Pi4 (default "8"). See eth_led0 for
                                legal values.

        eth_max_speed           Set the maximum speed a link is allowed
                                to negotiate. Legal values are 10, 100 and
                                1000 (default 1000). Pi3B+ only.

        i2c_arm                 Set to "on" to enable the ARM's i2c interface
                                (default "off")

        i2c_vc                  Set to "on" to enable the i2c interface
                                usually reserved for the VideoCore processor
                                (default "off")

        i2c                     An alias for i2c_arm

        i2c_arm_baudrate        Set the baudrate of the ARM's i2c interface
                                (default "100000")

        i2c_vc_baudrate         Set the baudrate of the VideoCore i2c interface
                                (default "100000")

        i2c_baudrate            An alias for i2c_arm_baudrate

        i2s                     Set to "on" to enable the i2s interface
                                (default "off")

        krnbt                   Set to "on" to enable autoprobing of Bluetooth
                                driver without need of hciattach/btattach
                                (default "off")

        krnbt_baudrate          Set the baudrate of the PL011 UART when used
                                with krnbt=on

        spi                     Set to "on" to enable the spi interfaces
                                (default "off")

        spi_dma4                Use to enable 40-bit DMA on spi interfaces
                                (the assigned value doesn't matter)
                                (2711 only)

        random                  Set to "on" to enable the hardware random
                                number generator (default "on")

        sd_overclock            Clock (in MHz) to use when the MMC framework
                                requests 50MHz

        sd_poll_once            Looks for a card once after booting. Useful
                                for network booting scenarios to avoid the
                                overhead of continuous polling. N.B. Using
                                this option restricts the system to using a
                                single card per boot (or none at all).
                                (default off)

        sd_force_pio            Disable DMA support for SD driver (default off)

        sd_pio_limit            Number of blocks above which to use DMA for
                                SD card (default 1)

        sd_debug                Enable debug output from SD driver (default off)

        sdio_overclock          Clock (in MHz) to use when the MMC framework
                                requests 50MHz for the SDIO/WLAN interface.

        tx_lpi_timer            Set the delay in microseconds between going idle
                                and entering the low power state (default 600).
                                Requires EEE to be enabled - see "eee".

        uart0                   Set to "off" to disable uart0 (default "on")

        uart1                   Set to "on" or "off" to enable or disable uart1
                                (default varies)

        watchdog                Set to "on" to enable the hardware watchdog
                                (default "off")

        act_led_trigger         Choose which activity the LED tracks.
                                Use "heartbeat" for a nice load indicator.
                                (default "mmc")

        act_led_activelow       Set to "on" to invert the sense of the LED
                                (default "off")
                                N.B. For Pi 3B, 3B+, 3A+ and 4B, use the act-led
                                overlay.

        act_led_gpio            Set which GPIO to use for the activity LED
                                (in case you want to connect it to an external
                                device)
                                (default "16" on a non-Plus board, "47" on a
                                Plus or Pi 2)
                                N.B. For Pi 3B, 3B+, 3A+ and 4B, use the act-led
                                overlay.

        pwr_led_trigger
        pwr_led_activelow
        pwr_led_gpio
                                As for act_led_*, but using the PWR LED.
                                Not available on Model A/B boards.

        N.B. It is recommended to only enable those interfaces that are needed.
        Leaving all interfaces enabled can lead to unwanted behaviour (i2c_vc
        interfering with Pi Camera, I2S and SPI hogging GPIO pins, etc.)
        Note also that i2c, i2c_arm and i2c_vc are aliases for the physical
        interfaces i2c0 and i2c1. Use of the numeric variants is still possible
        but deprecated because the ARM/VC assignments differ between board
        revisions. The same board-specific mapping applies to i2c_baudrate,
        and the other i2c baudrate parameters.

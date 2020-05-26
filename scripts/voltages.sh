#!/usr/bin/env bash

cat /sys/bus/iio/devices/iio\:device0/in_voltage0_user0_input # AIY GPIO A
cat /sys/bus/iio/devices/iio\:device0/in_voltage1_user1_input # AIY GPIO B
cat /sys/bus/iio/devices/iio\:device0/in_voltage2_user2_input # AIY GPIO C - voltage
cat /sys/bus/iio/devices/iio\:device0/in_voltage3_user3_input # AIY GPIO D - current


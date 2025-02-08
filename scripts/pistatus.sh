#!/bin/bash

pi_status()
{
  echo NAME  : $(uname -n)
  echo MODEL : $(cat /sys/firmware/devicetree/base/model | tr '\0' ' ')
  echo OS    : $(grep PRETTY_NAME /etc/os-release | cut -d \" -f 2)
  echo KERNEL: $(uname -r)
  echo IP    : $(ip address | grep inet |tr -s ' '| cut -d ' ' -f 3)
  echo UPTIME:$(uptime)
  # echo NodeRed: $(systemctl is-enabled nodered) $(systemctl is-active nodered)
  # echo Mosquitto: $(systemctl is-enabled mosquitto) $(systemctl is-active mosquitto)

  # vcgencmd docs https://www.raspberrypi.com/documentation/computers/os.html#vcgencmd
  echo
  echo CPU $(vcgencmd measure_temp)
  echo ARM cores $(vcgencmd measure_clock arm)
  echo GPU cores $(vcgencmd measure_clock core)

  echo
  echo Voltages:
  echo VC4 core voltage $(vcgencmd measure_volts core)
  echo SDRAM I/O voltage $(vcgencmd measure_volts sdram_i)
  echo SDRAM Phy Voltage $(vcgencmd measure_volts sdram_p)
  echo SDRAM Core Voltage $(vcgencmd measure_volts sdram_c)


  STATUS=$(vcgencmd get_throttled | sed -n 's|^throttled=\(.*\)|\1|p')
  if [[ ${STATUS} -ne 0 ]]; then
    echo ""
    if [ $((${STATUS} & 0x00001)) -ne 0 ]; then
      echo "Power is currently Under Voltage"
    elif [ $((${STATUS} & 0x10000)) -ne 0 ]; then
      echo "Power has previously been Under Voltage"
    fi
    if [ $((${STATUS} & 0x00002)) -ne 0 ]; then
      echo "ARM Frequency is currently Capped"
    elif [ $((${STATUS} & 0x20000)) -ne 0 ]; then
      echo "ARM Frequency has previously been Capped"
    fi
    if [ $((${STATUS} & 0x00004)) -ne 0 ]; then
      echo "CPU is currently Throttled"
    elif [ $((${STATUS} & 0x40000)) -ne 0 ]; then
      echo "CPU has previously been Throttled"
    fi
    if [ $((${STATUS} & 0x00008)) -ne 0 ]; then
      echo "Currently at Soft Temperature Limit"
    elif [ $((${STATUS} & 0x80000)) -ne 0 ]; then
      echo "Previously at Soft Temperature Limit"
    fi
    echo ""
  fi
}

if [[ $1 == --show ]]; then
  pi_status
else
  watch -n .5 $0 --show
fi

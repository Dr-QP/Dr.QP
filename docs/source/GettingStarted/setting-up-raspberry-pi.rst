Setting up Raspberry Pi for the Robot
=======================================

Install Ubuntu 24.04
---------------------

Ubuntu 24.04 is the only LTS currently supported by Raspberry Pi 5

Install ROS and docker

Update networkd to not wait for ethernet
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Run: ``sudo systemctl edit systemd-networkd-wait-online.service``

Paste an override into edit section:

::

   [Service]
   ExecStart=
   ExecStart=/lib/systemd/systemd-networkd-wait-online --ignore=eth0 --quiet

Install sc16is752 overlays
~~~~~~~~~~~~~~~~~~~~~~~~~~

The UART expansion board is based on sc16is752 and needs an overlay

Update config.txt
~~~~~~~~~~~~~~~~~

``sudo nano /boot/firmware/config.txt``

Installing deployment service
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Production docker container is deployed using Ansible.
Run the following command on your dev host to install docker and setup autorun service

::
  ansible-playbook -i inventory.yml playbooks/*.yaml --ask-become-pass


::

   [all]
   # Dr.QP: UART via I2C devices. addr is different according to status of A0/A1, default 0X48
   dtoverlay=sc16is752-i2c,int_pin=24,addr=0x48

   # Skip power supply check on RPi5
   usb_max_current_enable=1

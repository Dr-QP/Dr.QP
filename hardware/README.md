# Hardware setup manual/blog

Dr.QP platform utilities in the Google's AIY Vision Bonnet. The simplest way to get it up and running is to use AIY-projects' pre-built Raspbian image from https://github.com/google/aiyprojects-raspbian/releases.

Note: [HACKING.md](https://github.com/google/aiyprojects-raspbian/blob/aiyprojects/HACKING.md) describes a manual setup process, but some packages can't be really installed and need to be hacked even more around. So using pre-built image is kind of the only option right now (May 2020)

## Initial setup of Raspberry Pi

Basic setup is essentially same as for stock Raspbian image

1. Connect LAN cable and SSH into it
2. Run `sudo raspi-config`
3. Choose `Update` to get latest `raspi-config`
4. Change locale to match your desktop/laptom (en_US for me). This allows special keyboard keys (e.g. `!@#$%`) to be mapped correctly.
5. Set hostname to `dr-qp.local` (this name is going to be used in all other places as reference name)
6. Configure Wi-Fi
7. Disable Raspberry Desktop (Boot options => Desktop/CLI => Console)
8. Reboot
9. SSH back again 
10. Change password using `passwd`
11. (Optional) configure ssh to use key authentication (add key from id_rsa.pub on your machine to `.ssh/authorized_keys` on `dr-qp.local`)
12. (Risky, may fail and corrupt the AIY setup) Run `sudo apt update && sudo apt upgrade -y`
13. Disable stock demo `sudo systemctl stop joy_detection_demo && sudo systemctl disable joy_detection_demo`
14. Update `/boot/config.txt`

```
# Dr.QP: enable i2c
dtparam=i2c_arm=on
dtparam=spi=on

# Dr.QP: UART via I2C devices. addr is different according to status of A0/A1, default 0X48
dtoverlay=sc16is752-i2c,int_pin=24,addr=0x48
```

14. Check that I2C is running and UART HAT is connected using `sudo i2cdetect -y 1`
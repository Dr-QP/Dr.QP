#!/usr/bin/env bash

sudo dd if=/dev/sda bs=4M status=progress | xz -z -c > /media/anton/dev/backup-image.img.xz

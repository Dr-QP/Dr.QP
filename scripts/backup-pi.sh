#!/bin/bash
# script to backup Pi SD card
# 2017-06-05
# 2018-11-29    optional name
# DSK='disk4'   # manual set disk
set +e
set +x

OUTDIR=~/temp/Pi

# Find disk with Linux partition (works for Raspbian)
# Modified for PINN/NOOBS
export DSK=`diskutil list | grep "Linux" | sed 's/.*\(disk[0-9]\).*/\1/' | uniq`
if [ $DSK ]; then
    echo $DSK
    echo $OUTDIR
else
    echo "Disk not found"
    exit
fi

if [ $# -eq 0 ] ; then
    BACKUPNAME='Pi'
else
    BACKUPNAME=$1
fi
BACKUPNAME+="back"
echo $BACKUPNAME

diskutil unmountDisk /dev/$DSK
echo please wait - This takes some time
echo Ctl+T to show progress!
mkdir -p "$OUTDIR"
time sudo dd if=/dev/r$DSK bs=4m | gzip -9 > $OUTDIR/Piback.img.gz

#rename to current date
echo compressing completed - now renaming
mv -n $OUTDIR/Piback.img.gz $OUTDIR/$BACKUPNAME`date +%Y%m%d`.img.gz

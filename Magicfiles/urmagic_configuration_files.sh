#!/bin/sh
# 
# file   urmagic_copy_configuration_files.sh
# author Emil Pedersen <fep@universal-robots.com>
#  
# Script to copy robot configuration files to usb
# 

LOGGER="/usr/bin/logger"

if [ "$1" = "" ] ; then
    $LOGGER -p user.info "$0: no mountpoint supplied, exiting."
    exit 1 ; fi

MOUNTPOINT="$1"
SERIALNO_FILE="/root/ur-serial"
FILES2COPY=("/root/.urcontrol/*")

# Warn user not to remove USB key
echo "! USB !" | DISPLAY=:0 aosd_cat -R red -x 230 -y -210 -n "Arial Black 80"

# Find the serial number of the robot
if [ -e "$SERIALNO_FILE" ]; then
    SERIALNO=`head -n1 $SERIALNO_FILE`
    if [ -z $SERIALNO ]; then
	      SERIALNO="no-serial-no-found"
    fi
else
	SERIALNO="no-serial-no-found"    
fi

DATADIR="$SERIALNO"
DATAPATH="$MOUNTPOINT/$DATADIR"

# Find a name for the library on the USB key (based on the serial number) to copy the data to
CNT=0
while [ -e "$DATAPATH" ]; do
	DATAPATH="${MOUNTPOINT}/${DATADIR}_${CNT}"
	let CNT+=1
done 
mkdir -p $DATAPATH
$LOGGER -p user.info "$0: using datapath: $DATAPATH"    

# Copy the data in the list
for file in ${FILES2COPY[*]} 
do
    $LOGGER -p user.info "$0: file: ${file}"
    cp "${file}" "${DATAPATH}/." 
done
# Make sure data is written to the USB key
sync
sync

# Notify user it is ok to remove USB key
echo "<- USB" | DISPLAY=:0 aosd_cat -x 200 -y -210 -n "Arial Black 80"

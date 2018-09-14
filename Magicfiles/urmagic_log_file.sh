#!/bin/sh
#
# file   urmagic_cp_data.sh
#
# Script to extract data from a robot
#

LOGGER=/usr/bin/logger

if [ "$1" = "" ] ; then
    $LOGGER -p user.info "$0: no mountpoint supplied, exiting."
    exit 1 ; fi

MOUNTPOINT="$1"
SERIALNO_FILE="/root/ur-serial"
FILES2COPY=("/root/log_history.*" "ur-gui.*" "/root/flightreports")

# Warn user not to remove USB key
echo "! USB !" | DISPLAY=:0 aosd_cat -R red -x 230 -y -210 -n "Arial Black 80"

#save the log in PolyScope by messaging the dashboard server
echo "$(/usr/bin/python -c '
import socket;
s=socket.socket(socket.AF_INET,socket.SOCK_STREAM);
s.connect(("127.0.0.1",29999));
s.send("saveLog\n");
s.close();
')"

sleep 2

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
DATAPATH="${MOUNTPOINT}/${DATADIR}"

# Find a name for the library on the USB key (based on the serial number) to copy the data to
CNT=0
while [ -e "$DATAPATH" ]; do
  DATAPATH="${MOUNTPOINT}/${DATADIR}_${CNT}"
  let CNT+=1
done
mkdir -p "$DATAPATH"
$LOGGER -p user.info "$0: using datapath: ${DATAPATH}"

# Copy the data in the list
for file in ${FILES2COPY[*]}
do
    $LOGGER -p user.info "$0: file: $file"
    cp -r "${file}" "${DATAPATH}/."
done
# Make sure data is written to the USB key
sync
sync

# Notify user it is ok to remove USB key
echo "<- USB" | DISPLAY=:0 aosd_cat -x 200 -y -210 -n "Arial Black 80"

#!/bin/bash
################################################################################
# Copyright (c) 2019 ModalAI, Inc. All rights reserved.
#
# Installs the ipk package on target.
# Requires the ipk to be built and an adb connection.
#
# author: james@modalai.com
################################################################################
set -e

PACKAGE=$(cat ipk/control/control | grep "Package" | cut -d' ' -f 2)

# count ipk files in current directory
NUM_FILES=$(ls -1q $PACKAGE*.ipk | wc -l)

if [ $NUM_FILES -eq "0" ]; then
	echo "ERROR: no ipk file found"
	echo "run build.sh and make_ipk.sh first"
	exit 1
elif [ $NUM_FILES -gt "1" ]; then
	echo "ERROR: more than 1 ipk file found"
	echo "make sure there is only one ipk file in the current directory"
	exit 1
fi

# now we know only one ipk file exists
FILE=$(ls -1q $PACKAGE*.ipk)
echo "pushing $FILE to target"

echo "searching for ADB device"
adb wait-for-device
echo "adb device found"


adb push $FILE /home/root/ipk/$FILE
adb shell "opkg install --force-reinstall --force-downgrade --force-depends /home/root/ipk/$FILE"

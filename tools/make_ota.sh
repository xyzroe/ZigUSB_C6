#!/bin/bash

# Set the script to exit on any errors
set -e

# Description: This script creates an OTA file from the ZigUSB.bin file.
cd ./tools

# Install the required Python packages
python3 -m pip install -q zigpy

# Read the values from const.h
MANUFACTURER=$(grep -o '#define\s\+OTA_UPGRADE_MANUFACTURER\s\+0x[0-9a-fA-F]\+' ../main/const.h | awk '{print $3}')
IMAGE_TYPE=$(grep -o '#define\s\+OTA_UPGRADE_IMAGE_TYPE\s\+0x[0-9a-fA-F]\+' ../main/const.h | awk '{print $3}')
FILE_VERSION=$(grep -o '#define\s\+OTA_FW_VERSION\s\+0x[0-9a-fA-F]\+' ../main/const.h | awk '{print $3}')

# Check if the ZigUSB.bin file exists
if [ ! -f "../build/ZigUSB.bin" ]; then
    echo "ZigUSB.bin file not found!"
    exit 1
fi

# Print the values
echo "M: $MANUFACTURER | IT: $IMAGE_TYPE | FV: $FILE_VERSION";

# Create the output folder if it doesn't exist yet
mkdir -p ../output

# Create the OTA file
python3 create-ota.py -m "$MANUFACTURER" -i "$IMAGE_TYPE" -v "$FILE_VERSION" ../build/ZigUSB.bin ../output/ZigUSB_C6.ota

# Create a combined binary file
esptool.py --chip esp32 merge_bin -o ../output/ZigUSB_C6.bin \
--flash_mode dio --flash_freq 40m --flash_size 4MB \
0x0 ../build/bootloader/bootloader.bin \
0x8000 ../build/partition_table/partition-table.bin \
0xf000 ../build/ota_data_initial.bin \
0x20000 ../build/ZigUSB.bin

echo "OTA file and combined binary created successfully! Version: $FILE_VERSION"
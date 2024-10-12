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

# Copy the ZigUSB.bin file to the output folder
cp ../build/ZigUSB.bin ../output/ZigUSB_C6.bin

echo "OTA file created successfully! Version: $FILE_VERSION"
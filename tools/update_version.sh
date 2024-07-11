#!/bin/bash

# Set the first release date
first_release_date="20240101"

# Get the current date
current_date=$(date +%Y%m%d)

# Calculate the version number
version_number=$((current_date - first_release_date))
if [ $version_number -lt 1 ]; then
  version_number=1
fi

# Convert the version number to hexadecimal
hex_version=$(printf "%08X" $version_number)

# Print the new version number, HEX version, date
echo "New version number: $version_number"
echo "HEX: 0x$hex_version"
echo "build date: $current_date"

# Update the device version in ZigUSB.h using awk
awk -v hex_version="$hex_version" '/OTA_FW_VERSION/ {sub(/0x[0-9A-F]*/, "0x" hex_version)} 1' main/ZigUSB.h > temp && mv temp main/ZigUSB.h
awk -v fw_date="$current_date" '/FW_BUILD_DATE/ {sub(/"[^"]*"/, "\"" fw_date "\"")} 1' main/ZigUSB.h > temp && mv temp main/ZigUSB.h
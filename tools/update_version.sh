#!/bin/bash

current_date=$(date +%Y%m%d)

# Check if the script is being run in a GitHub workflow
if [ "$GITHUB_ACTIONS" = true ]; then
  # Get the latest tag
  latest_tag=$(git describe --tags --abbrev=0)
  # Extract the version number from the latest tag
  version_number=${latest_tag}

else
  # Set the first release date
  first_release_date="20240701"

  # Get the current date
  current_date=$(date +%Y%m%d)

  # Calculate the version number
  version_number=$((current_date - first_release_date))
  if [ $version_number -lt 1 ]; then
    version_number=1
  fi
fi

# Convert the version number to hexadecimal
hex_version=$(printf "%08X" $version_number)

# Print the new version number, HEX version, date
echo "New version number: $version_number"
echo "HEX: 0x$hex_version"
echo "build date: $current_date"

# Set the output date variable
echo "version=$version_number" >> $GITHUB_ENV
# Set the output date variable
echo "build_date=$current_date" >> $GITHUB_ENV

# Update the device version in ZigUSB.h using awk
awk -v hex_version="$hex_version" '/OTA_FW_VERSION/ {sub(/0x[0-9A-F]*/, "0x" hex_version)} 1' main/ZigUSB.h > temp && mv temp main/ZigUSB.h
awk -v fw_date="$current_date" '/FW_BUILD_DATE/ {sub(/"[^"]*"/, "\"" fw_date "\"")} 1' main/ZigUSB.h > temp && mv temp main/ZigUSB.h
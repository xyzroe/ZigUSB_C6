#!/bin/bash

# Define colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Ask for confirmation to pull changes
echo -e "${YELLOW}Do you want to pull the latest changes from the repository? 🔄 (Y/n)${NC}"
read -r response
response=${response:-y} # default 'yes' if empty
if [[ "$response" =~ ^[Yy]$ ]]; then
    git pull
    echo -e "${GREEN}Changes pulled successfully. ✔️${NC}"
else
    echo -e "${YELLOW}Pull skipped. Continuing without pulling changes. ⚠️${NC}"
fi

# Adding all changes to staging
git add -A
echo -e "${GREEN}All changes added to staging. ✔️${NC}"

# Path to the version and commit message files
COMMIT_MESSAGE_FILE="commit.md"

# Get the latest tag
latest_tag=$(git describe --tags --abbrev=0)

# Check if any tags exist
if [ -z "$latest_tag" ]; then
    echo -e "${RED}No tags found. ❌${NC}"
    git_version_number=1
else

    # Extract the version number from the latest tag
    git_version_number=${latest_tag}
fi

# Set the first release date
first_release_date="20240101"

# Get the current date
current_date=$(date +%Y%m%d)

# Calculate the version number
local_version_number=$((current_date - first_release_date))
if [ $local_version_number -lt 1 ]; then
local_version_number=1
fi

if [ "$git_version_number" -lt "$local_version_number" ]; then
    version_number=$((local_version_number))
else
    version_number=$((git_version_number + 1))
fi

tag=$version_number

echo "Using tag to $tag"

# Checking for commit message file
if [ -f "$COMMIT_MESSAGE_FILE" ]; then
    echo -e "${YELLOW}Commit message file found. Do you want to use the existing commit message? (y/N) 📝${NC}"
    read -r useExistingMessage
    useExistingMessage=${useExistingMessage:-n} # default 'no' if empty
    if [[ "$useExistingMessage" =~ ^[Yy]$ ]]; then
        commitMessage=$(cat "$COMMIT_MESSAGE_FILE")
        # Prepend version to the commit message with a newline for separation
        formattedCommitMessage="${tag}
        ${commitMessage}"
        # Cleaning up the commit message file, if used
        if [ -f "$COMMIT_MESSAGE_FILE" ]; then
            tools/clean_file.sh "$COMMIT_MESSAGE_FILE"
        fi
    else
        echo -e "${YELLOW}Please enter your commit message: 📝${NC}"
        read -r commitMessage
        formattedCommitMessage="${commitMessage}"
    fi
else
    echo -e "${YELLOW}Commit message file not found. Please enter your commit message: 📝${NC}"
    read -r commitMessage
    formattedCommitMessage="${commitMessage}"
fi

# Committing changes
git commit -m "$formattedCommitMessage"
echo -e "${GREEN}Changes committed with version prepended to message: ✔️${NC}"

# Tagging process
echo -e "${YELLOW}Do you want to create a new release by publishing a tag? 🏷️ (y/N)${NC}"
read -r tagCommit
tagCommit=${tagCommit:-n} # default 'no' if empty
if [[ "$tagCommit" =~ ^[Yy]$ ]]; then
    git tag "$tag"
    echo -e "${GREEN}Tag assigned: $tag 🏷️${NC}"
    
    # Pushing changes and tag
    echo -e "${YELLOW}Do you want to push the changes and the new tag to the remote repository? 🚀 (Y/n)${NC}"
    read -r pushChanges
    pushChanges=${pushChanges:-y} # default 'yes' if empty
    if [[ "$pushChanges" =~ ^[Yy]$ ]]; then
        git push
        git push origin "$tag"
        echo -e "${GREEN}Changes and new tag pushed successfully. ✔️${NC}"
    else
        echo -e "${RED}Push of changes and tag skipped. ❌${NC}"
    fi
else
    echo -e "${YELLOW}No new release will be created. Do you still want to push the changes? (Y/n) 🚀${NC}"
    read -r pushChanges
    pushChanges=${pushChanges:-y} # default 'yes' if empty
    if [[ "$pushChanges" =~ ^[Yy]$ ]]; then
        git push
        echo -e "${GREEN}Changes pushed successfully without creating a new release. ✔️${NC}"
    else
        echo -e "${RED}Push skipped. ❌${NC}"
    fi
fi


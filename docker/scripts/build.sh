#!/bin/bash

# Author: João Pedro Baltieca Garcia, aka JPBG-USP

# Essential arguments, other arguments are in docker_config.sh
WORKREPO=graspe                  # Name of the working repository
CONFIGFILE=docker/config/docker_configs.sh  # Path to the configuration file

# Simple color codes for output messages, additional colors are defined in tools.sh
BIYELLOW='\033[1;93m'  # Bright Yellow
BIRED='\033[1;91m'     # Bright Red
CYAN='\033[0;36m'      # Cyan
NC='\033[0m'           # No Color

# Check if the current working directory is the correct repository
if [[ $PWD = *$WORKREPO ]]; then
    # Attempt to source the configuration file
    if [[ -f $CONFIGFILE ]]; then
        source $CONFIGFILE
    else
        # Configuration file not found, exit with error
        echo -e "\n[$(date +"%T")]${BIRED}[ERROR]${NC} ${CYAN}docker_configs.sh file${NC} not found, the file must be in '${CYAN}$CONFIGFILE${NC}'.\n \a"
        exit 1
    fi

    # Attempt to source the tools file
    if [[ -f $TOOLSFILE ]]; then
        source $TOOLSFILE
    else
        # Tools file not found, exit with error
        echo -e "\n[$(date +"%T")]${BIRED}[ERROR]${NC} ${CYAN}tools.sh${NC} not found, the file must be in '${CYAN}$TOOLSFILE${NC}'.\n \a"
        exit 1
    fi
else
    # Not in the correct directory, exit with warning
    echo -e "\n[$(date +"%T")]${BIYELLOW}[WARN]${NC} You must be in the '${CYAN}$WORKREPO${NC}' directory to run this command.\n \a"
    exit 1
fi

# Check if the Docker image already exists
if docker image inspect "$IMAGENAME:$IMAGETAG" > /dev/null 2>&1; then
    # Image exists, prompt the user to create with a different tag
    if confirm "The ${CYAN}$IMAGENAME:$IMAGETAG${NC} image already exists. Do you want to create the image with a different tag?"; then
        # Read the new tag from the user
        read -p "Insert the new tag for the image: " new_image_tag
        IMAGETAG=$new_image_tag
    else
        # Confirm if the user wants to overwrite the existing image
        if ! confirm "Are you sure you want to overwrite the old image with the new one?"; then
            echo -e "\n[$(date +"%T")][INFO] Docker Image build canceled."
            exit 1
        fi
    fi
fi

# Build the Docker image with specified configurations
docker build \
    --network=host \
    -f $DOCKERFILE \
    -t $IMAGENAME:$IMAGETAG \
    --build-arg USERNAME=$USERNAME \
    --rm \
    .

echo -e "\n[$(date +"%T")][INFO] Docker image built successfully."
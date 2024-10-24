#!/bin/bash

# /author: João Pedro Baltieca Garcia, aka JPBG-USP

# Arguments for easy future changes
WORKREPO=graspe
CONFIGFILE=docker/config/docker_configs.sh  # Path to the configuration file

# Simples colors, other are in TOOLSFILE
BIYELLOW='\033[1;93m'
BIRED='\033[1;91m'       
NC='\033[0m' 

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

# Checking if the image with the selected tag exists
if ! docker image inspect "$IMAGENAME:$IMAGETAG" > /dev/null 2>&1; then
    echo -e "\n[$(date +"%T")]${BIRED}[ERRO]${NC} There is no ${CYAN}$IMAGENAME:$IMAGETAG${NC} image.\n \a"
fi

# Checking if a container with the selected name already exists
if [ -n "$(docker ps -a --filter "name=$CONTAINERNAME" --format "{{.ID}}")" ]; then
    if ! confirm "\n[$(date +"%T")]${BIYELLOW}[WARN]${NC} A container with the name ${CYAN}$CONTAINERNAME${NC} and the image ${CYAN}$IMAGENAME:$IMAGETAG${NC} already exists. Do you want to delete the old container and restart a new one? (To open a new terminal in the container, use exec.sh).\a"; then
        echo -e "\n[$(date +"%T")][INFO] Operation canceled.\n"
        exit 1
    fi
    echo -e -n "[$(date +"%T")][INFO] Container removed: "
    docker container rm $CONTAINERNAME
    echo -e "[$(date +"%T")][INFO] Starting container..."
fi

# run of the image
docker run -it \
    --network=host \
    --ipc=host \
    --device /dev/video0 \
    -v /dev/video0:/dev/video0 \
    -v /dev/dri:/dev/dri \
    -v $PWD/$ROSWS:/home/$USERNAME/$ROSWS \
    -e DISPLAY=$DISPLAY \
    --name $CONTAINERNAME \
    $IMAGENAME:$IMAGETAG
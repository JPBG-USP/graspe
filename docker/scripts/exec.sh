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

# Check if there is a container with the name: # CHange this to check 
if ! [ -n "$(docker ps -a --filter "name=$CONTAINERNAME" --format "{{.ID}}")" ]; then
    echo -e "\n[$(date +"%T")]${BIRED}[ERRO]${NC} There is no container named ${CYAN}$CONTAINERNAME${NC}.\n \a"
    exit 1
fi

if ! docker ps --format '{{.Names}}' | grep -q "^$CONTAINERNAME$"; then
    echo -e "\n[$(date +"%T")]${BIRED}[ERRO]${NC} The ${CYAN}$CONTAINERNAME${NC} container is not running.\n \a"
    exit 1
fi

read -p "Enter the command you want to execute in the container (Enter nothing to just open a new terminal): " command_terminal

if [ -z "$command_terminal" ]; then
    command_terminal=/bin/bash
fi
try docker exec -it $CONTAINERNAME $command_terminal

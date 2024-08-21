#!/bin/bash

# Color arguments
WHITE='\033[0;37m'
RED='\033[0;31m'
BIRED='\033[1;91m'
GREEN='\033[0;32m'
BIGREEN='\033[1;92m'        
YELLOW='\033[0;33m'
BIYELLOW='\033[1;93m'    
CYAN='\033[0;36m'  
NC='\033[0m' 

# confirm function
function confirm()
{
    echo -n -e "$1"
    read -p "[y/n] " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]
    then
	true && return
    fi
    false
}

function try()
{
    "$@"
    local status=$?
    if [ $status -ne 0 ]; then
        echo "\n[$(date +"%T")]${BIRED}[ERRO]${NC} The '$*' command failed with status: '$status'"
        return $status
    fi
    return 0
}

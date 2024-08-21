#!/bin/bash

# Files
WORKREPO=graspe
TOOLSFILE=docker/config/tools.sh

# Docker configs
IMAGENAME=$WORKREPO
IMAGETAG=humble
DOCKERFILE=docker/Dockerfile.roshumble
CONTAINERNAME=graspe_container

# Container configs
USERNAME=host
ROSWS=ros_ws
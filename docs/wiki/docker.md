# Docker 
We use Docker in the project because it avoids compatibility and versioning problems, helping with team development across multiple operating systems.

Is recommend having prior knowledge of docker, you can find the documentation [here](https://docs.docker.com/), we also recommend the playlist of [Docker for Robotics](https://www.youtube.com/watch?v=XcJzOYe3E6M&list=PLunhqkrRNRhaqt0UfFxxC_oj7jscss2qe) from [Articulated Robotics](https://www.youtube.com/@ArticulatedRobotics) channel.

## Docker image
Our image is based on Ubuntu 22.04, with ROS2 Humble. Some base info about our docker image can be found below
- **Base Image:** ```osrf/ros:humble-desktop-full```
- **Dockerfile:** [Dockerfile.roshumble](../docker/Dockerfile.roshumble)
- **Image memory:** 4.25 GB

### Installing new ros packages
When developing projects with ROS, it is common to need to install additional packages. If you are using Grasp-e as the basis for your project, you will likely need to install new packages. You can do this directly within the container by using the command ```apt install $ROS_DISTRO-<package-name>```. However, this installation will be temporary, as the package will not be present when you start a new container.

If your project requires the ROS package to be installed permanently, the ideal solution is to add it to the Dockerfile, you can add the package in the stage below

```Dockerfile
# Stage 3 : Installing ROS packages
RUN apt-get update \
    && apt-get install -y \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-robot-state-publisher \
        ros-humble-tf2-ros \
        ros-humble-xacro \
        ros-humble-rviz2 \
        ros-humble-<new-ros-package>
    && rm -rf /var/lib/apt/lists/*
```

## Docker scripts
To facilitate development and use, we have developed bash scripts for build and run. You can check out more details about the scripts below.

> ❗All scripts must be executed from the ```graspe``` folder.

### Configs files
In order to make the scripts to deal with docker more parameterized, we created configuration files, which can be found in the ```docker/config``` folder.

- ```bashrc```: The bashrc file is a configuration script that runs whenever a new interactive terminal is started
- ```docker_configs.sh```: Contains variables that are used in build and run scripts. Making it easier to modify them in both scripts (e.g. container name, image name, username)
- ```tools.sh```: Contains some useful functions for scripts

> ❗There are two variables that are essential for the scripts, and to be changed they must be changed script by script manually. They are the name of the `WORKREPO` and the path to ```CONFIGFILE```

### Build script
The **build** script checks that you are in the correct folder (`WORKREPO`) and then runs `source` in the `docker_configs.sh` and `tools.sh`scripts.

After that, it checks if there is an image already with the same name defined in the docker configuration file. If it already exists, it will offer to create the image with a new tag

If you are testing a new image with new packages and applications that you have added, it is recommended that you create the image with a new tag in order to preserve the original image while the changes you made are being validated.

If you choose not to create a new tag, the script will ask if you want to rewrite the existing image. Denying will cancel the operation and accepting will delete the old image and build the new one.

If everything goes well, you should receive the message:
```bash
[xx:xx:xx][INFO] Docker image built successfully.
```

### Run script

The **run** script works similar to the build, it checks if it is in the correct folder, runs `source` in the `docker_configs.sh` and `tools.sh` files.

After that it checks if there is a docker image with the name and tag defined in the sh docker configuration file. If it does not exist, it will return an error.

Then it checks if there is already a container with the same name, if so, it asks if you want to delete the container and start a new one.

> ❗The `run.sh` does not yet accept images with different tags natively, we will add this in the future. We recommend changing the tag manually in the run script, or running the docker run in a terminal without the script.

- If you want to start a new terminal in a container that is active, just run outside the container: 

    ```bash
    docker exec -it <container-name> /bin/bash
    ```
    (The default container name is `graspe_container`)

- If you want to restart a container that you had already started, just run:
    ```bash
    docker container restart <container-name>
    ```
    In this case, the container restarts but a terminal accessing it is not opened. To do this, simply execute the command shown previously.
    With this too, the restarted container will not close when you close the terminal, you will have to close it manually by running:
    ```bash
    docker container stop <container-name>
    ```

> ❗You can check out our docker cheat sheet at [Cheat Sheet](docker_cheat_sheet)

### Exec script

THIS SCRIPT IS UNDER MAINTENANCE, DO NOT USE IT

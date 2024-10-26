# Get Started

This project is built on containers, so it is necessary to have Docker installed. You can follow the steps to install Docker [here](https://docs.docker.com/get-started/get-docker/).

To simplify the use and execution of the Docker container, scripts have been created for **build** and **run** operations. They can be found in the `docker/scripts` folder.

First, in the graspe folder, you need to build the Docker image by running the following command:

```bash
bash docker/scripts/build.sh
```
If everything goes well, you should receive: ```[xx:xx:xx][INFO] Docker image built successfully.```.

To start the container, simply run the command:

```bash
bash docker/scripts/run.sh
```

>If this is not your first time starting the container, you will receive a message indicating that a container already exists, along with an option to delete the existing container and open a new one. If you are just getting started, feel free to open a new container. However, if you are developing a project using Grasp-e, we recommend reviewing the documentation available at [docs/index.md](docs/index.md).

Now inside the container you have the ROS2 Humble environment, with the workspace and the graspe packages. To run the demo, you can run it inside the container

```bash
ros2 launch graspe_common demo_graspe.launch.py
```
(This command is not implement yet)
# Setup

> ❗ **Docker Required**: This project runs in a containerized environment, so you'll need Docker installed on your system. Follow the [Docker installation guide](https://docs.docker.com/get-started/get-docker/) if Docker is not yet installed.

To streamline the Docker image build and container execution process, we provide scripts located in the `docker/scripts` folder for **building** and **running** the container.

### Step 1: Build the Docker Image

In the `graspe` project folder, build the Docker image by running:
```bash
bash docker/scripts/build.sh
```
If the image builds successfully, you should see a confirmation message: `[xx:xx:xx][INFO] Docker image built successfully.`

### Step 2: Run the Docker Container

To start the container, execute the following command:
```bash
bash docker/scripts/run.sh
```
> **Note**: If this is not your first time starting the container, you might see a message indicating an existing container, with the option to remove it and start a new one. For initial runs, simply open a new container. However, if you’re working on development with Grasp-e, review the detailed documentation at the project [wiki](https://github.com/JPBG-USP/graspe/wiki/4.-Docker).

# Running the demo 

Once inside the container, the container provides the ROS 2 Humble environment, with the workspace and the necessary Grasp-e packages preloaded.
First, go to the `ros_ws`, and build it:

```bash
cd ros_ws/
colcon build
```

Now you have every thing setup to launch a demo of Grasp-e:
```bash
ros2 launch graspe_common demo_graspe.launch.py
```
*(Note: This command is currently not implemented.)*

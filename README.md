<p align="center">
  <img src="./docs/images/logo.png" width="400">
</p>

# Introduction

This project was developed as part of an academic assignment during our undergraduate studies. The objective of the assignment was to design a robotic manipulator to solve a problem freely chosen by the students, applying the concepts learned throughout the course.

The Grasp-e project consists of a robotic manipulator designed to implement the thinning and harvesting technique in grapevines, with a special focus on table grapes.

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

### Team

- João Assuéro Rêgo de Alencar: [Github](https://github.com/JoaoAlencar21), [LinkedIn](https://br.linkedin.com/in/jo%C3%A3o-assu%C3%A9ro-r%C3%AAgo-de-alencar-b386692b3?original_referer=https%3A%2F%2Fwww.google.com%2F)

- João Pedro Baltieca Garcia: [Github](https://github.com/JPBG-USP), [LinkedIn](https://br.linkedin.com/in/jo%C3%A3o-assu%C3%A9ro-r%C3%AAgo-de-alencar-b386692b3?original_referer=https%3A%2F%2Fwww.google.com%2F)

- ...
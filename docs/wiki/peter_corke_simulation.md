# The Robotics Toolbox
In the early stages of our project, we used Peter Corke's Robotics Toolbox to carry out the first simulations of the manipulator's kinematics. The toolbox offers a range of tools, which helped us test and verify the calculations. All files relating to the first simulations using the toolbox are found in the folder: `./peter_corke`
 - [Robotics Toolbox](https://petercorke.com/toolboxes/robotics-toolbox/)

# Setting up the conda environment
To begin using the developed codes, it is necessary to create a Python virtual environment. In this project, we use Conda as the tool to manage environments and dependencies. Using a virtual environment ensures that the required libraries and packages are isolated, preventing conflicts with other Python installations on your system.

To create the Conda environment, use the following command in the main folder 

```bash
conda env create -f peter_corke/environment.yml
```

Now you need to activate the conda environment

```bash
conda activate graspe_env
```

Now you can run the demo

```bash
python3 peter_corke/scripts/demo.py 
```

This should start viewing the robot, like the image below

<p align="center">
  <img src="https://github.com/JPBG-USP/graspe/blob/main/docs/images/peter_corke_simulation.png" width="600">
</p>

---
# Grasp-e (Robotics Toolbox)

## Graspe class
This class defines the RRPR Grasp-E manipulator, integrating the simulation functions from Peter Corke's toolbox, as well as functions for trajectory calculation, forward kinematics, and inverse kinematics.

| **Attribute** | **Type** | **Description**                                                                                                        |
|---------------|----------|------------------------------------------------------------------------------------------------------------------------|
| `l1_`         | `float`  | Length of the first link of the Grasp-E manipulator, influencing its reach and configuration.                         |
| `l2_`         | `float`  | Length of the second link of the Grasp-E manipulator, affecting the kinematics and range of motion.                  |
| `l5_`         | `float`  | Length of the fifth link of the Grasp-E manipulator, primarily used for aesthetic purposes in the manipulator design. |
| `graspeDH`    | `DHRobot`| Instance of the DHRobot class from Peter Corke's toolbox, defining the robot's structure and kinematics.             |

> **Obs:** More about the Length atributes in the `__init__` function.


| **Method**                          | **Description**                                                                                                                                   | **Parameters**                                    | **Returns**                |
|-------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------|----------------------------|
| [`__init__(l1, l2, l5)`](#__init__self-l1-float-l2-float-l5-float---none)              | Initializes the Grasp-E manipulator class with the specified link lengths.                                                                      | `l1` (float), `l2` (float), `l5` (float)         | None                       |
| [`ik_calculator(end_effector)`](#ik_calculatorself-end_effector-se3---npndarray)      | Calculates the joint positions given the position and orientation of the end effector.                                                          | `end_effector` (SE3)                             | `q` (np.ndarray (1, 4))    |
| [`static_display()`](#static_displayself---none)                  | Displays a static plot of the manipulator for visualization.                                                                                     | None                                              | None                       |
| [`dynamic_display()`](#dynamic_displayself---none)                 | Displays a dynamic plot of the manipulator to visualize trajectories using Peter Corke's Robotics Toolbox.                                      | None                                              | None                       |
| [`plan_trajectory_by_joints(joint_positions, steps)`](#plan_trajectory_by_jointsself-joint_positions-npndarray-steps-int--100---npndarray) | Calculates and returns the trajectory passing through all specified joint positions.                                                            | `joint_positions` (np.ndarray), `steps` (int)    | `traj` (np.ndarray (n, 5)) |


### `__init__(self, l1: float, l2: float, l5: float) -> None`
The class initialization receives the parameters of the most important dimensions for the robot, `l1_`, `l2_` and `l5_`. You can see the image below:

- **Input:** 
    - `l1`: float
    - `l2`: float
    - `l5`: float


<p align="center">
  <img src="https://github.com/JPBG-USP/graspe/blob/main/docs/images/coordinate_axes.png" width="600">
</p>

### `ik_calculator(self, end_effector: SE3) -> np.ndarray:`
Calculates the joint positions given the position and orientation of the end effector.

- **Input:** 
    - `end_effector`: SE3
- **Output**
    - `q`: np.ndarray (Joint angles shape: (1, 4)).

### `ik_calculator(self, end_effector: SE3) -> np.ndarray:`
Calculates the joint positions given the position and orientation of the end effector.

- **Input:** 
    - `end_effector`: SE3
- **Output**
    - `q`: np.ndarray (Joint angles shape: (1, 4)).

### `plan_trajectory_by_joints(self, joint_positions: np.ndarray, steps: int = 100) -> np.ndarray`
Calculates and returns the trajectory passing through all specified joint positions.
- **Input:** 
    - `joint_positions` (np.ndarray): Joint values in the format (n, 4), where n is the number of positions.
    - `steps` (int): Number of steps for trajectory planning (default: 100).
- **Output**
    - `traj` (np.ndarray): Calculated trajectory in the format (n, 5), including the aesthetic joint.

### `static_display(self) -> None:`
Displays a static plot of the manipulator for visualization using Peter Corke's Robotics Toolbox.

- **Input:** 
    - `None`
- **Output**
    - `None`

### `dynamic_display(self) -> None`
Displays a dynamic plot of the manipulator for visualization using Peter Corke's Robotics Toolbox.

- **Input:** 
    - `None`
- **Output**
    - `None`
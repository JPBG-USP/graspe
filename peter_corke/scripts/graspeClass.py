#!/usr/bin/env python3
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3

class GraspeManipulator():
    def __init__(self, l1: float, l2: float, l5: float) -> None:
        """
            ## Grasp-E Manipulator Class
            This class defines the RRPR Grasp-E manipulator, integrating the simulation functions from Peter Corke's toolbox, as well as functions for trajectory calculation, forward kinematics, and inverse kinematics.
            
            - **Input:** 
                - `l1`: float
                - `l2`: float
                - `l5`: float
        
            (Refer to the image to check the definitions of the coordinate axes, `docs/images/coordinate_axes.png`.)
        """
        
        # robot dimensions
        self.l1_ = l1
        self.l2_ = l2
        self.l5_ = l5
        
        # Defining the robotic manipulator
        self.graspeDH = rtb.DHRobot([
            rtb.RevoluteDH(d=self.l1_, a=0.0, alpha=-np.pi/2.0), # Theta1
            rtb.RevoluteDH(d=self.l2_, a=0.0, alpha=np.pi/2.0, offset=0), # Theta2
            rtb.PrismaticDH(theta=0.0, a=0.0, alpha=np.pi/2.0, qlim=[-10, 10]), # D3
            rtb.RevoluteDH(d=self.l2_, a=0.0, alpha=-np.pi/2), # Theta4
            rtb.RevoluteDH(d=self.l5_, a=0.0, alpha=0), # Rotation joint for aesthetic purposes only, not sending a real command.
        ],
        name='Grasp-E Manipulator')
        
        
    def ik_calculator(self, end_effector: SE3) -> np.ndarray:
        '''
            ### Inverse Kinematics Calculator:
            Calculates the joint positions given the position and orientation of the end effector.
            - **Input:** `end_effector: SE3`
            
            - **Output:** `q: np.ndarray (1, 4)`  
            
            - (Obs: Peter Corke's toolbox already performs the inverse kinematics of the robot, however, in order to verify the analytical forward kinematics calculated by us, we implemented the function manually.)
        '''
        
        q = np.array([0.0, 0.0, 0.0, 0.0]) # Creating the command vector
        
        ef_T = end_effector.A # Getting the end effector matrix

        # For more details of the IK, check the docs folder in "/docs/robot_kinematics.md#inverse-kinematics"
        q[0] = np.arctan(ef_T[1][3]/ef_T[0][3])
        q[1] = np.arccos((np.sin(q[0])*ef_T[2][1])/(np.cos(q[0])*ef_T[0][1] + np.sin(q[0])*ef_T[1][1]))
        q[3] = q[1] - np.arctan((np.cos(q[0])*ef_T[0][1] + np.sin(q[0])*ef_T[1][1])/(ef_T[2][1])) 
        q[2] = np.cos(q[0])*np.sin(q[1])*ef_T[0][3] + np.sin(q[0])*np.sin(q[1])*ef_T[1][3] + np.cos(q[2])*ef_T[2][3] - self.l1_ * np.cos(q[1]) - self.l5_*np.cos(q[3])
        
        return q

    def static_display(self) -> None:
        '''
            ### Static Display
            Function for static plot of the manipulator in order to visualize using Peter Corke's Robotic Tool box.
            - no input needed
            - Display pose: `q0 = [ pi/4, pi/4, 5.0, -pi/4 ]`
        '''
        
        q0 = [
            np.pi/4,    # Theta1
            np.pi/4,    # Theta2
            5.0,        # D3
            -np.pi/4,   # Theta4
            0           # Just the aesthetic joint
        ]
        
        self.graspeDH.plot(q0)
        
    def dynamic_display(self) -> None:
        '''
            
            ### Dynamic display
            Function for dynamic plot of the manipulator in order to vizualize using Peter Corke's Robotic Tool box. In this function we use the trajectory planner of the toolbox.
            - 1º `q0 = [ pi/4, pi/4, 5.0, -pi/4 ]`
            - 2º `q1 = [ 0, pi/6, 5.0, -pi/4 ]`
            - 3º `q2 = [ 0, pi/6, 7.0, -pi/2 ]`
            
            (no input needed)
        '''
        
        # First position
        q_0 = [
            np.pi/4,
            np.pi/4,
            5.0,  
            -np.pi/4
        ]
        
        # Second position
        q_1 = [
            0,
            np.pi/6,
            5.0,
            -np.pi/4
        ]
        
        # Third position
        q_2 = [
            0,
            np.pi/6,
            7.0,
            -np.pi/2
        ]
        
        # Combining the joint positions
        joint_positions = np.vstack((q_0, q_1,q_2))
        # Calculating trajectoris
        traj = self.plan_trajectory_by_joints(joint_positions=joint_positions)
        
        # Plot trajectory
        self.graspeDH.plot(q=traj)  
    
    def plan_trajectory_by_joints(self, joint_positions: np.ndarray, steps: int = 100) -> np.ndarray:
        '''
            ### Plan Trajectory
            This function receives a sequence of joint values and calculates and returns the trajectory passing through all joint positions.
            - input: `joint_positions: np.ndarray (n, 4)`
            - output: `traj: np.ndarray (n, 5)`
            
            (n is the number of objective joint position)
        '''
        if joint_positions.shape[1] != 4:
            raise TypeError("The parameter must be a numpy.ndarray in format (n, 4).")

        # Adding the joint position for the aesthetic joint
        joint_positions = np.hstack((joint_positions, np.zeros((joint_positions.shape[0], 1))))


        # Calculating the trajectory
        traj = []
        for i in range(joint_positions.shape[0] - 1):
            traj_segment = rtb.jtraj(joint_positions[i], joint_positions[i + 1], steps)
            traj.append(traj_segment.q)

        return np.vstack(traj) if traj else np.array([])  # Ensure we return an array, even if empty

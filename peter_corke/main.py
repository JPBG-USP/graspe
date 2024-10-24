#!/home/joao/miniconda3/envs/robotic_env/bin/python
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
from scipy.spatial.transform import Rotation as R

class GraspeManipulator():
    def __init__(self, l1: float, l2: float, l5: float) -> None:
        """
            ## Grasp-E Manipulator Class
            This class defines the RRPR Grasp-E manipulator, integrating the simulation functions from Peter Corke's toolbox, as well as functions for trajectory calculation, forward kinematics, and inverse kinematics.
            
            - **Input:** 
                - `l1`: float
                - `l2`: float
                - `l5`: float
        
            (Refer to the image to check the definitions of the coordinate axes, `images/eixos_coordenados.png`.)
        """
        
        # Variables of the dimentions.
        self.l1_ = l1
        self.l2_ = l2
        self.l5_ = l5
        
        # Defining the robotic manipulator
        self.graspeDH = rtb.DHRobot([
            rtb.RevoluteDH(d=self.l1_, a=0.0, alpha=-np.pi/2.0),
            rtb.RevoluteDH(d=self.l2_, a=0.0, alpha=np.pi/2.0, offset=0),
            rtb.PrismaticDH(theta=0.0, a=0.0, alpha=np.pi/2.0, qlim=[-10, 10]),  
            rtb.RevoluteDH(d=self.l2_, a=0.0, alpha=-np.pi/2),
            rtb.RevoluteDH(d=self.l5_, a=0.0, alpha=0), # Rotation joint for aesthetic purposes only, not sending a real command.
        ],
        name='Grasp-E Manipulator')
        
        
    def ik_calculator(self, end_effector: SE3):
        '''
            ### Inverse Kinematics Calculator:
            Calculates the joint positions given the position and orientation of the end effector.
            - **Input:**
                - `end_effector`: SE3
            
            - Output:
                - `q` : np.ndarray   
        '''
        
        q = np.array([0.0, 0.0, 0.0, 0.0]) # Creating the command vector
        
        ef_T = end_effector.A # Getting the end effector matrix

        # For more details of the IK, check the docs folder in "./docs/inverse_kinematics.pdf"
        q[0] = np.arctan(ef_T[1][3]/ef_T[0][3])
        q[1] = np.arccos((np.sin(q[0])*ef_T[2][1])/(np.cos(q[0])*ef_T[0][1] + np.sin(q[0])*ef_T[1][1]))
        q[3] = q[1] - np.arctan((np.cos(q[0])*ef_T[0][1] + np.sin(q[0])*ef_T[1][1])/(ef_T[2][1])) 
        q[2] = np.cos(q[0])*np.sin(q[1])*ef_T[0][3] + np.sin(q[0])*np.sin(q[1])*ef_T[1][3] + np.cos(q[2])*ef_T[2][3] - self.l1_ * np.cos(q[1]) - self.l5_*np.cos(q[3])
        
        return q

    def static_display(self):
        '''
            ### Static Display
            Function for static plot of the manipulator in order to visualize using Peter Corke's Robotic Tool box.
            - no input needed
            - `q0 = [ pi/4, pi/4, 5.0, -pi/4 ]`
        '''
        
        q = [
            np.pi/4,
            np.pi/4,
            5.0,  # Deslocamento dentro do limite [-10, 10]
            -np.pi/4,
            0
        ]
        
        self.graspeDH.plot(q)
        
    def dynamic_display(self):
        '''
            ### Dynamic display
            Function for dynamic plot od the manipulator in order to vizualize using Peter Corke's Robotic Tool box.
            - no input needed
        '''
        
        # Como essa função é simples, usaremos o planejamento de trajetória do próprio peter corke
        q_0 = [
            np.pi/4,
            np.pi/4,
            5.0,  # Deslocamento dentro do limite [-10, 10]
            -np.pi/4,
            0
        ]
        
        q_1 = [
            0,
            np.pi/6,
            5.0,
            -np.pi/4,
            0
        ]
        
        q_2 = [
            0,
            np.pi/6,
            7.0,
            -np.pi/2,
            0
        ]
        
        
        traj_1 = rtb.jtraj(q_0, q_1, 50)
        traj_2 = rtb.jtraj(q_1, q_2, 100)
        traj = np.vstack((traj_1.q, traj_2.q))
        traj = np.vstack((traj, traj))
        self.graspeDH.plot(q=traj)
        
        
    def plan_trajectory(self, position_1: SE3, position_2: SE3, steps: int = 100):
        '''
            Hard tá, mt hard
        '''
        p1 = position_1.t
        p2 = position_2.t
        
        positions = []
        
        for i in range(steps):
            t = i/steps
            positions.append((1 - t) * p1 + t * p2)
            
    def trajectory():
        #T = SE3(1,2,3) * SE3.Ry(45, unit='rad')
        pass


class GraspCommand():
    def __init__(self, px: float, py: float, pz: float, pich: float, unit : list):
        '''
            ## Deffing a possition commando for grasp-e
        '''
        
    def go_to_position(self, px: float, py: float, pz: float, pich: float, unit : list):
        end_effector_position = 1
        return end_effector_position
        
        
        
def main():
    graspe_robot = GraspeManipulator(l1=10, l2=0.5, l5=2)
    graspe_robot.dynamic_display()
    graspe_robot.ik_calculator()
    
    
if __name__ == '__main__':
    main()
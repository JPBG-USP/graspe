import numpy as np
import roboticstoolbox as rtb

class GraspeKinematics:
    def __init__(self, l1=16.22, l2=10.28, l3=10.28, l4=5):
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.l4 = l4
        
        self.robot = rtb.DHRobot([
            rtb.RevoluteDH(d=self.l1, alpha=np.pi/2),
            rtb.RevoluteDH(a=self.l2),
            rtb.RevoluteDH(a=self.l3),
            rtb.RevoluteDH(a=self.l4)
        ], name='Graspe')
    
    def direct_kinematics(self, q):
        x = np.cos(q[0]) * (self.l4 * np.cos(q[1] + q[2] + q[3]) + self.l3 * np.cos(q[1] + q[2]) + self.l2 * np.cos(q[1]))
        y = np.sin(q[0]) * (self.l4 * np.cos(q[1] + q[2] + q[3]) + self.l3 * np.cos(q[1] + q[2]) + self.l2 * np.cos(q[1]))
        z = self.l4 * np.sin(q[1] + q[2] + q[3]) + self.l3 * np.sin(q[1] + q[2]) + self.l2 * np.sin(q[1]) + self.l1
        
        R = np.array([
            [np.cos(q[0]) * np.cos(q[1] + q[2] + q[3]), -np.cos(q[0]) * np.sin(q[1] + q[2] + q[3]), np.sin(q[0])],
            [np.sin(q[0]) * np.cos(q[1] + q[2] + q[3]), -np.sin(q[0]) * np.sin(q[1] + q[2] + q[3]), -np.cos(q[0])],
            [np.sin(q[1] + q[2] + q[3]), np.cos(q[1] + q[2] + q[3]), 0.0]
        ])
        
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        
        return T
    
#    def inverse_kinematics(self, T):
#        """ Computes inverse kinematics from a transformation matrix """
#        q = np.zeros(4)
#
#        q[0] = np.arctan2(T[1, 3], T[0, 3])
#
#        x_2 = np.sqrt(T[0, 3]**2 + T[1, 3]**2)
#        z_2 = T[2, 3] - l1
#
#        cos_phi = np.sqrt( T[0, 0]**2 + T[1,0]**2 ) / np.sqrt( T[0, 0]**2 + T[1,0]**2 + T[2, 0]**2 )
#        sin_phi = (T[2,0]) / np.sqrt( T[0, 0]**2 + T[1,0]**2 + T[2, 0]**2 )
#
#        phi = np.arctan2(sin_phi, cos_phi)
#
#        p3x = x_2 - l4 * cos_phi
#        p3y = z_2 - l4 * sin_phi
#
#        cos3 = (p3x**2 + p3y**2 - l2**2 - l3**2) / (2 * l2 * l3)
#        sin3 = -np.sqrt(1 - cos3**2)
#
#        q[2] = np.arctan2(sin3, cos3)
#
#        sin2 = ((l2 + l3 * cos3) * p3y - l3 * sin3 * p3x) / (p3x**2 + p3y**2)
#        cos2 = ((l2 + l3 * cos3) * p3x + l3 * sin3 * p3y) / (p3x**2 + p3y**2)
#        q[1] = np.arctan2(sin2, cos2)
#
#        q[3] = phi - q[1] - q[2]
#
#        return q
#
# Example test
graspe = GraspeKinematics()

# Define a test transformation matrix
#q_test = [np.pi/4, np.pi/4, -np.pi/4, -np.pi/4]
#T_test = graspe.direct_kinematics(q_test)  # Compute FK
#q_inv = graspe.inverse_kinematics(T_test)  # Compute IK
#T_recomputed = graspe.direct_kinematics(q_inv)  # Compute FK again
#
## Using Peter Corke's Robotics Toolbox
#T_toolbox = graspe.robot.fkine(q_test)
#q_toolbox = graspe.robot.ikine_LM(T_toolbox).q
#
#print("Initial Joint Values:", q_test)
#print("Forward Kinematics Result (Custom):")
#print(T_test)
#print("Recovered Joint Values (Custom IK):", q_inv)
#print("Recomputed Transformation Matrix (Custom FK):")
#print(T_recomputed)
#print("Forward Kinematics Result (Robotics Toolbox):")
#print(T_toolbox)
#print("Recovered Joint Values (Robotics Toolbox IK):", q_toolbox)

q = [0.0, np.pi/3, -np.pi/2, np.pi/6]
graspe.robot.plot(q, block=True)
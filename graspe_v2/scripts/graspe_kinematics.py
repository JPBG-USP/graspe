import math
import numpy as np

class GraspeKinematics:
    def __init__(self, l1, l2, l3, l4, joint_limits):
        self._l1 = l1
        self._l2 = l2
        self._l3 = l3
        self._l4 = l4
        self.joint_limits = joint_limits

    def inverse_kinematics_cylindrical(self, position):
        """
        Calculate inverse kinematics for cylindrical coordinates.
        
        Args:
            position: List of [theta1, radius, height, phi]
            
        Returns:
            Tuple: (success: bool, joint_states: list)
            joint_states contains [theta1, theta2, theta3, theta4]
        """
        joint_states = [0.0] * 4

        # theta1, rotation along the z axis
        if position[0] > self.joint_limits[0]["max"] or position[0] < self.joint_limits[0]["min"]:
            print("limite junta 1")
            return False, joint_states
        joint_states[0] = position[0]

        # radius distance
        #if position[1] >= (self._l2 + self._l3 + self._l4):
        #    return False, joint_states
        x_2 = position[1]

        # height distance
        #if position[2] >= (self._l2 + self._l3 + self._l4):
        #    return False, joint_states
        z_2 = position[2] - self._l1

        # Angle with the ground plane
        phi = position[3]

        # pose of the end of link 3
        p3x = x_2 - self._l4 * math.cos(phi)
        p3y = z_2 - self._l4 * math.sin(phi)

        # theta3
        cos3 = (p3x*p3x + p3y*p3y - self._l2*self._l2 - self._l3*self._l3) / (2*self._l2*self._l3)
        if (1 - cos3*cos3) < 0:
            return False, joint_states
        sin3 = -math.sqrt(1 - cos3*cos3)  # negative, so elbow points up

        joint_states[2] = math.atan2(sin3, cos3)
        if joint_states[2] > self.joint_limits[2]["max"] or joint_states[2] < self.joint_limits[2]["min"]:
            print("Limite junta 3")
            return False, joint_states

        # theta2
        sin2 = ((self._l2 + self._l3*cos3)*p3y - self._l3*sin3*p3x) / (p3x*p3x + p3y*p3y)
        cos2 = ((self._l2 + self._l3*cos3)*p3x + self._l3*sin3*p3y) / (p3x*p3x + p3y*p3y)

        joint_states[1] = math.atan2(sin2, cos2)
        if joint_states[1] > self.joint_limits[1]["max"] or joint_states[1] < self.joint_limits[1]["min"]:
            print("Linite junta 2")
            return False, joint_states

        # theta4
        joint_states[3] = phi - joint_states[1] - joint_states[2]
        if joint_states[3] > self.joint_limits[3]["max"] or joint_states[3] < self.joint_limits[3]["min"]:
            print("Limite junta 4")
            return False, joint_states

        return True, joint_states
""" This file is form graspe 2"""

from sys import float_repr_style
import roboticstoolbox as rtb
import numpy as np

class GraspeV2():
    """Class for graspe2"""
    
    def __init__(self, l1:float, l2:float, l3:float, l4:float):
        """Create the graspeV2 manipulator, capable of executin task,
        this code was made to work on kinematics

        Args:
            l1 (float): lenght of fist link
            l2 (float): lenght of second link
            l3 (float): lenght of thrid link
            l4 (float): lenght of four link
        """
        
        self._l1:float = l1
        self._l2:float = l2
        self._l3:float = l3
        self._l4:float = l4
        
        self._graspe_dh = rtb.DHRobot(
            [
                rtb.RevoluteDH(d=self._l1, alpha=np.pi/2),
                rtb.RevoluteDH(a=self._l2),
                rtb.RevoluteDH(a=self._l3),
                rtb.RevoluteDH(a=self._l4)
            ],
            name='GraspeV2',
        )
        
    def static_display(self):
        q =[
            np.pi/4,
            np.pi/4,
            -np.pi/4,
            -np.pi/4,
        ]
        self._graspe_dh.plot(q, block=True)
        
    def static_display2(self):
        q =[
            0,
            np.pi/4,
            -np.pi/2,
            np.pi/4,
        ]
        self._graspe_dh.plot(q, block=True)
        

eu = GraspeV2(0.1, 0.5, 0.5, 0.5)

eu.static_display2()

        
    
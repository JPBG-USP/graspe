#!/usr/bin/env python3
import numpy as np
import roboticstoolbox as rtb
from graspeClass import GraspeManipulator

        
def main():
    graspe_robot = GraspeManipulator(l1=10, l2=0.5, l5=2)
    graspe_robot.dynamic_display()
  
if __name__ == '__main__':
    main()
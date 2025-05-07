import roboticstoolbox as rtb
import numpy as np
import serial
import sys
import select
import termios
import tty

from graspe_kinematics import GraspeKinematics

class GraspeView:
    def __init__(self, l1=16.72, l2=10.26, l3=10.26, l4=5):
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.l4 = l4
        
        joint_limits = [
            {
                "max": 1.5707,
                "min": -1.5707
            },
            {
                "max": 1.5707,
                "min": -1.5707
            },
            {
                "max": 0.0,
                "min": -3.1415
            },
            {
                "max": 1.5707,
                "min": -1.5707
            },
        ]
        
        self.kinematics = GraspeKinematics(16.72, 10.26, 10.26, 5, joint_limits)
        
        self.robot = rtb.DHRobot([
            rtb.RevoluteDH(d=self.l1, alpha=np.pi/2),
            rtb.RevoluteDH(a=self.l2),
            rtb.RevoluteDH(a=self.l3),
            rtb.RevoluteDH(a=self.l4)
        ], name='Graspe')
        
    def run(self):
        try:
            with serial.Serial('/dev/ttyACM0', 115200) as ser:
                
                q = [0.0, 0.0, 0.0, 0.0]
                env = self.robot.plot(q=q, limits=[-20, 20, -20, 20, 0.0, 20])
                
                while True:
                    if ser.in_waiting > 0:
                        line = ser.readline().decode('utf-8').strip()
                        print(f"Line: {line}")
                        try:
                            q = [(float(v)-90)*np.pi/180 for v in line.split('/')]
                            print(f"Joint: {q}")
                            self.robot.q = q
                            env.step()
                        except ValueError:
                            print(f"Error to convert line to joint position")
                    else:
                        print("Nothing to read")
                
        except KeyboardInterrupt:
            print("Process ended by user")
            
        except serial.SerialException as e:
            print(f"Error in serial port: {e}")
            
    def test_plot(self):
        try:
            q = [0.0, 0.0, 0.0, 0.0]
            env = self.robot.plot(q=q, limits=[-20, 20, -20, 20, 0.0, 20])
            
            while True:
                q[0] +=0.2
                self.robot.q = q
                env.step()
                
        except KeyboardInterrupt:
            print("Process ended by user")
            
    def sim_view(self):
        try:
            pos = [0.0, 15.0, 16.72, 0.0]
            q = [0.0, 0.0, 0.0, 0.0]
            _, q = self.kinematics.inverse_kinematics_cylindrical(pos)
            env = self.robot.plot(q=q, limits=[-20, 20, -20, 20, 0.0, 20])
            
            step_size = 0.1
            
            # Save terminal settings
            old_settings = termios.tcgetattr(sys.stdin)
            try:
                # Set terminal to raw mode for single-character input
                tty.setcbreak(sys.stdin.fileno())
                
                print("Control the arm with the following keys:")
                print("a/d: Decrease/Increase theta1")
                print("w/s: Increase/Decrease radius")
                print("i/k: Increase/Decrease height")
                print("l/j: Increase/Decrease phi")
                print("q: Quit")
                
                while True:
                    # Check for terminal input without blocking
                    r, _, _ = select.select([sys.stdin], [], [], 0.1)
                    if r:
                        key = sys.stdin.read(1).lower()
                        if key == 'q':
                            break
                        elif key == 'a':
                            pos[0] -= step_size  # Decrease theta1
                        elif key == 'd':
                            pos[0] += step_size  # Increase theta1
                        elif key == 'w':
                            pos[1] += 2*step_size  # Increase radius
                        elif key == 's':
                            pos[1] -= 2*step_size  # Decrease radius
                        elif key == 'i':
                            pos[2] += step_size  # Increase height
                        elif key == 'k':
                            pos[2] -= step_size  # Decrease height
                        elif key == 'l':
                            pos[3] += step_size  # Increase phi
                        elif key == 'j':
                            pos[3] -= step_size  # Decrease phi
                    
                        # Compute inverse kinematics
                        possible, qnew = self.kinematics.inverse_kinematics_cylindrical(pos)
                        if possible:
                            q = qnew
                            print(q)
                    self.robot.q = q
                    env.step()
            
            finally:
                # Restore terminal settings
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        
        except KeyboardInterrupt:
            print("Process ended by user")
            
robot = GraspeView()
robot.sim_view()
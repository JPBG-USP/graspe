import roboticstoolbox as rtb
import numpy as np
import serial

class GraspeView:
    def __init__(self, l1=5, l2=10.26, l3=10.26, l4=10.26):
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
        
    def run(self):
        try:
            with serial.Serial('/dev/ttyUSB0', 115200) as ser:
                
                q = [0.0, 0.0, 0.0, 0.0]
                env = self.robot.plot(q=q, limits=[-20, 20, -20, 20, 0.0, 20])
                
                while True:
                    if ser.in_waiting > 0:
                        line = ser.readline().decode('utf-8').strip()
                        try:
                            q = [float(v) for v in line.split('/')]
                            self.robot.q = q
                            env.step()
                        except ValueError:
                            print(f"Error to convert line to joint position")
                
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
            
            
robot = GraspeView()
robot.run()
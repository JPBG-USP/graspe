#!/home/joao/miniconda3/envs/robotic_env/bin/python
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3

class graspe_manipulator():
    def __init__(self, l1_: float, l2_: float, l5_: float) -> None:
        ''' 
        Define o manipulador grasp-e
        '''
        
        self.l1 = l1_
        self.l2 = l2_
        self.l5 = l5_
        
        # Definindo manipulador
        self.graspeDH = rtb.DHRobot([
            rtb.RevoluteDH(d=self.l1, a=0.0, alpha=-np.pi/2.0),
            rtb.RevoluteDH(d=self.l2, a=0.0, alpha=np.pi/2.0, offset=0),
            rtb.PrismaticDH(theta=0.0, a=0.0, alpha=np.pi/2.0, qlim=[-10, 10]),  
            rtb.RevoluteDH(d=self.l2, a=0.0, alpha=-np.pi/2),
            rtb.RevoluteDH(d=self.l5, a=0.0, alpha=0), # Junta só de translação, não utilizamos como junta real, só estético
        ],
        name='Manipulador Graspe')
        
        
    def inverse_kinematics(self, end_effector: np.array):
        '''
            Calculador da cinemática inversa
            
            Calcula a posição da junta dado a posição e orientação do end effector
            input - np.array 4x4 (end effector position)
            output - np.array 1x4 (Joints states)
        '''
        q = np.array([0.0, 0.0, 0.0, 0.0])
        
        q[0] = np.arctan(end_effector[1][3]/end_effector[0][3])
        q[1] = np.arccos((np.sin(q[0])*end_effector[2][1])/(np.cos(q[0])*end_effector[0][1] + np.sin(q[0])*end_effector[1][1]))
        q[3] = q[1] - np.arctan((np.cos(q[0])*end_effector[0][1] + np.sin(q[0])*end_effector[1][1])/(end_effector[2][1])) 
        q[2] = np.cos(q[0])*np.sin(q[1])*end_effector[0][3] + np.sin(q[0])*np.sin(q[1])*end_effector[1][3] + np.cos(q[2])*end_effector[2][3] - self.l1 * np.cos(q[1]) - self.l5*np.cos(q[3])
        
        return q

    def static_display(self):
        '''
            Executa um plot simples do Grasp-e
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
            Executa um plot dinâmico com uma trajetória simples padrão
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

def graspe_func(l1: float, l2: float, l5: float):
    ''' 
    Define o manipulador Grasp-e
    '''
    
    # Definindo manipulador
    graspeDH = rtb.DHRobot([
        rtb.RevoluteDH(d=l1, a=0.0, alpha=-np.pi/2.0),
        rtb.RevoluteDH(d=l2, a=0.0, alpha=np.pi/2.0, offset=0),
        rtb.PrismaticDH(theta=0.0, a=0.0, alpha=np.pi/2.0, qlim=[-10, 10]),  
        rtb.RevoluteDH(d=l2, a=0.0, alpha=-np.pi/2),
        rtb.RevoluteDH(d=l5, a=0.0, alpha=0),
    ],
    name='Manipulador Graspe')

    q = [
        np.pi/4,
        np.pi/4,
        5.0,  # Deslocamento dentro do limite [-10, 10]
        -np.pi/4,
        0
    ]  # Ângulos em radianos para juntas revolutas e deslocamento prismático

    # Cinemática direta para encontrar a pose final
    T = graspeDH.fkine(q)

    # Exibir a pose final
    print("Pose final:")
    print(T)

    # Visualizar o manipulador na configuração definida
    graspeDH.plot(q)

if __name__ == '__main__':
    graspe_robot = graspe_manipulator(l1_=10, l2_=0.5, l5_=2)
    graspe_robot.dynamic_display()

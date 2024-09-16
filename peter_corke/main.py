#!/home/joao/miniconda3/envs/robotic_env/bin/python
import roboticstoolbox as rtb
import numpy as np

print('Robot Toolbox version: '+ rtb.__version__)

    
def graspe_func():
    
    
    graspeDH = rtb.DHRobot([
        rtb.RevoluteDH(d=10.0, a=0.0, alpha=-np.pi/2.0), 
        rtb.RevoluteDH(d=0.0, a=0.0, alpha=np.pi/2.0, offset=-np.pi/2.0), 
        rtb.PrismaticDH(theta=-np.pi/2.0,a=0.0, alpha=0.0), 
        rtb.RevoluteDH(d=0.0, a=0.0, alpha=-np.pi/2),
        rtb.RevoluteDH(d=0.0, a=0.0, alpha=np.pi/2),
        rtb.RevoluteDH(d=5.0, a=0.0, alpha=0.0)
        ],
        name='Manipulador Graspe')
    
    print(graspeDH)
    
    q = [
        np.pi/4, 
        np.pi/4, 
        10.0, 
        np.pi/4,
        np.pi/4,
        np.pi/4
        ] # Ângulos em radianos para juntas revolutas e deslocamento prismático

    # Cinemática direta para encontrar a pose final
    T = graspeDH.fkine(q)

    # Exibir a pose final
    print("Pose final:")
    print(T)

    # Plotar o manipulador na configuração definida
    graspeDH.plot(q, block=True, jointaxes=True, backend='pyplot')

if __name__ == '__main__':
    graspe_func()
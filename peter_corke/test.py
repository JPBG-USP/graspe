#!/home/joao/miniconda3/envs/robotic_env/bin/python
import roboticstoolbox as rtb
import numpy as np

print('Robot Toolbox version: ' + rtb.__version__)

def graspe_func():
    d1 = 5.0
    d2 = 0.5

    # Definindo o manipulador com uma junta prismática com limites [-10, 10]
    graspeDH = rtb.DHRobot([
        rtb.RevoluteDH(d=d1, a=0.0, alpha=-np.pi/2.0),
        rtb.RevoluteDH(d=d2, a=0.0, alpha=np.pi/2.0, offset=0),
        rtb.PrismaticDH(theta=0.0, a=0.0, alpha=np.pi/2.0, qlim=[-10, 10]),  # Limites da junta prismática
        rtb.RevoluteDH(d=d2, a=0.0, alpha=-np.pi/2),
        rtb.RevoluteDH(d=0.5, a=0.0, alpha=0),
    ],
        name='Manipulador Graspe')

    print(graspeDH)

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
    graspe_func()

#ifndef _GRASPE_KINEMATICS_
#define _GRASPE_KINEMATCIS_

#include<map>
#include<string.h>
#include<vector>
#include<math.h>

/**
 * @brief Type of variable to describe a rigid body in 3D space
 */
struct SE3
{
    std::vector<std::vector<float>> transform;      // Matriz de transformação homogênea 4x4
    std::vector<std::vector<float>> rotation_matrix; // Matriz de rotação 3x3
    std::vector<float> pos;  // Vetor de posição (x, y, z)
    std::vector<float> rpy;  // Ângulos de Euler (Roll, Pitch, Yaw)
    std::vector<float> quat; // Quaternion (x, y, z, w)

    // Construtor para inicializar os vetores com tamanho adequado
    SE3() : transform(4, std::vector<float>(4, 0)), 
            rotation_matrix(3, std::vector<float>(3, 0)), 
            pos(3, 0), rpy(3, 0), quat(4, 0) {}
};

class GraspeKinematics
{
private:
    float _l1 = 10.0;
    float _l2 = 10.0;
    float _l3 = 10.0;
    float _l4 = 10.0;

public:
    GraspeKinematics();
    SE3 directKinematics(std::vector<float> q);

};



#endif 
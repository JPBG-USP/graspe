#ifndef _GRASPE_KINEMATICS_
#define _GRASPE_KINEMATICS_

#include<map>
#include<string.h>
#include<vector>
#include<math.h>
#include<iostream>
#include<cstdlib>  


namespace graspe
{

    // Joint state variable for graspe (theta1, theta2, theta3, theta4)
    typedef float JointStates[4];
    typedef float CylindricalCoord[4]; 

} // namespace graspe



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


// position command theta1, r e z


class GraspeKinematics
{
private:
    float _l1;
    float _l2;
    float _l3;
    float _l4;

    std::vector<std::map<std::string, float>> joint_limits;

public:
    GraspeKinematics(float l1, float l2, float l3, float l4, std::vector<std::map<std::string, float>> joint_limits);
    SE3 directKinematics(graspe::JointStates q);
    std::vector<float> inverseKinematics(SE3 position);
    bool directKinematicsCylindrical(graspe::JointStates joint_states, graspe::CylindricalCoord& position);
    bool inverseKinematicsCylindrical(graspe::CylindricalCoord position, graspe::JointStates& joint_states);
};


#endif 
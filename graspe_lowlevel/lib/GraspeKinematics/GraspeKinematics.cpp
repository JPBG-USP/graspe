#include "GraspeKinematics.h"


GraspeKinematics::GraspeKinematics(){
    this->_l1=10;
};

/**
 * @brief This function retursn pararaprerre
 * 
 * @param q Joint states
 */
SE3 GraspeKinematics::directKinematics(std::vector<float> q){
    SE3 end_effector;

    // calculating x, y and z
    end_effector.pos[0] = cos(q[1]) * (_l4*cos(q[2]+q[3]+q[4]) + _l3*cos(q[2]+q[3]) + _l2*cos(q[2]));
    end_effector.pos[1] = sin(q[1]) * (_l4*cos(q[2]+q[3]+q[4]) + _l3*cos(q[2]+q[3]) + _l2*cos(q[2]));
    end_effector.pos[2] = _l4*sin(q[2]+q[3]+q[4]) + _l3*sin(q[2]+q[3]) + _l2*sin(q[2]) + _l1;

    // rotation matrix
    // normal (x)
    end_effector.rotation_matrix[0][0] = cos(q[1]) * cos(q[2]+q[3]+q[4]);
    end_effector.rotation_matrix[1][0] = sin(q[1]) * cos(q[2]+q[3]+q[4]);
    end_effector.rotation_matrix[2][0] = sin(q[2]+q[3]+q[4]);
    // slide (y)
    end_effector.rotation_matrix[0][1] = -cos(q[1]) * sin(q[2]+q[3]+q[4]);
    end_effector.rotation_matrix[1][1] = -sin(q[1]) * sin(q[2]+q[3]+q[4]);
    end_effector.rotation_matrix[2][1] = cos(q[2]+q[3]+q[4]);
    // approach (z)
    end_effector.rotation_matrix[0][2] = sin(q[1]);
    end_effector.rotation_matrix[1][2] = -cos(q[1]);
    end_effector.rotation_matrix[2][2] = 0.0;

    // transform
    for (int i = 0; i < 3; i++)
    {
        end_effector.transform[0][i] = end_effector.rotation_matrix[0][i];
        end_effector.transform[1][i] = end_effector.rotation_matrix[1][i];
        end_effector.transform[2][i] = end_effector.rotation_matrix[2][i];
    }

    end_effector.transform[0][3] = end_effector.pos[0]; 
    end_effector.transform[1][3] = end_effector.pos[1];
    end_effector.transform[2][3] = end_effector.pos[2];

    end_effector.transform[3] = {0.0, 0.0, 0.0, 1.0};

    // TODO: Need to add quat and RPY data

    return end_effector;
}
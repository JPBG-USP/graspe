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

    // add here the rpy and complete end_effector variable

    return end_effector;
}

std::vector<float> GraspeKinematics::inverseKinematics(SE3 end_effector){
    std::vector<float> joint_states = {0.0, 0.0, 0.0, 0.0};

    // theta1
    // theta1 = arctan(py/px)

    joint_states[0] = atan2(end_effector.transform[1][3], end_effector.transform[0][3]); 

    // Now on os more complex, I sugest to read the docs 
    
    // theta3
    // new coordinate system to use the solution of the 3 link plannar manipulation
    float x_2 = sqrt(end_effector.transform[0][3]*end_effector.transform[0][3] + end_effector.transform[1][3]*end_effector.transform[1][3]);
    float z_2 = end_effector.transform[2][3] - this->_l1;

    float cos_phi = sqrt(end_effector.transform[0][0]*end_effector.transform[0][0] + end_effector.transform[1][0]*end_effector.transform[1][0]) / sqrt(end_effector.transform[0][0]*end_effector.transform[0][0] + end_effector.transform[1][0]*end_effector.transform[1][0] + end_effector.transform[2][0]*end_effector.transform[2][0]);
    float sin_phi = end_effector.transform[2][0] / sqrt(end_effector.transform[0][0]*end_effector.transform[0][0] + end_effector.transform[1][0]*end_effector.transform[1][0] + end_effector.transform[2][0]*end_effector.transform[2][0]);

    // Angle with the ground plane
    float phi = atan2(sin_phi, cos_phi);

    // pose of the 3 joint
    float p3x = x_2 - this->_l4 * cos_phi;
    float p3y = z_2 - this->_l4 * sin_phi;

    float cos3 = (p3x*p3x +p3y*p3y - _l2*_l2 - _l3*_l3) / (2*_l2*_l3);
    float sin3 = - sqrt(1 - cos3*cos3); // negative, só elbow is point up

    joint_states[2] = atan2(sin3, cos3);
    
    // theta2
    float sin2 = ((_l2 + _l3*cos3)*p3y - _l3*sin3*p3x) / (p3x*p3x +p3y*p3y);
    float cos2 = ((_l2 + _l3*cos3)*p3x + _l3*sin3*p3y) / (p3x*p3x +p3y*p3y);
    joint_states[1] = atan2(sin2, cos2);

    // theta4
    joint_states[3] = phi - joint_states[1] - joint_states[2];

    return joint_states;
}
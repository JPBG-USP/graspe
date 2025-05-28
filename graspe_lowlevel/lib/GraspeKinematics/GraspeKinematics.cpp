#include "GraspeKinematics.h"

GraspeKinematics::GraspeKinematics(float l1, float l2, float l3, float l4, std::vector<std::map<std::string, float>> joint_limits){

    // Graspe dimentions
    _l1 = l1;
    _l2 = l2;
    _l3 = l3;
    _l4 = l4;

    // Joint limits
    this->joint_limits = joint_limits;
};


/**
 * @brief This function retursn pararaprerre
 * 
 * @param q Joint states
 */
SE3 GraspeKinematics::directKinematics(graspe::JointStates q){
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


/**
 * @brief Inverse Kinematics on cartesian system
 * 
 * @param end_effector This is the varible cabable of define a position on a 3D space
 */
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


/**
 * @brief Direct Kinematics on cylindrical coordinates. Returns ``true`` if a solution is found.
 * 
 * @param joint_states
 * @param position
 */
bool GraspeKinematics::directKinematicsCylindrical(graspe::JointStates joint_states, graspe::CylindricalCoord& position){
    graspe::CylindricalCoord new_position;

    // theta
    new_position[0] = joint_states[0];

    // r 
    new_position[1] = _l1 * sin(joint_states[1]) + _l2 * sin(joint_states[1] + joint_states[2]) + _l3 * sin(joint_states[1] + joint_states[2] + joint_states[3]);;
    
    // z
    new_position[2] = _l1 * cos(joint_states[1]) + _l2 * cos(joint_states[1] + joint_states[2]) + _l3 * cos(joint_states[1] + joint_states[2] + joint_states[3]);
    
    // phi
    new_position[3] = joint_states[3] + joint_states[2] + joint_states[1];

    if ( (joint_states[0] > joint_limits[0]["max"]) || (joint_states[0] < joint_limits[0]["min"]) ) {return false;} // Joint1 limits
    if ( (joint_states[1] > joint_limits[1]["max"]) || (joint_states[1] < joint_limits[1]["min"]) ) {return false;} // Joint2 limits
    if ( (joint_states[2] > joint_limits[2]["max"]) || (joint_states[2] < joint_limits[2]["min"]) ) {return false;} // Joint3 limits
    if ( (joint_states[3] > joint_limits[3]["max"]) || (joint_states[3] < joint_limits[3]["min"]) ) {return false;} // Joint14 limits

    position[0] = new_position[0];
    position[1] = new_position[1];
    position[2] = new_position[2];
    position[3] = new_position[3];
    return true;
};


/**
 * @brief Inverse Kinematics on cilindrical coordinates. Returns ``true`` if a solution is found.
 * 
 * @param position The vector of {theta1, r, z, phi} position on cilindrical coordinates
 * @param joint_states A reference to the joint state variable where the position will be stored
 */
bool GraspeKinematics::inverseKinematicsCylindrical(graspe::CylindricalCoord position, graspe::JointStates& joint_states){
    
    graspe::JointStates new_joint_states;

    // theta1, rotation along the z axis
    if ( (position[0] > joint_limits[0]["max"]) || (position[0] < joint_limits[0]["min"]) ) {return false;} // Joint1 limits
    new_joint_states[0] = position[0];
    
    // radius distance
    float x_2 = position[1];

    // height distance
    float z_2 = position[2] - this->_l1;

    // Angle with the ground plane
    float phi = position[3];

    // pose of the end of link 3
    float p3x = x_2 - this->_l4 * cos(phi);
    float p3y = z_2 - this->_l4 * sin(phi);

    // theta3
    float cos3 = (p3x*p3x +p3y*p3y - _l2*_l2 - _l3*_l3) / (2*_l2*_l3);

    if ((1- cos3*cos3) < 0){return false;}
    float sin3 = - sqrt(1 - cos3*cos3); // negative, so elbow is point up

    new_joint_states[2] = atan2(sin3, cos3);
    if ( (new_joint_states[2] > joint_limits[2]["max"]) || (new_joint_states[2] < joint_limits[2]["min"]) ) {return false;} // joint3 limits
    
    // theta2
    float sin2 = ((_l2 + _l3*cos3)*p3y - _l3*sin3*p3x) / (p3x*p3x +p3y*p3y);
    float cos2 = ((_l2 + _l3*cos3)*p3x + _l3*sin3*p3y) / (p3x*p3x +p3y*p3y);

    new_joint_states[1] = atan2(sin2, cos2);
    if ( (new_joint_states[1] > joint_limits[1]["max"]) || (new_joint_states[1] < joint_limits[1]["min"]) ) {return false;} // joint2 limits

    // theta4
    new_joint_states[3] = phi - new_joint_states[1] - new_joint_states[2];
    if ( (new_joint_states[3] > joint_limits[3]["max"]) || (new_joint_states[3] < joint_limits[3]["min"]) ) {return false;} // joint4 limits

    joint_states[0] = new_joint_states[0];
    joint_states[1] = new_joint_states[1];
    joint_states[2] = new_joint_states[2];
    joint_states[3] = new_joint_states[3];

    return true;
}
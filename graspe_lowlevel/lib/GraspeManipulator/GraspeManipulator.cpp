#include "GraspeManipulator.h"

namespace graspe
{
    // Joint Limits
    std::vector<std::map<std::string, float>> JointLimits = {
        { {"max", M_PI/2}, {"min", -M_PI/2} },
        { {"max", M_PI/2}, {"min", -0.57} },
        { {"max", 0}, {"min", -2.3} },
        { {"max", M_PI/2}, {"min", -M_PI/2} }
    };

    // Robot dimentions cm
    float l1 = 16.72;
    float l2 = 10.26;
    float l3 = 10.26;
    float l4 = 5.0;
} 

GraspeManipulator::GraspeManipulator()
    : Kinematics(graspe::l1, graspe::l2, graspe::l3, graspe::l4, graspe::JointLimits),
      joint1(32, graspe::JointLimits[0]),
      joint2(33, graspe::JointLimits[1]),
      joint3(25, graspe::JointLimits[2],180),
      joint4(26, graspe::JointLimits[3])
{
    this->reset_manipulator();
}

bool GraspeManipulator::set_pose(graspe::CylindricalCoord delta_pos){

    graspe::CylindricalCoord new_endeffector_pose;

    new_endeffector_pose[0] = endeffector_pose[0] + delta_pos[0];
    new_endeffector_pose[1] = endeffector_pose[1] + delta_pos[1];
    new_endeffector_pose[2] = endeffector_pose[2] + delta_pos[2];
    new_endeffector_pose[3] = endeffector_pose[3] + delta_pos[3];

    if(Kinematics.inverseKinematicsCylindrical(new_endeffector_pose, joint_state)){

        // new end effector position
        endeffector_pose[0] += delta_pos[0];
        endeffector_pose[1] += delta_pos[1];
        endeffector_pose[2] += delta_pos[2];
        endeffector_pose[3] += delta_pos[3];

        /// Set angle to the joints
        joint1.set_angle(joint_state[0]);
        joint2.set_angle(joint_state[1]);
        joint3.set_angle(joint_state[2]);
        joint4.set_angle(joint_state[3]);

        return true;
    };    
    return false;
}

void GraspeManipulator::reset_manipulator(){
    
    joint_state[0] = 0.0;
    joint_state[1] = 0.5;
    joint_state[2] = -1.0;
    joint_state[3] = 0.5;

    endeffector_pose[0] = 0.0;
    endeffector_pose[1] = 23.0;
    endeffector_pose[2] = 16.72;
    endeffector_pose[3] = 0.0;

    /// Set angle to the joints
    joint1.set_angle(joint_state[0]);
    joint2.set_angle(joint_state[1]);
    joint3.set_angle(joint_state[2]);
    joint4.set_angle(joint_state[3]);
}
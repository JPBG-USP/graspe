#include "GraspeManipulator.h"

GraspeManipulator::GraspeManipulator(){
    return;
}

bool GraspeManipulator::set_pose(graspe::CylindricalCoord delta_pos){

    graspe::CylindricalCoord new_endeffector_pose;

    new_endeffector_pose[0] = endeffector_pose[0] + delta_pos[0];
    new_endeffector_pose[1] = endeffector_pose[1] + delta_pos[1];
    new_endeffector_pose[2] = endeffector_pose[2] + delta_pos[2];
    new_endeffector_pose[3] = endeffector_pose[3] + delta_pos[3];

    if(Kinematics.inverseKinematicsCylindrical(new_endeffector_pose, &joint_state)){

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
    return false
}
#ifndef _GRASPE_MANIPULATOR_
#define _GRASPE_MANIPULATOR_

#include "GraspeKinematics.h"
#include "GraspeJoints.h"

namespace graspe
{
    // Joint Limits
    std::vector<std::map<std::string, float>> JointLimits = {
        { {"max", 0.0f}, {"min", 0.0f} },
        { {"max", 0.0f}, {"min", 0.0f} },
        { {"max", 0.0f}, {"min", 0.0f} },
        { {"max", 0.0f}, {"min", 0.0f} }
    };

    // Robot dimentions cm
    float l1 = 16.22;
    float l2 = 10.26;
    float l3 = 10.26;
    float l4 = 5.0;
} 

class GraspeManipulator
{
private:
    // Kinematics
    GraspeManipulator Kinematics(
        graspe::l1, graspe::l2, graspe::l3, graspe::l4, graspe::JointLimits
    );

    // Manipulator Joints
    GraspeJoints joint1(32, graspe::JointLimits[0]);
    GraspeJoints joint2(33, graspe::JointLimits[1]);
    GraspeJoints joint3(25,graspe::JointLimits[2]);
    GraspeJoints joint4(26, graspe::JointLimits[3]);

    // graspe state
    graspe::JointStates joint_state;
    graspe::CylindricalCoord endeffector_pose;

public:
    GraspeManipulator();
    bool set_pose(graspe::CylindricalCoord delta_pos);
};

#endif
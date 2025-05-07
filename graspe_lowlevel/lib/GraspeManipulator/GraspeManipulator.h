#ifndef _GRASPE_MANIPULATOR_
#define _GRASPE_MANIPULATOR_

#include "GraspeKinematics.h"
#include "GraspeJoints.h"

namespace graspe
{
    // Joint Limits
    extern std::vector<std::map<std::string, float>> JointLimits;

    // Robot dimentions cm
    extern float l1;
    extern float l2;
    extern float l3;
    extern float l4;
} 

class GraspeManipulator
{
private:
    // Kinematics
    GraspeKinematics Kinematics;

    // graspe state
    graspe::JointStates joint_state;

public:
    // Manipulator Joints
    GraspeJoints joint1;
    GraspeJoints joint2;
    GraspeJoints joint3;
    GraspeJoints joint4;

    graspe::CylindricalCoord endeffector_pose;
    
    GraspeManipulator();
    bool set_pose(graspe::CylindricalCoord delta_pos);
    void reset_manipulator();
};

#endif
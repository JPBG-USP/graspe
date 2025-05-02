#include "GraspeJoints.h"

/**
 * @brief GraspeJoints base constructor, all parameters are set to 0
 * 
 */
GraspeJoints::GraspeJoints(){
    angle_initial = 0.0;
    angle_current = 0.0;
    angle_limits = {{"max",0.0},{"min",0.0}};
    speed_modifier = 0.0;
    pin = -1;

};

/**
 * @brief GraspeJoints constructor with initial values
 * 
 * @param input_initial Initial angle for the Joint
 * @param input_limits Limits for the Joint, must have the form {{"max",float value},{"min",float value}}
 * @param input_modifier Speed modifier for the joint
 * @param input_pin Which pin the joint servo is connected to
 */
GraspeJoints::GraspeJoints(const float& input_initial, const std::map<std::string, float>& input_limits,const float& input_modifier, const int& input_pin){
    angle_initial = input_initial;
    angle_current = angle_initial;
    angle_limits = input_limits;
    speed_modifier = input_modifier;
    pin = input_pin;

};



float GraspeJoints::get_angle_initial(){
    return angle_initial;
}

float GraspeJoints::get_angle_current(){
    return angle_current;
}

std::map<std::string,float> GraspeJoints::get_angle_limits(){
    return angle_limits;
}

float GraspeJoints::get_speed_modifier(){
    return speed_modifier;
}

int GraspeJoints::get_pin(){
    return pin;
}


void GraspeJoints::set_angle_initial(const float& input_initial) {
    angle_initial = input_initial;
}

void GraspeJoints::set_angle_current(const float& input_current) {
    angle_current = input_current;
}

void GraspeJoints::set_angle_limits(const std::map<std::string, float>& limits) {
    angle_limits = limits;
}

void GraspeJoints::set_speed_modifier(const float& input_modifier) {
    speed_modifier = input_modifier;
}

void GraspeJoints::set_pin(const int& input_pin) {
    pin = input_pin;
}


/**
 * @brief Checks if the current angle is outside the permited boundaries and keeps it in the maximum or minimal values
 * 
 */
float GraspeJoints::check_limits(float input_angle){
    if(input_angle >= angle_limits["max"]){
        return angle_limits["max"];
    }else if(input_angle <= angle_limits["min"]){
        return angle_limits["min"];
    }
    return input_angle;
}





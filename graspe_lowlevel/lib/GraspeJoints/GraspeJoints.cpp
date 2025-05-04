#include "GraspeJoints.h"
/**
 * @brief GraspeJoints constructor with initial values
 * @param input_initial Initial angle for the Joint
 * @param input_limits Limits for the Joint, must have the form {{"max",float value},{"min",float value}}
 * @param input_modifier Speed modifier for the joint
 * @param input_pin Which pin the joint servo is connected to
 */
GraspeJoints::GraspeJoints(const int& input_pin, const std::map<std::string, float>& input_limits, const float& input_initial,const float& input_modifier){
    angle_initial = input_initial;
    angle_current = angle_initial;
    angle_limits = input_limits;
    speed_modifier = input_modifier;
    pin = input_pin;
    servo.setPeriodHertz(50);
    servo.attach(pin, 700, 2350);
    servo.write(int(angle_current));
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

void GraspeJoints::set_angle(float input_current) {
    angle_current = check_limits(input_current);
    angle_current *= 180/M_PI;
    angle_current+=90;
    servo.write(int(angle_current));
}

void GraspeJoints::set_speed_modifier(const float& input_modifier) {
    speed_modifier = input_modifier;
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

void GraspeJoints::reset_joint(){
    angle_current = angle_initial;
    servo.write(int(angle_current));
}
#ifndef _GRASPE_JOINTS_
#define _GRASPE_JOINTS_

#include<map>
#include<string.h>
#include<vector>
#include<math.h>
#include <iostream>
#include <cstdlib>  
#include <ESP32Servo.h>

class GraspeJoints
{
private:
    float angle_initial;
    float angle_current;
    std::map<std::string,float> angle_limits;
    float speed_modifier;
    int pin;
    Servo servo;

public:
    //Constructors and get_value methods
    GraspeJoints(const float& input_initial = 0.0, const std::map<std::string, float>& input_limits = {{"max",0.0},{"min",0.0}},const float& input_modifier = 1.0, const int& input_pin = -1);
    float get_angle_initial();
    float get_angle_current();
    std::map<std::string,float> get_angle_limits();
    float get_speed_modifier();
    int get_pin();

    //Set_value methods
    void set_angle(const float& input_current);
    void set_speed_modifier(const float& input_modifier);
    
    // Miscelaneous methods

    float check_limits(float input_angle);
    void GraspeJoints::reset_joint();
};

#endif 
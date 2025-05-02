#ifndef _GRASPE_JOINTS_
#define _GRASPE_JOINTS_

#include<map>
#include<string.h>
#include<vector>
#include<math.h>
#include <iostream>
#include <cstdlib>  



class GraspeJoints
{
private:
    float angle_initial;
    float angle_current;
    std::map<std::string,float> angle_limits;
    float speed_modifier;
    int pin;

public:
    //Constructors and get_value methods
    GraspeJoints();
    GraspeJoints(const float& input_initial, const std::map<std::string, float>& input_limits,const float& input_modifier, const int& input_pin);
    float get_angle_initial();
    float get_angle_current();
    std::map<std::string,float> get_angle_limits();
    float get_speed_modifier();
    int get_pin();

    //Set_value methods
    void set_angle_initial(const float& input_inital);
    void set_angle_current(const float& input_current);
    void set_angle_limits(const std::map<std::string, float>& limits);
    void set_speed_modifier(const float& input_modifier);
    void set_pin(const int& input_pin);
    
    //


    float check_limits(float input_angle);
    
};



#endif 
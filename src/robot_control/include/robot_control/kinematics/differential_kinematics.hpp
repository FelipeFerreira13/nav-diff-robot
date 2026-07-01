#ifndef DIFF_KINEMATICS
#define DIFF_KINEMATICS

#include <cmath>

#include "utils/UtilsGeneral.h"

#include "geometry_msgs/msg/twist.hpp"

struct DifferentialSpeeds {
    double front_left;  
    double front_right; 
    double rear_left; 
    double rear_right; 

};

class DifferentialKimematics {
public:
    DifferentialKimematics(){}
    DifferentialKimematics(double wheel_radius, double robot_radius, double max_motor_speed);

    Twist2D forward(double left_vel, double right_vel);
    DifferentialSpeeds inverse(Twist2D twist);

    double WheelSpeed( int encoder, double time );

    int fl_ = 0;
    int fr_ = 0;
    int rl_ = 0;
    int rr_ = 1;

    double max_motor_speed_ = 0.7;

    double R = 0.0; // wheel radius
    double L = 0.4; // distance from robot center to wheel
    double ticks_per_rotation_ = 1464;

private:

};



#endif // DIFF_KINEMATICS


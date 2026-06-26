#ifndef DIFF_KINEMATICS
#define DIFF_KINEMATICS

#include <cmath>

#include "utils/UtilsGeneral.h"

#include "geometry_msgs/msg/twist.hpp"

struct DifferentialSpeeds {
    double left;  
    double right; 
};

class DifferentialKimematics {
public:
    DifferentialKimematics(){}
    DifferentialKimematics(double wheel_radius, double robot_radius, double max_motor_speed);

    Twist2D forward(double left_vel, double right_vel);
    DifferentialSpeeds inverse(Twist2D twist);

    double WheelSpeed( int encoder, double time );

    int l_ = 0;
    int r_ = 1;

    double max_motor_speed_ = 0.7;

    double R = 0.0; // wheel radius
    double L = 0.4; // distance from robot center to wheel
    double ticks_per_rotation_ = 1464;

private:

};



#endif // DIFF_KINEMATICS


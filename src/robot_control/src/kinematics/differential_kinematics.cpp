#include "robot_control/kinematics/differential_kinematics.hpp"

DifferentialKimematics::DifferentialKimematics(double wheel_radius, double robot_radius, double max_motor_speed) 
    : R(wheel_radius), L(robot_radius), max_motor_speed_(max_motor_speed) {}

Twist2D DifferentialKimematics::forward(double left_vel, double right_vel){

    Twist2D twist;

    twist.vx    = (( right_vel + left_vel ) / 2);  // [cm/s]
    twist.vy    = 0;
    twist.omega = (( right_vel - left_vel ) / L);  // [rad/s]

    return twist;
}

DifferentialSpeeds DifferentialKimematics::inverse(Twist2D twist){

    DifferentialSpeeds wheels;

    wheels.front_right = ((2.0 * twist.vx) + (twist.omega * L * 2.0)) / 2.0;   // [m/s]
    wheels.rear_right  = wheels.front_right;
    wheels.front_left  = ((2.0 * twist.vx) - (twist.omega * L * 2.0)) / 2.0;   // [m/s]
    wheels.rear_left   = wheels.front_left;

    return wheels;
}

double DifferentialKimematics::WheelSpeed( int encoder, double time ){
    double speed  = ((2 * M_PI * R * encoder) / (ticks_per_rotation_ * time));   // [m/s]

    if ( time == 0 ) { speed  = 0; }

    return speed;
}
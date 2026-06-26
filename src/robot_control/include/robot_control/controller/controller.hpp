#ifndef CONTROLLER
#define CONTROLLER

#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include <control_toolbox/pid.hpp>

#include "robot_control/kinematics/differential_kinematics.hpp"

#include "robot_control/odometry/odometry.hpp"

#include "robot_control/srv/set_speed.hpp"
#include "robot_control/srv/get_encoder.hpp"
#include "robot_control/srv/pid.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"


class Controller : public rclcpp::Node {
public:
    Controller( );
    ~Controller(){}

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void encCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void publish_odometry();

    void stop_Callback(const std_msgs::msg::Bool::SharedPtr msg){ stop_state = msg->data; }

    void pidCallback( std::shared_ptr<robot_control::srv::Pid::Request> request, std::shared_ptr<robot_control::srv::Pid::Response> response );

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr enc_sub_; 
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_button_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::Client<robot_control::srv::SetSpeed>::SharedPtr speed_client_;
    rclcpp::Service<robot_control::srv::Pid>::SharedPtr pid_service_;


    rclcpp::TimerBase::SharedPtr timer_;


    std::string kinematics_;

    /* Differential Robot */
    DifferentialKimematics differential_kim;
    DifferentialSpeeds differential_vel;

    /* Encoder */
    std::vector<int> enc = {0,0,0,0};
    std::vector<int> previous_encoder = {0,0,0,0};

    /* PID */
    std::vector<control_toolbox::Pid> pid_wheels;
    std::vector<double> wheels_cmd = {0,0,0,0};

    geometry_msgs::msg::Twist robot_twist;

    double previous_time_cmd;
    double previous_time_odom;

    bool stop_state = true;

};

#endif  // CONTROLLER

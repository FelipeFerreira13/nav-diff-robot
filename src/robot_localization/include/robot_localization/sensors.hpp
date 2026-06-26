#ifndef SENSORS
#define SENSORS

#include <string.h>
#include <math.h>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp_action/rclcpp_action.hpp>

#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <robot_localization/srv/get_data.hpp>
#include <robot_localization/srv/get_angle.hpp>
#include <robot_localization/srv/sensor_driver.hpp>
#include <robot_localization/action/sensor_command.hpp>


#include <geometry_msgs/msg/twist.hpp>

#include "nav_msgs/msg/odometry.hpp"

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <utils/UtilsGeneral.h>


using SensorCommand = robot_localization::action::SensorCommand;
using GoalHandleSensor = rclcpp_action::ServerGoalHandle<SensorCommand>;


class Sensor : public rclcpp::Node
{
    public:
        Sensor();
        ~Sensor(){ }

        double GetRightSharp();
        double GetLeftSharp();
        double GetArmSharp();

        double GetRightUS();
        double GetLeftUS();

        float sensor_mean( double & sensor_dist, int samples );
        float get_angle_wall( int sample );

        void right_sharpCallback(const sensor_msgs::msg::Range::SharedPtr msg){ std::lock_guard<std::mutex> lock(mtx_); sharp_right_dist = msg->range; cv_.notify_all(); }
        void left_sharpCallback(const sensor_msgs::msg::Range::SharedPtr msg){ std::lock_guard<std::mutex> lock(mtx_); sharp_left_dist = msg->range; cv_.notify_all(); }
        void arm_sharpCallback(const sensor_msgs::msg::Range::SharedPtr msg){ std::lock_guard<std::mutex> lock(mtx_); sharp_arm_dist = msg->range; cv_.notify_all(); }
        void right_ultrasonicCallback(const sensor_msgs::msg::Range::SharedPtr msg){ std::lock_guard<std::mutex> lock(mtx_); us_right_dist = msg->range; cv_.notify_all(); }
        void left_ultrasonicCallback(const sensor_msgs::msg::Range::SharedPtr msg){ std::lock_guard<std::mutex> lock(mtx_); us_left_dist = msg->range; cv_.notify_all(); }
        void scanCallback( const sensor_msgs::msg::LaserScan::SharedPtr msg );

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

        rclcpp_action::GoalResponse handle_goal( const rclcpp_action::GoalUUID &, std::shared_ptr<const SensorCommand::Goal> goal);
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleSensor>);
        void handle_accepted(const std::shared_ptr<GoalHandleSensor> goal_handle);
        void execute(const std::shared_ptr<GoalHandleSensor> goal_handle);

        void get_sensor  ( const std::shared_ptr<GoalHandleSensor> goal_handle, std::shared_ptr<SensorCommand::Result> result );
        void get_angle   ( const std::shared_ptr<GoalHandleSensor> goal_handle, std::shared_ptr<SensorCommand::Result> result );
        void linear_align( const std::shared_ptr<GoalHandleSensor> goal_handle, std::shared_ptr<SensorCommand::Result> result );

        void PrintScan();
        double scan_range( double value, double range );

    private:

        rclcpp_action::Server<SensorCommand>::SharedPtr action_server_;

        rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr right_sharp_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr left_sharp_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr arm_sharp_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr right_ultrasonic_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr left_ultrasonic_sub_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

        rclcpp::Service<robot_localization::srv::GetData>::SharedPtr get_sensor_srv_;
        rclcpp::Service<robot_localization::srv::GetAngle>::SharedPtr get_angle_srv_;
        rclcpp::Service<robot_localization::srv::SensorDriver>::SharedPtr linear_align_srv_;

        Pose2D robot_position;
        
        double cobra_l  = 0;
        double cobra_r  = 0;
        double cobra_cl = 0;
        double cobra_cr = 0;

        /* Sharp */
        double sharp_right_dist = 0;
        double sharp_left_dist  = 0;
        double sharp_arm_dist   = 0;

        /* Ultrasonic */
        double us_right_dist = 0;
        double us_left_dist  = 0;
        double baseline_distance = 0.0;

        /* Lidar */
        int left_ang  = 180;
        int right_ang = 5;
        int front_ang = 93;

        double left_scan;
        double right_scan;
        double front_scan;

        sensor_msgs::msg::LaserScan last_scan;


        bool process_done = false;
        std::mutex mtx_;
        std::condition_variable cv_;

};


void Sensor::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
  robot_position.x     = msg->pose.pose.position.x;
  robot_position.y     = msg->pose.pose.position.y;
  robot_position.theta = tf2::getYaw(msg->pose.pose.orientation) * ( 180.0 / M_PI );
  if     ( robot_position.theta < 0   ){ robot_position.theta += 360; }
  else if( robot_position.theta > 360 ){ robot_position.theta -= 360; }
}

void Sensor::scanCallback( const sensor_msgs::msg::LaserScan::SharedPtr msg ){

    double ang_diff = (msg->angle_max - msg->angle_min);
    double n_ang    = (2 * M_PI) / msg->angle_increment;   

    int i_left  = left_ang  * ( n_ang / 360.0 );
    int i_right = right_ang * ( n_ang / 360.0 );
    int i_front = front_ang * ( n_ang / 360.0 );

    left_scan  = msg->ranges[i_left];
    right_scan = msg->ranges[i_right];
    front_scan = msg->ranges[i_front];

    last_scan = *msg;

}

void Sensor::PrintScan(){

    double ang_diff = (last_scan.angle_max - last_scan.angle_min);
    double n_ang    = (2 * M_PI) / last_scan.angle_increment;   

    RCLCPP_INFO(this->get_logger(), "angle_max: %f.2, angle_min: %f.2, angle_increment: %f.2", last_scan.angle_max ,last_scan.angle_min, last_scan.angle_increment);

    if( ang_diff >= 1 ){
        for( int i = 0; i < 360; i++ ){
            int angle  = i * ( n_ang / 360.0 );
            RCLCPP_INFO(this->get_logger(), "Scan %i: %f.2 Scan %i: %f.2", i, last_scan.ranges[i], angle, last_scan.ranges[angle]);
        }
    }

}

double Sensor::scan_range( double value, double range ){
    return value >= range ? value - range : value;
}

double Sensor::GetRightSharp(){ return sensor_mean( sharp_right_dist, 10 ); }
double Sensor::GetLeftSharp(){ return sensor_mean( sharp_left_dist, 10 ); }
double Sensor::GetArmSharp(){ return sensor_mean( sharp_arm_dist, 10 ); }
double Sensor::GetRightUS(){ return sensor_mean( us_right_dist, 10 ); }
double Sensor::GetLeftUS(){ return sensor_mean( us_left_dist, 10 ); }

#endif
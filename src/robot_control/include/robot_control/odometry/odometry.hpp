#ifndef ODOMETRY
#define ODOMETRY

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/utils.hpp>

#include "utils/UtilsGeneral.h"
#include "utils/srv/pose.hpp"

class Odometry : public rclcpp::Node {
public:
    Odometry();
    ~Odometry(){}

    bool updateAndPublish(double linear_x, double linear_y, double angular);

    double getX() const { return x_; }
    double getY() const { return y_; }
    double getHeading() const { return heading_; }
    double getLinearX() const { return linear_x_; }
    double getLinearY() const { return linear_y_; }
    double getAngular() const { return angular_; }


private:
    double getImu();
    void publishOdometry();
    void integrateExact(double vx, double vy, double vth);
    void resetAccumulators();
    
    void SetPosition( std::shared_ptr<utils::srv::Pose::Request> request, std::shared_ptr<utils::srv::Pose::Response> response );

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void twistVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Service<utils::srv::Pose>::SharedPtr set_service_;


    bool publish_tf_ = true;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    double x_, y_, heading_;
    double linear_x_, linear_y_, angular_;

    bool use_imu_ = true;
    sensor_msgs::msg::Imu imu_data_;
    std::mutex imu_data_mutex_;
    double prev_imu_yaw_;

    double previous_time = this->now().seconds();

    size_t velocity_rolling_window_size_ = 10;
    Utils::RollingMeanAccumulator linear_x_accumulator_;
    Utils::RollingMeanAccumulator linear_y_accumulator_;
    Utils::RollingMeanAccumulator angular_accumulator_;

};

#endif  // ODOMETRY

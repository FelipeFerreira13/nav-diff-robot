#ifndef DIFF_DRIVE_ODOMETRY_H
#define DIFF_DRIVE_ODOMETRY_H

#include <cmath>
#include <deque>
#include <mutex>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "utils/UtilsGeneral.h"

namespace studica_control {

class DiffOdometry : public rclcpp::Node {
public:
    static std::shared_ptr<DiffOdometry> initialize(rclcpp::Node *control);
    explicit DiffOdometry(const rclcpp::NodeOptions & options);
    explicit DiffOdometry(const std::string &name, bool use_imu, const std::string &imu_topic, const std::string &topic, bool publish_tf, size_t velocity_rolling_window_size = 10);
    ~DiffOdometry();

    void init(const rclcpp::Time &time);
    void publishOdometry();
    bool updateAndPublish(double left_pos, double right_pos, const rclcpp::Time &time);
    bool updateFromVelocity(double left_vel, double right_vel, const rclcpp::Time &time);
    void updateOpenLoop(double linear, double angular, const rclcpp::Time &time);
    void resetOdometry();

    double getX() const { return x_; }
    double getY() const { return y_; }
    double getHeading() const { return heading_; }
    double getLinear() const { return linear_; }
    double getAngular() const { return angular_; }

    void setWheelParams(float wheel_separation);
    void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void integrateRungeKutta2(double linear, double angular);
    void integrateExact(double linear, double angular);
    void resetAccumulators();

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Time timestamp_;
    sensor_msgs::msg::Imu imu_data_;
    std::mutex imu_data_mutex_;
    double x_, y_, heading_;
    double linear_, angular_;
    double wheel_separation_;
    double left_wheel_prev_pos_, right_wheel_prev_pos_;
    bool use_imu_;
    bool publish_tf_;
    std::string imu_topic_;
    std::string topic_;
    size_t velocity_rolling_window_size_;
    Utils::RollingMeanAccumulator linear_accumulator_;
    Utils::RollingMeanAccumulator angular_accumulator_;
};

} // namespace studica_control

#endif // DIFF_DRIVE_ODOMETRY_H

#ifndef OMNI_DRIVE_ODOMETRY_H
#define OMNI_DRIVE_ODOMETRY_H


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

namespace studica_control {


class OmniOdometry : public rclcpp::Node {
public:
    static std::shared_ptr<OmniOdometry> initialize(rclcpp::Node *control);
    explicit OmniOdometry(const rclcpp::NodeOptions & options);
    explicit OmniOdometry(const std::string &name, bool use_imu, const std::string &imu_topic, const std::string &topic, bool publish_tf, size_t velocity_rolling_window_size = 10);
    ~OmniOdometry();

    void init(const rclcpp::Time &time);
    void publishOdometry();
    bool updateAndPublish(double left_pos, double right_pos,  double back_pos, const rclcpp::Time &time);
    bool updateFromVelocity(double left_vel, double right_vel, double back_pos, const rclcpp::Time &time);
    void updateOpenLoop(double linear_x, double linear_y, double angular, const rclcpp::Time &time);
    void resetOdometry();
    void SetPosition( double x, double y, double th );

    double getX() const { return x_; }
    double getY() const { return y_; }
    double getHeading() const { return heading_; }
    double getLinearX() const { return linear_x_; }
    double getLinearY() const { return linear_y_; }
    double getAngular() const { return angular_; }

    void setWheelParams(float wheel_separation);
    void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    double getImu();
    void integrateExact(double vx, double vy, double vth);
    void resetAccumulators();

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Time timestamp_;
    sensor_msgs::msg::Imu imu_data_;
    std::mutex imu_data_mutex_;
    double x_, y_, heading_;
    double linear_x_, linear_y_, angular_;
    double wheel_separation_;
    double left_wheel_prev_pos_, right_wheel_prev_pos_, back_wheel_prev_pos_;
    double prev_imu_yaw_;
    bool use_imu_;
    bool publish_tf_;
    std::string imu_topic_;
    std::string topic_;
    size_t velocity_rolling_window_size_;
    Utils::RollingMeanAccumulator linear_x_accumulator_;
    Utils::RollingMeanAccumulator linear_y_accumulator_;
    Utils::RollingMeanAccumulator angular_accumulator_;
};

} // namespace studica_control

#endif // OMNI_DRIVE_ODOMETRY_H

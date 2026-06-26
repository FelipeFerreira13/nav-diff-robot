#include "robot_control/odometry/odometry.hpp"

Odometry::Odometry() : Node("odometry_node"){

    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu", 10, 
        std::bind(&Odometry::imuCallback, this, std::placeholders::_1));

    twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("robot/twist", 10, 
        std::bind(&Odometry::twistVelCallback, this, std::placeholders::_1));

    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    set_service_ = this->create_service<utils::srv::Pose>( "robot/position/set_pose", 
        std::bind(&Odometry::SetPosition, this, std::placeholders::_1, std::placeholders::_2));

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    previous_time = this->now().seconds();

    resetAccumulators();

    RCLCPP_INFO(this->get_logger(), "Odometry started");
}

void Odometry::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(imu_data_mutex_);
    imu_data_ = *msg;
}

void Odometry::twistVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {

    updateAndPublish(msg->linear.x, msg->linear.y, msg->angular.z);
}

double Odometry::getImu(){
    std::lock_guard<std::mutex> lk(imu_data_mutex_);

    double imu_yaw = tf2::getYaw(imu_data_.orientation);

    double raw = imu_yaw - prev_imu_yaw_;
    double delta_theta = std::atan2(std::sin(raw), std::cos(raw));

    delta_theta = delta_theta;  // Negative according to controller used

    prev_imu_yaw_ = imu_yaw;


    return delta_theta;
}

void Odometry::publishOdometry() {
    auto current_time = this->now();
    
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_footprint";

    tf2::Quaternion q;

    q.setRPY(0, 0, heading_);
    odom_msg.pose.pose.orientation = tf2::toMsg(q);
    odom_msg.twist.twist.angular.z = angular_;

    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;
    
    odom_msg.twist.twist.linear.x = linear_x_;
    odom_msg.twist.twist.linear.y = linear_y_;

    odom_msg.pose.covariance = {
        0.01, 0.0,  0.0,  0.0,  0.0,  0.0,
        0.0,  0.01, 0.0,  0.0,  0.0,  0.0,
        0.0,  0.0,  1e6,  0.0,  0.0,  0.0,
        0.0,  0.0,  0.0,  1e6,  0.0,  0.0,
        0.0,  0.0,  0.0,  0.0,  1e6,  0.0,
        0.0,  0.0,  0.0,  0.0,  0.0,  1e3
    };

    odom_msg.twist.covariance = {
        0.01,  0.0,  0.0,  0.0,  0.0,  0.0,
        0.0,  0.01,  0.0,  0.0,  0.0,  0.0,
        0.0,   0.0,  1e6,  0.0,  0.0,  0.0,
        0.0,   0.0,  0.0,  1e6,  0.0,  0.0,
        0.0,   0.0,  0.0,  0.0,  1e6,  0.0,
        0.0,   0.0,  0.0,  0.0,  0.0,  1e3
    };

    odom_publisher_->publish(odom_msg);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = current_time;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_footprint";

    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;

    tf.transform.rotation = odom_msg.pose.pose.orientation;
    
    if (publish_tf_) {
        tf_broadcaster_->sendTransform(tf);
    }

    // RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, th: %f",x_, y_, heading_ * ( 180.0 / M_PI ));
    // RCLCPP_INFO(this->get_logger(), "vx: %f, vy: %f, vth: %f",linear_x_, linear_y_, angular_ * ( 180.0 / M_PI ));


}

bool Odometry::updateAndPublish(double linear_x, double linear_y, double angular) {

    double current_time = this->now().seconds();
    double dt = current_time - previous_time;
    previous_time = current_time;
    
    if (dt < 0.0001) return false;

    integrateExact(linear_x * dt, linear_y * dt, angular * dt);
    
    linear_x_accumulator_.accumulate(linear_x);
    linear_y_accumulator_.accumulate(linear_y);
    angular_accumulator_.accumulate (angular );

    linear_x_ = linear_x_accumulator_.getRollingMean();
    linear_y_ = linear_y_accumulator_.getRollingMean();
    angular_  = angular_accumulator_.getRollingMean();

    publishOdometry();

    return true;
}

void Odometry::SetPosition( std::shared_ptr<utils::srv::Pose::Request> request, std::shared_ptr<utils::srv::Pose::Response> response ){

    integrateExact(0,0,0);

    x_ = request->x;
    y_ = request->y;
    heading_ = request->theta;

    integrateExact(0,0,0);
}

void Odometry::integrateExact(double vx, double vy, double vth)
{
    double x = vx * std::cos( heading_ ) - vy * std::sin( heading_ );
    double y = vx * std::sin( heading_ ) + vy * std::cos( heading_ );

    double imu = this->getImu();

    x_ += x;
    y_ += y;

    if (use_imu_) { heading_ += imu; }
    else          { heading_ += vth; }

    if      ( heading_ <  0       ) { heading_ += 2 * M_PI; }
    else if ( heading_ > 2 * M_PI ) { heading_ -= 2 * M_PI; }
}


void Odometry::resetAccumulators() {
    linear_x_accumulator_ = Utils::RollingMeanAccumulator(velocity_rolling_window_size_);
    linear_y_accumulator_ = Utils::RollingMeanAccumulator(velocity_rolling_window_size_);
    angular_accumulator_  = Utils::RollingMeanAccumulator(velocity_rolling_window_size_);
}


#include "studica_control/omni_drive_odometry.h"

namespace studica_control {

std::shared_ptr<OmniOdometry> OmniOdometry::initialize(rclcpp::Node *control){
    control->declare_parameter<std::string>("omni_drive_odometry.name",       "");
    control->declare_parameter<bool>       ("omni_drive_odometry.use_imu",    true);
    control->declare_parameter<std::string>("omni_drive_odometry.imu_topic",  "");
    control->declare_parameter<std::string>("omni_drive_odometry.topic",      "unknown");
    control->declare_parameter<bool>       ("omni_drive_odometry.publish_tf", true);

    std::string  name       = control->get_parameter("omni_drive_odometry.name"      ).as_string();
    bool         use_imu    = control->get_parameter("omni_drive_odometry.use_imu"   ).as_bool();
    std::string  imu_topic  = control->get_parameter("omni_drive_odometry.imu_topic" ).as_string();
    std::string  topic      = control->get_parameter("omni_drive_odometry.topic"     ).as_string();
    bool         publish_tf = control->get_parameter("omni_drive_odometry.publish_tf").as_bool();

    auto omni_odom = std::make_shared<OmniOdometry>(name, use_imu, imu_topic, topic, publish_tf, 10);
    return omni_odom;
}

OmniOdometry::OmniOdometry(const rclcpp::NodeOptions & options) : Node("omni_odometry", options) {}

OmniOdometry::OmniOdometry(const std::string &name, bool use_imu, const std::string &imu_topic, const std::string &topic, bool publish_tf, size_t velocity_rolling_window_size)
: Node(name),
  timestamp_(0.0), 
  x_(0.0), y_(0.0), 
  heading_(0.0), 
  linear_x_(0.0), 
  linear_y_(0.0), 
  angular_(0.0), 
  wheel_separation_(0.0), 
  left_wheel_prev_pos_(0.0),
  right_wheel_prev_pos_(0.0),
  back_wheel_prev_pos_(0.0),
  use_imu_(use_imu),
  publish_tf_(publish_tf),
  imu_topic_(imu_topic),
  topic_(topic),
  velocity_rolling_window_size_(velocity_rolling_window_size),
  linear_x_accumulator_(velocity_rolling_window_size),
  linear_y_accumulator_(velocity_rolling_window_size),
  angular_accumulator_(velocity_rolling_window_size) {}

OmniOdometry::~OmniOdometry() {}

void OmniOdometry::init(const rclcpp::Time &time) {
    resetAccumulators();
    timestamp_ = time;

    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic_, 10, std::bind(&OmniOdometry::imuCallback, this, std::placeholders::_1));
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(topic_, 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void OmniOdometry::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(imu_data_mutex_);
    imu_data_ = *msg;
    use_imu_ = true;
}

double OmniOdometry::getImu(){
    std::lock_guard<std::mutex> lk(imu_data_mutex_);

    double imu_yaw = tf2::getYaw(imu_data_.orientation);

    double raw = imu_yaw - prev_imu_yaw_;
    double delta_theta = std::atan2(std::sin(raw), std::cos(raw));

    prev_imu_yaw_ = imu_yaw;

    return delta_theta;
}

void OmniOdometry::publishOdometry() {
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

}

bool OmniOdometry::updateAndPublish(double left_pos, double right_pos, double back_pos, const rclcpp::Time &time) {
    const double dt = time.seconds() - timestamp_.seconds();
    if (dt < 0.0001) return false;

    const double left_wheel_cur_pos  = left_pos;
    const double right_wheel_cur_pos = right_pos;
    const double back_wheel_cur_pos  = back_pos;

    const double left_wheel_est_vel  = left_wheel_cur_pos  - left_wheel_prev_pos_;
    const double right_wheel_est_vel = right_wheel_cur_pos - right_wheel_prev_pos_;
    const double back_wheel_est_vel  = back_wheel_cur_pos  - back_wheel_prev_pos_;

    left_wheel_prev_pos_  = left_wheel_cur_pos;
    right_wheel_prev_pos_ = right_wheel_cur_pos;
    back_wheel_prev_pos_  = back_wheel_cur_pos;

    updateFromVelocity(left_wheel_est_vel, right_wheel_est_vel, back_wheel_est_vel, time);

    publishOdometry();

    return true;
}

bool OmniOdometry::updateFromVelocity(double left_vel, double right_vel, double back_vel, const rclcpp::Time &time) {
    const double dt = time.seconds() - timestamp_.seconds();
    if (dt < 0.0001) return false;

    const double linear_x =  (left_vel - right_vel) / std::sqrt(3.0);
    const double linear_y =  (-right_vel + 2.0*back_vel - left_vel) / 3.0; 
    const double angular  = -(back_vel + right_vel + left_vel) / (3.0 * wheel_separation_);

    integrateExact(linear_x, linear_y, angular);
    
    timestamp_ = time;

    linear_x_accumulator_.accumulate(linear_x / dt);
    linear_y_accumulator_.accumulate(linear_y / dt);
    angular_accumulator_.accumulate (angular  / dt);

    linear_x_ = linear_x_accumulator_.getRollingMean();
    linear_y_ = linear_x_accumulator_.getRollingMean();
    angular_  = angular_accumulator_.getRollingMean();

    return true;
}

void OmniOdometry::updateOpenLoop(double linear_x, double linear_y, double angular, const rclcpp::Time &time) {
    linear_x_ = linear_x;
    linear_y_ = linear_y;
    angular_  = angular;

    const double dt = time.seconds() - timestamp_.seconds();
    timestamp_ = time;
    integrateExact(linear_x * dt, linear_y * dt, angular * dt);
}

void OmniOdometry::resetOdometry() {
    x_ = 0.0;
    y_ = 0.0;
    heading_ = 0.0;
}

void OmniOdometry::SetPosition( double x, double y, double th ){
    x_ = x;
    y_ = y;
    heading_ = th;
}

void OmniOdometry::setWheelParams(float wheel_separation) {
    wheel_separation_ = wheel_separation;
}

void OmniOdometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size) {
    velocity_rolling_window_size_ = velocity_rolling_window_size;
    resetAccumulators();
}

void OmniOdometry::integrateExact(double vx, double vy, double vth)
{
    double x = vx * std::cos( heading_ ) - vy * std::sin( heading_ );
    double y = vx * std::sin( heading_ ) + vy * std::cos( heading_ );

    x_ += x;
    y_ += y;

    if (use_imu_) { heading_ += this->getImu();
    }else         { heading_ += vth; }

    if      ( heading_ <  0       ) { heading_ += 2 * M_PI; }
    else if ( heading_ > 2 * M_PI ) { heading_ -= 2 * M_PI; }
}


void OmniOdometry::resetAccumulators() {
    linear_x_accumulator_ = Utils::RollingMeanAccumulator(velocity_rolling_window_size_);
    linear_y_accumulator_ = Utils::RollingMeanAccumulator(velocity_rolling_window_size_);
    angular_accumulator_  = Utils::RollingMeanAccumulator(velocity_rolling_window_size_);
}

} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::OmniOdometry)

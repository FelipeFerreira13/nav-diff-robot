#include "studica_control/omni_drive_component.h"

namespace studica_control {

std::shared_ptr<rclcpp::Node> OmniDrive::initialize(rclcpp::Node *control, std::shared_ptr<OmniOdometry> odom, std::shared_ptr<VMXPi> vmx) {
    control->declare_parameter<std::string>("omni_drive_component.name", "");
    control->declare_parameter<int>("omni_drive_component.can_id", -1);
    control->declare_parameter<int>("omni_drive_component.motor_freq", -1);
    control->declare_parameter<int>("omni_drive_component.ticks_per_rotation", -1);
    control->declare_parameter<float>("omni_drive_component.wheel_radius", -1.0);
    control->declare_parameter<float>("omni_drive_component.wheelbase", -1.0);
    control->declare_parameter<float>("omni_drive_component.width", -1.0);
    control->declare_parameter<int>("omni_drive_component.left", -1);
    control->declare_parameter<int>("omni_drive_component.right", -1);
    control->declare_parameter<int>("omni_drive_component.back", -1);
    control->declare_parameter<bool>("omni_drive_component.invert_left", false);
    control->declare_parameter<bool>("omni_drive_component.invert_right", false);
    control->declare_parameter<bool>("omni_drive_component.invert_back", false);

    std::string name = control->get_parameter("omni_drive_component.name").as_string();
    int can_id = control->get_parameter("omni_drive_component.can_id").as_int();
    int motor_freq = control->get_parameter("omni_drive_component.motor_freq").as_int();
    int ticks_per_rotation = control->get_parameter("omni_drive_component.ticks_per_rotation").as_int();
    float wheel_radius = control->get_parameter("omni_drive_component.wheel_radius").get_value<float>();
    float wheelbase = control->get_parameter("omni_drive_component.wheelbase").get_value<float>();
    float width = control->get_parameter("omni_drive_component.width").get_value<float>();
    int l = control->get_parameter("omni_drive_component.left").as_int();
    int r = control->get_parameter("omni_drive_component.right").as_int();
    int b = control->get_parameter("omni_drive_component.back").as_int();
    bool invert_l = control->get_parameter("omni_drive_component.invert_left").as_bool();
    bool invert_r = control->get_parameter("omni_drive_component.invert_right").as_bool();
    bool invert_b = control->get_parameter("omni_drive_component.invert_back").as_bool();

    RCLCPP_INFO(control->get_logger(), "%s -> l: %d, r: %d, b: %d", name.c_str(), l, r, b);

    auto omni_drive_node = std::make_shared<OmniDrive>(vmx, odom, name, can_id, motor_freq, ticks_per_rotation, wheel_radius, wheelbase, width, l, r, b, invert_l, invert_r, invert_b );
    return omni_drive_node;
}

OmniDrive::OmniDrive(const rclcpp::NodeOptions & options) : Node("omni_drive", options) {}

OmniDrive::OmniDrive(
    std::shared_ptr<VMXPi> vmx,
    std::shared_ptr<studica_control::OmniOdometry> odom,
    const std::string &name,
    const uint8_t &can_id,
    const uint16_t &motor_freq,
    const uint16_t &ticks_per_rotation,
    const float &wheel_radius,
    const float &wheelbase,
    const float &width,
    const uint8_t &left,
    const uint8_t &right,
    const uint8_t &back,
    const bool &invert_left,
    const bool &invert_right,
    const bool &invert_back) 
    : Node(name),
      vmx_(vmx),
      odom_(odom),
      can_id_(can_id),
      motor_freq_(motor_freq),
      ticks_per_rotation_(ticks_per_rotation),
      wheel_radius_(wheel_radius),
      wheelbase_(wheelbase),
      width_(width),
      l_(left),
      r_(right),
      b_(back) {

    dist_per_tick_ = 2 * M_PI * wheel_radius_ / ticks_per_rotation_;

    titan_ = std::make_shared<studica_driver::Titan>(can_id_, motor_freq_, dist_per_tick_, vmx_);

    service_ = this->create_service<studica_control::srv::SetData>(
        "titan_cmd",
        std::bind(&OmniDrive::cmd_callback, this, std::placeholders::_1, std::placeholders::_2));

    titan_->ConfigureEncoder(l_, dist_per_tick_);
    titan_->ConfigureEncoder(r_, dist_per_tick_);
    titan_->ConfigureEncoder(b_, dist_per_tick_);

    titan_->ResetEncoder(l_);
    titan_->ResetEncoder(r_);
    titan_->ResetEncoder(b_);

    if (invert_left ) titan_->InvertMotor(l_);
    if (invert_right) titan_->InvertMotor(r_);
    if (invert_back ) titan_->InvertMotor(b_);

    titan_->Enable(true);

    odom_->setWheelParams(wheelbase_);
    odom_->init(this->now());

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&OmniDrive::publish_odometry, this));

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        10,
        std::bind(&OmniDrive::cmd_vel_callback, this, std::placeholders::_1)
    );
}

OmniDrive::~OmniDrive() {}

void OmniDrive::cmd_callback(std::shared_ptr<studica_control::srv::SetData::Request> request, std::shared_ptr<studica_control::srv::SetData::Response> response) {
    std::string params = request->params;
    cmd(params, request, response);
}

void OmniDrive::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    double linear_x = msg->linear.x;
    double linear_y = msg->linear.y;
    double angular  = msg->angular.z;

    double left_command  = ( ( std::sqrt(3.0) * linear_x - linear_y) / 2.0 - ( wheelbase_ * angular) );  // [m/s]
    double right_command = ( (-std::sqrt(3.0) * linear_x - linear_y) / 2.0 - ( wheelbase_ * angular) );  // [m/s]
    double back_command  = ( linear_y - ( wheelbase_ * angular) );  // [m/s]

    titan_->SetSpeed(l_, left_command );
    titan_->SetSpeed(r_, right_command);
    titan_->SetSpeed(b_, back_command );
}

void OmniDrive::cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Request> request, std::shared_ptr<studica_control::srv::SetData::Response> response) {
    if (params == "enable") {
        titan_->Enable(true);
        response->success = true;
        response->message = "Titan enabled";
    } else if (params == "disable") {
        titan_->Enable(false);
        response->success = true;
        response->message = "Titan disabled";
    } else if (params == "start") {
        titan_->Enable(true);
        response->success = true;
        response->message = "Titan started";
    } else if (params == "setup_encoder") {
        titan_->SetupEncoder(request->initparams.n_encoder);
        response->success = true;
        response->message = "Titan encoder setup complete";
    } else if (params == "configure_encoder") {
        titan_->ConfigureEncoder(request->initparams.n_encoder, request->initparams.dist_per_tick);
        response->success = true;
        response->message = "Titan encoder configured";
    } else if (params == "stop") {
        titan_->SetSpeed(request->initparams.n_encoder, 0.0);
        response->success = true;
        response->message = "Titan stopped";
    } else if (params == "reset") {
        titan_->ResetEncoder(request->initparams.n_encoder);
        response->success = true;
        response->message = "Titan reset";
    } else if (params == "set_speed") {
        response->success = true;
        float speed = request->initparams.speed;
        RCLCPP_INFO(this->get_logger(), "Setting speed to %f", speed);
        titan_->SetSpeed(request->initparams.n_encoder, speed);
        response->message = "Encoder " + std::to_string(request->initparams.n_encoder) + " speed set to " + std::to_string(request->initparams.speed);
    } else if (params == "get_encoder_distance") {
        response->success = true;
        response->message = std::to_string(titan_->GetEncoderDistance(request->initparams.n_encoder));
    } else if (params == "get_rpm") {
        response->success = true;
        response->message = std::to_string(titan_->GetRPM(request->initparams.n_encoder));
    } else if (params == "get_encoder_count") {
        response->success = true;
        response->message = std::to_string(titan_->GetEncoderCount(request->initparams.n_encoder));
    } else {
        response->success = false;
        response->message = "No such command '" + params + "'";
    }
}

void OmniDrive::publish_odometry() {
    // Get encoder distances from all 4 motors
    double l_encoder = titan_->GetEncoderDistance(l_);
    double r_encoder = titan_->GetEncoderDistance(r_);
    double b_encoder = titan_->GetEncoderDistance(b_);

    auto current_time = this->now();

    odom_->updateAndPublish(l_encoder, r_encoder, b_encoder, current_time);
}

} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::OmniDrive)

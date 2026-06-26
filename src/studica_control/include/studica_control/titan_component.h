#ifndef TITAN_COMPONENT_H
#define TITAN_COMPONENT_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "robot_control/srv/set_speed.hpp"
#include "robot_control/srv/get_encoder.hpp"
#include "studica_control/srv/set_data.hpp"

#include "titan.h"
#include "VMXPi.h" 

namespace studica_control {

class Titan : public rclcpp::Node {
public:
    static std::vector<std::shared_ptr<rclcpp::Node>> initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx);
    explicit Titan(const rclcpp::NodeOptions & options);
    Titan(std::shared_ptr<VMXPi> vmx, const std::string &name, const uint8_t &canID, const uint16_t &motor_freq, const std::string &topic);
    ~Titan();

private:
    std::shared_ptr<studica_driver::Titan> titan_;
    std::shared_ptr<VMXPi> vmx_;
    uint8_t canID_;
    uint16_t motor_freq_;
    float dist_per_tick_;

    rclcpp::Service<robot_control::srv::SetSpeed>::SharedPtr motor_speed_srv_;
    rclcpp::Service<robot_control::srv::GetEncoder>::SharedPtr get_encoder_srv_;

    rclcpp::Service<studica_control::srv::SetData>::SharedPtr service_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    void titanVelCallback(std::shared_ptr<robot_control::srv::SetSpeed::Request> request, std::shared_ptr<robot_control::srv::SetSpeed::Response> response);
    void getEncCallback(std::shared_ptr<robot_control::srv::GetEncoder::Request> request, std::shared_ptr<robot_control::srv::GetEncoder::Response> response);

    void cmd_callback(std::shared_ptr<studica_control::srv::SetData::Request> request, std::shared_ptr<studica_control::srv::SetData::Response> response);
    void cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Request> request, std::shared_ptr<studica_control::srv::SetData::Response> response);
    void publish_encoders();
};

}  // namespace studica_control

#endif  // TITAN_COMPONENT_H

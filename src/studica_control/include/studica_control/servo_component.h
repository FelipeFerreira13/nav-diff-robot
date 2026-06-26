#ifndef SERVO_COMPONENT_H
#define SERVO_COMPONENT_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include "servo.h"
#include "studica_control/srv/set_data.hpp"
#include "utils/srv/int32.hpp"
#include "VMXPi.h"

namespace studica_control {

class Servo : public rclcpp::Node {
public:
    static std::vector<std::shared_ptr<rclcpp::Node>> initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx);
    explicit Servo(const rclcpp::NodeOptions &options);
    Servo(std::shared_ptr<VMXPi> vmx, const std::string &name, VMXChannelIndex port, studica_driver::ServoType type, int min, int max, const std::string &topic);
    ~Servo();

private:
    std::shared_ptr<studica_driver::Servo> servo_;
    std::shared_ptr<VMXPi> vmx_;
    rclcpp::Service<studica_control::srv::SetData>::SharedPtr service_;
    rclcpp::Service<utils::srv::Int32>::SharedPtr servo_service_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    VMXChannelIndex port_;
    studica_driver::ServoType type_;
    void servo_callback(const std::shared_ptr<utils::srv::Int32::Request> request,
                                    std::shared_ptr<utils::srv::Int32::Response> response);
    void cmd_callback(const std::shared_ptr<studica_control::srv::SetData::Request> request,
        std::shared_ptr<studica_control::srv::SetData::Response> response);
    void publish_angle();
    void DisplayVMXError(VMXErrorCode vmxerr);
};

} // namespace studica_control

#endif // SERVO_COMPONENT_H

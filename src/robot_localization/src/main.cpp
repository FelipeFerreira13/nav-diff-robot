#include "rclcpp/rclcpp.hpp"

#include "robot_localization/sensors.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Create executor
  rclcpp::executors::MultiThreadedExecutor executor;

  // Create nodes
  auto sensor = std::make_shared<Sensor>();

  // Add nodes to executor
  executor.add_node(sensor);

  RCLCPP_INFO(rclcpp::get_logger("robot_localization"), "Localization Node");

  // Spin all nodes
  executor.spin();

  rclcpp::shutdown();
  return 0;
}

#include "rclcpp/rclcpp.hpp"

// Include your nodes
#include "robot_control/controller/controller.hpp"
#include "robot_control/odometry/odometry.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Create executor
  rclcpp::executors::MultiThreadedExecutor executor;

  // Create nodes
  auto controller     = std::make_shared<Controller>();
  auto odometry       = std::make_shared<Odometry>  ();

  // Add nodes to executor
  executor.add_node(controller);
  executor.add_node(odometry);

  // Spin all nodes
  executor.spin();

  rclcpp::shutdown();
  return 0;
}

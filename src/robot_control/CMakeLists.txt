cmake_minimum_required(VERSION 3.8)
project(robot_control)

# --------------------------------------------------
# Compiler options
# --------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --------------------------------------------------
# Dependencies
# --------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_typesupport_c REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(utils REQUIRED)

# --------------------------------------------------
# Action generation
# --------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetSpeed.srv"
  "srv/GetEncoder.srv"
  "srv/Pid.srv"
  DEPENDENCIES
)

# Get the interface library and then link it
rosidl_get_typesupport_target(typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")


# --------------------------------------------------
# Differential library
# --------------------------------------------------

add_library(differential_kinematics
  src/kinematics/differential_kinematics.cpp
)

target_include_directories(differential_kinematics
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(differential_kinematics
  rclcpp
  geometry_msgs
  utils
)

# --------------------------------------------------
# Odometry library
# --------------------------------------------------
add_library(odometry src/odometry/odometry.cpp)

target_include_directories(odometry
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(odometry 
  rclcpp 
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  utils
)


# --------------------------------------------------
# Controller library
# --------------------------------------------------

add_library(controller src/controller/controller.cpp)

# Links other libraries (internal or external) to a target so it can use their functions.
# Because there is a library being linked it can find the header files, 
# so no need for target_include_directories
target_link_libraries(controller
  differential_kinematics
  odometry
)

ament_target_dependencies(controller 
  rclcpp 
  std_msgs
  geometry_msgs
  utils
  control_toolbox
)

# Attaches generated ROS 2 interfaces (services/actions) to a target so headers are usable.
target_link_libraries(controller "${typesupport_target}")



# --------------------------------------------------
# Main executable
# --------------------------------------------------
add_executable(main src/main.cpp)

ament_target_dependencies(main rclcpp)

target_link_libraries(main
  odometry
  controller
)


# --------------------------------------------------
# Installation
# --------------------------------------------------

# Installs header files to a standard location (include/) for other packages to use.
install(
  DIRECTORY include/
  DESTINATION include
)

# Installs libraries and executables to standard locations (lib or bin) so they can be run or linked.
install(
  TARGETS
    odometry
    controller
    differential_kinematics
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install the executable separately
install(
  TARGETS main
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  DESTINATION share/${PROJECT_NAME}
)

# --------------------------------------------------
# Export
# --------------------------------------------------

# Exports include directories so downstream packages can use your headers.
ament_export_include_directories(include)
# Exports CMake targets for use in other packages (libraries/executables).
ament_export_targets(export_${PROJECT_NAME})
# Exports the dependencies of your package for downstream packages to automatically depend on.
ament_export_dependencies(
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  std_srvs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  utils
  control_toolbox
)

# Marks the end of the package configuration; performs finalization needed by ROS 2 packages.
ament_package()

cmake_minimum_required(VERSION 3.8)
project(utils)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Float32.srv"
  "srv/Int32.srv"
  "srv/Pose.srv"
  DEPENDENCIES 
)

# Library (RENAMED)
add_library(utils_lib src/UtilsGeneral.cpp)

target_include_directories(utils_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Ensure interfaces are built first
add_dependencies(utils_lib
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# Link against generated service types
target_link_libraries(utils_lib
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install library
install(
  TARGETS utils_lib
  EXPORT utilsTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export
ament_export_targets(utilsTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)

ament_package()

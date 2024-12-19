cmake_minimum_required(VERSION 3.5)
project(ewellix_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

find_package(ament_cmake REQUIRED)
find_package(ewellix_interfaces REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial REQUIRED)

include_directories(include)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ewellix_interfaces
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

# Plugin
add_library(
  ewellix_driver_plugin
  SHARED
  src/ewellix_driver_plugin/hardware_interface.cpp
)
target_link_libraries(
  ewellix_driver_plugin
  ewellix_serial
)
target_include_directories(
  ewellix_driver_plugin
  PRIVATE
  include
)
ament_target_dependencies(
  ewellix_driver_plugin
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)

# Serial API
add_library(
  ewellix_serial
  SHARED
  src/ewellix_serial/ewellix_serial.cpp
)
target_include_directories(
  ewellix_serial
  PRIVATE
  include
)
ament_target_dependencies(
  ewellix_serial
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  serial
)

# Node
add_executable(
  ewellix_node
  src/ewellix_node/ewellix_node.cpp
)
target_link_libraries(
  ewellix_node
  ewellix_serial
)
target_include_directories(
  ewellix_node
  PRIVATE
  include
)
ament_target_dependencies(
  ewellix_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# INSTALL
install(
  TARGETS ewellix_driver_plugin ewellix_serial
  DESTINATION lib
)
install(
  TARGETS ewellix_node
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ewellix_driver_plugin
  ewellix_serial
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
  serial
)

ament_package()

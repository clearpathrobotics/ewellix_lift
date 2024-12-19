cmake_minimum_required(VERSION 3.5)
project(ewellix_interfaces)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Error.msg"
  "msg/State.msg"
  "msg/Status.msg"
 )

ament_package()

cmake_minimum_required(VERSION 3.5)
project(ewellix_examples)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ewellix_interfaces REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/up_down
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()

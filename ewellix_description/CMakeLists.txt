cmake_minimum_required(VERSION 3.5)
project(ewellix_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config meshes urdf launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(delta_arms_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch meshes urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

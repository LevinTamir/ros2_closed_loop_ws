cmake_minimum_required(VERSION 3.8)
project(closed_loop_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY meshes urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

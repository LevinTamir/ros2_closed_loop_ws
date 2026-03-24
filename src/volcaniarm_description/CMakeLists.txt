cmake_minimum_required(VERSION 3.8)
project(volcaniarm_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch meshes urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

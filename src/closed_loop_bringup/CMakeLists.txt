cmake_minimum_required(VERSION 3.8)
project(closed_loop_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/description_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME description_publisher
)

install(
  PROGRAMS scripts/transformer_with_preview.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME transformer_with_preview
)

ament_package()

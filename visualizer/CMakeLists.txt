cmake_minimum_required(VERSION 3.5)
project(visualizer)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

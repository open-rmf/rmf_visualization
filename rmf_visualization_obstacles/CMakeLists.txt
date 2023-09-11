cmake_minimum_required(VERSION 3.8)
project(rmf_visualization_obstacles)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rmf_obstacle_msgs REQUIRED)
find_package(rmf_visualization_msgs REQUIRED)

#===============================================================================
add_library(obstacle_visualizer SHARED src/ObstacleVisualizer.cpp)

ament_target_dependencies(obstacle_visualizer
  PUBLIC
    rclcpp
    rclcpp_components
    visualization_msgs
    geometry_msgs
    vision_msgs
    rmf_obstacle_msgs
    rmf_visualization_msgs
)

target_compile_features(obstacle_visualizer INTERFACE cxx_std_17)

rclcpp_components_register_node(obstacle_visualizer
  PLUGIN "ObstacleVisualizer"
  EXECUTABLE obstacle_visualizer_node)

#===============================================================================
if(BUILD_TESTING)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(rmf_utils REQUIRED)
  find_file(uncrustify_config_file
    NAMES "rmf_code_style.cfg"
    PATHS "${rmf_utils_DIR}/../../../share/rmf_utils/")

  ament_uncrustify(
    ARGN src
    CONFIG_FILE ${uncrustify_config_file}
    MAX_LINE_LENGTH 80
  )
endif()

#===============================================================================
install(
  TARGETS
    obstacle_visualizer
  RUNTIME DESTINATION lib/rmf_visualization_obstacles
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_package()

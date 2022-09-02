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

target_link_libraries(obstacle_visualizer
  PRIVATE
    rclcpp::rclcpp
    ${rclcpp_components_LIBRARIES}
    ${visualization_msgs_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${vision_msgs_LIBRARIES}
    ${rmf_obstacle_msgs_LIBRARIES}
    ${rmf_visualization_msgs_LIBRARIES}
)

target_include_directories(obstacle_visualizer
  PUBLIC
    ${rclcpp_INCLUDE_DIRS}
    ${rclcpp_components_INCLUDE_DIRS}
    ${visualization_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${vision_msgs_INCLUDE_DIRS}
    ${rmf_obstacle_msgs_INCLUDE_DIRS}
    ${rmf_visualization_msgs_INCLUDE_DIRS}
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

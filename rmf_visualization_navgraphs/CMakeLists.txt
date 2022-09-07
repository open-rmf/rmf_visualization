cmake_minimum_required(VERSION 3.8)
project(rmf_visualization_navgraphs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

set(dep_pkgs
  rclcpp
  rclcpp_components
  rmf_building_map_msgs
  rmf_fleet_msgs
  rmf_visualization_msgs
  visualization_msgs
  rmf_traffic
  rmf_traffic_ros2
  geometry_msgs
)
foreach(pkg ${dep_pkgs})
  find_package(${pkg} REQUIRED)
endforeach()


#===============================================================================
add_library(navgraph_visualizer SHARED src/NavGraphVisualizer.cpp)

target_link_libraries(navgraph_visualizer
  PRIVATE
    rclcpp::rclcpp
    ${rclcpp_components_LIBRARIES}
    ${rmf_fleet_msgs_LIBRARIES}
    ${rmf_building_map_msgs_LIBRARIES}
    ${rmf_visualization_msgs_LIBRARIES}
    ${visualization_msgs_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    rmf_traffic::rmf_traffic
    rmf_traffic_ros2::rmf_traffic_ros2

)

target_include_directories(navgraph_visualizer
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${rclcpp_INCLUDE_DIRS}
    ${rclcpp_components_INCLUDE_DIRS}
    ${rmf_fleet_msgs_INCLUDE_DIRS}
    ${rmf_building_map_msgs_INCLUDE_DIRS}
    ${rmf_visualization_msgs_INCLUDE_DIRS}
    ${visualization_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${rmf_traffic_INCLUDE_DIRS}
    ${rmf_traffic_ros2_INCLUDE_DIRS}
)

target_compile_features(navgraph_visualizer INTERFACE cxx_std_17)

rclcpp_components_register_node(navgraph_visualizer
  PLUGIN "NavGraphVisualizer"
  EXECUTABLE navgraph_visualizer_node)

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
  TARGETS navgraph_visualizer
  RUNTIME DESTINATION lib/rmf_visualization_navgraphs
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_package()

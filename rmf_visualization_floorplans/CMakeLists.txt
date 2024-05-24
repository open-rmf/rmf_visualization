cmake_minimum_required(VERSION 3.8)
project(rmf_visualization_floorplans)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

find_package(eigen3_cmake_module REQUIRED)
set(dep_pkgs
  rclcpp
  rclcpp_components
  OpenCV
  Eigen3
  rmf_building_map_msgs
  rmf_visualization_msgs
  nav_msgs
  geometry_msgs
)
foreach(pkg ${dep_pkgs})
  find_package(${pkg} REQUIRED)
endforeach()

#===============================================================================
add_library(floorplan_visualizer SHARED src/FloorplanVisualizer.cpp)

ament_target_dependencies(floorplan_visualizer
  PUBLIC
    rclcpp
    Eigen3
    rclcpp_components
    OpenCV
    rmf_building_map_msgs
    rmf_visualization_msgs
    nav_msgs
    geometry_msgs
)


target_include_directories(floorplan_visualizer
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_compile_features(floorplan_visualizer INTERFACE cxx_std_17)

rclcpp_components_register_node(floorplan_visualizer
  PLUGIN "FloorplanVisualizer"
  EXECUTABLE floorplan_visualizer_node)

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
  TARGETS floorplan_visualizer
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_package()

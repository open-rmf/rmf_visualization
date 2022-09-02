cmake_minimum_required(VERSION 3.8)
project(rmf_visualization_floorplans)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

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

target_link_libraries(floorplan_visualizer
  PRIVATE
    rclcpp::rclcpp
    Eigen3::Eigen
    ${rclcpp_components_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${rmf_building_map_msgs_LIBRARIES}
    ${rmf_visualization_msgs_LIBRARIES}
    ${nav_msgs_LIBRARIES}
    ${geometry_msgs_LIBRARIES}

)

target_include_directories(floorplan_visualizer
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${rclcpp_INCLUDE_DIRS}
    ${rclcpp_components_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${rmf_fleet_msgs_INCLUDE_DIRS}
    ${rmf_building_map_msgs_INCLUDE_DIRS}
    ${rmf_visualization_msgs_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
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

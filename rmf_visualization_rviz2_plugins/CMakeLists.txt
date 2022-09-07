cmake_minimum_required(VERSION 3.5)
project(rmf_visualization_rviz2_plugins)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(POLICY CMP0072)
  cmake_policy(SET CMP0072 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

# Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmf_door_msgs REQUIRED)
find_package(rmf_lift_msgs REQUIRED)
find_package(rmf_visualization_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rmf_traffic_ros2 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Test)

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

add_definitions(-DQT_NO_KEYWORDS)

add_library(${PROJECT_NAME} SHARED
  src/SchedulePanel.cpp
  src/LiftPanel.cpp
  src/DoorPanel.cpp
  src/NegotiationModel.cpp
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${QT5_LIBRARIES}
    ${rclcpp_LIBRARIES}
    ${rmf_door_msgs_LIBRARIES}
    ${rmf_lift_msgs_LIBRARIES}
    ${rmf_visualization_msgs_LIBRARIES}
    rviz_common::rviz_common
    ${rviz_rendering_LIBRARIES}
    ${rviz_default_plugins_LIBRARIES}
    ${Qt5Widgets_LIBRARIES}
    rmf_traffic_ros2::rmf_traffic_ros2
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${rclcpp_INCLUDE_DIRS}
    ${rmf_door_msgs_INCLUDE_DIRS}
    ${rmf_lift_msgs_INCLUDE_DIRS}
    ${rmf_visualization_msgs_INCLUDE_DIRS}
    ${rviz_common_INCLUDE_DIRS}
    ${rviz_rendering_INCLUDE_DIRS}
    ${rviz_default_plugins_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${QT5_INCLUDE_DIRS}
)

target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_compile_definitions(${PROJECT_NAME} PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_export_include_directories(include)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
  Qt5
  rviz_common
  rclcpp
  Qt5Widgets
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()

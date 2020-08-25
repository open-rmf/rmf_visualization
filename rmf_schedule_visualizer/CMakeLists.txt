cmake_minimum_required(VERSION 3.5)
project(rmf_schedule_visualizer)
set(CMAKE_EXPORT_COMPILE_COMMANDS on)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rmf_traffic REQUIRED)
find_package(rmf_traffic_ros2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(websocketpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system date_time regex random)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rmf_traffic_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rmf_schedule_visualizer_msgs REQUIRED)
find_package(building_map_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

#===============================================================================
#file(GLOB_RECURSE visualizer_srcs "src/*.cpp")
#add_executable(schedule_visualizer ${visualizer_srcs})
add_executable(schedule_visualizer 
  src/Server.cpp
  src/VisualizerData.cpp
  src/main.cpp)

target_link_libraries(schedule_visualizer
  PUBLIC
    rmf_traffic_ros2::rmf_traffic_ros2
    rmf_traffic::rmf_traffic
    ${rclcpp_LIBRARIES}
    Boost::system
    Boost::date_time
    Boost::regex
    Boost::random
    ${rmf_traffic_msgs_LIBRARIES}
)

target_include_directories(schedule_visualizer
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${rclcpp_INCLUDE_DIRS}
    ${WEBSOCKETPP_INCLUDE_DIR}
)
#===============================================================================
add_executable(rviz2 src/VisualizerData.cpp src/main_rviz.cpp)

target_link_libraries(rviz2
  PUBLIC
    rmf_traffic_ros2::rmf_traffic_ros2
    rmf_traffic::rmf_traffic
    ${rclcpp_LIBRARIES}
    ${visualization_msgs_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    Boost::system
    Boost::date_time
    Boost::regex
    Boost::random
    ${rmf_traffic_msgs_LIBRARIES}
    ${rmf_schedule_visualizer_msgs_LIBRARIES}
    ${building_map_msgs_LIBRARIES}
)

target_include_directories(rviz2
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${rclcpp_INCLUDE_DIRS}
    ${visualization_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${rmf_schedule_visualizer_msgs_INCLUDE_DIRS}
    ${building_map_msgs_INCLUDE_DIRS}
)

#===============================================================================

install(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS schedule_visualizer
    rviz2
  EXPORT schedule_visualizer
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

add_subdirectory(test)

ament_package()

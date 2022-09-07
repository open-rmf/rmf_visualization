cmake_minimum_required(VERSION 3.5)
project(rmf_visualization_schedule)
set(CMAKE_EXPORT_COMPILE_COMMANDS on)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rmf_traffic 2 REQUIRED)
find_package(rmf_traffic_ros2 2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(websocketpp REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rmf_traffic_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rmf_visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads)

if(BUILD_TESTING)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(rmf_utils REQUIRED)
  find_file(uncrustify_config_file
    NAMES "rmf_code_style.cfg"
    PATHS "${rmf_utils_DIR}/../../../share/rmf_utils/")

  ament_uncrustify(
    ARGN include/rmf_visualization_schedule src test
    CONFIG_FILE ${uncrustify_config_file}
    MAX_LINE_LENGTH 80
  )
endif()

#===============================================================================
file(GLOB_RECURSE core_lib_srcs "src/rmf_visualization_schedule/*.cpp")
add_library(rmf_visualization_schedule SHARED ${core_lib_srcs})

target_link_libraries(rmf_visualization_schedule
  PUBLIC
    rmf_traffic_ros2::rmf_traffic_ros2
    rmf_traffic::rmf_traffic
    ${rclcpp_LIBRARIES}
    ${websocketpp_LIBRARIES}
    OpenSSL::SSL
    OpenSSL::Crypto
    Threads::Threads
    ${rmf_traffic_msgs_LIBRARIES}
)

target_include_directories(rmf_visualization_schedule
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${rclcpp_INCLUDE_DIRS}
    ${WEBSOCKETPP_INCLUDE_DIR}
    ${OpenSSL_INCLUDE_DIR}
)

ament_export_dependencies(
  rmf_traffic
  rmf_traffic_ros2
  rclcpp
  rmf_traffic_msgs
  websocketpp
  OpenSSL)
ament_export_include_directories(include)
ament_export_targets(rmf_visualization_schedule HAS_LIBRARY_TARGET)

#===============================================================================
add_library(schedule_visualizer SHARED src/ScheduleVisualizer.cpp)

target_link_libraries(schedule_visualizer
  PRIVATE
    rmf_visualization_schedule
    ${rclcpp_components_LIBRARIES}
    ${visualization_msgs_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${rmf_visualization_msgs_LIBRARIES}
)

target_include_directories(schedule_visualizer
  PRIVATE
    ${rclcpp_INCLUDE_DIRS}
    ${rclcpp_components_INCLUDE_DIRS}
    ${visualization_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${rmf_visualization_msgs_INCLUDE_DIRS}
)

target_compile_features(schedule_visualizer INTERFACE cxx_std_17)

rclcpp_components_register_node(schedule_visualizer
  PLUGIN "ScheduleVisualizer"
  EXECUTABLE schedule_visualizer_node)

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
  TARGETS rmf_visualization_schedule
  EXPORT rmf_visualization_schedule
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(
  TARGETS schedule_visualizer
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

add_subdirectory(test)

ament_package()

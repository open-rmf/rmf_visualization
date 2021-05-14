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

find_package(Git QUIET)
if(GIT_FOUND)
# Update submodules as needed
    option(GIT_SUBMODULE "Check submodules during build" ON)
    if(GIT_SUBMODULE)
        message(STATUS "Submodule update")
        execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
                        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                        RESULT_VARIABLE GIT_SUBMOD_RESULT)
        if(NOT GIT_SUBMOD_RESULT EQUAL "0")
            message(FATAL_ERROR "git submodule update --init failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
        endif()
    endif()
endif()

if(NOT EXISTS "${PROJECT_SOURCE_DIR}/jwt-cpp/CMakeLists.txt")
    message(FATAL_ERROR "The submodules were not downloaded! GIT_SUBMODULE was turned off or failed. Please update submodules and try again.")
endif()

include(GNUInstallDirs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rmf_traffic REQUIRED)
find_package(rmf_traffic_ros2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED )
find_package(websocketpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rmf_traffic_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rmf_visualization_msgs REQUIRED)
find_package(rmf_building_map_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_subdirectory(jwt-cpp)

find_package(rmf_cmake_uncrustify QUIET)
if(BUILD_TESTING AND rmf_cmake_uncrustify_FOUND)
  find_file(uncrustify_config_file 
    NAMES "rmf_code_style.cfg"
    PATHS "${rmf_utils_DIR}/../../../share/rmf_utils/")

  rmf_uncrustify(
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
    ${rmf_traffic_msgs_LIBRARIES}
    jwt-cpp
)

target_include_directories(rmf_visualization_schedule
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${rclcpp_INCLUDE_DIRS}
    ${WEBSOCKETPP_INCLUDE_DIR}
)

ament_export_targets(rmf_visualization_schedule HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rmf_traffic
  rmf_traffic_ros2
  rclcpp
  rmf_traffic_msgs
  websocketpp)

#===============================================================================
add_executable(schedule_visualizer src/main.cpp)

target_link_libraries(schedule_visualizer
  PRIVATE
  rmf_visualization_schedule
    ${OpenCV_LIBRARIES}
    ${visualization_msgs_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${nav_msgs_LIBRARIES}
    ${rmf_visualization_msgs_LIBRARIES}
    ${rmf_building_map_msgs_LIBRARIES}
    jwt-cpp
)

target_include_directories(schedule_visualizer
  PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${visualization_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    ${rmf_visualization_msgs_INCLUDE_DIRS}
    ${rmf_building_map_msgs_INCLUDE_DIRS}
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
  TARGETS rmf_visualization_schedule schedule_visualizer
  EXPORT rmf_visualization_schedule
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

add_subdirectory(test)

ament_package()

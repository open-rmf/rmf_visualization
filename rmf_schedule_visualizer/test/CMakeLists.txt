
find_package(Eigen3 REQUIRED)

add_executable(submit_trajectory SubmitTrajectory.cpp)

target_link_libraries(submit_trajectory
  PUBLIC
    rmf_traffic::rmf_traffic
    rmf_traffic_ros2::rmf_traffic_ros2
    ${rmf_traffic_msgs_LIBRARIES}
)

target_include_directories(submit_trajectory
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
)

install(
  TARGETS submit_trajectory
  EXPORT submit_trajectory
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

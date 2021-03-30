
find_package(Eigen3 REQUIRED)

add_executable(test_submit_trajectory SubmitTrajectory.cpp)

target_link_libraries(test_submit_trajectory
  PUBLIC
    rmf_traffic::rmf_traffic
    rmf_traffic_ros2::rmf_traffic_ros2
    ${rmf_traffic_msgs_LIBRARIES}
)

target_include_directories(test_submit_trajectory
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
)

install(
  TARGETS test_submit_trajectory
  EXPORT test_submit_trajectory
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef SRC__SCHEDULEVISUALIZER_HPP
#define SRC__SCHEDULEVISUALIZER_HPP

#include <rclcpp/rclcpp.hpp>

#include <rmf_visualization_schedule/CommonData.hpp>
#include <rmf_visualization_schedule/ScheduleDataNode.hpp>
#include <rmf_visualization_schedule/TrajectoryServer.hpp>

#include <rmf_traffic/schedule/Viewer.hpp>
#include <rmf_traffic/Time.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <rmf_visualization_msgs/msg/rviz_param.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <unordered_set>
#include <thread>

//==============================================================================
class ScheduleVisualizer : public rclcpp::Node
{
public:
  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using Point = geometry_msgs::msg::Point;
  using RequestParam = rmf_visualization_schedule::RequestParam;
  using Element = rmf_traffic::schedule::Viewer::View::Element;
  using RvizParamMsg = rmf_visualization_msgs::msg::RvizParam;
  using Color = std_msgs::msg::ColorRGBA;
  using ScheduleDataNode = rmf_visualization_schedule::ScheduleDataNode;
  using TrajectoryServer = rmf_visualization_schedule::TrajectoryServer;

  ScheduleVisualizer(const rclcpp::NodeOptions& options =
    rclcpp::NodeOptions());

  ~ScheduleVisualizer();

private:

  void timer_callback();

  void delete_marker(const uint64_t id, MarkerArray& marker_array);

  Marker make_location_marker(
    const Element& element,
    const RequestParam& param,
    const Eigen::Vector3d& position,
    const double radius,
    const double height,
    int32_t id,
    const Color& color);

  Marker make_footprint_marker(
    const Element& element,
    const RequestParam& param,
    const Eigen::Vector3d& position);

  Marker make_vicinity_marker(
    const Element& element,
    const RequestParam& param,
    const Eigen::Vector3d& position);

  void add_location_markers(
    std::vector<visualization_msgs::msg::Marker>& array,
    const Element& element,
    const RequestParam& param);

  Marker make_path_marker(
    Element element,
    const RequestParam param);

  Point make_point(const Eigen::Vector3d tp, bool z = false);

  bool is_conflict(int64_t id);

  Color make_color(float r, float g, float b, float a = 1.0);

  std::shared_ptr<ScheduleDataNode> _schedule_data_node;
  std::shared_ptr<TrajectoryServer> _trajectory_server;
  std::unordered_set<uint64_t> _marker_tracker;
  std::vector<rmf_traffic::Trajectory> _trajectories;
  std::vector<Element> _elements;
  double _rate;
  double _path_width;
  std::chrono::nanoseconds _timer_period;

  rclcpp::TimerBase::SharedPtr _timer;
  rclcpp::Publisher<MarkerArray>::SharedPtr _schedule_markers_pub;
  rclcpp::Subscription<RvizParamMsg>::SharedPtr _param_sub;

  RvizParamMsg::ConstSharedPtr _rviz_param;

  std::thread _spin_thread;
};

#endif // SRC__SCHEDULEVISUALIZER_HPP

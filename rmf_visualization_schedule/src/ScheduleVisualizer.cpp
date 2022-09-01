/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include "ScheduleVisualizer.hpp"

#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/Motion.hpp>

#include <rmf_traffic_ros2/Time.hpp>
#include <rmf_traffic_ros2/StandardNames.hpp>

#include <rclcpp_components/register_node_macro.hpp>


using namespace std::chrono_literals;

//==============================================================================
namespace {
struct quaternion
{
  double w, x, y, z;
};


quaternion convert(double yaw, double pitch = 0, double roll = 0)
{
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  quaternion q;
  q.w = cy * cp * cr + sy * sp * sr;
  q.x = cy * cp * sr - sy * sp * cr;
  q.y = sy * cp * sr + cy * sp * cr;
  q.z = sy * cp * cr - cy * sp * sr;

  return q;
}

builtin_interfaces::msg::Duration convert(rmf_traffic::Duration duration)
{
  builtin_interfaces::msg::Duration result;
  result.sec = std::chrono::duration_cast<
    std::chrono::seconds>(duration).count();
  const auto nanoseconds = duration - std::chrono::seconds(result.sec);
  result.nanosec = nanoseconds.count();

  return result;
}
} // anonymous namespace

//==============================================================================
ScheduleVisualizer::ScheduleVisualizer(
  const rclcpp::NodeOptions& options)
: Node("schedule_visualizer_node", options),
  _schedule_data_node(nullptr),
  _trajectory_server(nullptr)
{
  _rate = this->declare_parameter("rate", 1.0);
  _rate = std::max(0.1, _rate);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter rate to %f", _rate
  );

  _path_width = this->declare_parameter("path_width", 0.2);
  _path_width = std::max(0.1, _path_width);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter path_width to %f", _path_width
  );

  auto map_name = this->declare_parameter("initial_map_name", "L1");
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter initial_map_name to %s", map_name.c_str()
  );

  const std::size_t wait_durations_secs = this->declare_parameter(
    "wait_secs", 10);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter wait_secs to %ld", wait_durations_secs
  );

  uint16_t port = this->declare_parameter(
    "port", 8006);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter port to %d", port
  );

  uint retained_history_count = this->declare_parameter(
    "retained_history_count", 0);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter retained_history_count to %i", retained_history_count
  );

  _schedule_data_node = rmf_visualization_schedule::ScheduleDataNode::make(
    "schedule_data_node",
    std::chrono::seconds(wait_durations_secs),
    this->get_node_options()
  );

  if (_schedule_data_node == nullptr)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to create a Mirror of the RMF Schedule Database after waiting "
      "for %ld seconds. Please ensur rmf_traffic_schedule node is running. "
      "No schedule markers will be published to RViz.",
      wait_durations_secs
    );
  }
  else
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Successfully connected to rmf_traffic_schedule node."
    );

    _spin_thread = std::thread(
      [=]()
      {
        rclcpp::spin(_schedule_data_node);
      }
    );

    auto negotiation = _schedule_data_node->get_negotiation();
    negotiation->set_retained_history_count(retained_history_count);
    _trajectory_server = TrajectoryServer::make(
      port, _schedule_data_node);
    if (_trajectory_server == nullptr)
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Unable to initialize TrajectoryServer on port %d", port
      );
    }
  }

  _rviz_param = std::make_shared<RvizParamMsg>(
    rmf_visualization_msgs::build<RvizParamMsg>()
    .map_name(std::move(map_name))
    .query_duration(600)
    .start_duration(0)
  );

  _timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double, std::ratio<1>>(1.0 / _rate));
  _timer = this->create_wall_timer(
    _timer_period,
    [=]()
    {
      if (this->_schedule_data_node != nullptr)
        this->timer_callback();
    }
  );

  // Create publisher for schedule markers
  _schedule_markers_pub = this->create_publisher<MarkerArray>(
    "schedule_markers",
    rclcpp::QoS(10).reliable());

  // Create subscriber for rviz_param in separate thread
  _param_sub = this->create_subscription<RvizParamMsg>(
    "rmf_visualization/parameters",
    rclcpp::QoS(10).reliable(),
    [&](RvizParamMsg::ConstSharedPtr msg)
    {
      if (msg->map_name.empty())
        return;
      _rviz_param = msg;
      RCLCPP_DEBUG(this->get_logger(), "RViz parameters updated");
    }
  );
}

//==============================================================================
ScheduleVisualizer::~ScheduleVisualizer()
{
  if (_spin_thread.joinable())
    _spin_thread.join();
}
//==============================================================================
void ScheduleVisualizer::timer_callback()
{
  MarkerArray marker_array;

  RequestParam query_param;
  query_param.map_name = _rviz_param->map_name;
  query_param.start_time = _schedule_data_node->now();
  query_param.finish_time = query_param.start_time +
    std::chrono::seconds(_rviz_param->query_duration);

  _elements = _schedule_data_node->get_elements(query_param);

  RequestParam traj_param;
  traj_param.map_name = query_param.map_name;
  traj_param.start_time = query_param.start_time +
    std::chrono::seconds(_rviz_param->start_duration);
  traj_param.finish_time = query_param.finish_time;

  // Store the ids of active trajectories
  std::vector<uint64_t> active_id;

  // For each trajectory create two markers
  // 1) Current position
  // 2) Path until param.finish_time
  for (const auto& element : _elements)
  {
    active_id.push_back(element.participant);

    if (element.route->trajectory().find(traj_param.start_time) !=
      element.route->trajectory().end())
    {
      add_location_markers(marker_array.markers, element, traj_param);
    }
    if (traj_param.start_time < *element.route->trajectory().finish_time())
    {
      auto path_marker = make_path_marker(element, traj_param);
      marker_array.markers.push_back(path_marker);

      // Add id to _marker_tracker
      if (_marker_tracker.find(element.participant) == _marker_tracker.end())
        _marker_tracker.insert(element.participant);
    }
  }

  // Add deletion markers for trajectories no longer active
  std::unordered_set<uint64_t> removed_markers;
  for (const auto& marker : _marker_tracker)
  {
    if (std::find(active_id.begin(), active_id.end(), marker)
      == active_id.end())
    {
      delete_marker(marker, marker_array);
      removed_markers.insert(marker);
    }
  }

  for (const auto& r : removed_markers)
    _marker_tracker.erase(r);

  // Publish marker_array
  if (!marker_array.markers.empty())
  {
    RCLCPP_DEBUG(this->get_logger(),
      "Publishing marker array of size: %ld",
      marker_array.markers.size());
    _schedule_markers_pub->publish(std::move(marker_array));
  }
}

//==============================================================================
void ScheduleVisualizer::delete_marker(
  const uint64_t id, MarkerArray& marker_array)
{
  Marker marker_msg;
  marker_msg.header.frame_id = "map";
  marker_msg.header.stamp = rmf_traffic_ros2::convert(
    _schedule_data_node->now());
  marker_msg.ns = "trajectory";
  marker_msg.id = id;
  marker_msg.type = marker_msg.CYLINDER;
  marker_msg.action = marker_msg.DELETEALL;
  marker_array.markers.push_back(marker_msg);

  // Add a delete marker for the path
  marker_msg.id = -1 * id;
  marker_msg.type = marker_msg.LINE_STRIP;
  marker_array.markers.push_back(marker_msg);
}

//==============================================================================
auto ScheduleVisualizer::make_location_marker(
  const Element& element,
  const RequestParam& param,
  const Eigen::Vector3d& position,
  const double radius,
  const double height,
  int32_t id,
  const Color& color) -> Marker
{
  Marker marker_msg;

  marker_msg.header.frame_id = "map";
  marker_msg.header.stamp = rmf_traffic_ros2::convert(param.start_time);
  marker_msg.ns = "participant location " +
    std::to_string(element.participant);
  marker_msg.id = id;
  marker_msg.type = marker_msg.CYLINDER;
  marker_msg.action = marker_msg.ADD;

  marker_msg.pose.position.x = position[0];
  marker_msg.pose.position.y = position[1];
  marker_msg.pose.position.z = 0;

  auto quat = convert(position[2]);
  marker_msg.pose.orientation.x = quat.x;
  marker_msg.pose.orientation.y = quat.y;
  marker_msg.pose.orientation.z = quat.z;
  marker_msg.pose.orientation.w = quat.w;

  // Set the scale of the marker
  marker_msg.scale.x = 2.0 * radius;
  marker_msg.scale.y = 2.0 * radius;
  marker_msg.scale.z = height;

  // Set the color of the marker
  marker_msg.color = color;

  // Set the lifetime of the marker
  if (_rate <= 1)
    marker_msg.lifetime = convert(_timer_period);
  else
  {
    builtin_interfaces::msg::Duration duration;
    duration.sec = 1;
    duration.nanosec = 0;
    marker_msg.lifetime = duration;
  }

  return marker_msg;
}

//==============================================================================
auto ScheduleVisualizer::make_footprint_marker(
  const Element& element,
  const RequestParam& param,
  const Eigen::Vector3d& position) -> Marker
{
  const double r_footprint =
    static_cast<const rmf_traffic::geometry::Circle&>(
    element.description.profile().footprint()->source()).get_radius();

  return make_location_marker(element,
      param, position, r_footprint, 1.0, 0, make_color(1.0, 1.0, 0, 0.7));
}

//==============================================================================
auto ScheduleVisualizer::make_vicinity_marker(
  const Element& element,
  const RequestParam& param,
  const Eigen::Vector3d& position) -> Marker
{
  const double r_vicinity =
    static_cast<const rmf_traffic::geometry::Circle&>(
    element.description.profile().vicinity()->source()).get_radius();

  return make_location_marker(element,
      param, position, r_vicinity, 0.5, 1, make_color(0.5, 1.0, 0.9, 0.5));
}

//==============================================================================
void ScheduleVisualizer::add_location_markers(
  std::vector<visualization_msgs::msg::Marker>& array,
  const Element& element,
  const RequestParam& param)
{
  // TODO Link the color, shape and size of marker to profile of trajectory
  const auto& trajectory = element.route->trajectory();

  // Find the pose of the markers
  const auto it = trajectory.find(param.start_time);
  auto begin = it;
  if (begin != trajectory.begin())
    --begin;
  auto end = it; ++end;
  auto motion = rmf_traffic::Motion::compute_cubic_splines(begin, end);
  Eigen::Vector3d position = motion->compute_position(param.start_time);

  array.push_back(make_footprint_marker(element, param, position));

  if (element.description.profile().vicinity() !=
    element.description.profile().footprint())
    array.push_back(make_vicinity_marker(element, param, position));
}

//==============================================================================
auto ScheduleVisualizer::make_path_marker(
  Element element,
  const RequestParam param) -> Marker
{
  // TODO Link the color, shape and size of marker to profile of trajectory
  const auto& trajectory = element.route->trajectory();
  const bool conflict = is_conflict(element.participant);

  Marker marker_msg;

  marker_msg.header.frame_id = "map";
  marker_msg.header.stamp = rmf_traffic_ros2::convert(param.start_time);
  marker_msg.ns = "participant " + std::to_string(element.participant);
  marker_msg.id = element.route_id;
  marker_msg.type = marker_msg.LINE_STRIP;
  marker_msg.action = marker_msg.MODIFY;

  marker_msg.pose.orientation.w = 1;

  // Set the scale of the marker
  marker_msg.scale.x = _path_width;

  // Set the color
  if (conflict)
  {
    marker_msg.color = make_color(1.0, 0, 0, 0.7);
  }
  else
  {
    marker_msg.color = make_color(0.0, 1.0, 0, 0.7);
  }

  if (_rate <= 1)
    marker_msg.lifetime = convert(_timer_period);
  else
  {
    builtin_interfaces::msg::Duration duration;
    duration.sec = 1;
    duration.nanosec = 0;
    marker_msg.lifetime = std::move(duration);
  }

  const auto t_start_time = *trajectory.start_time();
  const auto start_time = std::max(t_start_time, param.start_time);
  const auto t_finish_time = *trajectory.finish_time();
  const auto end_time = std::min(t_finish_time, param.finish_time);

  auto it = trajectory.find(start_time);
  assert(it != trajectory.end());
  assert(trajectory.find(end_time) != trajectory.end());
  auto begin = it;
  if (begin != trajectory.begin())
    --begin;
  auto end = it; ++end;
  const auto motion = rmf_traffic::Motion::compute_cubic_splines(begin, end);
  marker_msg.points.push_back(
    make_point(motion->compute_position(start_time)));

  // Add segment points except the last segment
  for (; it < trajectory.find(end_time); it++)
  {
    assert(it != trajectory.end());
    const Eigen::Vector3d p = it->position();
    marker_msg.points.push_back(make_point(p));
  }

  // Add either last segment point or position at end_time
  if (t_finish_time <= param.finish_time)
  {
    marker_msg.points.push_back(make_point(it->position()));
  }
  else
  {
    const auto motion =
      rmf_traffic::Motion::compute_cubic_splines(--it, trajectory.end());
    marker_msg.points.push_back(
      make_point(motion->compute_position(end_time)));
  }

  return marker_msg;
}

//==============================================================================
auto ScheduleVisualizer::make_point(const Eigen::Vector3d tp, bool z) -> Point
{
  Point p;
  p.x = tp[0];
  p.y = tp[1];
  p.z = z ? tp[2] : 0.1;
  return p;
}

//==============================================================================
bool ScheduleVisualizer::is_conflict(int64_t id)
{
  const auto& conflicts = _schedule_data_node->get_conflict_ids();
  if (conflicts.find(id) != conflicts.end())
    return true;
  return false;
}

//==============================================================================
auto ScheduleVisualizer::make_color(
  float r, float g, float b, float a) -> Color
{
  return std_msgs::build<Color>()
    .r(r)
    .g(g)
    .b(b)
    .a(a);
}

RCLCPP_COMPONENTS_REGISTER_NODE(ScheduleVisualizer)

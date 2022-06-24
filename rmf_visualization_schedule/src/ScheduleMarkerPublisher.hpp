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

#ifndef SRC__SCHEDULEMARKERPUBLISHER_HPP
#define SRC__SCHEDULEMARKERPUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>

#include <rmf_visualization_schedule/CommonData.hpp>
#include <rmf_visualization_schedule/ScheduleDataNode.hpp>

#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/schedule/Viewer.hpp>
#include <rmf_traffic/Motion.hpp>
#include <rmf_traffic/Time.hpp>

#include <rmf_traffic_ros2/StandardNames.hpp>
#include <rmf_traffic_ros2/Time.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <rmf_visualization_msgs/msg/rviz_param.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <unordered_map>
#include <unordered_set>

using namespace std::chrono_literals;

//==============================================================================
class ScheduleMarkerPublisher : public rclcpp::Node
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

  ScheduleMarkerPublisher(
    const std::string& node_name,
    std::shared_ptr<ScheduleDataNode> schedule_data_node,
    const std::string& map_name,
    const double rate = 1.0,
    const std::string& frame_id = "/map")
  : Node(std::move(node_name)),
    _rate(rate),
    _schedule_data_node(std::move(schedule_data_node)),
    _frame_id(std::move(frame_id))
  {
    _rviz_param = std::make_shared<RvizParamMsg>(
      rmf_visualization_msgs::build<RvizParamMsg>()
      .map_name(std::move(map_name))
      .query_duration(600)
      .start_duration(0)
    );

    // Create a timer with specified rate. This timer runs on the main thread.
    const double period = 1.0/_rate;
    _timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double, std::ratio<1>>(period));
    _timer =
      this->create_wall_timer(_timer_period,
        std::bind(&ScheduleMarkerPublisher::timer_callback, this));

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
        RCLCPP_INFO(this->get_logger(), "Rviz parameters updated");
      }
    );
  }

private:

  void timer_callback()
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

  void delete_marker(const uint64_t id, MarkerArray& marker_array)
  {
    Marker marker_msg;
    marker_msg.header.frame_id = _frame_id; // map
    marker_msg.header.stamp = rmf_traffic_ros2::convert(
      _schedule_data_node->now());
    marker_msg.ns = "trajectory";
    marker_msg.id = id;
    marker_msg.type = marker_msg.CYLINDER;
    marker_msg.action = marker_msg.DELETE;
    marker_array.markers.push_back(marker_msg);

    // Add a delete marker for the path
    marker_msg.id = -1 * id;
    marker_msg.type = marker_msg.LINE_STRIP;
    marker_array.markers.push_back(marker_msg);
  }

  visualization_msgs::msg::Marker make_location_marker(
    const Element& element,
    const RequestParam& param,
    const Eigen::Vector3d& position,
    const double radius,
    const double height,
    int32_t id,
    const Color& color)
  {
    Marker marker_msg;

    marker_msg.header.frame_id = _frame_id; // map
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

  visualization_msgs::msg::Marker make_footprint_marker(
    const Element& element,
    const RequestParam& param,
    const Eigen::Vector3d& position)
  {
    const double r_footprint =
      static_cast<const rmf_traffic::geometry::Circle&>(
      element.description.profile().footprint()->source()).get_radius();

    return make_location_marker(element,
        param, position, r_footprint, 1.0, 0, make_color(1.0, 1.0, 0, 0.7));
  }

  visualization_msgs::msg::Marker make_vicinity_marker(
    const Element& element,
    const RequestParam& param,
    const Eigen::Vector3d& position)
  {
    const double r_vicinity =
      static_cast<const rmf_traffic::geometry::Circle&>(
      element.description.profile().vicinity()->source()).get_radius();

    return make_location_marker(element,
        param, position, r_vicinity, 0.5, 1, make_color(0.5, 1.0, 0.9, 0.5));
  }

  void add_location_markers(
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

  visualization_msgs::msg::Marker make_path_marker(
    Element element,
    const RequestParam param)
  {
    // TODO Link the color, shape and size of marker to profile of trajectory
    const auto& trajectory = element.route->trajectory();
    const bool conflict = is_conflict(element.participant);

    Marker marker_msg;

    marker_msg.header.frame_id = _frame_id; // map
    marker_msg.header.stamp = rmf_traffic_ros2::convert(param.start_time);
    marker_msg.ns = "participant " + std::to_string(element.participant);
    marker_msg.id = element.route_id;
    marker_msg.type = marker_msg.LINE_STRIP;
    marker_msg.action = marker_msg.MODIFY;

    marker_msg.pose.orientation.w = 1;

    // Set the scale of the marker
    marker_msg.scale.x = 0.2;

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
      marker_msg.lifetime = duration;
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

  Point make_point(const Eigen::Vector3d tp, bool z = false)
  {
    Point p;
    p.x = tp[0];
    p.y = tp[1];
    p.z = z ? tp[2] : 0;
    return p;
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

  bool is_conflict(int64_t id)
  {
    const auto& conflicts = _schedule_data_node->get_conflict_ids();
    if (conflicts.find(id) != conflicts.end())
      return true;
    return false;
  }

  Color make_color(float r, float g, float b, float a = 1.0)
  {
    return std_msgs::build<Color>()
      .r(r)
      .g(g)
      .b(b)
      .a(a);
  }

  double _rate;
  std::shared_ptr<ScheduleDataNode> _schedule_data_node;
  std::string _frame_id;
  std::unordered_set<uint64_t> _marker_tracker;
  std::vector<rmf_traffic::Trajectory> _trajectories;
  std::vector<Element> _elements;
  std::chrono::nanoseconds _timer_period;

  rclcpp::TimerBase::SharedPtr _timer;
  rclcpp::Publisher<MarkerArray>::SharedPtr _schedule_markers_pub;
  rclcpp::Subscription<RvizParamMsg>::SharedPtr _param_sub;

  RvizParamMsg::ConstSharedPtr _rviz_param;
};

#endif // SRC__SCHEDULEMARKERPUBLISHER_HPP

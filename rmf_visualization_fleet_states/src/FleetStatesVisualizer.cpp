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


#include "FleetStatesVisualizer.hpp"

#include <rmf_fleet_msgs/msg/robot_state.hpp>

#include <rclcpp_components/register_node_macro.hpp>

//==============================================================================
FleetStatesVisualizer::FleetStatesVisualizer(const rclcpp::NodeOptions& options)
: Node("fleet_states_visualizer", options),
  _next_available_id(0)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Configuring fleet_states_visualizer..."
  );

  _current_level = this->declare_parameter("initial_map_name", "L1");
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter initial_map_name to %s", _current_level.c_str()
  );

  _nose_scale = this->declare_parameter("fleet_state_nose_scale", 0.5);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter fleet_state_nose_scale to %f", _nose_scale
  );

  _param_sub = this->create_subscription<RvizParam>(
    "rmf_visualization/parameters",
    rclcpp::SystemDefaultsQoS(),
    [=](RvizParam::ConstSharedPtr msg)
    {
      if (msg->map_name.empty() || msg->map_name == _current_level)
        return;

      _current_level = msg->map_name;
      publish_markers();
    });

  // Match QoS used in rmf_fleet_adapter
  const auto qos = rclcpp::SystemDefaultsQoS().reliable().keep_last(100);
  _fleet_sub = this->create_subscription<FleetState>(
    "/fleet_states",
    qos,
    [=](FleetState::ConstSharedPtr msg)
    {
      using RobotState = rmf_fleet_msgs::msg::RobotState;
      using Location = rmf_fleet_msgs::msg::Location;

      auto set_body_pose =
      [](const Location& loc, Marker& marker)
      {
        marker.pose.position.x = loc.x;
        marker.pose.position.y = loc.y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.w = 1.0;
      };

      auto set_nose_pose =
      [](const Location& loc, const double radius, Marker& marker)
      {
        marker.pose.position.x = loc.x + radius * std::cos(loc.yaw);
        marker.pose.position.y = loc.y + radius * std::sin(loc.yaw);
        marker.pose.position.z = 0.0;
        marker.pose.orientation.w = 1.0;
      };
      auto set_text_pose =
      [](const Location& loc, const double radius, Marker& marker)
      {
        marker.pose.position.x =
        loc.x + 2.0 * radius * std::cos(loc.yaw - 0.7853);
        marker.pose.position.y =
        loc.y + 2.0 * radius * std::sin(loc.yaw - 0.7853);
        marker.pose.position.z = 0.0;
        marker.pose.orientation.w = 1.0;
      };
      auto fill_markers =
      [&](
        const RobotState& state,
        const double radius,
        const std::size_t id,
        MarkerArray& marker_array)
      {
        const auto& loc = state.location;

        Marker body_marker;
        body_marker.header.frame_id = "map";
        body_marker.header.stamp = state.location.t;
        body_marker.ns = "body";
        body_marker.id = id;
        body_marker.type = body_marker.SPHERE;
        body_marker.action = body_marker.MODIFY;
        set_body_pose(loc, body_marker);
        body_marker.scale.x = 2.0 * radius;
        body_marker.scale.y = 2.0 * radius;
        body_marker.scale.z = 2.0 * radius;
        // TODO(YV): Get the color to match that of the navigation graph
        body_marker.color.r = 1.0;
        body_marker.color.b = 1.0;
        body_marker.color.a = 1.0;
        body_marker.lifetime = rclcpp::Duration(std::chrono::seconds(1));

        auto nose_marker = body_marker;
        nose_marker.ns = "nose";
        set_nose_pose(loc, radius, nose_marker);
        nose_marker.scale.x = this->_nose_scale * radius;
        nose_marker.scale.y = this->_nose_scale * radius;
        nose_marker.scale.z = this->_nose_scale * radius;

        auto text_marker = body_marker;
        text_marker.ns = "name";
        text_marker.type = text_marker.TEXT_VIEW_FACING;
        set_text_pose(loc, radius, text_marker);
        text_marker.text = state.name;
        text_marker.scale.z = 0.3;

        marker_array.markers.push_back(std::move(body_marker));
        marker_array.markers.push_back(std::move(nose_marker));
        marker_array.markers.push_back(std::move(text_marker));
      };

      if (msg->name.empty() || msg->robots.empty())
        return;

      auto marker_array = std::make_unique<MarkerArray>();
      for (const auto& robot : msg->robots)
      {
        if (robot.name.empty() || robot.location.level_name != _current_level)
          continue;

        if (_ids.find(robot.name) == _ids.end())
        {
          _ids.insert({robot.name, _next_available_id});
          ++_next_available_id;
          if (_declared_radius.find(msg->name) == _declared_radius.end())
          {
            this->declare_parameter(msg->name + "_radius", 0.5);
            _declared_radius.insert(msg->name);
          }
        }
        // Attempt to retrieve the footprint radius for this fleet via
        /// ${FLEET_NAME}_radius parameter
        double radius = 0.5;
        this->get_parameter(msg->name + "_radius", radius);
        fill_markers(
          robot,
          radius,
          _ids[robot.name],
          *marker_array
        );
      }
      if (!marker_array->markers.empty())
        _marker_pub->publish(std::move(marker_array));
    });

  _marker_pub = this->create_publisher<MarkerArray>("/fleet_markers", qos);

  RCLCPP_INFO(
    this->get_logger(),
    "Running fleet_states_visualizer..."
  );

}

//==============================================================================
void FleetStatesVisualizer::publish_markers() const
{

}

RCLCPP_COMPONENTS_REGISTER_NODE(FleetStatesVisualizer)

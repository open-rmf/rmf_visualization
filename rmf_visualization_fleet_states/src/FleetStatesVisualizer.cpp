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

#include <rclcpp_components/register_node_macro.hpp>

//==============================================================================
FleetStatesVisualizer::FleetStatesVisualizer(const rclcpp::NodeOptions& options)
: Node("fleet_states_visualizer", options)
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
  const auto qos = rclcpp::SystemDefaultQoS().reliable().keep_last(100);
  _fleet_sub = this->create_subscription<FleetState>(
    "/fleet_states",
    qos,
    [=](FleetState::ConstSharedPtr msg)
    {
      if (msg->name.empty() || msg->robots.empty())
        return;

      for (const auto& robot : msg->robots)
      {
        if (robot.name.empty())
        auto insertion = _markers.insert({robot.name, nullptr});
        if (insertion.second)
        {
          // New robot
          auto robot_marker = std::make_shared<RobotMarker>();
          insertion.first->second = std::move(robot_marker);
        }
        else
        {

        }

      }
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

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

#ifndef SRC__FLEETSTATESVISUALIZER_HPP
#define SRC__FLEETSTATESVISUALIZER_HPP

#include <rclcpp/rclcpp.hpp>

#include <rmf_visualization_msgs/msg/rviz_param.hpp>
#include <rmf_fleet_msgs/msg/fleet_state.hpp>

#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <unordered_map>
#include <unordered_set>

//==============================================================================
class FleetStatesVisualizer : public rclcpp::Node
{
public:
  using RvizParam = rmf_visualization_msgs::msg::RvizParam;
  using FleetState = rmf_fleet_msgs::msg::FleetState;
  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using Color = std_msgs::msg::ColorRGBA;

/// Constructor
  FleetStatesVisualizer(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  void publish_markers() const;

  std::string _current_level;
  rclcpp::Subscription<RvizParam>::SharedPtr _param_sub;
  rclcpp::Subscription<FleetState>::SharedPtr _fleet_sub;
  rclcpp::Publisher<MarkerArray>::SharedPtr _marker_pub;

  // Map robot name to a unique marker id
  std::unordered_map<std::string, std::size_t> _ids;
  std::unordered_set<std::string> _declared_radius;
  double _nose_scale;
  std::size_t _next_available_id;
};


#endif // SRC__FLEETSTATESVISUALIZER_HPP

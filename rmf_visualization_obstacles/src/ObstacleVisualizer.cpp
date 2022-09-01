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


#include "ObstacleVisualizer.hpp"


#include <rclcpp_components/register_node_macro.hpp>

//==============================================================================
ObstacleVisualizer::ObstacleVisualizer(const rclcpp::NodeOptions& options)
: Node("rmf_obstacle_visualizer", options)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Beginning RMF obstacle visualizer node");

  _obstacle_markers_pub = this->create_publisher<MarkerArrayMsg>(
    "fleet_markers",
    rclcpp::QoS(10).reliable());

  // It is okay to capture this by reference here.
  _obstacles_sub = this->create_subscription<ObstaclesMsg>(
    "rmf_obstacles",
    rclcpp::QoS(10).best_effort(),
    [&](std::shared_ptr<const ObstaclesMsg> msg)
    {
      msg_cb(*msg);
    });

  // It is okay to capture this by reference here.
  _param_sub = this->create_subscription<RvizParamMsg>(
    "rmf_visualization/parameters",
    rclcpp::QoS(10),
    [&](std::shared_ptr<const RvizParamMsg> msg)
    {
      if (!msg->map_name.empty())
        _active_level_name = msg->map_name;
    });

  _active_level_name = this->declare_parameter("initial_map_name", "L1");
  _global_fixed_frame = this->declare_parameter("global_fixed_frame", "map");
}

//==============================================================================
void ObstacleVisualizer::msg_cb(const ObstaclesMsg& msg)
{
  if (msg.obstacles.empty())
    return;

  RCLCPP_DEBUG(
    this->get_logger(),
    "Received %ld obstacles",
    msg.obstacles.size());

  std::unique_ptr<MarkerArrayMsg> obstacle_msg =
    std::make_unique<MarkerArrayMsg>();
  for (const auto& obstacle : msg.obstacles)
  {
    if (obstacle.source.empty())
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Received obstacle message with empty source. Ignoring...");
      continue;
    }
    if (obstacle.level_name.empty() ||
      obstacle.level_name != _active_level_name)
    {
      RCLCPP_DEBUG(
        this->get_logger(),
        "Received obstacle in a different level. Ignoring...");
      continue;
    }

    MarkerMsg _msg;
    _msg.header.frame_id = obstacle.header.frame_id;
    _msg.header.stamp = this->get_clock()->now();
    _msg.ns = obstacle.header.frame_id + "_obstacles";
    _msg.text = obstacle.classification;
    _msg.id = obstacle.id;
    _msg.type = _msg.CUBE;
    _msg.action = _msg.ADD;
    _msg.pose = obstacle.bbox.center;
    _msg.scale = obstacle.bbox.size;
    _msg.color = make_color(0, 0.9, 0.3);
    _msg.lifetime = obstacle.lifetime;

    MarkerMsg _text = _msg;
    _text.ns = obstacle.header.frame_id + "_obstacle_texts";
    _text.type = _text.TEXT_VIEW_FACING;
    _text.pose.position.z = _text.pose.position.z + 1.0;
    _text.pose.position.x = _text.pose.position.x + 1.0;
    _text.scale.z = 0.3;
    obstacle_msg->markers.emplace_back(_msg);
    obstacle_msg->markers.emplace_back(_text);
  }

  if (!obstacle_msg->markers.empty())
    _obstacle_markers_pub->publish(std::move(obstacle_msg));
}

//==============================================================================
auto ObstacleVisualizer::make_color(
  float r, float g, float b, float a) const -> Color
{
  Color color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

RCLCPP_COMPONENTS_REGISTER_NODE(ObstacleVisualizer)

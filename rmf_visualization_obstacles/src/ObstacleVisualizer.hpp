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

#ifndef SRC__NAVGRAPHVISUALIZER_HPP
#define SRC__NAVGRAPHVISUALIZER_HPP

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <rmf_obstacle_msgs/msg/obstacles.hpp>
#include <rmf_obstacle_msgs/msg/obstacle.hpp>

#include <rmf_visualization_msgs/msg/rviz_param.hpp>


//==============================================================================
class ObstacleVisualizer : public rclcpp::Node
{
public:
  using ObstaclesMsg = rmf_obstacle_msgs::msg::Obstacles;
  using ObstacleMsg = rmf_obstacle_msgs::msg::Obstacle;
  using MarkerMsg = visualization_msgs::msg::Marker;
  using MarkerArrayMsg = visualization_msgs::msg::MarkerArray;
  using Color = std_msgs::msg::ColorRGBA;
  using RvizParamMsg = rmf_visualization_msgs::msg::RvizParam;

  ObstacleVisualizer(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:

  void msg_cb(const ObstaclesMsg& msg);
  Color make_color(float r, float g, float b, float a = 1.0) const;

  rclcpp::Subscription<ObstaclesMsg>::SharedPtr _obstacles_sub;
  rclcpp::Subscription<RvizParamMsg>::SharedPtr _param_sub;
  rclcpp::Publisher<MarkerArrayMsg>::SharedPtr _obstacle_markers_pub;
  std::string _active_level_name;
  std::string _global_fixed_frame;
};

#endif // SRC__OBSTACLEVISUALZIER_HPP
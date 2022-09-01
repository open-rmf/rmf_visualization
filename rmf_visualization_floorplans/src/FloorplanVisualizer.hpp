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

#ifndef SRC__FLOORPLANVISUALIZER_HPP
#define SRC__FLOORPLANVISUALIZER_HPP

#include <rclcpp/rclcpp.hpp>

#include <rmf_visualization_msgs/msg/rviz_param.hpp>
#include <rmf_building_map_msgs/msg/building_map.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <unordered_map>

//==============================================================================
class FloorplanVisualizer : public rclcpp::Node
{
public:
  using RvizParam = rmf_visualization_msgs::msg::RvizParam;
  using BuildingMap = rmf_building_map_msgs::msg::BuildingMap;
  using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
/// Constructor
  FloorplanVisualizer(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  void publish_grid();

  std::string _current_level;
  rclcpp::Subscription<RvizParam>::SharedPtr _param_sub;
  rclcpp::Subscription<BuildingMap>::SharedPtr _map_sub;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr _floorplan_pub;
  std::unordered_map<std::string, OccupancyGrid> _grids;
};


#endif // SRC__FLOORPLANVISUALIZER_HPP

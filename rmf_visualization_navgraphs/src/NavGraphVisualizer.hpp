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

#include <rmf_building_map_msgs/msg/graph.hpp>
#include <rmf_fleet_msgs/msg/lane_states.hpp>

#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <rmf_visualization_msgs/msg/rviz_param.hpp>

#include <unordered_map>
#include <unordered_set>


//==============================================================================
class NavGraphVisualizer : public rclcpp::Node
{
public:
  using TrafficGraph = rmf_traffic::agv::Graph;
  using NavGraph = rmf_building_map_msgs::msg::Graph;
  using LaneStates = rmf_fleet_msgs::msg::LaneStates;
  using RvizParam = rmf_visualization_msgs::msg::RvizParam;
  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using Color = std_msgs::msg::ColorRGBA;

/// Constructor
  NavGraphVisualizer(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // Data structure to handle markers for a given fleet
  struct FleetNavGraph
  {
    // Map marker id to Marker. Used for lookup and republishing when lane
    // state changes. The id is set to the index of the lane for easy lookup.
    // TODO(YV): Map level name
    using LaneMarkers = std::unordered_map<std::size_t, Marker::SharedPtr>;

    std::string fleet_name;
    std::optional<TrafficGraph> traffic_graph;
    LaneStates::ConstSharedPtr lane_states;
    // Map level name to LaneMarkers for that level. This cache is convenient
    // when switching maps
    std::unordered_map<std::string, LaneMarkers> lane_markers;
    // This map is purely for easy lookup when modifying lane markers
    LaneMarkers all_lane_markers;

    // Map level name to Marker for waypoint names
    std::unordered_map<std::string, std::vector<Marker>> text_markers;
    // Map level name to marker for waypoints
    std::unordered_map<std::string, Marker> waypoint_markers;
    // We store a weak pointer of the node for logging
    std::weak_ptr<rclcpp::Node> node;
    // Color for this fleet
    Color::ConstSharedPtr color;
    double lane_width;
    double waypoint_width;
    double text_size;
    double lane_transparency;

    std::unordered_set<std::size_t> currently_closed_lanes;
    std::unordered_set<std::size_t> currently_speed_limited_lanes;

    FleetNavGraph(
      const std::string& fleet_name,
      std::weak_ptr<rclcpp::Node> node,
      Color::ConstSharedPtr color,
      double lane_width,
      double waypoint_width,
      double text_size,
      double lane_transparency);

    void initialize_markers(
      const NavGraph& navgraph,
      const rclcpp::Time& now);

    // Update the lane markers and return a vector of markers that belong to
    // map_name_filter if provided. If map_name_filter is not provided, then
    // return all lane markers that were updated
    std::vector<Marker::ConstSharedPtr> update_lane_states(
      const LaneStates& lane_states,
      std::optional<std::string> map_name_filter = std::nullopt);

    // Fill marker_array with all markers that are present in given map_name
    void fill_with_markers(
      const std::string& map_name,
      MarkerArray& marker_array,
      const bool delete_markers = false);
  };

  using FleetNavGraphPtr = std::shared_ptr<FleetNavGraph>;

  void publish_map_markers(const bool delete_markers = false);
  void initialize_color_options(const double alpha);
  Color::ConstSharedPtr get_next_color();

  rclcpp::Subscription<RvizParam>::SharedPtr _param_sub;
  rclcpp::Subscription<NavGraph>::SharedPtr _navgraph_sub;
  rclcpp::Subscription<LaneStates>::SharedPtr _lane_states_sub;
  rclcpp::Publisher<MarkerArray>::SharedPtr _marker_pub;

  std::string _current_level;
  // Map fleet name to FleetNavGraphPtr
  std::unordered_map<std::string, FleetNavGraphPtr> _navgraphs;

  // Visualization
  std::size_t _next_color = 0;
  std::vector<Color::ConstSharedPtr> _color_options;
  double _lane_width;
  double _lane_transparency;
  double _waypoint_scale;
  double _text_scale;
};


#endif // SRC__NAVGRAPHVISUALIZER_HPP

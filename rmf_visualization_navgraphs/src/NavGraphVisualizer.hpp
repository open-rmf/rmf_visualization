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


//==============================================================================
class NavGraphVisualizer : public rclcpp::Node
{
public:
using NavGraph = rmf_building_map_msgs::msg::Graph;
using LaneStates = rmf_fleet_msgs::msg::LaneStates;
using RvizParam = rmf_visualization_msgs::msg::RvizParam;
using Marker = rmf_visualization_msgs::msg::Marker;

/// Constructor
NavGraphVisualizer(
	const rclcpp::NodeOptions& options  = rclcpp::NodeOptions());

private:

	struct FleetNavGraph
	{
		std::shared_ptr<rmf_traffic::agv::Graph> traffic_graph;
		std::shared_ptr<const NavGraph> navgraph;
		LaneStates lane_states;
		// Map marker id to Marker. Used for lookup and republishing when lane
		// state changes.
		std::unordered_map<std::size_t, Marker> lane_markers;

		FleetNavGraph::FleetNavGraph()
		{
			traffic_graph = nullptr;
			navgraph = nullptr;
		}
	};
	using FleetNavGraphPtr = std::shared_ptr<FleetNavGraph>;

	void param_cb(std::shared_ptr<const RvizParam> msg);
	void graph_cb(std::shared_ptr<const NavGraph> msg);
	void lane_states_cb(std::shared_ptr<const LaneStates> msg);
	void publish_navgraphs();

	rclcpp::Subscription<RvizParam>::SharedPtr _param_sub;
	rclcpp::Subscription<NavGraph>::SharedPtr _navgraph_sub;
	rclcpp::Subscription<LaneStates>::SharedPtr _lane_states_sub;

	std::string _current_level;
	// Map fleet name to FleetNavGraphPtr
	std::unordered_map<std::string, FleetNavGraphPtr> _navgraphs;
};


#endif // SRC__NAVGRAPHVISUALIZER_HPP
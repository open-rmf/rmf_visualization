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

#include <rmf_traffic/agv/Graph.hpp>

#include "NavGraphVisualizer.hpp"

#include <rmf_building_map_msgs/msg/graph_edge.hpp>

#include <rclcpp_components/register_node_macro.hpp>

//==============================================================================
NavGraphVisualizer::NavGraphVisualizer(const rclcpp::NodeOptions& options)
	: Node("navgraph_visualizer", options)
{
	_current_level = this->declare_parameter("initial_map_nam", "L1");
	RCLCPP_INFO(
		this->get_logger(),
		"Setting parameter initial_map_name to %s", _current_level.c_str());

	// It is okay to capture this by reference here.
	_param_sub = this->create_subscription<RvizParam>(
		"rmf_visualization/parameters",
		rclcpp::SystemDefaultsQoS(),
		[&](std::shared_ptr<const RvizParam> msg)
	{
		if (!msg->map_name.empty())
			_current_level = msg->map_name;

		RCLCPP_INFO(
			this->get_logger(),
			"Publishing navgraphs on level %s", _current_level.c_str());
		publish_navgraphs();
	});

	const auto transient_qos =
		rclcpp::QoS(10).transient_local();
	// It is okay to capture this by reference here.
	_navgraph_sub = this->create_subscription<NavGraph>(
		"/nav_graphs",
		transient_qos,
		[&](std::shared_ptr<const NavGraph> msg)
	{
		if (msg->name.empty())
			return;

		_navgraphs[msg->name] = msg;

		publish_navgraphs();
	});

	RCLCPP_INFO(
		this->get_logger(),
		"NavGraph visualizer is running...");
}

//==============================================================================
void NavGraphVisualizer::param_cb(std::shared_ptr<const RvizParam> msg)
{

}

//==============================================================================
void NavGraphVisualizer::graph_cb(std::shared_ptr<const NavGraph> msg)
{

}

//==============================================================================
void NavGraphVisualizer::lane_states_cb(std::shared_ptr<const LaneStates> msg)
{

}

//==============================================================================
void NavGraphVisualizer::publish_navgraphs()
{

}

RCLCPP_COMPONENTS_REGISTER_NODE(NavGraphVisualizer)

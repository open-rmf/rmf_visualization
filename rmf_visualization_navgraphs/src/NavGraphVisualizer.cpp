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


#include "NavGraphVisualizer.hpp"


#include <rclcpp_components/register_node_macro.hpp>

//==============================================================================
NavGraphVisualizer::NavGraphVisualizer(const rclcpp::NodeOptions& options)
	: Node("navgraph_visualizer", options)
{
	_data = std::make_shared<Data>();

	_data->current_level = this->declare_parameter("initial_map_nam", "L1");
	RCLCPP_INFO(
		this->get_logger(),
		"Setting parameter initial_map_name to %s", _data->current_level.c_str());

	_data->param_sub = this->create_subscription<RvizParam>(
		"rmf_visualization/parameters",
		rclcpp::SystemDefaultsQoS(),
		[data = _data, n = weak_from_this()](
			std::shared_ptr<const RvizParam> msg)
	{
		if (!msg->map_name.empty())
			data->current_level = msg->map_name;

		if (auto node = n.lock())
		{
		        RCLCPP_INFO(
				node->get_logger(),
				"Publishing navgraphs on level %s", data->current_level.c_str());
		}

		data->publish_navgraphs();
	});

	const auto transient_qos =
		rclcpp::QoS(10).reliable().transient_local();

	_data->navgraph_sub = this->create_subscription<NavGraph>(
		"/nav_graphs",
		transient_qos,
		[data = _data, n = weak_from_this()](
			std::shared_ptr<const NavGraph> msg)
	{
		if (msg->name.empty())
			return;
		data->navgraphs[msg->name] = msg;

		data->publish_navgraphs();

		if (auto node = n.lock())
		{
		        RCLCPP_INFO(
				node->get_logger(),
				"Received nav_graph from fleet %s. Publishing...",
				msg->name.c_str());
		}
	});

	RCLCPP_INFO(
		this->get_logger(),
		"NavGraph visualizer is running...");
}

//==============================================================================
void NavGraphVisualizer::Data::param_cb(std::shared_ptr<const RvizParam> msg)
{

}

//==============================================================================
void NavGraphVisualizer::Data::graph_cb(std::shared_ptr<const NavGraph> msg)
{

}

//==============================================================================
void NavGraphVisualizer::Data::lane_states_cb(std::shared_ptr<const LaneStates> msg)
{

}

//==============================================================================
void NavGraphVisualizer::Data::publish_navgraphs()
{

}

RCLCPP_COMPONENTS_REGISTER_NODE(NavGraphVisualizer)

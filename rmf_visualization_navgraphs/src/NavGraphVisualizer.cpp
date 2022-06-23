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

#include <rmf_traffic_ros2/agv/Graph.hpp>

#include <rclcpp/subscription_options.hpp>


#include <rclcpp_components/register_node_macro.hpp>

//==============================================================================
NavGraphVisualizer::FleetNavGraph::FleetNavGraph()
{
  traffic_graph = std::nullopt;
  lane_states = nullptr;
  lane_markers = {};
}

//==============================================================================
NavGraphVisualizer::NavGraphVisualizer(const rclcpp::NodeOptions& options)
	: Node("navgraph_visualizer", options)
{
	_current_level = this->declare_parameter("initial_map_name", "L1");
	RCLCPP_INFO(
		this->get_logger(),
		"Setting parameter initial_map_name to %s", _current_level.c_str());

	// It is okay to capture this by reference here.
	_param_sub = this->create_subscription<RvizParam>(
		"rmf_visualization/parameters",
		rclcpp::SystemDefaultsQoS(),
		[=](std::shared_ptr<const RvizParam> msg)
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
  // Selectively disable intra-process comms for this subscriber as
  // non-volatile QoS is not supported.
  rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>> ipc_options;
  ipc_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;

	_navgraph_sub = this->create_subscription<NavGraph>(
		"/nav_graphs",
		transient_qos,
		[=](NavGraph::ConstSharedPtr msg)
	{
		if (msg->name.empty())
			return;

		const auto insertion = _navgraphs.insert({msg->name, nullptr});
    if (insertion.second)
    {
      // Add the navgraph and generate traffic_graph
      auto navgraph = std::make_shared<FleetNavGraph>();
      navgraph->traffic_graph = rmf_traffic_ros2::convert(*msg);
      _navgraphs[msg->name] = std::move(navgraph);
    }
    else
    {
      if (!insertion.first->second->traffic_graph.has_value())
      {
        insertion.first->second->traffic_graph = rmf_traffic_ros2::convert(*msg);
      }
    }
		publish_navgraphs();
	},
  ipc_options);

  _lane_states_sub = this->create_subscription<LaneStates>(
    "/lane_states",
    rclcpp::QoS(10).reliable(),
    [=](LaneStates::ConstSharedPtr msg)
    {
      if (msg->fleet_name.empty())
      return;

      const auto insertion = _navgraphs.insert({msg->fleet_name, nullptr});
      if (insertion.second)
      {
        // Add the navgraph and generate traffic_graph
        auto navgraph = std::make_shared<FleetNavGraph>();
        navgraph->lane_states = msg;
        _navgraphs[msg->fleet_name] = std::move(navgraph);
      }
      else
      {
        if (insertion.first->second->lane_states == nullptr)
        {
          insertion.first->second->lane_states = msg;
        }
      }
      publish_navgraphs();
    },
    ipc_options);

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

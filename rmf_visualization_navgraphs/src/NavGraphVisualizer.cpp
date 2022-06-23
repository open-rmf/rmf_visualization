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

#include <rclcpp/publisher_options.hpp>
#include <rclcpp/subscription_options.hpp>

#include <geometry_msgs/msg/point.hpp>

#include <rclcpp_components/register_node_macro.hpp>

//==============================================================================
NavGraphVisualizer::FleetNavGraph::FleetNavGraph(
  const std::string& fleet_name_,
  std::weak_ptr<rclcpp::Node> node_,
  Color::ConstSharedPtr color_,
  double lane_width_,
  double waypoint_width_)
: fleet_name(std::move(fleet_name_)),
  node(node_),
  color(color_),
  lane_width(lane_width_),
  waypoint_width(waypoint_width_)
{
  traffic_graph = std::nullopt;
  lane_states = nullptr;
  lane_markers = {};
}

//==============================================================================
void NavGraphVisualizer::FleetNavGraph::initialize_markers(
  const NavGraph& navgraph,
  const rclcpp::Time& now)
{
  // We need to bucket each lane into the level it exists on. This was easy to
  // infer when using rmf_building_map_msgs::msg::BuildingMap but there is no
  // correlation between the indices of lanes in that message and the indices
  // of the lane in the graph used by the fleet adapters. Hence, we sort the
  // lanes here.
  if (!traffic_graph.has_value())
  {
    // Without the traffic graph we cannot sort the lanes
    if (const auto n = node.lock())
    {
      RCLCPP_WARN(
        n->get_logger(),
        "[bug] initialize_markers called before setting the traffic graph"
        ". This is a bug and should be reported"
      );
    }
    return;
  }

  using Point = geometry_msgs::msg::Point;
  auto make_point =
    [](const Eigen::Vector2d& loc, double z) -> Point
    {
      return geometry_msgs::build<Point>()
      .x(loc[0])
      .y(loc[1])
      .z(z);
    };

  for (std::size_t i = 0; i < navgraph.edges.size(); ++i)
  {
    const auto& edge = navgraph.edges[i];
    const auto& entry_wp = traffic_graph->get_waypoint(edge.v1_idx);
    const auto& exit_wp = traffic_graph->get_waypoint(edge.v2_idx);
    if (entry_wp.get_map_name() != exit_wp.get_map_name())
    {
      // Lane connecting two floors. We don't have to visualize this.
      continue;
    }
    const auto& map_name = entry_wp.get_map_name();
    if (lane_markers.find(map_name) == lane_markers.end())
    {
      lane_markers.insert({map_name, {}});
    }

    Marker marker;
    marker.header.stamp = now;
    marker.header.frame_id = "map";
    marker.ns = fleet_name + "/lanes/" + map_name;
    marker.id = i;
    marker.type = marker.MODIFY;
    marker.type = marker.LINE_STRIP;
    marker.pose.orientation.w = 1.0;
    // For now we implement a simple visual change if a lane is speed limited
    // where the width of the lane is halved if speed limited.
    const auto& lane = traffic_graph->get_lane(i);
    marker.scale.x = lane.properties().speed_limit().has_value() ?
      lane_width / 2.0 : lane_width;
    marker.color = *color;
    marker.points.push_back(make_point(entry_wp.get_location(), -0.01));
    marker.points.push_back(make_point(exit_wp.get_location(), -0.01));
    auto& marker_map = lane_markers[map_name];
    marker_map.insert({i, std::move(marker)});
  }

  auto make_text_marker =
    [=](
      const std::size_t id,
      const Eigen::Vector2d& loc,
      const std::string& text,
      const std::string& map_name) -> Marker
    {
      Marker marker;
      marker.header.stamp = now;
      marker.header.frame_id = "map";
      marker.ns = fleet_name + "/labels/" + map_name;
      marker.id = id;
      marker.type = marker.MODIFY;
      marker.type = marker.TEXT_VIEW_FACING;
      marker.pose.position.x = loc[0] + 0.4 * std::cos(0.7853);
      marker.pose.position.x = loc[1] + 0.4 * std::sin(0.7853);
      marker.pose.orientation.w = 1.0;
      marker.scale.z = 0.2;
      marker.text = text;
      marker.color = *color;
      return marker;
    };

  auto make_empty_waypoint_marker =
    [=](const std::string& map_name) -> Marker
    {
      Marker marker;
      marker.header.stamp = now;
      marker.header.frame_id = "map";
      marker.ns = fleet_name + "/waypoints/" + map_name;
      marker.id = 1;
      marker.type = marker.MODIFY;
      marker.type = marker.POINTS;
      marker.scale.x = waypoint_width;
      marker.scale.y = waypoint_width;
      marker.scale.z = 1.0;
      marker.pose.orientation.w = 1.0;
      // This will be filled inside the loop below
      marker.points = {};
      marker.color = *color;
      return marker;
    };

  for (std::size_t i = 0; i < navgraph.vertices.size(); ++i)
  {
    const auto& wp = traffic_graph->get_waypoint(i);
    const auto& map_name = wp.get_map_name();
    const auto& loc = wp.get_location();
    if (text_markers.find(map_name) == text_markers.end())
    {
      text_markers.insert({map_name, {}});
      waypoint_markers.insert({map_name, make_empty_waypoint_marker(map_name)});
    }

    if (wp.name() != nullptr)
    {
      text_markers[map_name].push_back(
        make_text_marker(i, loc, *wp.name(), map_name));
    }
    waypoint_markers[map_name].points.push_back(make_point(loc, 0.0));
  }

}

//==============================================================================
void NavGraphVisualizer::FleetNavGraph::fill_with_markers(
  const std::string& map_name,
  MarkerArray& marker_array,
  const bool delete_markers)
{
  if (lane_markers.find(map_name) != lane_markers.end())
  {
    for (const auto& [_, marker] :  lane_markers.find(map_name)->second)
    {
      if (delete_markers)
      {
        // We create a copy and modify action to delete
        auto m = marker;
        m.action = marker.DELETEALL;
        marker_array.markers.push_back(std::move(m));
      }
      else
      {
        marker_array.markers.push_back(marker);
      }
    }

    // We are guaranteed to have waypoint markers
    if (delete_markers)
    {
      auto wp_marker = waypoint_markers.at(map_name);
      wp_marker.action = wp_marker.DELETEALL;
      marker_array.markers.push_back(std::move(wp_marker));
    }
    else
    {
      marker_array.markers.push_back(waypoint_markers[map_name]);
    }

    for (const auto& marker : text_markers.at(map_name))
    {
      if (delete_markers)
      {
        auto m = marker;
        m.action = m.DELETEALL;
        marker_array.markers.push_back(std::move(m));
      }
      else
      {
        marker_array.markers.push_back(marker);
      }
    }
  }

}

//==============================================================================
NavGraphVisualizer::NavGraphVisualizer(const rclcpp::NodeOptions& options)
: Node("navgraph_visualizer", options)
{
	_current_level = this->declare_parameter("initial_map_name", "L1");
	RCLCPP_INFO(
		this->get_logger(),
		"Setting parameter initial_map_name to %s", _current_level.c_str());

	_lane_width = this->declare_parameter("lane_width", 0.5);
	RCLCPP_INFO(
		this->get_logger(),
		"Setting parameter lane_width to %f", _lane_width);

	_waypoint_width = this->declare_parameter("waypoint_width", 1.0);
	RCLCPP_INFO(
		this->get_logger(),
		"Setting parameter waypoint_width to %f", _waypoint_width);

	// It is okay to capture this by reference here.
	_param_sub = this->create_subscription<RvizParam>(
		"rmf_visualization/parameters",
		rclcpp::SystemDefaultsQoS(),
		[=](RvizParam::ConstSharedPtr msg)
	{
		if (msg->map_name.empty() || msg->map_name == _current_level)
      return;

    // Delete old markers before updating current_level
    publish_map_markers(true);

    _current_level = msg->map_name;
		RCLCPP_INFO(
			this->get_logger(),
			"Publishing navgraphs on level %s", _current_level.c_str());
    // Publish new markers
		publish_map_markers();
	});


	const auto transient_qos =
		rclcpp::QoS(10).transient_local();
  // Selectively disable intra-process comms for publishers abd subscriptions
  // for non-volatile topics so that this node can still run in a container
  // with intra-process comms enabled.
  rclcpp::PublisherOptionsWithAllocator<
  std::allocator<void>> ipc_pub_options;
  ipc_pub_options.use_intra_process_comm =
    rclcpp::IntraProcessSetting::Disable;
  _marker_pub = this->create_publisher<MarkerArray>(
    "/map_markers",
    transient_qos,
    std::move(ipc_pub_options)
  );

  rclcpp::SubscriptionOptionsWithAllocator<
  std::allocator<void>> ipc_sub_options;
  ipc_sub_options.use_intra_process_comm =
    rclcpp::IntraProcessSetting::Disable;

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
        auto navgraph = std::make_shared<FleetNavGraph>(
          msg->name,
          weak_from_this(),
          get_next_color(),
          this->_lane_width,
          this->_waypoint_width
        );
        navgraph->traffic_graph = rmf_traffic_ros2::convert(*msg);
        if (!navgraph->traffic_graph.has_value())
        {
          RCLCPP_ERROR(
            this->get_logger(),
            "Unable to convert NavGraph message from fleet %s into a Traffic "
            "Graph. Lane markers from this fleet will not be published.",
            msg->name.c_str()
          );
          return;
        }
        navgraph->initialize_markers(*msg, this->get_clock()->now());
        _navgraphs[msg->name] = navgraph;
      }
      else
      {
        // We received the LaneStates message before the NavGraph for the fleet
        if (!insertion.first->second->traffic_graph.has_value())
        {
          auto traffic_graph  =
            rmf_traffic_ros2::convert(*msg);
          if (!traffic_graph.has_value())
          {
            RCLCPP_ERROR(
              this->get_logger(),
              "Unable to convert NavGraph message from fleet %s into a Traffic"
              " Graph. Lane markers from this fleet will not be published.",
              msg->name.c_str()
            );

            // We erase the entry from navgraphs since we cannot generate
            // any markers.
            _navgraphs.erase(insertion.first);
            return;
          }
          else
          {
            insertion.first->second->traffic_graph = std::move(traffic_graph);
            insertion.first->second->initialize_markers(
              *msg, this->get_clock()->now());
          }
        }
      }

      publish_map_markers();
    },
  ipc_sub_options
  );

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
        auto navgraph = std::make_shared<FleetNavGraph>(
          msg->fleet_name,
          weak_from_this(),
          get_next_color(),
          this->_lane_width,
          this->_waypoint_width
        );
        navgraph->lane_states = msg;
        _navgraphs[msg->fleet_name] = std::move(navgraph);
      }
      // We received LaneStates message after NavGraph for this fleet
      else
      {
        if (insertion.first->second->lane_states == nullptr)
        {
          insertion.first->second->lane_states = msg;
        }
      }
      publish_map_markers();
    },
    ipc_sub_options
  );

  this->initialize_color_options();

	RCLCPP_INFO(
		this->get_logger(),
		"NavGraph visualizer is running...");
}

//==============================================================================
void NavGraphVisualizer::initialize_color_options()
{
  auto make_color =
  [](float r, float g, float b, float a = 0.5) -> Color
  {
    return std_msgs::build<Color>().r(r).g(g).b(b).a(a);
  };

  // Initialize 9 distinct colors that the human eye can distinguish easily
  // If there are more than 9 fleets, we will reuse the colors.
  // Orange
  _color_options.push_back(
    std::make_shared<Color>(make_color(0.99, 0.5, 0.19))
  );
  // Blue
  _color_options.push_back(
    std::make_shared<Color>(make_color(0, 0.5, 0.8))
  );
  // Purple
  _color_options.push_back(
    std::make_shared<Color>(make_color(0.57, 0.12, 0.7))
  );
  // Teal
  _color_options.push_back(
    std::make_shared<Color>(make_color(0, 0.5, 0.5))
  );
  // Lavender
  _color_options.push_back(
    std::make_shared<Color>(make_color(0.9, 0.74, 1.0))
  );
  // Olive
  _color_options.push_back(
    std::make_shared<Color>(make_color(0.5, 0.5, 0))
  );
  // Cyan
  _color_options.push_back(
    std::make_shared<Color>(make_color(0.27, 0.94, 0.94))
  );
  // Grey
  _color_options.push_back(
    std::make_shared<Color>(make_color(0.5, 0.5, 0.5))
  );
  // Maroon
  _color_options.push_back(
    std::make_shared<Color>(make_color(0.5, 0, 0))
  );
}

auto NavGraphVisualizer::get_next_color() -> Color::ConstSharedPtr
{
  assert(!_color_options.empty());
  _next_color = std::min(_next_color, _color_options.size() - 1);
  auto color = _color_options.at(_next_color);
  ++_next_color;
  return color;
}

//==============================================================================
void NavGraphVisualizer::publish_map_markers(const bool delete_markers)
{
  MarkerArray marker_array;
  for (const auto& [name, navgraph] : _navgraphs)
  {
    navgraph->fill_with_markers(_current_level, marker_array, delete_markers);
  }

  if (marker_array.markers.empty())
    return;
  _marker_pub->publish(std::move(marker_array));
}

RCLCPP_COMPONENTS_REGISTER_NODE(NavGraphVisualizer)

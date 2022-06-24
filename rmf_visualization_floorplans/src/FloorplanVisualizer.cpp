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


#include "FloorplanVisualizer.hpp"

#include <rclcpp_components/register_node_macro.hpp>

//==============================================================================
FloorplanVisualizer::FloorplanVisualizer(const rclcpp::NodeOptions& options)
: Node("floorplan_visualizer", options)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Configuring floorplan_visualizer..."
  );

  const auto transient_qos =
    rclcpp::QoS(10).transient_local();
  // Selectively disable intra-process comms for publishers abd subscriptions
  // for non-volatile topics so that this node can still run in a container
  // with intra-process comms enabled.
  rclcpp::PublisherOptionsWithAllocator<
    std::allocator<void>> ipc_pub_options;
  ipc_pub_options.use_intra_process_comm =
    rclcpp::IntraProcessSetting::Disable;
  _floorplan_pub = this->create_publisher<OccupancyGrid>(
    "/floorplan",
    transient_qos,
    std::move(ipc_pub_options)
  );

  rclcpp::SubscriptionOptionsWithAllocator<
    std::allocator<void>> ipc_sub_options;
  ipc_sub_options.use_intra_process_comm =
    rclcpp::IntraProcessSetting::Disable;
  _map_sub = this->create_subscription<BuildingMap>(
    "/map",
    transient_qos,
    [=](BuildingMap::ConstSharedPtr msg)
    {

    },
    ipc_sub_options);

  _param_sub = this->create_subscription<RvizParamMsg>(
    "rmf_visualization/parameters",
    rclcpp::SystemDefaultsQoS(),
    [=](RvizParamMsg::ConstSharedPtr msg)
    {
      if (msg->map_name.empty() || msg->map_name == _current_level)
        return;

      auto it = _grids.find(msg->map_name);
      if (it == _grids.end())
      {
        // Publish deletion
        return;
      }
      _floorplan_pub->publish(
        std::move(std::make_unique<OccupancyGrid>(it->second))
      );
    });

  RCLCPP_INFO(
    this->get_logger(),
    "Running floorplan_visualizer..."
  )
}

//==============================================================================

RCLCPP_COMPONENTS_REGISTER_NODE(FloorplanVisualizer)

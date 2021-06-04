/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include "ScheduleMarkerPublisher.hpp"

#include <rmf_visualization_schedule/ScheduleDataNode.hpp>
#include <rmf_visualization_schedule/TrajectoryServer.hpp>

#include <rclcpp/executors.hpp>

//==============================================================================
bool get_arg(
  const std::vector<std::string>& args,
  const std::string& key,
  std::string& value,
  const std::string& desc,
  const bool mandatory = true)
{
  const auto key_arg = std::find(args.begin(), args.end(), key);
  if (key_arg == args.end())
  {
    if (mandatory)
    {
      std::cerr << "You must specify a " << desc <<" using the " << key
                << " argument!" << std::endl;
    }
    return false;
  }
  else if (key_arg+1 == args.end())
  {
    std::cerr << "The " << key << " argument must be followed by a " << desc
              << "!" << std::endl;
    return false;
  }

  value = *(key_arg+1);
  return true;
}

//==============================================================================
int main(int argc, char* argv[])
{
  const std::vector<std::string> args =
    rclcpp::init_and_remove_ros_arguments(argc, argv);

  std::string node_name = "rmf_visualization_schedule_data_node";
  get_arg(args, "-n", node_name, "node name", false);

  std::string rate_string;
  get_arg(args, "-r", rate_string, "rate", false);
  double rate = rate_string.empty() ? 1.0 : std::stod(rate_string);

  std::string port_string;
  get_arg(args, "-p", port_string, "port", false);
  const uint16_t port = port_string.empty() ? 8006 : std::stoul(
    port_string, nullptr, 0);

  std::string retained_history_count_str;
  uint retained_history_count = 0;
  if (get_arg(args, "--history", retained_history_count_str,
    "retained history count", false))
  {
    std::stringstream ss;
    ss << retained_history_count_str;
    ss >> retained_history_count;
  }

  std::string map_name = "B1";
  get_arg(args, "-m", map_name, "map name", false);

  const auto schedule_data_node =
    rmf_visualization_schedule::ScheduleDataNode::make(node_name, 60s);

  if (!schedule_data_node)
  {
    std::cerr << "Failed to initialize the visualizer node" << std::endl;
    return 1;
  }

  auto negotiation = schedule_data_node->get_negotiation();
  negotiation->set_retained_history_count(retained_history_count);

  RCLCPP_INFO(
    schedule_data_node->get_logger(),
    "%s started...",
    node_name.c_str());

  const auto server_ptr = rmf_visualization_schedule::TrajectoryServer::make(
    port,
    schedule_data_node);

  if (!server_ptr)
  {
    std::cerr << "Failed to initialize the websocket server" << std::endl;
    return 1;
  }

  RCLCPP_INFO(
    schedule_data_node->get_logger(),
    "Websocket server started on port: %d",
    port);

  auto schedule_marker_publisher = std::make_shared<ScheduleMarkerPublisher>(
    "rmf_visualization_schedule_marker_publisher",
    schedule_data_node,
    std::move(map_name),
    rate);

  rclcpp::executors::MultiThreadedExecutor executor{
    rclcpp::ExecutorOptions(), 2
  };

  executor.add_node(schedule_data_node);
  executor.add_node(schedule_marker_publisher);
  executor.spin();

  RCLCPP_INFO(
    schedule_data_node->get_logger(),
    "Closing down");

  rclcpp::shutdown();
}

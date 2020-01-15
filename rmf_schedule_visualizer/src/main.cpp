/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include "VisualizerData.hpp"
#include "Server.hpp"

#include <rclcpp/rclcpp.hpp>

bool get_arg(
    const std::vector<std::string>& args,
    const std::string& key,
    std::string& value,
    const std::string& desc,
    const bool mandatory = true)
{
  const auto key_arg = std::find(args.begin(), args.end(), key);
  if(key_arg == args.end())
  {
    if(mandatory)
    {
      std::cerr << "You must specify a " << desc <<" using the " << key
                << " argument!" << std::endl;
    }
    return false;
  }
  else if(key_arg+1 == args.end())
  {
    std::cerr << "The " << key << " argument must be followed by a " << desc
              << "!" << std::endl;
    return false;
  }

  value = *(key_arg+1);
  return true;
}

int main(int argc, char* argv[])
{
  const std::vector<std::string> args =
      rclcpp::init_and_remove_ros_arguments(argc, argv);

  std::string node_name = "viz";
  get_arg(args, "-n", node_name, "node name");

  std::string port_string;
  get_arg(args, "-p", port_string, "port",false);
  const uint16_t port = port_string.empty()? 8006 : std::stoul(port_string, nullptr, 0);

  const auto visualizer_data_node =
    rmf_schedule_visualizer::VisualizerDataNode::make(node_name);

  if(!visualizer_data_node)
  {
    std::cerr << "Failed to initialize the visualizer node" << std::endl;
    return 1;
  }

  RCLCPP_INFO(
        visualizer_data_node->get_logger(),
        "VisualizerDataNode /" + node_name + " started...");


  const auto server_ptr = rmf_schedule_visualizer::Server::make(port, *visualizer_data_node);
  
  if(!server_ptr)
  {
    std::cerr << "Failed to initialize the websocket server" << std::endl;
    return 1;
  }
  
  RCLCPP_INFO(
        visualizer_data_node->get_logger(),
        "Websocket server started on port: " + std::to_string(port));

  rclcpp::spin(visualizer_data_node);
  

  RCLCPP_INFO(
        visualizer_data_node->get_logger(),
        "Closing down");

  rclcpp::shutdown();
}

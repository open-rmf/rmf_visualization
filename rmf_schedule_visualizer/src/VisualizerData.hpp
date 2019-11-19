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


#ifndef RMF_SCHEDULE_VISUALIZER__SRC__VISUALIZERDATA_HPP
#define RMF_SCHEDULE_VISUALIZER__SRC__VISUALIZERDATA_HPP

#include <rmf_traffic_ros2/schedule/MirrorManager.hpp>

#include <rmf_traffic/Trajectory.hpp>
#include <rmf_traffic/schedule/Viewer.hpp>

#include <rclcpp/node.hpp>
#include <std_msgs/msg/string.hpp>

#include <set>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <rmf_schedule_visualizer/CommonData.hpp>

namespace rmf_schedule_visualizer {

//==============================================================================
class VisualizerDataNode : public rclcpp::Node
{
public:
  /// Builder function which returns a pointer to VisualizerNode when
  /// the Mirror Manager is readied and websocket is started.
  /// A nullptr is returned if initialization fails. 
  static std::shared_ptr<VisualizerDataNode> make(
      std::string node_name,
      rmf_traffic::Duration wait_time = std::chrono::seconds(10));

  /// Function to query Mirror Manager for trajectories
  std::vector<rmf_traffic::Trajectory> get_trajectories(RequestParam request_param);

private:
  struct Data
  {
    rmf_traffic_ros2::schedule::MirrorManager mirror;

    Data(rmf_traffic_ros2::schedule::MirrorManager mirror_)
    : mirror(std::move(mirror_))
    {
      // Do nothing
    }
  };

  // Create a VisualizerData node with a specified name
  VisualizerDataNode(std::string _node_name);

  using DebugSub = rclcpp::Subscription<std_msgs::msg::String>;
  DebugSub::SharedPtr debug_sub;

  void start(Data data);

  
  std::vector<rmf_traffic::Trajectory> _trajectories;
  std::string _node_name;
  std::unique_ptr<Data> data;

};

} // namespace rmf_schedule_visualizer

#endif // RMF_SCHEDULE_VISUALIZER__SRC__VISUALIZERDATA_HPP

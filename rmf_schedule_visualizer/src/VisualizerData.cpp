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

#include <rmf_traffic_ros2/StandardNames.hpp>
#include <rmf_traffic_ros2/Time.hpp>
#include <rmf_traffic_ros2/Trajectory.hpp>

#include <rclcpp/rclcpp.hpp>

namespace rmf_schedule_visualizer {

//==============================================================================
std::shared_ptr<VisualizerDataNode> VisualizerDataNode::make(
    std::string node_name,
    rmf_traffic::Duration wait_time)
{
  const auto start_time = std::chrono::steady_clock::now();
  std::shared_ptr<VisualizerDataNode> visualizer_data(
        new VisualizerDataNode(std::move(node_name)));

  // Creating a mirror manager that queries over all Spacetime in the database schedule 
  auto mirror_mgr_future = rmf_traffic_ros2::schedule::make_mirror(
        *visualizer_data, rmf_traffic::schedule::Query::Spacetime());

  const auto stop_time = start_time + wait_time;
  while(rclcpp::ok() && std::chrono::steady_clock::now() < stop_time)
  {
    // NOTE(MXG): We need to spin the node in order to get the mirror manager
    // fully initialized.
    rclcpp::spin_some(visualizer_data);

    using namespace std::chrono_literals;
    bool ready = (mirror_mgr_future.wait_for(0s) == std::future_status::ready);

    if(ready)
    {

        visualizer_data->start(
            Data{
              mirror_mgr_future.get(),
              
            });

      return visualizer_data;
    }
  }

  RCLCPP_ERROR(
        visualizer_data->get_logger(),
        "Mirror was not initialized in enough time ["
        + std::to_string(rmf_traffic::time::to_seconds(wait_time)) + "s]!");
  return nullptr;
}

//==============================================================================
VisualizerDataNode::VisualizerDataNode(std::string node_name)
  : Node(node_name),
    _node_name(std::move(node_name))
{
  // Do nothing

  // NOTE(MXG): We need to initialize an empty node so we can spin up the
  // MirrorManagerFuture into a full MirrorManager. But also we don't want this
  // node to do anything until all its data fields are finalized, so this
  // constructor is more like a formality than a real constructor.
}

//==============================================================================
void VisualizerDataNode::start(Data _data)
{
  data = std::make_unique<Data>(std::move(_data));
  data->mirror.update();

}

//==============================================================================
void VisualizerDataNode::callback_websocket()
{


}


} // namespace rmf_schedule_visualizer

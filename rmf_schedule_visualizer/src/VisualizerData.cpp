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
#include <std_msgs/msg/string.hpp>

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
        *visualizer_data, rmf_traffic::schedule::query_everything().spacetime(),
        &visualizer_data->_mutex);

  const auto stop_time = start_time + wait_time;
  while(rclcpp::ok() && std::chrono::steady_clock::now() < stop_time)
  {
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

  //Create a subscriber to a /debug topic to print information from this node
  debug_sub= create_subscription<std_msgs::msg::String>(
      _node_name+"/debug",rclcpp::SystemDefaultsQoS(),
      [&](std_msgs::msg::String::UniquePtr msg)
  {

    if (msg->data == "info")
    {
      std::lock_guard<std::mutex> guard(_mutex);
      RCLCPP_INFO(this->get_logger(), "Schedule Info: ");

      // here we will display the latest changes made to the mirror
      // along with details of trajectories in the schedule 
      try
      {
        std::cout<<"Latest Version: "<<
            std::to_string(data->mirror.viewer().latest_version())<<std::endl;

        // query since database was created
        auto view = data->mirror.viewer().query(
            rmf_traffic::schedule::make_query(0));
        if (view.size()==0)
          RCLCPP_INFO(this->get_logger(), "View is empty");
        else if (view.size() <= 2)
        {
          // Do not want to iterate larger views
          for (const auto& element : view)
          {
            auto t = element.trajectory;
            std::cout<<"Trajectory ID: "<<element.id<<std::endl;
            std::cout<<"Segment Number: "<<t.size()<<std::endl;
            size_t s_count = 0;
            for (auto it = t.begin(); it!= t.end(); it++)
            {
              ++s_count;
              auto finish_time = it->get_finish_time();
              auto finish_position = it->get_finish_position();
              std::cout<<"Segment: "<<s_count<<std::endl;
              std::cout<<"\tfinish_time: "<<
                  std::to_string(
                      finish_time.time_since_epoch().count())<<std::endl;
                      
              std::cout<<"\tfinish_position: "<<finish_position[0]
                  <<" "<<finish_position[1]
                  <<" "<<finish_position[2]<<std::endl;
        
            }
          }
        }
      }
      catch (std::exception& e)
      {
        RCLCPP_ERROR(this->get_logger(), e.what());
      }

    }
  });

}

std::vector<rmf_traffic::Trajectory> VisualizerDataNode::get_trajectories(RequestParam request_param)
{
  std::vector<rmf_traffic::Trajectory> trajectories; 
  const std::vector<std::string> maps {request_param.map_name};
  const auto query = rmf_traffic::schedule::make_query(
      maps, &request_param.start_time,
      &request_param.finish_time);

  const auto view = data->mirror.viewer().query(query);
  for (const auto& element : view)
    trajectories.push_back(element.trajectory);

  return trajectories;
}
using Element = rmf_traffic::schedule::Viewer::View::Element;
std::vector<Element> VisualizerDataNode::get_elements(RequestParam request_param)
{
  std::vector<Element> elements; 
  const std::vector<std::string> maps {request_param.map_name};
  const auto query = rmf_traffic::schedule::make_query(
      maps, &request_param.start_time,
      &request_param.finish_time);

  const auto view = data->mirror.viewer().query(query);

  for (const auto& element : view)
    elements.push_back(element);

  return elements;
}

//==============================================================================
rmf_traffic::Time VisualizerDataNode::now()
{
  return rmf_traffic_ros2::convert(get_clock()->now());
}

//==============================================================================
std::mutex& VisualizerDataNode::get_mutex()
{
  return _mutex;
}

} // namespace rmf_schedule_visualizer

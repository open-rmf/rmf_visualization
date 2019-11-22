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

#include <rmf_traffic/Trajectory.hpp>
#include <rmf_traffic_ros2/Trajectory.hpp>
#include <rmf_traffic_msgs/srv/submit_trajectories.hpp>
#include <rmf_traffic/Time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmf_traffic/geometry/Box.hpp>
#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic_ros2/StandardNames.hpp>

using SubmitTrajectory = rmf_traffic_msgs::srv::SubmitTrajectories;
using SubmitTrajectoryClient = rclcpp::Client<SubmitTrajectory>;
using SubmitTrajectoryHandle = SubmitTrajectoryClient::SharedPtr;
using namespace std::chrono_literals;


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


class SubmitTrajectoryNode : public rclcpp::Node
{
public:

  SubmitTrajectoryNode(
      std::string node_name_ ="submit_trajectory_node",
      std::string map_name = "level1",
      rmf_traffic::Duration duration_ = 400s,
      Eigen::Vector3d position_ = Eigen::Vector3d{0,0,0},
      Eigen::Vector3d velocity_ = Eigen::Vector3d{0,0,0})
  : Node(node_name_),
    _position(position_),
    _velocity(velocity_)
  {
    _start_time = std::chrono::steady_clock::now();
    _finish_time = _start_time + duration_;

    auto profile = rmf_traffic::Trajectory::Profile::make_guided(
        rmf_traffic::geometry::make_final_convex<
          rmf_traffic::geometry::Circle>(1.5));
    
    rmf_traffic::Trajectory t(map_name);
    t.insert(
        _start_time,
        profile,
        _position,
        _velocity);

    t.insert(
        _finish_time,
        profile,
        _position,
        _velocity);

    SubmitTrajectory::Request request_msg;
    request_msg.trajectories.emplace_back(rmf_traffic_ros2::convert(t));

    auto submit_trajectory = this->create_client<SubmitTrajectory>(
        rmf_traffic_ros2::SubmitTrajectoriesSrvName);

    while (!submit_trajectory->wait_for_service(1s)) 
    {
      if (!rclcpp::ok()) 
      {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        // TODO(YV) use correct error code
        rclcpp::exceptions::throw_from_rcl_error(200, "Interrupted");
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    if (submit_trajectory->service_is_ready())
    {
      submit_trajectory->async_send_request(
          std::make_shared<SubmitTrajectory::Request>(std::move(request_msg)),
          [&](const SubmitTrajectoryClient::SharedFuture response)
      {
        this->parse_response(response);
      });
    }
    else
    {
      RCLCPP_ERROR(
          this->get_logger(),
          rmf_traffic_ros2::SubmitTrajectoriesSrvName +" service is unavailable!"
      );
    }

  }


private:
  void parse_response(
      const SubmitTrajectoryClient::SharedFuture& response)
  {
    const auto response_msg = response.get();
    if(response_msg->accepted)
    {
      RCLCPP_INFO(get_logger(), "Response: accepted");
    }
    else
    {
      std::string error_msg = "Response: " + response_msg->error + ". Conflicts:";
      for(const auto& conflict : response_msg->conflicts)
      {
        error_msg += "\n -- " + std::to_string(conflict.index) + " @ "
            + std::to_string(conflict.time);
      }
      RCLCPP_INFO(get_logger(), error_msg);
    }
  }
  rmf_traffic::Time _start_time;
  rmf_traffic::Time _finish_time;
  Eigen::Vector3d _position;
  Eigen::Vector3d _velocity;

};


int main(int argc, char* argv[])
{
  const std::vector<std::string> args =
      rclcpp::init_and_remove_ros_arguments(argc, argv);

  std::string node_name = "submit_trajectory_node";
  
  std::string map_name = "level1";
  get_arg(args, "-m", map_name, "map name", false);

  std::string duration_string;
  get_arg(args, "-d", duration_string, "duration(s)",false);
  rmf_traffic::Duration duration = 400s;


  Eigen::Vector3d position{0,0,0};

  std::string x_string;
  if(get_arg(args, "-x", x_string, "x-coordinate",false))
    position[0]=std::stod(x_string);

  std::string y_string;
  if(get_arg(args, "-y", y_string, "x-coordinate",false))
    position[1]=std::stod(y_string);

  Eigen::Vector3d velocity{0,0,0};


  rclcpp::spin_some(std::make_shared<SubmitTrajectoryNode>(
        node_name,
        map_name,
        duration,
        position,
        velocity));

  rclcpp::shutdown();
  return 0;

}
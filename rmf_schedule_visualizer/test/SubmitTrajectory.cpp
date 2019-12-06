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
#include <rmf_traffic_ros2/Time.hpp>
#include <rmf_traffic_msgs/srv/submit_trajectories.hpp>
#include <rmf_traffic/Time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmf_traffic/geometry/Box.hpp>
#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic_ros2/StandardNames.hpp>

using SubmitTrajectories = rmf_traffic_msgs::srv::SubmitTrajectories;
using SubmitTrajectoryClient = rclcpp::Client<SubmitTrajectories>;
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
      rmf_traffic::Duration start_delay = 0s,
      rmf_traffic::Duration duration_ = 60s,
      Eigen::Vector3d position_ = Eigen::Vector3d{0,0,0},
      Eigen::Vector3d velocity_ = Eigen::Vector3d{0,0,0},
      double radius = 1.0)
  : Node(node_name_),
    _position(position_),
    _velocity(velocity_)
  {
//    _start_time = std::chrono::steady_clock::now() + start_delay;
    _start_time = rmf_traffic_ros2::convert(get_clock()->now()) + start_delay;
    _finish_time = _start_time + duration_;

    auto profile = rmf_traffic::Trajectory::Profile::make_guided(
        rmf_traffic::geometry::make_final_convex<
          rmf_traffic::geometry::Circle>(radius));
    
    rmf_traffic::Trajectory t(map_name);
    t.insert(
        _start_time,
        profile,
        _position,
        _velocity);

    t.insert(
        _start_time + 10s,
        profile,
        _position + Eigen::Vector3d{10, 0, 0},
        _velocity);
    
    t.insert(
        _start_time + 15s,
        profile,
        _position + Eigen::Vector3d{10, 0, M_PI_2},
        _velocity);
    
    t.insert(
        _finish_time,
        profile,
        _position + Eigen::Vector3d{10, 10, M_PI_2},
        _velocity);

    SubmitTrajectories::Request request_msg;
    request_msg.trajectories.emplace_back(rmf_traffic_ros2::convert(t));
    request_msg.fleet.fleet_id = "test_fleet";
    request_msg.fleet.type = rmf_traffic_msgs::msg::FleetProperties::TYPE_NO_CONTROL;

    _submit_trajectory = this->create_client<SubmitTrajectories>(
        rmf_traffic_ros2::SubmitTrajectoriesSrvName);

    while (!_submit_trajectory->wait_for_service(1s)) 
    {
      if (!rclcpp::ok()) 
      {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        // TODO(YV) use correct error code
        rclcpp::exceptions::throw_from_rcl_error(200, "Interrupted");
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    if (_submit_trajectory->service_is_ready())
    {
      _submit_trajectory->async_send_request(
          std::make_shared<SubmitTrajectories::Request>(std::move(request_msg)),
          [&](const SubmitTrajectoryClient::SharedFuture future)
      {
        if (!future.valid() || std::future_status::ready != future.wait_for(0s))
        {
          RCLCPP_ERROR(
                this->get_logger(),
                "Failed to get response from schedule");
          return;
        }
        const auto response = *future.get();

        RCLCPP_INFO(this->get_logger(),"Accepted: " + response.accepted);
        if (!response.error.empty())
        {
          RCLCPP_ERROR(
                this->get_logger(),
                "Error response from schedule: " + response.error);
          return;
        }
      });
    }
    else
    {
      RCLCPP_ERROR(
          this->get_logger(),
          rmf_traffic_ros2::SubmitTrajectoriesSrvName +
          " service is unavailable!");
    }
  }


private:
  rmf_traffic::Time _start_time;
  rmf_traffic::Time _finish_time;
  Eigen::Vector3d _position;
  Eigen::Vector3d _velocity;
  SubmitTrajectoryHandle _submit_trajectory;
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
  rmf_traffic::Duration duration = duration_string.empty() ?
      60s : std::chrono::seconds(std::stoi(duration_string));

  std::string delay_string;
  get_arg(args, "-D", delay_string, "start delay(s)",false);
  rmf_traffic::Duration delay = delay_string.empty() ?
      0s : std::chrono::seconds(std::stoi(delay_string));

  std::string radius_string;
  get_arg(args, "-r", radius_string, "radius",false);
  double radius = radius_string.empty() ? 1.0 : std::stod(radius_string);

  Eigen::Vector3d position{0,0,0};

  std::string x_string;
  if(get_arg(args, "-x", x_string, "x-coordinate",false))
    position[0]=std::stod(x_string);

  std::string y_string;
  if(get_arg(args, "-y", y_string, "x-coordinate",false))
    position[1]=std::stod(y_string);

  Eigen::Vector3d velocity{0,0,0};


  rclcpp::spin(std::make_shared<SubmitTrajectoryNode>(
        node_name,
        map_name,
        delay,
        duration,
        position,
        velocity,
        radius));

  rclcpp::shutdown();
  return 0;

}

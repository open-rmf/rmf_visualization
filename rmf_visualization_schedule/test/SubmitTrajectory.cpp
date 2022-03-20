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
#include <rmf_traffic_ros2/schedule/Writer.hpp>
#include <rmf_traffic/Time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic_ros2/StandardNames.hpp>

using namespace std::chrono_literals;


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


class SubmitTrajectoryNode : public rclcpp::Node
{
public:

  SubmitTrajectoryNode(std::string node_name_ = "submit_trajectory_node")
  : Node(node_name_)
  {
    // Do nothing
  }

  static std::shared_ptr<SubmitTrajectoryNode> make(
    std::string node_name_ = "submit_trajectory_node",
    std::string map_name = "level1",
    rmf_traffic::Duration start_delay = 0s,
    rmf_traffic::Duration duration_ = 60s,
    Eigen::Vector3d position = Eigen::Vector3d{0, 0, 0},
    Eigen::Vector3d velocity = Eigen::Vector3d{0, 0, 0},
    double radius = 1.0)
  {
    auto node = std::make_shared<SubmitTrajectoryNode>(std::move(node_name_));
    const auto start_time =
      rmf_traffic_ros2::convert(node->get_clock()->now()) + start_delay;

    const auto finish_time = start_time + duration_;

    auto profile = rmf_traffic::Profile(
      rmf_traffic::geometry::make_final_convex<
        rmf_traffic::geometry::Circle>(radius));

    rmf_traffic::Trajectory t;
    t.insert(
      start_time,
      position,
      velocity);

    t.insert(
      start_time + 10s,
      position + Eigen::Vector3d{10, 0, 0},
      velocity);

    t.insert(
      start_time + 15s,
      position + Eigen::Vector3d{10, 0, M_PI_2},
      velocity);

    t.insert(
      finish_time,
      position + Eigen::Vector3d{10, 10, M_PI_2},
      velocity);

    rmf_traffic::Route route{std::move(map_name), std::move(t)};

    node->_writer = rmf_traffic_ros2::schedule::Writer::make(node);
    node->_writer->wait_for_service();

    node->_writer->async_make_participant(
      rmf_traffic::schedule::ParticipantDescription{
        "test",
        "submit_trajectory_node",
        rmf_traffic::schedule::ParticipantDescription::Rx::Unresponsive,
        std::move(profile)
      },
      [route = std::move(route)](rmf_traffic::schedule::Participant p)
      {
        p.set(p.assign_plan_id(), {std::move(route)});
      });

    return node;
  }


private:
  rmf_traffic_ros2::schedule::WriterPtr _writer;
};


int main(int argc, char* argv[])
{
  const std::vector<std::string> args =
    rclcpp::init_and_remove_ros_arguments(argc, argv);

  std::string node_name = "submit_trajectory_node";

  std::string map_name = "level1";
  get_arg(args, "-m", map_name, "map name", false);

  std::string duration_string;
  get_arg(args, "-d", duration_string, "duration(s)", false);
  rmf_traffic::Duration duration = duration_string.empty() ?
    60s : std::chrono::seconds(std::stoi(duration_string));

  std::string delay_string;
  get_arg(args, "-D", delay_string, "start delay(s)", false);
  rmf_traffic::Duration delay = delay_string.empty() ?
    0s : std::chrono::seconds(std::stoi(delay_string));

  std::string radius_string;
  get_arg(args, "-r", radius_string, "radius", false);
  double radius = radius_string.empty() ? 1.0 : std::stod(radius_string);

  Eigen::Vector3d position{0, 0, 0};

  std::string x_string;
  if (get_arg(args, "-x", x_string, "x-coordinate", false))
    position[0] = std::stod(x_string);

  std::string y_string;
  if (get_arg(args, "-y", y_string, "x-coordinate", false))
    position[1] = std::stod(y_string);

  Eigen::Vector3d velocity{0, 0, 0};


  rclcpp::spin(SubmitTrajectoryNode::make(
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

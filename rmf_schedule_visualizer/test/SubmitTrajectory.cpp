#include <rmf_traffic/Trajectory.hpp>
#include <rmf_traffic_ros2/Trajectory.hpp>
#include <rmf_traffic_msgs/srv/submit_trajectory.hpp>
#include <rmf_traffic/Time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmf_traffic/geometry/Box.hpp>
#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic_ros2/StandardNames.hpp>

using SubmitTrajectory = rmf_traffic_msgs::srv::SubmitTrajectory;
using SubmitTrajectoryClient = rclcpp::Client<SubmitTrajectory>;
using SubmitTrajectoryHandle = SubmitTrajectoryClient::SharedPtr;
using namespace std::chrono_literals;

class SubmitTrajectoryNode : public rclcpp::Node
{
public:

  SubmitTrajectoryNode(
      std::string node_name_ ="test_submit_trajectory",
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
        rmf_traffic_ros2::SubmitTrajectoryServiceName);

    while (!submit_trajectory->wait_for_service(1s)) 
    {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
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
          rmf_traffic_ros2::SubmitTrajectoryServiceName +" service is unavailable!"
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

  std::string node_name = "test_submit_trajectory";
  std::string map_name = "level1";
  rmf_traffic::Duration duration = 400s;
  Eigen::Vector3d position{0,0,0};
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
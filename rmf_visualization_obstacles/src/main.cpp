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


#include <rclcpp/rclcpp.hpp>

#include <rmf_obstacle_msgs/msg/obstacles.hpp>

#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <memory>

//==============================================================================
class ObstacleVisualizer :
public rclcpp::Node, public std::enable_shared_from_this<ObstacleVisualizer>
{
public:
  using ObstaclesMsg = rmf_obstacle_msgs::msg::Obstacles;
  using ObstacleMsg = rmf_obstacle_msgs::msg::Obstacle;
  using ObstacleDataMsg = rmf_obstacle_msgs::msg::ObstacleData;
  using MarkerMsg = visualization_msgs::msg::Marker;
  using MarkerArrayMsg = visualization_msgs::msg::MarkerArray;
  using Color = std_msgs::msg::ColorRGBA;

  // Constructor
  ObstacleVisualizer()
  : Node("rmf_obstacle_visualizer_node")
  {
    // TODO(YV): Subscribe to RVizpParams for level checking
    _obstacle_markers_pub = this->create_publisher<MarkerArrayMsg>(
      "obstacle_markers",
      rclcpp::QoS(10));

    // TODO(YV): Wrap sub into Data struct ptr to avoid reference capture
    _obstacles_sub = this->create_subscription<ObstaclesMsg>(
      "rmf_obstacles",
      rclcpp::QoS(10),
      [&](std::shared_ptr<const ObstaclesMsg> msg)
      {
        msg_cb(*msg);
      });
  }

  void msg_cb(const ObstaclesMsg& msg)
  {
    if (msg.obstacles.empty())
      return;

    RCLCPP_INFO(
      this->get_logger(),
      "Received %ld obstacles",
      msg.obstacles.size());

    std::unique_ptr<MarkerArrayMsg> obstacle_msg =
      std::make_unique<MarkerArrayMsg>();
    for (const auto& obstacle : msg.obstacles)
    {
      if (obstacle.source.empty())
      {
        RCLCPP_WARN(
          this->get_logger(),
          "Received obstacle message with empty source. Ignoring...");
        continue;;
      }
      if (obstacle.level_name.empty())
      {
        RCLCPP_WARN(
          this->get_logger(),
          "Received obstacle message with empty level_name. Ignoring...");
        continue;
      }
      if (obstacle.classification != "human")
      {
        RCLCPP_WARN(
          this->get_logger(),
          "ObstacleVisualizer currently only supports human classification. "
          "Ignoring...");
        continue;
      }
      MarkerMsg _msg;
      _msg.header.frame_id = obstacle.header.frame_id;
      _msg.header.stamp = this->get_clock()->now();
      _msg.ns = "humans";
      _msg.text = "human";
      _msg.id = obstacle.id;
      _msg.type = _msg.CYLINDER;
      _msg.action = _msg.ADD;
      _msg.pose = obstacle.data.box.center;
      _msg.scale = obstacle.data.box.size;
      _msg.color = make_color(0,0.9,0.3);
      _msg.lifetime = obstacle.lifetime;

      MarkerMsg _text = _msg;
      _text.type = _text.TEXT_VIEW_FACING;
      _text.id = -1 * (_msg.id + 1);
      _text.pose.position.z = _text.pose.position.z + 1.0;
      _text.pose.position.x = _text.pose.position.x + 1.0;

      obstacle_msg->markers.emplace_back(_msg);
      obstacle_msg->markers.emplace_back(_text);
    }

    if (!obstacle_msg->markers.empty())
      _obstacle_markers_pub->publish(std::move(obstacle_msg));
  }

  Color make_color(float r, float g, float b, float a = 1.0) const
  {
    Color color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
  }

private:
  rclcpp::Subscription<ObstaclesMsg>::SharedPtr _obstacles_sub;
  rclcpp::Publisher<MarkerArrayMsg>::SharedPtr _obstacle_markers_pub;
};

//==============================================================================
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  const auto node = std::make_shared<ObstacleVisualizer>();

  RCLCPP_INFO(
    node->get_logger(),
    "Beginning RMF obstacle visualizer node");

  rclcpp::spin(node);

  RCLCPP_INFO(
    node->get_logger(),
    "Closing RMF obstacle visualizer node");

  rclcpp::shutdown();
}

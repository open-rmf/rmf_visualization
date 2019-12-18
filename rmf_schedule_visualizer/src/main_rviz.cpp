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
#include <rclcpp/rclcpp.hpp>

#include <rmf_traffic/geometry/Box.hpp>
#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/Time.hpp>
#include <rmf_traffic_ros2/Time.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <rmf_traffic_msgs/msg/schedule_conflict.hpp>
#include "rmf_schedule_visualizer_msgs/msg/rviz_param.hpp"
#include <building_map_msgs/msg/building_map.hpp>
#include <building_map_msgs/msg/level.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include <mutex>

using namespace std::chrono_literals;

class RvizNode : public rclcpp::Node
{
public:
  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using Point = geometry_msgs::msg::Point;
  using RequestParam = rmf_schedule_visualizer::RequestParam;
  using ScheduleConflict = rmf_traffic_msgs::msg::ScheduleConflict;
  using Element = rmf_traffic::schedule::Viewer::View::Element;
  using RvizParamMsg = rmf_schedule_visualizer_msgs::msg::RvizParam;
  using BuildingMap = building_map_msgs::msg::BuildingMap;
  using Level = building_map_msgs::msg::Level;
  using Color = std_msgs::msg::ColorRGBA;

  RvizNode(
      std::string node_name,
      rmf_schedule_visualizer::VisualizerDataNode& visualizer_data_node,
      std::string map_name,
      double rate = 1.0,
      std::string frame_id = "/map")
  : Node(node_name),
    _rate(rate),
    _frame_id(frame_id),
    _visualizer_data_node(visualizer_data_node)
  {
    // TODO add a constructor for RvizParam
    _rviz_param.map_name = map_name;
    _rviz_param.query_duration = std::chrono::seconds(600);
    _rviz_param.start_duration = std::chrono::seconds(0);
    update_level_cache();

    // Create a timer with specified rate. This timer runs on the main thread.
    const double period = 1.0/_rate;
    _timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double, std::ratio<1>>(period));
    _timer = this->create_wall_timer(_timer_period, std::bind(&RvizNode::timer_callback, this));
    
    // Create publisher for marker_array
    _marker_array_pub = this->create_publisher<MarkerArray>(
        "dp2_marker_array",
        rclcpp::ServicesQoS());
    
    // Create subscriber for schedule_conflict in separate thread
    _cb_group_conflict_sub = this->create_callback_group(
        rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
    auto sub_conflict_opt = rclcpp::SubscriptionOptions();
    sub_conflict_opt.callback_group = _cb_group_conflict_sub;
    _conflcit_sub = this->create_subscription<ScheduleConflict>(
          "/rmf_traffic/schedule_conflict",
          rclcpp::QoS(10),
          [&](ScheduleConflict::SharedPtr msg)
          {
            std::lock_guard<std::mutex> guard(_visualizer_data_node.get_mutex());
            _conflict_id.clear();
            for (const auto& i : msg->indices)
              _conflict_id.push_back(i);
          },
          sub_conflict_opt);

    // Create subscriber for rviz_param in separate thread
    _cb_group_param_sub = this->create_callback_group(
        rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
    auto sub_param_opt = rclcpp::SubscriptionOptions();
    sub_param_opt.callback_group = _cb_group_param_sub;
    _param_sub = this->create_subscription<RvizParamMsg>(
        node_name + "/param",
        rclcpp::QoS(10),
        [&](RvizParamMsg::SharedPtr msg)
        {
          std::lock_guard<std::mutex> guard(_visualizer_data_node.get_mutex());

          if (!msg->map_name.empty() && _rviz_param.map_name != msg->map_name)
            {
              _rviz_param.map_name = msg->map_name;
              update_level_cache();
            }

          if (msg->query_duration > 0)
            {
              _rviz_param.query_duration =
                  std::chrono::seconds(msg->query_duration);
            }

          if (msg->start_duration >= 0 &&
              std::chrono::seconds(msg->start_duration)
              < _rviz_param.query_duration)
          {
            _rviz_param.start_duration =
                std::chrono::seconds(msg->start_duration);
          }
          RCLCPP_INFO(this->get_logger(),"Rviz parameters updated");
        },
        sub_param_opt);
    
    // Create subcscriber for building_map in separate thread.
    // The subscriber requies QoS profile with transient local durability
    _cb_group_map_sub = this->create_callback_group(
    rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
    auto sub_map_opt = rclcpp::SubscriptionOptions();
    sub_map_opt.callback_group = _cb_group_map_sub;
    auto qos_profile = rclcpp::QoS(10);
    qos_profile.transient_local();
    _map_sub = this->create_subscription<BuildingMap>(
        "/map",
        qos_profile,
        [&](BuildingMap::SharedPtr msg)
        {
          std::lock_guard<std::mutex> guard(_visualizer_data_node.get_mutex());
          RCLCPP_INFO(this->get_logger(),"Received map \""
              + msg->name + "\" containing "
              + std::to_string(msg->levels.size()) + " level(s)");
          // Cache building map message
          _map_msg = *msg;
          update_level_cache();
        },
        sub_map_opt);
    
    // Initialize boolean to indicate whether level cache is relevant
    _has_level = false;

    // Populate _color_list used to assign colors to graphs and robots
    set_color_list(0.5);
  }

private:

  void timer_callback()
  {
    MarkerArray marker_array;

    // TODO store a cache of trajectories to prevent frequent access.
    // Update cache whenever mirror manager updates
    std::lock_guard<std::mutex> guard(_visualizer_data_node.get_mutex());

    RequestParam query_param;
    query_param.map_name = _rviz_param.map_name;
    query_param.start_time = _visualizer_data_node.now();
    query_param.finish_time = query_param.start_time + _rviz_param.query_duration;

    _elements = _visualizer_data_node.get_elements(query_param);

    RequestParam traj_param;
    traj_param.map_name = query_param.map_name;
    traj_param.start_time = query_param.start_time + _rviz_param.start_duration;
    traj_param.finish_time = query_param.finish_time;

    // Store the ids of active trajectories
    std::vector<uint64_t> active_id;

    // For each trajectory create two markers
    // 1) Current position
    // 2) Path until param.finish_time
    for (const auto& element : _elements)
    {
      active_id.push_back(element.id);

      if (element.trajectory.find(traj_param.start_time) != element.trajectory.end())
      {
        auto location_marker = make_location_marker(element, traj_param);
        marker_array.markers.push_back(location_marker);
      }
      if (traj_param.start_time < *element.trajectory.finish_time())
      {
        auto path_marker = make_path_marker(element, traj_param);
        marker_array.markers.push_back(path_marker);

        // Add id to _marker_tracker
        if (_marker_tracker.find(element.id) == _marker_tracker.end())
          _marker_tracker.insert(element.id);
      }
    }
    // Add map markers
    add_map_markers(marker_array);

    // Add deletion markers for trajectories no longer active
    std::unordered_set<uint64_t> removed_markers;
    for (const auto marker : _marker_tracker)
    {
      if (std::find(active_id.begin(), active_id.end(), marker)
          == active_id.end())
      {
        delete_marker(marker, marker_array);
        removed_markers.insert(marker);
      }
    }

    for (const auto r : removed_markers)
      _marker_tracker.erase(r);

    // Publish marker_array
    if (!marker_array.markers.empty())
    {
      RCLCPP_DEBUG(this->get_logger(),
        "Publishing marker array of size: " + std::to_string(marker_array.markers.size()));
      _marker_array_pub->publish(marker_array);
    }
  }

  void add_map_markers(MarkerArray& marker_array)
  {
    if (_map_msg.levels.size() < 1)
      return;

    if (_has_level)
    {    
      // Marker for node locations
      Marker node_marker;
      node_marker.header.frame_id = _frame_id; // map
      node_marker.header.stamp = rmf_traffic_ros2::convert(
          _visualizer_data_node.now());
      node_marker.ns = "map";
      node_marker.id = 0;
      node_marker.type = node_marker.POINTS;
      node_marker.action = node_marker.ADD;

      node_marker.pose.orientation.w = 1;

      // Set the scale of the marker
      node_marker.scale.x = 0.1;
      node_marker.scale.y = 0.1;
      node_marker.scale.z = 1.0;

      // Set the color of the marker
      node_marker.color = make_color(1, 1, 1);

      // Set the lifetime of the marker
      if (_rate <= 1)
        node_marker.lifetime = convert(_timer_period);
      else
      {
        builtin_interfaces::msg::Duration duration;
        duration.sec = 1;
        duration.nanosec = 0;
        node_marker.lifetime = duration;
      }
      // Marker for lanes
      Marker lane_marker = node_marker;
      lane_marker.id = 2;
      lane_marker.type = lane_marker.LINE_LIST;
      lane_marker.scale.x = 0.5;

      // Add markers for nodes and lanes in each graph
      uint64_t graph_count = 0;
      for (const auto nav_graph : _level.nav_graphs)
      {
        for (const auto vertex : nav_graph.vertices)
        {
          node_marker.points.push_back(make_point({vertex.x, vertex.y, 0}));
        }
        // Unique lane marker for each graph
        lane_marker.id = graph_count + 2;
        lane_marker.points.clear();
        // TODO use graph id or graph name to lookup color
        lane_marker.color = get_color(graph_count);
        for (const auto edge : nav_graph.edges)
        {
          auto n1 = nav_graph.vertices[edge.v1_idx];
          auto n2 = nav_graph.vertices[edge.v2_idx];
          lane_marker.points.push_back(make_point({n1.x, n1.y, -0.2}, true));
          lane_marker.points.push_back(make_point({n2.x, n2.y, -0.2}, true));
        }
        graph_count++;
        // Insert lane marker to marker_array
         marker_array.markers.push_back(lane_marker);
      }
      marker_array.markers.push_back(node_marker);
    }
  }

  void delete_marker(const uint64_t id, MarkerArray& marker_array)
  {
    Marker marker_msg;
    marker_msg.header.frame_id = _frame_id; // map
    marker_msg.header.stamp = rmf_traffic_ros2::convert(
        _visualizer_data_node.now());
    marker_msg.ns = "trajectory";
    marker_msg.id = id;
    marker_msg.type = marker_msg.CYLINDER;
    marker_msg.action = marker_msg.DELETE;
    marker_array.markers.push_back(marker_msg);

    // Add a delete marker for the path
    marker_msg.id = -1 * id;
    marker_msg.type = marker_msg.LINE_STRIP;
    marker_array.markers.push_back(marker_msg);
  }

  visualization_msgs::msg::Marker make_location_marker(
        Element element,
        const RequestParam param)
  {
    Marker marker_msg;

    // TODO Link the color, shape and size of marker to profile of trajectory
    const auto& trajectory = element.trajectory;

    const double radius = static_cast<const rmf_traffic::geometry::Circle&>(
          trajectory.begin()->get_profile()->get_shape()->source()).get_radius();

    marker_msg.header.frame_id = _frame_id; // map
    marker_msg.header.stamp = rmf_traffic_ros2::convert(param.start_time);
    marker_msg.ns = "trajectory";
    marker_msg.id = element.id;
    marker_msg.type = marker_msg.CYLINDER;
    marker_msg.action = marker_msg.ADD;

    // Set the pose of the marker
    auto motion = trajectory.find(param.start_time)->compute_motion();
    Eigen::Vector3d position =  motion->compute_position(param.start_time);
    marker_msg.pose.position.x = position[0];
    marker_msg.pose.position.y = position[1];
    marker_msg.pose.position.z = 0;

    auto quat = convert(position[2]);
    marker_msg.pose.orientation.x = quat.x;
    marker_msg.pose.orientation.y = quat.y;
    marker_msg.pose.orientation.z = quat.z;
    marker_msg.pose.orientation.w = quat.w;

    // Set the scale of the marker
    marker_msg.scale.x = radius / 1.0;
    marker_msg.scale.y = radius / 1.0;
    marker_msg.scale.z = 1.0;

    // Set the color of the marker
    marker_msg.color = make_color(1.0, 1.0, 0, 0.7);
    
    // Set the lifetime of the marker
    if (_rate <= 1)
      marker_msg.lifetime = convert(_timer_period);
    else
    {
      builtin_interfaces::msg::Duration duration;
      duration.sec = 1;
      duration.nanosec = 0;
      marker_msg.lifetime = duration;
    }

    return marker_msg;
  }

  visualization_msgs::msg::Marker make_path_marker(
        Element element,
        const RequestParam param)
  {
    // TODO Link the color, shape and size of marker to profile of trajectory
    const auto& trajectory = element.trajectory;
    const bool conflict = is_conflict(element.id);

    Marker marker_msg;

    marker_msg.header.frame_id = _frame_id; // map
    marker_msg.header.stamp = rmf_traffic_ros2::convert(param.start_time);
    marker_msg.ns = "trajectory";
    marker_msg.id = -1* element.id;
    marker_msg.type = marker_msg.LINE_STRIP;
    marker_msg.action = marker_msg.ADD;

    marker_msg.pose.orientation.w = 1;

    // Set the scale of the marker
    marker_msg.scale.x =  0.2;

    // Set the color
    if (conflict)
    {
      marker_msg.color = make_color(1.0, 0, 0, 0.7);
    }
    else
    {
      marker_msg.color = make_color(0.0, 1.0, 0, 0.7);
    }
    
    if (_rate <= 1)
      marker_msg.lifetime = convert(_timer_period);
    else
    {
      builtin_interfaces::msg::Duration duration;
      duration.sec = 1;
      duration.nanosec = 0;
      marker_msg.lifetime = duration;
    }

    const auto t_start_time = *trajectory.start_time();
    const auto start_time = std::max(t_start_time, param.start_time);
    const auto t_finish_time = *trajectory.finish_time();
    const auto end_time = std::min(t_finish_time, param.finish_time);
 
    auto it = trajectory.find(start_time);
    assert(it != trajectory.end());
    assert(trajectory.find(end_time) != trajectory.end());
    const auto motion = it->compute_motion();
    marker_msg.points.push_back(
          make_point(motion->compute_position(start_time)));

    // Add segment points untill the last segment
    for (; it < trajectory.find(end_time); it++)
    {
      assert(it != trajectory.end());
      const Eigen::Vector3d p = it->get_finish_position();
      marker_msg.points.push_back(make_point(p));
    }

    // Add either last segment point or position at end_time
    const Eigen::Vector3d p = t_finish_time < param.finish_time ?
        it->get_finish_position()
        : it->compute_motion()->compute_position(end_time);
    marker_msg.points.push_back(make_point(p));

    return marker_msg;
  }

  Point make_point(const Eigen::Vector3d tp, bool z = false)
  {
    Point p;
    p.x = tp[0];
    p.y = tp[1];
    p.z = z ? tp[2] : 0;
    return p;
  }

  builtin_interfaces::msg::Duration convert(rmf_traffic::Duration duration)
  {
    builtin_interfaces::msg::Duration result;
    result.sec = std::chrono::duration_cast<
        std::chrono::seconds>(duration).count();
    const auto nanoseconds = duration - std::chrono::seconds(result.sec);
    result.nanosec = nanoseconds.count();
    
    return result;
  }

  struct quaternion
  {
    double w, x, y, z;
  };

  quaternion convert(double yaw, double pitch = 0, double roll =0)
  {
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    quaternion q;
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;

    return q;
  }

  bool is_conflict(int64_t id)
  {
    if (std::find(_conflict_id.begin(), _conflict_id.end(), id) 
        != _conflict_id.end())
      return true;
    else 
      return false;
  }

  Color make_color(float r, float g, float b, float a = 1.0)
  {
    Color color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
  }

  void set_color_list(float alpha = 1)
  { 
    // List of colors that will be paired with graph ids.
    // Add additional colors here.
    // Orange
    _color_list.push_back(make_color(0.99, 0.5, 0.19, alpha));
    // Blue
    _color_list.push_back(make_color(0, 0.5, 0.8, alpha));     
    // Purple 
    _color_list.push_back(make_color(0.57, 0.12, 0.7, alpha));
    // Teal
    _color_list.push_back(make_color(0, 0.5, 0.5, alpha));
    // Lavender 
    _color_list.push_back(make_color(0.9, 0.74, 1.0, alpha));
    // Olive
    _color_list.push_back(make_color(0.5, 0.5, 0, alpha));
    // Cyan    
    _color_list.push_back(make_color(0.27, 0.94, 0.94, alpha));
    // Grey
    _color_list.push_back(make_color(0.5, 0.5, 0.5, alpha));
    // Maroon
    _color_list.push_back(make_color(0.5, 0, 0, alpha));
  }

  Color get_color(uint64_t id)
  {
    if (_color_map.find(id) != _color_map.end())
        return _color_map[id];
    else
    {
      Color color;
      if (id < _color_list.size())
      {
        color = _color_list[id];
      }
      else
      {
        // We assign a default color
        color = make_color(1, 1, 0, 0.5);
      }
      _color_map.insert(std::make_pair(id, color));
      return color;
    }
  }

  void update_level_cache()
  {
    // Update Level cahce when map is updated
    if (_map_msg.levels.size() > 0)
    {
      bool found = false;
      for (auto level : _map_msg.levels)
      {
        if (level.name == _rviz_param.map_name)
        {
          found = true;
          _has_level = true;
          _level = level;
            RCLCPP_INFO(this->get_logger(),"Level cache updated");
          break;
        }
      }
      if (!found)
      {
        _has_level = false;
        RCLCPP_INFO(this->get_logger(),"Level cache not updated");
      }
    }
  }

  struct RvizParam
  {
    std::string map_name;
    rmf_traffic::Duration query_duration;
    rmf_traffic::Duration start_duration;
  };

  double _rate;
  std::string _frame_id;
  std::vector<int64_t> _conflict_id;
  std::unordered_set<uint64_t> _marker_tracker; 
  std::vector<rmf_traffic::Trajectory> _trajectories;
  std::vector<Element> _elements;
  std::chrono::nanoseconds _timer_period;

  BuildingMap _map_msg;
  bool _has_level;
  Level _level;
  std::unordered_map<uint64_t, std_msgs::msg::ColorRGBA> _color_map;
  // Cache of predefined colors for use
  std::vector<Color> _color_list;

  rclcpp::TimerBase::SharedPtr _timer;
  rclcpp::Publisher<MarkerArray>::SharedPtr _marker_array_pub;
  rclcpp::Subscription<ScheduleConflict>::SharedPtr _conflcit_sub;
  rclcpp::callback_group::CallbackGroup::SharedPtr _cb_group_conflict_sub;
  rclcpp::Subscription<RvizParamMsg>::SharedPtr _param_sub;
  rclcpp::callback_group::CallbackGroup::SharedPtr _cb_group_param_sub;
  rclcpp::Subscription<BuildingMap>::SharedPtr _map_sub;
  rclcpp::callback_group::CallbackGroup::SharedPtr _cb_group_map_sub;

  rmf_schedule_visualizer::VisualizerDataNode& _visualizer_data_node;

  RvizParam _rviz_param;
};


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
  get_arg(args, "-n", node_name, "node name", false);

  std::string rate_string;
  get_arg(args, "-r", rate_string, "rate",false);
  double rate = rate_string.empty()? 1.0 : std::stod(rate_string);

  std::string map_name = "B1";
  get_arg(args, "-m", map_name, "map name", false);

  const auto visualizer_data_node =
    rmf_schedule_visualizer::VisualizerDataNode::make(node_name);

  if(!visualizer_data_node)
  {
    std::cerr << "Failed to initialize the fleet adapter node" << std::endl;
    return 1;
  }

  RCLCPP_INFO(
        visualizer_data_node->get_logger(),
        "VisualizerDataNode /" + node_name + " started...");

  auto rviz_node = std::make_shared<RvizNode>(
      "rviz_node",
      *visualizer_data_node,
      std::move(map_name),
      rate);

  rclcpp::executors::MultiThreadedExecutor executor{
    rclcpp::executor::ExecutorArgs(), 2
  };

  executor.add_node(visualizer_data_node);
  executor.add_node(rviz_node);

  executor.spin();
  RCLCPP_INFO(
        visualizer_data_node->get_logger(),
        "Closing down");

  rclcpp::shutdown();
}

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

  RvizNode(
      std::string node_name,
      rmf_schedule_visualizer::VisualizerDataNode& visualizer_data_node,
      std::string map_name,
      double rate = 1.0,
      std::string frame_id = "/map")
  : Node(node_name),
    _visualizer_data_node(visualizer_data_node),
    _rate(rate),
    _frame_id(frame_id)
  {
    // Initializing boolean to indicate whether level cache is relevant 
    _has_level = false;

    // TODO add a constructor for RvizParam
    _rviz_param.map_name = map_name;
    _rviz_param.query_duration = std::chrono::seconds(600);
    _rviz_param.start_duration = std::chrono::seconds(0);

    // Creating a timer with specified rate. This timer runs on the main thread.
    int64_t s = 1/ _rate ;
    auto sec = std::chrono::seconds(s);
    _timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(sec);
    _timer = this->create_wall_timer(
        _timer_period,
        std::bind(&RvizNode::timer_callback, this));

    // Creating publisher for marker_array
    _marker_array_pub = this->create_publisher<MarkerArray>(
        "dp2_marker_array",
        rclcpp::SystemDefaultsQoS());
    
    // Creating subscriber for schedule_conflict in separate threead
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

    // Creating subscriber for rviz_param in separate thread
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
              // When the map_name is updated, we update the local cache of level
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

          if (msg->query_duration > 0)
            _rviz_param.query_duration = std::chrono::seconds(msg->query_duration);

          if (msg->start_duration >= 0)
            _rviz_param.start_duration = std::chrono::seconds(msg->start_duration);
          RCLCPP_INFO(this->get_logger(),"Rviz parameters updated");
        },
        sub_param_opt);
    
    // Creating subcscriber for building_map in separate thread.
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
          // Storing building map message 
          _map_msg = *msg;
        },
        sub_map_opt);
  }

private:

  void timer_callback()
  {
    MarkerArray marker_array;

    // TODO store a cache of trajectories to prevent frequent access
    // update chache whenever mirror manager updates 
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

    // store the ids of active trajectories
    std::vector<uint64_t> active_id;

    // For each trajectory create two markers
    // 1) Current position 
    // 2) Path until param.finish_time
    for (const auto& element : _elements)
    {
      // Create markers for trajectories that are active within time range
      if (element.trajectory.find(traj_param.start_time) != element.trajectory.end())
      {
        active_id.push_back(element.id);

        auto location_marker = make_location_marker(element, traj_param);
        marker_array.markers.push_back(location_marker);

        auto path_marker = make_path_marker(element, traj_param);
        marker_array.markers.push_back(path_marker);

        // Adding to id to _marker_tracker 
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

    // publish marker_array
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
      node_marker.id = 1;
      node_marker.type = node_marker.POINTS;
      node_marker.action = node_marker.ADD;

      node_marker.pose.orientation.w = 1;

      // Set the scale of the marker
      node_marker.scale.x = 1.0;
      node_marker.scale.y = 1.0;
      node_marker.scale.z = 1.0;

      // Set the color
      node_marker.color.r = 0.5f;
      node_marker.color.g = 1.0f;
      node_marker.color.b = 0.0f;
      node_marker.color.a = 1.0;

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
      lane_marker.type = lane_marker.LINE_STRIP;
      lane_marker.scale.x = 0.1;
      lane_marker.color.r = 1.0f;
      lane_marker.color.g = 1.0f;
      lane_marker.color.b = 0.0f;
      lane_marker.color.a = 0.5;

      // Add node locations to point list 
      std::cout<<"Level name: "<<_level.name<<std::endl;
      std::cout<<"Nav Graph size: "<<_level.nav_graphs.size()<<std::endl;
      std::cout<<"Wall Graph vertices size: "<<_level.wall_graph.vertices.size()<<std::endl;

      for (const auto nav_graph : _level.nav_graphs)
      {
        // for (const auto vertex : nav_graph.vertices)
        // {
        //   node_marker.points.push_back(make_point({vertex.x, vertex.y, 0}));
        //   std::cout<<"Added X: "<<vertex.x<<" Y: "<<vertex.y<<std::endl;
        // }

        // Adding lane markers

        for (const auto edge : nav_graph.edges)
        {
          auto n1 = nav_graph.vertices[edge.v1_idx];
          auto n2 = nav_graph.vertices[edge.v2_idx];
          lane_marker.points.push_back(make_point({n1.x, n1.y, 0}));
          lane_marker.points.push_back(make_point({n2.x, n2.y, 0}));
        }
      }
      // Markers for lane directionality 

      // marker_array.markers.push_back(node_marker);
      marker_array.markers.push_back(lane_marker);
      std::cout<<"Added map marker"<<std::endl;
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

    //deleting the path marker 
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

    // Set the color
    marker_msg.color.r = 1.0f;
    marker_msg.color.g = 1.0f;
    marker_msg.color.b = 0.0f;
    marker_msg.color.a = 1.0;
    
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
    marker_msg.scale.x =  0.5;
    marker_msg.scale.y =  1.0;
    marker_msg.scale.z = 1.0;

    // Set the color
    if (conflict)
    {
      marker_msg.color.r = 1.0f;
      marker_msg.color.g = 0.0f;
    }
    else
    {
      marker_msg.color.r = 0.0f;
      marker_msg.color.g = 1.0f;
    }
    marker_msg.color.b = 0.0f;
    marker_msg.color.a = 0.5;
    
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
 
    // auto make_point = [](const Eigen::Vector3d& tp) -> Point
    // {
    //   Point p;
    //   p.x = tp[0];
    //   p.y = tp[1];
    //   p.z = 0;
    //   return p;
    // };

    auto it = trajectory.find(start_time);
    assert(it != trajectory.end());
    assert(trajectory.find(end_time) != trajectory.end());
    const auto motion = it->compute_motion();
    marker_msg.points.push_back(
          make_point(motion->compute_position(start_time)));

    for (; it <= trajectory.find(end_time); it++)
    {
      assert(it != trajectory.end());
      const Eigen::Vector3d p = it->get_finish_position();
      marker_msg.points.push_back(make_point(p));
    }

    return marker_msg;
  }

  Point make_point(const Eigen::Vector3d tp)
  {
    Point p;
    p.x = tp[0];
    p.y = tp[1];
    p.z = 0;
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

  std::string map_name = "level1";
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

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(visualizer_data_node);
  executor.add_node(rviz_node);

  executor.spin();
  RCLCPP_INFO(
        visualizer_data_node->get_logger(),
        "Closing down");

  rclcpp::shutdown();
}

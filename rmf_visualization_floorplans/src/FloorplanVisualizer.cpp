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


#include "FloorplanVisualizer.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <Eigen/Geometry>

#include <rclcpp_components/register_node_macro.hpp>

//==============================================================================
FloorplanVisualizer::FloorplanVisualizer(const rclcpp::NodeOptions& options)
: Node("floorplan_visualizer", options)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Configuring floorplan_visualizer..."
  );

  _current_level = this->declare_parameter("initial_map_name", "L1");
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter initial_map_name to %s", _current_level.c_str()
  );

  const auto transient_qos =
    rclcpp::QoS(10).transient_local();
  // Selectively disable intra-process comms for publishers abd subscriptions
  // for non-volatile topics so that this node can still run in a container
  // with intra-process comms enabled.
  rclcpp::PublisherOptionsWithAllocator<
    std::allocator<void>> ipc_pub_options;
  ipc_pub_options.use_intra_process_comm =
    rclcpp::IntraProcessSetting::Disable;
  _floorplan_pub = this->create_publisher<OccupancyGrid>(
    "/floorplan",
    transient_qos,
    std::move(ipc_pub_options)
  );

  rclcpp::SubscriptionOptionsWithAllocator<
    std::allocator<void>> ipc_sub_options;
  ipc_sub_options.use_intra_process_comm =
    rclcpp::IntraProcessSetting::Disable;
  _map_sub = this->create_subscription<BuildingMap>(
    "/map",
    transient_qos,
    [=](BuildingMap::ConstSharedPtr msg)
    {
      if (msg->levels.empty())
        return;

      for (const auto& level : msg->levels)
      {
        const auto& map_name = level.name;
        if (map_name.empty() || level.images.empty())
          continue;

        cv::Mat cv_img = cv::imdecode(
          cv::Mat(level.images[0].data), cv::IMREAD_GRAYSCALE);
        auto it = level.images.begin();
        ++it;
        // We blend all the other images into the first image
        // TODO(YV): Consider blending all the other layers into one image
        // at 0.5:0.5 ratio and then blend the first layer to this result at
        // 0.7:0.3
        for (; it != level.images.end(); ++it)
        {
          cv::Mat next_img = cv::imdecode(
            cv::Mat(it->data), cv::IMREAD_GRAYSCALE);
          cv::addWeighted(cv_img, 0.7, next_img, 0.3, 0.0, cv_img);
        }
        const auto& image = level.images[0];
        OccupancyGrid grid;
        grid.info.resolution = image.scale;
        grid.header.stamp = this->get_clock()->now();
        grid.header.frame_id = "map";
        grid.info.width = cv_img.cols;
        grid.info.height = cv_img.rows;
        grid.info.origin.position.x = image.x_offset;
        grid.info.origin.position.y = image.y_offset;
        grid.info.origin.position.z = -0.01;
        Eigen::Quaternionf q = Eigen::AngleAxisf(
          M_PI,
          Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(
          image.yaw, Eigen::Vector3f::UnitZ());
        grid.info.origin.orientation.x = q.x();
        grid.info.origin.orientation.y = q.y();
        grid.info.origin.orientation.z = q.z();
        grid.info.origin.orientation.w = q.w();
        // TODO(YV): Avoid these copies
        std::vector<uint8_t> u_data(
          cv_img.data, cv_img.data + cv_img.rows*cv_img.cols);
        std::vector<int8_t> occupancy_data;
        for (auto pix : u_data)
        {
          pix = round((int)(0xFF - pix)/255.0*100);
          occupancy_data.push_back(pix);
        }
        grid.data = std::move(occupancy_data);
        _grids.insert({map_name, std::move(grid)});
      }
      publish_grid();
    },
    ipc_sub_options);

  _param_sub = this->create_subscription<RvizParam>(
    "rmf_visualization/parameters",
    rclcpp::SystemDefaultsQoS(),
    [=](RvizParam::ConstSharedPtr msg)
    {
      if (msg->map_name.empty() || msg->map_name == _current_level)
        return;

      _current_level = msg->map_name;
      publish_grid();
    });

  RCLCPP_INFO(
    this->get_logger(),
    "Running floorplan_visualizer..."
  );
}

//==============================================================================
void FloorplanVisualizer::publish_grid()
{
  if (_grids.find(_current_level) == _grids.end())
  {
    auto msg = std::make_unique<OccupancyGrid>();
    msg->header.stamp = this->get_clock()->now();
    msg->header.frame_id = "map";
    _floorplan_pub->publish(std::move(msg));
    return;
  }
  auto msg = std::make_unique<OccupancyGrid>(_grids.at(_current_level));
  _floorplan_pub->publish(std::move(msg));
}

RCLCPP_COMPONENTS_REGISTER_NODE(FloorplanVisualizer)

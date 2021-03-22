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


#ifndef RMF_SCHEDULE_VISUALIZER__VISUALIZERDATA_HPP
#define RMF_SCHEDULE_VISUALIZER__VISUALIZERDATA_HPP

#include <rmf_traffic_ros2/schedule/Negotiation.hpp>

#include <rmf_traffic/Trajectory.hpp>
#include <rmf_traffic/schedule/Viewer.hpp>
#include <rmf_traffic_ros2/StandardNames.hpp>

#include <rmf_utils/impl_ptr.hpp>

#include <rmf_schedule_visualizer/CommonData.hpp>

#include <rclcpp/node.hpp>

#include <set>
#include <mutex>
#include <vector>
#include <unordered_set>

namespace rmf_schedule_visualizer {

class ScheduleDataNode : public rclcpp::Node
{
public:
  using Element = rmf_traffic::schedule::Viewer::View::Element;
  using Negotiation = rmf_traffic_ros2::schedule::Negotiation;

  /// Builder function which returns a pointer to ScheduleDataNode when
  /// the Mirror Manager is readied.
  /// A nullptr is returned if initialization fails.
  ///
  /// \param[in] node_name
  ///   The name of the node
  ///
  /// \param[in] wait_time
  ///   The waiting duration to discover the rmf_traffic_schedule node
  static std::shared_ptr<ScheduleDataNode> make(
    const std::string& node_name,
    rmf_traffic::Duration wait_time = std::chrono::seconds(10));

  /// Query the shcedule database for elements
  std::vector<Element> get_elements(const RequestParam& request_param) const;

  /// Get the ids of all participants that have active conflicts
  const std::unordered_set<uint64_t> get_conflict_ids() const;

  /// Get the groups of participants ids that have active conflicts with each other
  const std::vector<std::vector<uint64_t>> get_conflict_groups() const;

  /// Get the elements submitted by participants during a negotiation
  const std::vector<Element> get_negotiation_trajectories(
    uint64_t conflict_version, const std::vector<uint64_t>& sequence) const;

  /// Get the current time
  rmf_traffic::Time now();

  /// Get a mutable reference to the mutex used by the ScheduleDataNode
  std::mutex& get_mutex();

  /// Get a shared pointer to the ROS 2 negotiation interface
  std::shared_ptr<Negotiation> get_negotiation();

  class Implementation;

private:
  ScheduleDataNode(std::string _node_name);
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

} // namespace rmf_schedule_visualizer

#endif // RMF_SCHEDULE_VISUALIZER__VISUALIZERDATA_HPP

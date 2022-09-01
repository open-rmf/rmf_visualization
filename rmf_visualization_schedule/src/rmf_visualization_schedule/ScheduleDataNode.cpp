/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <rmf_visualization_schedule/ScheduleDataNode.hpp>

#include <rmf_traffic_msgs/msg/negotiation_conclusion.hpp>
#include <rmf_traffic_msgs/msg/negotiation_notice.hpp>

#include <rmf_traffic_ros2/schedule/MirrorManager.hpp>
#include <rmf_traffic_ros2/StandardNames.hpp>
#include <rmf_traffic_ros2/Time.hpp>
#include <rmf_traffic_ros2/Trajectory.hpp>

namespace rmf_visualization_schedule {

//==============================================================================
class ScheduleDataNode::Implementation
{
public:

  using ConflictNotice = rmf_traffic_msgs::msg::NegotiationNotice;
  using ConflictConclusion = rmf_traffic_msgs::msg::NegotiationConclusion;

  Implementation() {}
  Implementation(const Implementation&) {}

  struct Data
  {
    rmf_traffic_ros2::schedule::MirrorManager mirror;

    Data(rmf_traffic_ros2::schedule::MirrorManager mirror_)
    : mirror(std::move(mirror_))
    {
      // Do nothing
    }
  };

  rclcpp::Subscription<ConflictNotice>::SharedPtr conflict_notice_sub;
  rclcpp::Subscription<ConflictConclusion>::SharedPtr conflict_conclusion_sub;

  std::unique_ptr<Data> data;
  mutable std::mutex mutex;
  std::unordered_map<
    rmf_traffic::schedule::Version,
    std::vector<rmf_traffic::schedule::ParticipantId>> conflicts;

  std::shared_ptr<rmf_traffic_ros2::schedule::Negotiation> negotiation =
    nullptr;

  void start(Data data);
};

//==============================================================================
std::shared_ptr<ScheduleDataNode> ScheduleDataNode::make(
  const std::string& node_name,
  rmf_traffic::Duration wait_time,
  const rclcpp::NodeOptions& options)
{
  const auto start_time = std::chrono::steady_clock::now();
  std::shared_ptr<ScheduleDataNode> schedule_data(
    new ScheduleDataNode(std::move(node_name), options));

  // Creating a mirror manager that queries over all
  // Spacetime in the database schedule
  auto mirror_mgr_future = rmf_traffic_ros2::schedule::make_mirror(
    schedule_data, rmf_traffic::schedule::query_all(),
    &schedule_data->_pimpl->mutex);

  const auto stop_time = start_time + wait_time;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < stop_time)
  {
    rclcpp::spin_some(schedule_data);
    using namespace std::chrono_literals;

    if (mirror_mgr_future.wait_for(0s) == std::future_status::ready)
    {
      schedule_data->_pimpl->start(
        Implementation::Data{mirror_mgr_future.get()});
      // retrieve/construct mirrors, snapshots and negotiation object
      schedule_data->_pimpl->negotiation = std::make_shared<Negotiation>(
        *schedule_data, schedule_data->_pimpl->data->mirror.view());
      return schedule_data;
    }
  }

  RCLCPP_ERROR(
    schedule_data->get_logger(),
    "Mirror was not initialized in enough time [%ss]!",
    std::to_string(rmf_traffic::time::to_seconds(wait_time)).c_str());
  return nullptr;
}

//==============================================================================
ScheduleDataNode::ScheduleDataNode(
  std::string node_name,
  const rclcpp::NodeOptions& options)
: Node(std::move(node_name), options),
  _pimpl(rmf_utils::make_impl<Implementation>(Implementation{}))
{
  this->_pimpl->conflict_notice_sub =
    this->create_subscription<Implementation::ConflictNotice>(
    rmf_traffic_ros2::NegotiationNoticeTopicName,
    rclcpp::ServicesQoS().reliable(),
    [this](Implementation::ConflictNotice::UniquePtr msg)
    {
      std::lock_guard<std::mutex> guard(this->_pimpl->mutex);
      this->_pimpl->conflicts[msg->conflict_version] =
      msg->participants;
    });

  this->_pimpl->conflict_conclusion_sub =
    this->create_subscription<Implementation::ConflictConclusion>(
    rmf_traffic_ros2::NegotiationConclusionTopicName,
    rclcpp::ServicesQoS(),
    [this](Implementation::ConflictConclusion::UniquePtr msg)
    {
      std::lock_guard<std::mutex> guard(this->_pimpl->mutex);
      this->_pimpl->conflicts.erase(msg->conflict_version);
    });
}

//==============================================================================
void ScheduleDataNode::Implementation::start(
  ScheduleDataNode::Implementation::Data data_)
{
  data = std::make_unique<Data>(std::move(data_));
  data->mirror.update();
}

//==============================================================================
auto ScheduleDataNode::get_elements(
  const RequestParam& request_param) const -> std::vector<Element>
{
  std::vector<Element> elements;
  const std::vector<std::string> maps {std::move(request_param.map_name)};
  const auto query = rmf_traffic::schedule::make_query(
    maps,
    &request_param.start_time,
    &request_param.finish_time);

  const auto view = _pimpl->data->mirror.view()->query(query);

  for (const auto& element : view)
    elements.push_back(element);

  return elements;
}

//==============================================================================
rmf_traffic::Time ScheduleDataNode::now()
{
  return rmf_traffic_ros2::convert(get_clock()->now());
}

//==============================================================================
std::mutex& ScheduleDataNode::get_mutex()
{
  return _pimpl->mutex;
}

//==============================================================================
const std::unordered_set<uint64_t> ScheduleDataNode::get_conflict_ids() const
{
  std::unordered_set<uint64_t> conflict_id;
  for (const auto& conflict : _pimpl->conflicts)
  {
    std::copy(conflict.second.begin(),
      conflict.second.end(),
      std::inserter(conflict_id, conflict_id.end()));
  }

  return conflict_id;
}

//==============================================================================
const std::vector<std::vector<uint64_t>>
ScheduleDataNode::get_conflict_groups() const
{
  std::vector<std::vector<uint64_t>> conflicts;
  for (const auto& conflict : _pimpl->conflicts)
    conflicts.push_back(conflict.second);

  return conflicts;
}

//==============================================================================
auto ScheduleDataNode::get_negotiation_trajectories(
  uint64_t conflict_version, const std::vector<uint64_t>& sequence) const
-> const std::vector<Element>
{
  std::vector<Element> trajectory_elements;

  const auto table_view = _pimpl->negotiation->table_view(
    conflict_version, sequence);
  if (!table_view)
  {
    RCLCPP_WARN(
      this->get_logger(), "table_view for conflict %ld not found!",
      conflict_version);
    return trajectory_elements;
  }

  rmf_traffic::RouteId route_id = 0;
  const auto add_route = [&route_id, &table_view, &trajectory_elements]
      (const rmf_traffic::Route& route,
      rmf_traffic::schedule::ParticipantId id)
    {
      Element e {
        id,
        0,
        route_id,
        std::make_shared<rmf_traffic::Route>(route),
        *table_view->get_description(id)
      };
      trajectory_elements.push_back(e);
      ++route_id;
    };

  auto itin = table_view->submission();
  if (itin)
  {
    const auto& routes = *itin;
    for (const auto& route : routes)
      add_route(route, table_view->participant_id());
  }

  for (auto proposal : table_view->base_proposals())
  {
    for (auto route : proposal.itinerary)
      add_route(route, proposal.participant);
  }
  return trajectory_elements;
}

//==============================================================================
std::shared_ptr<ScheduleDataNode::Negotiation>
ScheduleDataNode::get_negotiation()
{
  return _pimpl->negotiation;
}


} // namespace rmf_visualization_schedule

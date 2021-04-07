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

#ifndef RMF_VISUALIZATION_SCHEDULE__TRAJECTORYSERVER_HPP
#define RMF_VISUALIZATION_SCHEDULE__TRAJECTORYSERVER_HPP

#include <rmf_visualization_schedule/ScheduleDataNode.hpp>

#include <memory>

namespace rmf_visualization_schedule {

class TrajectoryServer
{
public:

  /// Builder function which returns a pointer to the TrajectoryServer
  /// after the websocket server has successfully started.
  /// A nullptr is returned if initialization fails.
  ///
  /// \param[in] port
  ///   The port number to run the websocket server on
  ///
  /// \param[in] schedule_data_node
  ///   A shared pointer to a ScheduleDataNode
  static std::shared_ptr<TrajectoryServer> make(
    uint16_t port,
    ScheduleDataNodePtr schedule_data_node);

  ~TrajectoryServer();

  class Implementation;

private:
  TrajectoryServer();
  rmf_utils::impl_ptr<Implementation> _pimpl;

};

} // namespace rmf_visualization_schedule

#endif // RMF_VISUALIZATION_SCHEDULE__TRAJECTORYSERVER_HPP
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

#include "Server.hpp"

namespace rmf_schedule_visualizer {


  std::shared_ptr<Server> Server::make(
      uint16_t port,
      VisualizerDataNode& visualizer_data_node)
  : _port(port),
    _visualizer_data_node(visualizer_data_node)
  {

    
  }
    
  /// Run the server after initialization
  void run();

private:
  using con_list= std::set<connection_hdl,std::owner_less<connection_hdl>>;

  /// Constructor with port number
  Server(uint16_t port = 8006);

  void on_open(connection_hdl hdl);

  void on_close(connection_hdl hdl);
 
  void on_message(connection_hdl hdl, server::message_ptr msg);

  /// Set the interanal reference to the visualizer_data_node
  void set_mirror(VisualizerDataNode& visualizer_data_node);

  ~Server();

} //namespace rmf_schedule_visualizer
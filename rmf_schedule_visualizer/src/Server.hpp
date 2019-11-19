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

#ifndef RMF_SCHEDULE_VISUALIZER__SRC__SERVER_HPP
#define RMF_SCHEDULE_VISUALIZER__SRC__SERVER_HPP

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <set>

#include "VisualizerData.hpp"
#include <rmf_traffic/Trajectory.hpp>
#include <rmf_schedule_visualizer/CommonData.hpp>

namespace rmf_schedule_visualizer {

using server = websocketpp::server<websocketpp::config::asio>;

using websocketpp::connection_hdl;
using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;

class Server
{
public:
  /// Builder function which returns a pointer to Server after
  /// the websocket has successfully started.
  /// A nullptr is returned if initialization fails. 
  static std::shared_ptr<Server> make(
      uint16_t port,
      VisualizerDataNode& visualizer_data_node);

~Server();

private:
  using con_list= std::set<connection_hdl,std::owner_less<connection_hdl>>;

  struct Data
  {
    std::string request;
    std::string response;

    Data(
      std::string request_,
      std::string response_)
    : request(std::move(request_)),
      response(std::move(response_))
    {
      // Do nothing
    }
  };


  /// Run the server after initialization
  void run();

  /// Constructor with port number
  Server(uint16_t port, VisualizerDataNode& visualizer_data_node);

  void on_open(connection_hdl hdl);

  void on_close(connection_hdl hdl);
 
  void on_message(connection_hdl hdl, server::message_ptr msg);

  /// Set the interanal reference to the visualizer_data_node
  void set_mirror(VisualizerDataNode& visualizer_data_node);

  bool parse_request(server::message_ptr msg, RequestParam& request_param);

  void parse_trajectories(
      std::vector<rmf_traffic::Trajectory>& trajectories,
      std::string& response);

  server _server;
  con_list _connections;
  uint16_t _port = 8006;
  std::thread _server_thread;
  VisualizerDataNode& _visualizer_data_node;
  bool _is_initialized = false;
  std::unique_ptr<Data> data;
};

} // namespace rmf_schedule_visualizer

#endif // RMF_SCHEDULE_VISUALIZER__SRC__SERVER_HPP
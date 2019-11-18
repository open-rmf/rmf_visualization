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
  {
    if (port<0)
      return nullptr;
    
    std::shared_ptr<Server> server_ptr(new Server(port, visualizer_data_node));

    return server_ptr;
    
  }
    
  /// Run the server after initialization
  void Server::run()
  {
    if (_is_initialized)
    {
      _server.set_reuse_addr(true);
      _server.listen(_port);
      _server.start_accept();
      _server_thread = std::thread([&](){ this->_server.run(); });
    }
  }

  /// Constructor with port number
  Server::Server(uint16_t port, VisualizerDataNode& visualizer_data_node )
  : _port(port),
    _visualizer_data_node(visualizer_data_node)
  {
    _server.init_asio();
    _server.set_open_handler(bind(&Server::on_open,this,_1));
    _server.set_close_handler(bind(&Server::on_close,this,_1));
    _server.set_message_handler(bind(&Server::on_message,this,_1,_2));
    _is_initialized = true; 
  }

  void Server::on_open(connection_hdl hdl)
  {
    _connections.insert(hdl);
  }

  void Server::on_close(connection_hdl hdl)
  {
    _connections.erase(hdl);
  }
 
  void Server::on_message(connection_hdl hdl, server::message_ptr msg)
  {
    if (msg->get_payload().compare("shutdown\n")==0)
    {
    std::string payload = "Shutting down...";
    auto response_msg = msg;
    response_msg->set_payload(payload);
    _server.send(hdl, response_msg);
    }
    else
    {
     _server.send(hdl, msg);
    }

  }

  /// Set the interanal reference to the visualizer_data_node
  void Server::set_mirror(VisualizerDataNode& visualizer_data_node)
  {
    //_visualizer_data_node=visualizer_data_node;

  }

  Server::~Server()
  {
    //thread safe access to _connections
    const auto connection_copies = _connections;
    for (auto& connection : connection_copies)
    {
      _server.close(connection, websocketpp::close::status::normal, "shutdown");
    }

    if(_server_thread.joinable())
    {
      _server.stop();

      _server_thread.join();
    }
  }

} //namespace rmf_schedule_visualizer
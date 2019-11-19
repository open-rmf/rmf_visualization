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
#include <json.hpp>

namespace rmf_schedule_visualizer {
using json = nlohmann::json;

std::shared_ptr<Server> Server::make(
    uint16_t port,
    VisualizerDataNode& visualizer_data_node)
{
  std::shared_ptr<Server> server_ptr(new Server(port, visualizer_data_node));
  try
  { 
    server_ptr->run();
  }
  catch (std::exception& e)
  {
    std::cerr << "Failed to start the Server: "<<e.what() << std::endl;
    return nullptr;
  }
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

/// Constructor with port number and reference to visualizer_data_node
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
  std::string response;
  RequestParam request_param;
  auto ok = parse_msg(msg,request_param);
  std::cout<<"OK: "<<ok<<std::endl;
  /*
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
  */
  if(ok)
  {
    auto trajectories = _visualizer_data_node.get_trajectories(request_param);
    parse_trajectories(trajectories, response);
  }

  server::message_ptr response_msg =std::move(msg);
  response_msg->set_payload(response);
  _server.send(hdl, response_msg);


}

bool Server::parse_msg(server::message_ptr msg, RequestParam& request_param)
{
  std::string msg_payload = msg->get_payload();
  try
  {
    json j = json::parse(msg_payload);
    for (json::iterator it = j.begin(); it != j.end(); ++it)
    {
    std::cout << *it << '\n';
      }
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }
  
  return true;

}

void Server::parse_trajectories(
    std::vector<rmf_traffic::Trajectory>& trajectories,
    std::string& response)
{
  for (rmf_traffic::Trajectory trajectory : trajectories)
  {
    for (auto it = trajectory.begin(); it!= trajectory.end(); it++)
    {
      auto finish_time = it->get_finish_time();
      auto finish_position = it->get_finish_position();
      auto profile = it->get_profile();  
    }
  } 
}

/// Set the interanal reference to the visualizer_data_node
void Server::set_mirror(VisualizerDataNode& visualizer_data_node)
{

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
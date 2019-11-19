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

  auto ok = parse_request(msg,request_param);
  std::cout<<"OK: "<<ok<<std::endl;

  if(ok)
  {
    auto trajectories = _visualizer_data_node.get_trajectories(request_param);
    parse_trajectories(trajectories, response);
  }

  std::cout<<"Response: "<<response<<std::endl;
 
  server::message_ptr response_msg =std::move(msg);
  response_msg->set_payload(response);
  _server.send(hdl, response_msg);


}

bool Server::parse_request(server::message_ptr msg, RequestParam& request_param)
{

  using namespace std::chrono_literals;

  // TODO(YV) get names of the keys from config yaml file 

  std::string msg_payload = msg->get_payload();
  try
  {
    json j = json::parse(msg_payload);
    if (j.size() != 2)
      return false;
    
    if(j.count("request") != 1 || j.count("param") != 1)
      return false;
    
    json j_param = j["param"];
    if (j_param.size() != 3)
      return false;
    
    if(
        j_param.count("map_name") == 1 and 
        j_param.count("start_time") == 1 and 
        j_param.count("finish_time") == 1 and 
        j_param["finish_time"] > j_param["start_time"])
        {
          request_param.map_name = j_param["map_name"];
          std::chrono::nanoseconds start_time_nano(j_param["start_time"]);
          std::chrono::nanoseconds finish_time_nano(j_param["finish_time"]);
          // TODO(YV) fix this 
          request_param.start_time = std::chrono::steady_clock::now() +
              start_time_nano;
          request_param.finish_time = std::chrono::steady_clock::now() +
              finish_time_nano ;

          return true;

        }
    else
      return false;
  }

  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
    return false;
  }
}

void Server::parse_trajectories(
    std::vector<rmf_traffic::Trajectory>& trajectories,
    std::string& response)
{

  json _j_res = { {"response", "trajectory"}, {"values", {}} };
  json _j_traj ={ {"shape", {}}, {"segments", {} } };
  json _j_seg = { {"x", {} }, {"v", {} }, {"t", {}} };

  auto j_res = _j_res;

  for (rmf_traffic::Trajectory trajectory : trajectories)
  {
    auto j_traj = _j_traj;
    j_traj["shape"].push_back("box");
    for (auto it = trajectory.begin(); it!= trajectory.end(); it++)
    {
      auto j_seg = _j_seg;

      auto finish_time = it->get_finish_time();
      auto finish_position = it->get_finish_position();
      auto finish_velocity = it->get_finish_velocity();

      j_seg["x"] = finish_position[0];
      j_seg["v"] = finish_velocity[0];
      j_seg["t"] = finish_time.time_since_epoch().count();
      j_traj["segments"].push_back(j_seg);
    }
    j_res["values"].push_back(j_traj);

  } 

  response = j_res.dump();
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
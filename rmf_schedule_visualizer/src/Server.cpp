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

#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/geometry/Box.hpp>


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

  if(ok)
  {
    std::cout << "Valid request received" << std::endl;
    std::cout << "map_name: "
        << request_param.map_name << std::endl;
    std::cout << "start_time: "
        << request_param.start_time.time_since_epoch().count() << std::endl;
    std::cout<<"finish_time: "
        << request_param.finish_time.time_since_epoch().count() << std::endl;

    std::lock_guard<std::mutex> lock(_visualizer_data_node.get_mutex());
    auto trajectories = _visualizer_data_node.get_trajectories(request_param);
    parse_trajectories(trajectories, response);
    std::cout << "Response: " << response << std::endl;
  }
  else
  {
    std::cout << "Invalid request received" << std::endl;
  }
 
  server::message_ptr response_msg = std::move(msg);
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
          // We assume the time parameters are passed as strings and 
          // require conversion to rmf_traffic::Time

          std::string start_time_string = j_param["start_time"];
          std::string finish_time_string = j_param["finish_time"];

          std::chrono::nanoseconds start_time_nano(
              std::stoull(start_time_string));
          std::chrono::nanoseconds finish_time_nano(
              std::stoull(finish_time_string));

          request_param.start_time = rmf_traffic::Time(
              rmf_traffic::Duration(start_time_nano));
          request_param.finish_time = rmf_traffic::Time(
              rmf_traffic::Duration(finish_time_nano));

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
  // Templates used for response generation
  json _j_res = { {"response", "trajectory"}, {"values", {} } };
  json _j_traj ={ {"shape", {} }, {"dimensions", {} }, {"segments", {} } };
  json _j_seg = { {"x", {} }, {"v", {} }, {"t", {} } };

  auto j_res = _j_res;

  try
  {
    for (rmf_traffic::Trajectory trajectory : trajectories)
    {
      auto j_traj = _j_traj;
      // TODO(YV) interpret the shape from profile 
      // This will fail if shape is Box
      j_traj["shape"].push_back("circle");
      const auto &circle = static_cast<const rmf_traffic::geometry::Circle&>(
          trajectory.begin()->get_profile()->get_shape()->source());
      j_traj["dimensions"].push_back(circle.get_radius());

      for (auto it = trajectory.begin(); it!= trajectory.end(); it++)
      {
        auto j_seg = _j_seg;
        auto finish_time = it->get_finish_time();
        auto finish_position = it->get_finish_position();
        auto finish_velocity = it->get_finish_velocity();
        j_seg["x"].push_back(
            {finish_position[0],finish_position[1],finish_position[2]});
        j_seg["v"].push_back(
            {finish_velocity[0],finish_velocity[1],finish_velocity[2]});
        j_seg["t"] = std::to_string(finish_time.time_since_epoch().count());
        j_traj["segments"].push_back(j_seg);
      }
      j_res["values"].push_back(j_traj);

    }
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }
  response = j_res.dump();
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
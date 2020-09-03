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

#include <rmf_traffic/Motion.hpp>

namespace rmf_schedule_visualizer {

std::shared_ptr<Server> Server::make(
  uint16_t port,
  std::shared_ptr<VisualizerDataNode> visualizer_data_node)
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
  assert(_is_initialized);
  _server.set_reuse_addr(true);
  _server.listen(_port);
  _server.start_accept();
  _server_thread = std::thread([&]() {this->_server.run();});
}

/// Constructor with port number and reference to visualizer_data_node
Server::Server(uint16_t port,
  std::shared_ptr<VisualizerDataNode> visualizer_data_node)
: _port(port),
  _visualizer_data_node(std::move(visualizer_data_node))
{
  _server.init_asio();
  _server.set_open_handler(bind(&Server::on_open, this, _1));
  _server.set_close_handler(bind(&Server::on_close, this, _1));
  _server.set_message_handler(bind(&Server::on_message, this, _1, _2));
  _is_initialized = true;

  // TODO: Eliminate undefined behavior from this implementation
  // https://github.com/osrf/rmf_schedule_visualizer/pull/69#discussion_r476166450

  //set up callbacks for negotiations
  auto status_update_cb = [this](
    uint64_t conflict_version,
    rmf_traffic::schedule::Negotiation::Table::ViewerPtr table_view)
    {
      RCLCPP_DEBUG(_visualizer_data_node->get_logger(),
        "======== conflict callback version: %llu! ==========",
        conflict_version);

      nlohmann::json negotiation_json;
      negotiation_json["type"] = "negotiation_status";
      negotiation_json["conflict_version"] = conflict_version;
      negotiation_json["participant_id"] = table_view->participant_id();
      negotiation_json["participant_name"] =
        table_view->get_description(table_view->participant_id())->name();
      negotiation_json["defunct"] = table_view->defunct();
      negotiation_json["rejected"] = table_view->rejected();
      negotiation_json["forfeited"] = table_view->forfeited();

      auto versioned_sequence = table_view->sequence();
      for (auto versionedkey : versioned_sequence)
        negotiation_json["sequence"].push_back(versionedkey.participant);

      std::string conflict_str = negotiation_json.dump();
      for (auto connection : _connections)
        _server.send(connection, conflict_str,
          websocketpp::frame::opcode::text);
    };
  _visualizer_data_node->_negotiation->on_status_update(std::move(
      status_update_cb));

  auto conclusion_cb = [this](
    uint64_t conflict_version, bool resolved)
    {
      RCLCPP_DEBUG(_visualizer_data_node->get_logger(),
        "======== conflict concluded: %llu resolved: %d ==========",
        conflict_version, resolved ? 1 : 0);

      nlohmann::json json_msg;
      json_msg["type"] = "negotiation_conclusion";
      json_msg["conflict_version"] = conflict_version;
      json_msg["resolved"] = resolved;

      std::string json_str = json_msg.dump();
      for (auto connection : _connections)
        _server.send(connection, json_str, websocketpp::frame::opcode::text);
    };
  _visualizer_data_node->_negotiation->on_conclusion(std::move(conclusion_cb));

}

void Server::on_open(connection_hdl hdl)
{
  _connections.insert(hdl);
  RCLCPP_INFO(_visualizer_data_node->get_logger(),
    "Connected with a client");
}

void Server::on_close(connection_hdl hdl)
{
  _connections.erase(hdl);
  RCLCPP_INFO(_visualizer_data_node->get_logger(),
    "Disconnected with a client");

}

void Server::on_message(connection_hdl hdl, server::message_ptr msg)
{
  std::string response = "";

  if (msg->get_payload().empty())
  {
    RCLCPP_INFO(_visualizer_data_node->get_logger(), "Empty request received");
    return;
  }

  auto ok = parse_request(msg, response);

  if (ok)
  {
    RCLCPP_INFO(_visualizer_data_node->get_logger(),
      "Response: %s", response.c_str());
    server::message_ptr response_msg = std::move(msg);
    response_msg->set_payload(response);
    _server.send(hdl, response_msg);
  }
  else
  {
    RCLCPP_INFO(_visualizer_data_node->get_logger(),
      "Invalid request received");
  }

}

bool Server::parse_request(server::message_ptr msg, std::string& response)
{
  using namespace std::chrono_literals;

  // TODO(YV) get names of the keys from config yaml file
  std::string msg_payload = msg->get_payload();

  try
  {
    json j = json::parse(msg_payload);

    if (j.size() != 2)
      return false;

    if (j.count("request") != 1 || j.count("param") != 1)
      return false;

    if (j["request"] == "trajectory")
    {
      json j_param = j["param"];

      if (j_param.size() != 3)
        return false;
      if (j_param.count("map_name") != 1)
        return false;
      if (j_param.count("duration") != 1)
        return false;
      if (j_param.count("trim") != 1)
        return false;

      // We assume the duration is passed as a string representing milliseconds
      std::uint64_t duration_num = j_param["duration"];
      std::chrono::milliseconds duration(duration_num);

      // All checks have passed
      RequestParam request_param;
      request_param.map_name = j_param["map_name"];
      request_param.start_time = _visualizer_data_node->now();
      request_param.finish_time = request_param.start_time +
        duration;

      RCLCPP_INFO(_visualizer_data_node->get_logger(),
        "Trajectory Response recived with map_name [%s] and duration [%s]ms",
        request_param.map_name.c_str(), std::to_string(duration_num).c_str());

      std::lock_guard<std::mutex> lock(_visualizer_data_node->get_mutex());
      auto elements = _visualizer_data_node->get_elements(request_param);

      bool trim = j_param["trim"];
      response = parse_trajectories("trajectory",
          _visualizer_data_node->get_server_conflicts(), elements, trim,
          request_param);

      return true;

    }

    else if (j["request"] == "time")
    {
      std::cout << "Time request received" << std::endl;
      json j_res = _j_res;
      j_res["response"] = "time";
      j_res["values"].push_back(
        _visualizer_data_node->now().time_since_epoch().count());
      response = j_res.dump();
      return true;
    }

    else if (j["request"] == "negotiation_trajectory")
    {
      RCLCPP_INFO(_visualizer_data_node->get_logger(),
        "Received Negotiation Trajectory request");

      uint64_t conflict_version = j["param"]["conflict_version"];
      std::vector<uint64_t> sequence = j["param"]["sequence"];

      auto trajectory_elements =
        _visualizer_data_node->get_negotiation_trajectories(conflict_version,
          sequence);
      const auto now = std::chrono::steady_clock::now();

      RequestParam req;
      req.start_time = now;
      req.finish_time = now + 3min;

      response = parse_trajectories("negotiation_trajectory",
          { { conflict_version } },
          trajectory_elements, false, req);

      return true;
    }

    else
    {
      return false;
    }

  }

  catch (const std::exception& e)
  {
    RCLCPP_ERROR(_visualizer_data_node->get_logger(),
      "Error: %s", std::to_string(*e.what()).c_str());
    return false;
  }

}

std::string Server::parse_trajectories(
  const std::string& response_type,
  const std::vector<std::vector<uint64_t>>& conflicts,
  const std::vector<Element>& elements,
  const bool trim,
  const RequestParam& request_param)
{
  std::string response;
  auto j_res = _j_res;
  j_res["response"] = response_type;
  j_res["conflicts"] = conflicts;

  try
  {
    for (const auto& element : elements)
    {
      const auto& trajectory = element.route.trajectory();

      auto j_traj = _j_traj;
      j_traj["robot_name"] = element.description.name();
      j_traj["fleet_name"] = element.description.owner();
      j_traj["id"] = element.participant;
      j_traj["route_id"] = element.route_id;
      j_traj["shape"] = "circle";
      j_traj["dimensions"] = element.description.profile().footprint()
        ->get_characteristic_length();

      auto add_segment = [&](rmf_traffic::Time finish_time,
          Eigen::Vector3d finish_position,
          Eigen::Vector3d finish_velocity)
        {
          auto j_seg = _j_seg;
          j_seg["x"] =
          {finish_position[0], finish_position[1], finish_position[2]};
          j_seg["v"] =
          {finish_velocity[0], finish_velocity[1], finish_velocity[2]};
          j_seg["t"] = std::chrono::duration_cast<std::chrono::milliseconds>(
            finish_time.time_since_epoch()).count();
          j_traj["segments"].push_back(j_seg);
        };

      if (trim)
      {
        const auto start_time = std::max(
          *trajectory.start_time(), request_param.start_time);
        const auto end_time = std::min(
          *trajectory.finish_time(), request_param.finish_time);
        auto it = trajectory.find(start_time);
        assert(it != trajectory.end());
        assert(trajectory.find(end_time) != trajectory.end());

        // Add the trimmed start
        auto begin = it; --begin;
        auto end = it; ++end;
        auto motion = rmf_traffic::Motion::compute_cubic_splines(begin, end);
        add_segment(start_time,
          motion->compute_position(start_time),
          motion->compute_velocity(start_time));

        // Add the segments in between
        for (; it < trajectory.find(end_time); it++)
        {
          assert(it != trajectory.end());
          add_segment(it->time(),
            it->position(),
            it->velocity());
        }

        // Add the trimmed end
        assert(it != trajectory.end());
        begin = it; --begin;
        end = it; ++end;
        motion = rmf_traffic::Motion::compute_cubic_splines(begin, end);
        add_segment(end_time,
          motion->compute_position(end_time),
          motion->compute_velocity(end_time));
      }

      else
      {
        for (auto it = trajectory.begin(); it != trajectory.end(); it++)
        {
          add_segment(it->time(),
            it->position(),
            it->velocity());
        }
      }
      j_res["values"].push_back(j_traj);
    }
  }

  catch (const std::exception& e)
  {
    RCLCPP_ERROR(_visualizer_data_node->get_logger(),
      "Error: %s", std::to_string(*e.what()).c_str());
    return "";
  }

  response = j_res.dump();
  return response;
}

Server::~Server()
{
  //Thread safe access to _connections
  const auto connection_copies = _connections;
  for (auto& connection : connection_copies)
  {
    _server.close(connection, websocketpp::close::status::normal, "shutdown");
  }

  if (_server_thread.joinable())
  {
    _server.stop();
    _server_thread.join();
  }
}

} //namespace rmf_schedule_visualizer

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
#include <json.hpp>
#include <jwt/jwt.hpp>

#include <rmf_visualization_schedule/CommonData.hpp>
#include <rmf_visualization_schedule/TrajectoryServer.hpp>

#include <rmf_traffic/Motion.hpp>
#include <rmf_traffic/Trajectory.hpp>

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include <exception>
#include <functional>
#include <thread>

namespace rmf_visualization_schedule {

//==============================================================================
class TrajectoryServer::Implementation
{
public:

  using connection_hdl = websocketpp::connection_hdl;
  using Connections = std::set<connection_hdl, std::owner_less<connection_hdl>>;
  using Server = websocketpp::server<websocketpp::config::asio>;
  using Json = nlohmann::json;
  using Element = rmf_traffic::schedule::Viewer::View::Element;

  Implementation() {}
  Implementation(const Implementation&) {}

  std::shared_ptr<Server> server;
  Connections connections;
  Connections negotiation_subscribed_connections;
  std::thread server_thread;
  ScheduleDataNodePtr schedule_data_node;
  bool initialized = false;

  // Templates used for response generation
  const Json _j_res = { {"response", {}}, {"values", {}}, {"conflicts", {}}};
  const Json _j_traj =
  { {"id", {}}, {"shape", {}}, {"dimensions", {}}, {"segments", {}}};
  const Json _j_seg = { {"x", {}}, {"v", {}}, {"t", {}}};
  const Json _j_err = { {"error", {}}};

  void on_open(connection_hdl hdl);

  void on_close(connection_hdl hdl);

  void on_message(connection_hdl hdl, Server::message_ptr msg);

  bool parse_request(connection_hdl hdl, const Server::message_ptr msg,
    std::string& response);

  void send_error_message(
    connection_hdl hdl,
    const Server::message_ptr msg,
    std::string& response,
    std::shared_ptr<Server> server,
    std::string err_excp);

  const std::string parse_trajectories(
    const std::string& response_type,
    const std::vector<std::vector<uint64_t>>& conflicts,
    const std::vector<Element>& elements,
    const bool trim,
    const RequestParam& request_param) const;

};

//==============================================================================
auto TrajectoryServer::Implementation::on_open(connection_hdl hdl) -> void
{
  connections.insert(hdl);
  RCLCPP_INFO(schedule_data_node->get_logger(),
    "[TrajectoryServer] Connected with a client");
}

//==============================================================================
auto TrajectoryServer::Implementation::on_close(connection_hdl hdl) -> void
{
  connections.erase(hdl);
  negotiation_subscribed_connections.erase(hdl);
  RCLCPP_INFO(schedule_data_node->get_logger(),
    "[TrajectoryServer] Disconnected with a client");

}

//==============================================================================
auto TrajectoryServer::Implementation::on_message(
  connection_hdl hdl, Server::message_ptr msg) -> void
{
  std::string response = "";
  std::string err_response = "";

  if (msg->get_payload().empty())
  {
    RCLCPP_DEBUG(
      schedule_data_node->get_logger(),
      "[TrajectoryServer] Empty request received. Ignoring...");
    return;
  }

  bool ok = parse_request(hdl, msg, response);

  // validate jwt only if public key is given (when running with dashboard)
  std::string public_key;
  std::string token;
  bool is_verified = true;
  if (std::getenv("JWT_PUBLIC_KEY"))
  {
    public_key = std::getenv("JWT_PUBLIC_KEY");
    try
    {
      token = Json::parse(msg->get_payload())["token"];
      auto decoded = jwt::decode(
        token,
        jwt::params::algorithms({"RS256"}),
        jwt::params::secret(public_key));
    }
    catch (std::exception& e)
    {
      is_verified = false;
      std::string err_excp = e.what();
      send_error_message(hdl, msg, err_response, server, err_excp);
      std::cerr << "Error: " << e.what() << std::endl;
    }
  }

  if (ok && is_verified)
  {
    RCLCPP_DEBUG(schedule_data_node->get_logger(),
      "Response: %s", response.c_str());
    Server::message_ptr response_msg = std::move(msg);
    response_msg->set_payload(response);
    server->send(hdl, response_msg);
  }
  else
  {
    RCLCPP_DEBUG(schedule_data_node->get_logger(),
      "[TrajectoryServer] Invalid request received: %s",
      msg->get_payload().c_str());
  }

}

//==============================================================================
auto TrajectoryServer::Implementation::parse_request(
  connection_hdl hdl, Server::message_ptr msg,
  std::string& response) -> bool
{
  using namespace std::chrono_literals;

  // TODO(YV) get names of the keys from config yaml file
  std::string msg_payload = msg->get_payload();

  try
  {
    Json j = Json::parse(msg_payload);

    if (j.count("request") != 1)
      return false;

    if (j["request"] == "trajectory")
    {
      Json j_param = j["param"];

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
      request_param.start_time = schedule_data_node->now();
      request_param.finish_time = request_param.start_time +
        duration;

      RCLCPP_DEBUG(schedule_data_node->get_logger(),
        "Trajectory Request received with map_name [%s] and duration [%s]ms",
        request_param.map_name.c_str(), std::to_string(duration_num).c_str());

      std::lock_guard<std::mutex> lock(schedule_data_node->get_mutex());
      auto elements = schedule_data_node->get_elements(request_param);

      bool trim = j_param["trim"];
      response = parse_trajectories("trajectory",
          schedule_data_node->get_conflict_groups(), elements, trim,
          request_param);

      return true;

    }

    else if (j["request"] == "time")
    {
      RCLCPP_DEBUG(schedule_data_node->get_logger(),
        "Time request received");
      Json j_res = _j_res;
      j_res["response"] = "time";
      j_res["values"].push_back(
        schedule_data_node->now().time_since_epoch().count());
      response = j_res.dump();
      return true;
    }

    else if (j["request"] == "negotiation_update_subscribe")
    {
      negotiation_subscribed_connections.insert(hdl);
      Json j_res = Json();
      j_res["response"] = "negotiation_update_subscribe";
      j_res["result"] = true;
      response = j_res.dump();
      return true;
    }

    else if (j["request"] == "negotiation_trajectory")
    {
      RCLCPP_DEBUG(schedule_data_node->get_logger(),
        "Received Negotiation Trajectory request");

      uint64_t conflict_version = j["param"]["conflict_version"];
      std::vector<uint64_t> sequence = j["param"]["sequence"];

      auto trajectory_elements =
        schedule_data_node->get_negotiation_trajectories(conflict_version,
          sequence);
      const auto now = schedule_data_node->now();

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
    RCLCPP_ERROR(schedule_data_node->get_logger(),
      "Error: %s", std::to_string(*e.what()).c_str());
    return false;
  }

}

//==============================================================================
const std::string TrajectoryServer::Implementation::parse_trajectories(
  const std::string& response_type,
  const std::vector<std::vector<uint64_t>>& conflicts,
  const std::vector<Element>& elements,
  const bool trim,
  const RequestParam& request_param) const
{
  std::string response;
  auto j_res = _j_res;
  j_res["response"] = response_type;
  j_res["conflicts"] = conflicts;

  try
  {
    for (const auto& element : elements)
    {
      const auto& trajectory = element.route->trajectory();

      auto j_traj = _j_traj;
      j_traj["map_name"] = element.route->map();
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
    RCLCPP_ERROR(schedule_data_node->get_logger(),
      "Error: %s", std::to_string(*e.what()).c_str());
    return "";
  }

  response = j_res.dump();
  return response;
}

//==============================================================================
auto TrajectoryServer::Implementation::send_error_message(
  connection_hdl hdl, Server::message_ptr msg,
  std::string& response, std::shared_ptr<Server> server,
  std::string err_excp) -> void
{
  auto j_err = _j_err;
  j_err["error"] = err_excp;
  Server::message_ptr err_msg = std::move(msg);
  response = j_err.dump();
  err_msg->set_payload(response);
  server->send(hdl, err_msg);
}

//==============================================================================
std::shared_ptr<TrajectoryServer> TrajectoryServer::make(
  uint16_t port,
  ScheduleDataNodePtr schedule_data_node)
{

  std::shared_ptr<TrajectoryServer> server_ptr(
    new TrajectoryServer());
  server_ptr->_pimpl->schedule_data_node = schedule_data_node;
  server_ptr->_pimpl->server = std::make_shared<
    websocketpp::server<websocketpp::config::asio>>();
  // Bind callbacks
  using websocketpp::lib::placeholders::_1;
  using websocketpp::lib::placeholders::_2;
  using websocketpp::lib::bind;
  server_ptr->_pimpl->server->init_asio();
  server_ptr->_pimpl->server->set_open_handler(bind(
      &TrajectoryServer::Implementation::on_open, std::ref(
        server_ptr->_pimpl), _1));
  server_ptr->_pimpl->server->set_close_handler(bind(
      &TrajectoryServer::Implementation::on_close, std::ref(
        server_ptr->_pimpl), _1));
  server_ptr->_pimpl->server->set_message_handler(bind(
      &TrajectoryServer::Implementation::on_message, std::ref(
        server_ptr->_pimpl), _1, _2));

  try
  {
    // Start the websocket server
    server_ptr->_pimpl->server->set_reuse_addr(true);
    server_ptr->_pimpl->server->listen(port);
    server_ptr->_pimpl->server->start_accept();
    server_ptr->_pimpl->server_thread = std::thread(
      [server_ptr]() {server_ptr->_pimpl->server->run();});
  }
  catch (std::exception& e)
  {
    std::cerr << "Failed to start the TrajectoryServer: " << e.what()
              << std::endl;
    return nullptr;
  }

  // Set up callbacks for negotiations
  auto status_update_cb = [server_ptr](
    uint64_t conflict_version,
    rmf_traffic::schedule::Negotiation::Table::ViewerPtr table_view)
    {
      RCLCPP_DEBUG(server_ptr->_pimpl->schedule_data_node->get_logger(),
        "======== conflict callback version: %lu! ==========",
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
      for (auto connection :
        server_ptr->_pimpl->negotiation_subscribed_connections)
        server_ptr->_pimpl->server->send(
          connection, conflict_str, websocketpp::frame::opcode::text);
    };

  server_ptr->_pimpl->schedule_data_node->get_negotiation()->on_status_update(
    std::move(status_update_cb));

  auto conclusion_cb = [server_ptr](
    uint64_t conflict_version, bool resolved)
    {
      RCLCPP_DEBUG(
        server_ptr->_pimpl->schedule_data_node->get_logger(),
        "======== conflict concluded: %lu resolved: %d ==========",
        conflict_version,
        resolved ? 1 : 0);

      nlohmann::json json_msg;
      json_msg["type"] = "negotiation_conclusion";
      json_msg["conflict_version"] = conflict_version;
      json_msg["resolved"] = resolved;

      std::string json_str = json_msg.dump();
      for (auto connection :
        server_ptr->_pimpl->negotiation_subscribed_connections)
        server_ptr->_pimpl->server->send(
          connection, json_str, websocketpp::frame::opcode::text);
    };

  server_ptr->_pimpl->schedule_data_node->get_negotiation()->on_conclusion(
    std::move(conclusion_cb));

  server_ptr->_pimpl->initialized = true;
  return server_ptr;
}

//==============================================================================
TrajectoryServer::TrajectoryServer()
: _pimpl(rmf_utils::make_impl<Implementation>())
{
  // Do nothing
}

//==============================================================================
TrajectoryServer::~TrajectoryServer()
{
  // Thread safe access to _connections
  const auto connection_copies = _pimpl->connections;
  for (auto& connection : connection_copies)
  {
    _pimpl->server->close(
      connection, websocketpp::close::status::normal, "shutdown");
  }

  if (_pimpl->server_thread.joinable())
  {
    _pimpl->server->stop();
    _pimpl->server_thread.join();
  }
}

} //namespace rmf_visualization_schedule

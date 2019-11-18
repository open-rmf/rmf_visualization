#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <rclcpp/rclcpp.hpp>
#include "json.hpp"
#include <iostream>
#include <set>

#include "VisualizerData.hpp"

namespace rmf_schedule_visualizer {

using server = websocketpp::server<websocketpp::config::asio>;

using websocketpp::connection_hdl;
using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;

class Server
{
public:
  /// Builder function which returns a pointer to Server when
  /// the websocket has started.
  /// A nullptr is returned if initialization fails. 
  static std::shared_ptr<Server> make(
      uint16_t port,
      VisualizerDataNode& visualizer_data_node);
    
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

  server _server;
  con_list _connections;
  uint16_t _port;
  std::thread _server_thread;
  VisualizerDataNode* _visualizer_data_node = nullptr;
  bool _is_initialized = false;
  std::string _data;
};

} // namespace rmf_schedule_visualizer

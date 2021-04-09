/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#ifndef RMF_VISUALIZATION_RVIZ2_PLUGINS__SRC__DOORPANEL_HPP
#define RMF_VISUALIZATION_RVIZ2_PLUGINS__SRC__DOORPANEL_HPP

#include <mutex>
#include <thread>
#include <unordered_map>

#include <QLabel>
#include <QComboBox>
#include <QGroupBox>
#include <QPushButton>
#include <QRadioButton>

#include <rclcpp/rclcpp.hpp>
#include <rmf_door_msgs/msg/door_mode.hpp>
#include <rmf_door_msgs/msg/door_state.hpp>
#include <rmf_door_msgs/msg/door_request.hpp>

#include <rviz_common/panel.hpp>

namespace rmf_visualization_rviz2_plugins {

class DoorPanel : public rviz_common::Panel
{

  Q_OBJECT

public:

  using DoorMode = rmf_door_msgs::msg::DoorMode;
  using DoorState = rmf_door_msgs::msg::DoorState;
  using DoorRequest = rmf_door_msgs::msg::DoorRequest;

  DoorPanel(QWidget* parent = 0);
  ~DoorPanel();

  virtual void load(const rviz_common::Config& config);
  virtual void save(rviz_common::Config config) const;

protected Q_SLOTS:

  void send_door_request();
  void update_door_name_selector();
  void update_state_visualizer();

private:

  QGroupBox* create_door_selection_group_box();
  QGroupBox* create_state_group_box();
  QGroupBox* create_request_group_box();
  QGroupBox* create_debug_group_box();
  void create_layout();
  void create_connections();

  QComboBox* _door_name_selector;

  std::string _requester_id;

  QRadioButton* _door_open_radio_button;
  QRadioButton* _door_closed_radio_button;

  QRadioButton* _supervisor_radio_button;
  QRadioButton* _manual_radio_button;

  QPushButton* _send_door_request_button;

  std::vector<QLabel*> _state_labels;

  QLabel* _debug_label;

  rclcpp::Node::SharedPtr _node;
  rclcpp::Subscription<DoorState>::SharedPtr _door_state_sub;
  rclcpp::Publisher<DoorRequest>::SharedPtr _door_request_pub;
  rclcpp::Publisher<DoorRequest>::SharedPtr _adapter_door_request_pub;

  std::thread _thread;
  std::mutex _mutex;

  std::unordered_map<std::string, DoorState> _door_states;

  void door_state_callback(DoorState::UniquePtr msg);

  void display_state(const DoorState& msg);

  std::string door_state_mode_string(uint8_t mode) const;

  QString door_state_mode_tooltip() const;

};

} // namespace rmf_visualization_rviz2_plugins

#endif // RMF_VISUALIZATION_RVIZ2_PLUGINS__SRC__DOORPANEL_HPP

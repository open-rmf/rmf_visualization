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

#ifndef RMF_VISUALIZATION_RVIZ2_PLUGINS__SRC__LIFTPANEL_HPP
#define RMF_VISUALIZATION_RVIZ2_PLUGINS__SRC__LIFTPANEL_HPP

#include <mutex>
#include <thread>
#include <unordered_map>

#include <QLabel>
#include <QLineEdit>
#include <QComboBox>
#include <QGroupBox>
#include <QPushButton>
#include <QRadioButton>

#include <rclcpp/rclcpp.hpp>
#include <rmf_lift_msgs/msg/lift_state.hpp>
#include <rmf_lift_msgs/msg/lift_request.hpp>

#include <rviz_common/panel.hpp>

namespace rmf_visualization_rviz2_plugins {

class LiftPanel : public rviz_common::Panel
{

  Q_OBJECT

public:

  using LiftState = rmf_lift_msgs::msg::LiftState;
  using LiftRequest = rmf_lift_msgs::msg::LiftRequest;

  LiftPanel(QWidget* parent = 0);
  ~LiftPanel();

  virtual void load(const rviz_common::Config& config);
  virtual void save(rviz_common::Config config) const;

protected Q_SLOTS:

  void send_lift_request();
  void update_lift_name_selector();
  void update_state_visualizer();

private:

  QGroupBox* create_lift_selector_group_box();
  QGroupBox* create_state_group_box();
  QGroupBox* create_request_group_box();
  QGroupBox* create_debug_group_box();
  void create_layout();
  void create_connections();

  QComboBox* _lift_name_selector;

  QLineEdit* _destination_floor_line_edit;

  QRadioButton* _end_session_radio_button;
  QRadioButton* _agv_mode_radio_button;
  QRadioButton* _human_mode_radio_button;

  QRadioButton* _door_closed_radio_button;
  QRadioButton* _door_open_radio_button;

  QRadioButton* _supervisor_radio_button;
  QRadioButton* _manual_radio_button;

  QPushButton* _send_lift_request_button;

  QComboBox* _state_lift_name_selector;
  std::vector<QLabel*> _state_labels;

  QLabel* _debug_label;

  rclcpp::Node::SharedPtr _node;
  rclcpp::Subscription<LiftState>::SharedPtr _lift_state_sub;
  rclcpp::Publisher<LiftRequest>::SharedPtr _lift_request_pub;
  rclcpp::Publisher<LiftRequest>::SharedPtr _adapter_lift_request_pub;

  std::thread _thread;
  std::mutex _mutex;

  std::string _session_id;

  std::unordered_map<std::string, LiftState> _lift_states;

  void lift_state_callback(LiftState::UniquePtr msg);

  void display_state(const LiftState& msg);

  std::string lift_door_state_string(uint8_t state) const;
  std::string lift_motion_state_string(uint8_t state) const;
  std::string lift_mode_string(uint8_t mode) const;

  QString door_state_tooltip() const;
  QString motion_state_tooltip() const;
  QString lift_mode_tooltip() const;

};

} // namespace rmf_visualization_rviz2_plugins

#endif // RMF_VISUALIZATION_RVIZ2_PLUGINS__SRC__LIFTPANEL_HPP

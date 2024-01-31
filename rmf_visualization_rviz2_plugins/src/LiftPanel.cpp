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

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>

#include "LiftPanel.hpp"
#include "StandardNames.hpp"

namespace rmf_visualization_rviz2_plugins {

//==============================================================================

LiftPanel::LiftPanel(QWidget* parent)
: rviz_common::Panel(parent),
  _session_id(LiftPanelSessionId)
{
  _node = std::make_shared<rclcpp::Node>(_session_id + "_node");
  _lift_state_sub = _node->create_subscription<LiftState>(
    LiftStateTopicName, 10, [&](LiftState::UniquePtr msg)
    {
      lift_state_callback(std::move(msg));
    });
  _lift_request_pub = _node->create_publisher<LiftRequest>(
    LiftRequestTopicName, rclcpp::QoS(10));
  _adapter_lift_request_pub = _node->create_publisher<LiftRequest>(
    AdapterLiftRequestTopicName, rclcpp::QoS(10));

  create_layout();
  create_connections();

  _debug_label->setText("Lift panel running...");

  _thread = std::thread([&]()
      {
        rclcpp::spin(_node);
      });
}

//==============================================================================

LiftPanel::~LiftPanel()
{
  if (_thread.joinable())
  {
    _thread.join();
    rclcpp::shutdown();
  }
}

//==============================================================================

void LiftPanel::load(const rviz_common::Config& config)
{
  rviz_common::Panel::load(config);
}

//==============================================================================

void LiftPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

//==============================================================================

void LiftPanel::send_lift_request()
{
  LiftRequest msg;
  msg.lift_name = _lift_name_selector->currentText().toStdString();
  msg.request_time = _node->get_clock()->now();
  msg.session_id = _session_id;

  if (_end_session_radio_button->isChecked())
    msg.request_type = LiftRequest::REQUEST_END_SESSION;
  else if (_agv_mode_radio_button->isChecked())
    msg.request_type = LiftRequest::REQUEST_AGV_MODE;
  else if (_human_mode_radio_button->isChecked())
    msg.request_type = LiftRequest::REQUEST_HUMAN_MODE;

  msg.destination_floor = _destination_floor_line_edit->text().toStdString();

  if (_door_closed_radio_button->isChecked())
    msg.door_state = LiftRequest::DOOR_CLOSED;
  else if (_door_open_radio_button->isChecked())
    msg.door_state = LiftRequest::DOOR_OPEN;

  if (_supervisor_radio_button->isChecked())
  {
    _adapter_lift_request_pub->publish(msg);
    _debug_label->setText("Sent request to lift supervisor...");
  }
  else
  {
    _lift_request_pub->publish(msg);
    _debug_label->setText("Sent request to lift manually...");
  }
}

//==============================================================================

void LiftPanel::update_lift_name_selector()
{
  _lift_name_selector->blockSignals(true);
  _lift_name_selector->clear();

  std::unique_lock<std::mutex> lift_states_lock(_mutex);
  for (auto it : _lift_states)
  {
    _lift_name_selector->addItem(QString(it.first.c_str()));
  }

  _lift_name_selector->blockSignals(false);
}

//==============================================================================

void LiftPanel::update_state_visualizer()
{
  std::unique_lock<std::mutex> lift_states_lock(_mutex);
  const auto it = _lift_states.find(
    _lift_name_selector->currentText().toStdString());
  if (it == _lift_states.end())
    return;

  display_state(it->second);
}

//==============================================================================

QGroupBox* LiftPanel::create_lift_selector_group_box()
{
  QLabel* lift_name_label = new QLabel("Name:");
  lift_name_label->setStyleSheet("font: italic;");

  _lift_name_selector = new QComboBox;

  QHBoxLayout* layout = new QHBoxLayout;
  layout->addWidget(lift_name_label);
  layout->addWidget(_lift_name_selector);

  QGroupBox* lift_selector_group_box = new QGroupBox("Lift Selection");
  lift_selector_group_box->setLayout(layout);
  return lift_selector_group_box;
}

//==============================================================================

QGroupBox* LiftPanel::create_state_group_box()
{
  QLabel* door_state_label = new QLabel("door state <?>");
  door_state_label->setToolTip(door_state_tooltip());

  QLabel* motion_state_label = new QLabel("motion state <?>");
  motion_state_label->setToolTip(motion_state_tooltip());

  QLabel* available_modes_label = new QLabel("available modes <?>");
  available_modes_label->setToolTip(lift_mode_tooltip());

  std::vector<QLabel*> state_field_labels = {
    new QLabel("time (sec)"),
    new QLabel("name"),
    new QLabel("floors"),
    new QLabel("current floor"),
    new QLabel("destination floor"),
    door_state_label,
    motion_state_label,
    available_modes_label,
    new QLabel("current mode"),
    new QLabel("session ID")
  };

  QGridLayout* layout = new QGridLayout;
  _state_labels.clear();
  for (size_t i = 0; i < state_field_labels.size(); ++i)
  {
    layout->addWidget(state_field_labels[i], i, 0, 1, 1);
    state_field_labels[i]->setStyleSheet(
      "border-width: 1px;"
      "border-style: solid;"
      "border-color: transparent darkgray darkgray transparent;"
      "font: italic;");

    QLabel* cell_label = new QLabel("");
    cell_label->setStyleSheet(
      "border-width: 1px;"
      "border-style: solid;"
      "border-color: transparent transparent darkgray transparent;");
    _state_labels.push_back(cell_label);
    layout->addWidget(cell_label, i, 1, 1, 4);
  }

  QGroupBox* state_group_box = new QGroupBox("State");
  state_group_box->setLayout(layout);
  return state_group_box;
}
//==============================================================================

QGroupBox* LiftPanel::create_request_group_box()
{
  _destination_floor_line_edit = new QLineEdit;

  _end_session_radio_button = new QRadioButton("&End Session");
  _end_session_radio_button->setChecked(true);
  _agv_mode_radio_button = new QRadioButton("&AGV Mode");
  _human_mode_radio_button = new QRadioButton("&Human Mode");

  QVBoxLayout* request_type_v_box = new QVBoxLayout;
  request_type_v_box->addWidget(_end_session_radio_button);
  request_type_v_box->addWidget(_agv_mode_radio_button);
  request_type_v_box->addWidget(_human_mode_radio_button);

  QGroupBox* request_type_group_box = new QGroupBox("Request Type");
  request_type_group_box->setLayout(request_type_v_box);

  _door_open_radio_button = new QRadioButton("&Open");
  _door_open_radio_button->setChecked(true);
  _door_closed_radio_button = new QRadioButton("&Close");

  QVBoxLayout* door_request_v_box = new QVBoxLayout;
  door_request_v_box->addWidget(_door_open_radio_button);
  door_request_v_box->addWidget(_door_closed_radio_button);

  QGroupBox* door_request_group_box = new QGroupBox("Lift Door Request");
  door_request_group_box->setLayout(door_request_v_box);

  _supervisor_radio_button = new QRadioButton("&Supervisor (Recommended)");
  _supervisor_radio_button->setChecked(true);
  _manual_radio_button = new QRadioButton("&Manual");

  QHBoxLayout* route_h_box = new QHBoxLayout;
  route_h_box->addWidget(_supervisor_radio_button);
  route_h_box->addWidget(_manual_radio_button);

  QGroupBox* route_group_box = new QGroupBox("Route");
  route_group_box->setLayout(route_h_box);

  _send_lift_request_button = new QPushButton("Send Request");

  QLabel* session_id_label = new QLabel("Session ID:");
  session_id_label->setStyleSheet("font: italic;");

  QLabel* destination_floor_label = new QLabel("Destination Floor:");
  destination_floor_label->setStyleSheet("font: italic;");

  QGridLayout* layout = new QGridLayout;
  layout->addWidget(session_id_label, 0, 0, 1, 1);
  layout->addWidget(
    new QLabel(QString::fromStdString(_session_id)), 0, 1, 1, 3);
  layout->addWidget(destination_floor_label, 1, 0, 1, 1);
  layout->addWidget(_destination_floor_line_edit, 1, 1, 1, 3);
  layout->addWidget(request_type_group_box, 2, 0, 3, 2);
  layout->addWidget(door_request_group_box, 2, 2, 3, 2);
  layout->addWidget(route_group_box, 5, 0, 1, 4);
  layout->addWidget(_send_lift_request_button, 6, 0, 1, 4);

  QGroupBox* request_group_box = new QGroupBox("Request");
  request_group_box->setLayout(layout);
  return request_group_box;
}

//==============================================================================

QGroupBox* LiftPanel::create_debug_group_box()
{
  _debug_label = new QLabel("Lift panel starting...");

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(_debug_label);

  QGroupBox* debug_group_box = new QGroupBox("Debug");
  debug_group_box->setLayout(layout);
  return debug_group_box;
}

//==============================================================================

void LiftPanel::create_layout()
{
  QGroupBox* lift_selector_gb = create_lift_selector_group_box();
  QGroupBox* state_gb = create_state_group_box();
  QGroupBox* request_gb = create_request_group_box();
  QGroupBox* debug_gb = create_debug_group_box();

  QGridLayout* layout = new QGridLayout;
  layout->addWidget(lift_selector_gb, 0, 0, 1, 1);
  layout->addWidget(state_gb, 1, 0, 8, 1);
  layout->addWidget(request_gb, 9, 0, 4, 1);
  layout->addWidget(debug_gb, 13, 0, 1, 1);
  setLayout(layout);
  setStyleSheet(
    "QGroupBox {"
    "  font: bold;"
    "  border: 1px solid silver;"
    "  border-radius: 6px;"
    "  margin-top: 6px;"
    "  padding-top: 10px;"
    "}"
    "QGroupBox::title {"
    "  subcontrol-origin: margin;"
    "  left: 7px;"
    "  padding: 0px 5px 0px 5px;"
    "}"
    "QRadioButton {"
    "  font: italic;"
    "}"
  );
}

//==============================================================================

void LiftPanel::create_connections()
{
  connect(this, SIGNAL(configChanged()), this,
    SLOT(update_lift_name_selector()));
  connect(_lift_name_selector,
    SIGNAL(currentTextChanged(const QString&)), this,
    SLOT(update_state_visualizer()));
  connect(_send_lift_request_button,
    SIGNAL(clicked()), this, SLOT(send_lift_request()));
}

//==============================================================================

void LiftPanel::lift_state_callback(LiftState::UniquePtr msg)
{
  std::unique_lock<std::mutex> lift_states_lock(_mutex);
  const std::string incoming_lift_name = msg->lift_name;

  LiftState new_msg = *(msg.get());
  std::string selected_lift_name =
    _lift_name_selector->currentText().toStdString();
  if (incoming_lift_name == selected_lift_name)
    display_state(new_msg);

  bool new_lift_found =
    _lift_states.find(incoming_lift_name) == _lift_states.end();
  _lift_states[incoming_lift_name] = new_msg;

  lift_states_lock.unlock();

  if (new_lift_found)
  {
    std::string debug_str =
      "New lift [" + incoming_lift_name  + "] found, refreshing...";
    RCLCPP_INFO(_node->get_logger(), debug_str.c_str());
    _debug_label->setText(QString::fromStdString(debug_str));
    Q_EMIT configChanged();
  }
  else
  {
    _debug_label->setText("Lift panel running...");
  }
}

//==============================================================================

void LiftPanel::display_state(const LiftState& msg)
{
  std::string floors_str = "";
  for (const auto f : msg.available_floors)
    floors_str += f + ", ";

  std::string available_modes_str = "";
  for (const auto m : msg.available_modes)
    available_modes_str += std::to_string(m) + ", ";

  _state_labels[0]->setText(
    QString::fromStdString(std::to_string(msg.lift_time.sec)));
  _state_labels[1]->setText(
    QString::fromStdString(msg.lift_name));
  _state_labels[2]->setText(
    QString::fromStdString(floors_str));
  _state_labels[3]->setText(
    QString::fromStdString(msg.current_floor));
  _state_labels[4]->setText(
    QString::fromStdString(msg.destination_floor));
  _state_labels[5]->setText(
    QString::fromStdString(lift_door_state_string(msg.door_state)));
  _state_labels[6]->setText(
    QString::fromStdString(lift_motion_state_string(msg.motion_state)));
  _state_labels[7]->setText(
    QString::fromStdString(available_modes_str));
  _state_labels[8]->setText(
    QString::fromStdString(lift_mode_string(msg.current_mode)));
  _state_labels[9]->setText(
    QString::fromStdString(msg.session_id));
}

//==============================================================================

std::string LiftPanel::lift_door_state_string(uint8_t state) const
{
  switch (state)
  {
    case LiftState::DOOR_CLOSED:
      return std::string("Closed");
    case LiftState::DOOR_MOVING:
      return std::string("Moving");
    case LiftState::DOOR_OPEN:
      return std::string("Open");
    default:
      return std::string("Undefined");
  }
}

//==============================================================================

std::string LiftPanel::lift_motion_state_string(uint8_t state) const
{
  switch (state)
  {
    case LiftState::MOTION_STOPPED:
      return std::string("Stopped");
    case LiftState::MOTION_UP:
      return std::string("Up");
    case LiftState::MOTION_DOWN:
      return std::string("Down");
    case LiftState::MOTION_UNKNOWN:
      return std::string("Unknown");
    default:
      return std::string("Undefined");
  }
}

//==============================================================================

std::string LiftPanel::lift_mode_string(uint8_t mode) const
{
  switch (mode)
  {
    case LiftState::MODE_UNKNOWN:
      return std::string("Unknown");
    case LiftState::MODE_HUMAN:
      return std::string("Human");
    case LiftState::MODE_AGV:
      return std::string("AGV");
    case LiftState::MODE_FIRE:
      return std::string("Fire");
    case LiftState::MODE_OFFLINE:
      return std::string("Offline");
    case LiftState::MODE_EMERGENCY:
      return std::string("Emergency");
    default:
      return std::string("Undefined");
  }
}

//==============================================================================

QString LiftPanel::door_state_tooltip() const
{
  std::stringstream ss;
  ss << std::to_string(LiftState::DOOR_CLOSED) << " - "
     << lift_door_state_string(LiftState::DOOR_CLOSED) << std::endl
     << std::to_string(LiftState::DOOR_MOVING) << " - "
     << lift_door_state_string(LiftState::DOOR_MOVING) << std::endl
     << std::to_string(LiftState::DOOR_OPEN) << " - "
     << lift_door_state_string(LiftState::DOOR_OPEN) << std::endl;
  return QString::fromStdString(ss.str());
}

//==============================================================================

QString LiftPanel::motion_state_tooltip() const
{
  std::stringstream ss;
  ss << std::to_string(LiftState::MOTION_STOPPED) << " - "
     << lift_motion_state_string(LiftState::MOTION_STOPPED) << std::endl
     << std::to_string(LiftState::MOTION_UP) << " - "
     << lift_motion_state_string(LiftState::MOTION_UP) << std::endl
     << std::to_string(LiftState::MOTION_DOWN) << " - "
     << lift_motion_state_string(LiftState::MOTION_DOWN) << std::endl
     << std::to_string(LiftState::MOTION_UNKNOWN) << " - "
     << lift_motion_state_string(LiftState::MOTION_UNKNOWN) << std::endl;
  return QString::fromStdString(ss.str());
}

//==============================================================================

QString LiftPanel::lift_mode_tooltip() const
{
  std::stringstream ss;
  ss << std::to_string(LiftState::MODE_UNKNOWN) << " - "
     << lift_mode_string(LiftState::MODE_UNKNOWN) << std::endl
     << std::to_string(LiftState::MODE_HUMAN) << " - "
     << lift_mode_string(LiftState::MODE_HUMAN) << std::endl
     << std::to_string(LiftState::MODE_AGV) << " - "
     << lift_mode_string(LiftState::MODE_AGV) << std::endl
     << std::to_string(LiftState::MODE_FIRE) << " - "
     << lift_mode_string(LiftState::MODE_FIRE) << std::endl
     << std::to_string(LiftState::MODE_OFFLINE) << " - "
     << lift_mode_string(LiftState::MODE_OFFLINE) << std::endl
     << std::to_string(LiftState::MODE_EMERGENCY) << " - "
     << lift_mode_string(LiftState::MODE_EMERGENCY) << std::endl;
  return QString::fromStdString(ss.str());
}

//==============================================================================

} // namespace rmf_visualization_rviz2_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  rmf_visualization_rviz2_plugins::LiftPanel,
  rviz_common::Panel)

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

#include "DoorPanel.hpp"
#include "StandardNames.hpp"

namespace rmf_visualization_rviz2_plugins {

//==============================================================================

DoorPanel::DoorPanel(QWidget* parent)
: rviz_common::Panel(parent),
  _requester_id(DoorPanelRequesterId)
{
  _node = std::make_shared<rclcpp::Node>(_requester_id + "_node");
  _door_state_sub = _node->create_subscription<DoorState>(
    DoorStateTopicName, 10, [&](DoorState::UniquePtr msg)
    {
      door_state_callback(std::move(msg));
    });
  _door_request_pub = _node->create_publisher<DoorRequest>(
    DoorRequestTopicName, rclcpp::QoS(10));
  _adapter_door_request_pub = _node->create_publisher<DoorRequest>(
    AdapterDoorRequestTopicName, rclcpp::QoS(10));

  create_layout();
  create_connections();

  _debug_label->setText("Door panel running...");

  _thread = std::thread([&]()
      {
        rclcpp::spin(_node);
      });
}

//==============================================================================

DoorPanel::~DoorPanel()
{
  if (_thread.joinable())
  {
    _thread.join();
    rclcpp::shutdown();
  }
}

//==============================================================================

void DoorPanel::load(const rviz_common::Config& config)
{
  rviz_common::Panel::load(config);
}

//==============================================================================

void DoorPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

//==============================================================================

void DoorPanel::send_door_request()
{
  DoorRequest msg;
  msg.request_time = _node->get_clock()->now();
  msg.requester_id = _requester_id;
  msg.door_name = _door_name_selector->currentText().toStdString();

  if (_door_open_radio_button->isChecked())
    msg.requested_mode.value = DoorMode::MODE_OPEN;
  else if (_door_closed_radio_button->isChecked())
    msg.requested_mode.value = DoorMode::MODE_CLOSED;

  if (_supervisor_radio_button->isChecked())
  {
    _adapter_door_request_pub->publish(msg);
    _debug_label->setText("Sent request to door supervisor...");
  }
  else
  {
    _door_request_pub->publish(msg);
    _debug_label->setText("Sent request to door manually...");
  }

}

//==============================================================================

void DoorPanel::update_door_name_selector()
{
  _door_name_selector->blockSignals(true);
  _door_name_selector->clear();

  std::unique_lock<std::mutex> door_states_lock(_mutex);
  for (auto it : _door_states)
  {
    _door_name_selector->addItem(QString(it.first.c_str()));
  }

  _door_name_selector->blockSignals(false);
}

//==============================================================================

void DoorPanel::update_state_visualizer()
{
  std::unique_lock<std::mutex> door_states_lock(_mutex);
  const auto it = _door_states.find(
    _door_name_selector->currentText().toStdString());
  if (it == _door_states.end())
    return;

  display_state(it->second);
}

//==============================================================================

QGroupBox* DoorPanel::create_door_selection_group_box()
{
  QLabel* door_name_label = new QLabel("Name:");
  door_name_label->setStyleSheet("font: italic;");

  _door_name_selector = new QComboBox;

  QHBoxLayout* layout = new QHBoxLayout;
  layout->addWidget(door_name_label);
  layout->addWidget(_door_name_selector);

  QGroupBox* groupbox = new QGroupBox("Door Selection");
  groupbox->setLayout(layout);
  return groupbox;
}

//==============================================================================

QGroupBox* DoorPanel::create_state_group_box()
{
  QLabel* door_state_mode_label = new QLabel("mode <?>");
  door_state_mode_label->setToolTip(door_state_mode_tooltip());

  std::vector<QLabel*> state_field_labels = {
    new QLabel("sec"),
    new QLabel("name"),
    door_state_mode_label
  };

  QGridLayout* layout = new QGridLayout;
  _state_labels.clear();
  for (size_t i = 0; i < state_field_labels.size(); ++i)
  {
    layout->addWidget(state_field_labels[i], i, 0, 1, 1);
    state_field_labels[i]->setStyleSheet(
      "border-width: 1px; "
      "border-style: solid; "
      "border-color: transparent darkgray darkgray transparent; "
      "font: italic; ");

    QLabel* cell_label = new QLabel("");
    cell_label->setStyleSheet(
      "border-width: 1px; "
      "border-style: solid; "
      "border-color: transparent transparent darkgray transparent; ");
    _state_labels.push_back(cell_label);
    layout->addWidget(cell_label, i, 1, 1, 4);
  }

  QGroupBox* groupbox = new QGroupBox("State");
  groupbox->setLayout(layout);
  return groupbox;
}

//==============================================================================

QGroupBox* DoorPanel::create_request_group_box()
{
  _door_open_radio_button = new QRadioButton("&Open");
  _door_open_radio_button->setChecked(true);
  _door_closed_radio_button = new QRadioButton("&Close");

  QHBoxLayout* door_request_type_h_box = new QHBoxLayout;
  door_request_type_h_box->addWidget(_door_open_radio_button);
  door_request_type_h_box->addWidget(_door_closed_radio_button);

  QGroupBox* door_request_type_group_box = new QGroupBox("Request Type");
  door_request_type_group_box->setLayout(door_request_type_h_box);

  _supervisor_radio_button = new QRadioButton("&Supervisor (Recommended)");
  _supervisor_radio_button->setChecked(true);
  _manual_radio_button = new QRadioButton("&Manual");

  QHBoxLayout* route_h_box = new QHBoxLayout;
  route_h_box->addWidget(_supervisor_radio_button);
  route_h_box->addWidget(_manual_radio_button);

  QGroupBox* route_group_box = new QGroupBox("Route");
  route_group_box->setLayout(route_h_box);

  _send_door_request_button = new QPushButton("Send Request");

  QLabel* requester_id_label = new QLabel("Requester ID:");
  requester_id_label->setStyleSheet("font: italic;");

  QGridLayout* layout = new QGridLayout;
  layout->addWidget(requester_id_label, 0, 0, 1, 1);
  layout->addWidget(
    new QLabel(QString::fromStdString(_requester_id)), 0, 1, 1, 3);
  layout->addWidget(door_request_type_group_box, 1, 0, 1, 4);
  layout->addWidget(route_group_box, 2, 0, 1, 4);
  layout->addWidget(_send_door_request_button, 3, 0, 1, 4);

  QGroupBox* groupbox = new QGroupBox("Door Request");
  groupbox->setLayout(layout);
  return groupbox;
}

//==============================================================================

QGroupBox* DoorPanel::create_debug_group_box()
{
  _debug_label = new QLabel("Door panel starting...");

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(_debug_label);

  QGroupBox* groupbox = new QGroupBox("Debug");
  groupbox->setLayout(layout);
  return groupbox;
}

//==============================================================================

void DoorPanel::create_layout()
{
  QGroupBox* door_selector_gb = create_door_selection_group_box();
  QGroupBox* state_gb = create_state_group_box();
  QGroupBox* request_gb = create_request_group_box();
  QGroupBox* debug_gb = create_debug_group_box();

  QGridLayout* layout = new QGridLayout;
  layout->addWidget(door_selector_gb, 0, 0, 1, 1);
  layout->addWidget(state_gb, 1, 0, 2, 1);
  layout->addWidget(request_gb, 3, 0, 2, 1);
  layout->addWidget(debug_gb, 5, 0, 1, 1);
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

void DoorPanel::create_connections()
{
  connect(this, SIGNAL(configChanged()), this,
    SLOT(update_door_name_selector()));
  connect(_door_name_selector,
    SIGNAL(currentTextChanged(const QString&)), this,
    SLOT(update_state_visualizer()));
  connect(_send_door_request_button,
    SIGNAL(clicked()), this,
    SLOT(send_door_request()));
}

//==============================================================================

void DoorPanel::door_state_callback(DoorState::UniquePtr msg)
{
  std::unique_lock<std::mutex> door_states_lock(_mutex);
  const std::string incoming_door_name = msg->door_name;

  DoorState new_msg = *(msg.get());
  std::string selected_door_name =
    _door_name_selector->currentText().toStdString();
  if (incoming_door_name == selected_door_name)
    display_state(new_msg);

  bool new_door_found =
    _door_states.find(incoming_door_name) == _door_states.end();
  _door_states[incoming_door_name] = new_msg;

  door_states_lock.unlock();

  if (new_door_found)
  {
    std::string debug_str =
      "New door [" + incoming_door_name  + "] found, refreshing...";
    RCLCPP_INFO(_node->get_logger(), debug_str.c_str());
    _debug_label->setText(QString::fromStdString(debug_str));
    Q_EMIT configChanged();
  }
  else
  {
    _debug_label->setText("Door panel running...");
  }
}

//==============================================================================

void DoorPanel::display_state(const DoorState& msg)
{
  _state_labels[0]->setText(
    QString::fromStdString(std::to_string(msg.door_time.sec)));
  _state_labels[1]->setText(
    QString::fromStdString(msg.door_name));
  _state_labels[2]->setText(
    QString::fromStdString(door_state_mode_string(msg.current_mode.value)));
}

//==============================================================================

std::string DoorPanel::door_state_mode_string(uint8_t mode) const
{
  switch (mode)
  {
    case DoorMode::MODE_CLOSED:
      return std::string("Closed");
    case DoorMode::MODE_MOVING:
      return std::string("Moving");
    case DoorMode::MODE_OPEN:
      return std::string("Open");
    default:
      return std::string("Undefined");
  }
}

//==============================================================================

QString DoorPanel::door_state_mode_tooltip() const
{
  std::stringstream ss;
  ss << std::to_string(DoorMode::MODE_CLOSED) << " - "
     << door_state_mode_string(DoorMode::MODE_CLOSED) << std::endl
     << std::to_string(DoorMode::MODE_MOVING) << " - "
     << door_state_mode_string(DoorMode::MODE_MOVING) << std::endl
     << std::to_string(DoorMode::MODE_OPEN) << " - "
     << door_state_mode_string(DoorMode::MODE_OPEN) << std::endl;
  return QString::fromStdString(ss.str());
}

//==============================================================================

} // namespace rmf_visualization_rviz2_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  rmf_visualization_rviz2_plugins::DoorPanel,
  rviz_common::Panel)

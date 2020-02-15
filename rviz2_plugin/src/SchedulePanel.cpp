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

#include "SchedulePanel.hpp"

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

namespace rviz2_plugin {

SchedulePanel::SchedulePanel(QWidget* parent)
: rviz_common::Panel(parent),
  Node("rviz_plugin_node"),
  _param_topic("/rviz_node/param"),
  _map_name("B1"),
  _finish_duration("600"),
  _start_duration_value(0)
{
  // Creating publisher 
  _param_pub = this->create_publisher<RvizParamMsg>(
      _param_topic.toStdString(), rclcpp::SystemDefaultsQoS());

  // Create layout for output topic and map_name
  QHBoxLayout* layout1 = new QHBoxLayout;
  layout1->addWidget(new QLabel("Output Topic:"));
  _topic_editor = new QLineEdit;
  _topic_editor->setFixedWidth(150);
  layout1->addWidget(_topic_editor);

  layout1->addWidget(new QLabel("Map Name:"));
  _map_name_editor = new QLineEdit;
  _map_name_editor->setFixedWidth(50);
  layout1->addWidget(_map_name_editor);
  layout1->addStretch();

  // Create layout for start_duration and finish_duration
  QHBoxLayout* layout2 = new QHBoxLayout;
  layout2->addWidget(new QLabel("Start Duration(s):"));
  _start_duration_editor = new QLineEdit;
  _start_duration_editor->setFixedWidth(50);
  layout2->addWidget(_start_duration_editor);

  layout2->addWidget(new QLabel("Query Duration(s):"));
  _finish_duration_editor = new QLineEdit;
  _finish_duration_editor->setFixedWidth(50);
  layout2->addWidget(_finish_duration_editor);
  layout2->addStretch();

  // Create layout for slider
  QHBoxLayout* layout3 = new QHBoxLayout;
  _start_duration_slider = new QSlider(Qt::Horizontal);
  _start_duration_slider->setMinimum(0);
  _start_duration_slider->setMaximum(600);
  _start_duration_slider->setSingleStep(5);
  layout3->addWidget(_start_duration_slider);
  _start_duration_max_editor = new QLineEdit;
  _start_duration_max_editor->setFixedWidth(50);
  layout3->addWidget(_start_duration_max_editor);
  layout3->addWidget(new QLabel("(s)"));

  // Combine all layouts in vertival layput
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addStretch();
  layout->addLayout(layout1);
  layout->addLayout(layout2);
  layout->addLayout(layout3);
  setLayout(layout);

  connect(_topic_editor,
      SIGNAL(editingFinished()), this, SLOT(update_topic()));
  connect(_map_name_editor,
      SIGNAL(editingFinished()), this, SLOT(update_map_name()));
  connect(_finish_duration_editor,
      SIGNAL(editingFinished()), this, SLOT(update_finish_duration()));
  connect(_start_duration_slider,
      SIGNAL(valueChanged(int)), this, SLOT(update_start_duration()));
  connect(_start_duration_max_editor,
      SIGNAL(editingFinished()), this, SLOT(update_start_duration_max()));
  connect(_start_duration_editor,
      SIGNAL(editingFinished()), this, SLOT(update_start_duration_editor()));

  // Updating text fields with default
  _topic_editor->setText(_param_topic);
  _map_name_editor->setText(_map_name);
  _finish_duration_editor->setText(_finish_duration);
  _start_duration_editor->setText("0");
  _start_duration_max_editor->setText("600");
}

void SchedulePanel::update_start_duration_max()
{
  set_start_duration_max(_start_duration_max_editor->text());
}

void SchedulePanel::update_start_duration()
{
  set_start_duration(_start_duration_slider->value());
}

void SchedulePanel::update_topic()
{
  set_topic(_topic_editor->text());
}

void SchedulePanel::update_map_name()
{
  set_map_name(_map_name_editor->text());
}

void SchedulePanel::update_finish_duration()
{
  set_finish_duration(_finish_duration_editor->text());
}

void SchedulePanel::update_start_duration_editor()
{
  set_start_duration(_start_duration_editor->text());
}

void SchedulePanel::set_start_duration_max(const QString& new_max)
{
  int finish_duration_value = std::stoi(_finish_duration.toStdString());
  int max_value = std::min(
        std::stoi(new_max.toStdString()),finish_duration_value);
  if (max_value > 0)
  {
    // Update the upper bound of the slider
    _start_duration_slider->setMaximum(max_value);
    _start_duration_max_editor->setText(QString::number(max_value));
    Q_EMIT configChanged();
  }
}

void SchedulePanel::set_start_duration(const QString& new_value)
{
  int value = std::stoi(new_value.toStdString());

  if (value < 0 or value == _start_duration_value)
    return;
  
  value = std::min(value, _start_duration_slider->maximum());
  _start_duration_value = value;
  _start_duration_slider->setValue(value);
  _start_duration_editor->setText(QString::number(value));
  send_param();
  Q_EMIT configChanged();
}

void SchedulePanel::set_start_duration(const int new_value)
{
  if (new_value != _start_duration_value && new_value >= 0)
  {
    _start_duration_value = new_value;
    // Update text box
    _start_duration_editor->setText(QString::number(_start_duration_value));
    send_param();
    Q_EMIT configChanged();
  }
}

void SchedulePanel::set_topic(const QString& new_topic)
{
  // Only take action if the name has changed.
  if (new_topic != _param_topic)
  {
    _param_topic = new_topic;
    // If the topic is the empty string, don't publish anything.
    if (_param_topic != "")
    {
      // Update publisher 
      _param_pub = this->create_publisher<RvizParamMsg>(
          _param_topic.toStdString(), rclcpp::SystemDefaultsQoS());
      // Send new message 
      send_param();
    }
    Q_EMIT configChanged();
  }
}

void SchedulePanel::set_map_name(const QString& new_name)
{
  if (new_name != _map_name)
  {
    _map_name = new_name;
    send_param();
    Q_EMIT configChanged();
  }
}

void SchedulePanel::set_finish_duration(const QString& new_duration)
{
  if (new_duration != _finish_duration)
  {
    _finish_duration = new_duration;
    send_param();
    Q_EMIT configChanged();
  }
}

void SchedulePanel::send_param()
{
  if (rclcpp::ok())
  {
    RvizParamMsg msg;
    msg.map_name = _map_name.toStdString();
    msg.query_duration = std::stoi(_finish_duration.toStdString());
    msg.start_duration = _start_duration_value;
    _param_pub->publish(msg);
  }
}

void SchedulePanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Topic", _param_topic);
  config.mapSetValue("Map", _map_name);
  config.mapSetValue("Finish", _finish_duration);
}

// Load all configuration data for this panel from the given Config object.
void SchedulePanel::load(const rviz_common::Config& config)
{
  rviz_common::Panel::load(config);
  QString topic;
  QString map;
  QString finish;
  if (config.mapGetString("Topic", &topic))
  {
    _topic_editor->setText(topic);
    update_topic();
  }
  if (config.mapGetString("Map", &map))
  {
    _map_name_editor->setText(map);
    update_map_name();
  }
  if (config.mapGetString("Finish", &finish))
  {
    _finish_duration_editor->setText(finish);
    update_finish_duration();
  }
}

} // namespace rviz2_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz2_plugin::SchedulePanel, rviz_common::Panel)

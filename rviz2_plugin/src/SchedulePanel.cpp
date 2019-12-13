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

  // creating publisher 
  _param_pub = this->create_publisher<RvizParamMsg>(
      _param_topic.toStdString(), rclcpp::SystemDefaultsQoS());

  // Create layout for output topic box
  QHBoxLayout* topic_layout = new QHBoxLayout;
  topic_layout->addWidget(new QLabel("Output Topic:"));
  _topic_editor = new QLineEdit;
  _topic_editor->setFixedWidth(400);
  topic_layout->addWidget(_topic_editor);
  topic_layout->addStretch();

  // Create layout for map_name box
  QHBoxLayout* map_name_layout = new QHBoxLayout;
  map_name_layout->addWidget(new QLabel("Map Name:"));
  _map_name_editor = new QLineEdit;
  _map_name_editor->setFixedWidth(400);
  map_name_layout->addWidget(_map_name_editor);
  map_name_layout->addStretch();

  // Create layout for finish_duration box
  QHBoxLayout* finish_duration_layout = new QHBoxLayout;
  finish_duration_layout->addWidget(new QLabel("Query Duration(s):"));
  _finish_duration_editor = new QLineEdit;
  _finish_duration_editor->setFixedWidth(200);
  finish_duration_layout->addWidget(_finish_duration_editor);
  finish_duration_layout->addStretch();

  // Create layout for start_duration slider box

  QVBoxLayout* start_duration_layout = new QVBoxLayout;
  QHBoxLayout* label_layout = new QHBoxLayout;
  label_layout->addWidget(new QLabel("Start Duration(s):"));
  _start_duration_editor = new QLineEdit;
  _start_duration_editor->setFixedWidth(100);
  label_layout->addWidget(_start_duration_editor);
  label_layout->addStretch();
  label_layout->addWidget(new QLabel("Max(s)"));
  start_duration_layout->addLayout(label_layout);

  QHBoxLayout* slider_layout = new QHBoxLayout;
  _start_duration_slider = new QSlider(Qt::Horizontal);
  _start_duration_slider->setMinimum(0);
  // set maximum to 1 hr
  // TODO read max value from text box
  _start_duration_slider->setMaximum(600);
  _start_duration_slider->setSingleStep(5);
  slider_layout->addWidget(_start_duration_slider);
  // slider_layout->addStretch();
  _start_duration_max_editor = new QLineEdit;
  _start_duration_max_editor->setFixedWidth(100);
  slider_layout->addWidget(_start_duration_max_editor);
  start_duration_layout->addLayout(slider_layout);

  // Combine all layouts in vertival layput
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(topic_layout);
  layout->addLayout(map_name_layout);
  layout->addLayout(start_duration_layout);
  layout->addLayout(finish_duration_layout);
  layout->addStretch();
  setLayout(layout);

  // _output_timer = new QTimer(this);

  connect( _topic_editor,
      SIGNAL(editingFinished()), this, SLOT(update_topic()));
  connect(_map_name_editor,
      SIGNAL(editingFinished()), this, SLOT(update_map_name()));
  connect(_finish_duration_editor,
      SIGNAL(editingFinished()), this, SLOT(update_finish_duration()));
  connect(_start_duration_slider,
      SIGNAL(valueChanged(int)), this, SLOT(update_start_duration()));

  //updating text fields with default
  _topic_editor->setText(_param_topic);
  _map_name_editor->setText(_map_name);
  _finish_duration_editor->setText(_finish_duration);
  _start_duration_editor->setText("0");

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

void SchedulePanel::set_start_duration(const int new_value)
{
  if (new_value != _start_duration_value && new_value >= 0)
  {
    _start_duration_value = new_value;
    // update text box
    _start_duration_editor->setText(QString::number(_start_duration_value));
    send_param();
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
      // update publisher 
      _param_pub = this->create_publisher<RvizParamMsg>(
          _param_topic.toStdString(), rclcpp::SystemDefaultsQoS());
      // send new message 
      send_param();
    }
    Q_EMIT configChanged();
  }
}

void SchedulePanel::set_map_name(const QString& new_name)
{
  // Only take action if the name has changed.
  if (new_name != _map_name)
  {
    _map_name = new_name;
    send_param();
    Q_EMIT configChanged();
  }
}


void SchedulePanel::set_finish_duration(const QString& new_duration)
{
  // Only take action if the name has changed.
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
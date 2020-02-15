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

#ifndef RVIZ2_PLUGIN__SRC__SCHEDULEPANEL_HPP
#define RVIZ2_PLUGIN__SRC__SCHEDULEPANEL_HPP

#include <rclcpp/rclcpp.hpp>

#include <rviz_common/panel.hpp>

#include <rmf_schedule_visualizer_msgs/msg/rviz_param.hpp>

#include<QSlider>
#include <QLineEdit>

namespace rviz2_plugin {

using  RvizParamMsg = rmf_schedule_visualizer_msgs::msg::RvizParam;

class SchedulePanel: public rviz_common::Panel, public rclcpp::Node
{
Q_OBJECT
public:
  SchedulePanel( QWidget* parent = 0 );
  virtual void load(const rviz_common::Config& config);
  virtual void save(rviz_common::Config config) const;

public Q_SLOTS:
  void set_start_duration(int seconds);
  void set_start_duration(const QString& max);
  void set_topic(const QString& topic);
  void set_map_name(const QString& map_name);
  void set_finish_duration(const QString& map_name);
  void set_start_duration_max(const QString& max);

protected Q_SLOTS:
  void send_param();
  void update_topic();
  void update_map_name();
  void update_start_duration();
  void update_finish_duration();
  void update_start_duration_max();
  void update_start_duration_editor();

protected:

  QSlider* _start_duration_slider;

  QLineEdit* _topic_editor;
  QLineEdit* _map_name_editor;
  QLineEdit* _finish_duration_editor;
  QLineEdit* _start_duration_editor;
  QLineEdit* _start_duration_max_editor;

  QString _param_topic;
  QString _map_name;
  QString _finish_duration;
  QString _start_duration;
  QString _start_duration_max;

  int _start_duration_value;

  rclcpp::Publisher<RvizParamMsg>::SharedPtr _param_pub;
};

} // namespace rviz2_plugin

#endif // RVIZ2_PLUGIN__SRC__SCHEDULEPANEL_HPP

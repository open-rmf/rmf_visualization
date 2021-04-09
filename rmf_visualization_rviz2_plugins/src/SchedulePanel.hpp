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

#ifndef RMF_VISUALIZATION_RVIZ2_PLUGINS__SRC__SCHEDULEPANEL_HPP
#define RMF_VISUALIZATION_RVIZ2_PLUGINS__SRC__SCHEDULEPANEL_HPP

#include <rclcpp/rclcpp.hpp>

#include <rviz_common/panel.hpp>

#include <rmf_visualization_msgs/msg/rviz_param.hpp>

#include <rmf_traffic_msgs/msg/negotiation_refusal.hpp>
#include <rmf_traffic_msgs/msg/negotiation_conclusion.hpp>
#include <rmf_traffic_msgs/msg/negotiation_notice.hpp>


#include <QPushButton>
#include <QSlider>
#include <QLineEdit>
#include <QTableWidget>

#include "NegotiationModel.hpp"

namespace rmf_visualization_rviz2_plugins {

using  RvizParamMsg = rmf_visualization_msgs::msg::RvizParam;
using  NegotiationNotice = rmf_traffic_msgs::msg::NegotiationNotice;
using  NegotiationConclusion = rmf_traffic_msgs::msg::NegotiationConclusion;
using  NegotiationRefusal = rmf_traffic_msgs::msg::NegotiationRefusal;

class SchedulePanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  SchedulePanel(QWidget* parent = 0);
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
  void cancel_negotiation();

private:
  void recieved_notification(const NegotiationNotice&);
  void recieved_conclusion(const NegotiationConclusion&);
  void spin();

protected:

  QSlider* _start_duration_slider;

  QLineEdit* _topic_editor;
  QLineEdit* _map_name_editor;
  QLineEdit* _finish_duration_editor;
  QLineEdit* _start_duration_editor;
  QLineEdit* _start_duration_max_editor;
  QPushButton* _cancel_button;

  QString _param_topic;
  QString _map_name;
  QString _finish_duration;
  QString _start_duration;
  QString _start_duration_max;

  QTableWidget* _negotiation_view;
  int _start_duration_value;

private:
  rclcpp::Publisher<RvizParamMsg>::SharedPtr _param_pub;
  rclcpp::Publisher<NegotiationRefusal>::SharedPtr _cancel_pub;
  rclcpp::Subscription<NegotiationNotice>::SharedPtr _notice_sub;
  rclcpp::Subscription<NegotiationConclusion>::SharedPtr _conclusion_sub;
  rclcpp::Node::SharedPtr _node;
  std::thread _thread;
  NegotiationModel* _nego_model;
};

} // namespace rmf_visualization_rviz2_plugins

#endif // RMF_VISUALIZATION_RVIZ2_PLUGINS__SRC__SCHEDULEPANEL_HPP

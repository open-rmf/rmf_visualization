import math
import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from rmf_fleet_msgs.msg import FleetState
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from building_map_msgs.msg import BuildingMap
from rmf_schedule_visualizer_msgs.msg import RvizParam


class FleetStateVisualizer(Node):
    def __init__(self, map_name):
        super().__init__('fleet_state_visualizer')
        self.get_logger().info('hello i am fleet state visualizer')
        self.display_names = True
        if (self.declare_parameter('display_names').value):
            self.display_names = self.get_parameter('display_names').value

        self.create_subscription(
            FleetState,
            'fleet_states',
            self.fleet_state_callback,
            10)

        self.marker_pub = self.create_publisher(
            MarkerArray,
            'fleet_markers',
            qos_profile=qos_profile_system_default)

        self.create_subscription(
            BuildingMap,
            'map', self.map_cb,
            qos_profile=qos_profile_system_default)

        self.create_subscription(
            RvizParam,
            'rviz_node/param',
            self.param_cb,
            qos_profile=qos_profile_system_default)

        self.robot_states = {}
        self.available_maps = []
        self.active_markers = {}
        self.map_name = map_name

    def map_cb(self, msg):
        print(f'Received building map with {len(msg.levels)} levels')
        for level in msg.levels:
            self.available_maps.append(level.name)

    def param_cb(self, msg):
        if msg.map_name not in self.available_maps:
            return
        self.map_name = msg.map_name
        marker_array = MarkerArray()
        # deleting previously avtive markers
        for name, marker in self.active_markers.items():
            if marker is not None:
                marker.action = Marker.DELETE
                marker_array.markers.append(marker)
        self.active_markers = {}
        self.marker_pub.publish(marker_array)

    def fleet_state_callback(self, msg):
        # print(msg)
        # store the RobotState msgs
        for robot in msg.robots:
            key = robot.name + "_" + robot.model
            self.robot_states[key] = robot

        ma = MarkerArray()
        marker_id = 0

        for key in self.robot_states.keys():
            rs = self.robot_states[key]
            if rs.location.level_name != self.map_name:
                continue

            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = rs.location.t
            m.ns = 'fleet_markers'
            m.id = marker_id
            marker_id += 1
            m.type = Marker.SPHERE  # 2
            m.action = Marker.MODIFY
            m.pose.position.x = rs.location.x
            m.pose.position.y = rs.location.y
            m.pose.position.z = 0.0
            m.pose.orientation.w = 1.0  # unit quaternion...
            if rs.name.split('_')[0] == 'mir100':
                m.scale.x = 0.6
                m.scale.y = 0.6
            elif rs.name.split('_')[0][:5] == 'magni':
                m.scale.x = 0.3
                m.scale.y = 0.3
            elif rs.name.split('_')[0] == 'Bed001':
                m.scale.x = 2.05
                m.scale.y = 2.05
            else:
                m.scale.x = 1.0
                m.scale.y = 1.0
            m.scale.z = 1.0
            m.color.r = 1.0  # todo
            m.color.b = 1.0  # todo
            m.color.a = 1.0
            # print(m)

            # now make the nose
            n = Marker()
            n.header.frame_id = 'map'
            n.header.stamp = rs.location.t
            n.ns = 'fleet_markers'
            n.id = marker_id
            marker_id += 1
            n.type = Marker.SPHERE  # 2
            n.action = Marker.MODIFY
            nose_distance = m.scale.x / 2.0
            n.pose.position.x = \
                rs.location.x + nose_distance * math.cos(rs.location.yaw)
            n.pose.position.y = \
                rs.location.y + nose_distance * math.sin(rs.location.yaw)
            n.pose.position.z = 0.0
            n.pose.orientation.w = 1.0  # unit quaternion...
            nose_radius = 0.25
            n.scale.x = nose_radius
            n.scale.y = nose_radius
            n.scale.z = nose_radius
            n.color.r = 1.0  # todo
            n.color.b = 1.0  # todo
            n.color.a = 1.0
            # print(n)

            if (self.display_names):
                # text marker
                t = Marker()
                t.header.frame_id = 'map'
                t.header.stamp = rs.location.t
                t.ns = 'fleet_markers'
                t.id = marker_id
                marker_id += 1
                t.type = Marker.TEXT_VIEW_FACING  # 9
                t.action = Marker.MODIFY
                t.pose.position.x = \
                    rs.location.x + 1.0 * math.cos(rs.location.yaw - 0.7853)
                t.pose.position.y = \
                    rs.location.y + 1.0 * math.sin(rs.location.yaw - 0.7853)
                t.pose.position.z = 0.0
                t.pose.orientation.w = 1.0
                t.text = rs.name
                t.scale.z = 0.3
                t.color.r = 1.0
                t.color.g = 1.0
                t.color.b = 1.0
                t.color.a = 1.0

            ma.markers.append(m)
            self.active_markers[f'{key}_m'] = m
            ma.markers.append(n)
            self.active_markers[f'{key}_n'] = n
            if (self.display_names):
                ma.markers.append(t)
                self.active_markers[f'{key}_t'] = t

        self.marker_pub.publish(ma)
        # print(ma)


def main(argv=sys.argv):
    rclpy.init(args=argv)
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser()
    parser.add_argument('-m', '--map-name', help='Default map name')
    args = parser.parse_args(args_without_ros[1:])

    n = FleetStateVisualizer(args.map_name)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass

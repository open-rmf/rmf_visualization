import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from rmf_fleet_msgs.msg import FleetState
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class FleetStateVisualizer(Node):
    def __init__(self):
        super().__init__('fleet_state_visualizer')
        self.get_logger().info('hello i am fleet state visualizer')
        self.create_subscription(
            FleetState,
            'fleet_states',
            self.fleet_state_callback,
            10)
        self.marker_pub = self.create_publisher(
            MarkerArray,
            'fleet_markers',
            qos_profile=qos_profile_system_default)
        
        self.robot_states = {}

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
            print("Robot Name: " + rs.name.split('_')[0])
            if rs.name.split('_')[0] == 'MiR':
                m.scale.x = 0.6
                m.scale.y = 0.6
            elif rs.name.split('_')[0][:5] == 'magni':
                m.scale.x = 0.3
                m.scale.y = 0.3
            elif rs.name.split('_')[0] == 'Bed001':
                m.scale.x = 2.05
                m.scale.y = 2.05
            else:
                m.scale.x = 0.5
                m.scale.y = 0.5
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
            n.pose.position.x = rs.location.x + nose_distance * math.cos(rs.location.yaw)
            n.pose.position.y = rs.location.y + nose_distance * math.sin(rs.location.yaw)
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

            ma.markers.append(m)
            ma.markers.append(n)
        
        self.marker_pub.publish(ma)
        # print(ma)

def main():
    rclpy.init()
    n = FleetStateVisualizer()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass

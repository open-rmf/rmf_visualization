import rclpy
from rclpy.node import Node

from rclpy.qos import qos_profile_system_default
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability

from rmf_door_msgs.msg import DoorState
from rmf_door_msgs.msg import DoorRequest
from rmf_door_msgs.msg import DoorMode

from rmf_lift_msgs.msg import LiftRequest
from rmf_lift_msgs.msg import LiftState

from rmf_building_map_msgs.msg import BuildingMap
from rmf_building_map_msgs.msg import Level
from rmf_building_map_msgs.msg import Door

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point

from rmf_visualization_msgs.msg import RvizParam

import math
import argparse
import sys


class BuildingSystemsVisualizer(Node):
    def __init__(self, map_name):
        super().__init__('building_systems_visualizer')
        self.get_logger().info('Building systems visualizer started...')

        qos = QoSProfile(
            history=History.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1,
            reliability=Reliability.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            durability=Durability.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)

        self.marker_pub = self.create_publisher(
            MarkerArray,
            'building_systems_markers',
            qos_profile=qos)

        self.create_subscription(
            BuildingMap,
            'map', self.map_cb,
            qos_profile=qos)

        self.create_subscription(
            DoorState, 'door_states',
            self.door_cb,
            qos_profile=qos_profile_system_default)

        self.create_subscription(
            LiftState,
            'lift_states',
            self.lift_cb,
            qos_profile=qos_profile_system_default)

        self.create_subscription(
            RvizParam,
            'rmf_visualization/parameters',
            self.param_cb,
            qos_profile=qos_profile_system_default)

        self.initialized = False

        # data obtained from BuildingMap
        self.building_doors = {}
        self.building_lifts = {}

        # name of the current map to display
        self.map_name = map_name

        # data obtained from /lift_states and /door_states
        self.lift_states = {}
        self.door_states = {}
        self.door_states[self.map_name] = {}

        # markers currently being displayed
        self.active_markers = {}

    def create_door_marker(self, msg):
        door_marker = Marker()
        door_marker.header.frame_id = 'map'
        door_marker.ns = msg.door_name
        door_marker.id = 0
        door_marker.type = Marker.LINE_LIST
        door_marker.action = Marker.ADD
        door_marker.pose.orientation.w = 1.0
        door_marker.scale.x = 0.3  # width of the door

        door = self.building_doors[self.map_name][msg.door_name]
        hinge1 = Point()
        hinge1.x = door.v1_x
        hinge1.y = door.v1_y
        hinge2 = Point()
        hinge2.x = door.v2_x
        hinge2.y = door.v2_y
        # print(f'    h1 [{hinge1.x},{hinge1.y}], h2 [{hinge2.x},{hinge2.y}]')
        door_marker.points.append(hinge1)
        door_marker.points.append(hinge2)

        # use red yellow and green color markers
        door_marker.color.a = 0.5
        door_mode = msg.current_mode.value
        if door_mode == 2:  # open
            door_marker.color.r = 0.60
            door_marker.color.g = 0.76
            door_marker.color.b = 0.25
        elif door_mode == 1:  # moving
            door_marker.color.r = 1.0
            door_marker.color.g = 0.86
            door_marker.color.b = 0.09
        else:
            door_marker.color.r = 0.78
            door_marker.color.g = 0.20
            door_marker.color.b = 0.20

        return door_marker

    def create_door_text_marker(self, msg):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.ns = msg.door_name + '_text'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.MODIFY
        marker.scale.z = 0.2

        door = self.building_doors[self.map_name][msg.door_name]
        hinge1 = Point()
        hinge1.x = door.v1_x
        hinge1.y = door.v1_y
        hinge2 = Point()
        hinge2.x = door.v2_x
        hinge2.y = door.v2_y

        marker.pose.position.z = 0.0
        marker.pose.position.x = 0.5*(hinge1.x + hinge2.x)
        marker.pose.position.y = 0.5*(hinge1.y + hinge2.y)
        # sadly text orientation is always "view facing" so no effect
        theta = math.atan2(abs(hinge2.y - hinge1.y), abs(hinge2.x - hinge1.x))
        marker.pose.orientation.w = math.cos(theta * 0.5)
        marker.pose.orientation.z = math.sin(theta * 0.5)

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.5
        door_mode = msg.current_mode.value
        if door_mode == 2:
            marker.text = msg.door_name + ":Open"
        elif door_mode == 1:
            marker.text = msg.door_name + ":Moving"
        else:
            marker.text = msg.door_name + ":Closed"

        return marker

    def create_lift_marker(self, lift_name):
        if lift_name not in self.building_lifts or \
          lift_name not in self.lift_states:
            return
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.ns = lift_name
        marker.id = 1
        marker.type = Marker.CUBE
        marker.action = Marker.MODIFY

        lift = self.building_lifts[lift_name]
        marker.pose.position.x = lift.ref_x
        marker.pose.position.y = lift.ref_y
        marker.pose.position.z = -0.5  # below lane markers
        marker.pose.orientation.w = math.cos(lift.ref_yaw * 0.5)
        marker.pose.orientation.z = math.sin(lift.ref_yaw * 0.5)
        marker.scale.x = lift.width
        marker.scale.y = lift.depth
        marker.scale.z = 0.1

        if self.lift_states[lift_name].door_state == 2:  # lift door open
            marker.color.r = 0.50
            marker.color.g = 0.70
            marker.color.b = 0.50
        else:
            marker.color.r = 0.40
            marker.color.g = 0.50
            marker.color.b = 0.65

        if self.lift_states[lift_name].current_floor != self.map_name or \
           self.lift_states[lift_name].motion_state == 1 or \
           self.lift_states[lift_name].motion_state == 2:
            # lift moving or not on current floor
            marker.color.a = 0.2
        else:
            marker.color.a = 0.8

        return marker

    def create_lift_text_marker(self, lift_name):
        if lift_name not in self.building_lifts or \
          lift_name not in self.lift_states:
            return
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.ns = lift_name + '_text'
        marker.id = 1
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.MODIFY
        marker.scale.z = 0.3

        lift = self.building_lifts[lift_name]

        marker.pose.position.z = 0.0
        marker.pose.position.x = lift.ref_x + 0.4
        marker.pose.position.y = 1.5 + lift.ref_y
        marker.pose.orientation.w = 1.0

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        lift_state = self.lift_states[lift_name]
        marker.text = "Lift:" + lift_name
        if self.lift_states[lift_name].motion_state != 0 and \
           self.lift_states[lift_name].motion_state != 3:  # lift is moving
            marker.text += "\n MovingTo:" + lift_state.destination_floor
        else:
            marker.text += "\n CurrentFloor:" + lift_state.current_floor
        if lift_state.door_state == 0:
            marker.text += "\n DoorState:Closed"
        elif lift_state.door_state == 1:
            marker.text += "\n DoorState:Moving"
        elif lift_state.door_state == 2:
            marker.text += "\n DoorState:Open"

        return marker

    def map_cb(self, msg):
        print(f'Received building map with {len(msg.levels)} levels \
          and {len(msg.lifts)} lifts')
        self.building_doors = {}
        self.building_lifts = {}
        for lift in msg.lifts:
            self.building_lifts[lift.name] = lift

        for level in msg.levels:
            self.building_doors[level.name] = {}
            self.door_states[level.name] = {}
            for door in level.doors:
                self.building_doors[level.name][door.name] = door
        self.initialized = True

    def door_cb(self, msg):
        if not self.initialized or \
          msg.door_name not in self.building_doors[self.map_name]:
            return

        publish_marker = False
        door_state = self.door_states[self.map_name]
        if msg.door_name not in door_state:
            door_state[msg.door_name] = msg
            publish_marker = True
        else:
            if msg.current_mode.value != \
              door_state[msg.door_name].current_mode.value:
                door_state[msg.door_name] = msg
                publish_marker = True

        if publish_marker:
            marker_array = MarkerArray()
            marker = self.create_door_marker(msg)
            text_marker = self.create_door_text_marker(msg)
            if marker is not None:
                marker_array.markers.append(marker)
                self.active_markers[msg.door_name] = marker
            if text_marker is not None:
                marker_array.markers.append(text_marker)
                self.active_markers[f'{msg.door_name}_text'] = text_marker
            self.marker_pub.publish(marker_array)

    def lift_cb(self, msg):
        if not self.initialized:
            return
        publish_marker = False
        if msg.lift_name not in self.lift_states:
            self.lift_states[msg.lift_name] = msg
            publish_marker = True
        else:
            stored_state = self.lift_states[msg.lift_name]
            if msg.current_floor != stored_state.current_floor or \
               msg.motion_state != stored_state.motion_state or \
               msg.door_state != stored_state.door_state:
                self.lift_states[msg.lift_name] = msg
                publish_marker = True

        if publish_marker:
            marker_array = MarkerArray()
            marker = self.create_lift_marker(msg.lift_name)
            text_marker = self.create_lift_text_marker(msg.lift_name)
            if marker is not None:
                marker_array.markers.append(marker)
            if text_marker is not None:
                marker_array.markers.append(text_marker)
            self.marker_pub.publish(marker_array)

    def param_cb(self, msg):
        if not self.initialized or \
          msg.map_name not in self.building_doors:
            return

        self.map_name = msg.map_name
        marker_array = MarkerArray()
        # deleting previously avtive door markers
        for name, marker in self.active_markers.items():
            if marker is not None:
                marker.action = Marker.DELETE
                marker_array.markers.append(marker)
        self.active_markers = {}
        self.marker_pub.publish(marker_array)

        # clearing door states and lift states so that markers can be updated
        self.door_states[self.map_name] = {}
        self.lift_states = {}


def main(argv=sys.argv):
    rclpy.init(args=argv)
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser()
    parser.add_argument('-m', '--map-name', help='Default map name')
    args = parser.parse_args(args_without_ros[1:])

    n = BuildingSystemsVisualizer(args.map_name)
    try:
        rclpy.spin(n)
        rclpy.get_logger.info('Shutting down...')
    except KeyboardInterrupt:
        rclpy.shutdown()

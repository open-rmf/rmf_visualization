import rclpy
from rclpy.node import Node

from rclpy.event_handler import PublisherEventCallbacks
from rclpy.event_handler import QoSPublisherMatchedInfo
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

        map_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL)

        marker_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=5,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL)

        self.marker_pub = self.create_publisher(
            MarkerArray,
            'building_systems_markers',
            qos_profile=marker_qos)

        self.create_subscription(
            BuildingMap,
            'map', self.map_cb,
            qos_profile=map_qos)

        self.create_subscription(
            RvizParam,
            'rmf_visualization/parameters',
            self.param_cb,
            qos_profile=qos_profile_system_default)

        # data obtained from BuildingMap
        self.building_doors = {}
        self.building_lifts = {}
        self.door_to_level_name = {}

        # name of the current map to display
        self.map_name = map_name

        # data obtained from /lift_states and /door_states
        self.lift_states = {}
        self.door_states = {}
        self.door_states[self.map_name] = {}

        # door markers currently being displayed
        self.active_markers = {}

    def publish_rviz_markers(self, map_name):
        marker_array = MarkerArray()
        if map_name != self.map_name:
            # deleting previously active door markers
            for marker in self.active_markers.values():
                marker.action = Marker.DELETE
                marker_array.markers.append(marker)
            self.active_markers = {}
            if map_name in self.building_doors:
                self.map_name = map_name
            else:
                self.marker_pub.publish(marker_array)
                return

        for msg in self.door_states[self.map_name].values():
            marker = self.create_door_marker(msg)
            text_marker = self.create_door_text_marker(msg)
            self.active_markers[msg.door_name] = marker
            self.active_markers[f'{msg.door_name}_text'] = text_marker
            marker_array.markers.append(marker)
            marker_array.markers.append(text_marker)
            self.marker_pub.publish(marker_array)
        for msg in self.lift_states.values():
            marker_array.markers.append(self.create_lift_marker(msg))
            marker_array.markers.append(self.create_lift_text_marker(msg))
        self.marker_pub.publish(marker_array)

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

    def create_lift_marker(self, msg):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.ns = msg.lift_name
        marker.id = 1
        marker.type = Marker.CUBE
        marker.action = Marker.MODIFY

        lift = self.building_lifts[msg.lift_name]
        marker.pose.position.x = lift.ref_x
        marker.pose.position.y = lift.ref_y
        marker.pose.position.z = -0.5  # below lane markers
        marker.pose.orientation.w = math.cos(lift.ref_yaw * 0.5)
        marker.pose.orientation.z = math.sin(lift.ref_yaw * 0.5)
        marker.scale.x = lift.width
        marker.scale.y = lift.depth
        marker.scale.z = 0.1

        if msg.door_state == 2:  # lift door open
            marker.color.r = 0.50
            marker.color.g = 0.70
            marker.color.b = 0.50
        else:
            marker.color.r = 0.40
            marker.color.g = 0.50
            marker.color.b = 0.65

        if msg.current_floor != self.map_name or \
           msg.motion_state == 1 or \
           msg.motion_state == 2:
            # lift moving or not on current floor
            marker.color.a = 0.2
        else:
            marker.color.a = 0.8

        return marker

    def create_lift_text_marker(self, msg):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.ns = msg.lift_name + '_text'
        marker.id = 1
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.MODIFY
        marker.scale.z = 0.3

        lift = self.building_lifts[msg.lift_name]

        marker.pose.position.z = 0.0
        marker.pose.position.x = lift.ref_x + 0.4
        marker.pose.position.y = 1.5 + lift.ref_y
        marker.pose.orientation.w = 1.0

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = "Lift:" + msg.lift_name
        if msg.motion_state != 0 and \
           msg.motion_state != 3:  # lift is moving
            marker.text += "\n MovingTo:" + msg.destination_floor
        else:
            marker.text += "\n CurrentFloor:" + msg.current_floor
        if msg.door_state == 0:
            marker.text += "\n DoorState:Closed"
        elif msg.door_state == 1:
            marker.text += "\n DoorState:Moving"
        elif msg.door_state == 2:
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
                self.door_to_level_name[door.name] = level.name
        self.init_subscriptions()

    def init_subscriptions(self):
        state_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=100,
            reliability=Reliability.RELIABLE,
            durability=Durability.VOLATILE)

        self.create_subscription(
            DoorState, 'door_states',
            self.door_cb,
            qos_profile=state_qos)

        self.create_subscription(
            LiftState,
            'lift_states',
            self.lift_cb,
            qos_profile=state_qos)

    def door_cb(self, msg):
        if msg.door_name not in self.door_to_level_name:
            return

        map_name = self.door_to_level_name[msg.door_name]

        publish_marker = False
        door_state = self.door_states[map_name]
        if msg.door_name not in door_state:
            door_state[msg.door_name] = msg
            publish_marker = self.map_name == map_name
        else:
            if msg.current_mode.value != \
              door_state[msg.door_name].current_mode.value:
                door_state[msg.door_name] = msg
                publish_marker = self.map_name == map_name

        if publish_marker:
            self.publish_rviz_markers(self.map_name)

    def lift_cb(self, msg):
        if msg.lift_name not in self.building_lifts:
            return

        if msg.lift_name not in self.lift_states:
            self.lift_states[msg.lift_name] = msg
        else:
            stored_state = self.lift_states[msg.lift_name]
            if msg.current_floor != stored_state.current_floor or \
               msg.motion_state != stored_state.motion_state or \
               msg.door_state != stored_state.door_state:
                self.lift_states[msg.lift_name] = msg
                self.publish_rviz_markers(self.map_name)


    def param_cb(self, msg):
        self.publish_rviz_markers(msg.map_name)


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

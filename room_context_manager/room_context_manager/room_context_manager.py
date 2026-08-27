#!/usr/bin/env python3

import os
import yaml
import rclpy
import math

from ament_index_python.packages import get_package_share_directory
from nav2_msgs.msg import SpeedLimit
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
)
from std_msgs.msg import Bool, Int32, String
from geometry_msgs.msg import PoseStamped


class RoomContextManager(Node):

    def __init__(self) -> None:
        super().__init__("room_context_manager")

        # -------------------------
        # Detection confirmation
        # -------------------------

        self.candidate_tag = None
        self.candidate_count = 0

        # Require the same tag this many times before
        # accepting a room change.
        self.required_detections = 3

        # Current room remains active even when the
        # AprilTag is no longer visible.
        self.current_tag = None
        self.current_room = None

        # Request current tag
        self.requested_tag = None
        self.create_subscription(
            Int32,
            "/room_context/confirm_tag",
            self.confirm_tag_callback,
            10,
        )

        # -------------------------
        # Load room configuration
        # -------------------------

        package_share = get_package_share_directory(
            "room_context_manager"
        )

        config_path = os.path.join(
            package_share,
            "config",
            "rooms.yaml",
        )

        with open(config_path, "r") as config_file:
            config = yaml.safe_load(config_file)

        self.rooms = config["rooms"]

        # YAML may interpret the tag IDs as integers,
        # but normalize them just in case.
        self.rooms = {
            int(tag_id): room_data
            for tag_id, room_data in self.rooms.items()
        }

        # -------------------------
        # Subscribers
        # -------------------------

        self.create_subscription(
            Int32,
            "/apriltag_scanner/detected_id",
            self.tag_callback,
            10,
        )

        # -------------------------
        # QoS Profile
        # -------------------------

        self.context_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # -------------------------
        # Publishers
        # -------------------------

        self.room_pub = self.create_publisher(
            String,
            "/room_context/current",
            self.context_qos,
        )

        self.navigation_allowed_pub = self.create_publisher(
            Bool,
            "/room_context/navigation_allowed",
            self.context_qos,
        )

        self.arm_allowed_pub = self.create_publisher(
            Bool,
            "/room_context/arm_motion_allowed",
            self.context_qos,
        )

        self.speed_limit_pub = self.create_publisher(
            SpeedLimit,
            "/speed_limit",
            10,
        )

        self.context_tag_pub = self.create_publisher(
            Int32,
            "/room_context/tag_id",
            self.context_qos,
        )

        self.get_logger().info(
            "Room context manager started"
        )

        self.get_logger().info(
            f"Loaded {len(self.rooms)} room definitions"
        )

        self.entry_pose_pub = self.create_publisher(
            PoseStamped,
            "/room_context/entry_pose",
            self.context_qos,
        )

    def tag_callback(self, msg: Int32) -> None:
        tag_id = int(msg.data)

        # Unknown tags should not change the current room.
        if tag_id not in self.rooms:
            self.get_logger().warning(
                f"AprilTag {tag_id} has no room configuration"
            )
            return
        # ---------------------------------
        # Room confirmation mode
        # ---------------------------------

        if self.requested_tag is not None:
            # Ignore other tags when confirming
            if tag_id != self.requested_tag:
                return

            if tag_id == self.candidate_tag:
                self.candidate_count += 1
            else:
                self.candidate_tag = tag_id
                self.candidate_count = 1

            self.get_logger().info(
                f"Tag confirmation {tag_id}: "
                f"{self.candidate_count}/"
                f"{self.required_detections}"
            )

            if self.candidate_count < self.required_detections:
                return

            self.activate_room(tag_id)
            self.requested_tag = None
            self.candidate_tag = None
            self.candidate_count = 0

            return

        # ---------------------------------
        # Room detectection mode
        # ---------------------------------

        # Already using this room.
        if tag_id == self.current_tag:
            self.candidate_tag = None
            self.candidate_count = 0
            return

        # Same candidate as previous detection.
        if tag_id == self.candidate_tag:
            self.candidate_count += 1

        else:
            # A different tag appeared.
            self.candidate_tag = tag_id
            self.candidate_count = 1

        self.get_logger().info(
            f"Room tag candidate {tag_id}: "
            f"{self.candidate_count}/"
            f"{self.required_detections}"
        )

        # Do not switch rooms until the tag has
        # been detected consistently.
        if self.candidate_count < self.required_detections:
            return

        self.activate_room(tag_id)

        self.candidate_tag = None
        self.candidate_count = 0

    def confirm_tag_callback(self, msg: Int32) -> None:
        tag_id = int(msg.data)
        if tag_id not in self.rooms:
            self.get_logger().warning(
                f"Cannot confirm unknown AprilTag {tag_id}"
            )

            return

        self.get_logger().info(
            f"Fresh confirmation requested for AprilTag {tag_id}"
        )

        self.requested_tag = tag_id
        self.candidate_tag = None
        self.candidate_count = 0

    def activate_room(self, tag_id: int) -> None:
        room = self.rooms[tag_id]

        self.current_tag = tag_id
        self.current_room = room["name"]

        max_speed = float(room["max_speed"])
        navigation_allowed = bool(
            room["navigation_allowed"]
        )

        arm_motion_allowed = bool(
            room["arm_motion_allowed"]
        )

        if navigation_allowed and "entry_pose" in room:
            entry = room["entry_pose"]

            entry_msg = PoseStamped()

            entry_msg.header.frame_id = "map"
            entry_msg.header.stamp = (
                self.get_clock().now().to_msg()
            )

            entry_msg.pose.position.x = float(entry["x"])
            entry_msg.pose.position.y = float(entry["y"])
            entry_msg.pose.position.z = 0.0

            yaw = float(entry["yaw"])

            entry_msg.pose.orientation.z = math.sin(yaw / 2.0)
            entry_msg.pose.orientation.w = math.cos(yaw / 2.0)

            self.entry_pose_pub.publish(entry_msg)

        self.get_logger().info(
            "================================"
        )

        self.get_logger().info(
            f"ROOM CHANGED: {self.current_room}"
        )

        self.get_logger().info(
            f"AprilTag: {tag_id}"
        )

        self.get_logger().info(
            f"Maximum speed: {max_speed:.2f} m/s"
        )

        self.get_logger().info(
            f"Navigation allowed: {navigation_allowed}"
        )

        self.get_logger().info(
            f"Arm motion allowed: {arm_motion_allowed}"
        )

        self.get_logger().info(
            "================================"
        )

        # -------------------------
        # Publish room name
        # -------------------------

        room_msg = String()
        room_msg.data = self.current_room

        self.room_pub.publish(room_msg)

        # -------------------------
        # Publish navigation rule
        # -------------------------

        navigation_msg = Bool()
        navigation_msg.data = navigation_allowed

        self.navigation_allowed_pub.publish(
            navigation_msg
        )

        # -------------------------
        # Publish arm rule
        # -------------------------

        arm_msg = Bool()
        arm_msg.data = arm_motion_allowed

        self.arm_allowed_pub.publish(
            arm_msg
        )

        # -------------------------
        # Publish Nav2 speed limit
        # -------------------------

        speed_msg = SpeedLimit()

        speed_msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        # False means absolute speed rather than
        # percentage of the robot's maximum.
        speed_msg.percentage = False
        speed_msg.speed_limit = max_speed

        self.speed_limit_pub.publish(speed_msg)

        tag_msg = Int32()
        tag_msg.data = tag_id
        self.context_tag_pub.publish(tag_msg)

def main(args=None) -> None:
    rclpy.init(args=args)

    node = RoomContextManager()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

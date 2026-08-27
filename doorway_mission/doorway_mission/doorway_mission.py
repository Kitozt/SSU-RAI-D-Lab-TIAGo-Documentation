#!/usr/bin/env python3

import math
from typing import Optional

import rclpy

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
)
from std_msgs.msg import Bool, Int32, String
from trajectory_msgs.msg import JointTrajectoryPoint


class DoorwayMission(Node):

    def __init__(self) -> None:
        super().__init__("doorway_mission")

        # ---------------------------------
        # Mission state
        # ---------------------------------

        self.state = "IDLE"

        self.detected_tag: Optional[int] = None
        self.current_context_tag: Optional[int] = None

        self.navigation_allowed: Optional[bool] = None
        self.current_room: Optional[str] = None

        self.nav_goal_handle = None

        self.confirm_tag_pub = self.create_publisher(
            Int32,
            "/room_context/confirm_tag",
            10,
        )

        # ---------------------------------
        # Doorway / entry poses
        # ---------------------------------

        self.declare_parameter("doorway_x", 0.0)
        self.declare_parameter("doorway_y", 0.0)
        self.declare_parameter("doorway_yaw", 0.0)

        self.room_entry_pose: Optional[PoseStamped] = None
        self.context_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.doorway_x = float(
            self.get_parameter("doorway_x").value
        )
        self.doorway_y = float(
            self.get_parameter("doorway_y").value
        )
        self.doorway_yaw = float(
            self.get_parameter("doorway_yaw").value
        )

        # ---------------------------------
        # Head scanning
        # ---------------------------------

        # radians
        self.scan_positions = [
            -0.55,
            0.0,
            0.55,
        ]

        self.scan_index = 0
        self.scan_timer = None
        self.policy_timer = None

        self.scan_dwell_seconds = 1.5
        self.policy_timeout_seconds = 8.0

        # ---------------------------------
        # QoS for persistent room context
        # ---------------------------------

        self.context_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ---------------------------------
        # Subscriptions
        # ---------------------------------

        self.create_subscription(
            Bool,
            "/doorway_mission/start",
            self.start_callback,
            10,
        )

        self.create_subscription(
            Int32,
            "/apriltag_scanner/detected_id",
            self.tag_callback,
            10,
        )

        self.create_subscription(
            Int32,
            "/room_context/tag_id",
            self.context_tag_callback,
            self.context_qos,
        )

        self.create_subscription(
            Bool,
            "/room_context/navigation_allowed",
            self.navigation_allowed_callback,
            self.context_qos,
        )

        self.create_subscription(
            String,
            "/room_context/current",
            self.room_callback,
            self.context_qos,
        )

        self.create_subscription(
            PoseStamped,
            "/room_context/entry_pose",
            self.entry_pose_callback,
            self.context_qos,
        )

        # ---------------------------------
        # Action clients
        # ---------------------------------

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose",
        )

        self.head_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/head_controller/follow_joint_trajectory",
        )

        self.head_goal_handle = None

        self.get_logger().info(
            "Doorway mission node started"
        )

    # =====================================
    # Start mission
    # =====================================

    def start_callback(self, msg: Bool) -> None:

        if not msg.data:
            return

        if self.state not in [
            "IDLE",
            "DONE",
            "REFUSED",
            "NO_TAG",
            "FAILED",
        ]:
            self.get_logger().warning(
                f"Mission already running: {self.state}"
            )
            return

        # We should not leave a room whose current
        # policy forbids navigation.
        if self.navigation_allowed is not True:
            self.get_logger().warning(
                "Cannot start mission: "
                "current room does not allow navigation "
                "or room context is unknown."
            )
            return

        if not self.nav_client.server_is_ready():
            self.get_logger().warning(
                "/navigate_to_pose is unavailable"
            )
            return

        self.detected_tag = None
        self.current_context_tag = None

        self.state = "GOING_TO_DOORWAY"

        self.get_logger().info(
            "MISSION: navigating to doorway"
        )

        doorway_pose = self.make_pose(
            self.doorway_x,
            self.doorway_y,
            self.doorway_yaw,
        )

        self.send_navigation_goal(doorway_pose)

    # =====================================
    # Navigation
    # =====================================

    def make_pose(
        self,
        x: float,
        y: float,
        yaw: float,
    ) -> PoseStamped:

        pose = PoseStamped()

        pose.header.frame_id = "map"
        pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        # Convert yaw to quaternion.
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        return pose

    def send_navigation_goal(
        self,
        pose: PoseStamped,
    ) -> None:

        goal = NavigateToPose.Goal()
        goal.pose = pose
        goal.behavior_tree = ""

        self.get_logger().info(
            f"Navigation goal: "
            f"x={pose.pose.position.x:.2f}, "
            f"y={pose.pose.position.y:.2f}"
        )

        future = self.nav_client.send_goal_async(
            goal
        )

        future.add_done_callback(
            self.navigation_goal_response
        )

    def navigation_goal_response(
        self,
        future,
    ) -> None:

        try:
            goal_handle = future.result()

        except Exception as error:
            self.get_logger().error(
                f"Could not send navigation goal: {error}"
            )
            self.state = "FAILED"
            return

        if not goal_handle.accepted:
            self.get_logger().warning(
                "Navigation goal rejected"
            )
            self.state = "FAILED"
            return

        self.nav_goal_handle = goal_handle

        result_future = (
            goal_handle.get_result_async()
        )

        result_future.add_done_callback(
            self.navigation_result_callback
        )

    def entry_pose_callback(
        self,
        msg:PoseStamped,
    ) -> None:
        self.room_entry_pose = msg

    def navigation_result_callback(
        self,
        future,
    ) -> None:

        try:
            wrapped_result = future.result()

        except Exception as error:
            self.get_logger().error(
                f"Navigation failed: {error}"
            )
            self.state = "FAILED"
            return

        if (
            wrapped_result.status
            != GoalStatus.STATUS_SUCCEEDED
        ):
            self.get_logger().warning(
                "Navigation did not succeed. "
                f"Status: {wrapped_result.status}"
            )
            self.state = "FAILED"
            return

        # --------------------------------
        # Arrived at doorway
        # --------------------------------

        if self.state == "GOING_TO_DOORWAY":

            self.get_logger().info(
                "Arrived at doorway"
            )

            self.begin_tag_scan()

        # --------------------------------
        # Entered allowed room
        # --------------------------------

        elif self.state == "ENTERING_ROOM":

            self.state = "DONE"

            self.get_logger().info(
                "MISSION COMPLETE: entered room"
            )

            self.move_head(0.0)

    # =====================================
    # Head scanning
    # =====================================

    def begin_tag_scan(self) -> None:

        if not self.head_client.server_is_ready():
            self.get_logger().warning(
                "Head action server unavailable"
            )
            self.state = "FAILED"
            return

        self.detected_tag = None

        # Do not accidentally use the previous room's waypoint.
        self.room_entry_pose = None

        self.scan_index = 0

        self.state = "SCANNING"

        self.get_logger().info(
            "Searching doorway for AprilTag"
        )

        self.move_head(
            self.scan_positions[self.scan_index]
        )

    def move_head(
        self,
        pan: float,
    ) -> None:

        goal = FollowJointTrajectory.Goal()

        goal.trajectory.joint_names = [
            "head_1_joint",
            "head_2_joint",
        ]

        point = JointTrajectoryPoint()

        point.positions = [
            float(pan),
            0.0,
        ]

        point.time_from_start.sec = 1
        point.time_from_start.nanosec = 0

        goal.trajectory.points = [point]

        future = self.head_client.send_goal_async(
            goal
        )

        future.add_done_callback(
            self.head_goal_response
        )

    def head_goal_response(
        self,
        future,
    ) -> None:

        try:
            goal_handle = future.result()

        except Exception as error:
            self.get_logger().error(
                f"Could not move head: {error}"
            )
            self.state = "FAILED"
            return

        if not goal_handle.accepted:
            self.get_logger().warning(
                "Head movement rejected"
            )
            self.state = "FAILED"
            return

        self.head_goal_handle = goal_handle

        result_future = (
            goal_handle.get_result_async()
        )

        result_future.add_done_callback(
            self.head_result_callback
        )

    def head_result_callback(
        self,
        future,
    ) -> None:

        try:
            future.result()

        except Exception as error:
            self.get_logger().error(
                f"Head movement failed: {error}"
            )

            return

        # If an AprilTag was found while moving,
        # tag_callback() already changed our state.
        if self.state != "SCANNING":
            return

        # Give the camera time to observe this
        # head position.
        self.start_scan_dwell_timer()

    def start_scan_dwell_timer(self) -> None:

        self.cancel_scan_timer()

        self.scan_timer = self.create_timer(
            self.scan_dwell_seconds,
            self.scan_dwell_callback,
        )

    def scan_dwell_callback(self) -> None:

        self.cancel_scan_timer()

        if self.state != "SCANNING":
            return

        if self.detected_tag is not None:
            return

        self.scan_index += 1

        if self.scan_index >= len(
            self.scan_positions
        ):
            self.get_logger().warning(
                "No AprilTag found at doorway. "
                "Robot will not enter."
            )

            self.state = "NO_TAG"

            self.move_head(0.0)

            return

        self.move_head(
            self.scan_positions[self.scan_index]
        )

    def cancel_scan_timer(self) -> None:

        if self.scan_timer is None:
            return

        timer = self.scan_timer
        self.scan_timer = None

        timer.cancel()
        self.destroy_timer(timer)

    def stop_head_movement(self) -> None:
        if self.head_goal_handle is None:
            return

        future = self.head_goal_handle.cancel_goal_async()
        future.add_done_callback(self.head_cancel_callback)

    def head_cancel_callback(self, future) -> None:
        try:
            response = future.result()

        except Exception as error:
            self.get_logger().warning(
                f"Could not cancel head movement: {error}"
            )

            return

        if len(response.goals_canceling) > 0:
            self.get_logger().info(
                "Head scan stopped because AprilTag was detected"
            )

        self.head_goal_handle = None

    # =====================================
    # AprilTag detection
    # =====================================

    def tag_callback(
        self,
        msg: Int32,
    ) -> None:

        if self.state != "SCANNING":
            return

        tag_id = int(msg.data)

        self.detected_tag = tag_id

        # Change state immediately so no new scan
        # positions are started.
        self.state = "WAITING_FOR_POLICY"

        # Cancel the dwell timer.
        self.cancel_scan_timer()

        # Stop the head wherever it currently is
        self.stop_head_movement()

        self.get_logger().info(
            f"AprilTag {tag_id} detected. "
            "Waiting for room policy confirmation."
        )

        # Confirm the tag for future checks
        confirm_msg = Int32()
        confirm_msg.data = tag_id

        self.confirm_tag_pub.publish(confirm_msg)

        self.start_policy_timeout()

    # =====================================
    # Room context
    # =====================================

    def room_callback(
        self,
        msg: String,
    ) -> None:

        self.current_room = msg.data

    def navigation_allowed_callback(
        self,
        msg: Bool,
    ) -> None:

        self.navigation_allowed = bool(
            msg.data
        )

    def context_tag_callback(
        self,
        msg: Int32,
    ) -> None:

        self.current_context_tag = int(
            msg.data
        )

        if self.state != "WAITING_FOR_POLICY":
            return

        # Only accept a policy if the room manager
        # confirmed the tag we just saw.
        if (
            self.current_context_tag
            != self.detected_tag
        ):
            return

        self.cancel_policy_timer()

        self.apply_confirmed_policy()

    def apply_confirmed_policy(self) -> None:

        self.get_logger().info(
            f"Confirmed room: {self.current_room}"
        )

        if self.navigation_allowed is not True:

            self.state = "REFUSED"

            self.get_logger().warning(
                "ENTRY REFUSED: room policy "
                "does not allow navigation."
            )

            self.move_head(0.0)
            return

        if self.room_entry_pose is None:
            self.state = "REFUSED"

            self.get_logger().warning(
                "ENTRY REFUSED: no entry waypoint "
                "is configured for this room."
            )

            self.move_head(0.0)
            return

        self.get_logger().info(
            "ENTRY ALLOWED"
        )

        self.get_logger().info(
            f"Entering {self.current_room}"
        )

        self.get_logger().info(
            f"Entry waypoint: "
            f"x={self.room_entry_pose.pose.position.x:.2f}, "
            f"y={self.room_entry_pose.pose.position.y:.2f}"
        )

        self.state = "ENTERING_ROOM"
        self.move_head(0.0)
        self.send_navigation_goal(
            self.room_entry_pose
        )

    # =====================================
    # Policy timeout
    # =====================================

    def start_policy_timeout(self) -> None:

        self.cancel_policy_timer()

        self.policy_timer = self.create_timer(
            self.policy_timeout_seconds,
            self.policy_timeout_callback,
        )

    def policy_timeout_callback(self) -> None:

        self.cancel_policy_timer()

        if self.state != "WAITING_FOR_POLICY":
            return

        self.get_logger().warning(
            "AprilTag was seen, but room policy "
            "was not confirmed. Robot will not enter."
        )

        self.state = "REFUSED"

        self.move_head(0.0)

    def cancel_policy_timer(self) -> None:

        if self.policy_timer is None:
            return

        timer = self.policy_timer
        self.policy_timer = None

        timer.cancel()
        self.destroy_timer(timer)


def main(args=None) -> None:

    rclpy.init(args=args)

    node = DoorwayMission()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.cancel_scan_timer()
        node.cancel_policy_timer()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

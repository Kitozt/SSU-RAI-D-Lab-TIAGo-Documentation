#!/usr/bin/env python3

from typing import Optional

import rclpy

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
)
from std_msgs.msg import Bool, String


class RoomNavigator(Node):

    def __init__(self) -> None:
        super().__init__("room_navigator")

        # We intentionally start with "unknown".
        # The robot should not autonomously navigate
        # until it knows the room policy.
        self.navigation_allowed: Optional[bool] = None
        self.current_room: Optional[str] = None

        self.current_goal_handle = None
        self.navigation_active = False

        self.context_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ----------------------------
        # Room-context subscriptions
        # ----------------------------

        self.create_subscription(
            String,
            "/room_context/current",
            self.room_callback,
            self.context_qos,
        )

        self.create_subscription(
            Bool,
            "/room_context/navigation_allowed",
            self.navigation_allowed_callback,
            self.context_qos,
        )

        # ----------------------------
        # Navigation goal input
        # ----------------------------

        # For now we can publish a PoseStamped here.
        # Later this will come from the room/mission logic.
        self.create_subscription(
            PoseStamped,
            "/room_navigation/goal",
            self.goal_callback,
            10,
        )

        # ----------------------------
        # Nav2 action client
        # ----------------------------

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose",
        )

        self.get_logger().info(
            "Room-aware navigator started"
        )

    def room_callback(self, msg: String) -> None:
        self.current_room = msg.data

        self.get_logger().info(
            f"Current room: {self.current_room}"
        )

    def navigation_allowed_callback(
        self,
        msg: Bool,
    ) -> None:

        previous_value = self.navigation_allowed
        self.navigation_allowed = bool(msg.data)

        self.get_logger().info(
            f"Navigation allowed: "
            f"{self.navigation_allowed}"
        )

        # If navigation suddenly becomes forbidden
        # while TIAGo is moving, cancel the goal.
        if (
            previous_value is not False
            and self.navigation_allowed is False
            and self.navigation_active
        ):
            self.get_logger().warning(
                "Room policy changed to restricted. "
                "Canceling active navigation."
            )

            self.cancel_navigation()

    def goal_callback(
        self,
        msg: PoseStamped,
    ) -> None:

        self.get_logger().info(
            "Received navigation request"
        )

        # No room context yet.
        if self.navigation_allowed is None:
            self.get_logger().warning(
                "Navigation refused: "
                "room context is unknown."
            )
            return

        # Current room forbids autonomous movement.
        if not self.navigation_allowed:
            self.get_logger().warning(
                f"Navigation refused in room: "
                f"{self.current_room}"
            )
            return

        # Prevent overlapping navigation goals for now.
        if self.navigation_active:
            self.get_logger().warning(
                "Navigation already active."
            )
            return

        if not self.nav_client.server_is_ready():
            self.get_logger().warning(
                "/navigate_to_pose action server "
                "is not available."
            )
            return

        self.send_navigation_goal(msg)

    def send_navigation_goal(
        self,
        pose: PoseStamped,
    ) -> None:

        goal = NavigateToPose.Goal()

        goal.pose = pose

        # Empty means use the default Nav2 behavior tree.
        goal.behavior_tree = ""

        self.get_logger().info(
            f"Sending navigation goal: "
            f"x={pose.pose.position.x:.2f}, "
            f"y={pose.pose.position.y:.2f}"
        )

        future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=self.navigation_feedback,
        )

        future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(
        self,
        future,
    ) -> None:

        try:
            goal_handle = future.result()

        except Exception as error:
            self.get_logger().error(
                f"Failed to send navigation goal: "
                f"{error}"
            )
            return

        if not goal_handle.accepted:
            self.get_logger().warning(
                "Navigation goal was rejected."
            )
            return

        self.current_goal_handle = goal_handle
        self.navigation_active = True

        self.get_logger().info(
            "Navigation goal accepted."
        )

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            self.navigation_result_callback
        )

    def navigation_feedback(
        self,
        feedback_msg,
    ) -> None:

        feedback = feedback_msg.feedback

        self.get_logger().debug(
            f"Distance remaining: "
            f"{feedback.distance_remaining:.2f} m"
        )

    def navigation_result_callback(
        self,
        future,
    ) -> None:

        self.navigation_active = False
        self.current_goal_handle = None

        try:
            wrapped_result = future.result()

        except Exception as error:
            self.get_logger().error(
                f"Navigation failed: {error}"
            )
            return

        status = wrapped_result.status

        self.get_logger().info(
            f"Navigation finished with "
            f"status {status}"
        )

    def cancel_navigation(self) -> None:

        if self.current_goal_handle is None:
            return

        future = (
            self.current_goal_handle.cancel_goal_async()
        )

        future.add_done_callback(
            self.cancel_callback
        )

    def cancel_callback(
        self,
        future,
    ) -> None:

        try:
            response = future.result()

        except Exception as error:
            self.get_logger().error(
                f"Could not cancel navigation: "
                f"{error}"
            )
            return

        if len(response.goals_canceling) > 0:
            self.get_logger().warning(
                "Navigation cancellation accepted."
            )

        else:
            self.get_logger().warning(
                "Navigation cancellation was "
                "not accepted."
            )


def main(args=None) -> None:
    rclpy.init(args=args)

    node = RoomNavigator()

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

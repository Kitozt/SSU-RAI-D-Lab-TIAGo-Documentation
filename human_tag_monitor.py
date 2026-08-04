#!/usr/bin/env python3

from typing import Optional

import rclpy
from hri_msgs.msg import IdsList
from rclpy.node import Node
from std_msgs.msg import Int32


class HumanTagMonitor(Node):
    def __init__(self) -> None:
        super().__init__("human_tag_monitor")

        self.human_ids: list[str] = []
        self.last_tag_id: Optional[int] = None
        self.last_tag_time = None
        self.previous_state = None

        self.create_subscription(
            IdsList,
            "/humans/bodies/tracked",
            self.humans_callback,
            10,
        )

        self.create_subscription(
            Int32,
            "/apriltag_scanner/detected_id",
            self.tag_callback,
            10,
        )

        self.timer = self.create_timer(
            1.0,
            self.report_status,
        )

        self.get_logger().info(
            "Human and AprilTag monitor started"
        )

    def humans_callback(self, msg: IdsList) -> None:
        self.human_ids = list(msg.ids)

    def tag_callback(self, msg: Int32) -> None:
        self.last_tag_id = int(msg.data)
        self.last_tag_time = self.get_clock().now()

    def report_status(self) -> None:
        humans_visible = len(self.human_ids) > 0
        tag_visible = False

        if self.last_tag_time is not None:
            age_seconds = (
                self.get_clock().now() - self.last_tag_time
            ).nanoseconds / 1e9

            # Treat the tag as visible if it was detected
            # during the last two seconds.
            tag_visible = age_seconds < 2.0

        current_state = (
            humans_visible,
            tag_visible,
            tuple(self.human_ids),
            self.last_tag_id if tag_visible else None,
        )

        if current_state == self.previous_state:
            return

        if humans_visible and tag_visible:
            self.get_logger().info(
                f"Human detected: {self.human_ids}; "
                f"AprilTag detected: {self.last_tag_id}"
            )
        elif humans_visible:
            self.get_logger().info(
                f"Human detected: {self.human_ids}; "
                "no AprilTag currently visible"
            )
        elif tag_visible:
            self.get_logger().info(
                f"AprilTag detected: {self.last_tag_id}; "
                "no human currently visible"
            )
        else:
            if self.previous_state is not None:
                previous_human_visible = self.previous_state[0]
                previous_tag_visible = self.previous_state[1]
                if previous_human_visible and previous_tag_visible:
                    self.get_logger().info(
                        "Human and AprilTag no longer detected"
                    )
                elif previous_human_visible:
                    self.get_logger().info(
                        "Human no longer detected"
                    )
                elif previous_tag_visible:
                    self.get_logger().info(
                        "AprilTag no longer detected"
                    )

        self.previous_state = current_state


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HumanTagMonitor()

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

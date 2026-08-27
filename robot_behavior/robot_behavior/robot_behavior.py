#!/usr/bin/env python3

from typing import Optional

import rclpy
from communication_skills.action import Say
from control_msgs.action import FollowJointTrajectory
from hri_msgs.msg import IdsList
from play_motion2_msgs.action import PlayMotion2
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Int32
from trajectory_msgs.msg import JointTrajectoryPoint


class RobotBehavior(Node):
    def __init__(self) -> None:
        super().__init__("robot_behavior")

        self.human_visible = False

        self.last_tag_id: Optional[int] = None
        self.last_tag_time = None
        self.tag_timeout = 2.0

        # Prevent the same tag from repeatedly triggering.
        self.active_tag_id: Optional[int] = None

        self.create_subscription(
            IdsList,
            "/humans/bodies/tracked",
            self.human_callback,
            10,
        )

        self.create_subscription(
            Int32,
            "/apriltag_scanner/detected_id",
            self.tag_callback,
            10,
        )

        self.create_timer(
            0.5,
            self.check_tag_timeout,
        )

        self.get_logger().info("Robot behavior node started")

        self.head_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/head_controller/follow_joint_trajectory",
        )

        self.head_busy = False

        self.say_client = ActionClient(
            self,
            Say,
            "/skill/say",
        )

        self.speech_busy = False

        self.motion_client = ActionClient(
            self,
            PlayMotion2,
            "/play_motion2",
        )

        self.motion_busy = False
        self.next_motion = None

    def human_callback(self, msg: IdsList) -> None:
        currently_visible = len(msg.ids) > 0

        # Human has just appeared.
        if currently_visible and not self.human_visible:
            self.get_logger().info(
                f"Human appeared: {list(msg.ids)}"
            )
            self.on_human_detected(list(msg.ids))

        # Human has just disappeared.
        elif not currently_visible and self.human_visible:
            self.get_logger().info("Human disappeared")
            self.on_human_lost()

        self.human_visible = currently_visible

    def tag_callback(self, msg: Int32) -> None:
        tag_id = int(msg.data)

        self.last_tag_id = tag_id
        self.last_tag_time = self.get_clock().now()

        # Only trigger when this tag first becomes active.
        if tag_id != self.active_tag_id:
            self.active_tag_id = tag_id
            self.get_logger().info(
                f"AprilTag {tag_id} appeared"
            )
            self.on_tag_detected(tag_id)

    def head_goal_response_callback(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:
            self.head_busy = False
            self.get_logger().error(
                f"Could not send head goal: {error}"
            )

            return

        if not goal_handle.accepted:
            self.head_busy = False
            self.get_logger().warning(
                "Head movement was rejected"
            )

            return

        self.get_logger().info(
            "Head movement accepted"
        )

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            self.head_result_callback
        )

    def head_result_callback(self, future) -> None:
        self.head_busy = False

        try:
            result = future.result()
        except Exception as error:
            self.get_logger().error(
                f"Head movement failed: {error}"
            )

            return

        self.get_logger().info(
            f"Head movement finished with status "
            f"{result.status}"
        )

    def motion_goal_response_callback(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:
            self.motion_busy = False
            self.get_logger().error(
                f"Could not send motion goal: {error}"
            )

            return

        if not goal_handle.accepted:
            self.motion_busy = False
            self.get_logger().warning(
                "Motion goal was rejected"
            )

            return

        self.get_logger().info("Motion accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            self.motion_result_callback
        )


    def motion_result_callback(self, future) -> None:
        try:
            wrapped_result = future.result()
            result = wrapped_result.result
        except Exception as error:
            self.motion_busy = False
            self.get_logger().error(
                f"Motion failed: {error}"
            )

            return


        follow_up_motion = self.next_motion
        self.motion_busy = False
        self.next_motion = None
        if not result.success:
            self.get_logger().error(
                f"Motion failed: {result.error}"
            )

            return

        self.get_logger().info("Motion finished")
        if follow_up_motion is not None:
            self.play_motion(follow_up_motion)

    def say_goal_response_callback(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:
            self.speech_busy = False
            self.get_logger().error(
                f"Could not send speech goal: {error}"
            )
            return

        if not goal_handle.accepted:
            self.speech_busy = False
            self.get_logger().warning(
                "Speech goal was rejected"
            )
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            self.say_result_callback
        )

    def say_result_callback(self, future) -> None:
        self.speech_busy = False

        try:
            future.result()
        except Exception as error:
            self.get_logger().error(
                f"Speech failed: {error}"
            )

            return

        self.get_logger().info("Speech finished")

    def check_tag_timeout(self) -> None:
        if self.last_tag_time is None:
            return

        age_seconds = (
            self.get_clock().now() - self.last_tag_time
        ).nanoseconds / 1e9

        if (
            age_seconds >= self.tag_timeout
            and self.active_tag_id is not None
        ):
            lost_tag_id = self.active_tag_id
            self.active_tag_id = None

            self.get_logger().info(
                f"AprilTag {lost_tag_id} disappeared"
            )
            self.on_tag_lost(lost_tag_id)

    def move_head(
        self,
        pan: float,
        tilt: float,
        duration: float = 2.0,
        ) -> None:
        if self.head_busy:
            self.get_logger().info(
                "Head is already moving; ignoring new command"
            )
            return

        if not self.head_action_client.server_is_ready():
            self.get_logger().warning(
                "Head action server is not available"
            )

            return

        goal = FollowJointTrajectory.Goal()

        goal.trajectory.joint_names = [
        "head_1_joint",
        "head_2_joint",
        ]

        point = JointTrajectoryPoint()
        point.positions = [
            float(pan),
            float(tilt),
        ]

        whole_seconds = int(duration)
        nanoseconds = int(
            (duration - whole_seconds) * 1_000_000_000
        )

        point.time_from_start.sec = whole_seconds
        point.time_from_start.nanosec = nanoseconds

        goal.trajectory.points = [point]

        self.head_busy = True

        future = self.head_action_client.send_goal_async(goal)
        future.add_done_callback(self.head_goal_response_callback)

    def play_motion(
        self,
        motion_name: str,
        next_motion: str | None = None,
        skip_planning: bool = False,
        ) -> None:
        if self.motion_busy:
            self.get_logger().info(
                "A motion is already active"
            )

            return

        if not self.motion_client.server_is_ready():
            self.get_logger().warning(
                "play_motion2 action server is not available"
            )

            return

        goal = PlayMotion2.Goal()
        goal.motion_name = motion_name
        goal.skip_planning = skip_planning

        self.motion_busy = True
        self.next_motion = next_motion

        future = self.motion_client.send_goal_async(goal)
        future.add_done_callback(
            self.motion_goal_response_callback
        )

    def speak(self, text: str) -> None:
        if self.speech_busy:
            self.get_logger().info(
                "Speech already active; ignoring new speech request"
            )

            return

        if not self.say_client.server_is_ready():
            self.get_logger().warning(
                "Say action server is not available"
            )

            return

        goal = Say.Goal()
        goal.meta.caller = "robot_behavior"
        goal.meta.priority = 128
        goal.person_id = ""
        goal.group_id = ""
        goal.input = text

        self.speech_busy = True

        future = self.say_client.send_goal_async(goal)
        future.add_done_callback(
            self.say_goal_response_callback
        )

    def on_human_detected(self, human_ids: list[str]) -> None:
        # Replace this with a real robot action.
        self.get_logger().info(
            "ACTION: greet and wave to the human"
        )

        # Tilt head upward slightly, since a person is likely
        # to be in close and in front of the robot.
        self.move_head(
            pan=0.0,
            tilt=0.15,
            duration=1.5,
        )

        self.speak(
            "Hello! It is nice to meet you."
        )

        self.play_motion(
            "wave",
            next_motion="home",
            skip_planning=False,
        )

    def on_human_lost(self) -> None:
        # Replace this with a real robot action.
        self.get_logger().info(
            "ACTION: return to waiting position"
        )

        self.move_head(
            pan=0.0,
            tilt = 0.0,
            duration=1.5,
        )

    def on_tag_detected(self, tag_id: int) -> None:
        if tag_id == 0:
            self.get_logger().info(
                "ACTION: acknowledge detecting [tag: 0] and wave"
            )

            self.speak(
                "I detect April Tag zero."
            )

            self.play_motion(
                "wave",
                next_motion="home",
                skip_planning=False,
            )

        elif tag_id == 1:
            self.get_logger().info(
                "ACTION: acknowledge detecting [tag: 1] and look left"
            )

            self.speak(
                "I detect April Tag one."
            )


            self.move_head(
                pan=-0.35,
                tilt=0.0,
                duration=1.5,
            )

        elif tag_id == 2:
            self.get_logger().info(
                "ACTION: acknowledge detecting [tag: 2] and look right"
            )

            self.speak(
                "I detect April Tag two."
            )

            self.move_head(
                pan=0.35,
                tilt=0.0,
                duration=1.5
            )

        else:
            self.get_logger().info(
                f"No action assigned to tag {tag_id}"
            )

    def on_tag_lost(self, tag_id: int) -> None:
        self.get_logger().info(
            f"ACTION: tag {tag_id} is no longer visible"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotBehavior()

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

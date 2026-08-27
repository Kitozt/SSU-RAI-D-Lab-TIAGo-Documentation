#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from communication_skills.action import Say
from hri_msgs.msg import LiveSpeech
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future

# The following demo subscribes to speech-to-text output and triggers TTS
# based on response

class ASRDemo(Node):
    def __init__(self):
        super().__init__("asr_tutorial")
        self.get_logger().info('Initializing parrot bot')

        self.asr_sub = self.create_subscription(
            LiveSpeech,
            '/humans/voices/anonymous_speaker/speech',
            self.asr_result,
            1)

        self.say_client = ActionClient(
            self,
            Say,
            "/skill/say",
            callback_group=MutuallyExclusiveCallbackGroup())
        self.say_client.wait_for_server()

        self.tts_goal_future_handle = Future()

        self.get_logger().info("ASR demo ready")

    def asr_result(self, msg: LiveSpeech):

        # the LiveSpeech message has two main field: incremental and final.
        # 'incremental' is updated has soon as a word is recognized, and
        # will change while the sentence recognition progresses.
        # 'final' is only set at the end, when a full sentence is
        # recognized.
        sentence = msg.final

        self.get_logger().info("Understood sentence: " + sentence)

        goal = Say.Goal()
        goal.meta.priority = 1
        if (sentence == "hello"):
            goal.input = "Hello!"
        elif (sentence == "how are you"):
            goal.input = "I am feeling great"
        elif (sentence == "goodbye"):
            goal.input = "See you!"

        self.say_client.send_goal(goal)



def main(args=None):
    rclpy.init(args=args)

    node = ASRDemo()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
    node = ASRDemo()
    node.get_logger().info('Parrot bot started')

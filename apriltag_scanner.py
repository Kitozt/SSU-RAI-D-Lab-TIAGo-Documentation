#!/usr/bin/env python3

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Int32



class AprilTagScanner(Node):
    def __init__(self) -> None:
        super().__init__("apriltag_scanner")

        self.bridge = CvBridge()

        # Start with AprilTag family 36h11.
        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_APRILTAG_36H11
        )

        # Compatible with OpenCV 4.5.4
        self.detector_parameters = cv2.aruco.DetectorParameters_create()

        self.subscription = self.create_subscription(
            Image,
            "/head_front_camera/rgb/image_raw",
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.debug_publisher = self.create_publisher(
            Image,
            "/apriltag_scanner/debug_image",
            10,
        )

        self.last_detected_ids = None
        self.get_logger().info(
            "scanning /head_front_camera/rgb/image_raw "
            "for AprilTag 36h11 markers"
        )

        self.tag_id_publisher = self.create_publisher(
            Int32,
            "/apriltag_scanner/detected_id",
            10,
	)

    def image_callback(self, msg: Image) -> None:
        try:
            # Camera publishes as rgb8, so we need to explicitly request it
            rgb_image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="rgb8",
            )

        except Exception as error:
            self.get_logger().error(
                f"Could not convert camera image: {error}"
            )

            return

        # OpenCV drawing functions conventionally expect BGR
        bgr_image = cv2.cvtColor(
            rgb_image,
            cv2.COLOR_RGB2BGR
        )

        corners, ids, _ = cv2.aruco.detectMarkers(
            bgr_image,
            self.dictionary,
            parameters=self.detector_parameters,
        )

        if ids is not None:
            detected_ids = tuple(int(tag_id) for tag_id in ids.flatten())

            # This publishes every detected AprilTag ID to ROS 2 message
            for tag_id in ids.flatten():
                tag_id = int(tag_id)
                id_msg = Int32()
                id_msg.data = tag_id
                self.tag_id_publisher.publish(id_msg)

            cv2.aruco.drawDetectedMarkers(
                bgr_image,
                corners,
                ids,
            )

            # Avoid printing the same results for each camera frame
            if detected_ids != self.last_detected_ids:
                self.get_logger().info(
                    f"Detected AprilTag IDs: {detected_ids}"
                )

            self.last_detected_ids = detected_ids
        else:
            if self.last_detected_ids is not None:
                self.get_logger().info(
                    "No AprilTags currently visible"
                )

            self.last_detected_ids = None

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(
                bgr_image,
                encoding="bgr8",
            )

            debug_msg.header = msg.header
            self.debug_publisher.publish(debug_msg)
        except Exception as error:
            self.get_logger().error(
                f"Could not publish debug image: {error}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AprilTagScanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

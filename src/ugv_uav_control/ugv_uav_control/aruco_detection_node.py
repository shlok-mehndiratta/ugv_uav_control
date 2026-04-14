"""
ArUco Detection Node
====================
Detects ArUco marker ID 55 from overhead camera imagery and publishes
the robot's world-frame pose to /aruco_pose.

Subscriptions:
    overhead_camera/image  (sensor_msgs/Image)

Publications:
    /aruco_pose  (geometry_msgs/PoseStamped)  — world frame
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import math


def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    q = [0] * 4
    q[0] = cy * cp * sr - sy * sp * cr
    q[1] = sy * cp * sr + cy * sp * cr
    q[2] = sy * cp * cr - cy * sp * sr
    q[3] = cy * cp * cr + sy * sp * sr
    return q


class ArucoDetectionNode(Node):
    def __init__(self):
        super().__init__('aruco_detection_node')

        # === SUBSCRIPTION ===
        self.create_subscription(
            Image, 'overhead_camera/image', self.image_callback, 10)

        # === PUBLISHER ===
        self.aruco_pose_pub = self.create_publisher(
            PoseStamped, '/aruco_pose', 10)

        # === CV BRIDGE & ARUCO ===
        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_6X6_250)
        try:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        except AttributeError:
            self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_params.minMarkerPerimeterRate = 0.01

        # === CAMERA CALIBRATION ===
        self.camera_height = 5.0
        self.fov = 1.047  # 60 degrees
        self.image_width = None
        self.image_height = None
        self.scale = None

        self.get_logger().info('ArUco Detection Node Started')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge exception: {e}')
            return

        # One-time camera calibration
        if self.image_width is None:
            self.image_height, self.image_width = cv_image.shape[:2]
            world_width = 2 * self.camera_height * math.tan(self.fov / 2)
            self.scale = world_width / self.image_width
            self.get_logger().info(
                f'Camera Calibrated. Scale: {self.scale:.5f} m/px')

        # Detect ArUco markers
        corners, ids, _ = cv2.aruco.detectMarkers(
            cv_image, self.aruco_dict, parameters=self.aruco_params)

        if ids is None:
            return

        for i in range(len(ids)):
            if ids[i][0] == 55:  # Robot Marker ID
                c = corners[i][0]
                cx = int(np.mean(c[:, 0]))
                cy = int(np.mean(c[:, 1]))

                # Heading from top edge midpoint
                top_mid_x = (c[0][0] + c[1][0]) / 2
                top_mid_y = (c[0][1] + c[1][1]) / 2
                robot_angle = math.atan2(top_mid_y - cy, top_mid_x - cx)

                # Publish world-frame pose
                aruco_msg = PoseStamped()
                aruco_msg.header.stamp = self.get_clock().now().to_msg()
                aruco_msg.header.frame_id = 'world'
                aruco_msg.pose.position.x = \
                    (cx - self.image_width / 2) * self.scale
                aruco_msg.pose.position.y = \
                    (cy - self.image_height / 2) * self.scale
                q = quaternion_from_euler(0, 0, robot_angle)
                aruco_msg.pose.orientation.x = q[0]
                aruco_msg.pose.orientation.y = q[1]
                aruco_msg.pose.orientation.z = q[2]
                aruco_msg.pose.orientation.w = q[3]
                self.aruco_pose_pub.publish(aruco_msg)
                break


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ArucoDetectionNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()

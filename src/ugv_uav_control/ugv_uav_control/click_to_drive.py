#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import math
import numpy as np

class ClickToDrive(Node):
    def __init__(self):
        super().__init__('click_to_drive')
        
        self.subscription = self.create_subscription(
            Image,
            '/overhead_camera/image',
            self.image_callback,
            10)
            
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
        self.br = CvBridge()
        self.current_image = None
        self.target_x = None
        self.target_y = None
        
        self.robot_x = None
        self.robot_y = None
        self.robot_theta = None
        
        # Camera parameters
        self.camera_height = 20.0
        self.fov = 1.047 # 60 degrees
        self.image_width = None
        self.image_height = None
        self.scale = None
        
        # ArUco Dictionary
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        try:
            self.aruco_params = aruco.DetectorParameters_create()
        except AttributeError:
            self.aruco_params = aruco.DetectorParameters()
        
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('Click to Drive Node Started (ArUco Mode).')

    def image_callback(self, msg):
        try:
            self.current_image = self.br.imgmsg_to_cv2(msg, "bgr8")
            
            if self.image_width is None:
                self.image_height, self.image_width = self.current_image.shape[:2]
                # Calculate scale (meters per pixel)
                world_width = 2 * self.camera_height * math.tan(self.fov / 2)
                self.scale = world_width / self.image_width
                self.get_logger().info(f'Camera Calibrated. Scale: {self.scale:.5f} m/px')
            
            # Detect ArUco
            gray = cv2.cvtColor(self.current_image, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
            
            if ids is not None and len(ids) > 0:
                self.get_logger().info(f'ArUco Marker Detected: {ids}', throttle_duration_sec=2.0)
                for i in range(len(ids)):
                    if ids[i][0] == 55: # Robot Marker ID
                        # Get center
                        c = corners[i][0]
                        cx = int(np.mean(c[:, 0]))
                        cy = int(np.mean(c[:, 1]))
                        
                        # Convert to world coordinates
                        self.robot_x = (self.image_height / 2 - cy) * self.scale
                        self.robot_y = (self.image_width / 2 - cx) * self.scale
                        
                        # Calculate orientation
                        # Front is average of top two corners (0 and 1)
                        front_x = (c[0][0] + c[1][0]) / 2
                        front_y = (c[0][1] + c[1][1]) / 2
                        
                        wx_front = (self.image_height / 2 - front_y) * self.scale
                        wy_front = (self.image_width / 2 - front_x) * self.scale
                        
                        self.robot_theta = math.atan2(wy_front - self.robot_y, wx_front - self.robot_x)
                        
                        # Draw marker (Manually to avoid segfaults)
                        cv2.polylines(self.current_image, [c.astype(np.int32)], True, (0, 255, 0), 2)
                        cv2.circle(self.current_image, (cx, cy), 5, (255, 0, 0), -1) # Blue Dot (Robot)
            
            # Draw target
            if self.target_x is not None:
                px = int(self.image_width / 2 - self.target_y / self.scale)
                py = int(self.image_height / 2 - self.target_x / self.scale)
                cv2.circle(self.current_image, (px, py), 5, (0, 0, 255), -1) # Red Dot (Target)

            # Resize for display (50% of original size)
            display_image = cv2.resize(self.current_image, (960, 540))
            cv2.imshow("Overhead Camera", display_image)
            cv2.setMouseCallback("Overhead Camera", self.mouse_callback)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.scale is None: return
            # Scale click coordinates back to original image size (from 960x540 to 1920x1080)
            orig_x = x * 2
            orig_y = y * 2
            # Convert pixel to world
            self.target_x = (self.image_height / 2 - orig_y) * self.scale
            self.target_y = (self.image_width / 2 - orig_x) * self.scale
            self.get_logger().info(f'New Target: ({self.target_x:.3f}, {self.target_y:.3f})')

    def control_loop(self):
        if self.target_x is None: return
        if self.robot_x is None:
            self.get_logger().warn('Waiting for ArUco marker detection...', throttle_duration_sec=2.0)
            return
            
        dx = self.target_x - self.robot_x
        dy = self.target_y - self.robot_y
        dist = math.sqrt(dx*dx + dy*dy)
        
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        
        if dist > 0.05: # 5cm tolerance
            angle_to_target = math.atan2(dy, dx)
            angle_diff = angle_to_target - self.robot_theta
            
            while angle_diff > math.pi: angle_diff -= 2*math.pi
            while angle_diff < -math.pi: angle_diff += 2*math.pi
            
            speed = 0.5 * dist
            cmd.twist.linear.x = min(max(speed, 0.05), 0.25)
            cmd.twist.angular.z = 1.5 * angle_diff
            
            if abs(angle_diff) > 0.5:
                cmd.twist.linear.x = 0.0
        else:
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0
            
        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ClickToDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.subscription = self.create_subscription(
            Image,
            'overhead_camera/image',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_pub = self.create_publisher(Image, '/line_follower/debug_image', 10)
        self.bridge = CvBridge()
        
        # ArUco detector setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        # Control parameters
        self.linear_speed = 0.2
        self.kp = 0.005 # Proportional gain for angular velocity
        
        self.get_logger().info('Line Follower Node Started')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'cv_bridge exception: {e}')
            return

        # 1. Detect Robot (ArUco)
        corners, ids, rejected = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.aruco_params)
        
        robot_pos = None
        robot_angle = 0.0
        
        if ids is not None:
            # Assume ID 0 is our robot
            for i in range(len(ids)):
                if ids[i][0] == 0:
                    # Get center of the marker
                    c = corners[i][0]
                    cx = int((c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4)
                    cy = int((c[0][1] + c[1][1] + c[2][1] + c[3][1]) / 4)
                    robot_pos = (cx, cy)
                    
                    # Calculate orientation (angle of the top edge)
                    # ArUco corners: 0=TL, 1=TR, 2=BR, 3=BL (in canonical view)
                    # But we need to check how it's mounted. 
                    # Usually top is between 0 and 1.
                    # Let's use the vector from center to the midpoint of 0-1 side (top)
                    top_mid_x = (c[0][0] + c[1][0]) / 2
                    top_mid_y = (c[0][1] + c[1][1]) / 2
                    
                    # Vector from center to top
                    dx = top_mid_x - cx
                    dy = top_mid_y - cy
                    robot_angle = math.atan2(dy, dx)
                    
                    # Draw robot position
                    cv2.circle(cv_image, robot_pos, 5, (0, 0, 255), -1)
                    # Draw direction
                    cv2.line(cv_image, robot_pos, (int(top_mid_x), int(top_mid_y)), (0, 255, 0), 2)
                    break
        
        if robot_pos is None:
            self.get_logger().info('Robot not detected')
            # Stop robot
            twist = Twist()
            self.publisher.publish(twist)
            return

        # 2. Detect Path (Lane Keeping via Distance Transform)
        # Convert to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Threshold: Lines are black (0), Background is white (255)
        # We want obstacles (lines) to be 0 for distanceTransform
        _, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
        
        # Compute distance transform
        # dist_map values are distance to nearest zero pixel (line)
        dist_map = cv2.distanceTransform(thresh, cv2.DIST_L2, 5)
        
        # Normalize for visualization
        cv2.normalize(dist_map, dist_map, 0, 1.0, cv2.NORM_MINMAX)
        
        # Find target point
        # We want a point that is:
        # 1. "In front" of the robot (lookahead)
        # 2. Has a high distance value (center of lane)
        
        lookahead_dist = 100 # pixels
        search_radius = 50 # pixels around the lookahead point
        
        # Project a point in front of the robot
        proj_x = robot_pos[0] + lookahead_dist * math.cos(robot_angle)
        proj_y = robot_pos[1] + lookahead_dist * math.sin(robot_angle)
        
        # Create a mask for the search region
        mask = np.zeros_like(gray)
        cv2.circle(mask, (int(proj_x), int(proj_y)), search_radius, 255, -1)
        
        # Apply mask to distance map
        # We only care about valid pixels (inside image)
        masked_dist = cv2.bitwise_and(dist_map, dist_map, mask=mask)
        
        # Find the max value location in the masked area
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(masked_dist)
        
        if max_val > 0:
            target_point = max_loc
        else:
            # Fallback: just go straight if no clear path found (e.g. out of bounds)
            target_point = (int(proj_x), int(proj_y))
            self.get_logger().warn('No clear path found, moving straight')

        # Draw debug info
        # Show distance map (scaled up)
        debug_img = (dist_map * 255).astype(np.uint8)
        debug_img = cv2.cvtColor(debug_img, cv2.COLOR_GRAY2BGR)
        
        # Draw robot
        cv2.circle(debug_img, robot_pos, 5, (0, 0, 255), -1)
        # Draw search area
        cv2.circle(debug_img, (int(proj_x), int(proj_y)), search_radius, (0, 255, 255), 1)
        # Draw target
        cv2.circle(debug_img, target_point, 8, (255, 0, 0), -1)
        cv2.line(debug_img, robot_pos, target_point, (0, 255, 0), 2)
        
        # Publish debug image
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, "bgr8")
            self.debug_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f'Debug image publish error: {e}')
            
        # Try to show window, but handle crash
        try:
            cv2.imshow("Line Follower Debug", debug_img)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().warn(f'cv2.imshow failed: {e}')
        
        # 3. Control
        # Calculate error
        dx = target_point[0] - robot_pos[0]
        dy = target_point[1] - robot_pos[1]
        target_angle = math.atan2(dy, dx)
        
        error = target_angle - robot_angle
        # Normalize
        error = (error + math.pi) % (2 * math.pi) - math.pi
        
        twist = Twist()
        twist.linear.x = self.linear_speed
        
        # P-Control for angular velocity
        # Increase Kp since we want responsive steering to stay in lane
        kp = 2.0 
        twist.angular.z = kp * error 
        
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

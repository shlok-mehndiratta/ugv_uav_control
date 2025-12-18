import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Point, PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place).
    quat = [x, y, z, w]
    """
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

class StanleyPerception(Node):
    def __init__(self):
        super().__init__('stanley_perception')
        self.get_logger().info('Initializing StanleyPerception...')
        
        self.subscription = self.create_subscription(
            Image,
            'overhead_camera/image',
            self.image_callback,
            10)
        self.debug_pub = self.create_publisher(Image, '/middle_path/debug_image', 10)
        self.bridge = CvBridge()
        
        # ArUco detector setup
        self.get_logger().info('Setting up ArUco...')
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        try:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        except AttributeError:
            self.aruco_params = cv2.aruco.DetectorParameters()
            
        # Tune parameters for small markers / high altitude
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 23
        self.aruco_params.adaptiveThreshWinSizeStep = 10
        self.aruco_params.minMarkerPerimeterRate = 0.01 
        self.aruco_params.polygonalApproxAccuracyRate = 0.05
        self.aruco_params.perspectiveRemovePixelPerCell = 4 
        self.aruco_params.maxErroneousBitsInBorderRate = 0.35 
        
        # Camera parameters
        self.camera_height = 10.0
        self.fov = 1.047 # 60 degrees
        self.image_width = None
        self.image_height = None
        self.scale = None
        
        self.waypoint_pub = self.create_publisher(PoseStamped, '/waypoints', 10) 
        self.frame_count = 0 
        
        self.get_logger().info('Stanley Perception Node Started')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'cv_bridge exception: {e}')
            return

        if self.image_width is None:
            self.image_height, self.image_width = cv_image.shape[:2]
            world_width = 2 * self.camera_height * math.tan(self.fov / 2)
            self.scale = world_width / self.image_width
            self.get_logger().info(f'Camera Calibrated. Scale: {self.scale:.5f} m/px')

        # 1. Detect Robot (ArUco)
        corners, ids, rejected = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.aruco_params)
        
        robot_pos = None
        robot_angle = 0.0
        
        if ids is not None:
            for i in range(len(ids)):
                if ids[i][0] == 55: # Robot Marker ID is 55
                    c = corners[i][0]
                    cx = int((c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4)
                    cy = int((c[0][1] + c[1][1] + c[2][1] + c[3][1]) / 4)
                    robot_pos = (cx, cy)
                    
                    top_mid_x = (c[0][0] + c[1][0]) / 2
                    top_mid_y = (c[0][1] + c[1][1]) / 2
                    
                    dx = top_mid_x - cx
                    dy = top_mid_y - cy
                    robot_angle = math.atan2(dy, dx)
                    break
        
        if robot_pos is None:
            return

        # 2. Compute Middle Path
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
        
        h, w = thresh.shape[:2]
        flood_mask = np.zeros((h+2, w+2), np.uint8)
        
        seed_point = None
        found_seed = False
        
        search_dists = [40, 50, 60, 30, 70] 
        search_angles = [0, 0.2, -0.2, 0.4, -0.4, 0.6, -0.6] 
        
        for dist in search_dists:
            for angle_offset in search_angles:
                check_angle = robot_angle + angle_offset
                sx = int(robot_pos[0] + dist * math.cos(check_angle))
                sy = int(robot_pos[1] + dist * math.sin(check_angle))
                sx = max(0, min(w-1, sx))
                sy = max(0, min(h-1, sy))
                if thresh[sy, sx] == 255:
                    seed_point = (sx, sy)
                    found_seed = True
                    break
            if found_seed: break
            
        if not found_seed:
            seed_point = robot_pos
        
        lane_mask_img = thresh.copy()
        cv2.floodFill(lane_mask_img, flood_mask, seed_point, 128)
        
        lane_mask = np.zeros_like(thresh)
        lane_mask[lane_mask_img == 128] = 255
        
        dist_map = cv2.distanceTransform(thresh, cv2.DIST_L2, 5)
        dist_map[lane_mask == 0] = 0
        
        min_val, max_val, _, _ = cv2.minMaxLoc(dist_map)
        if max_val > 0:
            dist_norm = dist_map / max_val
        else:
            dist_norm = dist_map.copy()
        
        # Visualization
        heatmap = (dist_norm * 255).astype(np.uint8)
        heatmap = cv2.applyColorMap(heatmap, cv2.COLORMAP_JET)
        final_img = cv2.addWeighted(cv_image, 0.6, heatmap, 0.4, 0)
        
        _, ridge_mask = cv2.threshold(dist_norm, 0.9, 1.0, cv2.THRESH_BINARY)
        ridge_mask = (ridge_mask * 255).astype(np.uint8)
        contours, _ = cv2.findContours(ridge_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(final_img, contours, -1, (0, 255, 0), 2)
        
        # --- Stanley Logic: Nearest Point & Tangent ---
        target_point = None
        path_heading = 0.0
        
        min_dist = float('inf')
        closest_pt = None
        
        if contours:
            for cnt in contours:
                for p in cnt:
                    pt = p[0]
                    d = math.hypot(pt[0] - robot_pos[0], pt[1] - robot_pos[1])
                    if d < min_dist:
                        min_dist = d
                        closest_pt = (pt[0], pt[1])
        
        if closest_pt is not None:
            target_point = closest_pt
            
            # Calculate Tangent
            best_tangent_pt = None
            best_score = -1
            
            robot_dir_x = math.cos(robot_angle)
            robot_dir_y = math.sin(robot_angle)
            
            if contours:
                for cnt in contours:
                    for p in cnt:
                        pt = p[0]
                        d = math.hypot(pt[0] - closest_pt[0], pt[1] - closest_pt[1])
                        
                        if 15 < d < 30: 
                            vx = pt[0] - closest_pt[0]
                            vy = pt[1] - closest_pt[1]
                            dot = vx * robot_dir_x + vy * robot_dir_y
                            
                            if dot > best_score:
                                best_score = dot
                                best_tangent_pt = (pt[0], pt[1])
            
            if best_tangent_pt:
                path_heading = math.atan2(best_tangent_pt[1] - closest_pt[1], 
                                          best_tangent_pt[0] - closest_pt[0])
            else:
                path_heading = robot_angle
        else:
             target_point = robot_pos
             path_heading = robot_angle

        # Draw overlays
        cv2.circle(final_img, robot_pos, 8, (255, 255, 255), -1) 
        cv2.circle(final_img, target_point, 8, (0, 0, 0), -1) 
        cv2.line(final_img, robot_pos, target_point, (255, 0, 255), 2)
        
        # Debug Publish
        self.frame_count += 1
        if self.frame_count % 5 == 0:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(final_img, "bgr8")
                self.debug_pub.publish(debug_msg)
            except Exception:
                pass

        # Publish Waypoint
        if self.scale is not None:
            dx_pixel = target_point[0] - robot_pos[0]
            dy_pixel = target_point[1] - robot_pos[1]
            
            v_x = target_point[0] - robot_pos[0]
            v_y = target_point[1] - robot_pos[1]
            
            dist_pixels = math.sqrt(v_x*v_x + v_y*v_y)
            dist_meters = dist_pixels * self.scale
            
            target_angle_image = math.atan2(v_y, v_x)
            relative_angle = target_angle_image - robot_angle
            relative_angle = (relative_angle + math.pi) % (2 * math.pi) - math.pi
            
            # Negate for ROS frame (Y Left vs Y Down)
            relative_angle = -relative_angle
            
            waypoint_x = dist_meters * math.cos(relative_angle)
            waypoint_y = dist_meters * math.sin(relative_angle)
            
            # Orientation: Path Heading relative to Robot Heading
            # path_heading is in Image Frame.
            relative_path_yaw = -(path_heading - robot_angle)
            relative_path_yaw = (relative_path_yaw + math.pi) % (2 * math.pi) - math.pi
            
            q = quaternion_from_euler(0, 0, relative_path_yaw)
            
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'base_link'
            
            pose_msg.pose.position.x = waypoint_x
            pose_msg.pose.position.y = waypoint_y
            pose_msg.pose.position.z = 0.0
            
            pose_msg.pose.orientation.x = q[0]
            pose_msg.pose.orientation.y = q[1]
            pose_msg.pose.orientation.z = q[2]
            pose_msg.pose.orientation.w = q[3]
            
            self.waypoint_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = StanleyPerception()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

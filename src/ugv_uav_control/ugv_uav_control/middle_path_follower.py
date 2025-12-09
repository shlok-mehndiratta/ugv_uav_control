import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Point, PoseStamped
# from tf_transformations import quaternion_from_euler # Not available
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place).
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
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

class MiddlePathFollower(Node):
    def __init__(self):
        super().__init__('middle_path_follower')
        self.get_logger().info('Initializing MiddlePathFollower...')
        
        self.subscription = self.create_subscription(
            Image,
            'overhead_camera/image',
            self.image_callback,
            10)
        # self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
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
        self.aruco_params.minMarkerPerimeterRate = 0.01 # Allow smaller markers (default 0.03)
        self.aruco_params.polygonalApproxAccuracyRate = 0.05
        self.aruco_params.perspectiveRemovePixelPerCell = 4 # Lower resolution processing
        self.aruco_params.maxErroneousBitsInBorderRate = 0.35 # Allow more border errors
        
        # Control parameters
        self.linear_speed = 0.2
        self.kp = 2.0 
        
        # Camera parameters for scale calculation
        self.camera_height = 10.0
        self.fov = 1.047 # 60 degrees
        self.image_width = None
        self.image_height = None
        self.scale = None
        
        self.waypoint_pub = self.create_publisher(PoseStamped, '/waypoints', 10) 
        self.nearest_pub = self.create_publisher(PoseStamped, '/nearest_point', 10)
        self.frame_count = 0 
        
        self.get_logger().info('Middle Path Follower Node Started')

    def image_callback(self, msg):
        #self.get_logger().info('Image callback received', throttle_duration_sec=1.0)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'cv_bridge exception: {e}')
            return

        if self.image_width is None:
            self.image_height, self.image_width = cv_image.shape[:2]
            # Calculate scale (meters per pixel)
            world_width = 2 * self.camera_height * math.tan(self.fov / 2)
            self.scale = world_width / self.image_width
            self.get_logger().info(f'Camera Calibrated. Scale: {self.scale:.5f} m/px')

        # 1. Detect Robot (ArUco)
        self.get_logger().info('Detecting markers...', throttle_duration_sec=1.0)
        corners, ids, rejected = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.aruco_params)
        
        if ids is not None:
             self.get_logger().info(f'Detected markers: {ids.flatten()}', throttle_duration_sec=2.0)
        else:
             self.get_logger().info('No markers detected', throttle_duration_sec=2.0)
        
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
            self.get_logger().info('Robot not detected', throttle_duration_sec=2.0)
            # twist = Twist()
            # self.publisher.publish(twist)
            return

        # 2. Compute Middle Path
        #self.get_logger().info('Computing middle path...', throttle_duration_sec=1.0)
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Threshold: Lines are black (0), Background is white (255)
        _, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
        
        # Floodfill to isolate the lane
        # Create a mask for floodFill (needs to be 2 pixels larger than image)
        h, w = thresh.shape[:2]
        flood_mask = np.zeros((h+2, w+2), np.uint8)
        
        # Robust Seed Search
        # Search in a cone in front of the robot for a valid white pixel (track)
        # This prevents failure if the exact front point is on a black line
        seed_point = None
        found_seed = False
        
        # Search parameters: Prioritize straight ahead and moderate distance
        search_dists = [40, 50, 60, 30, 70] 
        search_angles = [0, 0.2, -0.2, 0.4, -0.4, 0.6, -0.6] # Radians (~ +/- 35 deg)
        
        for dist in search_dists:
            for angle_offset in search_angles:
                check_angle = robot_angle + angle_offset
                sx = int(robot_pos[0] + dist * math.cos(check_angle))
                sy = int(robot_pos[1] + dist * math.sin(check_angle))
                
                # Clamp to image bounds
                sx = max(0, min(w-1, sx))
                sy = max(0, min(h-1, sy))
                
                # Check if valid track (white)
                if thresh[sy, sx] == 255:
                    seed_point = (sx, sy)
                    found_seed = True
                    break
            if found_seed: break
            
        if not found_seed:
            # Fallback: Just use robot center (might be trapped in marker, but better than nothing)
            seed_point = robot_pos
        
        # Perform floodfill on a copy to get the mask
        lane_mask_img = thresh.copy()
        cv2.floodFill(lane_mask_img, flood_mask, seed_point, 128) # Fill with 128
        
        # Extract the filled area (value 128)
        lane_mask = np.zeros_like(thresh)
        lane_mask[lane_mask_img == 128] = 255
        
        # Distance Transform
        #self.get_logger().info('Running distanceTransform...', throttle_duration_sec=1.0)
        # We only care about distance to zero pixels (lines)
        # But we only want the result INSIDE the lane mask
        dist_map = cv2.distanceTransform(thresh, cv2.DIST_L2, 5)
        
        # Mask the distance map immediately
        dist_map[lane_mask == 0] = 0
        
        # Normalize for visualization (Local max within the lane)
        min_val, max_val, _, _ = cv2.minMaxLoc(dist_map)
        if max_val > 0:
            dist_norm = dist_map / max_val
        else:
            dist_norm = dist_map.copy()
        
        # Create the "Middle Track" visualization
        # Apply colormap to distance transform
        heatmap = (dist_norm * 255).astype(np.uint8)
        heatmap = cv2.applyColorMap(heatmap, cv2.COLORMAP_JET)
        
        # Blend heatmap with original image
        final_img = cv2.addWeighted(cv_image, 0.6, heatmap, 0.4, 0)
        
        # Highlight the "ridge" (middle path)
        _, ridge_mask = cv2.threshold(dist_norm, 0.9, 1.0, cv2.THRESH_BINARY)
        ridge_mask = (ridge_mask * 255).astype(np.uint8)
        
        # Draw the ridge contours in Green
        contours, _ = cv2.findContours(ridge_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(final_img, contours, -1, (0, 255, 0), 2)
        
        # Find target point
        lookahead_dist = 80 # Reduced from 100 to stay closer to robot
        search_radius = 40 # Reduced from 50
        
        proj_x = robot_pos[0] + lookahead_dist * math.cos(robot_angle)
        proj_y = robot_pos[1] + lookahead_dist * math.sin(robot_angle)
        
        mask = np.zeros_like(gray)
        cv2.circle(mask, (int(proj_x), int(proj_y)), search_radius, 255, -1)
        
        # Apply search mask to the ALREADY MASKED distance map
        masked_dist = dist_map.copy()
        masked_dist[mask == 0] = 0
        
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(masked_dist)
        
        if max_val > 0:
            target_point = max_loc
        else:
            # If we can't find a point, STOP.
            # Setting target to robot pos will result in 0 distance -> Stop in Pure Pursuit
            target_point = robot_pos

        # Draw overlays on final image
        cv2.circle(final_img, robot_pos, 8, (255, 255, 255), -1) 
        cv2.circle(final_img, robot_pos, 6, (0, 0, 255), -1) 
        cv2.circle(final_img, target_point, 8, (0, 0, 0), -1) 
        cv2.circle(final_img, target_point, 6, (0, 255, 255), -1) 
        cv2.circle(final_img, seed_point, 5, (255, 0, 0), -1) # Seed Point (Blue)
        cv2.line(final_img, robot_pos, target_point, (255, 0, 255), 2)
        
        # Optimize Debug View: Only update every 5th frame to reduce lag
        self.frame_count += 1
        if self.frame_count % 5 == 0:
            # Create Composite Debug Image (2x2 Grid)
            # Resize all to be same size (half of original)
            h, w = cv_image.shape[:2]
            half_h, half_w = h // 2, w // 2
            
            # 1. Top-Left: Threshold (What the robot sees as lines)
            thresh_bgr = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
            cv2.putText(thresh_bgr, "1. Threshold (Lines=0)", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
            # 2. Top-Right: Lane Mask (Floodfill result)
            mask_bgr = cv2.cvtColor(lane_mask, cv2.COLOR_GRAY2BGR)
            cv2.putText(mask_bgr, "2. Lane Mask (Floodfill)", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # 3. Bottom-Left: Distance Heatmap
            cv2.putText(heatmap, "3. Distance Heatmap", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            
            # 4. Bottom-Right: Final Result
            cv2.putText(final_img, "4. Final Result", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            
            # Combine
            top_row = np.hstack((cv2.resize(thresh_bgr, (half_w, half_h)), cv2.resize(mask_bgr, (half_w, half_h))))
            bot_row = np.hstack((cv2.resize(heatmap, (half_w, half_h)), cv2.resize(final_img, (half_w, half_h))))
            debug_img = np.vstack((top_row, bot_row))

            # Publish debug image
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_img, "bgr8")
                self.debug_pub.publish(debug_msg)
            except Exception as e:
                self.get_logger().error(f'Debug image publish error: {e}')
                
            # Show window
            try:
                cv2.imshow("Middle Path Follower", debug_img)
                cv2.waitKey(1)
            except Exception:
                pass

        # 3. Control
        dx = target_point[0] - robot_pos[0]
        dy = target_point[1] - robot_pos[1]
        target_angle = math.atan2(dy, dx)
        
        error = target_angle - robot_angle
        error = (error + math.pi) % (2 * math.pi) - math.pi
        
        # twist = Twist()
        # twist.linear.x = 0.0 # Stop movement as requested
        # twist.angular.z = 0.0 # Stop rotation too just to be safe/static 
        
        # self.publisher.publish(twist)
        
        # 4. Publish Waypoint (World Coordinates)
        if self.scale is not None:
            # Convert pixel to world
            # Image Y -> World X (Forward)
            # Image X -> World Y (Left)
            
            # Robot World Pos (Relative to camera center, but we want relative to Robot Frame for control?)
            # Actually, let's publish the target point in the ROBOT'S FRAME (Base Link).
            # We know the robot's pixel position and orientation.
            
            # Pixel difference
            dx_pixel = target_point[0] - robot_pos[0] # Image X difference
            dy_pixel = target_point[1] - robot_pos[1] # Image Y difference
            
            # Rotate to align with robot heading
            # Robot heading in image is 'robot_angle'
            # We want to rotate the vector (dx_pixel, dy_pixel) by -robot_angle
            
            # Vector in image frame (X=Right, Y=Down)
            # Wait, standard image: X is Right, Y is Down.
            # Robot Angle is calculated as atan2(dy, dx) in image frame.
            
            # Let's project into a frame where X is Robot Forward, Y is Robot Left.
            
            # Vector from Robot to Target in Image Frame
            v_x = target_point[0] - robot_pos[0]
            v_y = target_point[1] - robot_pos[1]
            
            # Rotate this vector by -robot_angle to align X with robot heading? 
            # No, robot_angle is the angle of the robot in the image.
            # If we rotate the coordinate system so the robot points "Right" (0 angle), 
            # then the target's relative angle is (target_angle - robot_angle).
            
            dist_pixels = math.sqrt(v_x*v_x + v_y*v_y)
            dist_meters = dist_pixels * self.scale
            
            # Angle of target relative to robot
            target_angle_image = math.atan2(v_y, v_x)
            relative_angle = target_angle_image - robot_angle
            
            # Normalize angle
            relative_angle = (relative_angle + math.pi) % (2 * math.pi) - math.pi
            
            # FIX: Negate relative angle because Image Frame (Y Down) vs ROS Frame (Y Left)
            # Positive relative_angle in Image (CW) means Right Turn.
            # Positive angle in ROS (CCW) means Left Turn.
            relative_angle = -relative_angle
            
            # In Robot Frame:
            # X is forward (distance * cos(relative_angle))
            # Y is left (distance * sin(relative_angle))
            # Note: In image, Y is down (increasing). In standard math, Y is up.
            # But we calculated angles using standard atan2(y, x) on image coordinates.
            # So the relative angle should be correct for "deviation from heading".
            # But we need to check the sign.
            # If target is "above" robot in image (smaller Y), and robot points "up" (smaller Y),
            # v_y is negative. robot_angle is -pi/2.
            # target_angle is -pi/2. relative is 0. Correct.
            
            # If target is to the "left" of robot (smaller X), and robot points "up".
            # Robot angle -pi/2. Target is (-10, -10). Angle -3pi/4.
            # Relative = -3pi/4 - (-pi/2) = -pi/4.
            # In ROS, Y is Left. -pi/4 is Right.
            # So we might need to flip Y?
            # Let's stick to: X = Forward, Y = Left.
            
            waypoint_x = dist_meters * math.cos(relative_angle)
            waypoint_y = dist_meters * math.sin(relative_angle)
            
            # Calculate Orientation (Yaw) of the path
            # We used relative_angle as the direction TO the waypoint.
            # But we also want the orientation OF the path at that waypoint?
            # Or just the orientation of the vector pointing to it?
            # The user wants "orientation as well". 
            # Usually for Stanley, we need the path heading.
            # Let's approximate path heading as the angle of the vector from robot to target.
            # In Robot Frame, this is exactly 'relative_angle'.
            
            # Wait, Stanley needs the heading of the path *at the nearest point*.
            # Here we are giving a lookahead point.
            # If we assume the path is a straight line from robot to target, the heading is relative_angle.
            # Let's use that for now.
            
            path_yaw = relative_angle
            q = quaternion_from_euler(0, 0, path_yaw)
            
            # Publish
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
            # Heartbeat log to prove it's working
            self.get_logger().info(f'Pub Waypoint: x={waypoint_x:.2f}, y={waypoint_y:.2f}', throttle_duration_sec=2.0)

        # 5. Publish Nearest Point (for Stanley / P-Control)
        if self.scale is not None:
            # Find nearest point on the ridge mask
            # We can use the distance transform 'dist_map' or 'ridge_mask'
            # Let's use ridge_mask to find all points, then find the one closest to robot
            
            # Get all points in ridge_mask
            pts = cv2.findNonZero(ridge_mask)
            
            if pts is not None:
                # pts is (N, 1, 2) array of (x, y)
                pts = pts.squeeze() # (N, 2)
                
                if pts.ndim == 1:
                    pts = pts.reshape(1, 2)
                
                # Robot pos
                rx, ry = robot_pos
                
                # Calculate distances
                dists = np.sqrt((pts[:, 0] - rx)**2 + (pts[:, 1] - ry)**2)
                
                # Find index of min distance
                min_idx = np.argmin(dists)
                nearest_pt = pts[min_idx] # (x, y)
                
                # Calculate Heading at this point
                # We need a point slightly ahead to estimate tangent
                # Let's look for a point on the ridge that is further away?
                # Or just use the gradient of the distance map?
                # Or fit a line to local points?
                
                # Simple approach: Find a point 'delta' pixels away from nearest_pt along the ridge
                # that is ALSO further from the robot (to ensure forward direction)
                
                # Filter points that are further from robot than nearest_pt + buffer
                # dists > dists[min_idx] + 10
                
                future_mask = dists > (dists[min_idx] + 10)
                future_pts = pts[future_mask]
                
                path_heading = 0.0
                
                if len(future_pts) > 0:
                    # Filter for points that are "in front" of the robot
                    # Vector from Robot to Point
                    v_x = future_pts[:, 0] - rx
                    v_y = future_pts[:, 1] - ry
                    
                    # Robot Heading Vector (in Image Frame)
                    # robot_angle is angle of robot in image.
                    # Heading vector: (cos(robot_angle), sin(robot_angle))
                    h_x = math.cos(robot_angle)
                    h_y = math.sin(robot_angle)
                    
                    # Dot product
                    dot_prod = v_x * h_x + v_y * h_y
                    
                    # Keep only points with positive dot product (in front)
                    valid_indices = dot_prod > 0
                    valid_future_pts = future_pts[valid_indices]
                    
                    if len(valid_future_pts) > 0:
                        # Find the one closest to the nearest_pt among the VALID future points
                        dists_from_nearest = np.sqrt((valid_future_pts[:, 0] - nearest_pt[0])**2 + (valid_future_pts[:, 1] - nearest_pt[1])**2)
                        idx_next = np.argmin(dists_from_nearest)
                        next_pt = valid_future_pts[idx_next]
                        
                        # Vector from nearest to next
                        vx = next_pt[0] - nearest_pt[0]
                        vy = next_pt[1] - nearest_pt[1]
                        
                        # Angle in Image Frame
                        path_heading_img = math.atan2(vy, vx)
                        
                        # Convert to Robot Frame Angle
                        # Image: Y Down. Robot: Y Left.
                        # Relative angle logic same as before?
                        # We want the angle of the path vector relative to the robot's heading.
                        
                        # Robot heading in image is 'robot_angle'
                        # Path heading in image is 'path_heading_img'
                        
                        # Relative heading = path - robot
                        heading_diff = path_heading_img - robot_angle
                        
                        # Normalize
                        heading_diff = (heading_diff + math.pi) % (2 * math.pi) - math.pi
                        
                        # Negate for ROS frame (Right turn in image is + angle, but - in ROS)
                        path_heading = -heading_diff
                    
                # Calculate Position in Robot Frame
                # Vector from Robot to Nearest Point
                dx_pixel = nearest_pt[0] - robot_pos[0]
                dy_pixel = nearest_pt[1] - robot_pos[1]
                
                dist_pixels = math.sqrt(dx_pixel**2 + dy_pixel**2)
                dist_meters = dist_pixels * self.scale
                
                # Angle to nearest point
                angle_to_pt_img = math.atan2(dy_pixel, dx_pixel)
                rel_angle_pt = angle_to_pt_img - robot_angle
                rel_angle_pt = (rel_angle_pt + math.pi) % (2 * math.pi) - math.pi
                rel_angle_pt = -rel_angle_pt # Convert to ROS frame
                
                near_x = dist_meters * math.cos(rel_angle_pt)
                near_y = dist_meters * math.sin(rel_angle_pt)
                
                # Publish
                near_msg = PoseStamped()
                near_msg.header.stamp = self.get_clock().now().to_msg()
                near_msg.header.frame_id = 'base_link'
                
                near_msg.pose.position.x = near_x
                near_msg.pose.position.y = near_y
                near_msg.pose.position.z = 0.0
                
                # Orientation (Path Heading)
                q_near = quaternion_from_euler(0, 0, path_heading)
                near_msg.pose.orientation.x = q_near[0]
                near_msg.pose.orientation.y = q_near[1]
                near_msg.pose.orientation.z = q_near[2]
                near_msg.pose.orientation.w = q_near[3]
                
                self.nearest_pub.publish(near_msg)
                # self.get_logger().info(f'Pub Nearest: x={near_x:.2f}, y={near_y:.2f}, head={path_heading:.2f}', throttle_duration_sec=2.0)

def main(args=None):
    rclpy.init(args=args)
    node = MiddlePathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

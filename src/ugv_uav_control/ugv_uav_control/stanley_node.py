import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
# from tf_transformations import euler_from_quaternion
import math

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion[0]
    y = quaternion[1]
    z = quaternion[2]
    w = quaternion[3]

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp) # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class StanleyNode(Node):
    def __init__(self):
        super().__init__('stanley_node')
        
        self.subscription = self.create_subscription(
            PoseStamped,
            '/waypoints',
            self.waypoint_callback,
            10)
            
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Control Parameters
        self.k_cte = 2.0 # Cross-track error gain
        self.k_soft = 1.0 # Softening constant to prevent singularity at low speeds
        self.max_linear_speed = 0.22
        self.stop_distance = 0.1
        
        self.get_logger().info('Stanley Node Started')

    def waypoint_callback(self, msg):
        # Waypoint is in Robot Frame: X=Forward, Y=Left
        # Robot state in this frame: x=0, y=0, yaw=0
        
        x_target = msg.pose.position.x
        y_target = msg.pose.position.y
        
        # Extract target orientation (yaw)
        q = msg.pose.orientation
        _, _, target_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        distance = math.sqrt(x_target*x_target + y_target*y_target)
        
        twist = Twist()
        
        if distance > self.stop_distance:
            # Stanley Control Law
            # delta = psi_e + atan2(k * cte, v)
            
            # 1. Heading Error (psi_e)
            # psi_e = psi_path - psi_robot
            # psi_robot = 0
            # psi_path = target_yaw
            psi_e = target_yaw
            
            # Normalize angle
            psi_e = (psi_e + math.pi) % (2 * math.pi) - math.pi
            
            # 2. Cross-Track Error (cte)
            # Distance from robot (0,0) to the path line defined by (x_target, y_target) and target_yaw.
            # Line equation: (Y - y_target) = tan(target_yaw) * (X - x_target)
            # -sin(target_yaw)*X + cos(target_yaw)*Y + [sin(target_yaw)*x_target - cos(target_yaw)*y_target] = 0
            # Distance from (0,0) to ax + by + c = 0 is |c| / sqrt(a^2 + b^2)
            # Here a = -sin(target_yaw), b = cos(target_yaw), sqrt(a^2+b^2) = 1
            # c = sin(target_yaw)*x_target - cos(target_yaw)*y_target
            # cte = c
            
            # However, we need the SIGN of cte.
            # If we look at the vector from target to robot: (-x_target, -y_target)
            # And project it onto the normal vector (-sin, cos)?
            
            # Simpler approach:
            # Transform robot position (0,0) into the path frame (origin at target, aligned with target_yaw).
            # Then y-coordinate is the CTE.
            
            # Robot pos in target frame:
            # dx = 0 - x_target = -x_target
            # dy = 0 - y_target = -y_target
            # Rotate by -target_yaw
            # y_local = -x_target * sin(-target_yaw) + -y_target * cos(-target_yaw)
            # y_local = x_target * sin(target_yaw) - y_target * cos(target_yaw)
            
            # Wait, standard definition:
            # CTE is positive if robot is to the left of path?
            # Or depends on convention.
            # Let's stick to: we want to minimize y_local.
            # If y_local is positive, we are to the left, we should steer right (negative).
            # Formula: delta = psi_e + atan2(k * cte, v)
            # If cte > 0 (left), we want delta < 0?
            # atan2(positive) is positive. So we might need -atan2?
            # Or define cte such that positive means "error to the right"?
            
            # Let's use the formula: delta = psi_e + arctan(k * e / v)
            # Usually e is "lateral error".
            # If robot is to the left of path, e is positive. We want to steer right (negative).
            # So term should be -arctan? Or e should be defined differently.
            
            # Let's calculate e as "y coordinate of robot in path frame".
            # If robot is at (0, y_error) relative to path, e = y_error.
            # If e > 0, robot is left. We want to turn right.
            # So we need negative steering.
            # So delta = psi_e - arctan(k * e / v) ?
            
            # Let's check psi_e term.
            # If robot is heading 0, path is heading 0.1 (left). psi_e = 0.1.
            # We want to turn left (positive). So +psi_e is correct.
            
            # So: delta = psi_e + arctan(k * (-e) / v) ?
            # Which is delta = psi_e - arctan(k * e / v).
            
            # Let's calculate e:
            # e = distance from line.
            # e = y_target * cos(target_yaw) - x_target * sin(target_yaw)
            # (This is derived from cross product or rotation)
            # Let's verify:
            # Target at (10, 0), yaw=0. Robot at (0,0).
            # e = 0*1 - 10*0 = 0. Correct.
            # Target at (10, 1), yaw=0. Robot at (0,0).
            # Path is line y=1. Robot is at y=0.
            # Robot is "right" of path (error -1).
            # e = 1*1 - 10*0 = 1.
            # If e=1 (positive), and robot is right of path, we want to steer left (positive).
            # So +arctan(k*e/v) works!
            
            # So if e is defined as "lateral distance of target from robot's heading line"?
            # No, Stanley defines e as "cross track error of front axle".
            
            # Let's stick to:
            # e = y_target * cos(target_yaw) - x_target * sin(target_yaw)
            # Wait, if target is at (10, 1) and yaw=0.
            # Path line passes through (10,1) with angle 0. It's line y=1.
            # Robot at (0,0).
            # Robot is to the right of the path.
            # We want to steer Left (positive).
            # Formula: delta = psi_e + atan(k*e/v).
            # psi_e = 0.
            # We need atan(...) to be positive.
            # So e should be positive.
            # With e = 1*1 - 10*0 = 1. It is positive.
            # So this formula for e seems consistent with +atan.
            
            cte = y_target * math.cos(target_yaw) - x_target * math.sin(target_yaw)
            
            # Linear speed
            v = self.max_linear_speed
            
            # Steering angle
            # delta = psi_e + atan2(k * cte, v)
            # Note: using atan2 to handle signs correctly, but usually atan is used.
            # atan2(y, x). Here we have atan(val).
            
            delta = psi_e + math.atan2(self.k_cte * cte, self.k_soft + v)
            
            # Clamp steering angle
            max_steer = 1.0
            delta = max(min(delta, max_steer), -max_steer)
            
            # Convert steering angle to angular velocity
            # omega = (v / L) * tan(delta) ?
            # For differential drive, we can just set omega directly?
            # Or use a virtual wheelbase L.
            # Let's assume we control omega directly.
            # For small angles, omega ~ v * delta / L?
            # Or just omega = delta * Gain?
            # Let's use a gain.
            
            # Actually, for diff drive, we can just output v and omega.
            # If we treat it as a bicycle model:
            # omega = v * tan(delta) / L
            L = 0.2 # Virtual wheelbase
            angular_speed = v * math.tan(delta) / L
            
            # Clamp angular speed
            max_ang = 1.5
            angular_speed = max(min(angular_speed, max_ang), -max_ang)
            
            twist.linear.x = v
            twist.angular.z = angular_speed
            
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = StanleyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

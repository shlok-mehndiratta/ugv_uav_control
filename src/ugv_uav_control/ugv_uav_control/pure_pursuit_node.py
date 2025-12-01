import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, PoseStamped
import math

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        
        self.subscription = self.create_subscription(
            PoseStamped,
            '/waypoints',
            self.waypoint_callback,
            10)
            
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Control Parameters
        self.max_linear_speed = 0.22
        self.stop_distance = 0.1
        self.lookahead_distance_threshold = 0.5 # If waypoint is too far, maybe limit speed?
        
        self.get_logger().info('Pure Pursuit Node Started')

    def waypoint_callback(self, msg):
        # Waypoint is in Robot Frame: X=Forward, Y=Left
        x = msg.pose.position.x
        y = msg.pose.position.y
        
        distance = math.sqrt(x*x + y*y)
        
        twist = Twist()
        
        if distance > self.stop_distance:
            # Pure Pursuit Logic
            # Curvature gamma = 2y / L^2
            # Angular velocity omega = v * gamma
            
            # 1. Calculate Curvature
            # Avoid division by zero
            if distance < 0.01:
                curvature = 0.0
            else:
                curvature = (2.0 * y) / (distance * distance)
            
            # 2. Set Linear Speed
            # Simple proportional control for speed or constant speed
            # Let's use constant speed for now, or slow down for sharp turns
            target_speed = self.max_linear_speed
            
            # Optional: Slow down if curvature is high
            if abs(curvature) > 1.0:
                target_speed *= 0.5
            
            # 3. Calculate Angular Speed
            angular_speed = target_speed * curvature
            
            # Clamp angular speed
            max_ang = 1.5
            angular_speed = max(min(angular_speed, max_ang), -max_ang)
            
            twist.linear.x = target_speed
            twist.angular.z = angular_speed
            
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

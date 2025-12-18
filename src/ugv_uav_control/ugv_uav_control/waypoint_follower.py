import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, PoseStamped
import math

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        
        self.subscription = self.create_subscription(
            PoseStamped,
            '/waypoints',
            self.waypoint_callback,
            10)
            
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Control Parameters
        self.k_linear = 0.5
        self.k_angular = 2.0
        self.max_linear_speed = 0.22
        self.max_angular_speed = 1.0
        self.stop_distance = 0.05
        
        self.get_logger().info('Waypoint Follower Node Started')

    def waypoint_callback(self, msg):
        # Waypoint is in Robot Frame: X=Forward, Y=Left
        x = msg.pose.position.x
        y = msg.pose.position.y
        
        distance = math.sqrt(x*x + y*y)
        angle = math.atan2(y, x)
        
        twist = Twist()
        
        if distance > self.stop_distance:
            # Linear Control
            linear_speed = self.k_linear * distance
            linear_speed = min(linear_speed, self.max_linear_speed)
            
            # Angular Control
            angular_speed = self.k_angular * angle
            angular_speed = max(min(angular_speed, self.max_angular_speed), -self.max_angular_speed)
            
            # Slow down if turning sharply
            if abs(angle) > 0.5:
                linear_speed *= 0.5
            
            twist.linear.x = linear_speed
            twist.angular.z = angular_speed
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

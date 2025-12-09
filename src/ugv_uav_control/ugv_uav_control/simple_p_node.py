import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
import math

class SimplePNode(Node):
    def __init__(self):
        super().__init__('simple_p_node')
        
        self.subscription = self.create_subscription(
            PoseStamped,
            '/nearest_point',
            self.waypoint_callback,
            10)
            
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Control Parameters
        # High gain for "wonky" behavior
        self.kp = 15.0 
        self.linear_speed = 0.52 # Constant speed
        self.stop_distance = 0.1
        
        self.get_logger().info('Simple P Controller Node Started')

    def waypoint_callback(self, msg):
        # Waypoint is in Robot Frame: X=Forward, Y=Left
        x = msg.pose.position.x
        y = msg.pose.position.y
        
        distance = math.sqrt(x*x + y*y)
        
        # Angle to the waypoint
        error_angle = math.atan2(y, x)
        
        twist = Twist()
        
        if distance > self.stop_distance:
            # Constant Linear Speed (no slowdown for turns -> more error)
            twist.linear.x = self.linear_speed
            
            # Proportional Angular Control
            twist.angular.z = self.kp * error_angle
            
            # Clamp angular speed
            max_ang = 3.0
            twist.angular.z = max(min(twist.angular.z, max_ang), -max_ang)
            
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = SimplePNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

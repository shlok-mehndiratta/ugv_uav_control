import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from scipy.optimize import minimize
import numpy as np
import math
class MPCNode(Node):
    def __init__(self):
        super().__init__('mpc_node')
        
        # Subscriptions & Publishers
        self.create_subscription(PoseStamped, '/waypoints', self.waypoint_callback, 10)
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
        # MPC Parameters
        self.horizon = 5         # Number of steps to look ahead
        self.dt = 0.2           # Time step duration (seconds)
        self.max_v = 0.22       # Max linear speed (Turtlebot limit)
        self.max_w = 2.0        # Max angular speed
        
        self.get_logger().info('MPC Node Started (SciPy Optimized)')
    def kinematic_model(self, state, v, w, dt):
        """
        Unicycle Kinematic Model:
        x_next = x + v*cos(theta)*dt
        y_next = y + v*sin(theta)*dt
        theta_next = theta + w*dt
        """
        x, y, theta = state
        x_new = x + v * np.cos(theta) * dt
        y_new = y + v * np.sin(theta) * dt
        theta_new = theta + w * dt
        return [x_new, y_new, theta_new]
    def cost_function(self, u_flat, *args):
        """
        Cost Function to Minimize:
        J = sum( w_dist * distance_error^2 + w_ctrl * control_effort )
        """
        target_x, target_y = args
        
        # Reshape inputs: [v0, w0, v1, w1, ...]
        u = u_flat.reshape((self.horizon, 2))
        
        cost = 0.0
        # Robot starts at Local Origin (0,0,0) because waypoints are transformed to base_link
        state = [0.0, 0.0, 0.0] 
        
        for k in range(self.horizon):
            v = u[k, 0]
            w = u[k, 1]
            
            # Update State
            state = self.kinematic_model(state, v, w, self.dt)
            
            # Distance to Target
            dist_sq = (state[0] - target_x)**2 + (state[1] - target_y)**2
            
            # Costs
            cost += 10.0 * dist_sq          # Reach the target
            cost += 0.1 * (v - 0.22)**2     # Maintain speed
            cost += 0.01 * w**2             # Minimize turning (smoothness)
            
        return cost
    def waypoint_callback(self, msg):
        # Target in Robot Frame
        tx = msg.pose.position.x
        ty = msg.pose.position.y
        
        # Initial Guess (Constant forward max speed, zero turn)
        u0 = np.array([self.max_v, 0.0] * self.horizon)
        
        # Bounds for [v, w]
        bounds = [[0.0, self.max_v], [-self.max_w, self.max_w]] * self.horizon
        
        # Optimize
        result = minimize(
            self.cost_function,
            u0,
            args=(tx, ty),
            method='SLSQP',
            bounds=bounds,
            options={'ftol': 1e-3, 'disp': False}
        )
        
        # Extract optimal control for FIRST step
        u_opt = result.x.reshape((self.horizon, 2))
        v_cmd = u_opt[0, 0]
        w_cmd = u_opt[0, 1]
        
        # Publish
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = 'base_link'
        twist_msg.twist.linear.x = float(v_cmd)
        twist_msg.twist.angular.z = float(w_cmd)
        
        self.publisher.publish(twist_msg)
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MPCNode())
    rclpy.shutdown()
if __name__ == '__main__':
    main()
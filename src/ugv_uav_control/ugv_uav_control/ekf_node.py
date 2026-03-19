import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import numpy as np
import math
def wrap_angle(angle):
    """Normalize angle to [-π, π]."""
    return (angle + np.pi) % (2 * np.pi) - np.pi
    
def quaternion_from_euler(roll, pitch, yaw):
    """Convert Euler angles to quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return [
        cy * cp * sr - sy * sp * cr,  # x
        sy * cp * sr + cy * sp * cr,  # y
        sy * cp * cr - cy * sp * sr,  # z
        cy * cp * cr + sy * sp * sr   # w
    ]

def euler_from_quaternion(q):
    """Extract yaw from quaternion [x, y, z, w]."""
    siny_cosp = 2 * (q[3] * q[2] + q[0] * q[1])
    cosy_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
    return math.atan2(siny_cosp, cosy_cosp)

class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_node')
        
        # === STATE ===
        # [x, y, θ, v, ω]
        self.mu = np.zeros(5)
        self.Sigma = np.diag([0.1, 0.1, 0.1, 0.01, 0.01])
        
        # === PARAMETERS ===
        self.dt = 0.05  # 20 Hz prediction rate
        self.alpha_v = 2.0   # 1/τ_v (τ_v = 0.5s)
        self.alpha_w = 5.0   # 1/τ_w (τ_w = 0.2s)
        
        # Process Noise Q
        self.Q = np.diag([1e-4, 1e-4, 4e-4, 1e-4, 2.5e-3])
        
        # Measurement Noise
        self.R_full = np.diag([0.0025, 0.0025, 0.01, 0.0004, 0.0025])  # ArUco + Odom
        self.R_odom = np.diag([0.0004, 0.0025])  # Odom only
        
        # Control cache
        self.last_cmd = np.array([0.0, 0.0])
        self.last_aruco_time = self.get_clock().now()
        self.aruco_timeout = 2.0  # seconds
        
        # === SUBSCRIBERS ===
        self.create_subscription(TwistStamped, '/cmd_vel', self.cmd_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, '/aruco_pose', self.aruco_callback, 10)
        
        # === PUBLISHER ===
        self.belief_pub = self.create_publisher(PoseWithCovarianceStamped, '/belief_state', 10)
        
        # === TIMER for prediction ===
        self.create_timer(self.dt, self.prediction_step)
        
        self.get_logger().info('EKF Node Started (5-state: x, y, θ, v, ω)')
    
    # =========================================
    # PROCESS MODEL
    # =========================================
    def process_model(self, state, u, dt):
        """
        Propagate state: x_{k+1} = f(x_k, u_k)
        Uses simplified Euler integration.
        """
        x, y, theta, v, omega = state
        v_cmd, omega_cmd = u
        
        # Actuator dynamics
        v_next = v + self.alpha_v * (v_cmd - v) * dt
        omega_next = omega + self.alpha_w * (omega_cmd - omega) * dt
        
        # Average velocities for integration
        v_avg = (v + v_next) / 2.0
        omega_avg = (omega + omega_next) / 2.0
        
        # Pose kinematics
        x_next = x + v_avg * np.cos(theta) * dt
        y_next = y + v_avg * np.sin(theta) * dt
        theta_next = wrap_angle(theta + omega_avg * dt)
        
        return np.array([x_next, y_next, theta_next, v_next, omega_next])
    
    def compute_jacobian_F(self, state, u, dt):
        """
        Compute Jacobian of process model w.r.t. state.
        """
        x, y, theta, v, omega = state
        
        F = np.eye(5)
        
        # ∂x'/∂θ = -v * sin(θ) * dt
        F[0, 2] = -v * np.sin(theta) * dt
        # ∂x'/∂v = cos(θ) * dt
        F[0, 3] = np.cos(theta) * dt
        
        # ∂y'/∂θ = v * cos(θ) * dt
        F[1, 2] = v * np.cos(theta) * dt
        # ∂y'/∂v = sin(θ) * dt
        F[1, 3] = np.sin(theta) * dt
        
        # ∂θ'/∂ω = dt
        F[2, 4] = dt
        
        # ∂v'/∂v = 1 - α_v * dt
        F[3, 3] = 1 - self.alpha_v * dt
        
        # ∂ω'/∂ω = 1 - α_ω * dt
        F[4, 4] = 1 - self.alpha_w * dt
        
        return F
    
    # =========================================
    # EKF STEPS
    # =========================================
    def prediction_step(self):
        """
        Timer callback: Predict belief forward using last control.
        """
        # Predict mean
        self.mu = self.process_model(self.mu, self.last_cmd, self.dt)
        
        # Predict covariance
        F = self.compute_jacobian_F(self.mu, self.last_cmd, self.dt)
        self.Sigma = F @ self.Sigma @ F.T + self.Q
        
        # Publish belief
        self.publish_belief()
    
    def update_step(self, z, H, R):
        """
        Generic EKF update step.
        z: measurement vector
        H: measurement Jacobian
        R: measurement noise covariance
        """
        # Innovation
        z_pred = H @ self.mu
        y = z - z_pred
        
        # Handle angle wrapping for θ (index 2)
        if H.shape[0] >= 3 and H[2, 2] == 1.0:
            y[2] = wrap_angle(y[2])
        
        # Innovation covariance
        S = H @ self.Sigma @ H.T + R
        
        # Kalman gain
        K = self.Sigma @ H.T @ np.linalg.inv(S)
        
        # State update
        self.mu = self.mu + K @ y
        self.mu[2] = wrap_angle(self.mu[2])  # Normalize θ
        
        # Covariance update (Joseph form for numerical stability)
        I_KH = np.eye(5) - K @ H
        self.Sigma = I_KH @ self.Sigma @ I_KH.T + K @ R @ K.T
        
        # Ensure symmetry
        self.Sigma = (self.Sigma + self.Sigma.T) / 2.0
    
    # =========================================
    # CALLBACKS
    # =========================================
    def cmd_callback(self, msg):
        """Store latest control command."""
        self.last_cmd = np.array([
            msg.twist.linear.x,
            msg.twist.angular.z
        ])
    
    def odom_callback(self, msg):
        """
        Update with odometry measurement (v, ω).
        """
        z = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.angular.z
        ])
        
        H = np.array([
            [0, 0, 0, 1, 0],
            [0, 0, 0, 0, 1]
        ], dtype=float)
        
        self.update_step(z, H, self.R_odom)
    
    def aruco_callback(self, msg):
        """
        Update with ArUco pose measurement (x, y, θ).
        Full 5-state update: pose from ArUco, velocity from current estimate.
        """
        x_aruco = msg.pose.position.x
        y_aruco = msg.pose.position.y
        q = msg.pose.orientation
        theta_aruco = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        # Full measurement: [x, y, θ, v, ω]
        # We measure pose directly; velocity inherited from prediction
        z = np.array([x_aruco, y_aruco, theta_aruco, self.mu[3], self.mu[4]])
        
        H = np.eye(5)
        
        self.update_step(z, H, self.R_full)
        
        self.last_aruco_time = self.get_clock().now()
        self.get_logger().debug(f'ArUco update: x={x_aruco:.2f}, y={y_aruco:.2f}, θ={theta_aruco:.2f}')
    
    # =========================================
    # PUBLISHING
    # =========================================
    def publish_belief(self):
        """
        Publish belief as PoseWithCovarianceStamped.
        """
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        
        # Pose (x, y, θ)
        msg.pose.pose.position.x = float(self.mu[0])
        msg.pose.pose.position.y = float(self.mu[1])
        msg.pose.pose.position.z = 0.0
        
        q = quaternion_from_euler(0, 0, self.mu[2])
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        
        # Covariance: 6x6 matrix (x, y, z, roll, pitch, yaw)
        # We populate the relevant entries from our 5x5 Sigma
        cov = np.zeros((6, 6))
        cov[0, 0] = self.Sigma[0, 0]  # x-x
        cov[0, 1] = self.Sigma[0, 1]  # x-y
        cov[1, 0] = self.Sigma[1, 0]  # y-x
        cov[1, 1] = self.Sigma[1, 1]  # y-y
        cov[5, 5] = self.Sigma[2, 2]  # yaw-yaw
        cov[0, 5] = self.Sigma[0, 2]  # x-yaw
        cov[5, 0] = self.Sigma[2, 0]  # yaw-x
        cov[1, 5] = self.Sigma[1, 2]  # y-yaw
        cov[5, 1] = self.Sigma[2, 1]  # yaw-y
        
        msg.pose.covariance = cov.flatten().tolist()
        
        self.belief_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()
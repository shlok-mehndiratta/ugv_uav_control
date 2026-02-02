import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped, PoseWithCovarianceStamped
from scipy.optimize import minimize
import numpy as np
import math
def wrap_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi
class BeliefMPCNode(Node):
    def __init__(self):
        super().__init__('belief_mpc_node')
        
        # === SUBSCRIPTIONS ===
        self.create_subscription(PoseStamped, '/waypoints', self.waypoint_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/belief_state', self.belief_callback, 10)
        
        # === PUBLISHER ===
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
        # === MPC PARAMETERS ===
        self.horizon = 5
        self.dt = 0.2
        self.max_v = 0.22
        self.max_w = 2.0
        
        # Actuator dynamics
        self.alpha_v = 2.0
        self.alpha_w = 5.0
        
        # Process noise (same as EKF)
        self.Q = np.diag([1e-4, 1e-4, 4e-4, 1e-4, 2.5e-3])
        
        # === BELIEF STATE (from EKF) ===
        self.mu = np.zeros(5)  # [x, y, θ, v, ω]
        self.Sigma = np.eye(5) * 0.1
        self.world_pos = np.zeros(3)  # [x, y, theta] in world frame from EKF
        
        # === COST WEIGHTS ===
        self.w_dist = 10.0       # Distance to target
        self.w_head = 2.0        # Heading error
        self.w_rate = 5.0        # Control rate (smoothness)
        self.w_uncertainty = 1.0 # λ: Uncertainty penalty weight
        
        self.prev_ctrl = np.array([0.0, 0.0])
        
        self.get_logger().info('Belief-Space MPC Node Started (λ = {:.2f})'.format(self.w_uncertainty))

        # UAV parameters (static for now)
        self.uav_x = 0.0
        self.uav_y = 0.0
        self.uav_z = 10.0
        self.fov_angle = 1.047  # 60 degrees

        # Compute FoV radius
        self.fov_radius = self.uav_z * np.tan(self.fov_angle / 2)  # ~5.77 m

        # Observability smoothing parameter
        self.gamma_k = 5.0  # Sigmoid sharpness

        # Observation matrices (for when visible)
        self.H_obs = np.eye(5)
        self.R_obs = np.diag([0.0025, 0.0025, 0.01, 0.0004, 0.0025])
        self.get_logger().info(f'FoV radius: {self.fov_radius:.2f} m')

    
    # =========================================
    # PROCESS MODEL (same as existing MPC)
    # =========================================
    def process_model(self, state, u, dt):
        x, y, theta, v, omega = state
        v_cmd, omega_cmd = u
        
        v_next = v + self.alpha_v * (v_cmd - v) * dt
        omega_next = omega + self.alpha_w * (omega_cmd - omega) * dt
        
        v_avg = (v + v_next) / 2.0
        omega_avg = (omega + omega_next) / 2.0
        
        if abs(omega_avg) < 1e-4:
            x_next = x + v_avg * np.cos(theta) * dt
            y_next = y + v_avg * np.sin(theta) * dt
        else:
            x_next = x + (v_avg / omega_avg) * (np.sin(theta + omega_avg * dt) - np.sin(theta))
            y_next = y - (v_avg / omega_avg) * (np.cos(theta + omega_avg * dt) - np.cos(theta))
        
        theta_next = wrap_angle(theta + omega_avg * dt)
        
        return np.array([x_next, y_next, theta_next, v_next, omega_next])
    
    def compute_jacobian_F(self, state, dt):
        """Compute Jacobian of process model."""
        x, y, theta, v, omega = state
        
        F = np.eye(5)
        F[0, 2] = -v * np.sin(theta) * dt
        F[0, 3] = np.cos(theta) * dt
        F[1, 2] = v * np.cos(theta) * dt
        F[1, 3] = np.sin(theta) * dt
        F[2, 4] = dt
        F[3, 3] = 1 - self.alpha_v * dt
        F[4, 4] = 1 - self.alpha_w * dt
        
        return F
    
    # =========================================
    # BELIEF-SPACE COST FUNCTION
    # =========================================
    def cost_function(self, u_flat, *args):
        target_x, target_y, start_mu, start_Sigma, start_v, start_w, world_pos = args
        u = u_flat.reshape((self.horizon, 2))
        
        cost = 0.0
        mu = start_mu.copy()
        Sigma = start_Sigma.copy()
        prev_u = np.array([start_v, start_w])
        
        for k in range(self.horizon):
            v_cmd = u[k, 0]
            w_cmd = u[k, 1]
            
            # Mean propagation
            mu = self.process_model(mu, [v_cmd, w_cmd], self.dt)
            
            # Covariance prediction (no measurement)
            F = self.compute_jacobian_F(mu, self.dt)
            Sigma_pred = F @ Sigma @ F.T + self.Q
            
            # ========================================
            # FoV-DEPENDENT COVARIANCE UPDATE (NEW)
            # ========================================
            # Current predicted position in WORLD frame
            world_x = world_pos[0] + mu[0] * np.cos(world_pos[2]) - mu[1] * np.sin(world_pos[2])
            world_y = world_pos[1] + mu[0] * np.sin(world_pos[2]) + mu[1] * np.cos(world_pos[2])
            world_mu = np.array([world_x, world_y, 0, 0, 0])  # Only need x, y for FoV
            
            gamma, d = self.compute_observability(world_mu)
            
            # Blend between no-observation and observation
            if gamma > 0.01:  # Avoid computing Kalman update when clearly outside
                Sigma_obs = self.kalman_update(Sigma_pred)
                Sigma = (1 - gamma) * Sigma_pred + gamma * Sigma_obs
            else:
                Sigma = Sigma_pred
            
            # ========================================
            # TASK COST
            # ========================================
            x, y, theta = mu[0], mu[1], mu[2]
            
            dist_sq = (x - target_x)**2 + (y - target_y)**2
            target_angle = math.atan2(target_y - y, target_x - x)
            angle_err = wrap_angle(theta - target_angle)
            w_head_k = self.w_head if dist_sq > 0.05 else 0.0
            rate_penalty = (v_cmd - prev_u[0])**2 + (w_cmd - prev_u[1])**2
            
            # ========================================
            # UNCERTAINTY + FoV COST
            # ========================================
            uncertainty_cost = np.trace(Sigma)
            
            # Optional: Add explicit FoV-exit penalty
            fov_penalty = max(0, d - self.fov_radius)**2  # Penalty for leaving FoV
            
            # ========================================
            # TOTAL COST
            # ========================================
            cost += self.w_dist * dist_sq
            cost += w_head_k * angle_err**2
            cost += self.w_rate * rate_penalty
            cost += self.w_uncertainty * uncertainty_cost
            cost += 2.0 * fov_penalty  # Additional FoV penalty
            
            prev_u = [v_cmd, w_cmd]
        
        return cost
    
    # =========================================
    # CALLBACKS
    # =========================================
    def belief_callback(self, msg):
        """Receive belief state from EKF."""
        self.mu[0] = msg.pose.pose.position.x
        self.mu[1] = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.world_pos = np.array([self.mu[0], self.mu[1], self.mu[2]])
        self.mu[2] = math.atan2(siny_cosp, cosy_cosp)
        
        # Velocity states (keep from prediction, not in standard PoseWithCovariance)
        # self.mu[3], self.mu[4] are updated via control commands
        
        # Extract covariance (6x6 in message, we need 5x5)
        cov_flat = np.array(msg.pose.covariance).reshape((6, 6))
        self.Sigma[0, 0] = cov_flat[0, 0]
        self.Sigma[0, 1] = cov_flat[0, 1]
        self.Sigma[1, 0] = cov_flat[1, 0]
        self.Sigma[1, 1] = cov_flat[1, 1]
        self.Sigma[2, 2] = cov_flat[5, 5]  # yaw variance
        self.Sigma[0, 2] = cov_flat[0, 5]
        self.Sigma[2, 0] = cov_flat[5, 0]
        self.Sigma[1, 2] = cov_flat[1, 5]
        self.Sigma[2, 1] = cov_flat[5, 1]
    
    def waypoint_callback(self, msg):
        """Optimize control given target waypoint in ROBOT FRAME."""
        tx = msg.pose.position.x
        ty = msg.pose.position.y
        
        # Current velocities (from last command)
        start_v = self.prev_ctrl[0]
        start_w = self.prev_ctrl[1]
        
        # In robot frame, robot starts at origin facing forward
        # mu is [x, y, theta, v, omega] in ROBOT FRAME
        start_mu = np.array([0.0, 0.0, 0.0, start_v, start_w])
        
        # Use uncertainty from EKF (still valid)
        start_Sigma = self.Sigma.copy()
        
        # Initial guess
        u0 = np.zeros(self.horizon * 2)
        
        # Bounds
        bounds = []
        for _ in range(self.horizon):
            bounds.append((0.0, self.max_v))
            bounds.append((-self.max_w, self.max_w))
        
        # Optimize
        result = minimize(
            self.cost_function,
            u0,
            args=(tx, ty, start_mu, start_Sigma, start_v, start_w, self.world_pos.copy()),
            method='SLSQP',
            bounds=bounds,
            options={'ftol': 1e-3, 'maxiter': 20}
        )
        
        if result.success:
            u_opt = result.x.reshape((self.horizon, 2))
            v_cmd = u_opt[0, 0]
            w_cmd = u_opt[0, 1]
        else:
            # Fallback if optimization fails
            v_cmd = 0.1
            w_cmd = 0.0
        
        self.prev_ctrl = np.array([v_cmd, w_cmd])
        
        # Publish command
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = 'base_link'
        twist_msg.twist.linear.x = float(v_cmd)
        twist_msg.twist.angular.z = float(w_cmd)
        self.publisher.publish(twist_msg)


    def compute_observability(self, mu):
        """
        Compute smooth observability based on UGV position relative to UAV FoV.
        Returns γ ∈ [0, 1]
        """
        # Distance from UGV to UAV ground projection
        dx = mu[0] - self.uav_x
        dy = mu[1] - self.uav_y
        d = np.sqrt(dx**2 + dy**2)
        
        # Smooth sigmoid: γ = 1 / (1 + exp(k * (d - r)))
        gamma = 1.0 / (1.0 + np.exp(self.gamma_k * (d - self.fov_radius)))
        
        return gamma, d


    def kalman_update(self, Sigma_pred):
        """
        Apply Kalman update assuming full observation.
        Returns reduced covariance.
        """
        S = self.H_obs @ Sigma_pred @ self.H_obs.T + self.R_obs
        K = Sigma_pred @ self.H_obs.T @ np.linalg.inv(S)
        I_KH = np.eye(5) - K @ self.H_obs
        Sigma_updated = I_KH @ Sigma_pred @ I_KH.T + K @ self.R_obs @ K.T
        return Sigma_updated



def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(BeliefMPCNode())
    rclpy.shutdown()
if __name__ == '__main__':
    main()
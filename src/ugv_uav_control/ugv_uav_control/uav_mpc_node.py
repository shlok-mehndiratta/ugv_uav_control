import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
import numpy as np
import math
import time
from scipy.optimize import minimize

# Shared utilities
import ugv_uav_control.ugv_model as ugv_model
import ugv_uav_control.view_quality as view_quality

# FALLBACK CONSTANT
DEFAULT_VEL_VARIANCE = 0.01

class UAVMPCNode(Node):
    def __init__(self):
        super().__init__('uav_mpc_node')
        
        # === PARAMETERS ===
        self.declare_parameter('lambda', 1.0)
        self.declare_parameter('gamma_q', 1.5) # Updated default to 1.5 per Step 3 Fix B
        self.declare_parameter('beta', 0.1)
        self.declare_parameter('force_fallback', False) # New parameter for study
        
        self.w_uncertainty = self.get_parameter('lambda').value
        self.w_view = self.get_parameter('gamma_q').value
        self.w_effort = self.get_parameter('beta').value
        self.force_fallback = self.get_parameter('force_fallback').value
        
        # MPC Constants
        self.N = 8
        self.dt = 0.3 # Fix C: Reduced from 0.5s to 0.3s
        
        # Simulation Constants
        self.alpha_v = 2.0
        self.alpha_w = 5.0
        self.Q = np.diag([1e-4, 1e-4, 4e-4, 1e-4, 2.5e-3])
        self.R_base = 0.05
        self.fov_radius = 2.89
        
        # === STATE VARIABLES ===
        self.uav_pos = None
        self.ugv_mu = None
        self.ugv_sigma = None
        self.ugv_traj = None
        
        # Warm Start
        self.u_prev = np.zeros((self.N, 2))
        self.first_solve = True
        
        # Baseline Metrics
        self.total_cycles = 0
        self.successful_solves = 0
        self.solve_times = []
        self.fallback_stats = {'success_fail': 0, 'timeout': 0, 'sanity_fail': 0}
        
        self.history_q = []
        self.history_sigma_tr = []
        self.history_aruco_visible = []
        self.history_dist = []
        
        self.warmup_cycles = 10 # Ignore first 10s for 'steady-state' verification
        
        # === SUBSCRIPTIONS ===
        self.create_subscription(Pose, '/uav_gps_pose', self.uav_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/belief_state', self.belief_callback, 10)
        self.create_subscription(Path, '/ugv_predicted_traj', self.traj_callback, 10)
        
        # === PUBLISHERS ===
        self.uav_cmd_pub = self.create_publisher(Twist, '/uav_cmd_vel', 10)
        
        # === PLANNING TIMER (1 Hz) ===
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info('UAV MPC Node Started (Step D - Hardened & Tuned)')

    def uav_callback(self, msg):
        self.uav_pos = np.array([msg.position.x, msg.position.y])
        
    def belief_callback(self, msg):
        mu = np.zeros(5)
        mu[0] = msg.pose.pose.position.x
        mu[1] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        mu[2] = math.atan2(siny_cosp, cosy_cosp)
        mu[3] = msg.pose.pose.position.z 
        mu[4] = msg.pose.pose.orientation.x
        self.ugv_mu = mu
        cov_flat = np.array(msg.pose.covariance).reshape((6, 6))
        sigma = np.eye(5) * DEFAULT_VEL_VARIANCE
        sigma[0, 0] = cov_flat[0, 0]; sigma[0, 1] = cov_flat[0, 1]
        sigma[1, 0] = cov_flat[1, 0]; sigma[1, 1] = cov_flat[1, 1]
        sigma[2, 2] = cov_flat[5, 5]; sigma[0, 2] = cov_flat[0, 5]
        sigma[2, 0] = cov_flat[5, 0]; sigma[1, 2] = cov_flat[1, 5]
        sigma[2, 1] = cov_flat[5, 1]; self.ugv_sigma = sigma

    def traj_callback(self, msg):
        traj = []
        for pose in msg.poses:
            q = pose.pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y); cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp); traj.append([pose.pose.position.x, pose.pose.position.y, yaw])
        self.ugv_traj = traj

    def timer_callback(self):
        if self.uav_pos is None or self.ugv_mu is None or self.ugv_traj is None:
            return
            
        self.total_cycles += 1
        
        # 1. Warm Start Shifting
        if self.first_solve:
            x0_mat = np.zeros((self.N, 2))
            self.first_solve = False
        else:
            x0_mat = np.roll(self.u_prev, -1, axis=0)
            x0_mat[-1] = [0.0, 0.0]
        x0 = x0_mat.flatten()
        
        # 2. Optimization (or Forced Fallback)
        if self.force_fallback:
            use_fallback = True; reason = "forced_fallback"
            solve_duration = 0.0
        else:
            start_time = time.time()
            bounds = [(-1.5, 1.5)] * (self.N * 2) 
            
            result = minimize(self.cost_function, x0, method='SLSQP', bounds=bounds, options={'maxiter': 20, 'ftol': 1e-3})
            solve_duration = time.time() - start_time
            self.solve_times.append(solve_duration)
            use_fallback = False; reason = ""
            
            # Condition 0: Lost Target Recovery
            q_curr = view_quality.compute_view_quality(self.uav_pos[0], self.uav_pos[1], self.ugv_mu[0], self.ugv_mu[1], r_fov=self.fov_radius)
            if q_curr < 0.01:
                use_fallback = True; reason = "lost_target"
                
        # 3. Fallback Logic Processing
        if use_fallback:
            if reason != "forced_fallback":
                self.get_logger().warn(f'[MPC] Fallback! {reason} (Solve: {solve_duration:.3f}s)')
            
            if reason == "lost_target":
                cmd_v = self.calculate_fallback_vel(max_v=1.5)
            else:
                cmd_v = self.calculate_fallback_vel(max_v=0.3)
            self.u_prev = np.zeros((self.N, 2))
        else:
            # result check
            if not result.success:
                use_fallback = True; reason = "success_fail"
                self.fallback_stats['success_fail'] += 1
            elif solve_duration > 0.4:
                use_fallback = True; reason = "timeout"
                self.fallback_stats['timeout'] += 1
            else:
                u_opt = result.x.reshape((self.N, 2))
                p_next = self.uav_pos + u_opt[0] * self.dt
                dist_curr = np.linalg.norm(self.uav_pos - self.ugv_mu[:2])
                dist_next = np.linalg.norm(p_next - self.ugv_mu[:2])
                if dist_next > 0.6 and dist_next > (dist_curr + 0.05):
                    use_fallback = True; reason = "sanity_fail"
                    self.fallback_stats['sanity_fail'] += 1
            # Re-check use_fallback
            if use_fallback:
                self.get_logger().warn(f'[MPC] Fallback! {reason} (Solve: {solve_duration:.3f}s)')
                cmd_v = self.calculate_fallback_vel(max_v=0.3)
                self.u_prev = np.zeros((self.N, 2))
            else:
                self.successful_solves += 1
                u_opt = result.x.reshape((self.N, 2))
                cmd_v = u_opt[0]; self.u_prev = u_opt
            
        # 5. Metrics Recording (with Warm-up)
        q_curr = view_quality.compute_view_quality(self.uav_pos[0], self.uav_pos[1], self.ugv_mu[0], self.ugv_mu[1], r_fov=self.fov_radius)
        tr_curr = np.trace(self.ugv_sigma)
        dist_curr = np.linalg.norm(self.uav_pos - self.ugv_mu[:2])
        
        if self.total_cycles > self.warmup_cycles:
            self.history_q.append(q_curr); self.history_sigma_tr.append(tr_curr)
            self.history_aruco_visible.append(1 if q_curr > 0.1 else 0)
            self.history_dist.append(dist_curr)

        # 6. Publish
        msg = Twist()
        msg.linear.x = float(cmd_v[0]); msg.linear.y = float(cmd_v[1])
        self.uav_cmd_pub.publish(msg)
        
        if self.total_cycles % 10 == 0 and self.total_cycles > self.warmup_cycles:
            rate = (self.successful_solves / self.total_cycles)*100
            self.get_logger().info("\n" + "="*40 + "\nSTEP D STEADY-STATE SUMMARY (PERIODIC)\n" + "="*40)
            self.get_logger().info(f"Total Cycles: {self.total_cycles} (Warmup: {self.warmup_cycles})")
            self.get_logger().info(f"Solve Time: {np.mean(self.solve_times)*1000:.1f} ms")
            self.get_logger().info(f"q_k (View): Mean={np.mean(self.history_q):.3f}, Min={np.min(self.history_q):.3f}")
            self.get_logger().info(f"tr(Sigma): Mean={np.mean(self.history_sigma_tr):.4f}, Max={np.max(self.history_sigma_tr):.4f}")
            self.get_logger().info(f"ArUco Visibility: {np.mean(self.history_aruco_visible)*100:.1f}%")
            self.get_logger().info(f"Mean Tracker Dist: {np.mean(self.history_dist):.2f} m")
            self.get_logger().info("="*40 + "\n")

    def calculate_fallback_vel(self, max_v=0.3):
        traj_points = np.array(self.ugv_traj)[:, :2]
        centroid = np.mean(traj_points, axis=0)
        err = centroid - self.uav_pos
        dist = np.linalg.norm(err)
        return max_v * (err / dist) if dist > 0.01 else np.zeros(2)

    def cost_function(self, u_flat):
        u = u_flat.reshape((self.N, 2))
        curr_uav_pos = self.uav_pos.copy(); curr_ugv_mu = self.ugv_mu.copy(); curr_ugv_sigma = self.ugv_sigma.copy()
        
        # Performance optimization: Pre-convert traj to numpy array
        traj_arr = np.array(self.ugv_traj)
        
        # Extrapolation pre-calculations
        dt_ugv = 0.2
        if len(traj_arr) < 2:
            p_last = self.ugv_mu[:3] 
            v_extrap = np.zeros(3)
            t_max_ugv = 0.0
        else:
            t_max_ugv = (len(traj_arr) - 1) * dt_ugv
            p_last = traj_arr[-1]
            p_prev = traj_arr[-2]
            v_extrap = (p_last - p_prev) / dt_ugv
        
        J = 0.0
        for k in range(self.N):
            curr_uav_pos += u[k] * self.dt
            t_eval = (k + 1) * self.dt
            
            # Boundary handling for interpolation
            if t_eval < t_max_ugv - 1e-5:
                idx = t_eval / dt_ugv
                i = int(idx)
                alpha = idx - i
                mu_k = (1-alpha)*traj_arr[i] + alpha*traj_arr[i+1]
            else:
                # Constant Velocity Extrapolation (t_eval >= t_max_ugv)
                mu_k = p_last + v_extrap * (t_eval - t_max_ugv)
            
            curr_ugv_mu[0:3] = mu_k
            F = ugv_model.compute_jacobian_F(curr_ugv_mu, self.dt, self.alpha_v, self.alpha_w)
            curr_ugv_sigma = F @ curr_ugv_sigma @ F.T + self.Q
            R_k, q_k = ugv_model.get_dynamic_R(curr_uav_pos, mu_k[:2], self.R_base, self.fov_radius)
            H = np.zeros((2, 5)); H[0,0]=1.0; H[1,1]=1.0
            curr_ugv_sigma = ugv_model.kalman_update(curr_ugv_sigma, H, R_k)
            J += (self.w_uncertainty * np.trace(curr_ugv_sigma) - self.w_view * q_k + 
                  self.w_effort * np.sum(u[k]**2))
        return J

def main(args=None):
    rclpy.init(args=args)
    node = UAVMPCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.total_cycles > 0:
            print("\n" + "="*40 + "\nSTEP D FINAL VERIFICATION SUMMARY\n" + "="*40)
            print(f"Total Cycles: {node.total_cycles}\nFallbacks: {node.fallback_stats}")
            print(f"Solve Time: {np.mean(node.solve_times)*1000:.1f} ms")
            print(f"q_k (View): Mean={np.mean(node.history_q):.3f}, Min={np.min(node.history_q):.3f}")
            print(f"tr(Sigma): Mean={np.mean(node.history_sigma_tr):.4f}, Max={np.max(node.history_sigma_tr):.4f}")
            print(f"ArUco Visibility: {np.mean(node.history_aruco_visible)*100:.1f}%")
            print(f"Mean Tracker Dist: {np.mean(node.history_dist):.2f} m")
            print("="*40 + "\n")
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()

"""
Experiment Logger Node
Logs data from any controller (Stanley, MPC, Belief MPC, etc.) for comparison.
Subscribes to common topics and optionally to /belief_state for covariance data.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped, PoseWithCovarianceStamped, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, String
import numpy as np
import csv
import os
import math
from datetime import datetime
import ugv_uav_control.view_quality as view_quality


class ExperimentLogger(Node):
    def __init__(self):
        super().__init__('experiment_logger')

        # Parameters
        self.declare_parameter('experiment_name', 'exp_default')
        self.declare_parameter('duration', 60.0)
        self.declare_parameter('log_rate', 10.0)
        self.exp_name = self.get_parameter('experiment_name').value
        self.duration = self.get_parameter('duration').value
        log_rate = self.get_parameter('log_rate').value

        # Data storage
        self.data = {
            'timestamp': [],
            'x': [], 'y': [], 'theta': [],
            'odom_x': [], 'odom_y': [], 'odom_theta': [],
            'true_ugv_x': [], 'true_ugv_y': [], 'true_ugv_theta': [],
            'v_cmd': [], 'w_cmd': [],
            'target_x': [], 'target_y': [],
            'cov_trace': [],
            'cov_xx': [], 'cov_yy': [], 'cov_tt': [],
            'fov_distance': [],
            'aruco_visible': [],
            'uav_x': [], 'uav_y': [], 'uav_z': [],
            'uav_dist': [],
            'q_k': [],
            'replan_trigger': [],
        }

        # UAV FoV parameters
        self.uav_x, self.uav_y, self.uav_z = 0.0, 0.0, 0.0

        # State cache
        self.current_cmd = [0.0, 0.0]
        self.current_target = [0.0, 0.0]
        self.current_pose = [0.0, 0.0, 0.0]  # x, y, theta
        self.current_odom = [0.0, 0.0, 0.0]  # odom_x, odom_y, odom_theta
        self.current_true_ugv = [0.5, 0.0, 0.0]  # true_ugv_x, true_ugv_y, true_ugv_theta
        self.current_cov = [0.0, 0.0, 0.0, 0.0, 0.0]  # cov_xx, cov_yy, cov_vv, cov_ww, cov_tt
        self.aruco_visible = False
        self.has_pose = False
        self.replan_flag = 0
        
        self.ugv_solve_times = []
        self.uav_solve_times = []
        self.replan_reasons = {}

        # Timing
        self.start_time = self.get_clock().now()
        self.last_aruco_time = self.start_time
        self.aruco_dropouts = []
        self.saved = False

        # Subscriptions — all common topics
        self.create_subscription(TwistStamped, '/cmd_vel', self.cmd_cb, 10)
        self.create_subscription(PoseStamped, '/waypoints', self.waypoint_cb, 10)
        self.create_subscription(PoseStamped, '/aruco_pose', self.aruco_cb, 10)

        # Belief state (available when EKF is running)
        self.create_subscription(
            PoseWithCovarianceStamped, '/belief_state', self.belief_cb, 10)

        # Odometry fallback (always available from Gazebo)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        
        # UAV Position (for q_k and distance calculations)
        self.create_subscription(Pose, '/uav_gps_pose', self.uav_cb, 10)
        
        # UGV Ground Truth Position directly from Gazebo
        self.create_subscription(Pose, '/model/turtlebot3_burger/pose', self.true_ugv_cb, 10)
        
        # Replan trigger
        self.create_subscription(Bool, '/uav_replan_trigger', self.replan_cb, 10)
        self.create_subscription(String, '/uav_replan_reason', self.replan_reason_cb, 10)
        
        # Solve times
        self.create_subscription(Float32, '/ugv_mpc_solve_time', self.ugv_solve_cb, 10)
        self.create_subscription(Float32, '/uav_mpc_solve_time', self.uav_solve_cb, 10)

        # Timer for periodic logging
        self.create_timer(1.0 / log_rate, self.log_callback)

        self.get_logger().info(
            f'Experiment Logger: "{self.exp_name}" | '
            f'Duration: {self.duration}s | Rate: {log_rate} Hz')

    # =========================================
    # CALLBACKS
    # =========================================
    def true_ugv_cb(self, msg):
        self.current_true_ugv[0] = msg.position.x
        self.current_true_ugv[1] = msg.position.y
        q = msg.orientation
        self.current_true_ugv[2] = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z))

    def belief_cb(self, msg):
        """Primary pose source when EKF is active."""
        # print("[DEBUG] Belief received")
        self.current_pose[0] = msg.pose.pose.position.x
        self.current_pose[1] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_pose[2] = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z))

        cov = np.array(msg.pose.covariance).reshape(6, 6)
        self.current_cov[0] = cov[0, 0] # x
        self.current_cov[1] = cov[1, 1] # y
        self.current_cov[2] = cov[2, 2] # v (mapped to z)
        self.current_cov[3] = cov[3, 3] # w (mapped to roll)
        self.current_cov[4] = cov[5, 5] # theta (yaw)
        self.has_pose = True

    def odom_cb(self, msg):
        """Log true physical pose from odometry."""
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z))
        self.current_odom[0] = msg.pose.pose.position.x
        self.current_odom[1] = msg.pose.pose.position.y
        self.current_odom[2] = yaw
        
        if not self.has_pose:
            self.current_pose[0] = msg.pose.pose.position.x
            self.current_pose[1] = msg.pose.pose.position.y
            self.current_pose[2] = yaw

    def uav_cb(self, msg):
        # print("[DEBUG] UAV pose received")
        self.uav_x = msg.position.x
        self.uav_y = msg.position.y
        self.uav_z = msg.position.z

    def cmd_cb(self, msg):
        self.current_cmd = [msg.twist.linear.x, msg.twist.angular.z]

    def waypoint_cb(self, msg):
        self.current_target = [msg.pose.position.x, msg.pose.position.y]

    def aruco_cb(self, msg):
        now = self.get_clock().now()
        gap = (now - self.last_aruco_time).nanoseconds / 1e9
        if gap > 0.5:
            self.aruco_dropouts.append(gap)
        self.last_aruco_time = now
        self.aruco_visible = True

    def replan_cb(self, msg):
        if msg.data:
            self.replan_flag = 1

    def replan_reason_cb(self, msg):
        reason = msg.data
        if reason not in self.replan_reasons:
            self.replan_reasons[reason] = 0
        self.replan_reasons[reason] += 1

    def ugv_solve_cb(self, msg):
        self.ugv_solve_times.append(msg.data)

    def uav_solve_cb(self, msg):
        self.uav_solve_times.append(msg.data)

    # =========================================
    # PERIODIC LOGGING
    # =========================================
    def log_callback(self):
        if self.uav_z < 0.1:
            return  # Wait for drone to spawn and takeoff (avoids 0.0 glitch)
            
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed > self.duration:
            if not self.saved:
                self.save_and_exit()
            return

        # Check ArUco freshness
        aruco_age = (self.get_clock().now() -
                     self.last_aruco_time).nanoseconds / 1e9
        if aruco_age > 0.5:
            self.aruco_visible = False
            
        if elapsed < 2.0: # Check connectivity early
            self.get_logger().info(f"[DEBUG] has_pose: {self.has_pose}, uav_z: {self.uav_z}")

        x, y, theta = self.current_pose
        fov_dist = math.sqrt((x - self.uav_x)**2 + (y - self.uav_y)**2)
        cov_trace = sum(self.current_cov)

        self.data['timestamp'].append(elapsed)
        if len(self.data['timestamp']) % 100 == 0:
            self.get_logger().info(f"Logging: {elapsed:.1f}/{self.duration}s | Samples: {len(self.data['timestamp'])}")
            
        self.data['x'].append(x)
        self.data['y'].append(y)
        self.data['theta'].append(theta)
        self.data['odom_x'].append(self.current_odom[0])
        self.data['odom_y'].append(self.current_odom[1])
        self.data['odom_theta'].append(self.current_odom[2])
        self.data['true_ugv_x'].append(self.current_true_ugv[0])
        self.data['true_ugv_y'].append(self.current_true_ugv[1])
        self.data['true_ugv_theta'].append(self.current_true_ugv[2])
        self.data['v_cmd'].append(self.current_cmd[0])
        self.data['w_cmd'].append(self.current_cmd[1])
        self.data['target_x'].append(self.current_target[0])
        self.data['target_y'].append(self.current_target[1])
        self.data['cov_trace'].append(cov_trace)
        self.data['cov_xx'].append(self.current_cov[0])
        self.data['cov_yy'].append(self.current_cov[1])
        self.data['cov_tt'].append(self.current_cov[2])
        self.data['fov_distance'].append(fov_dist)
        self.data['aruco_visible'].append(1 if self.aruco_visible else 0)
        
        # Enhanced Study Metrics
        self.data['uav_x'].append(self.uav_x)
        self.data['uav_y'].append(self.uav_y)
        self.data['uav_z'].append(self.uav_z)
        self.data['uav_dist'].append(fov_dist)
        
        qk = view_quality.compute_view_quality(self.uav_x, self.uav_y, self.uav_z, x, y)
        self.data['q_k'].append(qk)
        self.data['replan_trigger'].append(self.replan_flag)
        self.replan_flag = 0

    # =========================================
    # SAVE AND EXIT
    # =========================================
    def save_and_exit(self):
        self.saved = True
        # Save directly to the results folder in the workspace
        output_dir = os.path.join(os.getcwd(), 'results')
        os.makedirs(output_dir, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'{output_dir}/{self.exp_name}_{timestamp}.csv'

        # Save CSV
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(self.data.keys())
            rows = zip(*self.data.values())
            writer.writerows(rows)

        # Compute summary metrics
        n = len(self.data['x'])
        if n > 0:
            tracking_errors = [
                abs(ty)
                for ty in self.data['target_y']]

            control_changes = [
                math.sqrt((v2 - v1)**2 + (w2 - w1)**2)
                for v1, w1, v2, w2 in zip(
                    self.data['v_cmd'][:-1], self.data['w_cmd'][:-1],
                    self.data['v_cmd'][1:], self.data['w_cmd'][1:])]

            fov_exits = 0
            for d, z in zip(self.data['fov_distance'], self.data['uav_z']):
                r_fov_z = z * math.tan(1.047 / 2.0)
                if d > r_fov_z:
                    fov_exits += 1

            aruco_pct = (sum(self.data['aruco_visible']) / n * 100
                         if n > 0 else 0)

            ugv_dist_traveled = sum(math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                                    for x1, y1, x2, y2 in zip(
                                        self.data['x'][:-1], self.data['y'][:-1],
                                        self.data['x'][1:], self.data['y'][1:]))
                                        
            uav_dist_traveled = sum(math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                                    for x1, y1, x2, y2 in zip(
                                        self.data['uav_x'][:-1], self.data['uav_y'][:-1],
                                        self.data['uav_x'][1:], self.data['uav_y'][1:]))

            summary = {
                'experiment': self.exp_name,
                'total_samples': n,
                'total_replans': sum(self.data['replan_trigger']),
                'ugv_dist_traveled': ugv_dist_traveled,
                'uav_dist_traveled': uav_dist_traveled,
                'mean_ugv_solve_ms': np.mean(self.ugv_solve_times) * 1000 if self.ugv_solve_times else 0,
                'mean_uav_solve_ms': np.mean(self.uav_solve_times) * 1000 if self.uav_solve_times else 0,
                'mean_tracking_error': np.mean(tracking_errors),
                'max_tracking_error': np.max(tracking_errors),
                'control_smoothness': (
                    np.mean(control_changes) if control_changes else 0),
                'mean_cov_trace': np.mean(self.data['cov_trace']),
                'max_cov_trace': np.max(self.data['cov_trace']),
                'mean_uav_z': np.mean(self.data['uav_z']),
                'min_uav_z': np.min(self.data['uav_z']),
                'max_uav_z': np.max(self.data['uav_z']),
                'fov_exit_pct': fov_exits / n * 100,
                'aruco_visible_pct': aruco_pct,
                'aruco_dropouts': len(self.aruco_dropouts),
                'max_dropout_s': (
                    max(self.aruco_dropouts) if self.aruco_dropouts else 0),
            }

            summary_file = f'{output_dir}/{self.exp_name}_{timestamp}_summary.txt'
            with open(summary_file, 'w') as f:
                for k, v in summary.items():
                    if isinstance(v, float):
                        f.write(f'{k}: {v:.4f}\n')
                    else:
                        f.write(f'{k}: {v}\n')
                
                f.write('\n--- Trigger Breakdown ---\n')
                for reason, count in sorted(self.replan_reasons.items()):
                    f.write(f'{reason}: {count}\n')

            self.get_logger().info(f'Data saved to: {filename}')

    def destroy_node(self):
        if not self.saved:
            self.save_and_exit()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ExperimentLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
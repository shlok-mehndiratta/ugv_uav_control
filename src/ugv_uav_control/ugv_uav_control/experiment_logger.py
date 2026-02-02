import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import numpy as np
import csv
import os
from datetime import datetime
class ExperimentLogger(Node):
    def __init__(self):
        super().__init__('experiment_logger')
        
        # Parameters
        self.declare_parameter('experiment_name', 'exp_default')
        self.declare_parameter('duration', 60.0)
        self.exp_name = self.get_parameter('experiment_name').value
        self.duration = self.get_parameter('duration').value
        
        # Data storage
        self.data = {
            'timestamp': [],
            'x': [], 'y': [], 'theta': [],
            'v_cmd': [], 'w_cmd': [],
            'target_x': [], 'target_y': [],
            'cov_trace': [],
            'cov_xx': [], 'cov_yy': [], 'cov_tt': [],
            'fov_distance': [],
        }
        
        # UAV FoV parameters
        self.uav_x, self.uav_y = 0.0, 0.0
        self.fov_radius = 5.77
        
        # Subscriptions
        self.create_subscription(PoseWithCovarianceStamped, '/belief_state', self.belief_cb, 10)
        self.create_subscription(TwistStamped, '/cmd_vel', self.cmd_cb, 10)
        self.create_subscription(PoseStamped, '/waypoints', self.waypoint_cb, 10)
        self.create_subscription(PoseStamped, '/aruco_pose', self.aruco_cb, 10)
        
        # State
        self.current_cmd = [0.0, 0.0]
        self.current_target = [0.0, 0.0]
        self.start_time = self.get_clock().now()
        self.last_aruco_time = self.start_time
        self.aruco_dropouts = []
        
        # Timer for logging
        self.create_timer(0.1, self.log_callback)  # 10 Hz
        
        self.get_logger().info(f'Logging experiment: {self.exp_name} for {self.duration}s')
    
    def belief_cb(self, msg):
        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if t > self.duration:
            self.save_and_exit()
            return
        
        # Extract pose
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        theta = 2 * np.arctan2(q.z, q.w)
        
        # Extract covariance
        cov = np.array(msg.pose.covariance).reshape(6, 6)
        cov_xx = cov[0, 0]
        cov_yy = cov[1, 1]
        cov_tt = cov[5, 5]
        cov_trace = cov_xx + cov_yy + cov_tt
        
        # FoV distance
        fov_dist = np.sqrt((x - self.uav_x)**2 + (y - self.uav_y)**2)
        
        # Store
        self.data['timestamp'].append(t)
        self.data['x'].append(x)
        self.data['y'].append(y)
        self.data['theta'].append(theta)
        self.data['v_cmd'].append(self.current_cmd[0])
        self.data['w_cmd'].append(self.current_cmd[1])
        self.data['target_x'].append(self.current_target[0])
        self.data['target_y'].append(self.current_target[1])
        self.data['cov_trace'].append(cov_trace)
        self.data['cov_xx'].append(cov_xx)
        self.data['cov_yy'].append(cov_yy)
        self.data['cov_tt'].append(cov_tt)
        self.data['fov_distance'].append(fov_dist)
    
    def cmd_cb(self, msg):
        self.current_cmd = [msg.twist.linear.x, msg.twist.angular.z]
    
    def waypoint_cb(self, msg):
        self.current_target = [msg.pose.position.x, msg.pose.position.y]
    
    def aruco_cb(self, msg):
        now = self.get_clock().now()
        gap = (now - self.last_aruco_time).nanoseconds / 1e9
        if gap > 0.5:  # Dropout threshold
            self.aruco_dropouts.append(gap)
        self.last_aruco_time = now
    
    def log_callback(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed > self.duration:
            self.save_and_exit()
    
    def save_and_exit(self):
        # Create output directory
        output_dir = os.path.expanduser('~/experiment_logs')
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
        if len(self.data['x']) > 0:
            tracking_errors = [np.sqrt((x - tx)**2 + (y - ty)**2) 
                             for x, y, tx, ty in zip(
                                 self.data['x'], self.data['y'],
                                 self.data['target_x'], self.data['target_y'])]
            
            control_changes = [np.sqrt((v2-v1)**2 + (w2-w1)**2)
                              for v1, w1, v2, w2 in zip(
                                  self.data['v_cmd'][:-1], self.data['w_cmd'][:-1],
                                  self.data['v_cmd'][1:], self.data['w_cmd'][1:])]
            
            fov_exits = sum(1 for d in self.data['fov_distance'] if d > self.fov_radius)
            
            summary = {
                'mean_tracking_error': np.mean(tracking_errors),
                'max_tracking_error': np.max(tracking_errors),
                'control_smoothness': np.mean(control_changes) if control_changes else 0,
                'mean_cov_trace': np.mean(self.data['cov_trace']),
                'max_cov_trace': np.max(self.data['cov_trace']),
                'fov_exit_rate': fov_exits / len(self.data['x']) * 100,
                'aruco_dropouts': len(self.aruco_dropouts),
                'max_dropout_duration': max(self.aruco_dropouts) if self.aruco_dropouts else 0,
            }
            
            summary_file = f'{output_dir}/{self.exp_name}_{timestamp}_summary.txt'
            with open(summary_file, 'w') as f:
                for k, v in summary.items():
                    f.write(f'{k}: {v:.4f}\n')
            
            self.get_logger().info(f'Saved: {filename}')
            self.get_logger().info(f'Summary: {summary}')
        
        rclpy.shutdown()
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ExperimentLogger())
if __name__ == '__main__':
    main()
#!/usr/bin/env bash
# Determinism probe: two identical short runs; compare logs offline.
cd "$(dirname "$0")/.."
set +u                       # ROS setup scripts reference unset vars
source /opt/ros/jazzy/setup.bash
source install/setup.bash
cleanup() {
  pkill -9 -f 'gz sim' 2>/dev/null
  pkill -9 -f 'parameter_bridge' 2>/dev/null
  pkill -9 -f ruby 2>/dev/null
  pkill -9 -f 'ekf_node\|uav_mpc_node\|belief_mpc_node\|uav_ancillary_node\|experiment_logger' 2>/dev/null
  sleep 6
}
for R in A2 B2; do
  echo "=== RUN $R START ==="
  timeout 100 ros2 launch ugv_uav_control middle_path_follower.launch.py \
      experiment_name:=det_run_$R duration:=25.0 > results/log_det_$R.txt 2>&1
  echo "=== RUN $R rc=$? ==="
  cleanup
done
ls -t results/det_run_A2*.csv results/det_run_B2*.csv 2>/dev/null

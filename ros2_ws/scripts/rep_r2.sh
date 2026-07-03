#!/usr/bin/env bash
cd "$(dirname "$0")/.."; set +u
source /opt/ros/jazzy/setup.bash; source install/setup.bash
cleanup(){ pkill -9 -f 'gz sim' 2>/dev/null; pkill -9 -f 'parameter_bridge' 2>/dev/null; pkill -9 -f ruby 2>/dev/null; pkill -9 -f 'ekf_node\|uav_mpc_node\|belief_mpc_node\|uav_ancillary_node\|experiment_logger\|middle_path' 2>/dev/null; sleep 6; }
for R in rep1 rep2; do
  echo "=== $R ==="
  timeout 120 ros2 launch ugv_uav_control middle_path_follower.launch.py experiment_name:=REPR2_$R duration:=50.0 track_name:=track2.png controller:=belief_mpc use_timer:=false z_opt:=2.5 z_min:=1.8 block_aruco:=false use_ancillary:=true > results/log_REPR2_$R.txt 2>&1
  echo " rc=$?"; cleanup
done
echo DONE

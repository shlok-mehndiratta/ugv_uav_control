#!/usr/bin/env bash
cd "$(dirname "$0")/.."; set +u
source /opt/ros/jazzy/setup.bash; source install/setup.bash
GZ=/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz
xz(){ timeout 5 ros2 topic echo /model/uav/pose --once 2>/dev/null | grep -A3 position | grep -E 'x:|z:'; }
timeout 60 ros2 launch ugv_uav_control middle_path_follower.launch.py \
   experiment_name:=vtest2 duration:=50 z_opt:=3.0 use_ancillary:=false > results/log_vtest2.txt 2>&1 &
LP=$!; sleep 18
echo "=== initial pos ==="; xz
echo "--- gz DIRECT pub linear.x=0.5, linear.z=1.0 (same message, both axes) ---"
$GZ topic -t /model/uav/cmd_vel -m gz.msgs.Twist -p "linear: {x: 0.5, z: 1.0}" 2>/dev/null
sleep 4
echo "=== pos after 4s ==="; xz
kill $LP 2>/dev/null; pkill -9 -f 'gz sim' 2>/dev/null; pkill -9 -f ruby 2>/dev/null; pkill -9 -f parameter_bridge 2>/dev/null; pkill -9 -f 'ekf_node\|uav_mpc\|belief_mpc\|middle_path\|experiment_logger' 2>/dev/null
echo VTEST2_DONE

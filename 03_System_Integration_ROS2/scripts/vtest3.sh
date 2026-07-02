#!/usr/bin/env bash
cd "$(dirname "$0")/.."; set +u
source /opt/ros/jazzy/setup.bash; source install/setup.bash
GZ=/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz
zread(){ timeout 5 ros2 topic echo /model/uav/pose --field position.z --once 2>/dev/null | head -1; }
timeout 70 ros2 launch ugv_uav_control middle_path_follower.launch.py \
   experiment_name:=vtest3 duration:=60 z_opt:=3.0 use_ancillary:=false > results/log_vtest3.txt 2>&1 &
LP=$!; sleep 18
echo "z_initial = $(zread)"
echo "--- CONTINUOUS direct-gz pub linear.z=1.0 (~20Hz for 6s) ---"
for i in $(seq 1 75); do $GZ topic -t /model/uav/cmd_vel -m gz.msgs.Twist -p "linear: {z: 1.0}" 2>/dev/null; sleep 0.08; done
echo "z_after_continuous_gz = $(zread)"
kill $LP 2>/dev/null; pkill -9 -f 'gz sim' 2>/dev/null; pkill -9 -f ruby 2>/dev/null; pkill -9 -f parameter_bridge 2>/dev/null; pkill -9 -f 'ekf_node\|uav_mpc\|belief_mpc\|middle_path\|experiment_logger' 2>/dev/null
echo VTEST3_DONE

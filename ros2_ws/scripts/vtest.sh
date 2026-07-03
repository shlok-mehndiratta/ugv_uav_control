#!/usr/bin/env bash
cd "$(dirname "$0")/.."; set +u
source /opt/ros/jazzy/setup.bash; source install/setup.bash
GZ=/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz
zread(){ timeout 5 ros2 topic echo /model/uav/pose --field position.z --once 2>/dev/null | head -1; }
# launch sim WITHOUT ancillary (no node publishes /model/uav/cmd_vel)
timeout 70 ros2 launch ugv_uav_control middle_path_follower.launch.py \
   experiment_name:=vtest duration:=60 z_opt:=3.0 use_ancillary:=false > results/log_vtest.txt 2>&1 &
LP=$!
sleep 18
echo "z_initial = $(zread)"
echo "--- TEST A: gz DIRECT pub linear.z=1.0 (bypasses ROS bridge) ---"
$GZ topic -t /model/uav/cmd_vel -m gz.msgs.Twist -p "linear: {z: 1.0}" 2>/dev/null
sleep 4
echo "z_after_gz = $(zread)"
$GZ topic -t /model/uav/cmd_vel -m gz.msgs.Twist -p "linear: {z: 0.0}" 2>/dev/null; sleep 2
echo "--- TEST B: ROS pub linear.z=1.0 @10Hz for 4s (via bridge) ---"
zb=$(zread); echo "z_before_ros = $zb"
timeout 4 ros2 topic pub /model/uav/cmd_vel geometry_msgs/msg/Twist "{linear: {z: 1.0}}" -r 10 >/dev/null 2>&1
sleep 1
echo "z_after_ros = $(zread)"
kill $LP 2>/dev/null
pkill -9 -f 'gz sim' 2>/dev/null; pkill -9 -f ruby 2>/dev/null; pkill -9 -f parameter_bridge 2>/dev/null; pkill -9 -f 'ekf_node\|uav_mpc\|belief_mpc\|middle_path\|experiment_logger' 2>/dev/null
echo VTEST_DONE

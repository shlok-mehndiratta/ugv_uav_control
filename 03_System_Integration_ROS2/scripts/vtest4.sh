#!/usr/bin/env bash
cd "$(dirname "$0")/.."; set +u
source /opt/ros/jazzy/setup.bash; source install/setup.bash
GZ=/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz
timeout 90 ros2 launch ugv_uav_control middle_path_follower.launch.py \
   experiment_name:=vtest4 duration:=80 z_opt:=3.0 use_ancillary:=false > results/log_vtest4.txt 2>&1 &
LP=$!; sleep 18
echo "z_initial = $(timeout 5 ros2 topic echo /model/uav/pose --field position.z --once 2>/dev/null|head -1)"
# continuous ROS publish of z=2.0
ros2 topic pub -r 20 /model/uav/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 2.0}}" >/dev/null 2>&1 &
PP=$!; sleep 2
echo "--- gz-side message actually reaching the plugin (does linear.z survive the bridge?) ---"
timeout 4 $GZ topic -e -t /model/uav/cmd_vel -n 1 2>/dev/null
sleep 4
echo "z_after_4s = $(timeout 5 ros2 topic echo /model/uav/pose --field position.z --once 2>/dev/null|head -1)"
kill $PP 2>/dev/null; kill $LP 2>/dev/null
pkill -9 -f 'gz sim' 2>/dev/null; pkill -9 -f ruby 2>/dev/null; pkill -9 -f parameter_bridge 2>/dev/null; pkill -9 -f 'ekf_node\|uav_mpc\|belief_mpc\|middle_path\|experiment_logger' 2>/dev/null
echo VTEST4_DONE

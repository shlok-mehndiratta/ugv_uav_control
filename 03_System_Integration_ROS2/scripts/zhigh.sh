#!/usr/bin/env bash
cd "$(dirname "$0")/.."; set +u
source /opt/ros/jazzy/setup.bash; source install/setup.bash
DUR=35.0; TO=95
cleanup(){ pkill -9 -f 'gz sim' 2>/dev/null; pkill -9 -f parameter_bridge 2>/dev/null; pkill -9 -f ruby 2>/dev/null; pkill -9 -f 'ekf_node\|uav_mpc_node\|belief_mpc_node\|uav_ancillary_node\|experiment_logger\|middle_path' 2>/dev/null; sleep 6; }
run(){ NAME="$1"; Z="$2"; echo "=== $(date +%H:%M:%S) $NAME z=$Z ==="; timeout $TO ros2 launch ugv_uav_control middle_path_follower.launch.py experiment_name:="$NAME" duration:=$DUR track_name:=track2.png controller:=belief_mpc use_ancillary:=true use_timer:=false z_opt:=$Z z_min:=3.0 z_max:=35.0 > results/log_$NAME.txt 2>&1; echo "  rc=$? -> $(ls -t results/${NAME}_*.csv 2>/dev/null|head -1|xargs -r basename)"; cleanup; }
run ZH_z08 8.0
run ZH_z12 12.0
run ZH_z18 18.0
run ZH_z26 26.0
echo "ZHIGH DONE $(date +%H:%M:%S)"

#!/usr/bin/env bash
cd "$(dirname "$0")/.."; set +u
source /opt/ros/jazzy/setup.bash; source install/setup.bash
DUR=40.0; TO=110
cleanup(){ pkill -9 -f 'gz sim' 2>/dev/null; pkill -9 -f parameter_bridge 2>/dev/null; pkill -9 -f ruby 2>/dev/null; pkill -9 -f 'ekf_node\|uav_mpc_node\|belief_mpc_node\|uav_ancillary_node\|experiment_logger\|middle_path' 2>/dev/null; sleep 6; }
run(){ NAME="$1"; Z="$2"; echo "=== $(date +%H:%M:%S) $NAME z=$Z ==="; timeout $TO ros2 launch ugv_uav_control middle_path_follower.launch.py experiment_name:="$NAME" duration:=$DUR track_name:=track2.png controller:=belief_mpc use_ancillary:=true use_timer:=false active_altitude:=false use_gsd_noise:=true block_aruco:=false z_opt:=$Z z_min:=2.5 z_max:=9.0 > results/log_$NAME.txt 2>&1; echo "  rc=$? -> $(ls -t results/${NAME}_*.csv 2>/dev/null|head -1|xargs -r basename)"; cleanup; }
for z in 3.0 4.0 5.0 6.0 7.0 8.0; do
  run GSD_z${z%.*} $z
done
echo "GSDSWEEP DONE $(date +%H:%M:%S)"

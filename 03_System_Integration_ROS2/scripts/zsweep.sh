#!/usr/bin/env bash
cd "$(dirname "$0")/.."; set +u
source /opt/ros/jazzy/setup.bash; source install/setup.bash
DUR=60.0; TO=130
cleanup(){ pkill -9 -f 'gz sim' 2>/dev/null; pkill -9 -f parameter_bridge 2>/dev/null; pkill -9 -f ruby 2>/dev/null; pkill -9 -f 'ekf_node\|uav_mpc_node\|belief_mpc_node\|uav_ancillary_node\|experiment_logger\|middle_path' 2>/dev/null; sleep 6; }
run(){ NAME="$1"; shift; echo "=== $(date +%H:%M:%S) $NAME : $* ==="; timeout $TO ros2 launch ugv_uav_control middle_path_follower.launch.py experiment_name:="$NAME" duration:=$DUR track_name:=track_hairpin.png controller:=belief_mpc use_ancillary:=true use_timer:=false "$@" > results/log_$NAME.txt 2>&1; echo "  rc=$? -> $(ls -t results/${NAME}_*.csv 2>/dev/null|head -1|xargs -r basename)"; cleanup; }
declare -A ZMIN=( [z25]=1.8 [z35]=2.5 [z50]=4.0 [z70]=6.0 )
declare -A ZOPT=( [z25]=2.5 [z35]=3.5 [z50]=5.0 [z70]=7.0 )
for tag in z25 z35 z50 z70; do
  for r in 1 2; do
    run ZS_${tag}_r$r z_opt:=${ZOPT[$tag]} z_min:=${ZMIN[$tag]} z_max:=10.0
  done
done
echo "ZSWEEP DONE $(date +%H:%M:%S)"

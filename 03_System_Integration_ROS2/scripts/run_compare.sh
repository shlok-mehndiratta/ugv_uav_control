#!/usr/bin/env bash
cd "$(dirname "$0")/.."; set +u
source /opt/ros/jazzy/setup.bash; source install/setup.bash
TRACK="${1:-track_hairpin.png}"; TAG="${2:-HAIR}"; REP="${3:-3}"; DUR="${4:-60.0}"
TO=$(python3 -c "print(int($DUR+70))")
cleanup(){ pkill -9 -f 'gz sim' 2>/dev/null; pkill -9 -f parameter_bridge 2>/dev/null; pkill -9 -f ruby 2>/dev/null; pkill -9 -f 'ekf_node\|uav_mpc_node\|belief_mpc_node\|uav_ancillary_node\|experiment_logger\|middle_path' 2>/dev/null; sleep 6; }
run(){ NAME="$1"; shift; echo "=== $(date +%H:%M:%S) $NAME : $* ==="; timeout $TO ros2 launch ugv_uav_control middle_path_follower.launch.py experiment_name:="$NAME" duration:=$DUR track_name:=$TRACK controller:=belief_mpc use_ancillary:=true "$@" > results/log_$NAME.txt 2>&1; echo "  rc=$? -> $(ls -t results/${NAME}_*.csv 2>/dev/null|head -1|xargs -r basename)"; cleanup; }
for r in $(seq 1 $REP); do
  run ${TAG}_C1_r$r use_timer:=true
  run ${TAG}_B_r$r  use_timer:=false freeze_alpha:=true  error_only:=true
  run ${TAG}_C3_r$r use_timer:=false freeze_alpha:=false error_only:=false
done
echo "COMPARE DONE $(date +%H:%M:%S)"

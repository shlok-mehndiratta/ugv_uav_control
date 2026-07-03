#!/usr/bin/env bash
# Day-1 go/no-go trio: track2, R2 (z_opt=2.5,z_min=1.8), occlusion at U-bend (44-50s),
# 65s for recovery tail. Same regime/track/occlusion; only the trigger set differs.
#   A perception : freeze_alpha=false error_only=false use_timer=false (all levels, adaptive a)
#   B honest base: freeze_alpha=true  error_only=true  use_timer=false (perception-free)
#   C continuous : use_timer=true (c1_baseline)
# Level 4 (plan expiry) stays ON for A and B (not gated by error_only).
cd "$(dirname "$0")/.."
set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
DUR=65.0; TO=$(python3 -c "print(int($DUR+70))")
COMMON="track_name:=track2.png duration:=$DUR use_ancillary:=true controller:=belief_mpc z_opt:=2.5 z_min:=1.8 block_aruco:=true block_aruco_start:=44.0 block_aruco_end:=50.0"
cleanup(){ pkill -9 -f 'gz sim' 2>/dev/null; pkill -9 -f 'parameter_bridge' 2>/dev/null; pkill -9 -f ruby 2>/dev/null; pkill -9 -f 'ekf_node\|uav_mpc_node\|belief_mpc_node\|uav_ancillary_node\|experiment_logger\|middle_path' 2>/dev/null; sleep 6; }
run(){ NAME="$1"; shift; echo "=== $(date +%H:%M:%S) $NAME : $* ==="; timeout $TO ros2 launch ugv_uav_control middle_path_follower.launch.py experiment_name:="$NAME" $COMMON "$@" > "results/log_${NAME}.txt" 2>&1; echo "  rc=$? -> $(ls -t results/${NAME}_*.csv 2>/dev/null|head -1|xargs -r basename)"; cleanup; }

run TRIO_A_perc  use_timer:=false freeze_alpha:=false error_only:=false
run TRIO_B_error use_timer:=false freeze_alpha:=true  error_only:=true
run TRIO_C_cont  use_timer:=true
echo "=== TRIO COMPLETE $(date +%H:%M:%S) ==="
ls -t results/TRIO_*.csv 2>/dev/null | head

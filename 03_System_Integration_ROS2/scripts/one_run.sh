#!/usr/bin/env bash
# one_run.sh NAME DUR [extra launch args...]
cd "$(dirname "$0")/.."
set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
NAME="$1"; DUR="$2"; shift 2
timeout $(python3 -c "print(int($DUR+70))") ros2 launch ugv_uav_control middle_path_follower.launch.py \
    experiment_name:="$NAME" duration:="$DUR" use_ancillary:=true "$@" > "results/log_${NAME}.txt" 2>&1
echo "rc=$? -> $(ls -t results/${NAME}_*.csv 2>/dev/null | head -1 | xargs -r basename)"
pkill -9 -f 'gz sim' 2>/dev/null; pkill -9 -f 'parameter_bridge' 2>/dev/null; pkill -9 -f ruby 2>/dev/null
pkill -9 -f 'ekf_node\|uav_mpc_node\|belief_mpc_node\|uav_ancillary_node\|experiment_logger\|middle_path' 2>/dev/null
sleep 5

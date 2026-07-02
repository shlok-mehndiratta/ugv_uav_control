#!/usr/bin/env bash
# Batch runner for the UAV-UGV study. Runs each (method x scenario x repeat)
# through the Gazebo mission, writing one fresh CSV per run into results/.
#
# NOTE ON VARIABILITY: the ROS2/Gazebo pipeline is *not* bit-reproducible under
# fixed configuration (verified by scripts/check_determinism.py: repeated runs
# differ by cm-level in ground-truth pose due to asynchronous scheduling). There
# is no settable seed, so we cannot pair noise realizations across methods.
# Instead we take N repeats per (method,scenario) and report mean+/-std (unpaired),
# and sweep an adversarial occlusion scenario in addition to the nominal track.
#
# Usage:  bash scripts/run_batch.sh [DURATION_S] [REPEATS]
# Reproduce everything:  make reproduce
cd "$(dirname "$0")/.."          # -> 03_System_Integration_ROS2
set +u                            # ROS setup scripts reference unset vars
source /opt/ros/jazzy/setup.bash
source install/setup.bash

DUR="${1:-40.0}"
REPEATS="${2:-3}"
LAUNCH="ros2 launch ugv_uav_control middle_path_follower.launch.py"
PER_RUN_TIMEOUT=$(python3 -c "print(int(float('$DUR')+70))")

cleanup() {
  pkill -9 -f 'gz sim' 2>/dev/null
  pkill -9 -f 'ros_gz'  2>/dev/null
  pkill -9 -f 'parameter_bridge' 2>/dev/null
  pkill -9 -f ruby 2>/dev/null
  pkill -9 -f 'ekf_node\|uav_mpc_node\|belief_mpc_node\|uav_ancillary_node\|experiment_logger\|stanley_node\|pure_pursuit_node\|mpc_node' 2>/dev/null
  sleep 6
}

run_one() {  # name controller use_timer block bstart bend
  local name="$1" controller="$2" use_timer="$3" block="$4" bstart="$5" bend="$6"
  echo "=== $(date +%H:%M:%S) RUN ${name} | ctrl=${controller} timer=${use_timer} block=${block}[${bstart},${bend}] ==="
  timeout "${PER_RUN_TIMEOUT}" $LAUNCH \
      experiment_name:="${name}" duration:="${DUR}" \
      controller:="${controller}" use_timer:="${use_timer}" use_ancillary:=true \
      block_aruco:="${block}" block_aruco_start:="${bstart}" block_aruco_end:="${bend}" \
      > "results/log_${name}.txt" 2>&1
  echo "    rc=$? -> $(ls -t results/${name}_*.csv 2>/dev/null | head -1 | xargs -r basename)"
  cleanup
}

# CLEAN, FUNCTIONAL 2-method comparison (the UAV planner needs belief_mpc's
# /ugv_predicted_traj to solve at all, so a passive-UGV baseline would leave the
# UAV planner idle -- an invalid comparison; see AUDIT_REPORT.md). Both configs
# use belief_mpc; they differ only in the UAV replanning trigger:
#   C1 = continuous replanning (use_timer -> c1_baseline)
#   C3 = perception-driven event-triggered replanning (proposed)
#   scenario: nominal (no occlusion) and occ (ArUco blocked ~20-28s EKF-time)
for r in $(seq 1 "${REPEATS}"); do
  run_one "C1_nominal_r${r}" belief_mpc true  false 0.0  0.0
  run_one "C3_nominal_r${r}" belief_mpc false false 0.0  0.0
  run_one "C1_occ_r${r}"     belief_mpc true  true  20.0 28.0
  run_one "C3_occ_r${r}"     belief_mpc false true  20.0 28.0
done

echo "=== BATCH COMPLETE $(date +%H:%M:%S) ==="
ls -t results/C[123]_*.csv 2>/dev/null | head -40

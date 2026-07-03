#!/bin/bash
# Envelope-sweep cell runner with per-track on-lane spawn. Usage: run_cell.sh <track> <z> <v> <tag>
source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
cd /home/omen/CRAZYFLY_SITL/ugv_uav_control/03_System_Integration_ROS2
source install/setup.bash >/dev/null 2>&1
SP=/tmp/claude-1000/-home-omen-CRAZYFLY-SITL-ugv-uav-control/a35db6a5-7828-43aa-a1f3-f7f81b0184ca/scratchpad
export GZ_HEADLESS=1
TRACK=$1; Z=$2; V=$3; TAG=$4
# per-track on-lane spawn (x y yaw) from find_spawn.py
case $TRACK in
  track2.png)        SX=0.46;  SY=-0.52; SYAW=-2.02 ;;
  track1.png)        SX=0.43;  SY=-4.22; SYAW=-2.40 ;;
  track3.png)        SX=0.81;  SY=3.77;  SYAW=2.15  ;;
  track4.png)        SX=0.03;  SY=0.08;  SYAW=-1.96 ;;
  track_hairpin.png) SX=1.73;  SY=-1.23; SYAW=-2.87 ;;
  track_slalom.png)  SX=-4.26; SY=1.8;   SYAW=-2.04 ;;
  *) echo "unknown track $TRACK"; exit 1 ;;
esac
timeout 70 ros2 launch ugv_uav_control middle_path_follower.launch.py \
    experiment_name:=$TAG duration:=45 track_name:=$TRACK \
    ugv_x:=$SX ugv_y:=$SY ugv_yaw:=$SYAW \
    active_altitude:=false z_opt:=$Z z_min:=$Z use_gsd_noise:=true ugv_max_v:=$V \
    > $SP/${TAG}.log 2>&1
echo "$TAG (track=$TRACK z=$Z v=$V spawn=$SX,$SY,$SYAW) exit=$? det=$(grep -c 'DETECTED IDS' $SP/${TAG}.log)"

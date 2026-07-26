# COAST — Shared-Camera Aerial-Ground Tracking (simulation code)

ROS 2 / Gazebo simulation code for the paper *COAST: Altitude as a Coupled Perception Variable in
Shared-Camera Aerial-Ground Tracking*. A single downward-facing UAV camera is the ground vehicle's only
exteroceptive sensor, serving **both** its navigation reference (lane segmentation) **and** its state
estimate (ArUco marker + EKF). Because one camera does both jobs, the UAV altitude couples navigation
coverage and estimation resolution: low altitude sharpens estimation (`R ∝ z²`) but shrinks the
footprint; high altitude does the reverse.

## Requirements

- **ROS 2 Jazzy** and **Gazebo (Harmonic)**
- Python packages in `ros2_ws/requirements.txt`

## Build & run

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash

# Headless is required for reproducible closed-loop runs:
GZ_HEADLESS=1 ros2 launch ugv_uav_control middle_path_follower.launch.py \
    experiment_name:=demo duration:=45
```

Each run writes a per-step CSV and a `_summary.txt` into `ros2_ws/results/`. Experiment options are
launch arguments — most notably the track and the altitude policy:

```bash
# choose a track (standard set: track_lowk / track_midk / track_highk / track_vark)
... track_name:=track_highk.png ugv_x:=3.0 ugv_y:=0.0 ugv_yaw:=-1.57

# adaptive altitude vs. a fixed altitude
... active_altitude:=true            # curvature-scheduled policy
... active_altitude:=false z_opt:=2.5 z_min:=2.5   # fixed altitude
```

See `ros2_ws/README.md` for the full argument list and the analysis scripts in `ros2_ws/scripts/`.

> **Note.** This repository contains the simulation code only; experiment data, figures, and the
> manuscript are not included here. Run the launch files above to regenerate run data locally.

## License

Released under the MIT License — see [`LICENSE`](LICENSE).

## Authors

Shlok Mehndiratta, Prashant Gupta, Divyam Sood, Agniva Banerjee, and Arijit Sen — Department of
Electrical Engineering and Computer Science, Indian Institute of Science Education and Research (IISER)
Bhopal.

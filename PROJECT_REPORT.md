# CrazySim — Event-Triggered Tube MPC for Crazyflie SITL

Software-in-the-Loop simulation of a Crazyflie nano quadrotor flying **M / S trajectories** under **Event-Triggered Tube MPC**, with the mRPI tube visualised live in RViz2.

**Stack:** Gazebo Sim Garden 7.9.0 + ROS 2 Humble + CrazySim firmware
**Platform:** Ubuntu 22.04 | Python 3.10 | cflib (CrazySim fork)

---

## 🐳 Pulling the Image

Already published on Docker Hub: **<https://hub.docker.com/r/rebelpg/crazysim_et_mpc/tags>**

Two flavours:

```bash
docker pull rebelpg/crazysim_et_mpc:latest    # slim, recommended (~2.6 GB pull)
docker pull rebelpg/crazysim_et_mpc:slim      # alias of :latest
docker pull rebelpg/crazysim_et_mpc:v1        # full layered version (~3.4 GB pull)
docker pull rebelpg/crazysim_et_mpc:complete  # alias of :v1
```

Run with X11 forwarding for Gazebo / RViz:

```bash
xhost +local:docker
docker run -it --rm \
  --env DISPLAY=$DISPLAY --env LIBGL_ALWAYS_SOFTWARE=1 \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --network host \
  rebelpg/crazysim_et_mpc:latest \
  bash
```

Inside the container, just type `phase2` (or `phase1`, `phase2 S`).

> First run note: the slim image flattens the build artifacts away. The `.bashrc` auto-restore copies `/root/.mpc_ws_baked/` source tree to `/root/mpc_ws/` on first shell. If you want to rebuild lcs from source: `cd /root/mpc_ws && colcon build --symlink-install`. Otherwise the prebuilt entry points in `/usr/local/bin/` work directly.

---

## 🚀 Quick Start

### Phase 1 — Hover Test (verify the SITL pipeline)

```bash
phase1
```

- **Window 0:** Gazebo opens, drone visible on the ground
- **Window 1:** drone takes off, hovers 1.0 m for 10 s, lands
- **Switch windows:** `Ctrl+B` then `0` / `1`

### Phase 2 — ET-Tube MPC Trajectory Following

```bash
phase2        # M-shape (default)
phase2 M      # M-shape explicit
phase2 S      # S-shape
```

Drone is spawned at the trajectory's start vertex and takes off straight to the first waypoint's altitude. Three tmux windows:

| Window | Name      | Contents                                                       |
| ------ | --------- | -------------------------------------------------------------- |
| 0      | `A-sitl`  | Gazebo + cf2                                                   |
| 1      | `B-mpc`   | cflib_bridge + ET-Tube MPC + waypoint pub + tube viz           |
| 2      | `C-rviz`  | RViz2 with the tube + waypoints + odom trail                   |

- **Switch:** `Ctrl+B` then `0` / `1` / `2`
- **Detach:** `Ctrl+B` then `D`
- **Kill:** `stop_sitl` (or `tmux kill-session -t phase2`)

### Stop Everything Cleanly

```bash
stop_sitl     # kills SITL + MPC stack + RViz + tmux sessions
# After clean kill, sleeps 5 s so cf2 / Gazebo daemon fully release UDP ports
# before the next `phase2` launch.  `stop_sitl && phase2 M` chains correctly.
```

---

## ⏱️ Timeline (phase2)

| Time   | Event                                                            |
| ------ | ---------------------------------------------------------------- |
| t=0 s  | Gazebo + cf2 start (drone spawned at trajectory's start xy)      |
| t=12 s | MPC stack starts → cflib_bridge connects + takes off             |
| t=20 s | RViz opens; drone visible hovering at start vertex               |
| t=24 s | First waypoint published → drone follows trajectory              |

**Total time from `phase2` to first waypoint motion: ~24 s.**

---

## 👀 What You Will See

### Gazebo
- Drone spawns at trajectory start xy and takes off automatically
- Follows M or S shaped path at constant altitude (1.0 m by default)
- Banks into turns

### RViz2
- **Cyan axes** = current `cf_1` odom pose (45 Hz)
- **Yellow axes** = MPC nominal pose (one step ahead in MPC plan)
- **Translucent blue cylinder** around nominal pose = mRPI safety tube Ω (radius 0.408 m, height 0.5 m, computed at startup via LP projection)
- **Odom trail** (Keep=500) draws the actual flight path
- **Green spheres** = waypoints (accumulated as published)
- **Red spheres** = ET trigger events (fade after 2 s)

### Terminal (Window 1)
- `cflib_bridge`: takeoff → `"CFLIB BRIDGE READY"`
- `et_tube_mpc`: `"Event trigger #N at step S (R% solve rate)"`
- ~30% solve rate while moving, ~5% when hovering
- `waypoint_pub`: `"Published waypoint [k]"` / `"Waypoint [k] reached!"`

---

## 🛤️ Trajectories

**M shape** (5 waypoints, z=1.0 m):

```
[0,0,1] → [0,2,1] → [1,1,1] → [2,2,1] → [2,0,1]
```

**S shape** (5 waypoints, z=1.0 m):

```
[0,0,1] → [1,0.5,1] → [0,1,1] → [-1,1.5,1] → [0,2,1]
```

### Add Your Own

1. Edit `/root/mpc_ws/phase2/trajectories.py`
2. Define the new path:
   ```python
   NEW = [[x0,y0,z0], [x1,y1,z1], ...]
   TRAJECTORIES = {"M": M_SHAPE, "S": S_SHAPE, "NEW": NEW}
   ```
3. Run it:
   ```bash
   phase2 NEW
   ```

Drone spawns at `NEW[0]` xy and takes off to `NEW[0]` z automatically.

---

## 📐 Arena Bounds

State constraints (set in `tube_calculation.py`):

| Quantity        | Bounds                       |
| --------------- | ---------------------------- |
| `px`, `py`      | `[-2.5, 2.5]` m              |
| `pz`            | `[0, 3.0]` m                 |
| `vx`, `vy`, `vz`| `[-2.5, 2.5]` m/s            |
| `ax`, `ay`      | `[-5.66, 5.66]` m/s²         |
| `az`            | `[-4, 4]` m/s²               |

After mRPI tightening (X_tight = X ⊖ Ω):

| Quantity   | Usable Range          |
| ---------- | --------------------- |
| `px`, `py` | 4.183 m wide          |
| `pz`       | 1.722 m               |
| `v`        | 3.762 m/s             |
| `az`       | -3, 1.5 m/s²          |

Both M and S waypoints fit inside the tightened arena. Trajectories that exceed those bounds will cause the QP to fight constraints and the drone will oscillate; in that case, expand the arena and re-run `tube_calculation.py` to regenerate `tube_data.npz`.

---

## 🏗️ Architecture (Phase 2)

```
SITL: gz sim Garden + libgz_crazysim_plugin + cf2 firmware
   ↕  CFLib UDP 127.0.0.1:19850 (single persistent client)
cflib_bridge_node (lcs.cflib_bridge_node)
   ↕  /drone/cmd_accel, /drone/nominal_state
et_tube_mpc_node (lcs.et_tube_mpc_node)
   ↕  /waypoint
waypoint_publisher_node (lcs.waypoint_publisher_node)
   →  /tube_viz
tube_viz_node (lcs.tube_viz_node)
```

### `cflib_bridge_node`
- Owns the CFLib connection for the whole run
- Kalman reset + takeoff to (configurable) `takeoff_height` on startup
- Log block at 50 Hz → `/cf_1/odom`
- Converts `/drone/cmd_accel` + `/drone/nominal_state` → `cf.commander.send_full_state_setpoint(pos, vel, acc, ...)`

### `et_tube_mpc_node`
- Subscribes: `/cf_1/odom` (remapped from `/drone/odom`), `/waypoint`
- Publishes: `/drone/cmd_accel`, `/drone/nominal_pose`, `/drone/nominal_state`, `/drone/et_trigger`, `/drone/mpc_status`
- Horizon `N=20`, `dt=0.05 s`, control rate 20 Hz
- Trigger threshold 0.15 m, consecutive 3 steps
- Nominal lookahead: `x_traj[10]` (500 ms ahead in MPC plan)

### `waypoint_publisher_node`
- Loads named trajectory (M / S / ...) from `trajectories.py` via `importlib`
- Advances when within 0.15 m of current target
- `start_delay` configurable (12 s in phase2 launcher so trajectory begins after RViz opens)

### `tube_viz_node`
- mRPI cylinder around nominal pose (LP-projected from `Omega_H`/`Omega_h`)
- Current pose / nominal pose / waypoints / triggers as markers
- Persistent markers (`lifetime=0`) except triggers (2 s fade)

> **Note:** Crazyswarm2 is **NOT** used in Phase 2. `cflib_bridge_node` is the sole CFLib client. (Crazyswarm2 + cf2 SITL had motor-activation issues that we diagnosed and bypassed.)

---

## 🎛️ Tuning (no code changes needed)

**Faster tracking** (drone too slow):

```bash
ros2 launch lcs phase2_bringup.launch.py \
    trajectory:=M nominal_lookahead:=15
```

**Earlier waypoint advance:**
Edit `waypoint_threshold` parameter in `phase2_bringup.launch.py` (default 0.15 m, try 0.30 for looser corners).

**More aggressive MPC:**
Edit `mpc_solvers.py` Q matrix (currently position 10× velocity).

**Different arena:**
Edit `x_lb` / `x_ub` in `tube_calculation.py` and rerun:

```bash
cd /root/mpc_ws/src/lcs_tube/simulations && python3 tube_calculation.py
```

Regenerated `tube_data.npz` is loaded by MPC at next run.

---

## 📁 File Locations

```
/root/CrazySim/                              SITL firmware + plugin
/root/crazyflie-clients-python/              cfclient (optional GUI client; not used by phase1/phase2)
/root/mpc_ws/
├── phase1/
│   ├── hover_test.py                        CFLib hover (Phase 1)
│   └── launch_sitl.sh                       SITL launcher (accepts X Y for spawn)
├── phase2/
│   ├── hover_hold.py                        CFLib warm-up (legacy, optional)
│   ├── trajectories.py                      M / S waypoint lists
│   ├── phase2.rviz                          RViz2 config (TubeMarkers + CFOdom + NominalPose)
│   └── README_PHASE2.md                     Phase 2 detail doc
├── src/lcs_tube/lcs/
│   ├── cflib_bridge_node.py                 Direct CFLib bridge (this is the key node)
│   ├── et_tube_mpc_node.py                  ET-Tube MPC controller
│   ├── waypoint_publisher_node.py           Trajectory sequencer
│   ├── tube_viz_node.py                     RViz visualisation
│   └── mpc_solvers.py                       CVXPY QP solver
├── src/lcs_tube/simulations/
│   ├── tube_calculation.py                  mRPI offline computation
│   └── tube_data.npz                        precomputed mRPI + system matrices
└── docs/
    ├── PHASE1_COMPLETE.md                   Phase 1 documentation
    └── PHASE2_COMPLETE.md                   Phase 2 documentation (architecture, bugs, fixes)

/usr/local/bin/phase1                        Phase 1 tmux launcher
/usr/local/bin/phase2                        Phase 2 tmux launcher
/usr/local/bin/stop_sitl                     Stop everything cleanly
```

---

## 🛠️ Troubleshooting

### Drone not moving after `phase2`
- Wait for full timeline (24 s). RViz opens at 20 s, trajectory starts at 24 s.
- **Window 0:** Gazebo unpaused? RTF should be 80–95%.
- **Window 1:** `"CFLIB BRIDGE READY"` must appear (around t=18–19 s).
- If wedged: `stop_sitl`, then `phase2` again.

### `cflib_bridge` dies with `"Connection timed out"` on first param call
The 1 s wait between `open_link` and the first `param.set_value` covers this. If it still happens, increase the wait in `cflib_bridge_node.py` (search for `"Param TOC sometimes isn't ready"`).

### Bridge connects but `/cf_1/odom` stays at 0 Hz (intermittent cf2 first-connect flake)
cf2 SITL sometimes doesn't enable log-block streaming on a fresh first connect. Rerun with the warmup cycle enabled (does an extra throwaway connect-disconnect to wake cf2's plugin):

```bash
ros2 launch lcs phase2_bringup.launch.py trajectory:=M warmup_enabled:=true
```

Or simply: `stop_sitl && phase2 M` (the `stop_sitl` 5 s settle plus a fresh cf2 process usually clears it).

### Topics not flowing

```bash
source /opt/ros/humble/setup.bash
source /root/mpc_ws/install/setup.bash
ros2 topic hz /cf_1/odom         # ~45 Hz
ros2 topic hz /drone/cmd_accel   # 20 Hz
ros2 topic hz /tube_viz          # 10 Hz
```

### RViz shows no markers
- **Fixed Frame** must be `world` (already set in `phase2.rviz`)
- If markers still missing: **Add** → **By Topic** → `/tube_viz` → MarkerArray
- Topic exists? `ros2 topic list | grep tube_viz`

### Wrong cflib (hangs at `_request_protocol_version`)

```bash
cd / && python3 -c "
import cflib, inspect
from cflib.crtp import udpdriver
src = inspect.getsource(udpdriver)
assert '_BASE_PORT = 19850' in src, 'WRONG cflib'
print('cflib OK:', cflib.__file__)
"
```

If **WRONG**:

```bash
pip install --no-deps /root/CrazySim/crazyflie-lib-python
```

### Drone oscillates and waypoints aren't reached
- Trajectory waypoints may exceed the tightened arena. See **Arena Bounds**.
- The default arena is 5 × 5 × 3 m → 4.18 × 4.18 × 1.72 m usable after tube tightening. M/S fit comfortably.

### cf2 won't start (port busy)

```bash
stop_sitl && phase2
```

---

## 📌 Key Facts

- **cflib URI:** `udp://127.0.0.1:19850` (loopback ONLY, not `0.0.0.0`)
- **Drone name:** `cf_1` everywhere
- **cflib:** CrazySim fork, **NOT** pip cflib (different UDP driver)
- **mRPI set:** 72 facets, precomputed offline, in `tube_data.npz`
- **Tube radius** (LP-projected): 0.408 m
- **Gazebo real-time factor:** ~90% (software rendering, normal)
- `cflib_bridge` owns the only CFLib connection; Crazyswarm2 is **NOT** used
- `transforms3d 0.4.x` is required for crazyswarm2 server (numpy 2.x compat)
- `matplotlib >= 3.10` required for pypoman (numpy 2.x compat)

---

## 🐍 Python Dependencies (already installed in this image)

| Package        | Version  | Purpose                                       |
| -------------- | -------- | --------------------------------------------- |
| cflib          | CrazySim fork | from `/root/CrazySim/crazyflie-lib-python` |
| cvxpy          | 1.7.x    | MPC solver                                    |
| osqp           | 1.1.x    | QP backend                                    |
| clarabel       | 0.11.x   | QP backend                                    |
| scipy          | 1.15.x   | arrays, LP                                    |
| numpy          | 2.2.x    | arrays                                        |
| pypoman        | 1.2.x    | polytope ops (only for tube_calc)             |
| pycddlib       | —        | polytope ops                                  |
| cvxopt         | —        | polytope ops                                  |
| matplotlib     | 3.10.x   | numpy 2.x compatible                          |
| transforms3d   | 0.4.x    | numpy 2.x compatible                          |
| rclpy          | —        | ROS 2 client                                  |

Plus: `nav_msgs`, `geometry_msgs`, `std_msgs`, `visualization_msgs`, `crazyflie_interfaces`.

---

## 📚 Based On

- **CrazySim:** Llanes et al., ICRA 2024 — [github.com/gtfactslab/CrazySim](https://github.com/gtfactslab/CrazySim)
- **Tube MPC:** Mayne et al., Automatica 2005
- **ET-Tube MPC:** this implementation
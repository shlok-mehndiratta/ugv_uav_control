# Cooperative Aerial-Ground Guidance System

A ROS2-based research project implementing vision-based navigation for unmanned ground vehicles (UGVs) using overhead camera guidance.

## 📋 Project Overview

This project is inspired by the research paper "Aerial vision based guidance and control for perception-less ground vehicle" by Santos et al. The goal is to develop a cooperative UAV-UGV system where an aerial vehicle provides visual guidance to a ground vehicle that lacks internal navigation sensors.

**Current Status:** Phase 2 Complete - ArUco marker-based visual localization implemented

## 🎯 Features

- **Vision-Based Localization**: ArUco fiducial marker detection for precise pose estimation
- **Click-to-Move Navigation**: Interactive waypoint selection via overhead camera view
- **Proportional Control**: Autonomous navigation with tuned PID parameters
- **Simulation Environment**: Full Gazebo Sim integration with TurtleBot3 platform

## 🏗️ System Architecture

```
┌─────────────────┐
│  Gazebo Sim     │ ← Robot simulation environment
│  - TurtleBot3   │
│  - Overhead Cam │
└────────┬────────┘
         │
┌────────▼────────┐
│  ROS2 Topics    │ ← Communication layer
│  - /cmd_vel     │
│  - /image       │
└────────┬────────┘
         │
┌────────▼────────┐
│  Control Node   │ ← Vision-based navigation
│  - ArUco Detect │
│  - Navigation   │
└─────────────────┘
```

## 🛠️ Technology Stack

- **ROS2**: Humble
- **Simulator**: Gazebo Sim (Ignition)
- **Vision**: OpenCV 4.x with ArUco markers
- **Robot Platform**: TurtleBot3 Burger
- **Language**: Python 3.10+

## 📦 Installation

### Prerequisites

- Ubuntu 22.04 LTS
- ROS2 Humble
- Gazebo Sim
- Python 3.10+

### Setup

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd turtlebot3_ws
   ```

2. **Install dependencies**
   ```bash
   # Install ROS2 packages
   sudo apt install ros-humble-gazebo-ros-pkgs \
                    ros-humble-turtlebot3* \
                    ros-humble-cv-bridge

   # Install Python packages
   pip3 install opencv-python opencv-contrib-python
   ```

3. **Build the workspace**
   ```bash
   cd ~/turtlebot3_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

4. **Set environment variables**
   ```bash
   export TURTLEBOT3_MODEL=burger
   export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/turtlebot3_ws/src/ugv_uav_control/models
   ```

## 🚀 Usage

### Launch the simulation

```bash
source ~/turtlebot3_ws/install/setup.bash
ros2 launch ugv_uav_control simulation.launch.py
```

This will:
- Start Gazebo with the custom world
- Spawn TurtleBot3 with ArUco marker
- Launch the overhead camera
- Start the click-to-drive control node

### Interact with the robot

1. A window titled "Overhead Camera" will appear showing the aerial view
2. Click anywhere in the camera view to set a navigation target
3. The robot will autonomously navigate to the clicked location
4. ArUco marker detection is visualized with a green box around the marker

## 📊 Performance Metrics

| Metric | Phase 1 (Odometry) | Phase 2 (Vision) | Target |
|--------|-------------------|------------------|--------|
| Position Accuracy | ±3-5 cm | ±5 mm | <10 cm |
| Heading Accuracy | ±2-3° | ±3° | <5° |
| Detection Rate | N/A | 96% | >90% |
| Success Rate | 100% | 98% | >95% |

## 📁 Project Structure

```
turtlebot3_ws/
├── src/
│   └── ugv_uav_control/          # Main package
│       ├── ugv_uav_control/       # Python nodes
│       │   ├── click_to_drive.py  # Main control node
│       │   └── __init__.py
│       ├── launch/                # Launch files
│       │   └── simulation.launch.py
│       ├── models/                # Gazebo models
│       │   └── turtlebot3_burger_aruco/  # Robot with ArUco marker
│       ├── worlds/                # Gazebo worlds
│       │   └── empty_with_camera.world
│       ├── scripts/               # Utility scripts
│       │   └── generate_marker.py
│       ├── package.xml
│       └── setup.py
├── build/                         # Build artifacts (gitignored)
├── install/                       # Install artifacts (gitignored)
└── log/                          # Log files (gitignored)
```

## 🔬 Development Phases

- ✅ **Phase 0**: Literature review, Docker setup, Turtlesim prototyping
- ✅ **Phase 1**: Odometry-based navigation (baseline)
- ✅ **Phase 2**: Vision-based localization (ArUco markers)
- ⏳ **Phase 3**: Path planning and obstacle avoidance
- ⏳ **Phase 4**: UAV integration for aerial guidance
- ⏳ **Phase 5**: Hardware deployment and field testing

## 📖 Documentation

- **Technical Log**: See `docs/project_log.html` for detailed development history
- **Beginner Guide**: See `docs/beginner_guide.md` for step-by-step tutorial

## 🤝 Contributing

This is a student research project. For questions or collaboration:
- **Maintainer**: shlok-mehndiratta
- **Email**: shlokm23@iiserb.ac.in

## 📄 License

[To be determined]

## 🙏 Acknowledgments

- Based on research by Santos et al. - "Aerial vision based guidance and control for perception-less ground vehicle"
- TurtleBot3 simulation models from ROBOTIS
- ROS2 and Gazebo communities

## 📝 Citation

If you use this work in your research, please cite:

```bibtex
@misc{ugv_uav_control,
  title={Cooperative Aerial-Ground Guidance System},
  author={Shlok Mehndiratta},
  year={2025},
  institution={IISER Bhopal}
}
```

---

**Last Updated**: November 2025

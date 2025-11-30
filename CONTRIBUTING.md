# Cooperative Aerial-Ground Guidance System

## Quick Start

```bash
# Build the workspace
cd ~/turtlebot3_ws
colcon build --symlink-install
source install/setup.bash

# Set environment
export TURTLEBOT3_MODEL=burger

# Launch simulation
ros2 launch ugv_uav_control simulation.launch.py
```

## Repository Structure

```
turtlebot3_ws/
├── README.md                    # Main project documentation
├── docs/                        # Detailed documentation
│   ├── project_log.html        # Technical development log
│   ├── beginner_guide.md       # Step-by-step tutorial
│   └── README.md               # Documentation guide
├── src/
│   └── ugv_uav_control/        # Main ROS2 package
│       ├── ugv_uav_control/    # Python nodes
│       ├── launch/             # Launch files
│       ├── models/             # Gazebo models
│       ├── worlds/             # Gazebo worlds
│       ├── scripts/            # Utility scripts
│       └── package.xml         # Package metadata
├── build/                       # Build artifacts (gitignored)
├── install/                     # Install artifacts (gitignored)
└── log/                        # Log files (gitignored)
```

## What to Push to GitHub

✅ **Include:**
- `src/ugv_uav_control/` - All source code
- `docs/` - Documentation
- `README.md` - Main documentation
- `.gitignore` - Git exclusions

❌ **Exclude (via .gitignore):**
- `build/` - Build artifacts
- `install/` - Install artifacts
- `log/` - Log files
- `__pycache__/` - Python cache
- `.vscode/` - IDE settings

## Git Workflow

### Initial Setup

```bash
cd ~/turtlebot3_ws
git init
git add .
git commit -m "Initial commit: Phase 2 complete - ArUco-based navigation"
git branch -M main
git remote add origin <your-repo-url>
git push -u origin main
```

### Regular Updates

```bash
# After making changes
git add .
git commit -m "Descriptive message about changes"
git push
```

### Recommended Commit Messages

- `feat: Add obstacle avoidance module`
- `fix: Correct coordinate transformation bug`
- `docs: Update project log with Phase 3 progress`
- `refactor: Improve control loop efficiency`
- `test: Add unit tests for ArUco detection`

## Branches

- `main` - Stable, working code
- `dev` - Development branch for new features
- `feature/name` - Specific feature development

## Version History

- **v0.1.0** (Current) - Phase 2 complete: ArUco-based visual navigation
- **v0.0.1** - Phase 1 complete: Odometry-based navigation
- **v0.0.0** - Initial project setup

---

For detailed documentation, see [README.md](README.md) and [docs/](docs/)

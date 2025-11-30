# Utility Scripts

This directory contains utility scripts for the project.

## 📜 Available Scripts

### generate_marker.py
**Purpose:** Generate ArUco marker images for robot localization

**Usage:**
```bash
python3 scripts/generate_marker.py
```

**What it does:**
- Generates ArUco marker from DICT_4X4_50 dictionary
- Adds white border (quiet zone) for better detection
- Saves to `models/turtlebot3_burger_aruco/materials/textures/aruco_marker_0.png`

**Parameters to modify:**
- `marker_id`: ID of the marker to generate (default: 0)
- `marker_size`: Size of marker in pixels (default: 200)
- `border_size`: White border width in pixels (default: 50)

### test_aruco.py
**Purpose:** Test ArUco marker detection offline

**Usage:**
```bash
python3 scripts/test_aruco.py
```

**What it does:**
- Loads a saved camera snapshot
- Tests ArUco detection with different parameters
- Prints detection statistics
- Useful for debugging detection issues

## 🔧 Adding New Scripts

When adding utility scripts:

1. Place them in this `scripts/` directory
2. Add a description here in the README
3. Make scripts executable if needed: `chmod +x script_name.py`
4. Use clear naming conventions: `verb_noun.py` (e.g., `generate_marker.py`)

## 📝 Script Template

```python
#!/usr/bin/env python3
"""
Script description here.

Usage:
    python3 script_name.py [arguments]
"""

import rclpy
# ... other imports

def main():
    # Script logic here
    pass

if __name__ == '__main__':
    main()
```

---

**Note:** These scripts are utilities and not part of the main ROS2 node execution.

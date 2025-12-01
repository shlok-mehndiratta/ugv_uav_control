import cv2
import cv2.aruco as aruco
import numpy as np
import sys

# Create dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
marker_size = 200 # pixels

try:
    # Try newer API
    marker_image = aruco.generateImageMarker(aruco_dict, 0, marker_size)
except AttributeError:
    try:
        # Try older API
        marker_image = aruco.drawMarker(aruco_dict, 0, marker_size)
    except AttributeError:
        print("Error: Could not find generateImageMarker or drawMarker in cv2.aruco")
        sys.exit(1)

# Add white padding
pad_width = 50
marker_image = cv2.copyMakeBorder(marker_image, pad_width, pad_width, pad_width, pad_width, cv2.BORDER_CONSTANT, value=255)

# Save
# Save
import os
script_dir = os.path.dirname(os.path.abspath(__file__))
# Go up to models/turtlebot3_burger_aruco/materials/textures
output_path = os.path.join(script_dir, '..', 'models', 'turtlebot3_burger_aruco', 'materials', 'textures', 'aruco_marker_0.png')
os.makedirs(os.path.dirname(output_path), exist_ok=True)
cv2.imwrite(output_path, marker_image)
print("Marker generated successfully.")

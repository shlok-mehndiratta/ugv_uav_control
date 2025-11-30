import cv2
import cv2.aruco as aruco
import numpy as np

print(f"OpenCV Version: {cv2.__version__}")

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
try:
    parameters = aruco.DetectorParameters_create()
except AttributeError:
    parameters = aruco.DetectorParameters()

# Generate marker
marker_img = np.zeros((200, 200), dtype=np.uint8)
try:
    marker_img = aruco.drawMarker(aruco_dict, 0, 200)
except AttributeError:
    marker_img = aruco.generateImageMarker(aruco_dict, 0, 200)

# Add white padding
pad_width = 50
marker_padded = cv2.copyMakeBorder(marker_img, pad_width, pad_width, pad_width, pad_width, cv2.BORDER_CONSTANT, value=255)

# Detect
corners, ids, rejected = aruco.detectMarkers(marker_padded, aruco_dict, parameters=parameters)

print(f"Detected IDs: {ids}")

if ids is not None and ids[0] == 0:
    print("Success!")
else:
    print("Failure!")

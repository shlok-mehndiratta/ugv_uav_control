import cv2
import cv2.aruco as aruco
import os

file_path = '/tmp/test_marker.png'

if not os.path.exists(file_path):
    print(f"Error: File not found at {file_path}")
    exit(1)

img = cv2.imread(file_path)
if img is None:
    print("Error: Could not read image.")
    exit(1)

# Resize to avoid potential memory issues and mimic camera view
img = cv2.resize(img, (512, 512))
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

dicts_to_check = {
    "DICT_4X4_50": aruco.DICT_4X4_50,
    "DICT_5X5_50": aruco.DICT_5X5_50,
    "DICT_6X6_50": aruco.DICT_6X6_50,
    "DICT_ARUCO_ORIGINAL": aruco.DICT_ARUCO_ORIGINAL
}

parameters = aruco.DetectorParameters_create() if hasattr(aruco, 'DetectorParameters_create') else aruco.DetectorParameters()

found = False
for name, dict_id in dicts_to_check.items():
    try:
        aruco_dict = aruco.getPredefinedDictionary(dict_id)
        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        
        if ids is not None and len(ids) > 0:
            print(f"Found markers with {name}: IDs {ids.flatten()}")
            found = True
            if name == "DICT_4X4_50" and 0 in ids:
                print("VERIFICATION SUCCESS: Correct Dictionary and ID.")
            elif name != "DICT_4X4_50":
                print(f"WARNING: Marker found but with WRONG dictionary: {name}")
            elif 0 not in ids:
                print(f"WARNING: Marker found with correct dictionary but WRONG ID: {ids.flatten()}")
    except Exception as e:
        print(f"Error checking {name}: {e}")

if not found:
    print("FAILURE: No markers detected with any common dictionary.")


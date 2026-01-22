# scripts/generate_aruco.py
import cv2
import numpy as np

# Generate ArUco markers
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

for i in range(5):
    marker = np.zeros((300, 300), dtype=np.uint8)
    marker = cv2.aruco.drawMarker(aruco_dict, i, 300, marker, 1)
    cv2.imwrite(f'aruco_marker_{i}.png', marker)
    print(f"Generated aruco_marker_{i}.png")

import cv2
import cv2.aruco as aruco
import numpy as np

# Default parameters
aruco_dictionary = aruco.Dictionary_get(aruco.DICT_4X4_250)
detector_parameters = aruco.DetectorParameters_create()
marker_size = 0.33  # Default marker size in meters
distance_coefficients = np.zeros((5, 1))  # Default distortion coefficients
camera_matrix = np.array([[1000.0, 0.0, 320.0], [0.0, 1000.0, 240.0], [0.0, 0.0, 1.0]])  # Default camera matrix

# Map settings
map_path = ""  # Default empty path
length_override = {}  # Dictionary to store marker ID to length mapping 
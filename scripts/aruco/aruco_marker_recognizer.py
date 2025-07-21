import cv2
import cv2.aruco as aruco

import numpy as np

from typing import Tuple, Dict, Optional

class ArucoRecognizer:

    def __init__(self, aruco_dictionary: aruco.DetectorParameters, marker_size: float, distance_coefficients: np.ndarray, 
                 detector_parameters: np.ndarray, camera_matrix: np.ndarray, length_override: Dict[int, float] = None) -> None:
        """
        When initializing an instance of the class, 
        parameters for the detector are passed, 
        as well as data that allows the function 
        to determine the distance and position of the markers

        Args:
            aruco_dictionary: Dictionary of ArUco markers
            marker_size: Default size of markers in meters
            distance_coefficients: Camera distortion coefficients
            detector_parameters: Parameters for ArUco detector
            camera_matrix: Camera intrinsic parameters matrix
            length_override: Dictionary mapping marker IDs to their specific sizes
        """
        
        self.aruco_dictionary = aruco_dictionary
        self.marker_size = marker_size
        self.distance_coefficients = distance_coefficients
        self.detector_parameters = detector_parameters
        self.camera_matrix = camera_matrix
        self.length_override = length_override if length_override is not None else {}


    def detect_aruco_markers(self, frame: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        This function recognizes markers on an image, draws local coordinate systems on it, 
        as well as information (ID and distance). 
        Returns information about markers and rotations 
        and translations matrices if they are recognized, otherwise none

        Args:
            frame(np.array): openCV image from airsim

        Returns:
            ndarray: openCV image   
            ndarray: recognized marker ids      
            ndarray: rotation vector   
            ndarray: translation vector         
        """
        
        markers_corners, markers_ids, rejected_img_points = aruco.detectMarkers(image = frame, 
                                                                                dictionary = self.aruco_dictionary, 
                                                                                parameters = self.detector_parameters)
        
        if markers_ids is not None:
            frame_with_axes = frame.copy()
            rotation_vectors = []
            translation_vectors = []
            
            # Process each marker potentially with its own size
            for i in range(len(markers_ids)):
                marker_id = markers_ids[i][0]
                # Use specific size if available, otherwise default
                marker_size = self.length_override.get(marker_id, self.marker_size)
                
                # Estimate pose for this specific marker
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                    corners=[markers_corners[i]], 
                    markerLength=marker_size,
                    cameraMatrix=self.camera_matrix, 
                    distCoeffs=self.distance_coefficients
                )
                
                rotation_vectors.append(rvec)
                translation_vectors.append(tvec)
                
                frame_with_axes = cv2.drawFrameAxes(image=frame_with_axes, 
                                               cameraMatrix=self.camera_matrix, 
                                               distCoeffs=self.distance_coefficients, 
                                               rvec=rvec[0], 
                                               tvec=tvec[0], 
                                               length=0.02, 
                                               thickness=2)
                
                #biases for drowing text, not for calculating 
                x_bias = 30
                y_bias = -20

                marker_corners = markers_corners[i][0]
                x = int(np.mean(marker_corners[:, 0]))
                y = int(np.mean(marker_corners[:, 1]))

                id_text = 'id:' + str(marker_id)
                cv2.putText(img=frame_with_axes,
                            text=id_text, 
                            org=(x + x_bias, y + y_bias), 
                            fontFace=cv2.FONT_HERSHEY_SIMPLEX, 
                            fontScale=0.75, 
                            color=(0, 255, 255), 
                            thickness=2, 
                            lineType=cv2.LINE_AA)
                
                distance_to_marker = tvec[0][0][2]
                dist_text = 'distance: ' + str(round(distance_to_marker, 3))

                cv2.putText(img=frame_with_axes,
                            text=dist_text, 
                            org=(x + x_bias, y - y_bias), 
                            fontFace=cv2.FONT_HERSHEY_SIMPLEX, 
                            fontScale=0.55, 
                            color=(0, 255, 255), 
                            thickness=2, 
                            lineType=cv2.LINE_AA)
            
            # Convert to numpy arrays with the right shape
            rotation_vectors = np.array(rotation_vectors)
            translation_vectors = np.array(translation_vectors)
            
            return frame_with_axes, markers_ids, rotation_vectors, translation_vectors 
        
        else:
            return None, None, None, None
        
    def set_length_override(self, marker_id: int, length: float) -> None:
        """
        Set a specific length for a marker ID
        
        Args:
            marker_id: ID of the marker
            length: Size of the marker in meters
        """
        self.length_override[marker_id] = length
        
    def clear_length_override(self) -> None:
        """Clear all length overrides"""
        self.length_override.clear()
        
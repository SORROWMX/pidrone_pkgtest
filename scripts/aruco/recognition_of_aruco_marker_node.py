#! /usr/bin/env python3
import rospy
from cv_bridge import CvBridge

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

import numpy as np
import os
import yaml

from aruco_marker_recognizer import ArucoRecognizer
from recognition_setting import aruco_dictionary, detector_parameters, marker_size, distance_coefficients, camera_matrix, length_override

# For dynamic reconfiguration
from dynamic_reconfigure.server import Server
try:
    from aruco_pose.cfg import DetectorConfig
except ImportError:
    rospy.logwarn("aruco_pose/DetectorConfig not found, dynamic reconfiguration won't be available")
    DetectorConfig = None

class RecognitionOfArucoMarker:

    def __init__(self):
        """
        When an instance of the Node class is initialized,
        a publisher and subscript for topics are created, 
        parameters for the detector and a dictionary of markers are also implemented
        """

        rospy.init_node("recognition_of_aruco_marker")
        rospy.loginfo("Recognition_of_aruco_marker_node has been started")

        self.cv_bridge = CvBridge()
        self.enabled = True
        self.marker_size = marker_size
        self.length_override = dict(length_override)  # Create a copy

        # Get parameters from ROS parameter server
        self.map_path = rospy.get_param("~map_path", "")
        self.frame_id_prefix = rospy.get_param("~frame_id_prefix", "aruco_")
        self.send_tf = rospy.get_param("~send_tf", True)
        
        # Изменяем топик для подписки на изображение с камеры
        image_topic = rospy.get_param("~image_topic", "/raspicam_node/image_raw")
        rospy.loginfo(f"Subscribing to image topic: {image_topic}")

        # Подписываемся на топик изображения с raspicam_node
        self.image_subscriber = rospy.Subscriber(image_topic, 
                                                Image, 
                                                self.image_from_sim_callback, 
                                                queue_size=10)
        
        # Публикуем изображение с отмеченными маркерами
        self.image_with_marker_publisher_ = rospy.Publisher("/drone_vision/image_with_marks", 
                                                          Image, 
                                                          queue_size=10)
        
        # Публикуем информацию о положении маркеров
        self.marker_tf_publisher_ = rospy.Publisher("/drone_vision/markers_tf", 
                                                   TransformStamped, 
                                                   queue_size=10)
                                                   
        # Публикуем информацию о маркерах в виде массива для веб-интерфейса
        self.markers_info_publisher = rospy.Publisher("/drone_vision/markers_info", 
                                                     String, 
                                                     queue_size=10)

        # Create services
        self.reload_map_service = rospy.Service('~reload_map', Trigger, self.reload_map_callback)

        # Initialize the recognizer
        self.aruco_recognizer = ArucoRecognizer(
            aruco_dictionary=aruco_dictionary,
            marker_size=self.marker_size,
            distance_coefficients=distance_coefficients,
            detector_parameters=detector_parameters,
            camera_matrix=camera_matrix,
            length_override=self.length_override
        )

        # Load map if specified
        if self.map_path:
            self.load_map(self.map_path)

        # Setup dynamic reconfiguration if available
        if DetectorConfig is not None:
            self.dyn_srv = Server(DetectorConfig, self.param_callback)
        

    def image_from_sim_callback(self, msg_image):
        """
        Gets an image from a topic, finds markers on it,
        saves markers transforms and an image with drawn markers in topics

        Args:
            msg_imag (Image): message with an image from the topic
        """
        if not self.enabled:
            return

        cv_image = self.cv_bridge.imgmsg_to_cv2(msg_image, desired_encoding="rgb8")

        cv_image_with_markers, markers_ids, rotation_vectors, translation_vectors = self.aruco_recognizer.detect_aruco_markers(cv_image)

        if markers_ids is not None:
            image_with_markers_to_msg = self.cv_bridge.cv2_to_imgmsg(cv_image_with_markers, encoding="rgb8")
            self.image_with_marker_publisher_.publish(image_with_markers_to_msg)

            self.position_and_orientation_detecting(markers_ids, rotation_vectors, translation_vectors)
            
            # Публикуем информацию о маркерах для веб-интерфейса
            self.publish_markers_info(markers_ids, translation_vectors)
           

    
    def position_and_orientation_detecting(self, ids, rvec, tvec):
        """
        This function publishes the rotation and the translation matrixs to the TransformStamped topic

        Args:
            ids (ndarray): identifiers of recognized markers
            rvec (ndarray): rotation matrixs
            tvec (ndarray): translation matrix
        """

        if ids is not None:
            for i in range(len(ids)):
                marker_transform = TransformStamped()
                marker_transform.header.frame_id = "camera_frame"
                marker_transform.header.stamp = rospy.Time.now()  # Добавлен timestamp для ROS1
                marker_transform.child_frame_id = self.frame_id_prefix + str(ids[i][0])  # Изменено для совместимости с форматом ids
                marker_transform.transform.translation.x = float(tvec[i][0][0])
                marker_transform.transform.translation.y = float(tvec[i][0][1])
                marker_transform.transform.translation.z = float(tvec[i][0][2])

                marker_transform.transform.rotation.x = float(rvec[i][0][0])
                marker_transform.transform.rotation.y = float(rvec[i][0][1])
                marker_transform.transform.rotation.z = float(rvec[i][0][2])
                marker_transform.transform.rotation.w = 0.0  # Добавлен компонент w для кватерниона в ROS1

                self.marker_tf_publisher_.publish(marker_transform)
    
    def publish_markers_info(self, ids, tvec):
        """
        Публикует информацию о маркерах в формате JSON для веб-интерфейса
        
        Args:
            ids (ndarray): идентификаторы распознанных маркеров
            tvec (ndarray): векторы перемещения
        """
        if ids is not None and len(ids) > 0:
            markers_info = []
            
            for i in range(len(ids)):
                marker_id = int(ids[i][0])
                marker_size = self.length_override.get(marker_id, self.marker_size)
                distance = float(tvec[i][0][2])
                
                marker_info = {
                    "id": marker_id,
                    "size": marker_size,
                    "x": float(tvec[i][0][0]),
                    "y": float(tvec[i][0][1]),
                    "z": distance,
                    "distance": distance
                }
                markers_info.append(marker_info)
            
            # Преобразуем в JSON строку
            import json
            markers_json = json.dumps({"markers": markers_info})
            self.markers_info_publisher.publish(markers_json)

    def load_map(self, map_path):
        """
        Load marker map from a file
        
        Args:
            map_path (str): Path to the map file
        """
        if not os.path.exists(map_path):
            rospy.logerr(f"Map file not found: {map_path}")
            return False
            
        try:
            # Clear existing length overrides
            self.length_override.clear()
            
            with open(map_path, 'r') as f:
                lines = f.readlines()
                
            for line in lines:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                    
                parts = line.split()
                if len(parts) < 2:
                    rospy.logwarn(f"Invalid line in map file: {line}")
                    continue
                    
                try:
                    marker_id = int(parts[0])
                    marker_length = float(parts[1])
                    
                    # Add to length override
                    self.length_override[marker_id] = marker_length
                    rospy.loginfo(f"Loaded marker {marker_id} with size {marker_length}")
                    
                except ValueError:
                    rospy.logwarn(f"Invalid marker data in line: {line}")
                    
            # Update recognizer with new overrides
            self.aruco_recognizer.length_override = self.length_override
            rospy.loginfo(f"Loaded {len(self.length_override)} markers from {map_path}")
            return True
            
        except Exception as e:
            rospy.logerr(f"Error loading map file: {e}")
            return False
            
    def reload_map_callback(self, req):
        """
        Service callback to reload the map file
        """
        response = TriggerResponse()
        if not self.map_path:
            response.success = False
            response.message = "No map path specified"
            return response
            
        success = self.load_map(self.map_path)
        response.success = success
        response.message = "Map reloaded successfully" if success else "Failed to reload map"
        return response
        
    def param_callback(self, config, level):
        """
        Dynamic reconfigure callback
        """
        if hasattr(config, 'enabled'):
            self.enabled = config.enabled
            
        if hasattr(config, 'length'):
            self.marker_size = config.length
            
        # Update detector parameters
        if hasattr(config, 'adaptiveThreshConstant'):
            self.aruco_recognizer.detector_parameters.adaptiveThreshConstant = config.adaptiveThreshConstant
            
        if hasattr(config, 'adaptiveThreshWinSizeMin'):
            self.aruco_recognizer.detector_parameters.adaptiveThreshWinSizeMin = config.adaptiveThreshWinSizeMin
            
        if hasattr(config, 'adaptiveThreshWinSizeMax'):
            self.aruco_recognizer.detector_parameters.adaptiveThreshWinSizeMax = config.adaptiveThreshWinSizeMax
            
        if hasattr(config, 'adaptiveThreshWinSizeStep'):
            self.aruco_recognizer.detector_parameters.adaptiveThreshWinSizeStep = config.adaptiveThreshWinSizeStep
            
        if hasattr(config, 'cornerRefinementMethod'):
            self.aruco_recognizer.detector_parameters.cornerRefinementMethod = config.cornerRefinementMethod
            
        if hasattr(config, 'minMarkerPerimeterRate'):
            self.aruco_recognizer.detector_parameters.minMarkerPerimeterRate = config.minMarkerPerimeterRate
            
        if hasattr(config, 'maxMarkerPerimeterRate'):
            self.aruco_recognizer.detector_parameters.maxMarkerPerimeterRate = config.maxMarkerPerimeterRate
            
        rospy.loginfo("Parameters updated through dynamic reconfiguration")
        return config


def main():
    recognition_of_aruco_marker_node = RecognitionOfArucoMarker()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

if __name__ == '__main__':
    main()
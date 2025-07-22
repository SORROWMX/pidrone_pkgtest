#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, CameraInfo, Image
from cv_bridge import CvBridge

class SimpleArucoDetector:
    def __init__(self):
        rospy.init_node('simple_aruco_detector', anonymous=True)
        
        # Параметры
        self.dictionary_id = rospy.get_param('~dictionary', 0)  # DICT_4X4_50
        self.marker_length = rospy.get_param('~length', 0.33)
        
        # Настройка ArUco детектора
        self.dictionary = cv2.aruco.Dictionary_get(self.dictionary_id)
        self.parameters = cv2.aruco.DetectorParameters_create()
        
        # Подписка на топики
        self.bridge = CvBridge()
        rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, 
                         self.compressed_image_callback, queue_size=1)
        rospy.Subscriber('/raspicam_node/camera_info', CameraInfo, 
                         self.camera_info_callback, queue_size=1)
        
        # Публикация обработанного изображения
        self.debug_pub = rospy.Publisher('~debug', Image, queue_size=1)
        
        # Переменные для хранения данных калибровки
        self.camera_matrix = None
        self.dist_coeffs = None
        
        rospy.loginfo("Simple ArUco detector initialized")
    
    def camera_info_callback(self, msg):
        """Обработка данных калибровки камеры"""
        if self.camera_matrix is None:
            self.camera_matrix = np.reshape(msg.K, (3, 3))
            self.dist_coeffs = np.array(msg.D)
            rospy.loginfo("Camera calibration received")
    
    def compressed_image_callback(self, msg):
        """Обработка сжатого изображения"""
        try:
            # Декодирование сжатого изображения
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # Обнаружение маркеров
            corners, ids, rejected = cv2.aruco.detectMarkers(
                image, self.dictionary, parameters=self.parameters)
            
            # Отрисовка обнаруженных маркеров
            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(image, corners, ids)
                rospy.loginfo(f"Detected markers: {ids.flatten()}")
                
                # Оценка поз маркеров, если доступна калибровка
                if self.camera_matrix is not None and self.dist_coeffs is not None:
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners, self.marker_length, self.camera_matrix, self.dist_coeffs)
                    
                    # Отрисовка осей и вывод информации
                    for i in range(len(ids)):
                        marker_id = ids[i][0]
                        tvec = tvecs[i][0]
                        cv2.aruco.drawAxis(image, self.camera_matrix, self.dist_coeffs, 
                                         rvecs[i], tvecs[i], self.marker_length/2)
                        rospy.loginfo(f"Marker {marker_id} position: X={tvec[0]:.3f}, Y={tvec[1]:.3f}, Z={tvec[2]:.3f}")
            
            # Публикация обработанного изображения
            debug_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)
            
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

if __name__ == '__main__':
    try:
        detector = SimpleArucoDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
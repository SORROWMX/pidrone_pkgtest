import cv2
import cv2.aruco as aruco
import numpy as np

# Проверяем версию OpenCV и используем соответствующий API
cv_version = cv2.__version__.split('.')
major_version = int(cv_version[0])
minor_version = int(cv_version[1]) if len(cv_version) > 1 else 0

print(f"OpenCV version: {cv2.__version__}")

# Default parameters
try:
    # Для OpenCV 4.x
    if major_version >= 4:
        aruco_dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        detector_parameters = cv2.aruco.DetectorParameters_create()
    # Для OpenCV 3.x
    else:
        aruco_dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        detector_parameters = aruco.DetectorParameters_create()
except AttributeError:
    # Альтернативный способ для некоторых версий
    try:
        aruco_dictionary = aruco.Dictionary_get(aruco.DICT_4X4_50)
        detector_parameters = aruco.DetectorParameters_create()
    except AttributeError:
        print("Ошибка: не удалось создать словарь ArUco и параметры детектора")
        print(f"Версия OpenCV: {cv2.__version__}")
        # Попытка использовать другие методы
        try:
            aruco_dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
            detector_parameters = aruco.createDetectorParameters()
        except:
            print("Критическая ошибка при создании параметров ArUco")
            aruco_dictionary = None
            detector_parameters = None

# Параметры маркеров и камеры
marker_size = 0.33  # Default marker size in meters
distance_coefficients = np.zeros((5, 1))  # Default distortion coefficients
camera_matrix = np.array([[1000.0, 0.0, 320.0], [0.0, 1000.0, 240.0], [0.0, 0.0, 1.0]])  # Default camera matrix

# Map settings
map_path = ""  # Default empty path
length_override = {}  # Dictionary to store marker ID to length mapping 
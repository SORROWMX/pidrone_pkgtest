#!/bin/bash

# Скрипт для настройки символических ссылок для aruco_pose

# Создаем директорию если она не существует
mkdir -p ~/catkin_ws/devel/lib/aruco_pose

# Копируем aruco_utils.py в директорию, где его ищет Python
cp ~/catkin_ws/src/aruco_pose/src/aruco_utils.py ~/catkin_ws/devel/lib/aruco_pose/

# Делаем скрипт исполняемым
chmod +x ~/catkin_ws/devel/lib/aruco_pose/aruco_utils.py

echo "Setup completed. Now try running roslaunch aruco_pose aruco.launch again." 
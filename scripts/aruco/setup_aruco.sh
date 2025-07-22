#!/bin/bash

# Скрипт для настройки символических ссылок для aruco_pose

echo "Setting up aruco_pose package..."

# Проверяем, существует ли исходный файл
SRC_FILE=~/catkin_ws/src/aruco_pose/src/aruco_utils.py
if [ ! -f "$SRC_FILE" ]; then
    echo "Error: Source file $SRC_FILE not found!"
    exit 1
fi

# Создаем директорию если она не существует
TARGET_DIR=~/catkin_ws/devel/lib/aruco_pose
mkdir -p $TARGET_DIR

# Создаем символическую ссылку на aruco_utils.py
TARGET_FILE=$TARGET_DIR/aruco_utils.py
if [ -f "$TARGET_FILE" ]; then
    echo "Removing existing file: $TARGET_FILE"
    rm "$TARGET_FILE"
fi

echo "Creating symbolic link: $SRC_FILE -> $TARGET_FILE"
ln -s "$SRC_FILE" "$TARGET_FILE"

# Делаем скрипт исполняемым
chmod +x "$SRC_FILE"

echo "Creating __init__.py file in target directory"
touch "$TARGET_DIR/__init__.py"

echo "Setup completed. Now try running roslaunch aruco_pose aruco.launch again." 
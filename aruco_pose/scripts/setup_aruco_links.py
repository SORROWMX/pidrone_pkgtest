#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Скрипт для создания символических ссылок для aruco_pose
Выполняется автоматически при установке пакета
"""

import os
import sys
import shutil
from pathlib import Path

def main():
    """Создает символические ссылки для модуля aruco_utils"""
    print("Setting up aruco_pose symbolic links...")
    
    # Определяем пути
    try:
        # Получаем путь к исходному коду пакета
        package_path = os.environ.get('CATKIN_PACKAGE_PATH')
        if not package_path:
            # Если переменная окружения не установлена, пробуем найти путь через ROS
            import rospkg
            rospack = rospkg.RosPack()
            package_path = rospack.get_path('aruco_pose')
        
        # Получаем путь к devel/lib
        catkin_ws = os.environ.get('CATKIN_WORKSPACE')
        if not catkin_ws:
            # Если не указан, используем стандартный путь
            home = str(Path.home())
            catkin_ws = os.path.join(home, 'catkin_ws')
        
        devel_lib = os.path.join(catkin_ws, 'devel', 'lib')
        
        # Создаем директорию для aruco_pose, если она не существует
        target_dir = os.path.join(devel_lib, 'aruco_pose')
        os.makedirs(target_dir, exist_ok=True)
        
        # Пути к файлам
        src_file = os.path.join(package_path, 'src', 'aruco_utils.py')
        target_file = os.path.join(target_dir, 'aruco_utils.py')
        init_file = os.path.join(target_dir, '__init__.py')
        
        # Создаем символическую ссылку
        if os.path.exists(target_file):
            print(f"Removing existing file: {target_file}")
            os.remove(target_file)
        
        print(f"Creating symbolic link: {src_file} -> {target_file}")
        os.symlink(src_file, target_file)
        
        # Создаем пустой файл __init__.py
        with open(init_file, 'w') as f:
            f.write("# This file makes the directory a Python package\n")
        
        print("Setup completed successfully.")
        return 0
    
    except Exception as e:
        print(f"Error during setup: {e}")
        return 1

if __name__ == '__main__':
    sys.exit(main()) 
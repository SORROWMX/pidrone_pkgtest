#!/usr/bin/env python

"""
Тестовый скрипт для получения hover_throttle через MSP протокол
и интеграции этого значения с PID контроллером.
"""

import sys
import time
import rospy
from h2rMultiWii import MultiWii
from pid_class_new import PID, PIDaxis, HOVER_THROTTLE, DEADBAND

def main():
    # Инициализация ROS ноды
    rospy.init_node('test_hover_throttle', anonymous=True)
    
    # Параметр serial_port может быть передан как аргумент командной строки
    if len(sys.argv) > 1:
        serial_port = sys.argv[1]
    else:
        # По умолчанию используем /dev/ttyACM0
        serial_port = '/dev/ttyACM0'
    
    print(f"Подключение к полетному контроллеру через порт {serial_port}...")
    
    try:
        # Инициализация подключения к полетному контроллеру
        board = MultiWii(serial_port)
        
        # Получение информации о версии прошивки
        print("Запрос идентификатора контроллера...")
        ident = board.getData(MultiWii.IDENT)
        print(f"Информация о контроллере: {ident}")
        
        # Получение текущего значения hover_throttle из INAV через MSP
        print("\nЗапуск теста получения hover_throttle...")
        hover_throttle_inav = board.test_get_hover_throttle()
        
        # Получение текущего значения из настроек PID контроллера
        print("\nТекущие значения в PID контроллере:")
        print(f"HOVER_THROTTLE = {HOVER_THROTTLE}")
        print(f"DEADBAND = {DEADBAND}")
        
        # Проверка соответствия значений
        if hover_throttle_inav is not None:
            if hover_throttle_inav == HOVER_THROTTLE:
                print("\nЗначения совпадают: PID контроллер уже использует правильное значение hover_throttle")
            else:
                print("\nЗначения не совпадают:")
                print(f"  - INAV hover_throttle: {hover_throttle_inav}")
                print(f"  - PID hover_throttle: {HOVER_THROTTLE}")
                print("\nРекомендуется обновить значение HOVER_THROTTLE в pid_class_new.py")
        
        # Получение данных с датчиков высоты
        print("\nЗапрос данных с барометра...")
        altitude_data = board.getData(MultiWii.ALTITUDE)
        print(f"Данные высоты: {altitude_data}")
        
        # Тестирование работы PID контроллера с текущими настройками
        print("\nСоздание тестового PID контроллера с текущими настройками...")
        test_pid = PID()
        
        # Проверка значения hover_throttle в PID контроллере
        print(f"Значение hover_throttle в созданном PID: {test_pid.throttle.hover_throttle}")
        print(f"Значение deadband в созданном PID: {test_pid.throttle.deadband}")
        
        # Создание тестового примера для PID контроллера
        if altitude_data and 'estalt' in altitude_data:
            current_height = altitude_data['estalt'] / 100.0  # convert from cm to m
            
            # Симуляция PID шага для текущей высоты
            from three_dim_vec import Error
            error = Error(0, 0, 0.65 - current_height)
            
            print(f"\nТестирование PID контроллера с текущей высотой: {current_height}м")
            print(f"Целевая высота: 0.65м, ошибка: {error.z}м")
            
            # Выполняем один шаг PID контроллера
            cmd = test_pid.step(error)
            print(f"Результат PID.step(): {cmd}")
            print(f"Команда газа (throttle): {cmd[2]}")
            
            # Для тестирования, что будет если дрон на земле
            ground_error = Error(0, 0, 0.65 - 0.02)  # дрон на земле, высота 2см
            print("\nТестирование с симуляцией дрона на земле (высота 2см):")
            ground_cmd = test_pid.step(ground_error)
            print(f"Результат PID.step() для дрона на земле: {ground_cmd}")
            print(f"Команда газа (throttle): {ground_cmd[2]}")
            
    except Exception as e:
        import traceback
        print(f"Ошибка при выполнении теста: {e}")
        traceback.print_exc()
    finally:
        # Закрываем соединение с полетным контроллером
        if 'board' in locals():
            board.close()
            print("Соединение с полетным контроллером закрыто")

if __name__ == "__main__":
    main() 
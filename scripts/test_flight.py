#!/usr/bin/env python3

import rospy
import time
import sys
import subprocess
import signal
import os
from unavlib import MSPy
from unavlib.control.uavcontrol import UAVControl
from std_srvs.srv import Empty

# Define MSP codes directly as in test_msp2_altitude_control.py
MSP_API_VERSION = 1
MSP_FC_VARIANT = 2
MSP_FC_VERSION = 3
MSP_BOARD_INFO = 4
MSP_STATUS = 101
MSP_BOXNAMES = 116
MSP_BOXIDS = 119
MSP_SET_BOX = 203
MSP_MODE_RANGES = 34
MSP2_INAV_SET_ALTITUDE = 0x2220  # 8736 decimal

def arm_via_rc(msp_board, arm=True):
    # Создаем массив RC каналов с дефолтными значениями
    rc_channels = [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000]
    
    # Находим, какой канал отвечает за ARM
    arm_channel_index = None
    
    # Получаем информацию о режимах
    if msp_board.send_RAW_msg(MSP_MODE_RANGES, data=[]):
        dataHandler = msp_board.receive_msg()
        msp_board.process_recv_data(dataHandler)
    
    # Ищем ARM в режимах активации
    for i, mode in enumerate(msp_board.CONFIG.get('modeRanges', [])):
        if mode.get('id') == 0:  # BOXARM имеет id=0
            arm_channel_index = mode.get('auxChannel') + 4  # +4 потому что первые 4 канала это RPYT
            arm_range_start = mode.get('start') * 25 + 900
            arm_range_end = mode.get('end') * 25 + 900
            break
    
    if arm_channel_index is not None:
        # Устанавливаем значение канала ARM
        rc_channels[arm_channel_index] = 1900 if arm else 1000  # Значение для ARM/DISARM
        
        # Отправляем команду RC
        if msp_board.send_RAW_RC(rc_channels):
            dataHandler = msp_board.receive_msg()
            return msp_board.process_recv_data(dataHandler)
    
    return False

def arm_drone_and_set_altitude():
    # Инициализация ROS ноды
    rospy.init_node('arm_and_altitude_test', anonymous=True)
    
    # Параметры подключения
    serial_port = rospy.get_param('~serial_port', '/dev/ttyACM0')
    baudrate = rospy.get_param('~baudrate', 115200)
    
    rospy.loginfo("Подключение к полетному контроллеру на порту {}".format(serial_port))
    
    try:
        # Подключение к полетному контроллеру с улучшенной обработкой ошибок
        board = None
        try:
            rospy.loginfo("Подключение к полетному контроллеру на {}".format(serial_port))
            board = UAVControl(serial_port, baudrate, receiver="serial")
            board.connect()
            rospy.loginfo("Подключение к полетному контроллеру успешно")
        except Exception as e:
            rospy.logerr("Не удалось подключиться к полетному контроллеру: {}".format(e))
            try:
                # Пробуем альтернативный порт
                alt_port = '/dev/ttyACM1'
                rospy.loginfo("Пробуем альтернативный порт {}".format(alt_port))
                board = UAVControl(alt_port, baudrate, receiver="serial")
                board.connect()
                rospy.loginfo("Подключение к полетному контроллеру на альтернативном порту успешно")
            except Exception as e:
                rospy.logerr("Не удалось подключиться к полетному контроллеру на альтернативном порту: {}".format(e))
                rospy.signal_shutdown("Не удалось подключиться к полетному контроллеру")
                return
        
        # Получаем доступ к низкоуровневому объекту MSPy
        msp_board = board.board
            
        # Получаем базовую информацию от контроллера полета
        command_list = ['MSP_API_VERSION', 'MSP_FC_VARIANT', 'MSP_FC_VERSION', 
                       'MSP_BOARD_INFO', 'MSP_STATUS', 'MSP_MODE_RANGES']
        
        # Create a dictionary mapping command names to codes
        msp_codes = {
            'MSP_API_VERSION': MSP_API_VERSION,
            'MSP_FC_VARIANT': MSP_FC_VARIANT,
            'MSP_FC_VERSION': MSP_FC_VERSION,
            'MSP_BOARD_INFO': MSP_BOARD_INFO,
            'MSP_STATUS': MSP_STATUS,
            'MSP_MODE_RANGES': MSP_MODE_RANGES
        }
        
        for msg in command_list:
            if msp_board.send_RAW_msg(msp_codes[msg], data=[]):
                dataHandler = msp_board.receive_msg()
                msp_board.process_recv_data(dataHandler)
        
        # Проверка флагов, запрещающих ARM
        arm_disable_flags = msp_board.process_armingDisableFlags(msp_board.CONFIG['armingDisableFlags'])
        if arm_disable_flags:
            rospy.logwarn("Внимание: Обнаружены флаги, запрещающие ARM: {}".format(arm_disable_flags))
            rospy.loginfo("Попытка ARM будет выполнена, но может не сработать из-за этих флагов")
        
        # Арм дрона через эмуляцию RC команд
        rospy.loginfo("Армирование дрона через эмуляцию RC команд...")
        if arm_via_rc(msp_board, arm=True):
            rospy.loginfo("Команда ARM отправлена")
        else:
            rospy.logerr("Не удалось отправить команду ARM")
            return

        # Ждем немного для обработки FC
        time.sleep(1.0)

        # Проверяем статус ARM
        if msp_board.send_RAW_msg(msp_codes['MSP_STATUS'], data=[]):
            dataHandler = msp_board.receive_msg()
            msp_board.process_recv_data(dataHandler)

        # Проверяем статус ARM
        is_armed = msp_board.bit_check(msp_board.CONFIG["mode"], 0)
        if is_armed:
            rospy.loginfo("Дрон успешно армирован!")
        else:
            rospy.logerr("Не удалось армировать дрон!")
            return

        # ВАЖНО: После арма БОЛЬШЕ НЕ ОТПРАВЛЯЕМ RC команды!
        # Дальше работаем только через MSP2_INAV_SET_ALTITUDE

        # Запускаем altitude controller с целевой высотой 0.8 метра
        rospy.loginfo("Запускаем altitude controller с целевой высотой 0.8 метра...")

        # Устанавливаем параметр целевой высоты
        rospy.set_param('/altitude_controller/target_altitude', 0.8)

        # Запускаем altitude controller с помощью rosrun, который обеспечит правильное окружение ROS
        # Используем bash для запуска с правильным окружением
        cmd = "rosrun pidrone_pkg test_msp2_altitude_control.py _serial_port:={} _baudrate:={}".format(serial_port, baudrate)
        altitude_process = subprocess.Popen(
            cmd,
            shell=True,
            env=dict(os.environ)
        )

        rospy.loginfo("Altitude controller запущен")

        # Ждем 5 секунд для инициализации контроллера высоты (увеличиваем время ожидания)
        time.sleep(5.0)

        try:
            # Проверяем доступность сервиса
            rospy.wait_for_service('/altitude_controller/set_target', timeout=5.0)
            
            # Вызываем сервис для установки целевой высоты
            set_target_service = rospy.ServiceProxy('/altitude_controller/set_target', Empty)
            
            rospy.loginfo("Вызываем сервис установки целевой высоты 0.8м...")
            rospy.set_param('/altitude_controller/target_altitude', 0.8)
            set_target_service()
            rospy.loginfo("Команда на установку высоты 0.8м отправлена")
            
            # Ждем 15 секунд на высоте 0.8м
            rospy.loginfo("Ожидаем 15 секунд на высоте 0.8м...")
            time.sleep(15)
            
            # Плавное снижение с более мелкими шагами
            heights = [0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.15, 0.1, 0.05]
            for height in heights:
                rospy.loginfo("Снижаемся до высоты {}м...".format(height))
                rospy.set_param('/altitude_controller/target_altitude', height)
                set_target_service()
                # Ждем 3 секунды на каждой высоте
                time.sleep(3)
            
            # Финальная посадка (высота 0)
            rospy.loginfo("Выполняем посадку (высота 0м)...")
            rospy.set_param('/altitude_controller/target_altitude', 0.0)
            set_target_service()
            
            # Ждем 10 секунд после посадки перед дизармом
            rospy.loginfo("Ожидаем 10 секунд после посадки перед дизармом...")
            time.sleep(10)
            
        except rospy.ServiceException as e:
            rospy.logerr("Не удалось вызвать сервис: {}".format(e))
        except rospy.ROSException as e:
            rospy.logerr("Ошибка ROS: {}".format(e))
        finally:
            # Останавливаем altitude controller
            if altitude_process:
                altitude_process.send_signal(signal.SIGINT)
                altitude_process.wait()
                rospy.loginfo("Altitude controller остановлен")
            
            # Дизармим дрон через эмуляцию RC команд
            rospy.loginfo("Дизармим дрон...")
            if arm_via_rc(msp_board, arm=False):
                rospy.loginfo("Команда DISARM отправлена")
            else:
                rospy.logerr("Не удалось отправить команду DISARM")
            
            # Проверяем статус ARM после дизарма
            time.sleep(0.5)
            if msp_board.send_RAW_msg(msp_codes['MSP_STATUS'], data=[]):
                dataHandler = msp_board.receive_msg()
                msp_board.process_recv_data(dataHandler)
            
            is_armed = msp_board.bit_check(msp_board.CONFIG["mode"], 0)
            if not is_armed:
                rospy.loginfo("Дрон успешно дизармирован")
            else:
                rospy.logwarn("Дрон все еще армирован!")
                
            # Отключаемся от полетного контроллера
            try:
                board.disconnect()
                rospy.loginfo("Отключение от полетного контроллера выполнено успешно")
            except Exception as e:
                rospy.logwarn("Ошибка при отключении от полетного контроллера: {}".format(e))
    
    except Exception as e:
        rospy.logerr("Произошла ошибка: {}".format(e))
        import traceback
        rospy.logerr(traceback.format_exc())

if __name__ == '__main__':
    try:
        arm_drone_and_set_altitude()
    except rospy.ROSInterruptException:
        pass
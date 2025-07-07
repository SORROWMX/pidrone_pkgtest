from inavmspapi import MultirotorControl
from inavmspapi.transmitter import TCPTransmitter
from agrotechsimapi import SimClient
import time
import math
import numpy as np

# Настройка соединения
HOST = '127.0.0.1'
PORT = 5762
ADDRESS = (HOST, PORT)

# Целевая высота, которую хотим удерживать (в метрах)
TARGET_HEIGHT = 0.02

# Параметры PID-регулятора - экстремально быстрая реакция
KP = 90.0  # Пропорциональный коэффициент
KI = 0.1   # Интегральный коэффициент
KD = 45.0  # Дифференциальный коэффициент

# Параметры для позиционного контроля
POS_KP = 900.0  # Базовый коэффициент для позиционирования
POSITION_THRESHOLD = 0.015  # Порог достижения цели 5 мм

# Пороги точности для навигации (в метрах)
PRECISION_THRESHOLD_HIGH = 0.005   # Высокая точность (5 мм)
PRECISION_THRESHOLD_MEDIUM = 0.015  # Средняя точность (15 мм)
PRECISION_THRESHOLD_LOW = 0.05     # Низкая точность (50 мм)
PRECISION_TIMEOUT = 30.0           # Максимальное время попытки достижения точки (сек)

# Пример координатных точек для полета (x, y, z, yaw)
# X - отрицательные значения = влево, положительные = вправо
# Y - отрицательные значения = назад, положительные = вперед
# Z - высота (положительные значения = вверх)
COORDINATE_WAYPOINTS = [
    (-0.02, 0.09, TARGET_HEIGHT, 0),   
    (-0.05, 0.1, TARGET_HEIGHT, 0),
    (-0.05, 0.02, TARGET_HEIGHT, 0),
    (-0.075, 0.02, TARGET_HEIGHT, 0),
    (-0.075, 0.085, TARGET_HEIGHT, 0),
    (0, 0.01, TARGET_HEIGHT, 0),
]

def calculate_throttle(current_height, previous_height, previous_error, integral, dt):
    # Вычисляем скорость изменения высоты
    height_change_rate = (current_height - previous_height) / max(dt, 0.0001)
    
    # Экстренное торможение только при очень быстром подъеме
    if current_height > TARGET_HEIGHT * 1.5 or height_change_rate > 0.05:
        return 1270, previous_error, integral
    
    # Вычисляем ошибку
    error = TARGET_HEIGHT - current_height
    
    # Сглаживание ошибки для уменьшения резких изменений
    smoothed_error = error * 0.8 + previous_error * 0.2
    
    # PID-составляющие с учетом времени цикла
    # Более медленное накопление интеграла
    integral = integral + error * dt * 0.5  
    integral = max(-0.1, min(integral, 0.1))  # Более жесткое ограничение интеграла
    
    # Используем сглаженную производную для уменьшения шума
    derivative = (smoothed_error - previous_error) / max(dt, 0.0001)
    
    # Вычисляем throttle значение на основе PID-регулятора
    p_term = KP * smoothed_error
    i_term = KI * integral
    d_term = KD * derivative
    
    # Более сильное демпфирование колебаний по высоте
    rate_term = -100 * height_change_rate
    
    # Адаптивное управление в зависимости от положения и скорости
    if current_height > TARGET_HEIGHT:
        # Если выше цели
        if height_change_rate > 0:
            # Если продолжаем подниматься - агрессивное торможение
            base_throttle = 1250
            throttle_adjustment = p_term + i_term + d_term + rate_term
            throttle = base_throttle + int(throttle_adjustment * 0.1)
            
            # Дополнительное торможение при подъеме
            throttle -= int(height_change_rate * 350)
        else:
            # Если уже снижаемся - плавное управление
            base_throttle = 1375
            throttle_adjustment = p_term + i_term + d_term + rate_term
            throttle = base_throttle + int(throttle_adjustment * 0.15)
    else:
        # Если ниже цели - плавное управление
        # Адаптивный базовый throttle в зависимости от расстояния до цели
        height_diff = abs(TARGET_HEIGHT - current_height)
        
        if height_diff > 0.01:
            # Значительное отклонение - более сильная реакция
            base_throttle = 1400
            gain = 0.8
        else:
            # Незначительное отклонение - тонкая настройка
            base_throttle = 1375
            gain = 0.6
            
        throttle_adjustment = p_term + i_term + d_term + rate_term
        throttle = base_throttle + int(throttle_adjustment * gain)
    
    # Более плавное ограничение throttle
    throttle = max(1100, min(throttle, 1425))
    
    return throttle, smoothed_error, integral

# Функция для расчета управляющих сигналов для достижения заданных координат
def navigate_to_position(current_pos, target_pos, orientation):
    """
    Рассчитывает управляющие сигналы для перемещения к заданной точке
    
    current_pos - текущие координаты (x, y, z)
    target_pos - целевые координаты (x, y, z)
    orientation - текущая ориентация дрона (roll, pitch, yaw)
    
    Возвращает: (roll, pitch) значения для управления
    """
    # Вычисляем разницу между целевой и текущей позицией
    # X - отрицательные значения = влево, положительные = вправо
    # Y - отрицательные значения = назад, положительные = вперед
    dx = target_pos[0] - current_pos[0]  # X - влево/вправо
    dy = target_pos[1] - current_pos[1]  # Y - вперед/назад
    
    # Определяем текущий угол рыскания (yaw) в радианах
    yaw_rad = math.radians(orientation[2])
    
    # Преобразуем разницу координат с учетом ориентации дрона
    # Эти формулы переводят перемещение из глобальной системы координат в локальную систему дрона
    dx_local = dx * math.cos(yaw_rad) + dy * math.sin(yaw_rad)
    dy_local = -dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)
    
    # Рассчитываем управляющие сигналы с пропорциональным регулированием
    # Добавляем нелинейное усиление для больших расстояний
    distance = math.sqrt(dx*dx + dy*dy)
    
    # Динамическая корректировка коэффициентов в зависимости от расстояния
    # Чем ближе к цели, тем точнее и агрессивнее управление
    if distance > 0.1:
        # Далеко от цели - максимальная скорость
        effective_kp = POS_KP * 1.8
        gain_factor = 0.95
    elif distance > 0.02:
        # Средняя дистанция - сбалансированное управление
        effective_kp = POS_KP * 1.5  # Увеличен с 1.3
        gain_factor = 0.9  # Увеличен с 0.85
    elif distance > 0.01:
        # Близко к цели - агрессивное точное позиционирование
        effective_kp = POS_KP * 2.2  # Увеличен для более агрессивного наведения
        gain_factor = 1.0  # Максимальный коэффициент усиления
    else:
        # Очень близко к цели - сверхточное позиционирование
        effective_kp = POS_KP * 2.5  # Максимальная агрессивность на малых расстояниях
        gain_factor = 1.1  # Сверхусиление для точного наведения
    
    # Применяем нелинейную функцию для более резкого изменения при разных расстояниях
    # Для малых расстояний используем квадратичную зависимость для большей точности
    if distance < 0.01:
        # Квадратичная зависимость для сверхточного позиционирования
        position_gain = min(1.2, (distance * 50) ** 0.5) * gain_factor
    else:
        # Линейная зависимость для обычного позиционирования
        position_gain = min(0.95, distance * 20) * gain_factor
    
    # Рассчитываем управляющие сигналы с учетом динамических параметров
    # Roll (крен) - влево/вправо (X), Pitch (тангаж) - вперед/назад (Y)
    # Используем одинаковый коэффициент для обоих направлений
    roll_value = 1500 + int(effective_kp * dx_local * position_gain)  # X - влево/вправо (крен)
    pitch_value = 1500 + int(effective_kp * dy_local * position_gain)  # Y - вперед/назад (тангаж)
    
    # Более агрессивное ограничение значений для быстрого реагирования
    # Для очень малых расстояний расширяем диапазон для более точного позиционирования
    if distance < 0.01:
        roll_value = max(1150, min(roll_value, 1950))  # Расширенный диапазон для сверхточного позиционирования
        pitch_value = max(1200, min(pitch_value, 2000))
    else:
        roll_value = max(1200, min(roll_value, 1900))
        pitch_value = max(1250, min(pitch_value, 1950))
    
    return roll_value, pitch_value

# Пример функции для полета по координатам
def coordinate_flight():
    """
    Функция для полета по заданным координатам
    """
    # Инициализация подключения
    tcp_transmitter = TCPTransmitter(ADDRESS)
    tcp_transmitter.connect()
    control = MultirotorControl(tcp_transmitter)
    client = SimClient(address="127.0.0.1", port=8080)
    
    print(f"Полет по координатам с удержанием высоты {TARGET_HEIGHT}м")
    
    # ARM дрона и перевод в MSP режим
    control.send_RAW_RC([1500, 1500, 1000, 1500, 1000, 1000, 1000])
    control.receive_msg()
    control.send_RAW_RC([1500, 1500, 1000, 1500, 2000, 1000, 1000])
    control.receive_msg()
    
    # Начальные значения для PID-регулятора
    previous_error = 0
    integral = 0
    previous_height = 0
    last_time = time.time()
    
    # Массивы для хранения истории полета
    position_history = []
    orientation_history = []
    
    # Получаем начальное положение
    result = client.get_kinametics_data()
    start_position = result['location'].copy()
    print(f"Начальное положение: X={start_position[0]:.4f}, Y={start_position[1]:.4f}, Z={start_position[2]:.4f}")
    
    # Начальная стабилизация высоты
    print("Стабилизация высоты перед полетом...")
    stabilization_start = time.time()
    
    while time.time() - stabilization_start < 3:  # Увеличиваем время стабилизации до 3 секунд
        result = client.get_kinametics_data()
        current_position = result['location']
        current_orientation = result['orientation']
        current_height = current_position[2]
        
        # Запоминаем координаты
        position_history.append(current_position.copy())
        orientation_history.append(current_orientation.copy())
        
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time
        
        throttle, previous_error, integral = calculate_throttle(
            current_height, previous_height, previous_error, integral, dt
        )
        previous_height = current_height
        
        control.send_RAW_RC([1500, 1500, throttle, 1500, 2000, 1000, 1000])
        control.receive_msg()
        
        print(f"Стабилизация | Позиция: X={current_position[0]:.4f}, Y={current_position[1]:.4f}, Z={current_height:.4f} | Throttle: {throttle}")
        time.sleep(0.005)  # Уменьшаем задержку для более быстрого реагирования
    
    # Полет по точкам-координатам
    for i, (target_x, target_y, target_z, target_yaw) in enumerate(COORDINATE_WAYPOINTS):
        print(f"\nПеремещение к точке {i+1}: X={target_x:.4f}, Y={target_y:.4f}, Z={target_z:.4f}")
        
        # Получаем текущее положение перед началом движения к точке
        result = client.get_kinametics_data()
        start_pos_for_waypoint = result['location'].copy()
        
        # Рассчитываем полное расстояние до цели
        dx_total = target_x - start_pos_for_waypoint[0]
        dy_total = target_y - start_pos_for_waypoint[1]
        total_distance = math.sqrt(dx_total*dx_total + dy_total*dy_total)
        
        print(f"Общее расстояние до точки {i+1}: {total_distance:.4f}м")
        
        # Время начала движения к точке
        waypoint_start = time.time()
        
        # Счетчик стабильности (сколько раз подряд были близко к цели)
        stability_counter = 0
        required_stability = 30  # Требуемое количество стабильных итераций
        
        # Флаги для отслеживания достижения разных уровней точности
        high_precision_achieved = False
        medium_precision_achieved = False
        low_precision_achieved = False
        
        # Время достижения разных уровней точности
        low_precision_time = None
        medium_precision_time = None
        
        print(f"Начинаем движение к точке {i+1}...")
        
        # Основной цикл движения к точке
        while time.time() - waypoint_start < PRECISION_TIMEOUT:
            # Получаем текущие данные о положении дрона
            result = client.get_kinametics_data()
            current_position = result['location']
            current_orientation = result['orientation']
            current_height = current_position[2]
            
            # Запоминаем координаты
            position_history.append(current_position.copy())
            orientation_history.append(current_orientation.copy())
            
            # Вычисляем время цикла
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            
            # Рассчитываем throttle для удержания высоты
            throttle, previous_error, integral = calculate_throttle(
                current_height, previous_height, previous_error, integral, dt
            )
            previous_height = current_height
            
            # Рассчитываем управляющие сигналы для достижения целевой точки
            roll, pitch = navigate_to_position(
                current_position, 
                [target_x, target_y, target_z],
                current_orientation
            )
            
            # Отправляем команду управления дроном
            control.send_RAW_RC([roll, pitch, throttle, 1500 + target_yaw, 2000, 1000, 1000])
            control.receive_msg()
            
            # Вычисляем расстояние до цели
            dx = target_x - current_position[0]  # X - отрицательные значения = влево, положительные = вправо
            dy = target_y - current_position[1]  # Y - отрицательные значения = назад, положительные = вперед
            distance_to_target = math.sqrt(dx*dx + dy*dy)  # Учитываем только X и Y координаты
            
            # Отслеживаем достижение разных уровней точности
            if not low_precision_achieved and distance_to_target < PRECISION_THRESHOLD_LOW:
                low_precision_achieved = True
                low_precision_time = time.time() - waypoint_start
                print(f"Достигнута низкая точность ({PRECISION_THRESHOLD_LOW*1000:.0f} мм) за {low_precision_time:.2f} сек")
                
            if not medium_precision_achieved and distance_to_target < PRECISION_THRESHOLD_MEDIUM:
                medium_precision_achieved = True
                medium_precision_time = time.time() - waypoint_start
                print(f"Достигнута средняя точность ({PRECISION_THRESHOLD_MEDIUM*1000:.0f} мм) за {medium_precision_time:.2f} сек")
            
            # Выводим более точные координаты с прогрессом
            print(f"К точке {i+1}  Дист: {distance_to_target:.4f}м | " +
                  f"Поз: X={current_position[0]:.4f}, Y={current_position[1]:.4f}, Z={current_height:.4f} | " +
                  f"Цель: X={target_x:.4f}, Y={target_y:.4f} | " +
                  f"Roll: {roll}, Pitch: {pitch}, Throttle: {throttle}")
            
            # Проверка на достижение высокой точности
            if distance_to_target < PRECISION_THRESHOLD_HIGH:
                stability_counter += 1
                if stability_counter >= required_stability:
                    high_precision_achieved = True
                    print(f"Достигнута высокая точность ({PRECISION_THRESHOLD_HIGH*1000:.0f} мм)! Стабильное положение подтверждено.")
                    break
            else:
                # Сбрасываем счетчик, если отдалились от цели
                stability_counter = 0
            
            # Если мы уже достигли средней точности и прошло больше 10 секунд,
            # но высокая точность не достигается, прекращаем попытки
            if medium_precision_achieved and (time.time() - waypoint_start) > (medium_precision_time + 10.0):
                print(f"Не удалось достичь высокой точности за 10 секунд после достижения средней. Продолжаем полет.")
                break
                
            # Если прошло больше 15 секунд с момента достижения низкой точности,
            # но средняя точность не достигается, прекращаем попытки
            if low_precision_achieved and not medium_precision_achieved and (time.time() - waypoint_start) > (low_precision_time + 15.0):
                print(f"Не удалось достичь средней точности за 15 секунд после достижения низкой. Продолжаем полет.")
                break
            
            time.sleep(0.005)  # Уменьшаем задержку для более быстрого реагирования
        
        # Проверяем, достигли ли мы хотя бы низкой точности
        if not low_precision_achieved:
            print(f"Внимание! Не удалось достичь даже низкой точности ({PRECISION_THRESHOLD_LOW*1000:.0f} мм) за {PRECISION_TIMEOUT} секунд.")
            print(f"Минимальное достигнутое расстояние: {distance_to_target:.4f}м")
        
        # Стабилизация в достигнутой точке
        print(f"Стабилизация в точке {i+1}...")
        stabilize_start = time.time()
        
        while time.time() - stabilize_start < 1.0:  # Уменьшаем время стабилизации до 1 секунды
            result = client.get_kinametics_data()
            current_position = result['location']
            current_orientation = result['orientation']
            current_height = current_position[2]
            
            # Запоминаем координаты
            position_history.append(current_position.copy())
            orientation_history.append(current_orientation.copy())
            
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            
            throttle, previous_error, integral = calculate_throttle(
                current_height, previous_height, previous_error, integral, dt
            )
            previous_height = current_height
            
            # Продолжаем удерживать позицию с небольшими корректировками
            roll, pitch = navigate_to_position(
                current_position, 
                [target_x, target_y, target_z],
                current_orientation
            )
            
            # Уменьшаем влияние roll и pitch для более стабильного зависания
            roll = int(1500 + (roll - 1500) * 0.5)
            pitch = int(1500 + (pitch - 1500) * 0.5)
            
            control.send_RAW_RC([roll, pitch, throttle, 1500, 2000, 1000, 1000])
            control.receive_msg()
            
            print(f"Стабилизация в точке {i+1} | Поз: X={current_position[0]:.4f}, Y={current_position[1]:.4f}, Z={current_height:.4f}")
            time.sleep(0.005)  # Уменьшаем задержку для более быстрого реагирования
    
    # Завершение полета и посадка (очень плавная)
    print("\nПосадка...")
    landing_start = time.time()
    landing_duration = 10.0  # Увеличиваем время посадки до 10 секунд для большей плавности
    
    # Сбрасываем интеграл для посадки, чтобы избежать накопленных ошибок
    integral = 0
    
    # Запоминаем начальную высоту для расчета посадки
    result = client.get_kinametics_data()
    start_landing_height = result['location'][2]
    print(f"Начальная высота посадки: {start_landing_height:.4f}м")
    
    # Коэффициенты PID для посадки (более мягкие)
    landing_kp = 15.0  # Уменьшенный P-коэффициент для более плавной реакции
    landing_ki = 0.01  # Минимальный I-коэффициент
    landing_kd = 35.0  # Увеличенный D-коэффициент для лучшего демпфирования
    
    while time.time() - landing_start < landing_duration:
        # Постепенно снижаем целевую высоту до 0 с нелинейным профилем для большей плавности
        elapsed = time.time() - landing_start
        progress = elapsed / landing_duration
        
        # Применяем нелинейную функцию для более плавного снижения в конце
        smooth_progress = math.pow(progress, 1.5)  # Более быстрое снижение в начале
        current_target = start_landing_height * (1.0 - smooth_progress)
        
        # Ограничиваем минимальную целевую высоту для безопасности
        current_target = max(0.0, current_target)
        
        # Получаем текущие данные о положении дрона
        result = client.get_kinametics_data()
        current_position = result['location']
        current_orientation = result['orientation']
        current_height = current_position[2]
        
        # Запоминаем координаты
        position_history.append(current_position.copy())
        orientation_history.append(current_orientation.copy())
        
        # Вычисляем время цикла
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time
        
        # Вычисляем скорость изменения высоты
        height_change_rate = (current_height - previous_height) / max(dt, 0.0001)
        previous_height = current_height
        
        # Вычисляем ошибку относительно постепенно снижающейся цели
        error = current_target - current_height
        
        # PID-составляющие с учетом времени цикла и более мягкими параметрами
        integral = integral + error * dt * 0.3  # Медленнее накапливаем интеграл
        integral = max(-5, min(integral, 5))  # Жесткое ограничение интеграла
        derivative = (error - previous_error) / max(dt, 0.0001)
        
        # Вычисляем throttle с мягкими параметрами для плавной посадки
        p_term = landing_kp * error
        i_term = landing_ki * integral
        d_term = landing_kd * derivative
        
        # Более сильное демпфирование для предотвращения колебаний
        rate_term = -60 * height_change_rate
        
        # Базовый throttle с постепенным снижением
        # Начинаем с меньшего значения, чтобы сразу начать снижение
        base_throttle = 1300 - int(300 * smooth_progress)
        
        # Если дрон начинает подниматься, агрессивно снижаем throttle
        if height_change_rate > 0.005:
            throttle_adjustment = p_term + i_term + d_term + rate_term - 200
            # Используем более низкий базовый throttle при подъеме
            base_throttle = 1200
        else:
            throttle_adjustment = p_term + i_term + d_term + rate_term
        
        throttle = base_throttle + int(throttle_adjustment * 0.5)  # Уменьшаем влияние PID
        
        # Более жесткое ограничение throttle для посадки
        # Постепенно снижаем максимальный throttle по мере снижения
        max_throttle = 1350 - int(300 * smooth_progress)
        throttle = max(1000, min(throttle, max_throttle))
        
        # Отправляем команду управления дроном с минимальным креном и тангажом
        control.send_RAW_RC([1500, 1500, throttle, 1500, 2000, 1000, 1000])
        control.receive_msg()
        
        # Сохраняем текущую ошибку
        previous_error = error
        
        print(f"Посадка: {progress*100:.0f}% | Цель: {current_target:.4f}м | Поз: Z={current_height:.4f}м | " +
              f"Скорость: {height_change_rate:.4f}м/с | Throttle: {throttle}")
        
        time.sleep(0.005)  # Уменьшаем задержку для более быстрого реагирования
    
    # Финальное снижение газа для очень мягкого касания
    print("\nФинальное касание...")
    
    # Проверяем, что дрон уже находится на очень низкой высоте
    result = client.get_kinametics_data()
    current_height = result['location'][2]
    
    # Если дрон еще высоко, выполняем дополнительное снижение
    if current_height > 0.002:
        # Быстрое снижение до почти касания земли
        for i in range(20):
            throttle = 1050 - i * 2
            control.send_RAW_RC([1500, 1500, throttle, 1500, 2000, 1000, 1000])
            control.receive_msg()
            
            # Получаем текущую высоту для контроля
            result = client.get_kinametics_data()
            current_height = result['location'][2]
            print(f"Дополнительное снижение: {i+1}/20 | Throttle: {throttle} | Высота: {current_height:.4f}м")
            
            if current_height <= 0.001:
                break
                
            time.sleep(0.1)
    
    # Очень плавное снижение газа с большим количеством шагов
    for throttle_percent in range(100, -1, -1):  # Уменьшаем с шагом 1%
        actual_throttle = 1030 + int(throttle_percent * 0.2)  # От 1030 до 1000
        control.send_RAW_RC([1500, 1500, actual_throttle, 1500, 2000, 1000, 1000])
        control.receive_msg()
        
        # Получаем текущую высоту для контроля
        result = client.get_kinametics_data()
        current_height = result['location'][2]
        print(f"Финальное снижение: {throttle_percent}% | Throttle: {actual_throttle} | Высота: {current_height:.4f}м")
        
        time.sleep(0.05)  # Увеличиваем задержку для более плавного снижения
    
    # Отключение моторов
    for i in range(5):  # Отправляем команду несколько раз для надежности
        control.send_RAW_RC([1500, 1500, 1000, 1500, 2000, 1000, 1000])
        control.receive_msg()
        time.sleep(0.1)
    
    # Выводим итоговую информацию о полете
    print("\nПолет по координатам завершен!")
    print(f"Пройденный путь: {len(position_history)} точек")
    print(f"Начальная позиция: X={position_history[0][0]:.4f}, Y={position_history[0][1]:.4f}, Z={position_history[0][2]:.4f}")
    print(f"Конечная позиция: X={position_history[-1][0]:.4f}, Y={position_history[-1][1]:.4f}, Z={position_history[-1][2]:.4f}")
    
    # Анализируем точность достижения целей
    print("\nАнализ точности достижения целевых точек:")
    accuracy_results = analyze_flight_accuracy(position_history, COORDINATE_WAYPOINTS)
    
    for result in accuracy_results:
        i = result['waypoint_index']
        target = result['target']
        best_pos = result['best_position']
        distance = result['best_distance']
        
        print(f"Точка {i+1}:")
        print(f"  Целевые координаты: X={target[0]:.4f}, Y={target[1]:.4f}, Z={target[2]:.4f}")
        print(f"  Наилучшее приближение: X={best_pos[0]:.4f}, Y={best_pos[1]:.4f}, Z={best_pos[2]:.4f}")
        print(f"  Погрешность (XY): {distance:.4f}м")
        print(f"  Погрешность по высоте: {abs(target[2] - best_pos[2]):.4f}м")
        
        # Оценка точности
        if distance < POSITION_THRESHOLD:
            print(f"  Оценка: Отлично! Точка достигнута с высокой точностью.")
        elif distance < POSITION_THRESHOLD * 3:
            print(f"  Оценка: Хорошо. Точка достигнута с приемлемой точностью.")
        elif distance < POSITION_THRESHOLD * 10:
            print(f"  Оценка: Удовлетворительно. Есть погрешность в позиционировании.")
        else:
            print(f"  Оценка: Требуется улучшение. Значительное отклонение от цели.")

def analyze_flight_accuracy(position_history, waypoints):
    """
    Анализирует точность достижения целевых точек
    
    position_history - массив с историей позиций дрона
    waypoints - массив целевых точек
    
    Возвращает статистику по каждой точке
    """
    results = []
    
    for i, (target_x, target_y, target_z, _) in enumerate(waypoints):
        # Инициализируем переменные для поиска лучшего приближения к точке
        best_distance = float('inf')
        best_position = None
        best_index = -1
        
        # Проходим по всей истории позиций
        for j, pos in enumerate(position_history):
            dx = target_x - pos[0]
            dy = target_y - pos[1]
            # Учитываем только X и Y координаты для расчета расстояния
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance < best_distance:
                best_distance = distance
                best_position = pos
                best_index = j
        
        # Добавляем результат для этой точки
        results.append({
            'waypoint_index': i,
            'target': (target_x, target_y, target_z),
            'best_position': best_position,
            'best_distance': best_distance,
            'time_index': best_index
        })
    
    return results

if __name__ == "__main__":
    coordinate_flight() 
#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.gridspec import GridSpec
import time
import sys
import os

# Импортируем автономную версию PID контроллера
from pid_standalone import PIDaxis, TARGET_HEIGHT

# Простая структура для хранения ошибок
class Error:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

class DroneSimulator:
    def __init__(self):
        # Параметры симуляции
        self.dt = 0.01  # шаг времени (секунды)
        self.sim_time = 60  # общее время симуляции (секунды)
        self.time_points = np.arange(0, self.sim_time, self.dt)
        self.num_points = len(self.time_points)
        
        # Параметры дрона
        self.mass = 0.5  # масса дрона (кг)
        self.gravity = 9.81  # ускорение свободного падения (м/с²)
        self.max_thrust = 2.0 * self.mass * self.gravity  # максимальная тяга (Н)
        self.drag_coefficient = 0.1  # коэффициент сопротивления воздуха
        
        # Начальные условия
        self.height = 0.0  # начальная высота (м)
        self.velocity = 0.0  # начальная скорость (м/с)
        self.acceleration = 0.0  # начальное ускорение (м/с²)
        
        # Массивы для хранения данных симуляции
        self.heights = np.zeros(self.num_points)
        self.velocities = np.zeros(self.num_points)
        self.accelerations = np.zeros(self.num_points)
        self.throttle_commands = np.zeros(self.num_points)
        self.target_heights = np.zeros(self.num_points)
        
        # Инициализация PID контроллера
        self.pid_controller = PIDaxis(60.0, 0.12, 40.0, 
                                     i_range=(-400, 400), 
                                     control_range=(1100, 1900),
                                     midpoint=1450)
        self.pid_controller.is_throttle_controller = True
        
        # Параметры INAV-подобного поведения
        self.throttle_hover = 1450  # приблизительное значение для зависания
        self.throttle_min = 1400    # минимальное значение дросселя
        self.throttle_max = 1500    # максимальное значение дросселя
        
        # Для визуализации
        self.fig = None
        self.axes = None
        self.lines = None
        self.animation = None
        
        # Сценарии для тестирования
        self.scenarios = [
            {"name": "Взлет и зависание", "changes": [(0, TARGET_HEIGHT)]},
            {"name": "Ступенчатое изменение высоты", "changes": [(0, 0.5), (10, 1.0), (20, 0.7), (30, 1.2), (40, 0.3)]},
            {"name": "Резкое изменение высоты", "changes": [(0, 0.5), (15, 1.5), (30, 0.3)]},
            {"name": "Плавное изменение высоты", "changes": [(0, 0.5)], "sine_wave": True},
            {"name": "Внешние возмущения", "changes": [(0, 0.75)], "disturbances": [(10, -0.5), (25, 0.7), (40, -0.3)]},
            {"name": "Тест конкретных значений дросселя", "manual_throttle": True, "throttle_values": [
                (0, 1450),    # Начинаем с hover
                (5, 1470),    # Увеличиваем до 1470
                (10, 1450),   # Возвращаемся к hover
                (15, 1430),   # Снижаем до 1430
                (20, 1450),   # Возвращаемся к hover
                (25, 1480),   # Значительно увеличиваем
                (30, 1450),   # Возвращаемся к hover
                (35, 1420),   # Значительно снижаем
                (40, 1450),   # Возвращаемся к hover
                (45, 1460),   # Немного увеличиваем
                (50, 1440),   # Немного снижаем
                (55, 1450),   # Возвращаемся к hover
            ]}
        ]
        
        # Текущий сценарий
        self.current_scenario = None
        
    def throttle_to_thrust(self, throttle):
        """
        Convert throttle command to thrust in Newtons.
        
        Based on real drone behavior:
        - At 1400-1410: drone starts to lift off but not enough thrust
        - At 1450: stable hover
        - Below 1400: not enough thrust to lift off
        - Above 1450: ascent
        """
        # Calculate hover thrust (N) - thrust needed to maintain hover
        hover_thrust = self.mass * self.gravity
        
        # Define throttle range based on real drone behavior
        throttle_min = 1400  # Minimum throttle that produces some lift
        throttle_hover = 1450  # Hover throttle
        throttle_max = 1500  # Maximum throttle
        
        # Calculate thrust with realistic mapping
        if throttle < throttle_min:
            # Below minimum - not enough to lift off
            return 0.8 * hover_thrust * (throttle / throttle_min)
        elif throttle < throttle_hover:
            # Between min and hover - gradual increase to hover thrust
            throttle_range = throttle_hover - throttle_min
            throttle_ratio = (throttle - throttle_min) / throttle_range
            # Non-linear mapping for more realistic behavior
            thrust_ratio = 0.8 + 0.2 * (throttle_ratio ** 1.5)
            return hover_thrust * thrust_ratio
        else:
            # Above hover - linear increase in thrust
            throttle_range = throttle_max - throttle_hover
            throttle_ratio = min(1.0, (throttle - throttle_hover) / throttle_range)
            # Maximum thrust is 1.5x hover thrust
            max_thrust_ratio = 1.5
            thrust_ratio = 1.0 + (max_thrust_ratio - 1.0) * throttle_ratio
            return hover_thrust * thrust_ratio
    
    def update_physics(self, throttle, external_force=0):
        """
        Обновляет физическое состояние дрона на основе команды дросселя
        и внешних сил (если есть)
        """
        # Преобразуем команду дросселя в тягу
        thrust = self.throttle_to_thrust(throttle)
        
        # Вычисляем силу сопротивления воздуха (пропорциональна скорости)
        drag = self.drag_coefficient * self.velocity * abs(self.velocity)
        
        # Суммарная сила (тяга - вес + внешняя сила - сопротивление)
        total_force = thrust - (self.mass * self.gravity) + external_force - drag
        
        # Вычисляем ускорение (F = ma)
        self.acceleration = total_force / self.mass
        
        # Обновляем скорость (v = v0 + a*t)
        self.velocity += self.acceleration * self.dt
        
        # Обновляем высоту (h = h0 + v*t + 0.5*a*t^2)
        self.height += self.velocity * self.dt + 0.5 * self.acceleration * self.dt**2
        
        # Не позволяем дрону уйти ниже земли
        if self.height < 0:
            self.height = 0
            self.velocity = 0
            self.acceleration = 0
            
    def get_target_height(self, time):
        """
        Возвращает целевую высоту в зависимости от текущего времени и сценария
        """
        if not self.current_scenario:
            return TARGET_HEIGHT
            
        base_height = 0
        
        # Применяем изменения высоты из сценария
        for change_time, height in self.current_scenario["changes"]:
            if time >= change_time:
                base_height = height
                
        # Добавляем синусоидальное изменение, если указано в сценарии
        if self.current_scenario.get("sine_wave", False):
            # Синусоидальное изменение с периодом 20 секунд и амплитудой 0.3 метра
            base_height += 0.3 * np.sin(2 * np.pi * time / 20)
            
        return base_height
    
    def get_external_force(self, time):
        """
        Возвращает внешнюю силу (возмущение) в зависимости от текущего времени и сценария
        """
        force = 0
        
        # Если в сценарии есть возмущения
        if self.current_scenario and "disturbances" in self.current_scenario:
            for dist_time, dist_force in self.current_scenario["disturbances"]:
                # Применяем импульсное возмущение в течение 1 секунды
                if dist_time <= time < dist_time + 1:
                    force = dist_force * self.mass  # Сила в Ньютонах (F = m*a)
                    
        return force
    
    def analyze_throttle_data(self):
        """
        Анализирует данные о командах дросселя и выводит статистику
        """
        # Проверяем, есть ли данные для анализа
        if not hasattr(self, 'throttle_commands') or len(self.throttle_commands) == 0:
            print("Нет данных о командах дросселя для анализа")
            return
            
        # Базовая статистика
        min_throttle = np.min(self.throttle_commands)
        max_throttle = np.max(self.throttle_commands)
        avg_throttle = np.mean(self.throttle_commands)
        
        # Анализ по фазам полета
        # Разделим полет на фазы: взлет, стабилизация, маневры
        
        # Определяем фазу взлета (первые 20% времени или до достижения 90% целевой высоты)
        takeoff_end_idx = min(
            int(len(self.time_points) * 0.2),  # 20% времени
            next((i for i, h in enumerate(self.heights) if h >= self.target_heights[i] * 0.9), len(self.heights) - 1)
        )
        
        # Фаза стабилизации - следующие 30% времени или когда высота в пределах 5% от целевой
        stabilization_end_idx = min(
            takeoff_end_idx + int(len(self.time_points) * 0.3),
            next((i for i in range(takeoff_end_idx, len(self.heights)) 
                 if abs(self.heights[i] - self.target_heights[i]) <= 0.05 * self.target_heights[i]), 
                len(self.heights) - 1)
        )
        
        # Оставшееся время - фаза маневров
        
        # Статистика по фазам
        takeoff_throttles = self.throttle_commands[:takeoff_end_idx]
        stabilization_throttles = self.throttle_commands[takeoff_end_idx:stabilization_end_idx]
        maneuver_throttles = self.throttle_commands[stabilization_end_idx:]
        
        # Вывод результатов
        print("\n===== АНАЛИЗ КОМАНД ДРОССЕЛЯ =====")
        print(f"Общая статистика:")
        print(f"  Минимальный дроссель: {min_throttle:.1f}")
        print(f"  Максимальный дроссель: {max_throttle:.1f}")
        print(f"  Средний дроссель: {avg_throttle:.1f}")
        
        if len(takeoff_throttles) > 0:
            print(f"\nФаза взлета (0-{self.time_points[takeoff_end_idx]:.1f} с):")
            print(f"  Мин: {np.min(takeoff_throttles):.1f}, Макс: {np.max(takeoff_throttles):.1f}, Средний: {np.mean(takeoff_throttles):.1f}")
        
        if len(stabilization_throttles) > 0:
            print(f"\nФаза стабилизации ({self.time_points[takeoff_end_idx]:.1f}-{self.time_points[stabilization_end_idx]:.1f} с):")
            print(f"  Мин: {np.min(stabilization_throttles):.1f}, Макс: {np.max(stabilization_throttles):.1f}, Средний: {np.mean(stabilization_throttles):.1f}")
        
        if len(maneuver_throttles) > 0:
            print(f"\nФаза маневров ({self.time_points[stabilization_end_idx]:.1f}-{self.time_points[-1]:.1f} с):")
            print(f"  Мин: {np.min(maneuver_throttles):.1f}, Макс: {np.max(maneuver_throttles):.1f}, Средний: {np.mean(maneuver_throttles):.1f}")
        
        # Анализ распределения значений дросселя
        print("\nРаспределение значений дросселя:")
        ranges = [(1400, 1425), (1425, 1450), (1450, 1475), (1475, 1500)]
        for start, end in ranges:
            count = np.sum((self.throttle_commands >= start) & (self.throttle_commands < end))
            percentage = count / len(self.throttle_commands) * 100
            print(f"  {start}-{end}: {count} команд ({percentage:.1f}%)")
            
        print("===================================\n")
        
    def save_throttle_data(self, filename=None):
        """
        Сохраняет данные о командах дросселя в CSV-файл
        
        Args:
            filename: имя файла для сохранения. Если None, генерируется автоматически.
        """
        if not hasattr(self, 'throttle_commands') or len(self.throttle_commands) == 0:
            print("Нет данных о командах дросселя для сохранения")
            return
            
        # Если имя файла не указано, генерируем его на основе текущего времени и сценария
        if filename is None:
            scenario_name = self.current_scenario["name"].replace(" ", "_").lower()
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f"throttle_data_{scenario_name}_{timestamp}.csv"
        
        # Создаем данные для сохранения
        data = np.column_stack((
            self.time_points,
            self.heights,
            self.target_heights,
            self.velocities,
            self.accelerations,
            self.throttle_commands
        ))
        
        # Сохраняем в CSV
        try:
            header = "Time,Height,TargetHeight,Velocity,Acceleration,Throttle"
            np.savetxt(filename, data, delimiter=',', header=header, comments='')
            print(f"Данные успешно сохранены в файл: {filename}")
        except Exception as e:
            print(f"Ошибка при сохранении данных: {e}")
            
    def get_manual_throttle(self, time):
        """
        Возвращает значение дросселя для ручного режима управления
        на основе текущего времени и заданных в сценарии значений
        """
        if not self.current_scenario or not self.current_scenario.get("manual_throttle"):
            return self.throttle_hover
            
        throttle_values = self.current_scenario.get("throttle_values", [])
        if not throttle_values:
            return self.throttle_hover
            
        # Находим последнее значение дросселя, которое должно быть применено
        last_throttle = self.throttle_hover
        for t, throttle in throttle_values:
            if time >= t:
                last_throttle = throttle
            else:
                break
                
        return last_throttle
        
    def run_simulation(self, scenario_index=0, save_data=False):
        """
        Запускает симуляцию с выбранным сценарием
        
        Args:
            scenario_index: индекс сценария для симуляции
            save_data: если True, данные будут сохранены в CSV-файл
        """
        if scenario_index < 0 or scenario_index >= len(self.scenarios):
            print("Неверный индекс сценария")
            return
            
        self.current_scenario = self.scenarios[scenario_index]
        print(f"Запуск сценария: {self.current_scenario['name']}")
        
        # Сбрасываем начальные условия
        self.height = 0.0
        self.velocity = 0.0
        self.acceleration = 0.0
        self.pid_controller.reset()
        
        # Проверяем, используется ли ручной режим управления дросселем
        is_manual_throttle = self.current_scenario.get("manual_throttle", False)
        
        # Основной цикл симуляции
        for i, t in enumerate(self.time_points):
            # Получаем целевую высоту для текущего времени
            target_height = self.get_target_height(t)
            self.target_heights[i] = target_height
            
            # Определяем команду дросселя
            if is_manual_throttle:
                # В ручном режиме берем значение из сценария
                throttle = self.get_manual_throttle(t)
            else:
                # В автоматическом режиме используем PID контроллер
                # Вычисляем ошибку (в см для совместимости с PID контроллером)
                error = (target_height - self.height) * 100
                throttle = self.pid_controller.adaptive_height_step(error, self.dt)
                
            self.throttle_commands[i] = throttle
            
            # Получаем внешнюю силу (если есть)
            external_force = self.get_external_force(t)
            
            # Обновляем физическое состояние дрона
            self.update_physics(throttle, external_force)
            
            # Сохраняем данные для визуализации
            self.heights[i] = self.height
            self.velocities[i] = self.velocity
            self.accelerations[i] = self.acceleration
            
        # Анализируем данные о командах дросселя
        self.analyze_throttle_data()
        
        # Сохраняем данные, если требуется
        if save_data:
            self.save_throttle_data()
        
        # Визуализируем результаты
        self.visualize_results()
            
    def visualize_results(self):
        """
        Визуализирует результаты симуляции
        """
        plt.style.use('ggplot')
        self.fig = plt.figure(figsize=(12, 10))
        gs = GridSpec(5, 1, height_ratios=[3, 1, 1, 1, 2])  # Добавляем еще одну секцию для гистограммы
        
        # График высоты
        ax1 = plt.subplot(gs[0])
        ax1.plot(self.time_points, self.heights, 'b-', linewidth=2, label='Фактическая высота')
        ax1.plot(self.time_points, self.target_heights, 'r--', linewidth=1.5, label='Целевая высота')
        ax1.set_ylabel('Высота (м)')
        ax1.set_title(f'Симуляция дрона с INAV: {self.current_scenario["name"]}')
        ax1.legend(loc='upper right')
        ax1.grid(True)
        
        # График скорости
        ax2 = plt.subplot(gs[1], sharex=ax1)
        ax2.plot(self.time_points, self.velocities, 'g-', linewidth=1.5)
        ax2.set_ylabel('Скорость (м/с)')
        ax2.grid(True)
        
        # График ускорения
        ax3 = plt.subplot(gs[2], sharex=ax1)
        ax3.plot(self.time_points, self.accelerations, 'm-', linewidth=1.5)
        ax3.set_ylabel('Ускорение (м/с²)')
        ax3.grid(True)
        
        # График команд дросселя
        ax4 = plt.subplot(gs[3], sharex=ax1)
        ax4.plot(self.time_points, self.throttle_commands, 'k-', linewidth=1.5)
        ax4.axhline(y=self.throttle_hover, color='r', linestyle='--', alpha=0.5, label='Hover')
        ax4.set_ylabel('Дроссель')
        ax4.set_xlabel('Время (с)')
        ax4.set_ylim(self.throttle_min - 10, self.throttle_max + 10)
        ax4.grid(True)
        ax4.legend(loc='upper right')
        
        # Гистограмма распределения значений дросселя
        ax5 = plt.subplot(gs[4])
        bins = np.arange(self.throttle_min - 10, self.throttle_max + 15, 5)  # Шаг 5 единиц
        n, bins, patches = ax5.hist(self.throttle_commands, bins=bins, alpha=0.7, color='darkblue')
        
        # Добавляем вертикальную линию для значения hover
        ax5.axvline(x=self.throttle_hover, color='r', linestyle='--', alpha=0.7, label='Hover')
        
        # Добавляем текстовые метки с процентами для каждого столбца
        for i in range(len(n)):
            if n[i] > 0:  # Только для непустых столбцов
                percentage = 100 * n[i] / len(self.throttle_commands)
                if percentage > 3:  # Показываем метки только для значимых столбцов
                    ax5.text(bins[i] + 2.5, n[i] + 5, f'{percentage:.1f}%', 
                             ha='center', va='bottom', fontsize=8)
        
        ax5.set_xlabel('Значение дросселя')
        ax5.set_ylabel('Частота')
        ax5.set_title('Распределение команд дросселя')
        ax5.grid(True)
        ax5.legend()
        
        plt.tight_layout()
        plt.show()
        
    def create_animation(self, scenario_index=0):
        """
        Создает анимацию симуляции дрона
        """
        if scenario_index < 0 or scenario_index >= len(self.scenarios):
            print("Неверный индекс сценария")
            return
            
        self.current_scenario = self.scenarios[scenario_index]
        print(f"Создание анимации для сценария: {self.current_scenario['name']}")
        
        # Сбрасываем начальные условия
        self.height = 0.0
        self.velocity = 0.0
        self.acceleration = 0.0
        self.pid_controller.reset()
        
        # Подготовка графиков
        plt.style.use('ggplot')
        self.fig = plt.figure(figsize=(12, 10))
        gs = GridSpec(4, 1, height_ratios=[3, 1, 1, 1])
        
        # График высоты
        ax1 = plt.subplot(gs[0])
        line_height, = ax1.plot([], [], 'b-', linewidth=2, label='Фактическая высота')
        line_target, = ax1.plot([], [], 'r--', linewidth=1.5, label='Целевая высота')
        ax1.set_ylabel('Высота (м)')
        ax1.set_title(f'Симуляция дрона с INAV: {self.current_scenario["name"]}')
        ax1.legend(loc='upper right')
        ax1.grid(True)
        ax1.set_xlim(0, self.sim_time)
        ax1.set_ylim(0, 2)
        
        # График скорости
        ax2 = plt.subplot(gs[1], sharex=ax1)
        line_velocity, = ax2.plot([], [], 'g-', linewidth=1.5)
        ax2.set_ylabel('Скорость (м/с)')
        ax2.grid(True)
        ax2.set_ylim(-2, 2)
        
        # График ускорения
        ax3 = plt.subplot(gs[2], sharex=ax1)
        line_acceleration, = ax3.plot([], [], 'm-', linewidth=1.5)
        ax3.set_ylabel('Ускорение (м/с²)')
        ax3.grid(True)
        ax3.set_ylim(-5, 5)
        
        # График команд дросселя
        ax4 = plt.subplot(gs[3], sharex=ax1)
        line_throttle, = ax4.plot([], [], 'k-', linewidth=1.5)
        ax4.axhline(y=1500, color='r', linestyle='--', alpha=0.5)  # Линия зависания
        ax4.set_ylabel('Дроссель')
        ax4.set_xlabel('Время (с)')
        ax4.set_ylim(self.throttle_min, self.throttle_max)
        ax4.grid(True)
        
        self.axes = [ax1, ax2, ax3, ax4]
        self.lines = [line_height, line_target, line_velocity, line_acceleration, line_throttle]
        
        # Инициализация данных
        self.heights = np.zeros(self.num_points)
        self.velocities = np.zeros(self.num_points)
        self.accelerations = np.zeros(self.num_points)
        self.throttle_commands = np.zeros(self.num_points)
        self.target_heights = np.zeros(self.num_points)
        
        # Функция инициализации анимации
        def init():
            for line in self.lines:
                line.set_data([], [])
            return self.lines
        
        # Функция обновления анимации
        def update(frame):
            # Индекс текущего кадра
            i = frame
            
            # Если это первый кадр, сбрасываем начальные условия
            if i == 0:
                self.height = 0.0
                self.velocity = 0.0
                self.acceleration = 0.0
                self.pid_controller.reset()
                
            # Текущее время
            t = self.time_points[i]
            
            # Получаем целевую высоту для текущего времени
            target_height = self.get_target_height(t)
            self.target_heights[i] = target_height
            
            # Определяем команду дросселя
            if self.current_scenario.get("manual_throttle", False):
                # В ручном режиме берем значение из сценария
                throttle = self.get_manual_throttle(t)
            else:
                # В автоматическом режиме используем PID контроллер
                # Вычисляем ошибку (в см для совместимости с PID контроллером)
                error = (target_height - self.height) * 100
                throttle = self.pid_controller.adaptive_height_step(error, self.dt)
                
            self.throttle_commands[i] = throttle
            
            # Получаем внешнюю силу (если есть)
            external_force = self.get_external_force(t)
            
            # Обновляем физическое состояние дрона
            self.update_physics(throttle, external_force)
            
            # Сохраняем данные для визуализации
            self.heights[i] = self.height
            self.velocities[i] = self.velocity
            self.accelerations[i] = self.acceleration
            
            # Обновляем графики
            x_data = self.time_points[:i+1]
            
            self.lines[0].set_data(x_data, self.heights[:i+1])
            self.lines[1].set_data(x_data, self.target_heights[:i+1])
            self.lines[2].set_data(x_data, self.velocities[:i+1])
            self.lines[3].set_data(x_data, self.accelerations[:i+1])
            self.lines[4].set_data(x_data, self.throttle_commands[:i+1])
            
            return self.lines
        
        # Создаем анимацию
        self.animation = animation.FuncAnimation(
            self.fig, update, frames=self.num_points,
            init_func=init, blit=True, interval=20)
        
        plt.tight_layout()
        plt.show()
        
    def run_interactive(self):
        """
        Запускает интерактивный режим для выбора сценария
        """
        print("Доступные сценарии:")
        for i, scenario in enumerate(self.scenarios):
            print(f"{i}: {scenario['name']}")
            
        try:
            choice = int(input("Выберите сценарий (номер): "))
            animate = input("Создать анимацию? (y/n): ").lower() == 'y'
            save_data = input("Сохранить данные в CSV? (y/n): ").lower() == 'y'
            
            if animate:
                self.create_animation(choice)
                if save_data:
                    self.save_throttle_data()
            else:
                self.run_simulation(choice, save_data)
                
        except ValueError:
            print("Неверный ввод. Запуск сценария по умолчанию.")
            self.run_simulation(0)


if __name__ == "__main__":
    simulator = DroneSimulator()
    
    if len(sys.argv) > 1:
        try:
            scenario_index = int(sys.argv[1])
            animate = len(sys.argv) > 2 and sys.argv[2].lower() == 'animate'
            save_data = len(sys.argv) > 3 and sys.argv[3].lower() == 'save'
            
            if animate:
                simulator.create_animation(scenario_index)
                if save_data:
                    simulator.save_throttle_data()
            else:
                simulator.run_simulation(scenario_index, save_data)
        except ValueError:
            print("Неверный индекс сценария. Запуск интерактивного режима.")
            simulator.run_interactive()
    else:
        simulator.run_interactive() 
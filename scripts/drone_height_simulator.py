#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.gridspec import GridSpec
import time
import sys
import os

# Добавляем путь к директории скриптов, чтобы импортировать pid_class
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from pid_class import PIDaxis, TARGET_HEIGHT

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
        self.pid_controller = PIDaxis(90.0, 0.15, 45.0, 
                                     i_range=(-400, 400), 
                                     control_range=(1100, 1900),
                                     midpoint=1500)
        self.pid_controller.is_throttle_controller = True
        
        # Параметры INAV-подобного поведения
        self.throttle_hover = 1500  # приблизительное значение для зависания
        self.throttle_min = 1000    # минимальное значение дросселя
        self.throttle_max = 2000    # максимальное значение дросселя
        
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
            {"name": "Внешние возмущения", "changes": [(0, 0.75)], "disturbances": [(10, -0.5), (25, 0.7), (40, -0.3)]}
        ]
        
        # Текущий сценарий
        self.current_scenario = None
        
    def throttle_to_thrust(self, throttle):
        """
        Преобразует команду дросселя (1000-2000) в тягу в Ньютонах
        Имитирует поведение прошивки INAV:
        - 1500 примерно соответствует зависанию (компенсация гравитации)
        - < 1400 - снижение
        - > 1600 - подъем
        """
        # Нормализуем дроссель от 0 до 1
        normalized_throttle = (throttle - self.throttle_min) / (self.throttle_max - self.throttle_min)
        normalized_throttle = np.clip(normalized_throttle, 0, 1)
        
        # Нелинейное отображение для более реалистичного поведения
        # Экспоненциальная кривая для более точного контроля в середине диапазона
        if normalized_throttle < 0.5:
            thrust_factor = 0.8 * (normalized_throttle / 0.5) ** 1.5
        else:
            thrust_factor = 0.8 + 0.4 * ((normalized_throttle - 0.5) / 0.5) ** 0.8
            
        # Вычисляем тягу (вес дрона компенсируется при throttle ~= 1500)
        hover_thrust = self.mass * self.gravity
        thrust = thrust_factor * self.max_thrust
        
        return thrust
    
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
    
    def run_simulation(self, scenario_index=0):
        """
        Запускает симуляцию с выбранным сценарием
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
        
        # Основной цикл симуляции
        for i, t in enumerate(self.time_points):
            # Получаем целевую высоту для текущего времени
            target_height = self.get_target_height(t)
            self.target_heights[i] = target_height
            
            # Вычисляем ошибку (в см для совместимости с PID контроллером)
            error = (target_height - self.height) * 100
            
            # Получаем команду дросселя от PID контроллера
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
            
        # Визуализируем результаты
        self.visualize_results()
            
    def visualize_results(self):
        """
        Визуализирует результаты симуляции
        """
        plt.style.use('ggplot')
        self.fig = plt.figure(figsize=(12, 10))
        gs = GridSpec(4, 1, height_ratios=[3, 1, 1, 1])
        
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
        ax4.axhline(y=1500, color='r', linestyle='--', alpha=0.5)  # Линия зависания
        ax4.set_ylabel('Дроссель')
        ax4.set_xlabel('Время (с)')
        ax4.set_ylim(self.throttle_min, self.throttle_max)
        ax4.grid(True)
        
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
            
            # Вычисляем ошибку (в см для совместимости с PID контроллером)
            error = (target_height - self.height) * 100
            
            # Получаем команду дросселя от PID контроллера
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
            
            if animate:
                self.create_animation(choice)
            else:
                self.run_simulation(choice)
                
        except ValueError:
            print("Неверный ввод. Запуск сценария по умолчанию.")
            self.run_simulation(0)


if __name__ == "__main__":
    simulator = DroneSimulator()
    
    if len(sys.argv) > 1:
        try:
            scenario_index = int(sys.argv[1])
            animate = len(sys.argv) > 2 and sys.argv[2].lower() == 'animate'
            
            if animate:
                simulator.create_animation(scenario_index)
            else:
                simulator.run_simulation(scenario_index)
        except ValueError:
            print("Неверный индекс сценария. Запуск интерактивного режима.")
            simulator.run_interactive()
    else:
        simulator.run_interactive() 
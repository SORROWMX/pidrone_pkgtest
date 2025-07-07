#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
import matplotlib.gridspec as gridspec
import time
import sys
import os

# Импортируем автономную версию PID контроллера
from pid_standalone import PIDaxis, TARGET_HEIGHT

class PIDTuningSimulator:
    def __init__(self):
        # Параметры симуляции
        self.dt = 0.01  # шаг времени (секунды)
        self.sim_time = 20  # общее время симуляции (секунды)
        self.time_points = np.arange(0, self.sim_time, self.dt)
        self.num_points = len(self.time_points)
        
        # Параметры дрона
        self.mass = 0.5  # масса дрона (кг)
        self.gravity = 9.81  # ускорение свободного падения (м/с²)
        self.max_thrust = 2.0 * self.mass * self.gravity  # максимальная тяга (Н)
        self.drag_coefficient = 0.1  # коэффициент сопротивления воздуха
        
        # Начальные значения PID параметров
        self.kp = 90.0
        self.ki = 0.15
        self.kd = 45.0
        
        # Инициализация PID контроллера
        self.pid_controller = PIDaxis(self.kp, self.ki, self.kd, 
                                     i_range=(-400, 400), 
                                     control_range=(1100, 1900),
                                     midpoint=1500)
        self.pid_controller.is_throttle_controller = True
        
        # Параметры INAV-подобного поведения
        self.throttle_hover = 1500  # приблизительное значение для зависания
        self.throttle_min = 1000    # минимальное значение дросселя
        self.throttle_max = 2000    # максимальное значение дросселя
        
        # Создаем интерфейс
        self.setup_ui()
        
    def setup_ui(self):
        """
        Настраивает интерфейс с графиками и слайдерами
        """
        self.fig = plt.figure(figsize=(14, 10))
        gs = gridspec.GridSpec(3, 2, height_ratios=[3, 1, 1])
        
        # График высоты
        self.ax_height = plt.subplot(gs[0, 0])
        self.ax_height.set_ylabel('Высота (м)')
        self.ax_height.set_title('Высота дрона')
        self.ax_height.grid(True)
        
        # График скорости
        self.ax_velocity = plt.subplot(gs[1, 0], sharex=self.ax_height)
        self.ax_velocity.set_ylabel('Скорость (м/с)')
        self.ax_velocity.grid(True)
        
        # График команд дросселя
        self.ax_throttle = plt.subplot(gs[2, 0], sharex=self.ax_height)
        self.ax_throttle.set_ylabel('Дроссель')
        self.ax_throttle.set_xlabel('Время (с)')
        self.ax_throttle.grid(True)
        
        # График ошибки
        self.ax_error = plt.subplot(gs[0, 1])
        self.ax_error.set_ylabel('Ошибка (м)')
        self.ax_error.set_title('Ошибка высоты')
        self.ax_error.grid(True)
        
        # График PID компонентов
        self.ax_pid = plt.subplot(gs[1, 1], sharex=self.ax_error)
        self.ax_pid.set_ylabel('PID компоненты')
        self.ax_pid.grid(True)
        
        # Область для слайдеров
        slider_ax = plt.subplot(gs[2, 1])
        slider_ax.set_axis_off()
        
        # Создаем слайдеры
        ax_kp = plt.axes([0.55, 0.15, 0.35, 0.03])
        ax_ki = plt.axes([0.55, 0.1, 0.35, 0.03])
        ax_kd = plt.axes([0.55, 0.05, 0.35, 0.03])
        
        self.slider_kp = Slider(ax_kp, 'Kp', 0.0, 200.0, valinit=self.kp)
        self.slider_ki = Slider(ax_ki, 'Ki', 0.0, 1.0, valinit=self.ki)
        self.slider_kd = Slider(ax_kd, 'Kd', 0.0, 100.0, valinit=self.kd)
        
        # Добавляем кнопку симуляции
        ax_button = plt.axes([0.7, 0.25, 0.15, 0.04])
        self.button_simulate = Button(ax_button, 'Симулировать')
        self.button_simulate.on_clicked(self.on_simulate_clicked)
        
        # Добавляем кнопки для предустановленных сценариев
        ax_scenario1 = plt.axes([0.55, 0.25, 0.15, 0.04])
        self.button_scenario1 = Button(ax_scenario1, 'Ступеньки')
        self.button_scenario1.on_clicked(self.on_scenario1_clicked)
        
        ax_scenario2 = plt.axes([0.55, 0.3, 0.15, 0.04])
        self.button_scenario2 = Button(ax_scenario2, 'Синусоида')
        self.button_scenario2.on_clicked(self.on_scenario2_clicked)
        
        ax_scenario3 = plt.axes([0.7, 0.3, 0.15, 0.04])
        self.button_scenario3 = Button(ax_scenario3, 'Возмущения')
        self.button_scenario3.on_clicked(self.on_scenario3_clicked)
        
        plt.tight_layout()
        
        # Инициализация переменных для хранения данных симуляции
        self.heights = np.zeros(self.num_points)
        self.velocities = np.zeros(self.num_points)
        self.accelerations = np.zeros(self.num_points)
        self.throttle_commands = np.zeros(self.num_points)
        self.target_heights = np.zeros(self.num_points)
        self.errors = np.zeros(self.num_points)
        self.p_terms = np.zeros(self.num_points)
        self.i_terms = np.zeros(self.num_points)
        self.d_terms = np.zeros(self.num_points)
        
        # Текущий сценарий
        self.current_scenario = "step"
        
    def on_simulate_clicked(self, event):
        """
        Обработчик нажатия кнопки симуляции
        """
        # Получаем значения с слайдеров
        self.kp = self.slider_kp.val
        self.ki = self.slider_ki.val
        self.kd = self.slider_kd.val
        
        # Обновляем PID контроллер
        self.pid_controller = PIDaxis(self.kp, self.ki, self.kd, 
                                     i_range=(-400, 400), 
                                     control_range=(1100, 1900),
                                     midpoint=1500)
        self.pid_controller.is_throttle_controller = True
        
        # Запускаем симуляцию
        self.run_simulation()
        
    def on_scenario1_clicked(self, event):
        """
        Обработчик нажатия кнопки сценария 1 (ступеньки)
        """
        self.current_scenario = "step"
        self.on_simulate_clicked(event)
        
    def on_scenario2_clicked(self, event):
        """
        Обработчик нажатия кнопки сценария 2 (синусоида)
        """
        self.current_scenario = "sine"
        self.on_simulate_clicked(event)
        
    def on_scenario3_clicked(self, event):
        """
        Обработчик нажатия кнопки сценария 3 (возмущения)
        """
        self.current_scenario = "disturbance"
        self.on_simulate_clicked(event)
        
    def get_target_height(self, time):
        """
        Возвращает целевую высоту в зависимости от текущего времени и сценария
        """
        if self.current_scenario == "step":
            # Ступенчатое изменение высоты
            if time < 2:
                return 0.5
            elif time < 6:
                return 1.0
            elif time < 10:
                return 0.7
            elif time < 14:
                return 1.2
            else:
                return 0.5
        elif self.current_scenario == "sine":
            # Синусоидальное изменение
            return 0.8 + 0.3 * np.sin(2 * np.pi * time / 10)
        elif self.current_scenario == "disturbance":
            # Постоянная высота с возмущениями
            return 0.75
        else:
            return TARGET_HEIGHT
            
    def get_external_force(self, time):
        """
        Возвращает внешнюю силу (возмущение) в зависимости от текущего времени и сценария
        """
        if self.current_scenario == "disturbance":
            # Импульсные возмущения
            if 5 <= time < 6:
                return -0.5 * self.mass  # Отрицательное возмущение (вниз)
            elif 10 <= time < 11:
                return 0.7 * self.mass  # Положительное возмущение (вверх)
            elif 15 <= time < 16:
                return -0.3 * self.mass  # Отрицательное возмущение (вниз)
        
        return 0
        
    def throttle_to_thrust(self, throttle):
        """
        Преобразует команду дросселя (1000-2000) в тягу в Ньютонах
        Имитирует поведение прошивки INAV
        """
        # Нормализуем дроссель от 0 до 1
        normalized_throttle = (throttle - self.throttle_min) / (self.throttle_max - self.throttle_min)
        normalized_throttle = np.clip(normalized_throttle, 0, 1)
        
        # Нелинейное отображение для более реалистичного поведения
        if normalized_throttle < 0.5:
            thrust_factor = 0.8 * (normalized_throttle / 0.5) ** 1.5
        else:
            thrust_factor = 0.8 + 0.4 * ((normalized_throttle - 0.5) / 0.5) ** 0.8
            
        # Вычисляем тягу (вес дрона компенсируется при throttle ~= 1500)
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
            
    def run_simulation(self):
        """
        Запускает симуляцию с текущими параметрами
        """
        # Сбрасываем начальные условия
        self.height = 0.0
        self.velocity = 0.0
        self.acceleration = 0.0
        self.pid_controller.reset()
        
        # Очищаем графики
        self.ax_height.clear()
        self.ax_velocity.clear()
        self.ax_throttle.clear()
        self.ax_error.clear()
        self.ax_pid.clear()
        
        # Настраиваем графики
        self.ax_height.set_ylabel('Высота (м)')
        self.ax_height.set_title('Высота дрона')
        self.ax_height.grid(True)
        
        self.ax_velocity.set_ylabel('Скорость (м/с)')
        self.ax_velocity.grid(True)
        
        self.ax_throttle.set_ylabel('Дроссель')
        self.ax_throttle.set_xlabel('Время (с)')
        self.ax_throttle.grid(True)
        
        self.ax_error.set_ylabel('Ошибка (м)')
        self.ax_error.set_title('Ошибка высоты')
        self.ax_error.grid(True)
        
        self.ax_pid.set_ylabel('PID компоненты')
        self.ax_pid.grid(True)
        
        # Основной цикл симуляции
        for i, t in enumerate(self.time_points):
            # Получаем целевую высоту для текущего времени
            target_height = self.get_target_height(t)
            self.target_heights[i] = target_height
            
            # Вычисляем ошибку (в см для совместимости с PID контроллером)
            error_cm = (target_height - self.height) * 100
            self.errors[i] = target_height - self.height  # в метрах для графика
            
            # Получаем команду дросселя от PID контроллера
            throttle = self.pid_controller.adaptive_height_step(error_cm, self.dt)
            self.throttle_commands[i] = throttle
            
            # Сохраняем PID компоненты
            self.p_terms[i] = self.pid_controller._p
            self.i_terms[i] = self.pid_controller._i
            self.d_terms[i] = self.pid_controller._d
            
            # Получаем внешнюю силу (если есть)
            external_force = self.get_external_force(t)
            
            # Обновляем физическое состояние дрона
            self.update_physics(throttle, external_force)
            
            # Сохраняем данные для визуализации
            self.heights[i] = self.height
            self.velocities[i] = self.velocity
            self.accelerations[i] = self.acceleration
            
        # Отображаем результаты
        self.ax_height.plot(self.time_points, self.heights, 'b-', linewidth=2, label='Фактическая высота')
        self.ax_height.plot(self.time_points, self.target_heights, 'r--', linewidth=1.5, label='Целевая высота')
        self.ax_height.legend(loc='upper right')
        self.ax_height.set_ylim(0, 1.5)
        
        self.ax_velocity.plot(self.time_points, self.velocities, 'g-', linewidth=1.5)
        self.ax_velocity.set_ylim(-1, 1)
        
        self.ax_throttle.plot(self.time_points, self.throttle_commands, 'k-', linewidth=1.5)
        self.ax_throttle.axhline(y=1500, color='r', linestyle='--', alpha=0.5)  # Линия зависания
        self.ax_throttle.set_ylim(1300, 1700)
        
        self.ax_error.plot(self.time_points, self.errors, 'm-', linewidth=1.5)
        self.ax_error.set_ylim(-0.5, 0.5)
        
        self.ax_pid.plot(self.time_points, self.p_terms, 'r-', linewidth=1.5, label='P')
        self.ax_pid.plot(self.time_points, self.i_terms, 'g-', linewidth=1.5, label='I')
        self.ax_pid.plot(self.time_points, self.d_terms, 'b-', linewidth=1.5, label='D')
        self.ax_pid.legend(loc='upper right')
        
        # Обновляем графики
        self.fig.canvas.draw_idle()
        
    def show(self):
        """
        Показывает интерфейс и запускает первую симуляцию
        """
        self.run_simulation()
        plt.show()


if __name__ == "__main__":
    simulator = PIDTuningSimulator()
    simulator.show() 
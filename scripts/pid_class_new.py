#!/usr/bin/env python

import rospy
import math

# Target height to maintain in meters
TARGET_HEIGHT = 0.75

# PID parameters for height control - tuned for new throttle values
KP = 60.0   # Оптимизировано для более быстрой реакции
KI = 0.12   # Оптимизировано для лучшего удержания высоты
KD = 40.0   # Оптимизировано для лучшего демпфирования

# New parameters for improved height control
MAX_VERTICAL_SPEED = 0.25  # м/с - максимальная скорость по вертикали
HOVER_THROTTLE = 1450     # Значение дросселя для зависания (из реальных данных)
THROTTLE_MIN = 1400       # Минимальное значение, при котором дрон начинает отрываться от земли
THROTTLE_MAX = 1500       # Максимальное безопасное значение

# Takeoff parameters
TAKEOFF_ACCELERATION_LIMIT = 0.15  # м/с² - ограничение ускорения при взлете
TAKEOFF_VELOCITY_LIMIT = 0.3       # м/с - максимальная скорость при взлете
TAKEOFF_DECELERATION_START = 0.65  # % от целевой высоты - начало торможения

#####################################################
#						PID							#
#####################################################
class PIDaxis():
    def __init__(self, kp, ki, kd, i_range=None, d_range=None, control_range=(1000, 2000), midpoint=1500, smoothing=True):
        # Tuning
        self.kp = kp
        self.ki = ki
        self.kd = kd
        # Config
        self.i_range = i_range
        self.d_range = d_range
        self.control_range = control_range
        self.midpoint = midpoint
        self.smoothing = smoothing
        # Internal
        self.reset()
        # Additional parameters for advanced height control
        self.previous_height = 0
        self.height_integral = 0
        self.previous_height_error = 0
        # Initialize last_time to None, will be set on first step call
        self.last_time = None
        self.is_throttle_controller = False
        # New parameters for improved height control
        self.height_history = []  # Store recent height measurements
        self.time_history = []    # Store corresponding timestamps
        self.vertical_velocity = 0.0  # Estimated vertical velocity
        self.height_setpoint = TARGET_HEIGHT  # Current target height
        self.debug = True  # Enable/disable debug printing
        # Takeoff phase detection
        self.takeoff_phase = True  # Start in takeoff mode
        self.takeoff_start_time = None
        self.max_height_reached = 0.0
        self.prev_throttle = HOVER_THROTTLE  # Для плавного изменения дросселя
        self.takeoff_stage = 0  # 0: начальный разгон, 1: подъем, 2: торможение
        # Флаг для определения, находится ли дрон на земле
        self.on_ground = True
        
    def reset(self):
        self._old_err = None
        self._p = 0
        self._i = 0
        self._d = 0
        self._dd = 0
        self._ddd = 0
        # Reset height control variables too
        self.previous_height = 0
        self.height_integral = 0
        self.previous_height_error = 0
        # Reset last_time to None
        self.last_time = None
        # Reset new parameters
        self.height_history = []
        self.time_history = []
        self.vertical_velocity = 0.0
        # Reset takeoff variables
        self.takeoff_phase = True
        self.takeoff_start_time = None
        self.max_height_reached = 0.0
        self.prev_throttle = HOVER_THROTTLE
        self.takeoff_stage = 0
        self.on_ground = True
        
    def step(self, err, time_elapsed):
        # Special handling for throttle controller with improved height control logic
        if self.is_throttle_controller:
            return self.adaptive_height_step(err, time_elapsed)
            
        # Standard PID logic for other axes
        if self._old_err is None:
            # First time around prevent d term spike
            self._old_err = err

        # Find the p component
        self._p = err * self.kp

        # Find the i component
        self._i += err * self.ki * time_elapsed
        if self.i_range is not None:
            self._i = max(self.i_range[0], min(self._i, self.i_range[1]))

        # Find the d component
        self._d = (err - self._old_err) * self.kd / time_elapsed
        if self.d_range is not None:
            self._d = max(self.d_range[0], min(self._d, self.d_range[1]))
        self._old_err = err

        # Smooth over the last three d terms
        if self.smoothing:
            self._d = (self._d * 8.0 + self._dd * 5.0 + self._ddd * 2.0)/15.0
            self._ddd = self._dd
            self._dd = self._d

        # Calculate control output
        raw_output = self._p + self._i + self._d
        output = min(max(raw_output + self.midpoint, self.control_range[0]), self.control_range[1])

        return output
        
    def adaptive_height_step(self, err, time_elapsed):
        """
        Enhanced adaptive throttle control with improved behavior for height maintenance
        Features:
        - Vertical velocity estimation for better D-term calculation
        - Anti-windup mechanism for integral term
        - Dynamic gain scheduling based on height error and vertical speed
        - Feedforward term for more responsive control
        - Improved emergency handling
        - Special takeoff handling to prevent overshoot
        - Calibrated for real drone behavior
        """
        # Get current height from error (err = TARGET_HEIGHT - current_height)
        current_height = TARGET_HEIGHT - err/100.0  # Convert from cm to m
        
        # Calculate time elapsed since last call
        current_time = rospy.Time.now()
        if self.last_time is None:
            self.last_time = current_time
            dt = 0.01  # Default small value for first iteration
            if self.takeoff_phase:
                self.takeoff_start_time = current_time
        else:
            dt = (current_time - self.last_time).to_sec()
            
        self.last_time = current_time
        
        # Ensure dt is reasonable to prevent division by zero or extreme values
        if dt <= 0.001 or dt > 0.5:
            dt = 0.01  # Safeguard against unreasonable time deltas
            
        # Store height and time history (keep last 5 points)
        self.height_history.append(current_height)
        self.time_history.append(current_time.to_sec())
        if len(self.height_history) > 5:
            self.height_history.pop(0)
            self.time_history.pop(0)
        
        # Track maximum height reached (for takeoff phase detection)
        self.max_height_reached = max(self.max_height_reached, current_height)
            
        # Calculate vertical velocity using linear regression if we have enough points
        if len(self.height_history) >= 3:
            # Simple velocity calculation using last two points for responsiveness
            instant_velocity = (self.height_history[-1] - self.height_history[-2]) / dt
            
            # Apply low-pass filter to smooth velocity estimate
            self.vertical_velocity = 0.7 * instant_velocity + 0.3 * self.vertical_velocity
        
        # Определение, находится ли дрон на земле
        if current_height < 0.05 and self.vertical_velocity <= 0:
            self.on_ground = True
        else:
            self.on_ground = False
            
        # Определение стадии взлета
        if self.takeoff_phase:
            height_ratio = current_height / TARGET_HEIGHT
            if height_ratio < 0.3:
                # Начальный разгон - плавное увеличение тяги
                self.takeoff_stage = 0
            elif height_ratio < TAKEOFF_DECELERATION_START:
                # Основной подъем - контролируемая скорость
                self.takeoff_stage = 1
            else:
                # Торможение - подготовка к стабилизации
                self.takeoff_stage = 2
        
        # Detect takeoff phase completion
        # Exit takeoff phase if:
        # 1. We've reached at least 90% of target height
        # 2. We've been in takeoff phase for at least 2 seconds
        # 3. Our vertical velocity is low (approaching stabilization)
        if self.takeoff_phase and current_height >= TARGET_HEIGHT * 0.9 and \
           (current_time - self.takeoff_start_time).to_sec() > 2.0:
            # Only exit takeoff if we're moving slowly
            if abs(self.vertical_velocity) < 0.1:
                self.takeoff_phase = False
                if self.debug:
                    rospy.loginfo("Exiting takeoff phase at height %.2fm with velocity %.2fm/s", 
                                 current_height, self.vertical_velocity)
        
        # Special handling for early takeoff phase - prevent integral windup
        if self.takeoff_phase and current_height < TARGET_HEIGHT * 0.6:
            # Reset integral term during early takeoff to prevent windup
            self.height_integral = 0.0
        
        if self.debug:
            rospy.loginfo("Height: %.2fm, Target: %.2fm, V_speed: %.2fm/s, Takeoff: %s, Stage: %d, OnGround: %s", 
                         current_height, TARGET_HEIGHT, self.vertical_velocity, 
                         "Yes" if self.takeoff_phase else "No", 
                         self.takeoff_stage if self.takeoff_phase else -1,
                         "Yes" if self.on_ground else "No")
        
        # Emergency handling with improved logic
        if current_height > TARGET_HEIGHT * 1.4:  # Снижено с 1.75 до 1.4
            if self.debug:
                rospy.logwarn("EMERGENCY BRAKING: too high!")
            return THROTTLE_MIN  # Minimum throttle to quickly descend
        
        # Prevent rapid descent if falling too fast
        if self.vertical_velocity < -MAX_VERTICAL_SPEED * 1.1:  # Снижено с 1.5 до 1.1
            if self.debug:
                rospy.logwarn("EMERGENCY THROTTLE: falling too fast! %.2f m/s", self.vertical_velocity)
            return HOVER_THROTTLE + 10  # Снижено с +30 до +10
            
        # Calculate error
        error = TARGET_HEIGHT - current_height
        
        # Smooth error to reduce sudden changes (exponential smoothing)
        smoothed_error = error * 0.6 + self.previous_height_error * 0.4  # Увеличиваем сглаживание
        
        # Update integral term with anti-windup mechanism
        # Only integrate when within reasonable range of target and not moving too fast
        if abs(error) < 0.15 and abs(self.vertical_velocity) < MAX_VERTICAL_SPEED * 0.7:  # Сужен диапазон интегрирования
            # Reduce integral gain during takeoff
            integral_gain = 0.15 if self.takeoff_phase else 0.3
            self.height_integral = self.height_integral + error * dt * integral_gain
            
            # Dynamic integral limits based on error magnitude
            i_limit = 0.06 * (1.0 - min(1.0, abs(error) / 0.15))  # Уменьшен предел интегрирования
            self.height_integral = max(-i_limit, min(self.height_integral, i_limit))
        else:
            # Outside normal operating range - quickly decay integral term
            self.height_integral *= 0.85  # Быстрее сбрасываем интеграл
        
        # Calculate derivative with vertical velocity for better performance
        derivative = -self.vertical_velocity  # Negative because positive velocity means decreasing error
        
        # Calculate PID terms - adjust gains during takeoff
        if self.takeoff_phase:
            # Use different gains based on takeoff stage
            if self.takeoff_stage == 0:
                # Начальный разгон - мягкий P, почти нет I, высокий D для контроля ускорения
                takeoff_kp = KP * 0.5
                takeoff_ki = KI * 0.1
                takeoff_kd = KD * 1.5
            elif self.takeoff_stage == 1:
                # Основной подъем - средний P, низкий I, высокий D
                takeoff_kp = KP * 0.6
                takeoff_ki = KI * 0.3
                takeoff_kd = KD * 1.3
            else:  # stage 2
                # Торможение - низкий P, средний I, очень высокий D
                takeoff_kp = KP * 0.5
                takeoff_ki = KI * 0.4
                takeoff_kd = KD * 1.6
            
            p_term = takeoff_kp * smoothed_error
            i_term = takeoff_ki * self.height_integral
            d_term = takeoff_kd * derivative
        else:
            # Normal operation
            p_term = KP * smoothed_error
            i_term = KI * self.height_integral
            d_term = KD * derivative
        
        # Add feedforward term based on target height
        # This helps maintain hover without relying solely on integral term
        ff_term = HOVER_THROTTLE - self.midpoint
        
        # Dynamic gain scheduling based on error and vertical speed
        if current_height > TARGET_HEIGHT:
            # Above target height - need to descend
            height_diff = current_height - TARGET_HEIGHT
            
            # Adjust descent rate based on how high we are
            if height_diff > 0.2:  # Снижено с 0.3 до 0.2
                # Significantly above target - stronger descent
                base_throttle = HOVER_THROTTLE - 25  # Уменьшено с -40 до -25
                gain = 0.5   # Уменьшено с 0.6 до 0.5
            elif height_diff > 0.08:  # Снижено с 0.1 до 0.08
                base_throttle = HOVER_THROTTLE - 15  # Уменьшено с -20 до -15
                gain = 0.55  # Уменьшено с 0.67 до 0.55
            else: 
                # Close to target - gentle descent
                base_throttle = HOVER_THROTTLE - 7   # Уменьшено с -10 до -7
                gain = 0.65  # Уменьшено с 0.76 до 0.65
                
            # Reduce descent rate if already descending quickly
            if self.vertical_velocity < -0.12:  # Снижено с -0.2 до -0.12
                gain *= (1.0 + 0.35 * min(1.0, -self.vertical_velocity / MAX_VERTICAL_SPEED))
                
            throttle_adjustment = p_term + i_term + d_term
            throttle = base_throttle + int(throttle_adjustment * gain)
        else:
            # Below target - need to ascend
            height_diff = TARGET_HEIGHT - current_height
            
            # Special handling for takeoff phase
            if self.takeoff_phase:
                # During takeoff, use more conservative approach
                if self.takeoff_stage == 0:
                    # Начальный разгон - очень плавное увеличение тяги
                    if self.on_ground:
                        # Если дрон на земле, нужно дать больше тяги для отрыва
                        base_throttle = HOVER_THROTTLE + 15
                        gain = 0.6
                    else:
                        base_throttle = HOVER_THROTTLE + 10
                        gain = 0.5
                    
                    # Ограничение ускорения при взлете
                    if self.vertical_velocity > TAKEOFF_VELOCITY_LIMIT * 0.5:
                        # Если уже набрали половину целевой скорости, начинаем ограничивать ускорение
                        base_throttle = HOVER_THROTTLE + 5
                        gain = 0.4
                        
                elif self.takeoff_stage == 1:
                    # Основной подъем - контролируемая скорость
                    if height_diff > 0.25:
                        base_throttle = HOVER_THROTTLE + 12
                        gain = 0.6
                    else:
                        base_throttle = HOVER_THROTTLE + 8
                        gain = 0.55
                        
                    # Контроль скорости подъема
                    if self.vertical_velocity > TAKEOFF_VELOCITY_LIMIT:
                        # Если превысили целевую скорость, снижаем тягу
                        velocity_excess = (self.vertical_velocity - TAKEOFF_VELOCITY_LIMIT) / TAKEOFF_VELOCITY_LIMIT
                        base_throttle -= int(velocity_excess * 12)
                        
                else:  # stage 2 - торможение
                    # Торможение перед достижением цели
                    # Чем ближе к цели и выше скорость, тем сильнее тормозим
                    height_ratio = current_height / TARGET_HEIGHT
                    
                    if height_ratio > 0.9:
                        # Очень близко к цели - почти зависание
                        base_throttle = HOVER_THROTTLE - 2
                        gain = 0.5
                    elif height_ratio > 0.8:
                        # Близко к цели - начинаем тормозить
                        base_throttle = HOVER_THROTTLE + 2
                        gain = 0.55
                    else:
                        # Приближаемся к цели - готовимся к торможению
                        base_throttle = HOVER_THROTTLE + 5
                        gain = 0.6
                    
                    # Прогрессивное торможение на основе скорости
                    if self.vertical_velocity > 0.1:
                        # Тормозим тем сильнее, чем выше скорость
                        braking = min(1.0, self.vertical_velocity / TAKEOFF_VELOCITY_LIMIT) * min(1.0, height_ratio / 0.9)
                        base_throttle -= int(braking * 15)  # До 15 единиц снижения тяги
                
                # Общее ограничение скорости для всех стадий взлета
                if self.vertical_velocity > TAKEOFF_VELOCITY_LIMIT * 1.2:
                    # Экстренное торможение при превышении скорости
                    base_throttle = HOVER_THROTTLE - 8
                    gain = 0.4
            else:
                # Normal operation (not takeoff)
                # Adjust ascent rate based on how low we are
                if height_diff > 0.2:  # Снижено с 0.3 до 0.2
                    # Significantly below target - stronger response
                    base_throttle = HOVER_THROTTLE + 15  # Снижено с +30 до +15
                    gain = 0.8  # Снижено с 0.9 до 0.8
                elif height_diff > 0.08:  # Снижено с 0.1 до 0.08
                    # Moderately below target
                    base_throttle = HOVER_THROTTLE + 10  # Снижено с +20 до +10
                    gain = 0.7  # Снижено с 0.8 до 0.7
                elif height_diff > 0.04:  # Снижено с 0.05 до 0.04
                    # Slightly below target
                    base_throttle = HOVER_THROTTLE + 5   # Снижено с +10 до +5
                    gain = 0.6  # Снижено с 0.7 до 0.6
                else:
                    # Very close to target
                    base_throttle = HOVER_THROTTLE
                    gain = 0.55  # Снижено с 0.65 до 0.55
                
            # Reduce ascent rate if already ascending quickly
            if self.vertical_velocity > 0.12:  # Снижено с 0.2 до 0.12
                gain *= (1.0 - 0.3 * min(1.0, self.vertical_velocity / MAX_VERTICAL_SPEED))
                
            throttle_adjustment = p_term + i_term + d_term
            throttle = base_throttle + int(throttle_adjustment * gain)
        
        # Add feedforward term
        throttle += ff_term
        
        # Ограничение скорости изменения дросселя для более плавного управления
        max_throttle_change = 8  # Максимальное изменение за один шаг
        if self.takeoff_phase:
            # Во время взлета еще более плавное изменение
            max_throttle_change = 4
            
        throttle_change = throttle - self.prev_throttle
        if abs(throttle_change) > max_throttle_change:
            # Ограничиваем изменение
            throttle = self.prev_throttle + max_throttle_change * (1 if throttle_change > 0 else -1)
        
        # Apply nonlinear mapping to throttle for better control at extremes
        # This makes control more sensitive near hover point and less sensitive at extremes
        throttle_range = THROTTLE_MAX - THROTTLE_MIN
        normalized_throttle = (throttle - THROTTLE_MIN) / float(throttle_range)
        normalized_throttle = max(0.0, min(normalized_throttle, 1.0))  # Ensure in [0,1]
        
        # Apply smooth nonlinear curve (sigmoid-like)
        if normalized_throttle < 0.5:
            normalized_throttle = 0.5 * math.pow(normalized_throttle * 2, 1.05)  # Более плавная кривая
        else:
            normalized_throttle = 0.5 + 0.5 * (1.0 - math.pow(2.0 - normalized_throttle * 2, 1.05))  # Более плавная кривая
            
        # Convert back to throttle value
        throttle = int(THROTTLE_MIN + normalized_throttle * throttle_range)
        
        # Ensure throttle is within safe limits
        throttle = max(THROTTLE_MIN, min(throttle, THROTTLE_MAX))
        
        # Store values for next iteration
        self.previous_height = current_height
        self.previous_height_error = smoothed_error
        self.prev_throttle = throttle
        
        if self.debug:
            rospy.loginfo("Throttle: %d, Error: %.2f, V: %.2f m/s, P: %.2f, I: %.2f, D: %.2f", 
                         throttle, error, self.vertical_velocity, p_term, i_term, d_term)
        
        return throttle


class PID:

    def __init__(self,

                 roll=PIDaxis(2.0, 1.0, 0.0, control_range=(1400, 1600), midpoint=1500, i_range=(-100, 100)),
                 roll_low=PIDaxis(0.0, 0.5, 0.0, control_range=(1400, 1600), midpoint=1500, i_range=(-150, 150)),

                 pitch=PIDaxis(2.0, 1.0, 0.0, control_range=(1400, 1600), midpoint=1500, i_range=(-100, 100)),
                 pitch_low=PIDaxis(0.0, 0.5, 0.0, control_range=(1400, 1600), midpoint=1500, i_range=(-150, 150)),

                 yaw=PIDaxis(0.0, 0.0, 0.0),

                 # Kv 2300 motors have midpoint 1300, Kv 2550 motors have midpoint 1250
                 throttle=PIDaxis(1,
                                  0, #0.5/height_factor * battery_factor,
                                  1,
                                  i_range=(-400, 400), control_range=(1100, 1600),
                                  d_range=(-40, 40), midpoint=1450)  # Изменено с 1300 на 1450
                 ):

        self.trim_controller_cap_plane = 0.05
        self.trim_controller_thresh_plane = 0.0001

        self.roll = roll
        self.roll_low = roll_low

        self.pitch = pitch
        self.pitch_low = pitch_low

        self.yaw = yaw

        self.trim_controller_cap_throttle = 5.0
        self.trim_controller_thresh_throttle = 5.0

        self.throttle = throttle
        # Mark throttle controller for special handling
        self.throttle.is_throttle_controller = True

        self._t = None

        # Tuning values specific to each drone
        self.roll_low.init_i = 0.0
        self.pitch_low.init_i = 0.0
        self.reset()

    def reset(self):
        """ Reset each pid and restore the initial i terms """
        # reset time variable
        self._t = None

        # reset individual PIDs
        self.roll.reset()
        self.roll_low.reset()
        self.pitch.reset()
        self.pitch_low.reset()
        self.throttle.reset()

        # restore tuning values
        self.roll_low._i = self.roll_low.init_i
        self.pitch_low._i = self.pitch_low.init_i

    def step(self, error, cmd_yaw_velocity=0):
        """ Compute the control variables from the error using the step methods
        of each axis pid.
        """
        # First time around prevent time spike
        if self._t is None:
            time_elapsed = 1
        else:
            time_elapsed = rospy.get_time() - self._t

        self._t = rospy.get_time()

        # Compute roll command
        ######################
        # if the x velocity error is within the threshold
        if abs(error.x) < self.trim_controller_thresh_plane:
            # pass the high rate i term off to the low rate pid
            self.roll_low._i += self.roll._i
            self.roll._i = 0
            # set the roll value to just the output of the low rate pid
            cmd_r = self.roll_low.step(error.x, time_elapsed)
        else:
            if error.x > self.trim_controller_cap_plane:
                self.roll_low.step(self.trim_controller_cap_plane, time_elapsed)
            elif error.x < -self.trim_controller_cap_plane:
                self.roll_low.step(-self.trim_controller_cap_plane, time_elapsed)
            else:
                self.roll_low.step(error.x, time_elapsed)

            cmd_r = self.roll_low._i + self.roll.step(error.x, time_elapsed)

        # Compute pitch command
        #######################
        if abs(error.y) < self.trim_controller_thresh_plane:
            self.pitch_low._i += self.pitch._i
            self.pitch._i = 0
            cmd_p = self.pitch_low.step(error.y, time_elapsed)
        else:
            if error.y > self.trim_controller_cap_plane:
                self.pitch_low.step(self.trim_controller_cap_plane, time_elapsed)
            elif error.y < -self.trim_controller_cap_plane:
                self.pitch_low.step(-self.trim_controller_cap_plane, time_elapsed)
            else:
                self.pitch_low.step(error.y, time_elapsed)

            cmd_p = self.pitch_low._i + self.pitch.step(error.y, time_elapsed)

        # Compute yaw command
        cmd_y = 1500 + cmd_yaw_velocity

        # Use adaptive height control for throttle
        cmd_t = self.throttle.adaptive_height_step(error.z, time_elapsed)
        
        if self.throttle.debug:
            rospy.loginfo("%d, %.3f, %.3f, %.3f, %.3f", 
                         cmd_t, error.z, self.throttle._p, self.throttle._i, self.throttle._d)
        
        # Return commands in order [ROLL, PITCH, THROTTLE, YAW] (changed from ROLL, PITCH, YAW, THROTTLE)
        return [cmd_r, cmd_p, cmd_t, cmd_y]

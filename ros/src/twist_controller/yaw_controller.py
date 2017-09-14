import math
from pid import PID
from lowpass import LowPassFilter

class YawController(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.min_speed = min_speed
        self.max_lat_accel = max_lat_accel
        self.previous_dbw_enabled = False
        self.min_angle = -max_steer_angle
        self.max_angle = max_steer_angle
        self.linear_pid = PID(0.9, 0.001, 0.0004, self.min_angle, self.max_angle)
        self.tau = 0.2
        self.ts = 0.1
        self.low_pass_filter = LowPassFilter(self.tau, self.ts)

    def get_angle(self, radius, current_velocity):
        angle = current_velocity * math.atan(self.wheel_base / radius) * self.steer_ratio
        return max(self.min_angle, min(self.max_angle, angle))

    def get_steering_calculated(self, linear_velocity, angular_velocity, current_velocity):
        """
        Formulas:
        angular_velocity_new / current_velocity = angular_velocity_old / linear_velocity
        radius = current_velocity / angular_velocity_new
        angle = atan(wheel_base / radius) * self.steer_ratio
        """
        angular_velocity = current_velocity * angular_velocity / linear_velocity if abs(linear_velocity) > 0. else 0.

        if abs(current_velocity) > 0.1:
            max_yaw_rate = abs(self.max_lat_accel / current_velocity)
            angular_velocity = max(-max_yaw_rate, min(max_yaw_rate, angular_velocity))

        return self.get_angle(max(current_velocity, self.min_speed) / angular_velocity, current_velocity) if abs(angular_velocity) > 0. else 0.0;

    def get_steering_pid(self, angular_velocity, angular_current, dbw_enabled):
        angular_error = angular_velocity - angular_current
        #sample_step = 0.05  # ???
        sample_step = 0.02
        if not(self.previous_dbw_enabled) and dbw_enabled:
            self.previous_dbw_enabled = True
            self.linear_pid.reset()
            self.low_pass_filter = LowPassFilter(self.tau, self.ts)
        else:
            self.previous_dbw_enabled = False
        steering = self.linear_pid.step(angular_error, sample_step)
        #steering = self.low_pass_filter.filt(steering)
        return steering


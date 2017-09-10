import math
from pid import PID
from lowpass import LowPassFilter

def deg2rad(x):
    return x * math.pi / 180


def rad2deg(x):
    return x * 180 / math.pi

class YawController(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.min_speed = min_speed
        self.max_lat_accel = max_lat_accel
        self.previous_dbw_enabled = False
        self.min_angle = -max_steer_angle
        self.max_angle = max_steer_angle
        self.linear_pid = PID(2.0, 0.0024, 0.0001, self.min_angle, self.max_angle)
        self.tau = 0.2
        self.ts = 0.1
        self.low_pass_filter = LowPassFilter(self.tau, self.ts)

    def get_angle(self, radius):
        angle = math.atan(self.wheel_base / radius) * self.steer_ratio
        return max(self.min_angle, min(self.max_angle, angle))

    def get_steering(self, angular_velocity, angular_current, dbw_enabled):
        angular_error = angular_velocity - angular_current
        sample_step = 0.05  # ???
        if not(self.previous_dbw_enabled) and dbw_enabled:
            self.previous_dbw_enabled = True
            self.linear_pid.reset()
            self.low_pass_filter = LowPassFilter(self.tau, self.ts)
        else:
            self.previous_dbw_enabled = False
        steering = self.linear_pid.step(angular_error, sample_step)
        steering = self.low_pass_filter.filt(steering)
        return steering


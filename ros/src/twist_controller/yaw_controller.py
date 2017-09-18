import math
import numpy as np
from pid import PID
from lowpass import LowPassFilter

class YawController(object):
    def __init__(self, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.min_speed = 5
        self.max_lat_accel = max_lat_accel
        self.previous_dbw_enabled = False
        self.min_angle = -max_steer_angle
        self.max_angle = max_steer_angle
        self.linear_pid = PID(0.9, 0.001, 0.0004, self.min_angle, self.max_angle)
        #self.cte_pid = PID(0.4, 0.1, 0.1, self.min_angle, self.max_angle)
        self.cte_pid = PID(0.4, 0.1, 0.2, self.min_angle, self.max_angle)
        self.tau = 0.2
        self.ts = 0.1
        self.low_pass_filter = LowPassFilter(self.tau, self.ts)

    def get_angle(self, radius, current_velocity):
         angle = math.atan(self.wheel_base / radius) * self.steer_ratio
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
        sample_step = 0.02
        if not(self.previous_dbw_enabled) and dbw_enabled:
            self.previous_dbw_enabled = True
            self.linear_pid.reset()
            self.low_pass_filter = LowPassFilter(self.tau, self.ts)
        else:
            self.previous_dbw_enabled = False
        steering = self.linear_pid.step(angular_error, sample_step)
        steering = self.low_pass_filter.filt(steering)
        return steering

    def get_steering_pid_cte(self, final_waypoint1, final_waypoint2, current_location, dbw_enabled):
        steering = 0
        if final_waypoint1 and final_waypoint2:
            # vector from car to first way point
            a = np.array([current_location.x - final_waypoint1.pose.pose.position.x, current_location.y - final_waypoint1.pose.pose.position.y, current_location.z - final_waypoint1.pose.pose.position.z])
            # vector from first to second way point
            b = np.array([final_waypoint2.pose.pose.position.x-final_waypoint1.pose.pose.position.x, final_waypoint2.pose.pose.position.y-final_waypoint1.pose.pose.position.y, final_waypoint2.pose.pose.position.z-final_waypoint1.pose.pose.position.z])
            # progress on vector b
            # term = (a.b / euclidian_norm(b)**2) * b where a.b is dot product
            # term = progress * b => progress = term / b => progress = (a.b / euclidian_norm(b)**2)
            progress = (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / (b[0] * b[0] + b[1] * b[1] + b[2] * b[2])
            # position where the car should be: waypoint1 + progress * b
            error_pos = np.array([final_waypoint1.pose.pose.position.x, final_waypoint1.pose.pose.position.y, final_waypoint1.pose.pose.position.z]) + progress * b
            # difference vector between where the car should be and where the car currently is
            error = (error_pos - np.array([current_location.x, current_location.y, current_location.z]))
            # is ideal track (b) left or right of the car's current heading?
            dot_product = a[0]*-b[1] + a[1]*b[0]
            direction = 1.0
            if dot_product >= 0:
                direction = -1.0
            else:
                direction = 1.0
            # Cross track error is the squared euclidian norm of the error vector: CTE = direction*euclidian_norm(error)**2
            cte = direction * math.sqrt(error[0]*error[0]+error[1]*error[1]+error[2]*error[2])
            sample_step = 0.02
            if not(self.previous_dbw_enabled) and dbw_enabled:
                self.previous_dbw_enabled = True
                self.cte_pid.reset()
                self.low_pass_filter = LowPassFilter(self.tau, self.ts)
            else:
                self.previous_dbw_enabled = False
            steering = self.cte_pid.step(cte, sample_step)
            #steering = self.low_pass_filter.filt(steering)
        return steering


from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import math

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        decel_limit 	= kwargs['decel_limit']
        accel_limit 	= kwargs['accel_limit']
        wheel_radius 	= kwargs['wheel_radius']
        wheel_base 		= kwargs['wheel_base']
        steer_ratio 	= kwargs['steer_ratio']
        max_lat_accel 	= kwargs['max_lat_accel']
        max_steer_angle = kwargs['max_steer_angle']
        self.previous_dbw_enabled = False
        min_speed = 0. #??
        yaw_params = [wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle]
        self.yaw_controller = YawController(*yaw_params)
        #self.linear_pid = PID(0.12, 0.0005, 0.04, -accel_limit, accel_limit)
        self.linear_pid = PID(0.12, 0.0005, 0.04, -accel_limit, accel_limit)
        self.tau_throttle = 0.2
        self.ts_throttle = 0.1
        self.low_pass_filter_throttle = LowPassFilter(self.tau_throttle, self.ts_throttle)
        self.tau_brake = 0.2
        self.ts_brake = 0.1
        self.low_pass_filter_brake = LowPassFilter(self.tau_brake, self.ts_brake)
        pass

    def control(self, linear_velocity_setpoint, angular_velocity_setpoint, linear_current_velocity, angular_current, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if not(self.previous_dbw_enabled) and dbw_enabled:
            self.previous_dbw_enabled = True
            self.linear_pid.reset()
        else:
            self.previous_dbw_enabled = False
        linear_velocity_error = linear_velocity_setpoint - linear_current_velocity
        #sample_step = 0.05 # ???
        #sample_step = 0.02
        sample_step = 0.02

        if abs(linear_velocity_setpoint)<0.01 and abs(linear_current_velocity) < 0.1:
            brake = 10000.
            throttle = 0.
        else:
            velocity_correction = self.linear_pid.step(linear_velocity_error, sample_step)
            throttle = velocity_correction
            brake = 0.
            if throttle < 0:
                brake = 10000*abs(throttle)
                throttle = 0.
            throttle = self.low_pass_filter_throttle.filt(throttle)
        
        #[alexm]::NOTE this lowpass leads to sending both throttle and brake nonzero. Maybe it is better to filter velocity_correction
        #brake = self.low_pass_filter_brake.filt(brake)
        #steering = self.yaw_controller.get_steering_pid(angular_velocity_setpoint, angular_current, dbw_enabled)
        
        #[alexm]::NOTE changed static 10.0 to linear_current_velocity and surprisingly car behave better on low speeds. Need to look close to formulas...
        steering = linear_current_velocity * self.yaw_controller.get_steering_calculated(linear_velocity_setpoint, angular_velocity_setpoint, linear_current_velocity)
        #[alexm]::NOTE and here is good place to think about filtering to eliminate jitter on steering wheel
        return throttle, brake, steering

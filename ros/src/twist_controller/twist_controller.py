from yaw_controller import YawController
from pid import PID
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

        min_speed = 0. #??
        yaw_params = [wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle]
        self.yaw_controller = YawController(*yaw_params)
        self.linear_pid = PID(0.12, 0.0005, 0.04, -accel_limit, accel_limit)
        pass

    def control(self, linear_velocity_setpoint, angular_velocity_setpoint, linear_current_velocity, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        linear_velocity_error = linear_velocity_setpoint - linear_current_velocity
        sample_step = 0.05 # ???
        velocity_correction = self.linear_pid.step(linear_velocity_error, sample_step)
        throttle = velocity_correction
        brake = 0.
        if throttle < 0:
            brake = 10000*abs(throttle)
            throttle = 0.

        steering = self.yaw_controller.get_steering(linear_velocity_setpoint, angular_velocity_setpoint, linear_current_velocity)
        steering = 10*steering
        return throttle, brake, steering

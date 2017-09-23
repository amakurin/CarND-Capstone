from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import math
import rospy
from styx_msgs.msg import Lane, Waypoint
from geometry_msgs.msg import PoseStamped, Pose

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        vehicle_mass    = kwargs['vehicle_mass']
        fuel_capacity   = kwargs['fuel_capacity']
        self.brake_deadband  = kwargs['brake_deadband']
        self.decel_limit 	= kwargs['decel_limit']
        accel_limit 	= kwargs['accel_limit']
        wheel_radius 	= kwargs['wheel_radius']
        wheel_base 		= kwargs['wheel_base']
        steer_ratio 	= kwargs['steer_ratio']
        max_lat_accel 	= kwargs['max_lat_accel']
        max_steer_angle = kwargs['max_steer_angle']

        self.brake_tourque_const = (vehicle_mass + fuel_capacity * GAS_DENSITY) * wheel_radius
        self.current_dbw_enabled = False
        yaw_params = [wheel_base, steer_ratio, max_lat_accel, max_steer_angle]
        self.yaw_controller = YawController(*yaw_params)
        self.linear_pid = PID(0.9, 0.0005, 0.07, self.decel_limit, accel_limit)
        self.tau_correction = 0.2
        self.ts_correction = 0.1
        self.low_pass_filter_correction = LowPassFilter(self.tau_correction, self.ts_correction)
        self.previous_time = None
        pass

    def update_sample_step(self):
        current_time = rospy.get_time() 
        sample_step = current_time - self.previous_time if self.previous_time else 0.05
        self.previous_time = current_time
        return sample_step

    def control(self, linear_velocity_setpoint, angular_velocity_setpoint, linear_current_velocity, angular_current, dbw_enabled, final_waypoint1, final_waypoint2, current_location):
        if (not self.current_dbw_enabled) and dbw_enabled:
            self.current_dbw_enabled = True
            self.linear_pid.reset()
            self.previous_time = None
        else:
            self.current_dbw_enabled = False
        linear_velocity_error = linear_velocity_setpoint - linear_current_velocity

        sample_step = self.update_sample_step()

        velocity_correction = self.linear_pid.step(linear_velocity_error, sample_step)
        velocity_correction = self.low_pass_filter_correction.filt(velocity_correction)
        if abs(linear_velocity_setpoint)<0.01 and abs(linear_current_velocity) < 0.3:
            velocity_correction = self.decel_limit
        throttle = velocity_correction
        brake = 0.
        if throttle < 0.:
            decel = abs(throttle)
            #[alexm]NOTE: let engine decelerate the car if required deceleration below brake_deadband
            brake = self.brake_tourque_const * decel if decel > self.brake_deadband else 0.
            throttle = 0.
        
        #[alexm]::NOTE this lowpass leads to sending both throttle and brake nonzero. Maybe it is better to filter velocity_correction
        #brake = self.low_pass_filter_brake.filt(brake)
        #steering = self.yaw_controller.get_steering_pid(angular_velocity_setpoint, angular_current, dbw_enabled)
        #steering = self.yaw_controller.get_steering_pid_cte(final_waypoint1, final_waypoint2, current_location, dbw_enabled)

        #[alexm]::NOTE changed static 10.0 to linear_current_velocity and surprisingly car behave better on low speeds. Need to look close to formulas...
        #PID also improves the same with the factor
        #moved factor into function because limits are checked in that function
        steering = self.yaw_controller.get_steering_calculated(linear_velocity_setpoint, angular_velocity_setpoint, linear_current_velocity)

        return throttle, brake, steering

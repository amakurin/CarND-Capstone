#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped 
from styx_msgs.msg import Lane, Waypoint, TrafficLight, TrafficLightArray
from std_msgs.msg import Int32 
import math
import copy
import tf
from scipy.interpolate import CubicSpline

LOOKAHEAD_WPS = 100  # Number of waypoints we will publish. You can change this number
POSITION_SEARCH_RANGE = 10  # Number of waypoints to search current position back and forth

MAX_DECEL = 1.

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
#        rospy.Subscriber('/vehicle/traffic_lights',
#                         TrafficLightArray, self.traffic_cb, queue_size=1)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.next_wp_pub = rospy.Publisher('/next_wp', Int32, queue_size=1)

        # current path
        # NOTE: supposedly comes from top level planner (from file for simulator) at rate 40Hz
        self.current_waypoints = None

        # current pose
        # NOTE: supposedly comes from fusion (briged from simulator) at unknown rate
        self.current_pose = None
        self.next_waypoint_index = None
        
        self.stop_trajectory = None

        rospy.spin()

    def euclidean_distance_2d(self, position1, position2):
        a = position1
        b = position2
        return math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2)

    def get_closest_waypoint(self):
        min_dist = 100000
        min_ind = 0
        ind = 0
        
        start = 0 
        end = len(self.current_waypoints)
        if (self.next_waypoint_index):
            start = max (self.next_waypoint_index - POSITION_SEARCH_RANGE, 0)
            end = min (self.next_waypoint_index + POSITION_SEARCH_RANGE, end)

        position1 = self.current_pose.pose.position
        for i in range(start, end):
            position2 = self.current_waypoints[i].pose.pose.position
            dist = self.euclidean_distance_2d(position1, position2)
            if dist < min_dist:
                min_dist = dist
                min_ind = i
        return min_ind

    def current_yaw(self):
        quaternion = (
            self.current_pose.pose.orientation.x,
            self.current_pose.pose.orientation.y,
            self.current_pose.pose.orientation.z,
            self.current_pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        return euler[2]

    def update_next_waypoint(self):
        ind = self.get_closest_waypoint()

        map_x = self.current_waypoints[ind].pose.pose.position.x
        map_y = self.current_waypoints[ind].pose.pose.position.y

        x = self.current_pose.pose.position.x
        y = self.current_pose.pose.position.y

        yaw = self.current_yaw()
        
        x_car_system = ((map_x-x) * math.cos(yaw) + (map_y-y) * math.sin(yaw))
        if ( x_car_system < 0. ):
            ind += 1
        self.next_waypoint_index = ind
        return ind

    def pose_cb(self, msg):
        self.current_pose = msg

        if (not rospy.is_shutdown() and (self.current_waypoints is not None)):
            next_waypoint_index = self.update_next_waypoint()
            self.next_wp_pub.publish(Int32(next_waypoint_index))

            lane = Lane()
            lane.header.frame_id = self.current_pose.header.frame_id
            lane.header.stamp = rospy.Time(0)
            lane.waypoints = self.current_waypoints[next_waypoint_index:next_waypoint_index+LOOKAHEAD_WPS] 
            
            if (self.stop_trajectory):
                start_index = self.stop_trajectory[0]
                velocities = self.stop_trajectory[1]
                shift = 0 if (start_index == next_waypoint_index) else (next_waypoint_index - start_index)
                for i in range(min(LOOKAHEAD_WPS, len(lane.waypoints))):
                    shifted_i = i + shift
                    lane.waypoints[i] = copy.deepcopy(lane.waypoints[i])
                    lane.waypoints[i].twist.twist.linear.x = velocities[shifted_i] if (0 <= shifted_i < len(velocities)) else 0.

            self.final_waypoints_pub.publish(lane)


    def waypoints_cb(self, lane):
        self.current_waypoints = lane.waypoints
        #[alexm]NOTE: full search current position on each update of waypoints
        self.next_waypoint_index = None

    def set_stop_trajectory(self, next_waypoint_index, stop_line_index):
        if (self.stop_trajectory):
            old_start = self.stop_trajectory[0]
            velocities = self.stop_trajectory[1]
            self.stop_trajectory = [next_waypoint_index, velocities[next_waypoint_index-old_start:]]
        else:   
            #[alexm]NOTE: gererate simplest linear trajectory. Consider fitting polynomial. 
            if (stop_line_index >= next_waypoint_index):
                stop_distance = self.distance(self.current_waypoints, next_waypoint_index, stop_line_index) 
                full_stop_velocity = math.sqrt(2 * MAX_DECEL * stop_distance)
                target_velocity = self.current_waypoints[next_waypoint_index].twist.twist.linear.x
                v0 = min(full_stop_velocity, target_velocity)
                cs = CubicSpline([0., stop_distance/2, stop_distance], [v0, v0/2, 0.])
                distance = 0
                velocities = []
                for i in range(next_waypoint_index, stop_line_index+1):
                    velocity_setpoint = cs(distance).tolist()
                    velocities.append(velocity_setpoint)
                    distance += self.distance(self.current_waypoints, i, i + 1)
                self.stop_trajectory = [next_waypoint_index, velocities]

    def traffic_cb(self, traffic_waypoint):
        if (self.next_waypoint_index is None):
            return
        if (self.current_waypoints is None):
            return

        stop_line_index = traffic_waypoint.data
        if (stop_line_index > -1):
            self.set_stop_trajectory(self.next_waypoint_index, stop_line_index)
            #rospy.logerr("stop_trajectory: %s\n ", self.stop_trajectory)
        else:  
            self.stop_trajectory = None


    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0

        def dl(a, b): return math.sqrt((a.x - b.x) **
                                       2 + (a.y - b.y)**2 + (a.z - b.z)**2)
        for i in range(wp1, wp2 + 1):
            dist += dl(waypoints[wp1].pose.pose.position,
                       waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

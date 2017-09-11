#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLight, TrafficLightArray

import math
import tf

# Alternatives
# 1. Find the nearest traffic light in front of the car, check whether it is changed (IMPLEMENTED)
# 2. Get a static list of traffic lights in the first iteration, assign waypoints (to stop) to them

# Decisions
# 1. When accelerating,
#   a. the linear.x should be equal to the target speed (IMPLEMENTED)
#   b. should increased like in the decelerate function
# 2. Yellow light behavior
# 3. Ignore Red Light distance

DEBUG_ROSPY_TL = False # On ropsy terminal, debug nearest traffic light and status
DEBUG_PYTHON_TL = False # On python output, debug nearest traffic light and status
DEBUG_PYTHON_PUB = False # On python output, debug published waypoints
DEBUG_PYTHON_WP = False # On python output, waypoint to stop for traffic light
DEBUG_PYTHON_ACCDEC = False # On python output, acceleration and deceleration of waypoints

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number
TRAFFIC_LIGHT_WPS = 50 # Number of waypoints to update velocity (linear.x) for traffic light changes

TRAFFIC_LIGHT_DISTANCE = 100.0 # The maximum distance to investigate a traffic light
TRAFFIC_LIGHT_STOP_DISTANCE = 35.0 # Target distance to stop before a traffic light
TRAFFIC_LIGHT_SAME_DISTANCE = 1.0 # Distance to assume two traffic lights to be same

MAX_ACC = 0.5  # Acceleration rate
MAX_SPEED = 11.2 # Target speed

IGNORE_YELLOW_DISTANCE = 40.0 # Maximum distance to ignore YELLOW light
IGNORE_RED_DISTANCE = 30.0 # Maximum distance to ignore RED light


class WaypointUpdater(object):
    def __init__(self):
        # DEBUG
        self.tlc = 0 # Traffic light callback counter

        # Nearest traffic light
        self.min_dist_light = None

        #rospy.init_node('waypoint_updater')
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)
        rospy.Subscriber('/vehicle/traffic_lights',
                         TrafficLightArray, self.traffic_cb, queue_size=1)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher(
            'final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        # current path
        # NOTE: supposedly comes from top level planner (from file for simulator) at rate 40Hz
        self.current_waypoints = None
        self.current_waypoint_size = 0

        # current pose
        # NOTE: supposedly comes from fusion (briged from simulator) at unknown rate
        self.current_pose = None

        self.loop()

    def loop(self):
        rate = rospy.Rate(20)  # 40Hz
        while not rospy.is_shutdown():
            if ((self.current_pose is not None) and (self.current_waypoints is not None)):
                next_waypoint_index = self.get_next_waypoint()
                last_waypoint_index = self.add_waypoint_index(next_waypoint_index,LOOKAHEAD_WPS)
                lane = Lane()
                lane.header.frame_id = '/world'
                lane.header.stamp = rospy.Time(0)
                lane.waypoints = self.get_waypoint_slice(next_waypoint_index,last_waypoint_index)

                for i in range(LOOKAHEAD_WPS):
                    wp_index = self.add_waypoint_index(next_waypoint_index,i)
                    wp = self.current_waypoints[wp_index]
                    if DEBUG_PYTHON_PUB:
                        print("PUB ", self.tlc, wp_index, "wp_size:", self.current_waypoint_size, "wp:",
                              self.position_str(wp.pose.pose.position), "v:", wp.twist.twist.linear.x)

                self.final_waypoints_pub.publish(lane)

            rate.sleep()

    def euclidean_distance(self, position1, position2):
        a = position1
        b = position2
        return math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2 + (a.z - b.z)**2)

    def euclidean_distance_2d(self, position1, position2):
        a = position1
        b = position2
        return math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2)

    def get_closest_waypoint(self):
        min_dist = 100000
        min_ind = 0
        ind = 0
        position1 = self.current_pose.pose.position
        for wp in self.current_waypoints:
            position2 = wp.pose.pose.position
            dist = self.euclidean_distance(position1, position2)
            if dist < min_dist:
                min_dist = dist
                min_ind = ind
            ind += 1
        return min_ind

    def current_yaw(self):
        quaternion = (self.current_pose.pose.orientation.x)
        quaternion = (
            self.current_pose.pose.orientation.x,
            self.current_pose.pose.orientation.y,
            self.current_pose.pose.orientation.z,
            self.current_pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        return euler[2]

    def get_next_waypoint(self):
        ind = self.get_closest_waypoint()

        map_x = self.current_waypoints[ind].pose.pose.position.x
        map_y = self.current_waypoints[ind].pose.pose.position.y

        x = self.current_pose.pose.position.x
        y = self.current_pose.pose.position.y

        heading = math.atan2((map_y - y), (map_x - x))
        yaw = self.current_yaw()
        angle = abs(yaw - heading)

        if (angle > math.pi / 4):
            ind += 1

        return ind

    def pose_cb(self, msg):
        self.current_pose = msg

    def waypoints_cb(self, lane):
        if self.current_waypoints is None:
            self.current_waypoints = lane.waypoints
            self.current_waypoint_size = len(self.current_waypoints)

    def add_waypoint_index(self, index, add_to_index):
        """Find new_index, 'add_to_index' waypoints after the 'index'
           To prevent out of index
        """
        new_index = index + add_to_index
        return new_index % self.current_waypoint_size

    def get_waypoint_slice(self, first_index, last_index):
        """ Get a slice from self.current_waypoints,
            preventing out of index
        """
        if last_index >= first_index:
            return self.current_waypoints[first_index:last_index]
        else:
            return self.current_waypoints[first_index:] + self.current_waypoints[:last_index]

    def position_str(self, position):
        """FOR DEBUGGING ONLY"""
        return "x:%s,y:%s,z:%s" % (position.x, position.y, position.z)

    def orientation_str(self, orientation):
        """FOR DEBUGGING ONLY"""
        return "x:%s,y:%s,z:%s,w:%s" % (orientation.x, orientation.y, orientation.z, orientation.w)

    def accelerate(self, waypoint_index):
        """ if vel_to_max = True
            Set the velocity to MAX_SPEED
            For the waypoints from waypoint_index to
            waypoint_index + TRAFFIC_LIGHT_WPS
            if vel_to_max = False (NOT IMPLEMENTED!!!)
            Increase the velocity for each waypoint

        """
        last = self.current_waypoints[waypoint_index]
        vel_to_max = True
        last_index = self.add_waypoint_index(waypoint_index, TRAFFIC_LIGHT_WPS)
        for wp in self.get_waypoint_slice(waypoint_index, last_index):
            if vel_to_max:
                wp.twist.twist.linear.x = MAX_SPEED
            else:
                dist = self.euclidean_distance(wp.pose.pose.position,
                                               last.pose.pose.position)
                vel = math.sqrt(2 * MAX_ACC * dist) * 3.6
                # if vel < 1.: # ACC > 10 ?
                if vel < 0.:
                    vel = 0.
                elif vel > MAX_SPEED:
                    vel = MAX_SPEED
                current_v = wp.twist.twist.linear.x
                if DEBUG_PYTHON_ACCDEC:
                    print("ACC pre_v:", current_v, "vel:", vel,
                          "dist:", dist, "wp:", self.position_str(wp.pose.pose.position))
                wp.twist.twist.linear.x = vel

    def decelerate(self, waypoint_index):
        """ Starting from waypoint_index, and going backwards set the velocities.
            Waypoint at waypoint_index should have a velocity of 0.0
            Increase the velocity backwards.

            From waypoint_index to waypoint_index + TRAFFIC_LIGHT_WPS,
            set the velocity to 0.0.
            Otherwise, car don't stop.
        """
        last = self.current_waypoints[waypoint_index]
        if DEBUG_PYTHON_ACCDEC:
            print("DEC pre_v:", last.twist.twist.linear.x, "vel:", 0,
                  "dist:", 0, "wp:", self.position_str(last.pose.pose.position))
        last.twist.twist.linear.x = 0.

        for wp in self.current_waypoints[:waypoint_index][::-1]:
            dist = self.euclidean_distance(wp.pose.pose.position,
                                           last.pose.pose.position)
            vel = math.sqrt(2 * MAX_ACC * dist) * 3.6
            if vel < 1.:
                vel = 0.
            current_v = wp.twist.twist.linear.x
            if DEBUG_PYTHON_ACCDEC:
                print("DEC pre_v:", current_v, "vel:", vel,
                      "dist:", dist, "wp:", self.position_str(wp.pose.pose.position))
            if (vel > current_v):
                break
            else:
                wp.twist.twist.linear.x = vel
        last_index = self.add_waypoint_index(waypoint_index, TRAFFIC_LIGHT_WPS)
        for wp in self.get_waypoint_slice(waypoint_index, last_index):
            wp.twist.twist.linear.x = 0.0

    def same_light(self, light1, light2):
        position1 = light1.pose.pose.position
        position2 = light2.pose.pose.position
        dist = self.euclidean_distance(position1, position2)
        return (dist <= TRAFFIC_LIGHT_SAME_DISTANCE)

    def waypoint_to_stop(self, next_waypoint_index, light):
        light_wp_idx = - 1
        light_position = light.pose.pose.position
        for i in range(LOOKAHEAD_WPS):
            wp_index = self.add_waypoint_index(next_waypoint_index,i)
            wp = self.current_waypoints[wp_index]
            wp_position = wp.pose.pose.position
            wp_dist = self.euclidean_distance_2d(
                wp_position, light_position)
            if (DEBUG_PYTHON_WP):
                print("WP", self.tlc, "nwidx:", next_waypoint_index, "i:", i, "dist:", wp_dist, "wp: ", self.position_str(
                    wp_position), "wo:", self.orientation_str(wp.pose.pose.orientation))
            if (wp_dist < TRAFFIC_LIGHT_STOP_DISTANCE):
                if (DEBUG_PYTHON_WP):
                    print("light distance is less than thres, stop")
                light_wp_idx = next_waypoint_index + i - 1
                break
        if (DEBUG_PYTHON_WP):
            print("lwidx:", light_wp_idx, "nwidx:", next_waypoint_index)
        return light_wp_idx


    def traffic_cb(self, traffic_lights):
        """ Finds the nearest traffic_light within TRAFFIC_LIGHT_DISTANCE
            If it doesn't change from the previous callback, do nothing.
            If the light changed, or it's status changed, update the velocities
                of waypoints.
            If no near light exists, or the existing light is GREEN, accelerate
            If near light is red or yellow, and the light is not too close to the car
                decelerate.
        """
        # TODO: Callback for /traffic_waypoint message. Implement

        if (self.current_pose is None):
            return
        if (self.current_waypoints is None):
            return

        lights = traffic_lights.lights

        car_position = self.current_pose.pose.position
        next_waypoint_index = self.get_next_waypoint()
        min_dist = TRAFFIC_LIGHT_DISTANCE
        min_dist_light = None
        status_changed = False
        for light in lights:
            light_position = light.pose.pose.position
            dist = self.euclidean_distance_2d(car_position, light_position)
            if dist < min_dist:
                next_position = self.current_waypoints[next_waypoint_index].pose.pose.position
                next_dist = self.euclidean_distance_2d(next_position, light_position)
                # if the Traffic Light is in front of the car
                # if TL is getting closer at the next way point
                # TODO next waypoint may be too close to the current pose,
                #   which will result in error when detecting nearest light.
                if next_dist < dist:
                    min_dist = dist
                    min_dist_light = light

        if ((self.min_dist_light is None) and
            (min_dist_light is None)):
            status_changed = False
        elif (self.min_dist_light is None):
            status_changed = True
        elif (min_dist_light is None):
            status_changed = True
        elif (not self.same_light(self.min_dist_light, min_dist_light)):
            status_changed = True
        elif (self.min_dist_light.state != min_dist_light.state):
            status_changed = True

        if (status_changed):
            update_min_dist_light = True
            DEBUG_TL = DEBUG_ROSPY_TL or DEBUG_PYTHON_TL
            logstr = ""
            if (min_dist_light is None):
                if (DEBUG_TL):
                    logstr = ("Light is NONE" +
                               " nwpidx: " + str(next_waypoint_index) +
                               " cp: " + self.position_str(car_position))
                self.accelerate(next_waypoint_index)
            else:
                light = min_dist_light
                light_position = light.pose.pose.position
                ststr = "RED"
                if light.state == 1:
                    ststr = "YELLOW"
                elif light.state == 2:
                    ststr = "GREEN"
                elif light.state != 0:
                    ststr = "UNKWN"
                if (DEBUG_TL):
                    logstr = ("Light is " + ststr + "(" + str(light.state) + ") dist:" + str(min_dist) + " lp: " +
                                   self.position_str(light_position) +
                                   " nwpidx: " + str(next_waypoint_index) +
                                   " cp: " + self.position_str(car_position) +
                                   " seq:%s" %(traffic_lights.header.seq))

                if light.state in [0,1]:
                    if light.state == 0 and min_dist < IGNORE_RED_DISTANCE:
                        if (DEBUG_TL):
                            logstr += " IGNORED "
                            logstr += str(min_dist)
                            logstr += " < "
                            logstr += str(IGNORE_RED_DISTANCE)
                    elif light.state == 1 and min_dist < IGNORE_YELLOW_DISTANCE:
                        if (DEBUG_TL):
                            logstr += " IGNORED "
                            logstr += str(min_dist)
                            logstr += " < "
                            logstr += str(IGNORE_YELLOW_DISTANCE)
                    else:
                        light_wp_idx = self.waypoint_to_stop(next_waypoint_index, light)
                        if (DEBUG_TL):
                            logstr += " lwidx:"
                            logstr += str(light_wp_idx)
                        if light_wp_idx != -1:
                            self.decelerate(light_wp_idx)
                            if (DEBUG_TL):
                                logstr += " lwp: "
                                logstr += self.position_str(self.current_waypoints[light_wp_idx].pose.pose.position)
                elif light.state in [2]:
                    self.accelerate(next_waypoint_index)
                    if (DEBUG_TL):
                        logstr += " ACC FROM nwidx:"
                        logstr += str(next_waypoint_index)
                        logstr += " nwp: "
                        logstr += self.position_str(self.current_waypoints[next_waypoint_index].pose.pose.position)
                elif light.state in [4]:
                    update_min_dist_light = False
                    if (DEBUG_TL):
                        logstr += " UNKNOWN "
            if DEBUG_ROSPY_TL:
                rospy.logerr(logstr)
            if DEBUG_PYTHON_TL:
                print(logstr)

            if (update_min_dist_light):
                self.min_dist_light = min_dist_light

        self.tlc += 1

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

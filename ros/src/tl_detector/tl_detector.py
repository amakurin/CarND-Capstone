#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose, Point
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import math
import cv2
import yaml
import numpy as np

STATE_COUNT_THRESHOLD = 3

TRAFFIC_LIGHT_DISTANCE = 80.0 # The maximum distance to investigate a traffic light

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.stop_line_positions = None

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)

        self.cascade = cv2.CascadeClassifier('./cascade.xml') # Haar cascade for TL detection

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and 
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        #[alexm]NOTE: we should rely on this topic's data except state of the light 
        #[alexm]NOTE: according to this: https://carnd.slack.com/messages/C6NVDVAQ3/convo/C6NVDVAQ3-1504625321.000063/
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb, queue_size=1)
        rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1)
        rospy.Subscriber('/next_wp', Int32, self.next_wp_cb, queue_size=1)
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.next_wp = None

        rospy.spin()

    def next_wp_cb(self, val):
        self.next_wp = val.data

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, lane):
        self.waypoints = lane.waypoints

    def find_stop_line_position(self, light):
        stop_line_positions = self.config['light_positions']
        min_distance = 100000
        result = None
        light_pos = light.pose.pose.position
        for pos in stop_line_positions:
            distance = self.euclidean_distance_2d(pos, light_pos)
            if (distance< min_distance):
                min_distance = distance
                result = pos
        return result

    def traffic_cb(self, msg):
        if self.lights != msg.lights:
            self.stop_line_positions = []
            for light in msg.lights:
                self.stop_line_positions.append(self.find_stop_line_position(light))
            self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state in [TrafficLight.RED, TrafficLight.YELLOW] else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """

        fx = self.config['camera_info']['focal_length_x']
        fy = self.config['camera_info']['focal_length_y']
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        # get transform between pose of camera and world frame
        trans = None
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link",
                  "/world", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link",
                  "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")

        #TODO Use tranform and rotation to calculate 2D position of light in image

        x = 0
        y = 0

        return (x, y)

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        x, y = self.project_to_image_plane(light.pose.pose.position)

        #TODO use light location to zoom in on traffic light in image

        #Get classification
        #return self.light_classifier.get_classification(cv_image)
        #[alexm]NOTE: Temporal stub till detection\classification readiness
        box = self.cascade.detectMultiScale(cv_image, 1.25, 20)
        state = TrafficLight.UNKNOWN
        for (x,y,w,h) in box:
            w1 = int(w*0.75) # Fix aspect ratio and size
            h1 = int(h*0.5)
            x1 = x+int((w-w1)/2)
            y1 = y+int((h-h1)/2)
            dh=int(h1*0.05)
            line = cv_image[(y1+dh):(y1+h1-dh),int(x1+w1/2),:]
            if np.max(line[:,2]) > 245 and np.max(line[:,1]) > 245: # Yellow
                state = TrafficLight.YELLOW
                continue
            if np.max(line[:,1]) > 245: # Green
                state = TrafficLight.GREEN
                continue
            if np.max(line[:,2]) > 245: # Red
                state = TrafficLight.RED
                break  # Red has high priority, so, return it if it is seen
        #print(state)
        return state

    def create_light(self, x, y, z, yaw, state):
        light = TrafficLight()

        light.header = Header()
        light.header.stamp = rospy.Time.now()
        light.header.frame_id = '/world'

        light.pose = self.create_pose(x, y, z, yaw)
        light.state = state

        return light

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        light = None
        stop_line_position = None
        light_wp = -1
        if(self.pose and self.waypoints and self.next_wp and self.stop_line_positions):
            #[alexm]NOTE: first find closest light to next wp
            #[alexm]NOTE: this is simple (WRONG) version. Actually we should select all lights ahead that in range. 
            min_distance = TRAFFIC_LIGHT_DISTANCE
            waypoints_size = len(self.waypoints)
            next_pose = self.waypoints[min(self.next_wp, waypoints_size-1)].pose.pose
            for light_index in range(len(self.lights)):
                current_light = self.lights[light_index]
                light_position = current_light.pose.pose.position
                distance = self.euclidean_distance_2d(next_pose.position, light_position)
                if (distance < min_distance) and self.is_ahead(next_pose, light_position):
                    min_distance = distance
                    light = current_light
                    stop_line_position = self.get_stop_line_position(light_index)
            
            #[alexm]NOTE: next if light was found, try to find it on path ahead
            if light:
                #rospy.logerr("has light: %s\n ", stop_line_position)
                current_position = next_pose.position
                min_distance = TRAFFIC_LIGHT_DISTANCE 
                total_distance = 0
                for i in range(self.next_wp, waypoints_size):
                    wp_pose = self.waypoints[i].pose.pose
                    distance = self.euclidean_distance_2d(wp_pose.position, stop_line_position)                                
                    if (distance < min_distance) and self.is_ahead(wp_pose, stop_line_position):
                        min_distance = distance
                        light_wp = max(i - 1, 0)
                    if (i+1 < waypoints_size):
                        total_distance += self.euclidean_distance_2d(wp_pose.position, self.waypoints[i+1].pose.pose.position)
                        if total_distance > TRAFFIC_LIGHT_DISTANCE:
                            break;
        if light_wp > -1:
            state = self.get_light_state(light)
            return light_wp, state
        #self.waypoints = None
        return -1, TrafficLight.UNKNOWN

    def get_stop_line_position(self, light_index):
        return self.stop_line_positions[light_index]


    def is_ahead(self, origin_pose, test_position):
        test_x = self.get_x(test_position)
        test_y = self.get_y(test_position)

        orig_posit = origin_pose.position
        orig_orient = origin_pose.orientation
        quaternion = (orig_orient.x, orig_orient.y, orig_orient.z, orig_orient.w)
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
        orig_x = ((test_x - orig_posit.x) * math.cos(yaw) \
                + (test_y - orig_posit.y) * math.sin(yaw))
        return orig_x > 0.

    def euclidean_distance_2d(self, position1, position2):

        a_x = self.get_x(position1)
        a_y = self.get_y(position1)
        b_x = self.get_x(position2)
        b_y = self.get_y(position2)
        return math.sqrt((a_x - b_x)**2 + (a_y - b_y)**2)

    def get_x(self, pos):
        return pos.x if isinstance(pos, Point) else pos[0] 
    def get_y(self, pos):
        return pos.y if isinstance(pos, Point) else pos[1] 


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')

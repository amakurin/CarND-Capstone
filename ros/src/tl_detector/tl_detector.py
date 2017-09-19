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
from scipy import spatial
import os
import glob
import time
import itertools
import scipy.misc
import numpy as np
import cv2
import copy
import math
import matplotlib.image as mpimg
import matplotlib.pyplot as plt

from sklearn.svm import LinearSVC, SVC
from sklearn.naive_bayes import MultinomialNB
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.ensemble import ExtraTreesClassifier
from sklearn.feature_selection import SelectFromModel
from sklearn.externals import joblib
from sklearn.pipeline import Pipeline
from scipy.ndimage.measurements import label
from scipy import ndimage as ndi
from skimage.feature import hog, blob_doh, peak_local_max
from skimage.morphology import watershed, disk
from skimage.filters import rank, gaussian_filter
from skimage.util import img_as_ubyte

STATE_COUNT_THRESHOLD = 3

MAX_DECEL = 1.

#Functions for traffic light feature extraction (Traffic light state classification)
# Define a function to return HOG features and visualization
def get_hog_features(img, orient, pix_per_cell, cell_per_block,
                     vis=False, feature_vec=True):
    # Call with two outputs if vis==True
    if vis == True:
        features, hog_image = hog(img, orientations=orient,
                                  pixels_per_cell=(pix_per_cell, pix_per_cell),
                                  cells_per_block=(cell_per_block, cell_per_block),
                                  transform_sqrt=True,
                                  visualise=vis, feature_vector=feature_vec)
        return features, hog_image
    # Otherwise call with one output
    else:
        features = hog(img, orientations=orient,
                       pixels_per_cell=(pix_per_cell, pix_per_cell),
                       cells_per_block=(cell_per_block, cell_per_block),
                       transform_sqrt=True,
                       visualise=vis, feature_vector=feature_vec)
        return features


# Define a function to compute binned color features
def bin_spatial(img, size=(32, 32)):
    # Use cv2.resize().ravel() to create the feature vector
    features = cv2.resize(img, size).ravel()
    # Return the feature vector
    return features


# Define a function to compute color histogram features
# NEED TO CHANGE bins_range if reading .png files with mpimg!
def color_hist(img, nbins=32, bins_range=(0, 256)):
    # Compute the histogram of the color channels separately
    channel1_hist = np.histogram(img[:, :, 0], bins=nbins, range=bins_range)
    channel2_hist = np.histogram(img[:, :, 1], bins=nbins, range=bins_range)
    channel3_hist = np.histogram(img[:, :, 2], bins=nbins, range=bins_range)
    # Concatenate the histograms into a single feature vector
    hist_features = np.concatenate((channel1_hist[0], channel2_hist[0], channel3_hist[0]))
    # Return the individual histograms, bin_centers and feature vector
    return hist_features


# Define a function to extract features from a list of images
# Have this function call bin_spatial() and color_hist()
def extract_features(img, color_space='RGB', spatial_size=(32, 32),
                     hist_bins=32, orient=9,
                     pix_per_cell=8, cell_per_block=2, hog_channel=0,
                     spatial_feat=True, hist_feat=True, hog_feat=True, resize=False):
    # Create a list to append feature vectors to
    features = []
    # Iterate through the list of images
    file_features = []
    # Read in each one by one
    image = img
    if resize == True:
        image = cv2.resize(image, (15, 30))
    # apply color conversion if other than 'RGB'
    if color_space != 'RGB':
        if color_space == 'HSV':
            feature_image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        elif color_space == 'LUV':
            feature_image = cv2.cvtColor(image, cv2.COLOR_RGB2LUV)
        elif color_space == 'HLS':
            feature_image = cv2.cvtColor(image, cv2.COLOR_RGB2HLS)
        elif color_space == 'YUV':
            feature_image = cv2.cvtColor(image, cv2.COLOR_RGB2YUV)
        elif color_space == 'YCrCb':
            feature_image = cv2.cvtColor(image, cv2.COLOR_RGB2YCrCb)
    else:
        feature_image = np.copy(image)

    if spatial_feat == True:
        spatial_features = bin_spatial(feature_image, size=spatial_size)
        file_features.append(spatial_features)
    if hist_feat == True:
        # Apply color_hist()
        hist_features = color_hist(feature_image, nbins=hist_bins)
        file_features.append(hist_features)
    if hog_feat == True:
        # Call get_hog_features() with vis=False, feature_vec=True
        if hog_channel == 'ALL':
            hog_features = []
            for channel in range(feature_image.shape[2]):
                hog_features.append(get_hog_features(feature_image[:, :, channel],
                                                     orient, pix_per_cell, cell_per_block,
                                                     vis=False, feature_vec=True))
            hog_features = np.ravel(hog_features)
        else:
            hog_features = get_hog_features(feature_image[:, :, hog_channel], orient,
                                            pix_per_cell, cell_per_block, vis=False, feature_vec=True)
        # Append the new feature vector to the features list
        file_features.append(hog_features)
    features.append(np.concatenate(file_features))
    # Return list of feature vectors
    return features

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.KDTree = None
        self.camera_image = None
        self.lights = []
        self.stop_lines = None

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
        if self.waypoints != lane.waypoints:
            data = []
            for wp in lane.waypoints:
                data.append((wp.pose.pose.position.x, wp.pose.pose.position.y))
            self.KDTree = spatial.KDTree(data)
            self.waypoints = lane.waypoints
            self.stop_lines = None

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
        if not self.stop_lines and self.KDTree:
            stop_lines = []
            for light in msg.lights:
                # find corresponding stop line position from config
                stop_line_pos = self.find_stop_line_position(light)
                # find corresponding waypoint indicex
                closest_index = self.KDTree.query(np.array([stop_line_pos]))[1][0]
                closest_wp = self.waypoints[closest_index]
                if not self.is_ahead(closest_wp.pose.pose, stop_line_pos):
                    closest_index = max(closest_index - 1, 0)
                # add index to list
                stop_lines.append(closest_index)
            # update lights and stop line waypoint indices
            self.lights = msg.lights
            self.stop_lines = stop_lines

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
        #cv_imagergb = cv2.cvtColor(cv_image,cv2.COLOR_BGR2RGB)

        x, y = self.project_to_image_plane(light.pose.pose.position)

        #TODO use light location to zoom in on traffic light in image
        ###############################
        # load model from pickle file
        ###############################
        data = joblib.load('models/simimg.pkl')
        # data = joblib.load('models/clf_9869.pkl')
        # svc = data['model']
        clf = data['model']

        config = data['config']
        color_space = config['color_space']
        orient = config['orient']
        pix_per_cell = config['pix_per_cell']
        cell_per_block = config['cell_per_block']
        hog_channel = config['hog_channel']
        spatial_size = config['spatial_size']
        hist_bins = config['hist_bins']
        spatial_feat = config['spatial_feat']
        hist_feat = config['hist_feat']
        hog_feat = config['hog_feat']
        resize = config['resize']


        #Get classification
        #return self.light_classifier.get_classification(cv_image)
        #[alexm]NOTE: Temporal stub till detection\classification readiness
        box = self.cascade.detectMultiScale(cv_image, 1.25, 20)
        state = TrafficLight.UNKNOWN
        for (x,y,w,h) in box:
            #w1 = int(w*0.75) # Fix aspect ratio and size
            #h1 = int(h*0.5)
            #x1 = x+int((w-w1)/2)
            #y1 = y+int((h-h1)/2)
            #dh=int(h1*0.05)
            #line = cv_image[(y1+dh):(y1+h1-dh),int(x1+w1/2),:]
            detectedlight = cv_image[y:(y+h), x:(x+w)]
            #cv2.namedWindow("Input")
            #cv2.imshow("Input", detectedlight)
            #cv2.waitKey(1)
            #cv2.imwrite("tlight" + str(time.time()) + ".png", detectedlight)
            classifylight_features = extract_features(detectedlight, color_space=color_space,
                                                      spatial_size=spatial_size, hist_bins=hist_bins,
                                                      orient=orient, pix_per_cell=pix_per_cell,
                                                      cell_per_block=cell_per_block,
                                                      hog_channel=hog_channel, spatial_feat=spatial_feat,
                                                      hist_feat=hist_feat, hog_feat=hog_feat, resize=True)
            classifydata = np.vstack((classifylight_features)).astype(np.float64)
            predictedclass = clf.predict(classifydata)
            if predictedclass == 1:
                state = TrafficLight.YELLOW
                #print "Yellow Light"
                continue
            elif predictedclass == 2:
                state = TrafficLight.GREEN
                #print "Green light"
                continue
            elif predictedclass == 0:
                state = TrafficLight.RED
                #print "Red Light"
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
        light_wp = -1

        if(self.waypoints and self.next_wp and self.stop_lines):
            next_wp = self.waypoints[min(self.next_wp, len(self.waypoints)-1)]
            target_velocity = next_wp.twist.twist.linear.x
            search_distance = target_velocity * target_velocity / 2 / MAX_DECEL
            min_distance = search_distance
            for i in range(len(self.stop_lines)):
                stop_line_wp_index = self.stop_lines[i]
                if stop_line_wp_index >= self.next_wp:
                    stop_line_wp = self.waypoints[stop_line_wp_index]
                    distance = self.euclidean_distance_2d(next_wp.pose.pose.position, stop_line_wp.pose.pose.position)
                    if (distance < min_distance):
                        light_wp = stop_line_wp_index
                        light = self.lights[i]
        if light_wp > -1:
            state = self.get_light_state(light)
            return light_wp, state
        #self.waypoints = None
        return -1, TrafficLight.UNKNOWN

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

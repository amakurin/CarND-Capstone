#!/usr/bin/env python
# A simple node for saving images from simulator
# Run it by 'python img_saver_node.py' command while running the project ros

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

save_dir = '.' # EDIT Set the directory for images

i = 0
def img_save(img):
    global i
    img.encoding = "rgb8"
    cv_image = CvBridge().imgmsg_to_cv2(img, "bgr8")
    cv2.imwrite(save_dir+'/img_'+'%06d'%i+'.png', cv_image)
    i += 1

def img_listener():
    rospy.init_node('img_saver', anonymous=True)
    rospy.Subscriber('/image_color', Image, img_save)
    rospy.spin()


if __name__ == '__main__':
    try:
        img_listener()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start image saver node.')

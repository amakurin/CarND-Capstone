from styx_msgs.msg import TrafficLight
from keras.models import load_model
from keras.preprocessing.image import img_to_array, load_img
import tensorflow as tensorflow
import cv2
import numpy as np

class TLClassifier(object):
    def __init__(self):
        self.cascade = cv2.CascadeClassifier('./cascade_gen.xml') # Haar cascade for TL detection
        self.test_model = load_model('./models/tl_state_aug_v3.h5')
        self.graph = tensorflow.get_default_graph()
        self.save_counter = 0
        pass

    def get_classification(self, cv_image):
        """Determines the color of the traffic light in the image

        Args:
            cv_image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
#        clonned = cv_image.copy()

        box = self.cascade.detectMultiScale(cv_image, 1.3, 3)
        state = TrafficLight.UNKNOWN
        img_width, img_height = 150, 150
        for (x,y,w,h) in box:
            # FP filter
            dh=int(round(h*0.1))
            line = cv_image[(y+dh):(y+h-dh),int(round(x+w/2)),:]
            if np.std(line) < 32: # Magic number out of experiments
                print "False Detection!"
                continue # FP detection
            tl_img = cv_image[y:(y + h), x:(x + w)]
            tl_img_rgb = cv2.resize(tl_img, (img_width, img_height))
            tl_img_rgb = cv2.cvtColor(tl_img_rgb , cv2.COLOR_BGR2RGB)
            tl_img_data = img_to_array(tl_img_rgb)
            tl_img_data = np.expand_dims(tl_img_data, axis=0)
            with self.graph.as_default():
                predictedclass = self.test_model.predict_classes(tl_img_data, verbose=False)

            if int(predictedclass) == 2:
                state = TrafficLight.YELLOW
#                cv2.rectangle(clonned, (x,y), (x+w,y+h), (0, 255, 255))
                print "Yellow Light"
                continue
            elif int(predictedclass) == 1:
                state = TrafficLight.GREEN
#                cv2.rectangle(clonned, (x,y), (x+w,y+h), (0, 255, 0))
                print "Green light"
                continue
            elif int(predictedclass) == 3:
                state = TrafficLight.RED
#                cv2.rectangle(clonned, (x,y), (x+w,y+h), (0, 0, 255))
                print "Red Light"
                break  # Red has high priority, so, return it if it is seen
            else:
#                cv2.rectangle(clonned, (x,y), (x+w,y+h), (0, 0, 0))
                continue

#        cv2.imwrite('/home/student/imgs/img_{num:03d}.jpg'.format(num=self.save_counter), clonned, )
        self.save_counter += 1

        return state

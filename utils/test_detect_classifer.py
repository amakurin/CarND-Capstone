# A script for testing traffic light detector and classifier on a set of images

import os
import cv2
import numpy as np
import tensorflow as tensorflow
from keras.models import load_model
from keras.preprocessing.image import img_to_array, load_img

input_dir = 'loop' # A path to the directory with input images

# Faster Non-Maximum Suppression
# From http://www.pyimagesearch.com/2015/02/16/faster-non-maximum-suppression-python/
# Malisiewicz et al.
def non_max_suppression_fast(boxes, overlapThresh):
    if len(boxes) == 0:
        return []
        boxes = np.array(boxes)
    if boxes.dtype.kind == "i":
        boxes = boxes.astype("float")
    pick = []
    x1 = boxes[:,0]
    y1 = boxes[:,1]
    x2 = x1+boxes[:,2]
    y2 = y1+boxes[:,3]
    area = (x2 - x1 + 1) * (y2 - y1 + 1)
    idxs = np.argsort(y1)
    while len(idxs) > 0:
        last = len(idxs) - 1
        i = idxs[last]
        pick.append(i)
        xx1 = np.maximum(x1[i], x1[idxs[:last]])
        yy1 = np.maximum(y1[i], y1[idxs[:last]])
        xx2 = np.minimum(x2[i], x2[idxs[:last]])
        yy2 = np.minimum(y2[i], y2[idxs[:last]])
        w = np.maximum(0, xx2 - xx1 + 1)
        h = np.maximum(0, yy2 - yy1 + 1)
        overlap = (w * h) / area[idxs[:last]]
        idxs = np.delete(idxs, np.concatenate(([last],
            np.where(overlap > overlapThresh)[0])))
    return boxes[pick].astype("int")

cascade = cv2.CascadeClassifier('../ros/src/tl_detector/cascade_gen.xml')
test_model = load_model('../ros/src/tl_detector/models/tl_state_vgg.h5')
graph = tensorflow.get_default_graph()

fourcc = cv2.VideoWriter_fourcc(*'XVID')
video_out = cv2.VideoWriter('test_output.avi',fourcc, 20.0, (1368,1096))

files = sorted(os.listdir(input_dir))
for fname in files:
    cv_image = cv2.imread(os.path.join(input_dir, fname))
    box = cascade.detectMultiScale(cv_image, 1.3, 3)
    box = non_max_suppression_fast(box, 0.2)
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
        col = None
        with graph.as_default():
            predictedclass = test_model.predict_classes(tl_img_data, verbose=False)
        if int(predictedclass) == 2:
            cv2.rectangle(cv_image,(x,y),(x+w,y+h),(0,255,255),2)
            print "Yellow Light"
            continue
        elif int(predictedclass) == 1:
            cv2.rectangle(cv_image,(x,y),(x+w,y+h),(0,255,0),2)
            print "Green light"
            continue
        elif int(predictedclass) == 3:
            cv2.rectangle(cv_image,(x,y),(x+w,y+h),(0,0,255),2)
            print "Red Light"
            break  # Red has high priority, so, return it if it is seen
        else:
            cv2.rectangle(cv_image,(x,y),(x+w,y+h),(255,255,0),2) # No class assigned
            continue
    #cv2.imshow('img', cv_image)
    #cv2.waitKey(0)
    video_out.write(cv_image)
video_out.release()
cv2.destroyAllWindows()

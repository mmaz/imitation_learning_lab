from __future__ import print_function
import cv2 as cv
import rospy
import numpy as np
import os
import glob
import math
import random
import datetime
from ackermann_msgs.msg import AckermannDriveStamped
from collections import deque

import tensorflow as tf
from keras.models import load_model
from keras.backend.tensorflow_backend import set_session
config = tf.ConfigProto()
config.gpu_options.per_process_gpu_memory_fraction = 0.25 
set_session(tf.Session(config=config))

SAVE_RUN = False
CENTER_CAMERA_VIDEO_ID = 3 # /dev/video*

angle_filter = deque(maxlen=5)

MODEL='model_name.h5'
model = load_model(MODEL)
model._make_predict_function() # http://projectsfromtech.blogspot.com/2017/10/visual-object-recognition-in-ros-using.html
graph = tf.get_default_graph()

DRIVE_TOPIC = "/vesc/low_level/ackermann_cmd_mux/input/navigation"

def pilotnet_crop(image):
    """assumes 320x240 input, resizes to 200x66"""
    # rows:    (240 - 66) / 2 == 87
    # columns: (320 - 200) /2 == 60 
    return image[87:-87, 60:-60] 

if __name__ == "__main__":
    rospy.init_node('PilotNetNode', anonymous=True)
    commandPub = rospy.Publisher( DRIVE_TOPIC, AckermannDriveStamped, queue_size=1)

    cap = cv.VideoCapture(CENTER_CAMERA_VIDEO_ID)
    #cam.set(cv.CAP_PROP_FPS, 15)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 240)
    
    if SAVE_RUN:
        start = datetime.datetime.now()
        data_folder = "run_{}".format(start.strftime("%m_%d_%H_%M"))
        os.mkdir(data_folder)
        os.chdir(data_folder)

        csv_fn = "{}.csv".format(data_folder)

    while True:
        ret, image = cap.read()

        crop = pilotnet_crop(image)
        crop = np.array([crop])
        with graph.as_default():
            ngl = model.predict(crop, batch_size=1)[0,0]

        angle_filter.append(ngl)
        # TODO: try a median filter, removing the filter, etc
        ngl = np.mean(angle_filter)
        print(ngl)
        command = AckermannDriveStamped()
        command.drive.steering_angle = ngl
        command.drive.speed = 4000
        commandPub.publish(command)
        if SAVE_RUN:
            now = datetime.datetime.now()
            img_fn = "{}.jpg".format(now.strftime('%m_%d_%H_%M_%S_%f')[:-3])
            #cv.imwrite(img_fn, frames, [int(cv.IMWRITE_PNG_COMPRESSION), 5])
            cv.imwrite(img_fn, image, [int(cv.IMWRITE_JPEG_QUALITY), 85])
            with open(csv_fn, 'a') as fh:
              line = "{},{}\n".format(img_fn, ngl)
              fh.write(line)
        if rospy.is_shutdown():
            cap.release()
            cv.destroyAllWindows()

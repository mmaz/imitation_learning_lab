import cv2 as cv
import numpy as np
import os
import glob
import math
import random
import zmq
import datetime
from collections import deque

import tensorflow as tf
from tensorflow.keras.models import load_model
config = tf.ConfigProto()
config.gpu_options.per_process_gpu_memory_fraction = 0.25 
sess = tf.InteractiveSession(config=config)

SAVE_RUN = False
CENTER_CAMERA_VIDEO_ID = 1 # /dev/video*

angle_filter = deque(maxlen=5)

MODEL='model_name.h5'
model = load_model(MODEL)
model._make_predict_function() # http://projectsfromtech.blogspot.com/2017/10/visual-object-recognition-in-ros-using.html
graph = tf.get_default_graph()

def pilotnet_crop(image):
    """assumes 320x240 input, resizes to 200x66"""
    # rows:    (240 - 66) / 2 == 87
    # columns: (320 - 200) /2 == 60 
    return image[87:-87, 60:-60] 

if __name__ == "__main__":
    port = "5556"
    context = zmq.Context()
    socket = context.socket(zmq.PUB)  # zeromq publisher
    socket.setsockopt(zmq.SNDHWM, 1)  #  "send highwatermark" - do not queue up messages
    socket.bind("tcp://*:%s" % port)  

    cap = cv.VideoCapture(CENTER_CAMERA_VIDEO_ID)
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
        socket.send_string("{}".format(ngl))

        if SAVE_RUN:
            now = datetime.datetime.now()
            img_fn = "{}.jpg".format(now.strftime('%m_%d_%H_%M_%S_%f')[:-3])
            #cv.imwrite(img_fn, frames, [int(cv.IMWRITE_PNG_COMPRESSION), 5])
            cv.imwrite(img_fn, image, [int(cv.IMWRITE_JPEG_QUALITY), 85])
            with open(csv_fn, 'a') as fh:
              line = "{},{}\n".format(img_fn, ngl)
              fh.write(line)


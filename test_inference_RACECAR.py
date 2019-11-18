
import cv2 as cv
import numpy as np
import os
import glob
import math
import random
import datetime

import tensorflow as tf
from tensorflow.keras.models import load_model
config = tf.ConfigProto()
config.gpu_options.per_process_gpu_memory_fraction = 0.25 
sess = tf.InteractiveSession(config=config)

#MODEL='pilotnet-RACECAR-001.h5'
#MODEL='tensorflow2.0.0.h5'
MODEL='tf_115_p3.h5'
model = load_model(MODEL)

model._make_predict_function() # http://projectsfromtech.blogspot.com/2017/10/visual-object-recognition-in-ros-using.html
graph = tf.get_default_graph()

for ix in range(103):
    # goofy wait for warmup
    if ix == 3:
        start = datetime.datetime.now()
    #random_img = np.random.rand(1,66,200,3)
    random_img = np.random.rand(120,280,3)
    random_img = np.expand_dims(random_img, 0)
    with graph.as_default():
        ngl = model.predict(random_img, batch_size=1)[0,0]
    print("model result:", ngl)
end = datetime.datetime.now()
print(end-start, "for 100 inferences")

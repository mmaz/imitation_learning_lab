from __future__ import print_function
import argparse
import base64
from datetime import datetime
import os

import numpy as np
import socketio
import eventlet
import eventlet.wsgi
from flask import Flask
import cv2 as cv

from tensorflow.keras.models import load_model

sio = socketio.Server()
app = Flask(__name__)
model = None
prev_image_array = None

MAX_SPEED = 25.
MIN_SPEED = 10.

speed_limit = MAX_SPEED

cvwindow = 'center_camera'
cv.namedWindow(cvwindow, cv.WINDOW_AUTOSIZE)

@sio.on('telemetry')
def telemetry(sid, data):
    if data:
        # The current steering angle of the car
        steering_angle = float(data["steering_angle"])
        # The current throttle of the car
        throttle = float(data["throttle"])
        # The current speed of the car
        speed = float(data["speed"])
        # The current image from the center camera of the car
        nparr = np.fromstring(base64.b64decode(data["image"]), np.uint8)
        image = cv.imdecode(nparr, cv.IMREAD_COLOR)
        cv.imshow(cvwindow, image)
        cv.waitKey(1)

        send_control(0, 8)

        # save frame
        if args.image_folder != '':
            timestamp = datetime.utcnow().strftime('%Y_%m_%d_%H_%M_%S_%f')[:-3]
            image_filename = "{}.jpg".format(os.path.join(args.image_folder, timestamp))
            cv.imwrite(image_filename, image) 
        return    
        try:
            image = utils.preprocess(image) # apply the preprocessing
            image = np.array([image])       # the model expects 4D array

            # predict the steering angle for the image
            steering_angle = float(model.predict(image, batch_size=1))
            # If the current speed is above speed_limit, we are probably driving downhill.
            #  - in this case, lower the throttle until the speed decreases
            global speed_limit
            if speed > speed_limit:
                speed_limit = MIN_SPEED  # slow down
            else:
                speed_limit = MAX_SPEED
            throttle = 1.0 - steering_angle**2 - (speed/speed_limit)**2
            # OPTIONAL TODO: change this if you implement throttle prediction

            print('steering angle: {} throttle: {} speed: {}'.format(steering_angle, throttle, speed))
            send_control(steering_angle, throttle)
        except Exception as e:
            print(e)
        
    else:
        # NOTE: DON'T EDIT THIS.
        sio.emit('manual', data={}, skip_sid=True)


@sio.on('connect')
def connect(sid, environ):
    print("connect ", sid)
    send_control(0, 0)


def send_control(steering_angle, throttle):
    sio.emit(
        "steer",
        data={
            'steering_angle': steering_angle.__str__(),
            'throttle': throttle.__str__()
        },
        skip_sid=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Remote Driving')
    parser.add_argument(
        'model',
        type=str,
        help='Path to model h5 file. Model should be on the same path.'
    )
    parser.add_argument(
        'image_folder',
        type=str,
        nargs='?',
        default='',
        help='Path to image folder. This is where the images from the run will be saved.'
    )
    args = parser.parse_args()

    model = load_model(args.model)

    if args.image_folder != '':
        if not os.path.exists(args.image_folder):
            print("Creating image folder at {}".format(args.image_folder))
            os.makedirs(args.image_folder)
        print("[Recording this run!]")
    else:
        print("[Not recording this run]")

    # wrap Flask application with engineio's middleware
    app = socketio.Middleware(sio, app)

    # deploy as an eventlet WSGI server
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app)

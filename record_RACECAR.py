from __future__ import print_function
import rospy
import cv2 as cv
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
import os
import datetime
import cameras_RACECAR as dev

ngl = None
speed = None


def joycb(msg):
    global ngl
    global speed
    ngl = msg.drive.steering_angle
    speed = msg.drive.speed


def main():
    global ngl
    global speed
    rospy.init_node("record")
    JOY_TOPIC = "/vesc/low_level/ackermann_cmd_mux/input/teleop"

    rospy.Subscriber(JOY_TOPIC, AckermannDriveStamped, joycb)

    dev.Video.notify()
    cap_l = cv.VideoCapture(dev.Video.LEFT)
    # cap_l.set(cv.CAP_PROP_FPS, 15)
    # cap_l.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    # cap_l.set(cv.CAP_PROP_FRAME_HEIGHT, 360)

    cap_r = cv.VideoCapture(dev.Video.RIGHT)
    # cap_r.set(cv.CAP_PROP_FPS, 15)
    # cap_r.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    # cap_r.set(cv.CAP_PROP_FRAME_HEIGHT, 360)

    start = datetime.datetime.now()

    data_folder = "run_{}".format(start.strftime("%m_%d_%H_%M"))
    os.mkdir(data_folder)
    os.chdir(data_folder)

    csv_fn = "{}.csv".format(data_folder)

    while True and (ngl is not None and speed is not None):
        ret_l, frame_l = cap_l.read()
        ret_r, frame_r = cap_r.read()
        frames = np.hstack([frame_l, frame_r])

        now = datetime.datetime.now()
        img_fn = "{}.jpg".format(now.strftime("%m_%d_%H_%M_%S_%f")[:-3])
        # If you prefer PNGs:
        # cv.imwrite(img_fn, frames, [int(cv.IMWRITE_PNG_COMPRESSION), 5])
        cv.imwrite(img_fn, frames, [int(cv.IMWRITE_JPEG_QUALITY), 85])
        with open(csv_fn, "a") as fh:
            fh.write("{},{},{}\n".format(img_fn, ngl, speed))
        if rospy.is_shutdown():
            cap_l.release()
            cap_r.release()
            cv.destroyAllWindows()


if __name__ == "__main__":
    main()

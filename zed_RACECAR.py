from __future__ import print_function
import rospy
import cv2 as cv
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
import os
import datetime
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


IMAGE_TOPIC = "/zed/zed_node/left/image_rect_color"

def send_array(socket, A, flags=0, copy=True, track=False):
    """send a numpy array with metadata"""
    md = dict(
        dtype = str(A.dtype),
        shape = A.shape,
    )
    socket.send_json(md, flags|zmq.SNDMORE)
    return socket.send(A, flags, copy=copy, track=track)

class Zed:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(IMAGE_TOPIC, Image, self.imcb)
        port = "5557"
        context = zmq.Context()
        self.socket = context.socket(zmq.PUB)  # zeromq publisher
        self.socket.setsockopt(zmq.SNDHWM, 1)  #  "send highwatermark" - do not queue up messages
        self.socket.bind("tcp://*:%s" % port)  


    def imcb(self, data):
        if self.angle is None or self.speed is None:
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        img = cv.resize(cv_image, (320, 240))
        send_array(self.socket, img)



def main():
    rospy.init_node("zed_zmq")
    _ = Zed()
    rospy.spin()


if __name__ == "__main__":
    main()

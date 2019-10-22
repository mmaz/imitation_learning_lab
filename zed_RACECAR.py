from __future__ import print_function
import rospy
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import zmq


IMAGE_TOPIC = "/zed/zed_node/left/image_rect_color"


class Zed:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(IMAGE_TOPIC, Image, self.imcb)
        port = "5557"
        context = zmq.Context()
        self.socket = context.socket(zmq.PUB)  # zeromq publisher
        self.socket.setsockopt(
            zmq.SNDHWM, 1
        )  #  "send highwatermark" - do not queue up messages
        self.socket.bind("tcp://*:%s" % port)

    def imcb(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        img = cv.resize(cv_image, (320, 240))
        bimg = img.tobytes()
        self.socket.send(bimg)


def main():
    rospy.init_node("zed_zmq")
    _ = Zed()
    rospy.spin()


if __name__ == "__main__":
    main()

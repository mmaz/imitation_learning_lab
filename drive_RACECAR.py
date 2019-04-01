from __future__ import print_function
import rospy
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
import zmq

DRIVE_TOPIC = "/vesc/low_level/ackermann_cmd_mux/input/navigation"

if __name__ == "__main__":
    rospy.init_node('PilotNetNode', anonymous=True)
    commandPub = rospy.Publisher( DRIVE_TOPIC, AckermannDriveStamped, queue_size=1)

    port = "5556"
    context = zmq.Context()
    socket = context.socket(zmq.PAIR)
    socket.bind("tcp://*:%s" % port)

    while True:
        msg = socket.recv()
        ngl = float(msg)
        print(ngl)
        command = AckermannDriveStamped()
        command.drive.steering_angle = ngl
        command.drive.speed = 4000
        commandPub.publish(command)

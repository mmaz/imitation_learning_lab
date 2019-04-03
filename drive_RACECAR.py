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
    socket = context.socket(zmq.SUB)      # subscriber socket
    socket.setsockopt(zmq.CONFLATE, 1)    # only receive the latest message 
    socket.setsockopt(zmq.SUBSCRIBE, '')  # no message filter
    socket.setsockopt(zmq.RCVTIMEO, 1000) # wait 1sec before raising EAGAIN
    socket.connect("tcp://127.0.0.1:%s" % port)

    while True:
        try:
            msg = socket.recv()
        except zmq.Again:
            if rospy.is_shutdown(): #catches ctrlc
                break
            print("no msg")
            continue # try to recv again
        ngl = float(msg)
        print(ngl)
        command = AckermannDriveStamped()
        command.drive.steering_angle = ngl
        command.drive.speed = 4000
        commandPub.publish(command)
        if rospy.is_shutdown():
            break
    #cleanup zmq
    socket.close()
    context.term()

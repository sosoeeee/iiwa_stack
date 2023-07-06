#!/usr/bin/env python3

import rospy
from iiwa_msgs.msg import CartesianPose

CartesianPosition = [0, 0, 0]

def callBack(msg):
    global CartesianPosition
    CartesianPosition[0] = msg.poseStamped.pose.position.x
    CartesianPosition[1] = msg.poseStamped.pose.position.y
    CartesianPosition[2] = msg.poseStamped.pose.position.z
    
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/iiwa/state/CartesianPose', CartesianPose, callBack, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    listener()
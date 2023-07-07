#!/usr/bin/env python3

import rospy
from pynput import keyboard
from iiwa_msgs.msg import CartesianPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

CartesianPosition_ini = [0, 0, 0]
currentPos = CartesianPose()
cmdPos = PoseStamped()

firstFlag = True
cmdReady = False

def cmd_callBack(msg):
    global cmdReady, cmdPos
    cmd = msg.data
    [x, y, z] = cmd.split(',')
    target_x = cmdPos.pose.position.x + float(x)
    target_y = cmdPos.pose.position.y + float(y)
    
    if abs(target_x - CartesianPosition_ini[0]) > 0.2 or abs(target_y - CartesianPosition_ini[1]) > 0.2:
        cmdReady = False
    else:
        cmdReady = True

    if cmdReady:
        # cmdPos = currentPos.poseStamped
        cmdPos.pose.position.y = target_y
        cmdPos.pose.position.x = target_x
        print("cmdPos: ", cmdPos.pose.position.x, cmdPos.pose.position.y) 
        pub.publish(cmdPos)
    
    rate.sleep()

def state_callBack(msg):
    # initalization msg
    global currentPos, cmdPos, firstFlag, CartesianPosition_ini
    if firstFlag:
        print("initialization msg")
        currentPos = msg
        CartesianPosition_ini[0] = msg.poseStamped.pose.position.x
        CartesianPosition_ini[1] = msg.poseStamped.pose.position.y
        CartesianPosition_ini[2] = msg.poseStamped.pose.position.z
        cmdPos = msg.poseStamped
        firstFlag = False
    else:
        currentPos = msg


if __name__ == '__main__':
    try:
        rospy.init_node('talker', anonymous=True)
        pub = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=10)
        rospy.Subscriber('/iiwa/state/CartesianPose', CartesianPose, state_callBack, queue_size=1)
        rospy.Subscriber('/keyboardCmd', String, cmd_callBack, queue_size=1)
        rate = rospy.Rate(0.5) # 10hz

        rospy.spin()
    except rospy.ROSInterruptException:
        pass

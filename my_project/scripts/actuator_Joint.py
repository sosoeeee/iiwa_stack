#!/usr/bin/env python3

import numpy as np
import rospy
from pynput import keyboard
from iiwa_msgs.msg import JointPosition
from iiwa_msgs.msg import JointPositionVelocity
from std_msgs.msg import String
from Jacobian import Jacobian

from utils import *

currentJointPosition = np.zeros(7)
currentJointVelocity = np.zeros(7)

firstFlag = True
cmdJointPosition = JointPosition()
controllerFreq = 20                     # Hz

def cmd_callBack(msg):
     cmd = msg.data
     [x, y, z, vx, vy, vz] = cmd.split(',')
     x = float(x)
     y = float(y)
     z = float(z)
     vx = float(vx)
     vy = float(vy)
     vz = float(vz)

     Jcob = Jacobian(currentJointPosition)
     Jcob = Jcob[:3, :]
     # 求解伪逆
     JcobInv = np.linalg.pinv(Jcob)
     # 计算关节角速度
     qdot = JcobInv.dot(np.array([vx, vy, vz]))
     # 计算关节角度
     q = currentJointPosition + qdot / controllerFreq
     print("q: ", q)
     # 发布关节角度指令
     cmdJointPosition.position.a1 = q[0]
     cmdJointPosition.position.a2 = q[1]
     cmdJointPosition.position.a3 = q[2]
     cmdJointPosition.position.a4 = q[3]
     cmdJointPosition.position.a5 = q[4]
     cmdJointPosition.position.a6 = q[5]
     cmdJointPosition.position.a7 = q[6]

     pub.publish(cmdJointPosition)

def state_callBack(msg):
     global currentJointPosition, currentJointVelocity
     currentJointPosition = toArray(msg.position)
     currentJointVelocity = toArray(msg.velocity)


if __name__ == '__main__':
    try:
        rospy.init_node('actuatorJoint', anonymous=False)
        pub = rospy.Publisher('/iiwa/command/JointPosition', JointPosition, queue_size=10)
        rospy.Subscriber('/iiwa/state/JointPositionVelocity', JointPositionVelocity, state_callBack, queue_size=1)
        rospy.Subscriber('/nextState', String, cmd_callBack, queue_size=1)
        rate = rospy.Rate(0.5) # 10hz

        rospy.spin()
    except rospy.ROSInterruptException:
        pass

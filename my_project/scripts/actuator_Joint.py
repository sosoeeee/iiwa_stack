#!/usr/bin/env python3

import numpy as np
import rospy
import time
from pynput import keyboard
from iiwa_msgs.msg import JointPosition
from iiwa_msgs.msg import JointVelocity
from iiwa_msgs.msg import JointPositionVelocity
from std_msgs.msg import String
from Jacobian import Jacobian

from utils import *

currentJointPosition = np.zeros(7)
currentJointVelocity = np.zeros(7)

firstFlag = True
cmdJointPosition = JointPosition()
cmdJointVelocity = JointVelocity()
controllerFreq = 10                     # Hz

outTxt = None
startTime = 0
endTime = 0

def cmd_callBack(msg):
    global currentJointPosition, cmdJointPosition, outTxt

    cmd = msg.data
    [x, y, z, vx, vy, vz] = cmd.split(',')
    x = float(x)
    y = float(y)
    z = float(z)
    vx = float(vx)
    vy = float(vy)
    vz = float(vz)

    Jcob = Jacobian(currentJointPosition)
    currentJointPositionVector = np.array(currentJointPosition).reshape(7, 1)
    Jcob = Jcob[:3, :]
    # 求解伪逆
    JcobInv = np.linalg.pinv(Jcob)
    # 计算关节角速度
    qdot = JcobInv.dot(np.array([vx, vy, vz]).reshape(3, 1))
    # 计算关节角度
    q = currentJointPositionVector + qdot / controllerFreq
    print("q", q)
    # # 发布关节角度指令
    cmdJointPosition.position.a1 = q[0]
    cmdJointPosition.position.a2 = q[1]
    cmdJointPosition.position.a3 = q[2]
    cmdJointPosition.position.a4 = q[3]
    cmdJointPosition.position.a5 = q[4]
    cmdJointPosition.position.a6 = q[5]
    cmdJointPosition.position.a7 = q[6]

    pub.publish(cmdJointPosition)

    # # # 发布关节角速度指令
    # cmdJointVelocity.velocity.a1 = qdot[0]
    # cmdJointVelocity.velocity.a2 = qdot[1]
    # cmdJointVelocity.velocity.a3 = qdot[2]
    # cmdJointVelocity.velocity.a4 = qdot[3]
    # cmdJointVelocity.velocity.a5 = qdot[4]
    # cmdJointVelocity.velocity.a6 = qdot[5]
    # cmdJointVelocity.velocity.a7 = qdot[6]

    # pubVelocity.publish(cmdJointVelocity)

    # Debug
    # outTemp = np.hstack((np.array(currentJointVelocity).reshape(1, 7), np.array(qdot[0]).reshape(1, 1)))
    # outTemp = np.hstack((currentJointPosition[0], q[0]))
    # outTxt = np.vstack((outTxt, outTemp)) if outTxt is not None else outTemp

# 消息频率不稳定，在500Hz — 700Hz之间
def state_callBack(msg):
    global currentJointPosition, currentJointVelocity
    global startTime, endTime
    currentJointPosition = toArray(msg.position)
    currentJointVelocity = toArray(msg.velocity)

def saveData():
    global outTxt
    np.savetxt("outData.txt", outTxt, fmt='%.4f')

if __name__ == '__main__':
    try:
        rospy.init_node('actuatorJoint', anonymous=False)
        pub = rospy.Publisher('/iiwa/command/JointPosition', JointPosition, queue_size=10)
        pubVelocity = rospy.Publisher('/iiwa/command/JointVelocity', JointVelocity, queue_size=10)
        rospy.Subscriber('/iiwa/state/JointPositionVelocity', JointPositionVelocity, state_callBack, queue_size=1)
        rospy.Subscriber('/nextState', String, cmd_callBack, queue_size=1)

        # rospy.on_shutdown(saveData)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass

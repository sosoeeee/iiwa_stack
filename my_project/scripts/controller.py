#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from iiwa_msgs.msg import CartesianPose
import numpy as np

# 初始化
firstFlag = True
initPos = PoseStamped()
currentPos = PoseStamped()
initX = -0.35
initY = -0.35
initZ = 0.13
initOrientationX = -0.3351
initOrientationY = 0.9418
initOrientationZ = -0.0253
initOrientationW = -0.0037

controllerFreq = 20         # Hz

def state_callBack(msg):
    # initalization msg
    global initPos, firstFlag, currentPos
    if firstFlag:
        print("initialization msg")
        initPos = msg.poseStamped
        firstFlag = False
    else:
        currentPos = msg.poseStamped

##########################################################################

# 指令输出
rospy.init_node('controller', anonymous=False)
pub = rospy.Publisher('referTrajectory', String, queue_size=10)
pubInit = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=10)
rospy.Subscriber('/iiwa/state/CartesianPose', CartesianPose, state_callBack, queue_size=1)
rate = rospy.Rate(controllerFreq)

# 等待初始化
while firstFlag:
    rospy.sleep(1)

# 发布初始位置
if firstFlag is False:
    print("go to initial position")
    initPos.pose.position.x = initX
    initPos.pose.position.y = initY
    initPos.pose.position.z = initZ
    initPos.pose.orientation.x = initOrientationX
    initPos.pose.orientation.y = initOrientationY
    initPos.pose.orientation.z = initOrientationZ
    while np.linalg.norm(np.array([currentPos.pose.position.x, currentPos.pose.position.y, currentPos.pose.position.z]) - np.array([initX, initY, initZ])) > 0.005:
        pubInit.publish(initPos)
        rospy.sleep(1)

print("start to publish referTrajectory")

##########################################################################

# 轨迹规划
# 二维测试轨迹——圆形
R = 0.1
targetSpeed = 0.02         # m/s
stepDis = targetSpeed/controllerFreq
stepTheta = stepDis/R
theta = np.arange(0, 2*np.pi, stepTheta)

x = R*np.cos(theta) + initX
y = R*np.sin(theta) + initY
z = np.ones(len(theta)) * initZ

# 前向差分求速度
speedMatrix = np.eye(len(theta)) * (-1)
for i in range(len(theta)):
    speedMatrix[i][(i - 1) % len(theta)] = 1
speedMatrix = speedMatrix.T

vx = speedMatrix.dot(x) * controllerFreq
vy = speedMatrix.dot(y) * controllerFreq
vz = speedMatrix.dot(z) * controllerFreq

i = 0
while not rospy.is_shutdown():
    referTrajectory = str(x[i]) + ',' + str(y[i]) + ',' + str(z[i]) + ',' + str(vx[i]) + ',' + str(vy[i]) + ',' + str(vz[i])
    pub.publish(referTrajectory)
    i = (i + 1) % len(theta)
    rate.sleep()
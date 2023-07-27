#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import PoseStamped
from iiwa_msgs.msg import CartesianPose
from iiwa_msgs.msg import JointPosition

import numpy as np
import time

from TrajectoryGenerator import *
from sharedController import *


## 负责综合共享控制器、遥操作杆、机械臂状态信息，生成轨迹信息，并通过ROS发布给执行器
class Controller:
    def __init__(self, controllerFreq):
        self.controllerFreq = controllerFreq  # Hz

        rospy.init_node('controller', anonymous=False)
        self.pub = rospy.Publisher('nextState', String, queue_size=10)
        self.pubInitPose = rospy.Publisher('/iiwa/command/JointPosition', JointPosition, queue_size=10)
        self.pubInitPosition = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=10)
        self.pubMonitor_robot = rospy.Publisher('referRobotTrajectory', Float32MultiArray, queue_size=10)
        self.pubMonitor_human = rospy.Publisher('referHumanTrajectory', Float32MultiArray, queue_size=10)
        rospy.Subscriber('/iiwa/state/CartesianPose', CartesianPose, self.cartesianState_callBack, queue_size=1)
        rospy.Subscriber("stickSignal", String, self.stickSignal_callback)
        self.rate = rospy.Rate(self.controllerFreq)

        # 定义用于控制初始位姿的变量
        self.firstFlag = True
        self.initPos = PoseStamped()

        # 定义用于反馈状态变量的变量
        self.startTime = 0
        self.endTime = 0
        self.currentPos = PoseStamped()
        self.lastPos = PoseStamped()
        self.currentVel = None
        self.currentState = np.zeros((6, 1))

        # 定义用于遥操作杆的变量
        self.stickPos = np.zeros((3, 1))

    def setInitPose(self, q1, q2, q3, q4, q5, q6, q7):
        while self.firstFlag:
            rospy.sleep(1)
        
        print("initializing pose")

        cmdJointPosition = JointPosition()
        cmdJointPosition.position.a1 = q1
        cmdJointPosition.position.a2 = q2
        cmdJointPosition.position.a3 = q3
        cmdJointPosition.position.a4 = q4
        cmdJointPosition.position.a5 = q5
        cmdJointPosition.position.a6 = q6
        cmdJointPosition.position.a7 = q7

        self.pubInitPose.publish(cmdJointPosition)
        rospy.sleep(2) # 等待机械臂到达初始位姿

    def setInitPos(self, initX, initY, initZ):
        while self.firstFlag:
            rospy.sleep(1)

        self.initPos.pose.position.x = initX
        self.initPos.pose.position.y = initY
        self.initPos.pose.position.z = initZ

        while np.linalg.norm(np.array([self.currentPos.pose.position.x, self.currentPos.pose.position.y, self.currentPos.pose.position.z]) - np.array([initX, initY, initZ])) > 0.005:
            self.pubInitPosition.publish(self.initPos)
            rospy.sleep(1)

        print("initialization of pose is done")
    
    def stickSignal_callback(self, msg):
        point = msg.data.split(',')
        self.stickPos[0, 0] = float(point[0])
        self.stickPos[1, 0] = float(point[1])
        self.stickPos[2, 0] = float(point[2])

    # 笛卡尔空间状态回调函数
    def cartesianState_callBack(self, msg):
        if self.firstFlag:
            print("initialization msg")
            self.initPos = msg.poseStamped
            self.currentPos = msg.poseStamped
            self.startTime = time.time()
            self.firstFlag = False
        else:
            self.endTime = time.time()
            deltaT = self.endTime - self.startTime
            self.lastPos = self.currentPos
            self.currentPos = msg.poseStamped
            self.currentVel = (np.array([self.currentPos.pose.position.x, self.currentPos.pose.position.y, self.currentPos.pose.position.z]) - np.array([self.lastPos.pose.position.x, self.lastPos.pose.position.y, self.lastPos.pose.position.z])) / deltaT
            if abs(self.currentVel[0]) < 0.0001:
                self.currentVel[0] = 0
            if abs(self.currentVel[1]) < 0.0001:
                self.currentVel[1] = 0
            if abs(self.currentVel[2]) < 0.0001:
                self.currentVel[2] = 0
            self.currentState = np.array([self.currentPos.pose.position.x, self.currentPos.pose.position.y, self.currentPos.pose.position.z, self.currentVel[0], self.currentVel[1], self.currentVel[2]]).reshape((6, 1))

            # print("current state is: ", self.currentState)

    def publishState(self, nextState):
        self.pub.publish(nextState)

    def publishRobotTrajectory(self, Trajectory):
        print("publish robot trajectory")
        # 只取位置信息
        Trajectory = Trajectory[:3, :]
        referTrajectory = Float32MultiArray()
        referTrajectory.data = Trajectory.flatten()

        # 机器人轨迹发布后，等待一段时间，确保可以绘制出轨迹
        rospy.sleep(1)
        self.pubMonitor_robot.publish(referTrajectory)
    
    def publishHumanTrajectory(self, Trajectory):
        print("publish human trajectory")
        Trajectory = Trajectory[:3, :]
        referTrajectory = Float32MultiArray()
        referTrajectory.data = Trajectory.flatten()

        # 机器人轨迹发布后，等待一段时间，确保可以绘制出轨迹
        rospy.sleep(1)
        self.pubMonitor_human.publish(referTrajectory)


    def getCurrentState(self):
        return self.currentState
    
    def getStickPos(self):
        return self.stickPos
    

## 控制器初始化
##########################################################################
controllerFreq = 10
initX = -0.35
initY = -0.5
initZ = 0.2

# iniQ1 = 0.891
# iniQ2 = -0.733
# iniQ3 = 0.163
# iniQ4 = 1.984
# iniQ5 = -0.232
# iniQ6 = -0.438
# iniQ7 = 2.435

controller = Controller(controllerFreq)

##########################################################################

# 轨迹规划
# 二维测试轨迹——圆形
R = 0.08
avrSpeed = 0.02
circleTrajectory = getCircle(R, avrSpeed, controllerFreq, initX, initY, initZ)
x = circleTrajectory[0, :]
y = circleTrajectory[1, :]
z = circleTrajectory[2, :]
vx = circleTrajectory[3, :]
vy = circleTrajectory[4, :]
vz = circleTrajectory[5, :]
controller.publishRobotTrajectory(circleTrajectory)

# 修改生成人类轨迹的参数
circleTrajectoryHuman = getCircleHuman(R, avrSpeed, controllerFreq, initX, initY, initZ)
controller.publishHumanTrajectory(circleTrajectoryHuman)

# lineTrajectory = getLine(0.1, Speed, controllerFreq, initX, initY, initZ, 'x')
# x = lineTrajectory[0, :]
# y = lineTrajectory[1, :]
# z = lineTrajectory[2, :]
# vx = lineTrajectory[3, :]
# vy = lineTrajectory[4, :]
# vz = lineTrajectory[5, :]
# controller.publishRobotTrajectory(lineTrajectory)
# print("lineTrajectory: ", lineTrajectory.shape)

# 初始化机器人位姿
# controller.setInitPos(initX + R, initY, initZ, initOrientationX, initOrientationY, initOrientationZ, initOrientationW)
# controller.setInitPose(iniQ1, iniQ2, iniQ3, iniQ4, iniQ5, iniQ6, iniQ7)
controller.setInitPos(initX + R, initY, initZ)

# 引入共享控制器
sharedcontroller = sharedController(controllerFreq)
sharedcontroller.initialize(1, 0.1, 10000)
sharedcontroller.setRobotGlobalTraj(circleTrajectory)

i = 0
while not rospy.is_shutdown():    
    # 共享控制
    w = controller.getCurrentState()
    sharedcontroller.updateState(w)
    endEffectorPos = w[0:3, 0]
    stickPos = controller.getStickPos()
    sharedcontroller.gethumanLocalTraj(stickPos, endEffectorPos)
    humanIntent = sharedcontroller.getHumanIntent()

    # if humanIntent == 2:
        # changedTrajectory = sharedcontroller.changeGlobalTraj(i, obstacles, avrSpeed)
        # controller.publishRobotTrajectory(changedTrajectory)

    w_next = sharedcontroller.computeLocalTraj(i)
    # print("humanIntent: ", humanIntent)

    TrajectoryString = str(w_next[0, 0]) + ',' + str(w_next[1, 0]) + ',' + str(w_next[2, 0]) + ',' + str(w_next[3, 0]) + ',' + str(w_next[4, 0]) + ',' + str(w_next[5, 0])

    controller.publishState(TrajectoryString)
    # print("TrajectoryString: ", TrajectoryString)

    i = (i + 1) % len(x)

    # 开环控制
    # TrajectoryString = str(x[i]) + ',' + str(y[i]) + ',' + str(z[i]) + ',' + str(vx[i]) + ',' + str(vy[i]) + ',' + str(vz[i])
    # controller.publishState(TrajectoryString)
    # i = i + 1
    # if i == len(x):
    #     break


    controller.rate.sleep()
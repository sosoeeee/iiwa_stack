#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import PoseStamped
from iiwa_msgs.msg import CartesianPose
from iiwa_msgs.msg import JointPositionVelocity

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
        self.pubInit = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=10)
        self.pubMonitor_robot = rospy.Publisher('referRobotTrajectory', Float32MultiArray, queue_size=10)
        self.pubMonitor_human = rospy.Publisher('referHumanTrajectory', Float32MultiArray, queue_size=10)
        rospy.Subscriber('/iiwa/state/CartesianPose', CartesianPose, self.cartesianState_callBack, queue_size=1)
        rospy.Subscriber("controllerSignal", String, self.stickSignal_callback)
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

    def setInitPos(self, initX, initY, initZ, initOrientationX, initOrientationY, initOrientationZ, initOrientationW):
        while self.firstFlag:
            rospy.sleep(1)
        
        print("initializing pose")

        self.initPos.pose.position.x = initX
        self.initPos.pose.position.y = initY
        self.initPos.pose.position.z = initZ
        self.initPos.pose.orientation.x = initOrientationX
        self.initPos.pose.orientation.y = initOrientationY
        self.initPos.pose.orientation.z = initOrientationZ

        while np.linalg.norm(np.array([self.currentPos.pose.position.x, self.currentPos.pose.position.y, self.currentPos.pose.position.z]) - np.array([initX, initY, initZ])) > 0.005:
            self.pubInit.publish(self.initPos)
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
controllerFreq = 20
initX = -0.35
initY = -0.35
initZ = 0.13
initOrientationX = -0.3351
initOrientationY = 0.9418
initOrientationZ = -0.0253
initOrientationW = -0.0037

controller = Controller(controllerFreq)
# controller.setInitPos(initX + R, initY, initZ, initOrientationX, initOrientationY, initOrientationZ, initOrientationW)

##########################################################################

# 轨迹规划
# 二维测试轨迹——圆形
R = 0.1
circleTrajectory = getCircle(R, 0.02, 20, initX, initY, initZ)
x = circleTrajectory[0, :]
y = circleTrajectory[1, :]
z = circleTrajectory[2, :]
vx = circleTrajectory[3, :]
vy = circleTrajectory[4, :]
vz = circleTrajectory[5, :]
controller.publishRobotTrajectory(circleTrajectory)

# 修改生成人类轨迹的参数
circleTrajectoryHuman = getCircleHuman(0.1, 0.02, 20, initX, initY, initZ)
controller.publishHumanTrajectory(circleTrajectoryHuman)

# 初始化机器人位姿
controller.setInitPos(initX + R, initY, initZ, initOrientationX, initOrientationY, initOrientationZ, initOrientationW)

# 引入共享控制器
sharedcontroller = sharedController(controllerFreq)
sharedcontroller.initialize(1, 0.1, 10000)
sharedcontroller.setRobotGlobalTraj(circleTrajectory)

i = 0
while not rospy.is_shutdown():
    TrajectoryString = str(x[i]) + ',' + str(y[i]) + ',' + str(z[i]) + ',' + str(vx[i]) + ',' + str(vy[i]) + ',' + str(vz[i])
    
    # # 共享控制
    # w = controller.getCurrentState()
    # sharedcontroller.updateState(w)
    # endEffectorPos = w[0:3, 0]
    # stickPos = controller.getStickPos()
    # sharedcontroller.gethumanLocalTraj(stickPos, endEffectorPos)
    # humanIntent = sharedcontroller.getHumanIntent()

    # # if humanIntent == 2:
    # #     changedTrajectory = sharedcontroller.changeGlobalTraj()
    # #     controller.publishRobotTrajectory(changedTrajectory)
    # # else:
    # #     w_next = sharedcontroller.computeLocalTraj(i)   

    # w_next = sharedcontroller.computeLocalTraj(i)   

    # TrajectoryString = str(w_next[0]) + ',' + str(w_next[1]) + ',' + str(w_next[2]) + ',' + str(w_next[3]) + ',' + str(w_next[4]) + ',' + str(w_next[5])

    controller.publishState(TrajectoryString)
    i = (i + 1) % len(x)
    controller.rate.sleep()
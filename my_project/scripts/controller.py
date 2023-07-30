#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
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
        self.pubObstacle = rospy.Publisher('Obstacles', String, queue_size=10)

        rospy.Subscriber('/iiwa/state/CartesianPose', CartesianPose, self.cartesianState_callBack, queue_size=1)
        rospy.Subscriber("stickSignal", String, self.stickSignal_callback)
        # vicon obstacles
        try:
            rospy.Subscriber('/vicon/Obstacle_1/Obstacle_1', TransformStamped, self.obstacle_1_callBack, queue_size=1)
        except:
            raise Exception("can't find topic: /vicon/Obstacle_1/Obstacle_1")

        self.rate = rospy.Rate(self.controllerFreq)

        # 定义用于控制初始位姿的变量
        self.firstFlag = True
        self.initPos = PoseStamped()

        # 定义用于反馈状态变量的变量np.array([0.4, 0.4, 0]).reshape((3, 1)), 0.3
        self.startTime = 0
        self.endTime = 0
        self.currentPos = PoseStamped()
        self.lastPos = PoseStamped()
        self.currentVel = None
        self.currentState = np.zeros((6, 1))

        # 定义用于遥操作杆的变量
        self.stickPos = np.zeros((3, 1))
        self.stickForce = np.zeros((3, 1))

        # 障碍物变量
        self.ObstacleSet = []
        # 将障碍物从VICON坐标系转换到机器人坐标系
        self.cooridinateTransformX =  2.2289 + 0.1762
        self.cooridinateTransformY = -1.5596 + 0.4873
        self.cooridinateTransformZ =  0.6921 - 0.2822

    # 根据订阅的VICON个数初始化障碍物个数
    def initObstacleSet(self, num):
        for i in range(num):
            center = np.array([0, 0, 0]).reshape((3, 1))
            radius = 0.1

            # obstacle['center'], obstacle['radius']
            obstacle = {
                'center': center,
                'radius': radius,
                'state': 0              # 0: 创建 1: 实时
            }

            self.ObstacleSet.append(obstacle)

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
        posAndForce = msg.data.split(',')
        self.stickPos[0, 0] = float(posAndForce[0])
        self.stickPos[1, 0] = float(posAndForce[1])
        self.stickPos[2, 0] = float(posAndForce[2])

        self.stickForce[0, 0] = float(posAndForce[3])
        self.stickForce[1, 0] = float(posAndForce[4])
        self.stickForce[2, 0] = float(posAndForce[5])

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

    # 设置障碍物1的状态更新
    def obstacle_1_callBack(self, msg):
        x = msg.transform.translation.x
        y = msg.transform.translation.y
        z = msg.transform.translation.z

        # 转换到机器人坐标系下
        x = x - self.cooridinateTransformX
        y = y - self.cooridinateTransformY
        z = z - self.cooridinateTransformZ

        center = np.array([x, y, z]).reshape((3, 1))
        radius = 0.03

        centerLast = self.ObstacleSet[0]['center']

        if np.linalg.norm(center - centerLast) > 0.01:
            self.ObstacleSet[0]['center'] = center
            self.ObstacleSet[0]['radius'] = radius
            self.ObstacleSet[0]['state'] = 1

            self.publishObstacle(0)

    # 更新障碍物index在monitor中的显示的位置
    def publishObstacle(self, index):
        print("publish obstacles")
        x = self.ObstacleSet[index]['center'][0, 0]
        y = self.ObstacleSet[index]['center'][1, 0]
        z = self.ObstacleSet[index]['center'][2, 0]
        radius = self.ObstacleSet[index]['radius']

        msg = str(x) + ',' + str(y) + ',' + str(z) + ',' + str(radius)
        self.pubObstacle.publish(msg)
        
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
    
    def getObstacles(self):
        return self.ObstacleSet

    def getCurrentState(self):
        return self.currentState
    
    def getStickPos(self):
        return self.stickPos

    def getStickForce(self):
        return self.stickForce
    

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
controller.initObstacleSet(1)

##########################################################################

# 轨迹规划
# 二维测试轨迹——圆形
R = 0.08
avrSpeed = 0.01
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
    # 获取其他模块的信息
    w = controller.getCurrentState()
    stickPos = controller.getStickPos()
    stickForce = controller.getStickForce()
    obstacles = controller.getObstacles()

    # 更新共享控制器内部阻抗模型的状态变量
    sharedcontroller.updateState(w)

    sharedcontroller.computeLambda(obstacles, i)
    sharedcontroller.gethumanLocalTraj(stickPos, stickForce)
    humanIntent = sharedcontroller.getHumanIntent()

    # 可以考虑减小轨迹重规划的频率
    if humanIntent == 2:
        print("wait for trajectory replanning")
        startTime = time.time()

        changedTrajectory = sharedcontroller.changeGlobalTraj(i, stickForce, obstacles, avrSpeed)
        controller.publishRobotTrajectory(changedTrajectory)

        endTime = time.time()
        print("trajectory replanning is done, time cost: ", endTime - startTime)

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
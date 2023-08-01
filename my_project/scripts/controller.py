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
from RRT_PathPlanner import PathPlanner


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
            rospy.Subscriber('/vicon/Obstacle_2/Obstacle_2', TransformStamped, self.obstacle_2_callBack, queue_size=1)
            rospy.Subscriber('/vicon/Obstacle_3/Obstacle_3', TransformStamped, self.obstacle_3_callBack, queue_size=1)
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
        self.cooridinateTransformZ =  0.6921 - 0.2822 + 0.1
        # 运动空间(运动空间为矩形，由两个顶点确定)
        self.pos1 = None
        self.pos2 = None

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
    
    def setMoveSpace(self, pos1, pos2):
        self.pos1 = pos1
        self.pos2 = pos2
    
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
            print("Get CartesianState_callBack")
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
        radius = 0.07

        centerLast = self.ObstacleSet[0]['center']

        if np.linalg.norm(center - centerLast) > 0.01:
            self.ObstacleSet[0]['center'] = center
            self.ObstacleSet[0]['radius'] = radius
            self.ObstacleSet[0]['state'] = 1

            self.publishObstacle(0)
    
    # 设置障碍物2的状态更新
    def obstacle_2_callBack(self, msg):
        x = msg.transform.translation.x
        y = msg.transform.translation.y
        z = msg.transform.translation.z

        # 转换到机器人坐标系下
        x = x - self.cooridinateTransformX
        y = y - self.cooridinateTransformY
        z = z - self.cooridinateTransformZ

        center = np.array([x, y, z]).reshape((3, 1))
        radius = 0.03

        centerLast = self.ObstacleSet[1]['center']

        if np.linalg.norm(center - centerLast) > 0.01:
            self.ObstacleSet[1]['center'] = center
            self.ObstacleSet[1]['radius'] = radius
            self.ObstacleSet[1]['state'] = 1

            self.publishObstacle(1)

    # 设置障碍物3的状态更新
    def obstacle_3_callBack(self, msg):
        x = msg.transform.translation.x
        y = msg.transform.translation.y
        z = msg.transform.translation.z

        # 转换到机器人坐标系下
        x = x - self.cooridinateTransformX
        y = y - self.cooridinateTransformY
        z = z - self.cooridinateTransformZ

        center = np.array([x, y, z]).reshape((3, 1))
        radius = 0.12

        centerLast = self.ObstacleSet[2]['center']

        if np.linalg.norm(center - centerLast) > 0.01:
            self.ObstacleSet[2]['center'] = center
            self.ObstacleSet[2]['radius'] = radius
            self.ObstacleSet[2]['state'] = 1

            self.publishObstacle(2)

    # 更新障碍物index在monitor中的显示的位置
    def publishObstacle(self, index):
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
    
    def getAllObstaclesPoints(self):
        # 生成点集的步长
        distanceStep = 0.01
        # 获取长方体运动空间表面的点
        if self.pos1 is None or self.pos2 is None:
            raise Exception("move space is not set")
            
        cuboidPoints = getCuboidPoints(self.pos1, self.pos2, distanceStep * 3)
        obstaclePoints = cuboidPoints
        # obstaclePoints = None

        # 将Obstacles转换为球面障碍物点集
        for obstacle in self.ObstacleSet:
            if obstacle['state'] == 1:
                obstaclePos = obstacle['center']
                obstacleRadius = obstacle['radius']
                
                if obstaclePoints is None:
                    obstaclePoints = getSphericalPoints(obstaclePos, obstacleRadius, distanceStep)
                else:
                    obstaclePoints = np.hstack((obstaclePoints, getSphericalPoints(obstaclePos, obstacleRadius, distanceStep)))
        
        # DEBUG
        # import matplotlib.pyplot as plt
        # fig = plt.figure()
        # ax = fig.add_subplot(111, projection='3d')
        # ax.set_xlabel('x')
        # ax.set_ylabel('y')
        # ax.set_zlabel('z')
        # ax.scatter(obstaclePoints[0, :], obstaclePoints[1, :], obstaclePoints[2, :], c='r', marker='.')
        # plt.show()

        return obstaclePoints

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
initX = -0.46015
initY = 0.11484
initZ = 0.20458

# 绝对运动边界
# Xrange = [-0.94, -0.36]
# Yrange = [-1, 0.2]
# Zrange = [-1, 1]

Xrange = [-0.26, 0.1]
Yrange = [-1, 0]
Zrange = [-1, 1]

iniQ1 = -1.0912
iniQ2 = -1.0999
iniQ3 = 1.16505
iniQ4 = 1.8945
iniQ5 = -1.2382
iniQ6 = -1.0227
iniQ7 = -0.2546

controller = Controller(controllerFreq)
controller.initObstacleSet(3)

##########################################################################

# 初始化机器人位姿
# controller.setInitPos(initX + R, initY, initZ, initOrientationX, initOrientationY, initOrientationZ, initOrientationW)
controller.setInitPose(iniQ1, iniQ2, iniQ3, iniQ4, iniQ5, iniQ6, iniQ7)
controller.setInitPos(initX, initY, initZ)

##########################################################################

# 轨迹规划
# 二维测试轨迹——圆形
# R = 0.08
# avrSpeed = 0.01
# circleTrajectory = getCircle(R, avrSpeed, controllerFreq, initX, initY, initZ)
# x = circleTrajectory[0, :]
# y = circleTrajectory[1, :]
# z = circleTrajectory[2, :]
# vx = circleTrajectory[3, :]
# vy = circleTrajectory[4, :]
# vz = circleTrajectory[5, :]
# controller.publishRobotTrajectory(circleTrajectory)

# # 修改生成人类轨迹的参数
# circleTrajectoryHuman = getCircleHuman(R, avrSpeed, controllerFreq, initX, initY, initZ)
# controller.publishHumanTrajectory(circleTrajectoryHuman)

# 二维测试轨迹——直线
# lineTrajectory = getLine(0.1, Speed, controllerFreq, initX, initY, initZ, 'x')
# x = lineTrajectory[0, :]
# y = lineTrajectory[1, :]
# z = lineTrajectory[2, :]
# vx = lineTrajectory[3, :]
# vy = lineTrajectory[4, :]
# vz = lineTrajectory[5, :]
# controller.publishRobotTrajectory(lineTrajectory)
# print("lineTrajectory: ", lineTrajectory.shape)

# RRT规划
avrSpeed = 0.01
startPoint = np.array([initX, initY, initZ]).reshape((3, 1))
endPoint = np.array([initX + 0.06, initY - 0.6, initZ]).reshape((3, 1))
pathPLanner = PathPlanner(startPoint, endPoint, np.array([Xrange, Yrange, Zrange]))

# 读取环境信息
obstacles = controller.getObstacles()
while True:
    state = 1
    for obstacle in obstacles:
        state = state * obstacle['state']
    if state == 1:
        print("all obstacles are ready !")
        break
    else:
        print("wait for obstacles...")
        rospy.sleep(1)
    obstacles = controller.getObstacles()
i = 0
for obstacle in obstacles:
    pathPLanner.addObstacle(obstacle['center'], obstacle['radius'])
    controller.publishObstacle(i)
    i = i + 1
    print("publish obstacle %d" % i)
    rospy.sleep(0.5)

print("start RRT path planning")
path = pathPLanner.RRT(True)

# 绘制路径规划的结果
print(path.shape)
controller.publishRobotTrajectory(path)
rospy.sleep(3)

print("RRT path planning is done")

# 轨迹规划
print("start trajectory planning")
v0 = np.array([0, 0, 0]).reshape((3, 1))
a0 = np.array([0, 0, 0]).reshape((3, 1))
vt = np.array([0, 0, 0]).reshape((3, 1))
at = np.array([0, 0, 0]).reshape((3, 1))
miniJerkTrajPlanner = MinimumTrajPlanner(path, avrSpeed, controllerFreq, v0, a0, vt, at, 3)
traj = miniJerkTrajPlanner.computeTraj()
controller.publishRobotTrajectory(traj)
print("trajectory planning is done")

##########################################################################

# 设置机器人运动边界
pos1 = np.array([initX + Xrange[0], initY + Yrange[0], initZ + Zrange[0]]).reshape((3, 1))
pos2 = np.array([initX + Xrange[1], initY + Yrange[1], initZ + Zrange[1]]).reshape((3, 1))
controller.setMoveSpace(pos1, pos2)

# 引入共享控制器
sharedcontroller = sharedController(controllerFreq)
sharedcontroller.initialize(1, 10, 10000)
sharedcontroller.setRobotGlobalTraj(traj)

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

    sharedcontroller.computeLambda(controller.getAllObstaclesPoints(), i) # 计算SPI
    sharedcontroller.gethumanLocalTraj(stickPos, stickForce)
    humanIntent = sharedcontroller.getHumanIntent()

    # 可以考虑减小轨迹重规划的频率, 慢10倍
    if i % 10 == 0:
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

    # i = (i + 1) % len(traj.shape[1])
    i = i + 1
    if i == traj.shape[1]:
        break

    # 开环控制
    # TrajectoryString = str(x[i]) + ',' + str(y[i]) + ',' + str(z[i]) + ',' + str(vx[i]) + ',' + str(vy[i]) + ',' + str(vz[i])
    # controller.publishState(TrajectoryString)
    
    controller.rate.sleep()
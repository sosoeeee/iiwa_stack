#!/usr/bin/env python3

import rospy

import numpy as np
import time

from TrajectoryGenerator import *
from sharedController import *
from RRT_PathPlanner import PathPlanner

from controller import *
    
usr = 7
times = 2
mode = ["replan", "allHuman"]
curMode = mode[0]

print("--------------START--------------")
print("USR is %d" % usr)
print("System Mode is %s" % curMode)

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

# # 绘制路径规划的结果
# print(path.shape)
# controller.publishRobotTrajectory(path)
# rospy.sleep(3)

# print("RRT path planning is done")

# # 轨迹规划
# print("start trajectory planning")
# v0 = np.array([0, 0, 0]).reshape((3, 1))
# a0 = np.array([0, 0, 0]).reshape((3, 1))
# vt = np.array([0, 0, 0]).reshape((3, 1))
# at = np.array([0, 0, 0]).reshape((3, 1))
# miniJerkTrajPlanner = MinimumTrajPlanner(path, avrSpeed, controllerFreq, v0, a0, vt, at, 3)
# traj = miniJerkTrajPlanner.computeTraj()
# np.savetxt("oriTraj.txt", traj)
# controller.publishRobotTrajectory(traj)
# print("trajectory planning is done")

traj = np.loadtxt('oriTraj.txt')
controller.publishRobotTrajectory(traj)

# 添加人类兴趣点
pointSet = np.array([-0.57, -0.005, 0.2]).reshape(3, 1)
# pointSet = np.array([-0.6, -0.18, 0.2]).reshape(3, 1)
# pointSet = np.hstack((pointSet, np.array([-0.55, 0.07, 0.2]).reshape(3, 1)))
# pointSet = np.hstack((pointSet, np.array([-0.56, 0.04, 0.2]).reshape(3, 1)))
# pointSet = np.hstack((pointSet, np.array([-0.59, -0.25, 0.2]).reshape(3, 1)))
controller.publishHumanTrajectory(pointSet)

##########################################################################

# 设置机器人运动边界
pos1 = np.array([initX + Xrange[0], initY + Yrange[0], initZ + Zrange[0]]).reshape((3, 1))
pos2 = np.array([initX + Xrange[1], initY + Yrange[1], initZ + Zrange[1]]).reshape((3, 1))
controller.setMoveSpace(pos1, pos2)

# 引入共享控制器
replanLen = 250
localLen = 50
sharedcontroller = sharedController(controllerFreq, localLen, replanLen)
sharedcontroller.initialize(1, 10, 10000)
sharedcontroller.setRobotGlobalTraj(traj)

##########################################################################

# 记录数据(第一行数据为0，无意义)
forceSet = np.array([0, 0]).reshape((1, 2))
distanceSet = np.array([0, 0]).reshape((1, 2))
realTraj = np.array([0, 0, 0]).reshape((1, 3))
PSISet = [0]

i = 0
totalLen = traj.shape[1]
changeTimes = 0
while not rospy.is_shutdown():    
    # 共享控制
    # 获取其他模块的信息
    w = controller.getCurrentState()
    stickPos = controller.getStickPos()
    stickForce = controller.getStickForce()
    obstacles = controller.getObstacles()

    # 更新共享控制器内部阻抗模型的状态变量
    sharedcontroller.updateState(w)

    sharedcontroller.gethumanLocalTraj(stickPos, stickForce)
    humanIntent = sharedcontroller.getHumanIntent()

    # 模式切换
    if curMode == 'replan':
        PSI = sharedcontroller.computeLambda(controller.getAllObstaclesPoints(), i) # 计算SPI

        # 可以考虑减小轨迹重规划的频率, 慢10倍
        if totalLen - i > replanLen  and i % 10 == 0:
            if humanIntent == 2:
                print("wait for trajectory replanning")
                changeTimes = changeTimes + 1
                startTime = time.time()

                changedTrajectory = sharedcontroller.changeGlobalTraj(i, stickForce, obstacles, avrSpeed)
                controller.publishRobotTrajectory(changedTrajectory)

                np.savetxt("Data/%d-%s-CHANGETraj-%d-change-%d.txt" % (usr, curMode, times, changeTimes), changedTrajectory)

                endTime = time.time()
                print("trajectory replanning is done, time cost: ", endTime - startTime)

    w_next = sharedcontroller.computeLocalTraj(i)
    # print("humanIntent: ", humanIntent)

    TrajectoryString = str(w_next[0, 0]) + ',' + str(w_next[1, 0]) + ',' + str(w_next[2, 0]) + ',' + str(w_next[3, 0]) + ',' + str(w_next[4, 0]) + ',' + str(w_next[5, 0])

    controller.publishState(TrajectoryString)
    controller.publishIndex(i, replanLen)
    # print("TrajectoryString: ", TrajectoryString)

    # i = (i + 1) % len(traj.shape[1])
    i = i + 1
    if i == traj.shape[1] - localLen:
        break

    # 开环控制
    # TrajectoryString = str(x[i]) + ',' + str(y[i]) + ',' + str(z[i]) + ',' + str(vx[i]) + ',' + str(vy[i]) + ',' + str(vz[i])
    # controller.publishState(TrajectoryString)

    # 数据收集
    forceSet = np.vstack((forceSet, np.array([stickForce[0], stickForce[1]]).reshape((1, 2))))
    distanceSet = np.vstack((distanceSet, np.array([stickPos[0], stickPos[1]]).reshape((1, 2))))
    if curMode == 'replan':
        PSISet.append(PSI)
    realTraj = np.vstack((realTraj, w[0:3, 0].reshape((1, 3))))
    
    controller.rate.sleep()

# 保存数据
# forceSet = np.array(forceSet).reshape((-1, 1))
PSISet = np.array(PSISet).reshape((-1, 1))
np.savetxt("Data/%d-%s-forceSet-%d.txt" % (usr, curMode, times), forceSet, fmt='%.4f')
np.savetxt("Data/%d-%s-distanceSet-%d.txt" % (usr, curMode, times), distanceSet, fmt='%.4f')
np.savetxt("Data/%d-%s-realTraj-%d.txt" % (usr, curMode, times), realTraj)
if curMode == 'replan':
    np.savetxt("Data/%d-%s-PSISet-%d.txt" % (usr, curMode, times), PSISet, fmt='%.4f')
print("--------------END--------------")
print("SAVE DATA")
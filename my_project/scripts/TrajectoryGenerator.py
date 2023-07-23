#!/usr/bin/env python3

import numpy as np

# @brief 生成一条xy平面的圆形轨迹
# @param R 圆形轨迹半径
# @param targetSpeed 目标速度（m/s）（推荐 0.02）
# @param controllerFreq 控制频率
# @param initX 初始位置x
# @param initY 初始位置y
# @param initZ 初始位置z
# @return 6*len(theta)的矩阵，第一行为x，第二行为y，第三行为z，第四行为vx，第五行为vy，第六行为vz
def getCircle(R, targetSpeed, controllerFreq, initX, initY, initZ):
    # 二维测试轨迹——圆形
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

    # 合并为一个6*len(theta)的矩阵
    trajectory = np.vstack((x, y, z, vx, vy, vz))

    return trajectory


def getCircleHuman(R, targetSpeed, controllerFreq, initX, initY, initZ):
    # 二维测试轨迹——圆形
    stepDis = targetSpeed/controllerFreq
    stepTheta = stepDis/R
    theta = np.arange(0, 2*np.pi, stepTheta)

    xInit = R*np.cos(theta) + initX
    yInit = R*np.sin(theta) + initY

    changeIndex1 = int(len(theta) / 3)
    changeIndex2 = int(len(theta) / 3 * 2)

    midX = (xInit[changeIndex1] + xInit[changeIndex2]) / 2 - R/2
    midY = (yInit[changeIndex1] + yInit[changeIndex2]) / 2 

    DistanceToMid = ((xInit[changeIndex1] - midX)**2 + (yInit[changeIndex1] - midY)**2)**0.5
    num = int(DistanceToMid/stepDis) # 直线段的点数

    Xmid1 = np.linspace(xInit[changeIndex1], midX, num)
    Ymid1 = np.linspace(yInit[changeIndex1], midY, num)
    Xmid2 = np.linspace(midX, xInit[changeIndex2], num)
    Ymid2 = np.linspace(midY, yInit[changeIndex2], num)

    x = np.hstack((xInit[:changeIndex1], Xmid1, Xmid2, xInit[changeIndex2:]))
    y = np.hstack((yInit[:changeIndex1], Ymid1, Ymid2, yInit[changeIndex2:]))
    z = np.ones(len(x)) * initZ

    # 前向差分求速度
    speedMatrix = np.eye(len(z)) * (-1)
    for i in range(len(z)):
        speedMatrix[i][(i - 1) % len(z)] = 1
    speedMatrix = speedMatrix.T

    vx = speedMatrix.dot(x) * controllerFreq
    vy = speedMatrix.dot(y) * controllerFreq
    vz = speedMatrix.dot(z) * controllerFreq

    # 合并为一个6*len(theta)的矩阵
    trajectory = np.vstack((x, y, z, vx, vy, vz))

    return trajectory

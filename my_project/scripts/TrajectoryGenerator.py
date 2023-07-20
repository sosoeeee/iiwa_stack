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
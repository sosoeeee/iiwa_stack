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
    stepDis = targetSpeed / controllerFreq
    stepTheta = stepDis / R
    theta = np.arange(0, 2 * np.pi, stepTheta)

    x = R * np.cos(theta) + initX
    y = R * np.sin(theta) + initY
    z = np.ones(len(theta)) * initZ

    # 前向差分求速度
    speedMatrix = np.eye(len(theta)) * (-1)
    for i in range(len(theta)):
        speedMatrix[i][(i - 1) % len(theta)] = 1
    speedMatrix = speedMatrix.T

    vx = speedMatrix.dot(x) * controllerFreq
    vy = speedMatrix.dot(y) * controllerFreq
    vz = speedMatrix.dot(z) * controllerFreq

    # vx = -targetSpeed * np.sin(theta)
    # vy =  targetSpeed * np.cos(theta)
    # vz = np.zeros(len(theta))

    # 合并为一个6*len(theta)的矩阵
    trajectory = np.vstack((x, y, z, vx, vy, vz))

    return trajectory


def getCircleHuman(R, targetSpeed, controllerFreq, initX, initY, initZ):
    # 二维测试轨迹——圆形
    stepDis = targetSpeed / controllerFreq
    stepTheta = stepDis / R
    theta = np.arange(0, 2 * np.pi, stepTheta)

    xInit = R * np.cos(theta) + initX
    yInit = R * np.sin(theta) + initY

    changeIndex1 = int(len(theta) / 3)
    changeIndex2 = int(len(theta) / 3 * 2)

    midX = (xInit[changeIndex1] + xInit[changeIndex2]) / 2 - R / 2
    midY = (yInit[changeIndex1] + yInit[changeIndex2]) / 2

    DistanceToMid = ((xInit[changeIndex1] - midX) ** 2 + (yInit[changeIndex1] - midY) ** 2) ** 0.5
    num = int(DistanceToMid / stepDis)  # 直线段的点数

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


def getLine(distance, targetSpeed, controllerFreq, initX, initY, initZ, Direction):
    # 二维测试轨迹——直线
    stepDis = targetSpeed / controllerFreq
    num = int(distance / stepDis)  # 直线段的点数

    if Direction == 'x':
        x = np.linspace(initX, initX + distance, num)
        y = np.ones(num) * initY
    elif Direction == 'y':
        x = np.ones(num) * initX
        y = np.linspace(initY, initY + distance, num)

    z = np.ones(len(x)) * initZ

    # # 前向差分求速度
    # speedMatrix = np.eye(len(z)) * (-1)
    # for i in range(len(z)):
    #     speedMatrix[i][(i - 1) % len(z)] = 1
    # speedMatrix = speedMatrix.T

    # vx = speedMatrix.dot(x) * controllerFreq
    # vy = speedMatrix.dot(y) * controllerFreq
    # vz = speedMatrix.dot(z) * controllerFreq

    if Direction == 'x':
        vx = np.ones(len(x)) * targetSpeed
        vy = np.zeros(len(x))
    elif Direction == 'y':
        vx = np.zeros(len(x))
        vy = np.ones(len(x)) * targetSpeed

    vz = np.zeros(len(x))

    # 合并为一个6*len(theta)的矩阵
    trajectory = np.vstack((x, y, z, vx, vy, vz))

    return trajectory


class MinimumTrajPlanner:
    def __init__(self, path, arvSpeed, controllerFreq, v0, a0, vt, at, r):
        # r:derivertive order, 1:minimum vel 2:minimum acc 3:minimum jerk 4:minimum snap
        self.r = r

        self.path = path
        self.arvSpeed = arvSpeed  # 轨迹平均速度
        self.Freq = controllerFreq  # 控制频率

        # 检查速度和加速度是否为3*1
        if v0.shape != (3, 1) or a0.shape != (3, 1) or vt.shape != (3, 1) or at.shape != (3, 1):
            raise ValueError('v0, a0, vt, at must be 3*1')

        # 起始速度、加速度
        self.v0 = v0.T
        self.a0 = a0.T

        # 终止速度、加速度
        self.vt = vt.T
        self.at = at.T

        # 路径点的时刻
        self.ts = None
        self.T = None

        # 轨迹阶数
        self.order = 5

    def arrangeT(self):
        # 计算每个路径点的时间戳

        dx = np.diff(self.path)

        distance = np.sum(np.sqrt(np.sum(dx ** 2, axis=0)))
        self.T = distance / self.arvSpeed

        self.ts = [0]
        for i in range(self.path.shape[1] - 1):
            self.ts.append(self.ts[i] + np.sqrt(np.sum(dx[:, i] ** 2)) / self.arvSpeed)

        # # debug
        # if self.ts[-1] == self.T:
        #     print('T is right')

        self.ts = np.array(self.ts)
        # np.savetxt('ts.txt', self.ts, fmt='%.4f', delimiter=',')

    def computeQ(self, n, r, t1, t2):
        # n:polynormial order
        # r:derivertive order, 1:minimum vel 2:minimum acc 3:minimum jerk 4:minimum snap
        # t1:start timestamp for polynormial
        # t2:end timestap for polynormial

        T = np.zeros(((n - r) * 2 + 1, 1))
        for i in range((n - r) * 2 + 1):
            T[i] = t2 ** (i + 1) - t1 ** (i + 1)

        Q = np.zeros((n + 1, n + 1))
        for i in range(r + 1, n + 2):  # i:row, 从1开始
            for j in range(i, n + 2):
                k1 = i - r - 1
                k2 = j - r - 1
                k = k1 + k2 + 1
                Q[i - 1, j - 1] = np.prod(np.arange(k1 + 1, k1 + r + 1)) * np.prod(np.arange(k2 + 1, k2 + r + 1)) / k * \
                                  T[k - 1]
                Q[j - 1, i - 1] = Q[i - 1, j - 1]

        return Q

    def computeSingleAxisTraj(self, path, v0, a0, vt, at):
        # 计算单维度轨迹多项式参数

        path = path.reshape(1, -1)
        n_coef = self.order + 1  # 多项式系数个数
        n_seg = path.shape[1] - 1  # 轨迹段数

        # compute Q
        Q = np.zeros((n_coef * n_seg, n_coef * n_seg))
        for k in range(n_seg):
            Q_k = self.computeQ(self.order, self.r, self.ts[k], self.ts[k + 1])
            Q[k * n_coef:(k + 1) * n_coef, k * n_coef:(k + 1) * n_coef] = Q_k

        # compute Tk Tk(i,j) = ts(i)^(j)
        Tk = np.zeros((n_seg + 1, n_coef))
        for i in range(n_coef):
            Tk[:, i] = self.ts ** i

        # compute A
        n_continuous = 3  # 1:p  2:pv  3:pva  4:pvaj  5:pvajs
        A = np.zeros((n_continuous * 2 * n_seg, n_coef * n_seg))
        for i in range(1, n_seg + 1):
            for j in range(1, n_continuous + 1):
                for k in range(j, n_coef + 1):
                    # if k == j:
                    #     t1 = 1
                    #     t2 = 1
                    # else:
                    #     t1 = Tk[i - 1, k - j]
                    #     t2 = Tk[i, k - j]
                    t1 = Tk[i - 1, k - j]
                    t2 = Tk[i, k - j]
                    A[n_continuous * 2 * (i - 1) + j - 1, n_coef * (i - 1) + k - 1] = np.prod(
                        np.arange(k - j + 1, k)) * t1
                    A[n_continuous * 2 * (i - 1) + j - 1 + n_continuous, n_coef * (i - 1) + k - 1] = np.prod(
                        np.arange(k - j + 1, k)) * t2

        # np.savetxt('A.txt', A, fmt='%f', delimiter=',')
        # # 计算A的行列式
        # print('A的行列式为:', np.linalg.det(A))
        # # 计算A的秩
        # print('A的秩为:', np.linalg.matrix_rank(A), A.shape)

        # compute M
        M = np.zeros((n_continuous * 2 * n_seg, n_continuous * (n_seg + 1)))
        for i in range(1, 2 * n_seg + 1):
            j = int(np.floor(i / 2)) + 1
            rbeg = (i - 1) * n_continuous + 1
            cbeg = (j - 1) * n_continuous + 1
            M[rbeg - 1:rbeg + n_continuous - 1, cbeg - 1:cbeg + n_continuous - 1] = np.eye(n_continuous)

        # compute C
        num_d = n_continuous * (n_seg + 1)
        C = np.eye(num_d)
        df = np.hstack((path, np.array(v0).reshape(1, -1), np.array(a0).reshape(1, -1), np.array(vt).reshape(1, -1),
                        np.array(at).reshape(1, -1))).T
        fix_idx = np.array(range(1, num_d, n_continuous))
        fix_idx = np.hstack((fix_idx, 2, 3, num_d - 1, num_d))
        free_idx = np.setdiff1d(range(1, num_d + 1), fix_idx)
        C = np.hstack((C[:, fix_idx - 1], C[:, free_idx - 1]))

        AiMC = np.linalg.inv(A).dot(M).dot(C)
        R = AiMC.T.dot(Q).dot(AiMC)

        n_fix = len(fix_idx)
        # Rff = R[:n_fix, :n_fix]
        Rfp = R[:n_fix, n_fix:]
        # Rpf = R[n_fix:, :n_fix]
        Rpp = R[n_fix:, n_fix:]

        dp = -np.linalg.inv(Rpp).dot(Rfp.T).dot(df)

        p = AiMC.dot(np.vstack((df, dp)))

        ploys = p.reshape((n_coef, n_seg), order='F')

        return ploys

    # 计算轨迹位置[1, t, t^2, ... , t^n]
    def polys_vals(self, ploys, t1, t2, dt):
        # 检查ploys是否为n*1
        ploys = ploys.reshape(-1, 1)
        if ploys.shape[1] != 1:
            print('ploys shape error')
            return None

        t = np.arange(t1, t2, dt)
        n = ploys.shape[0]
        traj = ploys[0, 0] * np.ones(len(t))
        for i in range(1, n):
            traj = traj + ploys[i, 0] * t ** i

        return traj

    # 计算轨迹速度[0, 1, 2t, 3t^2, ... , nt^(n-1)]
    def polys_d_vals(self, ploys, t1, t2, dt):
        # 检查ploys是否为n*1
        ploys = ploys.reshape(-1, 1)
        if ploys.shape[1] != 1:
            print('ploys shape error')
            return None

        t = np.arange(t1, t2, dt)
        n = ploys.shape[0]
        traj = np.zeros(len(t))
        for i in range(1, n):
            traj = traj + i * ploys[i, 0] * t ** (i - 1)

        return traj

    def computeTraj(self):
        # 检查path是否为3*n
        if self.path.shape[0] != 3:
            print('path shape error')
            return None

        self.arrangeT()

        # 计算多项式系数
        polys_x = self.computeSingleAxisTraj(self.path[0, :], self.v0[0, 0], self.a0[0, 0], self.vt[0, 0],
                                             self.at[0, 0])
        polys_y = self.computeSingleAxisTraj(self.path[1, :], self.v0[0, 1], self.a0[0, 1], self.vt[0, 1],
                                             self.at[0, 1])
        polys_z = self.computeSingleAxisTraj(self.path[2, :], self.v0[0, 2], self.a0[0, 2], self.vt[0, 2],
                                             self.at[0, 2])

        # 计算轨迹
        n_seg = self.path.shape[1] - 1

        x = self.polys_vals(polys_x[:, 0], self.ts[0], self.ts[1], 1 / self.Freq)
        y = self.polys_vals(polys_y[:, 0], self.ts[0], self.ts[1], 1 / self.Freq)
        z = self.polys_vals(polys_z[:, 0], self.ts[0], self.ts[1], 1 / self.Freq)

        vx = self.polys_d_vals(polys_x[:, 0], self.ts[0], self.ts[1], 1 / self.Freq)
        vy = self.polys_d_vals(polys_y[:, 0], self.ts[0], self.ts[1], 1 / self.Freq)
        vz = self.polys_d_vals(polys_z[:, 0], self.ts[0], self.ts[1], 1 / self.Freq)

        for i in range(1, n_seg):
            x = np.hstack((x, self.polys_vals(polys_x[:, i], self.ts[i], self.ts[i + 1], 1 / self.Freq)))
            y = np.hstack((y, self.polys_vals(polys_y[:, i], self.ts[i], self.ts[i + 1], 1 / self.Freq)))
            z = np.hstack((z, self.polys_vals(polys_z[:, i], self.ts[i], self.ts[i + 1], 1 / self.Freq)))
            vx = np.hstack((vx, self.polys_d_vals(polys_x[:, i], self.ts[i], self.ts[i + 1], 1 / self.Freq)))
            vy = np.hstack((vy, self.polys_d_vals(polys_y[:, i], self.ts[i], self.ts[i + 1], 1 / self.Freq)))
            vz = np.hstack((vz, self.polys_d_vals(polys_z[:, i], self.ts[i], self.ts[i + 1], 1 / self.Freq)))

        trajectory = np.vstack((x, y, z, vx, vy, vz))

        return trajectory

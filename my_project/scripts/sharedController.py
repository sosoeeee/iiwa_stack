#!/usr/bin/env python3

import numpy as np
from RRT_PathPlanner import PathPlanner
from TrajectoryGenerator import MinimumTrajPlanner

class sharedController:
    def __init__(self, controllerFreq):
        # 控制频率
        self.controllerFreq = controllerFreq    # Hz

        # 阻抗模型
        self.Md = 5 * np.eye(3)
        self.Cd = 75 * np.eye(3)

        # 状态变量
        self.w = np.zeros((6, 1))

        # 状态空间模型
        self.Ad = None
        self.Brd = None
        self.Bhd = None

        # 局部期望轨迹
        self.localLen = 50
        self.robotLocalTraj = None  # 3*localLen
        self.humanLocalTraj = None  # 3*localLen

        # 全局期望轨迹
        self.robotGlobalTraj = None
        self.robotGLobalLen = None

        # 安全系数
        self.lambda_ = 0.5

        # 控制器参数
        self.Qr = None
        self.Qh = None
        self.theta_rg = None
        self.theta_hg = None
        self.phi_g = None

        # 人类意图大小
        # 0：无意图
        # 1: 有微小意图
        # 2: 有明显意图
        self.hunmanIntent = 0

        # 轨迹重规划长度
        self.replanLen = 100
        self.R = None
        self.replanPathNum = 10 # 备选重规划轨迹数量
        self.alpha = 100 # 重规划轨迹优化时人类意图的权重
    
    def initialize(self, weight_r, weight_h, weight_error):
        # 机械臂状态空间模型建立
        Md_inv = np.linalg.inv(self.Md)
        A = np.zeros((6, 6))
        A[0:3, 3:6] = np.eye(3)
        A[3:6, 3:6] = -Md_inv.dot(self.Cd)
        Br = np.zeros((6, 3))
        Br[3:6, :] = Md_inv
        Bh = np.zeros((6, 3))
        Bh[3:6, :] = Md_inv
        C = np.zeros((3, 6))
        C[:, 0:3] = np.eye(3)

        # 离散化
        T = 1/self.controllerFreq
        self.Ad = np.eye(6) + A*T
        self.Brd = Br*T
        self.Bhd = Bh*T

        # 控制器参数设置
        phi = np.zeros((3*self.localLen, 6))
        theta_h = np.zeros((3*self.localLen, 3*self.localLen))
        theta_r = np.zeros((3*self.localLen, 3*self.localLen))
        phi_row = C.dot(self.Ad)
        Qii_r = np.eye(3) * weight_r * weight_error
        Qii_h = np.eye(3) * weight_h * weight_error
        self.Qr = np.zeros((3*self.localLen, 3*self.localLen))
        self.Qh = np.zeros((3*self.localLen, 3*self.localLen))

        for i in range(self.localLen):
            phi[(i*3):(3*i+3), :] = phi_row
            self.Qr[(i*3):(3*i+3), (i*3):(3*i+3)] = Qii_r
            self.Qh[(i*3):(3*i+3), (i*3):(3*i+3)] = Qii_h
            theta_h_row = np.eye(6)
            for j in range(i+1):
                theta_h[(i*3):(3*i+3),(3*(i-j)):(3*(i-j)+3)] = np.dot(C, theta_h_row.dot(self.Bhd))
                theta_r[(i*3):(3*i+3),(3*(i-j)):(3*(i-j)+3)] = np.dot(C, theta_h_row.dot(self.Brd))
                theta_h_row = theta_h_row.dot(self.Ad)
            phi_row = np.dot(phi_row, self.Ad)

        # print("theta_h", theta_h)
        # print("theta_r", theta_r)
        # print("phi", phi)

        self.theta_rg = np.vstack((theta_h, theta_h))
        self.theta_hg = np.vstack((theta_r, theta_r))
        self.phi_g = np.vstack((phi, phi))

        # 初始化重规划参数
        lenReplanA = self.replanLen
        replanA = np.zeros((lenReplanA + 3, lenReplanA))
        for i in range(1, lenReplanA + 3):
            for j in range(1, lenReplanA):
                if i == j:
                    replanA[i-1, j-1] = 1
                elif i == j + 1:
                    replanA[i-1, j-1] = -3
                elif i == j + 2:
                    replanA[i-1, j-1] = 3
                elif i == j + 3:
                    replanA[i-1, j-1] = -1
        self.R = replanA.T.dot(replanA)

    def updateState(self, w):
        # 检查w是否为6*1的矩阵
        if w.shape[0] != 6 or w.shape[1] != 1:
            print("Error: The shape of w is worng")
            return
        

        self.w = w

    def setRobotGlobalTraj(self, robotGlobalTraj):
        # 检查轨迹是否为6*n的矩阵
        if robotGlobalTraj.shape[0] != 6:
            print("Error: The shape of robotGlobalTraj is not 6*n")
            return 
        elif robotGlobalTraj.shape[1] < self.localLen:
            print("Error: The length of robotGlobalTraj is less than localLen")
            return

        self.robotGLobalLen = robotGlobalTraj.shape[1]
        self.robotGlobalTraj = robotGlobalTraj

    def gethumanLocalTraj(self, stickPos, stickForce, endEffectorPos):

        distance = (stickPos[0]**2 + stickPos[1]**2)**0.5 ## 只考虑二维情况
        # print("distance", distance)
        deltaT = 1/self.controllerFreq
        t = np.arange(0, self.localLen*deltaT, deltaT)
        speedAmplitude = 0.5                    # 遥操作杆偏离中心的位置与机器人末端运动速度的比例系数

        force = (stickForce[0]**2 + stickForce[1]**2)**0.5 ## 只考虑二维情况
        # print("force", force)

        if distance > 0.01:
            speed = distance * speedAmplitude  # max：2.5cm/s
            cos = stickPos[0] / distance
            sin = stickPos[1] / distance
            PosX = speed * cos * t + endEffectorPos[0]
            PosY = speed * sin * t + endEffectorPos[1]
            PosZ = np.ones(len(t)) * endEffectorPos[2]
            self.humanLocalTraj = np.vstack((PosX, PosY, PosZ))

            if force > 2.5:
                self.hunmanIntent = 2
            else:
                self.hunmanIntent = 1

        else:
            self.hunmanIntent = 0
            self.humanLocalTraj = np.zeros((3, self.localLen))

    def computeLambda(self, obstacles):
        pass

    def reshapeLocalTraj(self, localTraj):
        # 检查localTraj是否为3*n的矩阵
        if localTraj.shape[0] != 3:
            print("Error: The shape of localTraj is not 3*n")
            return
        elif localTraj.shape[1] != self.localLen:
            print("Error: The length of localTraj is not equal to localLen")
            print("localTraj.shape[1]", localTraj.shape[1])
            return

        reshaped = np.zeros((3*self.localLen, 1))
        for i in range(self.localLen):
            reshaped[(i*3):(3*i+3), 0] = localTraj[:, i]

        return reshaped
    
    def getHumanIntent(self):

        # print("humanIntent", self.hunmanIntent)

        return self.hunmanIntent
    
    def changeGlobalTraj(self, currentTrajIndex, humanForce, obstacles, avrSpeed):
        startPoint = self.robotGlobalTraj[:3, currentTrajIndex].reshape((3, 1))
        # endPoint = self.robotGlobalTraj[:3, currentTrajIndex+self.replanLen-1].reshape((3, 1))

        # ----- 轨迹循环读取 -----
        if currentTrajIndex + self.replanLen > self.robotGLobalLen:
            endPoint = self.robotGlobalTraj[:3, currentTrajIndex+self.replanLen-self.robotGLobalLen-1].reshape((3, 1))
        else:
            endPoint = self.robotGlobalTraj[:3, currentTrajIndex+self.replanLen-1].reshape((3, 1))
        # ----- 轨迹循环读取 -----

        pathPlanner = PathPlanner(startPoint, endPoint)
        for obstacle in obstacles:
            if obstacle['state'] == 1:
                pathPlanner.addObstacle(obstacle['center'], obstacle['radius'])
        
        trajSet = []

        for i in range(self.replanPathNum):
            # 轨迹筛选
            path = pathPlanner.RRT(False)

            startVel = self.robotGlobalTraj[3:6, currentTrajIndex].reshape((3, 1))
            # endVel = self.robotGlobalTraj[3:6, currentTrajIndex+self.replanLen-1].reshape((3, 1))

            # ----- 轨迹循环读取 -----
            if currentTrajIndex + self.replanLen > self.robotGLobalLen:
                endVel = self.robotGlobalTraj[3:6, currentTrajIndex+self.replanLen-self.robotGLobalLen-1].reshape((3, 1))
            else:
                endVel = self.robotGlobalTraj[3:6, currentTrajIndex+self.replanLen-1].reshape((3, 1))
            # ----- 轨迹循环读取 -----

            # 由于轨迹信息中没有存储加速度，这里的加速度将不连续
            startAcc = np.array([0, 0, 0]).reshape((3, 1))
            endAcc = np.array([0, 0, 0]).reshape((3, 1)) 
            miniJerkTrajPlanner = MinimumTrajPlanner(path, avrSpeed, self.controllerFreq, startVel, startAcc, endVel, endAcc, 3)
            miniJerkTraj = miniJerkTrajPlanner.computeTraj()

            # 轨迹抽样（保证了更新的局部轨迹与原轨迹在时间上的一致性，但是牺牲了轨迹的平均速度期望）
            index = np.arange(0, self.replanLen, 1) * (miniJerkTraj.shape[1] / self.replanLen)
            miniJerkTrajSampled = np.zeros((6, self.replanLen))
            for j in range(self.replanLen):
                miniJerkTrajSampled[:, j] = miniJerkTraj[:, int(index[j])]
            
            if miniJerkTrajSampled.shape != (6, self.replanLen):
                raise Exception("Error: The shape of miniJerkTrajSampled is wrong")

            trajSet.append(miniJerkTrajSampled)
        
        originTrajPosition = self.robotGlobalTraj[:3, currentTrajIndex:(currentTrajIndex+self.replanLen)]
        humanForceVector = np.ones((3, self.replanLen))
        humanForceVector[0, :] = humanForce[0]
        humanForceVector[1, :] = humanForce[1]
        humanForceVector[2, :] = 0 ## 只考虑二维情况

        # 计算每条备选轨迹的能量
        energySet = []
        for i in range(self.replanPathNum):
            Ex = 1/(2*self.alpha) * trajSet[i][0, :].dot(self.R.dot(trajSet[i][0, :].T)) + humanForceVector[0, :].dot(trajSet[i][0, :].T) - 1/self.alpha * originTrajPosition[0, :].dot(self.R.dot(trajSet[i][0, :].T))
            Ey = 1/(2*self.alpha) * trajSet[i][1, :].dot(self.R.dot(trajSet[i][1, :].T)) + humanForceVector[1, :].dot(trajSet[i][1, :].T) - 1/self.alpha * originTrajPosition[1, :].dot(self.R.dot(trajSet[i][1, :].T))
            Ez = 1/(2*self.alpha) * trajSet[i][2, :].dot(self.R.dot(trajSet[i][2, :].T)) + humanForceVector[2, :].dot(trajSet[i][2, :].T) - 1/self.alpha * originTrajPosition[2, :].dot(self.R.dot(trajSet[i][2, :].T))

            energySet.append(Ex + Ey + Ez)
        # 最小能量对应的轨迹
        miniEnergyIndex = energySet.index(min(energySet))

        if trajSet[miniEnergyIndex].shape != (6, self.replanLen):
            raise Exception("Error: The shape of miniEnergyTraj is wrong")

        # 改变轨迹
        # self.robotGlobalTraj[:, currentTrajIndex:(currentTrajIndex+self.replanLen)] = trajSet[miniEnergyIndex][:, :self.replanLen]

        # ----- 轨迹循环写入 -----
        if currentTrajIndex + self.replanLen > self.robotGLobalLen:
            self.robotGlobalTraj[:, currentTrajIndex:] = trajSet[miniEnergyIndex][:, :self.robotGLobalLen-currentTrajIndex]
            self.robotGlobalTraj[:, :(currentTrajIndex+self.replanLen-self.robotGLobalLen)] = trajSet[miniEnergyIndex][:, self.robotGLobalLen-currentTrajIndex:]
        else:
            self.robotGlobalTraj[:, currentTrajIndex:(currentTrajIndex+self.replanLen)] = trajSet[miniEnergyIndex][:, :self.replanLen]
        # ----------------------

        return self.robotGlobalTraj

    def computeLocalTraj(self, robotGlobalTrajStartIndex):
        # if robotGlobalTrajStartIndex + self.localLen > self.robotGlobalTraj.shape[1]:
        #     print("Error: Index out of the range of robotGlobalTraj")

        # 当人类有意图时，返回共享控制器计算的局部轨迹
        # self.robotLocalTraj = self.robotGlobalTraj[:3, robotGlobalTrajStartIndex:(robotGlobalTrajStartIndex+self.localLen)]

        # ----- 轨迹循环读取 -----
        if robotGlobalTrajStartIndex + self.localLen > self.robotGLobalLen:
            self.robotLocalTraj = self.robotGlobalTraj[:3, robotGlobalTrajStartIndex:]
            self.robotLocalTraj = np.hstack((self.robotLocalTraj, self.robotGlobalTraj[:3, :(robotGlobalTrajStartIndex+self.localLen-self.robotGLobalLen)]))
        else:
            self.robotLocalTraj = self.robotGlobalTraj[:3, robotGlobalTrajStartIndex:(robotGlobalTrajStartIndex+self.localLen)]
        # ----------------------

        # 当人类无意图时，人类局部轨迹与机器人局部轨迹相同
        if self.hunmanIntent == 0:
            self.humanLocalTraj = self.robotLocalTraj

        X_dr = self.reshapeLocalTraj(self.robotLocalTraj)
        X_dh = self.reshapeLocalTraj(self.humanLocalTraj)
        X_d = np.vstack((X_dr, X_dh))

        # # 将状态变量与X_d写入txt
        # np.savetxt("w.txt", self.w)
        # np.savetxt("X_d.txt", X_d)

        # 将Q_h和Q_r对角拼接
        Q = np.vstack((np.hstack((self.Qh * self.lambda_, np.zeros((3*self.localLen, 3*self.localLen)))), np.hstack((np.zeros((3*self.localLen, 3*self.localLen)), self.Qr * (1-self.lambda_)))))
        SQ = np.sqrt(Q)
        SP = np.eye(3*self.localLen)
        wx = np.vstack((self.w, X_d))

        tmp1_L_h = np.linalg.pinv(np.vstack((SQ.dot(self.theta_hg), np.sqrt(self.lambda_) * SP)))
        tmp2_L_h = np.vstack((SQ, np.zeros((3*self.localLen, 6*self.localLen))))
        L_h = tmp1_L_h.dot(tmp2_L_h)

        tmp1_L_r = np.linalg.pinv(np.vstack((SQ.dot(self.theta_rg), np.sqrt(1-self.lambda_) * SP)))
        tmp2_L_r = np.vstack((SQ, np.zeros((3*self.localLen, 6*self.localLen))))
        L_r = tmp1_L_r.dot(tmp2_L_r)

        H_h = np.hstack((-L_h.dot(self.phi_g), L_h))
        H_r = np.hstack((-L_r.dot(self.phi_g), L_r))

        k_r1 = np.eye(3*self.localLen) - np.dot(L_r.dot(self.theta_hg), L_h.dot(self.theta_rg))
        k_r2 = H_r - np.dot(L_r.dot(self.theta_hg), H_h)
        k_r = np.linalg.pinv(k_r1).dot(k_r2)

        k_h1 = np.eye(3*self.localLen) - np.dot(L_h.dot(self.theta_rg), L_r.dot(self.theta_hg))
        k_h2 = H_h - np.dot(L_h.dot(self.theta_rg), H_r)
        k_h = np.linalg.pinv(k_h1).dot(k_h2)

        k_0 = np.zeros((3, 3*self.localLen))
        k_0[:3, :3] = np.eye(3)

        u_r = np.dot(k_0, k_r.dot(wx))
        u_h = np.dot(k_0, k_h.dot(wx))

        w_next = self.Ad.dot(self.w) + self.Brd.dot(u_r) + self.Bhd.dot(u_h)

        return w_next

        



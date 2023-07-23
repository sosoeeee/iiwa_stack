#!/usr/bin/env python3

import numpy as np

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

    def gethumanLocalTraj(self, stickPos, endEffectorPos):
        distance = (stickPos[0]**2 + stickPos[1]**2)**0.5

        deltaT = 1/self.controllerFreq
        t = np.arange(0, self.localLen*deltaT, deltaT)
        speedAmplitude = 0.5                    # 遥操作杆偏离中心的位置与机器人末端运动速度的比例系数

        if distance > 0.01:
            speed = distance * speedAmplitude  # max：2.5cm/s
            cos = stickPos[0] / distance
            sin = stickPos[1] / distance
            PosX = speed * cos * t + endEffectorPos[0]
            PosY = speed * sin * t + endEffectorPos[1]
            PosZ = np.ones(len(t)) * endEffectorPos[2]
            self.hunmanIntent = 1
            self.humanLocalTraj = np.vstack((PosX, PosY, PosZ))
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

        x = localTraj[0, :].reshape((self.localLen, 1))
        y = localTraj[1, :].reshape((self.localLen, 1))
        z = localTraj[2, :].reshape((self.localLen, 1))
        reshaped = np.vstack((x, y, z))
        return reshaped
    
    def getHumanIntent(self):

        # print("humanIntent", self.hunmanIntent)

        return self.hunmanIntent
    
    def changeGlobalTraj(self, obstacles):
        pass

    def computeLocalTraj(self, robotGlobalTrajStartIndex):
        # if robotGlobalTrajStartIndex + self.localLen > self.robotGlobalTraj.shape[1]:
        #     print("Error: Index out of the range of robotGlobalTraj")

        # 当人类无意图时，返回机器人全局轨迹
        if self.hunmanIntent == 0:
            return self.robotGlobalTraj[:, robotGlobalTrajStartIndex]

        # 当人类有意图时，返回共享控制器计算的局部轨迹
        # self.robotLocalTraj = self.robotGlobalTraj[:3, robotGlobalTrajStartIndex:(robotGlobalTrajStartIndex+self.localLen)]

        # ----- 轨迹循环读取 -----
        if robotGlobalTrajStartIndex + self.localLen > self.robotGLobalLen:
            self.robotLocalTraj = self.robotGlobalTraj[:3, robotGlobalTrajStartIndex:]
            self.robotLocalTraj = np.hstack((self.robotLocalTraj, self.robotGlobalTraj[:3, :(robotGlobalTrajStartIndex+self.localLen-self.robotGLobalLen)]))
        else:
            self.robotLocalTraj = self.robotGlobalTraj[:3, robotGlobalTrajStartIndex:(robotGlobalTrajStartIndex+self.localLen)]
        # ----------------------

        X_dr = self.reshapeLocalTraj(self.robotLocalTraj)
        X_dh = self.reshapeLocalTraj(self.humanLocalTraj)
        X_d = np.vstack((X_dr, X_dh))

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

        



#!/usr/bin/env python3

import sys
from PyQt5.Qt import *
from pyqtgraph import PlotWidget
from PyQt5 import QtCore
import numpy as np
import pyqtgraph as pq

import rospy
from std_msgs.msg import String, Float32MultiArray
from iiwa_msgs.msg import CartesianPose


monitorAmplitude = 3000
monitorLen = 800
monitorRefreshTime = 50                # ms

humanX = [0]
humanY = [0]
controllerFreq = 10                    # 控制频率
deltaT = 1/controllerFreq
len = 50                                # 人生成的期望轨迹长度
t = np.arange(0, len*deltaT, deltaT)
speedAmplitude = 0.5                    # 遥操作杆偏离中心的位置与机器人末端运动速度的比例系数

robotX = [0] * monitorLen
robotY = [0] * monitorLen
initX = -0.41
initY = 0.15
initZ = 0.18
index = 0
currentRobotX = 0
currentRobotY = 0

class TrajectoryListenerThread(QtCore.QThread):
    # 定义一个信号
    dataSignal = QtCore.pyqtSignal(dict)

    def run(self):
        rospy.Subscriber("referRobotTrajectory", Float32MultiArray, self.callback_robot)
        rospy.Subscriber("referHumanTrajectory", Float32MultiArray, self.callback_human)
        rospy.spin()
    
    def callback_robot(self, msg):
        dataDict = {}
        dataDict['data'] = msg.data
        dataDict['type'] = 0

        self.dataSignal.emit(dataDict)

    def callback_human(self, msg):
        dataDict = {}
        dataDict['data'] = msg.data
        dataDict['type'] = 1

        self.dataSignal.emit(dataDict)

class StickListenerThread(QtCore.QThread):
    # 定义一个信号
    rosSignal = QtCore.pyqtSignal(str)

    def run(self):
        rospy.Subscriber("stickSignal", String, self.callback)
        rospy.spin()

    def callback(self, msg):
        self.rosSignal.emit(msg.data)


class RobotListenerThread(QtCore.QThread):
    # 定义一个信号
    rosSignal = QtCore.pyqtSignal(str)

    def run(self):
        rospy.Subscriber('/iiwa/state/CartesianPose', CartesianPose, self.callback, queue_size=1)
        rospy.spin()

    def callback(self, msg):
        global currentRobotX, currentRobotY
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.data)
        currentRobotX = msg.poseStamped.pose.position.x
        currentRobotY = msg.poseStamped.pose.position.y


class ObstacleListenerThread(QtCore.QThread):
    dataSignal = QtCore.pyqtSignal(str)

    def run(self):
        rospy.Subscriber("Obstacles", String, self.callback, queue_size=10)
        rospy.spin()
    
    def callback(self, msg):
        self.dataSignal.emit(msg.data)

class Figure(QWidget):
    def __init__(self):
        super().__init__()
        # 添加 PlotWidget 控件
        self.plotWidget_ted = PlotWidget(self)

        # 设置 PlotWidget 控件的坐标轴范围
        xmin = -600
        xmax = 600
        ymin = -2000
        ymax = 100
        self.plotWidget_ted.setXRange(xmin, xmax)
        self.plotWidget_ted.setYRange(ymin, ymax)
        # 设置坐标轴分度一致
        self.plotWidget_ted.setAspectLocked(True)
        # 设置下尺寸
        self.length = 800
        self.resize(self.length, self.length)
        # 设置该控件尺寸和相对位置
        self.plotWidget_ted.setGeometry(QtCore.QRect(25, 25, self.length - 50, self.length - 50))
                
        # 添加图像
        self.humanIntent = self.plotWidget_ted.plot([0], [0], pen=None, symbol='o', symbolSize=1, symbolBrush='w', name="mode1")
        self.robotPosition = self.plotWidget_ted.plot([0], [0], pen=None, symbol='o', symbolSize=1, symbolBrush='r', name="mode2")
        self.robotTrajectory = self.plotWidget_ted.plot([0], [0], pen=None, symbol='o', symbolSize=1, symbolBrush='b', name="mode3")
        self.humanTrajectory = self.plotWidget_ted.plot([0], [0], pen=None, symbol='o', symbolSize=1, symbolBrush='g', name="mode4")

        # Trajcetory 监听器
        self.trajectoryListener_thread = TrajectoryListenerThread()
        self.trajectoryListener_thread.dataSignal.connect(self.updateTrajectory)
        self.trajectoryListener_thread.start()

        # stick 监听器
        self.stickListener_thread = StickListenerThread()
        self.stickListener_thread.rosSignal.connect(self.updatestick)
        self.stickListener_thread.start()

        # robot 监听器
        self.robotListener_thread = RobotListenerThread()
        # self.robotListener_thread.rosSignal.connect(self.updateRobot)
        self.robotListener_thread.start()

        # obstacle 监听器
        self.obstacleListener_thread = ObstacleListenerThread()
        self.obstacleListener_thread.dataSignal.connect(self.updateObstacle)
        self.obstacleListener_thread.start()
        self.obstaclesSet = None
        
        # 设定轨迹更新定时器
        self.timer = pq.QtCore.QTimer()
        # 定时器信号绑定 update_data 函数
        self.timer.timeout.connect(self.updateRobotTrajectory)
        # 定时器间隔50ms，可以理解为 50ms 刷新一次数据
        self.timer.start(monitorRefreshTime)

    def updateObstacle(self, data):
        # 按逗号分割字符串
        data = data.split(',')
        # 将字符串转换为浮点数
        data = [float(i) for i in data]

        x = data[0]
        y = data[1]
        z = data[2]
        radius = data[3]

        # 检查是否已经存在该障碍物，若存在则找到该障碍物并更新其轨迹
        update = False
        if self.obstaclesSet is None:
            self.obstaclesSet = []
            self.obstaclesSet.append({
                'center': np.array([x, y, z]).reshape((3, 1)),
                'trajectory': self.getCircleTrajectory(x, y, radius)
            })
        else:
            for obstacle in self.obstaclesSet:
                if np.linalg.norm(np.array([x, y, z]).reshape((3, 1)) - obstacle['center']) < 0.03:
                    obstacle['trajectory'] = self.getCircleTrajectory(x, y, radius)
                    update = True # 旧有障碍物发生移动
                    break
            # 新障碍物
            if update is False:
                self.obstaclesSet.append({
                    'center': np.array([x, y, z]).reshape((3, 1)),
                    'trajectory': self.getCircleTrajectory(x, y, radius)
                })
        
        self.DrawObstacles()

    def DrawObstacles(self):
        for obstacle in self.obstaclesSet:
            self.plotWidget_ted.plot(obstacle['trajectory'][0], obstacle['trajectory'][1], pen=None, symbol='o', symbolSize=1, symbolBrush='y')

    def getCircleTrajectory(self, x, y, radius):
        # 生成圆形轨迹
        theta = np.arange(0, 2*np.pi, 0.1)
        x = x + radius * np.cos(theta)
        y = y + radius * np.sin(theta)
        x = (x - initX) * monitorAmplitude
        y = (y - initY) * monitorAmplitude

        return [x.tolist(), y.tolist()]

    def updatestick(self, pointAndForce): ## 只考虑二维情况
        # 按逗号分割字符串
        pointAndForce = pointAndForce.split(',')
        # 将字符串转换为浮点数
        pointAndForce = [float(i) for i in pointAndForce]

        distance = (pointAndForce[0]**2 + pointAndForce[1]**2)**0.5
        if distance > 0.01:
            speed = distance * speedAmplitude  # max：2.5cm/s
            cos = pointAndForce[0] / distance
            sin = pointAndForce[1] / distance
            PosX = speed * cos * t * monitorAmplitude + (currentRobotX - initX) * monitorAmplitude
            PosY = speed * sin * t * monitorAmplitude + (currentRobotY - initY) * monitorAmplitude
            self.humanIntent.setData(PosX, PosY)
        else:
            self.humanIntent.setData([0], [0])

    def updateRobotTrajectory(self): ## 只考虑二维情况
        global currentRobotX, currentRobotY, index
        robotX[index] = (currentRobotX - initX) * monitorAmplitude 
        robotY[index] = (currentRobotY - initY) * monitorAmplitude 
        index = (index + 1) % monitorLen
        # print(robotX)

        self.robotPosition.setData(robotX, robotY)

    def updateTrajectory(self, dataDict): ## 只考虑二维情况
        # 检查data类型
        data = dataDict['data']
        typeTraj = dataDict['type']
        
        print(typeTraj)

        data = np.array(data)
        len = data.shape[0]
        data = data.reshape((3, int(len/3)))
        data = data * monitorAmplitude

        # 显示修正
        data[0, :] = data[0, :] - initX * monitorAmplitude
        data[1, :] = data[1, :] - initY * monitorAmplitude

        # Robot
        if typeTraj == 0:
            self.robotTrajectory.setData(data[0, :], data[1, :])
        # Human
        elif typeTraj == 1:
            self.humanTrajectory.setData(data[0, :], data[1, :])
        

if __name__ == '__main__':
    # ROS 节点初始化
    rospy.init_node('monitorListener', anonymous=True)

    # PyQt5 程序固定写法
    app = QApplication(sys.argv)

    # 将绑定了绘图控件的窗口实例化并展示
    window = Figure()
    window.show()

    # PyQt5 程序固定写法
    sys.exit(app.exec())

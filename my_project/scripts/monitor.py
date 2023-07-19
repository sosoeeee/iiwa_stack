#!/usr/bin/env python3

import sys
from PyQt5.Qt import *
from pyqtgraph import PlotWidget
from PyQt5 import QtCore
import numpy as np
import pyqtgraph as pq

import rospy
from std_msgs.msg import String
from iiwa_msgs.msg import CartesianPose


monitorAmplitude = 3000
monitorLen = 800
monitorRefreshTime = 50                # ms

humanX = [0]
humanY = [0]
controllerFreq = 20                     # 控制频率
deltaT = 1/controllerFreq
len = 50                                # 人生成的期望轨迹长度
t = np.arange(0, len*deltaT, deltaT)
speedAmplitude = 0.5                    # 遥操作杆偏离中心的位置与机器人末端运动速度的比例系数

robotX = [0] * monitorLen
robotY = [0] * monitorLen
initX = -0.35
initY = -0.35
index = 0
currentRobotX = 0
currentRobotY = 0

class ControllerListenerThread(QtCore.QThread):
    # 定义一个信号
    rosSignal = QtCore.pyqtSignal(str)

    def run(self):
        rospy.Subscriber("controllerSignal", String, self.callback)
        rospy.spin()

    def callback(self, msg):
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.data)
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

class Figure(QWidget):
    def __init__(self):
        super().__init__()
        # 设置下尺寸
        self.resize(600, 600)
        # 添加 PlotWidget 控件
        self.plotWidget_ted = PlotWidget(self)
        # 设置该控件尺寸和相对位置
        self.plotWidget_ted.setGeometry(QtCore.QRect(25, 25, 550, 550))
        # 设置 PlotWidget 控件的坐标轴范围
        self.plotWidget_ted.setXRange(-300, 300)
        self.plotWidget_ted.setYRange(-300, 300)

        self.humanTrajectory = self.plotWidget_ted.plot([0], [0], pen=None, symbol='o', symbolSize=1, symbolPen='r', symbolBrush='w', name="mode2")
        self.robotTrajectory = self.plotWidget_ted.plot([0], [0], pen=None, symbol='o', symbolSize=1, symbolPen='w', symbolBrush='r', name="mode2")

        # controller 监听器
        self.controllerListener_thread = ControllerListenerThread()
        self.controllerListener_thread.rosSignal.connect(self.updateController)
        self.controllerListener_thread.start()

        # robot 监听器
        self.robotListener_thread = RobotListenerThread()
        # self.robotListener_thread.rosSignal.connect(self.updateRobot)
        self.robotListener_thread.start()
        
        # 设定轨迹更新定时器
        self.timer = pq.QtCore.QTimer()
        # 定时器信号绑定 update_data 函数
        self.timer.timeout.connect(self.updateRobotTrajectory)
        # 定时器间隔50ms，可以理解为 50ms 刷新一次数据
        self.timer.start(monitorRefreshTime)

    def updateController(self, point):
        # 按逗号分割字符串
        point = point.split(',')
        # 将字符串转换为浮点数
        point = [float(i) for i in point]

        distance = (point[0]**2 + point[1]**2)**0.5
        if distance > 0.01:
            speed = distance * speedAmplitude  # max：2.5cm/s
            cos = point[0] / distance
            sin = point[1] / distance
            PosX = speed * cos * t * monitorAmplitude + (currentRobotX - initX) * monitorAmplitude
            PosY = speed * sin * t * monitorAmplitude + (currentRobotY - initY) * monitorAmplitude
            self.humanTrajectory.setData(PosX, PosY)
        else:
            self.humanTrajectory.setData([0], [0])

    def updateRobotTrajectory(self):
        global currentRobotX, currentRobotY, index
        robotX[index] = currentRobotX * monitorAmplitude - initX * monitorAmplitude
        robotY[index] = currentRobotY * monitorAmplitude - initY * monitorAmplitude
        index = (index + 1) % monitorLen
        # print(robotX)

        self.robotTrajectory.setData(robotX, robotY)
        

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

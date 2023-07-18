#!/usr/bin/env python3

import sys
from PyQt5.Qt import *
from pyqtgraph import PlotWidget
from PyQt5 import QtCore
import numpy as np
import pyqtgraph as pq

import rospy
from std_msgs.msg import String

PosX = [0]
PosY = [0]
deltaT = 0.05
len = 50
t = np.arange(0, len*deltaT, deltaT)
speedAmplitude = 0.5
monitorAmplitude = 3000

class ROSListenerThread(QtCore.QThread):
    # 定义一个信号
    rosSignal = QtCore.pyqtSignal(str)

    def run(self):
        rospy.Subscriber("controllerSignal", String, self.callback)
        rospy.spin()

    def callback(self, msg):
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.data)
        self.rosSignal.emit(msg.data)


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

        self.trajectory = self.plotWidget_ted.plot([0], [0], pen=None, symbol='o', symbolSize=1, symbolPen='w', symbolBrush='w', name="mode2")

        # ROS 监听器
        self.listener_thread = ROSListenerThread()
        self.listener_thread.rosSignal.connect(self.updateTrajectory)
        self.listener_thread.start()
        
        # # 设定定时器
        # self.timer = pq.QtCore.QTimer()
        # # 定时器信号绑定 update_data 函数
        # self.timer.timeout.connect(self.update_data)
        # # 定时器间隔50ms，可以理解为 50ms 刷新一次数据
        # self.timer.start(50)

    # def keyPressEvent(self, event):
    #     # print(event.key())
    #     if event.key() == Qt.Key_W:
    #         PosX.append(PosX[-1] + 1)
    #         PosY.append(PosY[-1])

    #     if event.key() == Qt.Key_S:
    #         PosX.append(PosX[-1] - 1)
    #         PosY.append(PosY[-1])

    #     if event.key() == Qt.Key_A:
    #         PosX.append(PosX[-1])
    #         PosY.append(PosY[-1] - 1)

    #     if event.key() == Qt.Key_D:
    #         PosX.append(PosX[-1])
    #         PosY.append(PosY[-1] + 1)

    #     self.trajectory.setData(PosY, PosX)
        # self.curve2.setPos(self.ptr1, 0)

    def updateTrajectory(self, point):
        # 按逗号分割字符串
        point = point.split(',')
        # 将字符串转换为浮点数
        point = [float(i) for i in point]

        distance = (point[0]**2 + point[1]**2)**0.5
        if distance > 0.01:
            speed = distance * speedAmplitude
            cos = point[0] / distance
            sin = point[1] / distance
            PosX = speed * cos * t * monitorAmplitude
            PosY = speed * sin * t * monitorAmplitude
            self.trajectory.setData(PosX, PosY)
        else:
            self.trajectory.setData([0], [0])
        


    # # 数据左移
    # def update_data(self):
    #     self.data1[:-1] = self.data1[1:]
    #     self.data1[-1] = np.random.normal()
    #     # 数据填充到绘制曲线中
    #     self.curve2.setData(self.data1)
    #     # x 轴记录点
    #     self.ptr1 += 1
    #     # 重新设定 x 相关的坐标原点
    #

# https://blog.csdn.net/qq_39550025/article/details/126043789

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

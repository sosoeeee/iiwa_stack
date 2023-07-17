__author__ = 'Ted'

import sys
from PyQt5.Qt import *
from pyqtgraph import PlotWidget
from PyQt5 import QtCore
import numpy as np
import pyqtgraph as pq


PosX = [0]
PosY = [0]


class Window(QWidget):
    def __init__(self):
        super().__init__()
        # 设置下尺寸
        self.resize(600, 600)
        # 添加 PlotWidget 控件
        self.plotWidget_ted = PlotWidget(self)
        # 设置该控件尺寸和相对位置
        self.plotWidget_ted.setGeometry(QtCore.QRect(25, 25, 550, 550))
        # 设置 PlotWidget 控件的坐标轴范围
        self.plotWidget_ted.setXRange(-100, 100)
        self.plotWidget_ted.setYRange(-100, 100)
        # 设置 PlotWidget 控件的坐标轴分度值
        self.plotWidget_ted.setXTicks(np.linspace(-100, 100, 21))

        # 仿写 mode1 代码中的数据
        # 生成 300 个正态分布的随机数
        self.data1 = np.random.normal(size=300)

        self.curve2 = self.plotWidget_ted.plot([0], [0], pen=None, symbol='o', symbolSize=1, symbolPen='w', symbolBrush='w', name="mode2")
        self.ptr1 = 0

        # 设定定时器
        self.timer = pq.QtCore.QTimer()
        # 定时器信号绑定 update_data 函数
        self.timer.timeout.connect(self.update_data)
        # 定时器间隔50ms，可以理解为 50ms 刷新一次数据
        self.timer.start(50)

    def keyPressEvent(self, event):
        # print(event.key())
        if event.key() == Qt.Key_W:
            PosX.append(PosX[-1] + 1)
            PosY.append(PosY[-1])

        if event.key() == Qt.Key_S:
            PosX.append(PosX[-1] - 1)
            PosY.append(PosY[-1])

        if event.key() == Qt.Key_A:
            PosX.append(PosX[-1])
            PosY.append(PosY[-1] - 1)

        if event.key() == Qt.Key_D:
            PosX.append(PosX[-1])
            PosY.append(PosY[-1] + 1)

        self.curve2.setData(PosY, PosX)
        # self.curve2.setPos(self.ptr1, 0)



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


if __name__ == '__main__':
    # PyQt5 程序固定写法
    app = QApplication(sys.argv)

    # 将绑定了绘图控件的窗口实例化并展示
    window = Window()
    window.show()

    # PyQt5 程序固定写法
    sys.exit(app.exec())

import numpy as np
import matplotlib.pyplot as plt

class TrajectoryPrinter:
    def __init__(self, initZ):
        self.Zplane = initZ
        self.trajectorySet = []

        # 创建图窗
        self.fig = plt.figure()
        # 设置x,y标度一致
        plt.axis('equal')
        # 设置x名称
        plt.xlabel('x (m)')
        # 设置y名称
        plt.ylabel('y (m)')
        

    def loadTrajectory(self, trajectory, Label, color, Linestyle='-'):
        print(trajectory.shape)

        # 绘制轨迹并设置粗细
        plt.plot(trajectory[0, :], trajectory[1, :], color, linestyle=Linestyle, linewidth=1, label=Label)
        plt.legend(loc='upper right')


    def loadObstacle(self, obstacle):
        print(obstacle.shape) 

        x = obstacle[0]
        y = obstacle[1]
        z = obstacle[2]
        sphereRadius = obstacle[3]

        dz = self.Zplane  - z
        circleRadius = np.sqrt(sphereRadius**2 - dz**2)

         # 生成圆形轨迹
        theta = np.arange(0, 2*np.pi, 0.1)
        x = x + circleRadius * np.cos(theta)
        y = y + circleRadius * np.sin(theta)

        # 绘制障碍物
        plt.plot(x, y, 'grey', linewidth=1)
    
    def loadIntrestingPoint(self, point):
        plt.plot(point[:, 0], point[:, 1], 'r.', markersize=5)

    def print(self):
        plt.show()

class ForcePrinter:
    def __init__(self, totalLength, threshold):
        self.forceSet = []
        self.totalLength = totalLength
        self.threshold = threshold

        # 创建图窗
        self.fig = plt.figure()
        # 设置图窗窗口大小

        # 设置x名称
        plt.xlabel('time (s)')
        # 设置y名称
        plt.ylabel('force (N)')

        controllerFreq = 10
        self.t = np.arange(0, self.totalLength/controllerFreq, 1/controllerFreq)
        
    def loadForce(self, force, color, Label, Linestyle='-'):
        print(force.shape)

        # 绘制轨迹并设置粗细, 设置绘制在第二幅图
        plt.plot(self.t, force[:self.totalLength], color, linewidth=1, label=Label, linestyle=Linestyle)
        plt.legend(loc='upper right')

        # 绘制轨迹散点
        # plt.plot(self.t, force[:self.totalLength], color + '.', markersize=5)

    def loadChangePoint(self, index):
        # 绘制点
        plt.plot(self.t[index], self.threshold, 'r.', markersize=5)

    def loadThreshold(self):
        # 使用虚线绘制阈值
        plt.plot(self.t, np.ones(self.totalLength) * self.threshold, 'k', linestyle='--', linewidth=1)

    def print(self):
        plt.show()


def calculatePointError(trajectory, pointSet):
    amount = pointSet.shape[0]

    error = np.zeros(amount)

    for i in range(amount):
        error[i] = 999
        for j in range(trajectory.shape[1]):
            # print(trajectory[:2, j].T)
            # print(pointSet[i, :])
            error_ = np.linalg.norm(trajectory[:2, j].T - pointSet[i, :])
            if error_ < error[i]:
                error[i] = error_
    
    print(error)
    return error

# 返回轨迹改变的点的第一个索引(计算重规划开始的起点)
def checkTrajectoryDifference(trajectory1, trajectory2):
    amount = trajectory1.shape[1]

    for i in range(amount):
        if np.linalg.norm(trajectory1[:2, i].T - trajectory2[:2, i].T) > 0:
            return i
        


if __name__ == "__main__":
    #=========================================================# # 画轨迹图
    human = 3
    initZ = 0.20458
    trajectoryPrinter = TrajectoryPrinter(initZ)

    # # 绘制兴趣点
    intrestingPointSet = np.loadtxt("my_project\scripts\Data\intrestingPoint.txt")
    trajectoryPrinter.loadIntrestingPoint(intrestingPointSet)

    # # # 绘制原始轨迹
    # oriTrajectory = np.loadtxt("my_project\scripts\oriTraj.txt")
    # trajectoryPrinter.loadTrajectory(oriTrajectory, 'original','k', '--')

    # # # 绘制纯人控制的实际轨迹
    humanTrajectory = np.loadtxt("my_project\scripts\Data\\" + str(human) + "-allHuman-realTraj-2.txt")
    trajectoryPrinter.loadTrajectory(humanTrajectory, 'human leading', 'orange')

    # # # 绘制改变后的期望轨迹
    # index = 2
    # expectTrajectory1 = np.loadtxt("my_project\scripts\Data\\" + str(human) + "-replan-CHANGETraj-1-change-"+str(index)+".txt")
    # trajectoryPrinter.loadTrajectory(expectTrajectory1,'the second replanning', 'k', '--')

    # index = 1
    # expectTrajectory2 = np.loadtxt("my_project\scripts\Data\\" + str(human) + "-replan-CHANGETraj-1-change-"+str(index)+".txt")
    # trajectoryPrinter.loadTrajectory(expectTrajectory2, 'the first replanning', 'limegreen', '--')

    # # print(checkTrajectoryDifference(expectTrajectory1, expectTrajectory2))

    # # # 绘制算法辅助的实际轨迹
    replanTrajectory = np.loadtxt("my_project\scripts\Data\\" + str(human) + "-replan-realTraj-2.txt")
    trajectoryPrinter.loadTrajectory(replanTrajectory, 'shared control', 'b')

    # 绘制障碍物
    for i in range(3):
        obstacle = np.loadtxt("my_project\scripts\Data\obstacle" + str(i))
        trajectoryPrinter.loadObstacle(obstacle)

    trajectoryPrinter.print()

    #=========================================================# # 画交互力图——方案对比

    # forcePrinter = ForcePrinter(600, 3.7) 
    # force = np.loadtxt("my_project\scripts\Data\\" + str(human) + "-allHuman-forceSet-" + str(times+1) + ".txt")
    # forcePrinter.loadForce(force, 'orange', 'human leading')

    # force = np.loadtxt("my_project\scripts\Data\\" + str(human) + "-replan-forceSet-" + str(times) + ".txt")
    # forcePrinter.loadForce(force, 'b', 'shared control')

    # changeIndex = [131, 201, 231]
    # for index in changeIndex:
    #     forcePrinter.loadChangePoint(index)

    # forcePrinter.print()

    #=========================================================# # 画交互力图——对比一致性

    # for human in range(1, 5):
    #     fig = plt.figure()

    #     totalLength = 580
    #     threshold = 3.7
    #     controllerFreq = 10
    #     t = np.arange(0, totalLength/controllerFreq, 1/controllerFreq) 

    #     ax1 = plt.subplot(211)
    #     times = 1
    #     force = np.loadtxt("my_project\scripts\Data\\" + str(human) + "-allHuman-forceSet-" + str(times) + ".txt")
    #     add1 = np.sum(force)
    #     ax1.plot(t, force[:totalLength], 'orange', linewidth=1, label='human leading '+ str(times), linestyle='-')

    #     times = 2
    #     force = np.loadtxt("my_project\scripts\Data\\" + str(human) + "-allHuman-forceSet-" + str(times) + ".txt")
    #     add2 = np.sum(force)
    #     ax1.plot(t, force[:totalLength], 'orange', linewidth=1, label='human leading '+ str(times), linestyle='--')

    #     ax1.legend(loc='upper right')

    #     # 设置y名称
    #     ax1.set_ylabel('force (N)')

    #     ax2 = plt.subplot(212)
    #     times = 1
    #     force = np.loadtxt("my_project\scripts\Data\\" + str(human) + "-replan-forceSet-" + str(times) + ".txt")
    #     add3 = np.sum(force)
    #     ax2.plot(t, force[:totalLength], 'b', linewidth=1, label='shared control '+ str(times), linestyle='-')

    #     times = 2
    #     force = np.loadtxt("my_project\scripts\Data\\" + str(human) + "-replan-forceSet-" + str(times) + ".txt")
    #     add4 = np.sum(force)
    #     ax2.plot(t, force[:totalLength], 'b', linewidth=1, label='shared control '+ str(times), linestyle='--')

    #     ax2.plot(t, np.ones(totalLength) * threshold, 'k', linestyle='--', linewidth=1)

    #     ax2.legend(loc='upper right')

    #     ax2.set_xlabel('time (s)')
    #     ax2.set_ylabel('force (N)')

    #     print([add1, add2, add3, add4]) # 力数据求和比对

    #     # # 保存图片
    #     # fig.set_size_inches(10, 4)
    #     # plt.savefig("my_project\scripts\Data\\" + str(human) + "-forceContrast.png", dpi=300)

    #=========================================================# # 画交互力图——对比一致性

    # fig = plt.figure()
    # totalLength = 580
    # threshold = 3.7
    # controllerFreq = 10
    # t = np.arange(0, totalLength/controllerFreq, 1/controllerFreq) 

    # colorSet = ['orange', 'b', 'limegreen', 'r']
    # forceSUM = np.zeros(totalLength)

    # ax1 = plt.subplot(211)
    # for human in range(1, 5):
    #     times = 2
    #     force = np.loadtxt("my_project\scripts\Data\\" + str(human) + "-allHuman-forceSet-" + str(times) + ".txt")
    #     forceSUM += force[:totalLength]
    #     ax1.plot(t, force[:totalLength], colorSet[human-1], linewidth=1, label='human'+ str(human), linestyle='dotted')

    # ax1.plot(t, forceSUM/4, 'k', linewidth=1, label='mean', linestyle='-')

    # ax1.legend(loc='upper right')
    # ax1.set_ylabel('force (N)')
    # ax1.set_xlabel('time (s)')

    # forceSUM = np.zeros(totalLength)
    # ax2 = plt.subplot(212)
    # for human in range(1, 5):
    #     times = 2
    #     force = np.loadtxt("my_project\scripts\Data\\" + str(human) + "-replan-forceSet-" + str(times) + ".txt")
    #     forceSUM += force[:totalLength]
    #     ax2.plot(t, force[:totalLength], colorSet[human-1], linewidth=1, label='human'+ str(human), linestyle='dotted')

    # # ax2.plot(t, forceSUM/4, 'k', linewidth=1, label='sum', linestyle='-')
    # ax2.plot(t, np.ones(totalLength) * threshold, 'k', linestyle='--', linewidth=1)

    # ax2.legend(loc='upper right')     
    # ax2.set_ylabel('force (N)')
    # ax2.set_xlabel('time (s)')  
    
    # plt.show()

    # # 保存图片
    # fig.set_size_inches(10, 4)
    # plt.savefig("my_project\scripts\Data\\" + str(human) + "-forceContrast.png", dpi=300)

    #=========================================================# # 计算兴趣点误差

    # intrestingPointSet = np.loadtxt("my_project\scripts\Data\intrestingPoint.txt")

    # labels = ['Point1', 'Point2', 'Point3', 'Point4']
    # x = np.arange(len(labels))  # the label locations
    # width = 0.1  # the width of the bars

    # for human in range(1, 5):
    #     fig, ax = plt.subplots()    

    #     for times in range(1, 3):
    #         humanTrajectory = np.loadtxt("my_project\scripts\Data\\" + str(human) + "-allHuman-realTraj-" + str(times) + ".txt")
    #         ax.bar(x - width/2 - (times-1)*width, calculatePointError(humanTrajectory, intrestingPointSet), width, label='human leading '+ str(times))
        
    #     for times in range(1, 3):
    #         replanTrajectory = np.loadtxt("my_project\scripts\Data\\" + str(human) + "-replan-realTraj-" + str(times) + ".txt")
    #         ax.bar(x + width/2 + (times-1)*width, calculatePointError(replanTrajectory, intrestingPointSet), width, label='shared control '+ str(times))
        
    #     # Add some text for labels, title and custom x-axis tick labels, etc.
    #     ax.set_ylabel('error (m)')
    #     ax.set_title('Error of different points')
    #     ax.set_xticks(x)
    #     ax.set_xticklabels(labels)
    #     ax.legend(loc='upper right')

    #     # 保存图片
    #     fig.set_size_inches(10, 4)
    #     plt.savefig("my_project\scripts\Data\\" + str(human) + "-pointError.png", dpi=300)
    
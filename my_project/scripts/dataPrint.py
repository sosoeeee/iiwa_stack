import numpy as np
import matplotlib.pyplot as plt

class TrajectoryPrinter:
    def __init__(self, initZ):
        self.Zplane = initZ

        # 创建图窗
        self.fig = plt.figure()
        # 设置x,y标度一致
        plt.axis('equal')
        # 设置x名称
        plt.xlabel('x (m)')
        # 设置y名称
        plt.ylabel('y (m)')
        

    def loadTrajectory(self, trajectory, Label, color, Linestyle='-', endIndex=-1, startIndex=1):
        if trajectory.shape[1] == 3:
            trajectory = trajectory.T

        print(trajectory.shape)

        # 绘制轨迹并设置粗细
        plt.plot(trajectory[0, startIndex:endIndex], trajectory[1, startIndex:endIndex], color, linestyle=Linestyle, linewidth=1, label=Label)
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
        theta = np.arange(0, 2*np.pi + 0.1, 0.1)
        x = x + circleRadius * np.cos(theta)
        y = y + circleRadius * np.sin(theta)

        # 绘制障碍物
        plt.plot(x, y, 'grey', linewidth=1)
    
    def loadIntrestingPoint(self, point):
        plt.plot(point[:, 0], point[:, 1], 'r.', markersize=6)

    def loadReplanPoint(self, trajectory, index):
        # 绘制点
        replanLen = 250 
        # 使用星形绘制改变点
        plt.plot(trajectory[0][index], trajectory[1][index], 'black', marker='*', markersize=10)
        plt.plot(trajectory[0][index + replanLen], trajectory[1][index + replanLen], 'black', marker='*', markersize=10)

        controllerFreq = 10
        time = index / controllerFreq
        time = 'time: ' + str(time) + 's'
        plt.text(trajectory[0][index] - 0.16, trajectory[1][index], time, fontsize=8)
    
    def print(self):
        plt.show()
    
    def save(self, fileName):
        plt.savefig('my_project\scripts\pic\\' + fileName + '.pdf')

class ForcePrinter:
    def __init__(self, totalLength, threshold):
        self.forceSet = []
        self.totalLength = totalLength
        self.threshold = threshold

        # 创建图窗
        self.fig = plt.figure()
        # 设置图窗窗口大小

        # 设置x名称
        # plt.xlabel('time (s)')
        # 设置y名称
        # plt.ylabel('force (N)')

        self.controllerFreq = 10
        self.t = np.arange(0, self.totalLength/self.controllerFreq, 1/self.controllerFreq)
        
    def loadForce(self, force, color, Label, Linestyle='-'):
        print(force.shape)

        if force.shape[1] == 2:
            force = np.sqrt(force[:, 0] ** 2 + force[:, 1] ** 2)

        # 绘制轨迹并设置粗细, 设置绘制在第二幅图
        plt.plot(self.t, force[:self.totalLength], color, linewidth=1, label=Label, linestyle=Linestyle)
        plt.legend(loc='upper right')

        # 绘制轨迹散点
        # plt.plot(self.t, force[:self.totalLength], color + '.', markersize=5)

    def loadBackgroud(self, startIndex, endIndex):
        # 绘制背景
        plt.axvspan(startIndex/self.controllerFreq, endIndex/self.controllerFreq, facecolor='grey', alpha=0.2)

    def loadChangePoint(self, index):
        # 绘制点
        plt.plot(self.t[index], self.threshold, 'black', marker='*', markersize=10)

        # 标记时间
        time = index / self.controllerFreq
        time = 'time: ' + str(time) + 's'

        if index == 201:
            compensation = 0.2
        else:
            compensation = 0

        plt.text(self.t[index] - 1, self.threshold + 0.3 + compensation, time, fontsize=8)

    def loadThreshold(self):
        # 使用虚线绘制阈值
        plt.plot(self.t, np.ones(self.totalLength) * self.threshold, 'k', linestyle='--', linewidth=1)
        # 文字标记
        plt.text(0, self.threshold + 0.1, str(self.threshold) + 'N', fontsize=8)

    def print(self):
        plt.show()
    
    def save(self, fileName):
        plt.savefig('my_project\scripts\pic\\' + fileName + '.pdf')



def calculatePointError(trajectory, pointSet):
    amount = pointSet.shape[0]

    error = np.zeros(amount)
    pointIndex = np.zeros(amount, int)

    for i in range(amount):
        error[i] = 999
        for j in range(trajectory.shape[1]):
            # print(trajectory[:2, j].T)
            # print(pointSet[i, :])
            error_ = np.linalg.norm(trajectory[:2, j].T - pointSet[i, :])
            if error_ < error[i]:
                error[i] = error_
                pointIndex[i] = j
    
    # print(error)
    return error, pointIndex

# 返回轨迹改变的点的第一个索引(计算重规划开始的起点)
def checkTrajectoryDifference(trajectory1, trajectory2):
    amount = trajectory1.shape[1]

    for i in range(amount):
        if np.linalg.norm(trajectory1[:2, i].T - trajectory2[:2, i].T) > 0:
            return i

def checkTrajectoryDifferenceALL(trajectory1, trajectory2):
    amount = trajectory1.shape[1]
    indexSet = []

    for i in range(1, amount):
        if 0.005 < np.linalg.norm(trajectory1[:2, i].T - trajectory2[:2, i].T)<0.02:
            indexSet.append(i)

    print(indexSet)    

    return indexSet

def getTrajectoryDifferenceALL(trajectory1, trajectory2):
    amount = trajectory1.shape[1]
    dif = np.zeros(amount)

    for i in range(amount):
        dif[i] = np.linalg.norm(trajectory1[:2, i].T - trajectory2[:2, i].T) 

    return dif

def sampleVariance(group):
    n = group.shape[0] # 样本数

    group = group.reshape(n, -1)

    # 计算均值
    mean = np.mean(group, axis=0)

    # 计算方差
    sum = 0
    for i in range(n):
        sum += (group[i, :] - mean)**2

    return sum / (n-1)

def OneFactorANOVA(group1, group2):
    if group1.shape != group2.shape:
        print("error: group1.shape != group2.shape")
        return

    n = group1.shape[0] # 样本数

    # 计算均值
    mean1 = np.mean(group1)
    mean2 = np.mean(group2)

    MSB = (mean1 - mean2)**2 / 2 * n # 组间均方

    # 计算方差
    var1 = sampleVariance(group1)
    var2 = sampleVariance(group2)

    MSE = (var1 + var2) / 2 # 组内均方

    print("Numerator df", 1)
    print("Denominator df", 2*(n-1))

    return MSB / MSE


if __name__ == "__main__":
    #=========================================================# # 画轨迹图
    # human = 5
    # initZ = 0.20458
    # trajectoryPrinter = TrajectoryPrinter(initZ)

    # 绘制兴趣点
    # intrestingPointSet = np.loadtxt("my_project\scripts\Data\intrestingPoint.txt")
    # trajectoryPrinter.loadIntrestingPoint(intrestingPointSet)

    # 绘制纯人控制的实际轨迹
    # humanTrajectory = np.loadtxt("my_project\scripts\Data\\" + str(human) + "-allHuman-realTraj-1.txt")

    # 绘制原始轨迹
    # oriTrajectory = np.loadtxt("my_project\scripts\oriTraj.txt")
    # trajectoryPrinter.loadTrajectory(oriTrajectory, 'original','k', '--')
    # trajectoryPrinter.loadTrajectory(humanTrajectory, 'master-slave', 'orange','--')

    # # 绘制改变后的期望轨迹
    # index = 2
    # expectTrajectory1 = np.loadtxt("my_project\scripts\Data\\" + str(human) + "-replan-CHANGETraj-1-change-"+str(index)+".txt")
    # trajectoryPrinter.loadTrajectory(expectTrajectory1,'the second replanning', 'k', '--')

    # index = 3
    # expectTrajectory2 = np.loadtxt("my_project\scripts\Data\\" + str(human) + "-replan-CHANGETraj-1-change-"+str(index)+".txt")
    # trajectoryPrinter.loadTrajectory(expectTrajectory2, 'the third replanning', 'limegreen', '--')

    # print(checkTrajectoryDifference(expectTrajectory1, expectTrajectory2))
# 
    # replanPoint = checkTrajectoryDifference(expectTrajectory1, expectTrajectory2)
    # print(replanPoint)
    # trajectoryPrinter.loadReplanPoint(expectTrajectory1, replanPoint)

    # # 绘制算法辅助的实际轨迹
    # replanTrajectory = np.loadtxt("my_project\scripts\Data\\" + str(human) + "-replan-realTraj-1.txt")
    # trajectoryPrinter.loadTrajectory(replanTrajectory, 'real-time', 'b','-', replanPoint)

    # 绘制时间戳
    # shared [81, 153, 213, 471]
    # ml [82, 179, 261, 546]
    # controllerFreq = 10
    # timeStampIndex = checkTrajectoryDifferenceALL(humanTrajectory.T, oriTrajectory)
    # timeStampIndex = [81, 153, 213, 471]
    # for index in timeStampIndex:
    #     plt.plot(replanTrajectory.T[0][index], replanTrajectory.T[1][index], 'black', marker='.', markersize=6)
    #     time = index / controllerFreq
    #     time = 'time: ' + str(time) + 's'

    #     if index == 81:
    #         plt.text(oriTrajectory[0][index] - 0.1, oriTrajectory[1][index] + 0.02, time, fontsize=8)
    #     elif index == 471:
    #         plt.text(oriTrajectory[0][index] + 0.02, oriTrajectory[1][index] + 0.02, time, fontsize=8)
    #     else:
    #         plt.text(oriTrajectory[0][index] - 0.12, oriTrajectory[1][index], time, fontsize=8)

        # if index == 82:
        #     plt.text(oriTrajectory[0][index] - 0.1, oriTrajectory[1][index] + 0.02, time, fontsize=8)
        # else:
        #     plt.text(oriTrajectory[0][index] - 0.12, oriTrajectory[1][index], time, fontsize=8)
            
            

        # plt.text(oriTrajectory[0][index] - 0.12, oriTrajectory[1][index], time, fontsize=8)



    # 绘制障碍物
    # for i in range(3):
    #     obstacle = np.loadtxt("my_project\scripts\Data\obstacle" + str(i))
    #     trajectoryPrinter.loadObstacle(obstacle)

    # trajectoryPrinter.print()
    # trajectoryPrinter.save("replan-3")

    #=========================================================# # PSI测试
    human = 7
    initZ = 0.20458
    # trajectoryPrinter = TrajectoryPrinter(initZ)

    intrestingPointSet = np.array([[-0.57, -0.005]])
    # trajectoryPrinter.loadIntrestingPoint(intrestingPointSet)

    startIndex = 100
    endIndex = 300

    oriTrajectory = np.loadtxt("my_project\scripts\oriTraj.txt")
    # trajectoryPrinter.loadTrajectory(oriTrajectory, 'original','k', '-', endIndex, startIndex)

    ConstantPSI = np.loadtxt("my_project\scripts\Data\\" + str(human) + "-replan-realTraj-1.txt")
    # trajectoryPrinter.loadTrajectory(ConstantPSI, 'Constant PSI', '#8f17da','--', endIndex, startIndex)

    variablePSI = np.loadtxt("my_project\scripts\Data\\" + str(human) + "-replan-realTraj-2.txt")
    # trajectoryPrinter.loadTrajectory(variablePSI, 'Variable PSI', '#62da17','--', endIndex, startIndex)

    #  绘制障碍物
    # obstacle = np.loadtxt("my_project\scripts\Data\obstacle_PSI" + str(0))
    # trajectoryPrinter.loadObstacle(obstacle)

    # 设置label位置
    # plt.legend(loc='upper left')

    # trajectoryPrinter.print()
    # trajectoryPrinter.save("PSITrajectory")

    # 绘制力曲线
    forcePrinter = ForcePrinter(endIndex - startIndex - 1, 3.7)

    plt.subplot(311)

    plt.subplots_adjust(bottom=0.15)
    plt.subplots_adjust(hspace=0.55)

    force_ConstantPSI = np.loadtxt("my_project\scripts\Data\\" + str(human) + "-replan-forceSet-" + str(1) + ".txt")
    force_ConstantPSI = force_ConstantPSI[startIndex:endIndex]
    forcePrinter.loadForce(force_ConstantPSI, '#8f17da', 'Constant PSI')

    force_variablePSI = np.loadtxt("my_project\scripts\Data\\" + str(human) + "-replan-forceSet-" + str(2) + ".txt")
    force_variablePSI = force_variablePSI[startIndex:endIndex]
    forcePrinter.loadForce(force_variablePSI, '#62da17', 'Variable PSI')

    forcePrinter.loadThreshold()

    # 设置label位置
    # plt.legend(loc='lower right')
    # 不设置label
    plt.legend().set_visible(False)

    plt.ylim(-0.5, 5)
    plt.ylabel('force (N)')
    plt.xlim(-0.5, 20.5)

    # 图表下方插入(a)
    plt.text(10, -2.7, '(a)', horizontalalignment='center', verticalalignment='center', fontsize=11)

    # forcePrinter.print()

    totalLength = endIndex - startIndex 
    controllerFreq = 10
    t = np.arange(0, totalLength/controllerFreq, 1/controllerFreq)

    # 绘制轨迹偏离曲线
    plt.subplot(312)
    oriTrajectory = oriTrajectory[:, startIndex:endIndex]
    ConstantPSI = ConstantPSI[startIndex:endIndex, :]
    variablePSI = variablePSI[startIndex:endIndex, :]

    dif_ConstantPSI = getTrajectoryDifferenceALL(oriTrajectory, ConstantPSI.T)
    dif_variablePSI = getTrajectoryDifferenceALL(oriTrajectory, variablePSI.T)

    plt.plot(t, dif_ConstantPSI * 1000, '#8f17da', linewidth=1, label='Constant PSI', linestyle='-')
    plt.plot(t, dif_variablePSI * 1000, '#62da17', linewidth=1, label='Variable PSI', linestyle='-')

    # 设置label位置
    # plt.legend(loc='upper left')
    plt.ylabel('deviation (mm)')
    plt.ylim(-5, 105)
    plt.yticks(np.arange(0, 100.5, 50))
    plt.xlim(-0.5, 20.5)
    # plt.show()

    plt.text(10, -50.5, '(b)', horizontalalignment='center', verticalalignment='center', fontsize=11)

    # 绘制PSI曲线
    plt.subplot(313)

    PSI_variablePSI = np.loadtxt("my_project\scripts\Data\\" + str(human) + "-replan-PSISet-" + str(2) + ".txt")
    PSI_variablePSI = PSI_variablePSI[startIndex:endIndex]
    plt.plot(t, PSI_variablePSI, '#62da17', linewidth=1, label='Variable PSI', linestyle='-')

    PSI_ConstantPSI = np.loadtxt("my_project\scripts\Data\\" + str(human) + "-replan-PSISet-" + str(1) + ".txt")
    PSI_ConstantPSI = PSI_ConstantPSI[startIndex:endIndex]
    plt.plot(t, PSI_ConstantPSI, '#8f17da', linewidth=1, label='Constant PSI', linestyle='-')

    plt.legend(loc='lower left')
    plt.ylim(-0.2, 1.2)
    plt.ylabel('PSI')
    plt.xlim(-0.5, 20.5)

    plt.text(10, -0.78, '(c)', horizontalalignment='center', verticalalignment='center', fontsize=11)
    plt.text(10, -1.08, 'time (s)', horizontalalignment='center', verticalalignment='center', fontsize=11)

    # plt.xlabel('time (s)')
    # plt.show()

    # plt.savefig('my_project\scripts\pic\\' + "PSI" + '.pdf')

    #=========================================================# # 画交互力图——方案对比
    human = 6
    times = 1
    forcePrinter = ForcePrinter(600, 3.7) 
    plt.subplot(3, 1, 2)
    force = np.loadtxt("my_project\scripts\Data\\" + str(human) + "-allHuman-forceSet-" + str(times) + ".txt")
    forcePrinter.loadForce(force, 'orange', 'master-slave')
    forcePrinter.loadBackgroud(82, 179)
    force1Sum = np.sum(force[82:179])
    forcePrinter.loadBackgroud(261, 546)
    force2Sum = np.sum(force[261:546])
    plt.ylabel('force (N)')
    plt.ylim(-0.2, 5.5)
    plt.xlim(-2, 62)

    plt.legend(loc='right')
    
    plt.text(30, -2.5, '(b)', horizontalalignment='center', verticalalignment='center', fontsize=11)

    plt.subplot(3, 1, 3)
    force = np.loadtxt("my_project\scripts\Data\\" + str(human) + "-replan-forceSet-" + str(times) + ".txt")
    forcePrinter.loadForce(force, 'b', 'shared control')
    forcePrinter.loadThreshold()
    forcePrinter.loadBackgroud(81, 153)
    force1Sum_share = np.sum(force[81:153])
    forcePrinter.loadBackgroud(213, 471)
    force2Sum_share = np.sum(force[213:471])
    plt.legend(loc='right')
    # 设置y轴范围
    plt.ylim(-0.2, 5.5)
    # 设置x名称
    # plt.xlabel('time (s)')
    # 设置y名称
    plt.ylabel('force (N)')
    plt.xlim(-2, 62)

    plt.subplots_adjust(bottom=0.2)

    plt.text(30, -2.5, '(c)', horizontalalignment='center', verticalalignment='center', fontsize=11)

    plt.text(30, -3.8, 'time (s)', horizontalalignment='center', verticalalignment='center', fontsize=11)

    forcePrinter.loadChangePoint(131)
    forcePrinter.loadChangePoint(201)
    forcePrinter.loadChangePoint(231)

    print("force sum:", force1Sum, force2Sum, force1Sum_share, force2Sum_share)
    print("1:", abs(force1Sum_share - force1Sum) / force1Sum) # 21.51%
    print("2:", abs(force2Sum_share - force2Sum) / force2Sum) # 52.83%


    force = np.loadtxt("my_project\scripts\Data\\" + str(human) + "-replan-forceSet-" + str(times) + ".txt")
    forcePrinter.loadForce(force, 'b', 'shared control')

    changeIndex = [131, 201, 231]
    for index in changeIndex:
        forcePrinter.loadChangePoint(index)

    forcePrinter.print()
    # forcePrinter.save("replan-force")

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

        # # 保存图片
        # fig.set_size_inches(10, 4)
        # plt.savefig("my_project\scripts\Data\\" + str(human) + "-forceContrast.png", dpi=300)

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

    # # ax1.plot(t, forceSUM/4, 'k', linewidth=1, label='mean', linestyle='-')

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
    
    # # plt.show()

    # # 保存图片
    # fig.set_size_inches(10, 4)
    # plt.savefig("my_project\scripts\Data\\" + str(human) + "-forceContrast.png", dpi=300)

    #=========================================================# # 计算兴趣点折线图
    # human = 6
    # times = 1
    # forcePrinter = ForcePrinter(600, 3.7) 

    # intrestingPointSet = np.loadtxt("my_project\scripts\Data\intrestingPoint.txt")
    # humanTrajectory = np.loadtxt("my_project\scripts\Data\\" + str(human) + "-allHuman-realTraj-1.txt")
    # replanTrajectory = np.loadtxt("my_project\scripts\Data\\" + str(human) + "-replan-realTraj-1.txt")
    # humanErr, humanIndex = calculatePointError(humanTrajectory.T, intrestingPointSet)
    # shareErr, shareIndex = calculatePointError(replanTrajectory.T, intrestingPointSet)
    # print(humanErr, humanIndex)
    # print(shareErr, shareIndex)

    # # 绘制折线图
    # plt.subplot(3, 1, 1)
    # # fig = plt.figure()
    # plt.plot(humanErr * 1000, 'orange', linewidth=1, label='master-slave', linestyle='-')
    # plt.plot(shareErr * 1000, 'b', linewidth=1, label='shared control', linestyle='-')
    # plt.legend(loc='upper left', ncol = 1)
    # plt.ylabel('error (mm)')

    # # 绘制点
    # pointIndex = [0, 1,2,3]
    # plt.plot(pointIndex, humanErr[pointIndex] * 1000, 'orange', marker='.', markersize=5)
    # plt.plot(pointIndex, shareErr[pointIndex] * 1000, 'b', marker='.', markersize=5)

    # # 设置y轴范围,分度值为5, 显示0、5、10，但是从-0.2开始
    # plt.ylim(-0.2, 10.2)
    # plt.yticks(np.arange(0, 10.1, 5))
    # # 设置x轴显示1，2，3，4
    # plt.xticks(pointIndex, ('Point1', 'Point2', 'Point3', 'Point4'))

    # # 图表下方插入
    # plt.subplots_adjust(hspace=0.55)

    # # 图表下方插入（a）
    # plt.text(1.5, -3.6, '(a)', horizontalalignment='center', verticalalignment='center', fontsize=11)

    # plt.subplot(3, 1, 2)
    # force = np.loadtxt("my_project\scripts\Data\\" + str(human) + "-allHuman-forceSet-" + str(times) + ".txt")
    # forcePrinter.loadForce(force, 'orange', 'master-slave')
    # forcePrinter.loadBackgroud(82, 179)
    # force1Sum = np.sum(force[82:179])
    # forcePrinter.loadBackgroud(261, 546)
    # force2Sum = np.sum(force[261:546])
    # plt.ylabel('force (N)')
    # plt.ylim(-0.2, 5.5)
    # plt.yticks(np.arange(0, 5.6, 2.5))
    # plt.xlim(-2, 62)

    # plt.legend(loc='lower right')
    
    # plt.text(30, -2.5, '(b)', horizontalalignment='center', verticalalignment='center', fontsize=11)

    # plt.subplot(3, 1, 3)
    # force = np.loadtxt("my_project\scripts\Data\\" + str(human) + "-replan-forceSet-" + str(times) + ".txt")
    # forcePrinter.loadForce(force, 'b', 'shared control')
    # forcePrinter.loadThreshold()
    # forcePrinter.loadBackgroud(81, 153)
    # force1Sum_share = np.sum(force[81:153])
    # forcePrinter.loadBackgroud(213, 471)
    # force2Sum_share = np.sum(force[213:471])
    # plt.legend(loc='lower right')
    # # 设置y轴范围
    # plt.ylim(-0.2, 5.5)
    # plt.yticks(np.arange(0, 5.6, 2.5))
    # # 设置x名称
    # # plt.xlabel('time (s)')
    # # 设置y名称
    # plt.ylabel('force (N)')
    # plt.xlim(-2, 62)

    # plt.subplots_adjust(bottom=0.2)

    # plt.text(30, -2.5, '(c)', horizontalalignment='center', verticalalignment='center', fontsize=11)

    # plt.text(30, -3.8, 'time (s)', horizontalalignment='center', verticalalignment='center', fontsize=11)

    # # plt.show()
    # plt.savefig("my_project\scripts\pic\\" + "contrast" + '.pdf')

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

    #=========================================================# # 单因素方差分析
    # intrestingPointSet = np.loadtxt("my_project\scripts\Data\intrestingPoint.txt")

    # # Human
    # human = 5
    # times = 1
    # humanTrajectory = np.loadtxt("my_project\scripts\Data\\" + str(human) + "-allHuman-realTraj-" + str(times) + ".txt")
    # HumanPointErrorSet = calculatePointError(humanTrajectory, intrestingPointSet)
    # for human in range(1, 5):
    #     for times in range(1, 3):
    #         humanTrajectory = np.loadtxt("my_project\scripts\Data\\" + str(human) + "-allHuman-realTraj-" + str(times) + ".txt")
    #         HumanPointErrorSet = np.vstack((HumanPointErrorSet, calculatePointError(humanTrajectory, intrestingPointSet)))

    # # 计算均值与方差
    # humanMean = np.mean(HumanPointErrorSet, axis=0)
    # humanVar = sampleVariance(HumanPointErrorSet)

    # # Shared control    
    # human = 5
    # times = 1
    # replanTrajectory = np.loadtxt("my_project\scripts\Data\\" + str(human) + "-replan-realTraj-" + str(times) + ".txt")
    # SharedPointErrorSet = calculatePointError(replanTrajectory, intrestingPointSet)
    # for human in range(1, 5):
    #     for times in range(1, 3):
    #         replanTrajectory = np.loadtxt("my_project\scripts\Data\\" + str(human) + "-replan-realTraj-" + str(times) + ".txt")
    #         SharedPointErrorSet = np.vstack((SharedPointErrorSet, calculatePointError(replanTrajectory, intrestingPointSet)))

    # # 计算均值与方差
    # sharedMean = np.mean(SharedPointErrorSet, axis=0)
    # sharedVar = sampleVariance(SharedPointErrorSet)

    # labels = ['Point1', 'Point2', 'Point3', 'Point4']
    # x = np.arange(len(labels))  # the label locations
    # width = 0.1  # the width of the bars
    # fig, ax = plt.subplots()    
    
    # ax.bar(x - width/2, humanMean, width, label='human leading', error_kw=dict(ecolor='gray', lw=1, capsize=3, capthick=1))
    # ax.bar(x + width/2, sharedMean, width, label='shared control', error_kw=dict(ecolor='gray', lw=1, capsize=3, capthick=1))

    # # Add some text for labels, title and custom x-axis tick labels, etc.
    # ax.set_ylabel('error (m)')
    # ax.set_title('Error of different points')
    # ax.set_xticks(x)
    # ax.set_xticklabels(labels)
    # ax.legend(loc='upper right')
    
    # plt.show()

    # F_Point1 = OneFactorANOVA(HumanPointErrorSet[:, 0], SharedPointErrorSet[:, 0])
    # F_Point2 = OneFactorANOVA(HumanPointErrorSet[:, 1], SharedPointErrorSet[:, 1])
    # F_Point3 = OneFactorANOVA(HumanPointErrorSet[:, 2], SharedPointErrorSet[:, 2])
    # F_Point4 = OneFactorANOVA(HumanPointErrorSet[:, 3], SharedPointErrorSet[:, 3])

    # # F>4.5 即可在95%置信度下认为两组数据有显著差异

    # print([F_Point1, F_Point2, F_Point3, F_Point4])
    
    # #=========================================================# # 交互力的方差分析
    # controllerFreq = 10 
    # avrSpeed = 0.01
    # distance = 0.1

    # # human
    # human = 5
    # times = 1
    # force = np.loadtxt("my_project\scripts\Data\\" + str(human) + "-allHuman-forceSet-" + str(times) + ".txt")
    # energy = np.sum(force[:580])
    # HumanEnergySet = energy

    # for human in range(1, 5):
    #     for times in range(1, 3):
    #         force = np.loadtxt("my_project\scripts\Data\\" + str(human) + "-allHuman-forceSet-" + str(times) + ".txt")
    #         force = force[:580]
    #         energy = np.sum(force)
    #         HumanEnergySet = np.vstack((HumanEnergySet, energy))
    
    # print(HumanEnergySet.shape)
    # # 计算均值与方差
    # humanMean = np.mean(HumanEnergySet, axis=0)
    # humanVar = sampleVariance(HumanEnergySet)
    # # print(humanMean, humanVar)

    # # shared control    
    # human = 5
    # times = 1
    # force = np.loadtxt("my_project\scripts\Data\\" + str(human) + "-replan-forceSet-" + str(times) + ".txt")
    # SharedEnergySet = np.sum(force[:580])    

    # for human in range(1, 5):
    #     for times in range(1, 3):
    #         force = np.loadtxt("my_project\scripts\Data\\" + str(human) + "-replan-forceSet-" + str(times) + ".txt")
    #         force = force[:580]
    #         energy = np.sum(force)
    #         SharedEnergySet = np.vstack((SharedEnergySet, energy))
    
    # # 计算均值与方差
    # sharedMean = np.mean(SharedEnergySet, axis=0)
    # sharedVar = sampleVariance(SharedEnergySet)
    # # print(sharedMean, sharedVar)

    # labels = ['huamn leading', 'shared control']
    # x = np.arange(len(labels))  # the label locations
    # width = 0.2  # the width of the bars
    # fig, ax = plt.subplots()

    # ax.bar(x - width/2, humanMean, width, label='human leading')
    # ax.bar(x + width/2, sharedMean, width, label='shared control')

    # # Add some text for labels, title and custom x-axis tick labels, etc.
    # ax.set_ylabel('? ? ? ?')
    # # ax.set_title('Force of different control methods')
    # ax.set_xticks(x)
    # ax.set_xticklabels(labels)
    # ax.legend(loc='upper right')

    # plt.show()

    # F = OneFactorANOVA(HumanEnergySet, SharedEnergySet)
    # print(F)
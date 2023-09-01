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
        

    def loadTrajectory(self, trajectory, color):
        print(trajectory.shape)

        # 绘制轨迹并设置粗细
        plt.plot(trajectory[0, :], trajectory[1, :], color, linewidth=1)


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
        plt.plot(x, y, 'r', linewidth=1)
    
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

        self.t = np.arange(0, totalLength, 1)
        # 使用虚线绘制阈值
        plt.plot(self.t, np.ones(totalLength) * threshold, 'r--', linewidth=1)
        
    def loadForce(self, force, color):
        print(force.shape)

        # 绘制轨迹并设置粗细
        plt.plot(self.t, force[:self.totalLength], color, linewidth=1)

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


if __name__ == "__main__":
    
    #=========================================================# # 画轨迹图
    # initZ = 0.20458
    # trajectoryPrinter = TrajectoryPrinter(initZ)

    # intrestingPointSet = np.loadtxt("my_project\scripts\Data\intrestingPoint.txt")
    # trajectoryPrinter.loadIntrestingPoint(intrestingPointSet)

    # oriTrajectory = np.loadtxt("my_project\scripts\oriTraj.txt")
    # trajectoryPrinter.loadTrajectory(oriTrajectory, 'black')

    # # humanTrajectory = np.loadtxt("my_project\scripts\Data\\0-allHuman-realTraj.txt")
    # # trajectoryPrinter.loadTrajectory(humanTrajectory, 'g')

    # replanTrajectory = np.loadtxt("my_project\scripts\Data\\0-replan-realTraj.txt")
    # trajectoryPrinter.loadTrajectory(replanTrajectory, 'b')

    # for i in range(3):
    #     obstacle = np.loadtxt("my_project\scripts\Data\obstacle" + str(i))
    #     trajectoryPrinter.loadObstacle(obstacle)

    # trajectoryPrinter.print()

    #=========================================================# # 画交互力图

    # forcePrinter = ForcePrinter(600, 3.7)
    # force = np.loadtxt("my_project\scripts\Data\\0-allHuman-forceSet.txt")
    # forcePrinter.loadForce(force, 'black')

    # force = np.loadtxt("my_project\scripts\Data\\0-replan-forceSet.txt")
    # forcePrinter.loadForce(force, 'b')

    # forcePrinter.print()

    #=========================================================# # 计算兴趣点误差

    intrestingPointSet = np.loadtxt("my_project\scripts\Data\intrestingPoint.txt")
    # Trajectory = np.loadtxt("my_project\scripts\oriTraj.txt")
    # Trajectory = np.loadtxt("my_project\scripts\Data\\0-allHuman-realTraj.txt")
    Trajectory = np.loadtxt("my_project\scripts\Data\\0-replan-realTraj.txt")
    calculatePointError(Trajectory, intrestingPointSet)
    
import numpy as np


class PathPlanner:
    def __init__(self, startPoint, endPoint):
        # 起点和终点
        # 检查起点和终点是否为3*1
        if startPoint.shape != (3, 1) or endPoint.shape != (3, 1):
            raise Exception('The shape of startPoint and endPoint must be (3, 1).')

        self.startPoint = startPoint
        self.endPoint = endPoint

        # 障碍物默认均为球星
        self.obstacle = []

        # RRT搜索步长
        self.step = 0.01
        # RRT最大迭代步数
        self.maxIterNum = 100000
        # 目标阈值
        self.targetThreshold = 0.01
        # 搜索空间尺寸
        self.searchSpace = np.array([[-0.3, 0.3], [-1, 1], [0, 0]])  # 二维运动z轴方向搜索空间为0 ## 只考虑二维情况
        self.searchSpace = self.searchSpace.reshape((3, 2))
        # 贪婪搜索概率
        self.greedyProb = 0.3

        # 路径的安全距离
        self.safeDistance = 0.03

        # 路径
        self.path = []

    def addObstacle(self, center, radius):
        # 添加障碍物
        # center：障碍物中心点
        # radius：障碍物半径

        # 检查圆心是否为3*1
        if center.shape != (3, 1):
            raise Exception('The shape of center must be (3, 1).')

        obstacle = {
            'center': center,
            'radius': radius
        }

        self.obstacle.append(obstacle)

    def pointCollisionDetection(self, point):
        # 检测点是否与障碍物相交
        # 返回值：True——相交，False——不相交

        for i in range(len(self.obstacle)):
            
            # # DEBUG
            # print("-------------------")
            # print('obstacle %d' % i)
            # print("point is")
            # print(point)
            # print("center is")
            # print(self.obstacle[i]['center'])
            # print("distance: ", np.linalg.norm(point - self.obstacle[i]['center']))
            # print("radius", self.obstacle[i]['radius'])

            if np.linalg.norm(point - self.obstacle[i]['center']) < self.obstacle[i]['radius'] + self.safeDistance:
                return True

        return False

    def collisionDetection(self, startPoint, endPoint):
        # 检测路径是否与障碍物相交
        # 返回值：True——相交，False——不相交

        if startPoint.shape != (3, 1) or endPoint.shape != (3, 1):
            raise Exception('The shape of startPoint and endPoint must be (3, 1).')

        checkVector = endPoint - startPoint
        checkStep = self.step / 5
        checkNum = int(np.linalg.norm(checkVector) / checkStep)

        for i in range(checkNum):
            checkPoint = startPoint + checkVector * i / checkNum
            if self.pointCollisionDetection(checkPoint):
                return True

        return False

    def RRT(self, optimize=True):
        # RRT搜索
        # optimize：是否进行路径优化

        iterTime = 0
        RRTTree = [[self.startPoint, -1]]

        while iterTime < self.maxIterNum:
            iterTime += 1

            # 按概率生成随机点
            if np.random.rand() > self.greedyProb:
                randPoint = np.random.rand(3) * (self.searchSpace[:, 1] - self.searchSpace[:, 0]) + self.searchSpace[:, 0]
                randPoint = randPoint.reshape((3, 1)) + self.startPoint
            else:
                randPoint = self.endPoint

            # 找到树上的最近点
            minDis = 1000000
            for i in range(len(RRTTree)):
                dis = np.linalg.norm(randPoint - RRTTree[i][0])
                if dis < minDis:
                    minDis = dis
                    minIndex = i

            # 生成新点
            newPoint = RRTTree[minIndex][0] + (randPoint - RRTTree[minIndex][0]) / minDis * self.step

            # 检测新点是否与障碍物相交
            if self.collisionDetection(RRTTree[minIndex][0], newPoint):
                continue

            # 将新点加入树
            RRTTree.append([newPoint, minIndex])

            # 检测是否到达目标点
            if np.linalg.norm(newPoint - self.endPoint) < self.targetThreshold:
                # print("RRT search done")
                startIndex = minIndex
                break

        # 从终点回溯到起点
        self.path = self.endPoint
        while startIndex != -1:
            self.path = np.hstack((RRTTree[startIndex][0], self.path))
            startIndex = RRTTree[startIndex][1]

        # DEBUG
        if np.linalg.norm(self.path[:, 0] - self.startPoint) == 0 and np.linalg.norm(self.path[:, -1] - self.endPoint) == 0:
            print("RRT search done")

        # 路径优化
        if optimize:
            self.pathOptimization()

        return self.path

    
    def pathOptimization(self):
        # 贪婪优化，去除冗余点

        startIndex = 0
        endIndex = self.path.shape[1] - 1
        detectTimes = self.path.shape[1] - 1

        newPath = self.path[:, 0].reshape((3, 1))

        while detectTimes > 0:
            detectTimes -= 1
            startPoint = self.path[:, startIndex].reshape((3, 1))
            endPoint = self.path[:, endIndex].reshape((3, 1))

            if self.collisionDetection(startPoint, endPoint):
                endIndex -= 1
            else:
                newPath = np.hstack((newPath, endPoint))
                startIndex = endIndex
                endIndex = self.path.shape[1] - 1
                detectTimes = self.path.shape[1] - 1 - startIndex

        # 设置点之间的最大距离，如果超过这个距离则插入新点
        maxDis = self.step * 2
        i = 0
        while i < newPath.shape[1] - 1:
            startPoint = newPath[:, i].reshape((3, 1))
            endPoint = newPath[:, i + 1].reshape((3, 1))
            if np.linalg.norm(startPoint - endPoint) > maxDis:
                newPoint = startPoint + (endPoint - startPoint) / np.linalg.norm(endPoint - startPoint) * maxDis
                newPath = np.hstack((newPath[:, :i + 1], newPoint, newPath[:, i + 1:]))
            i += 1

        self.path = newPath

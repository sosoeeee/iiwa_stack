import numpy as np
from RRT_PathPlanner import PathPlaner
from TrajectoryGenerator import MinimumTrajPlanner
import matplotlib.pyplot as plt

import time

start = time.time()

startPoint = np.array([0, 0, 0]).reshape((3, 1))
endPoint = np.array([1, 1, 0]).reshape((3, 1))

pathPlaner = PathPlaner(startPoint, endPoint)
pathPlaner.addObstacle(np.array([0.4, 0.4, 0]).reshape((3, 1)), 0.3)

path = pathPlaner.RRT(False)

end = time.time()

print("pathPlanner: ", end - start)

# 可视化
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(startPoint[0], startPoint[1], startPoint[2], c='r', marker='o')
ax.scatter(endPoint[0], endPoint[1], endPoint[2], c='r', marker='o')

# 绘制障碍物
for i in range(len(pathPlaner.obstacle)):
    obstacle = pathPlaner.obstacle[i]
    ax.scatter(obstacle['center'][0], obstacle['center'][1], obstacle['center'][2], c='b', marker='o')
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    x = obstacle['radius'] * np.outer(np.cos(u), np.sin(v)) + obstacle['center'][0]
    y = obstacle['radius'] * np.outer(np.sin(u), np.sin(v)) + obstacle['center'][1]
    z = obstacle['radius'] * np.outer(np.ones(np.size(u)), np.cos(v)) + obstacle['center'][2]
    ax.plot_surface(x, y, z, color='g')

# 绘制路径
for i in range(path.shape[1] - 1):
    # 绘制直线
    ax.plot([path[0, i], path[0, i + 1]], [path[1, i], path[1, i + 1]], [path[2, i], path[2, i + 1]], c='r')

plt.show()

print(path)
# 轨迹规划

start = time.time()

v0 = np.array([0, 0, 0]).reshape((3, 1))
a0 = np.array([0, 0, 0]).reshape((3, 1))
vt = np.array([0, 0, 0]).reshape((3, 1))
at = np.array([0, 0, 0]).reshape((3, 1))
miniJerkTrajPlanner = MinimumTrajPlanner(path, 0.01, 20, v0, a0, vt, at, 3)
traj = miniJerkTrajPlanner.computeTraj()

print(traj.shape)

end = time.time()

print("trajPlanner: ", end - start)

# # 可视化
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.scatter(startPoint[0], startPoint[1], startPoint[2], c='r', marker='o')
# ax.scatter(endPoint[0], endPoint[1], endPoint[2], c='r', marker='o')
#
# # 绘制障碍物
# for i in range(len(pathPlaner.obstacle)):
#     obstacle = pathPlaner.obstacle[i]
#     ax.scatter(obstacle['center'][0], obstacle['center'][1], obstacle['center'][2], c='b', marker='o')
#     u = np.linspace(0, 2 * np.pi, 100)
#     v = np.linspace(0, np.pi, 100)
#     x = obstacle['radius'] * np.outer(np.cos(u), np.sin(v)) + obstacle['center'][0]
#     y = obstacle['radius'] * np.outer(np.sin(u), np.sin(v)) + obstacle['center'][1]
#     z = obstacle['radius'] * np.outer(np.ones(np.size(u)), np.cos(v)) + obstacle['center'][2]
#     ax.plot_surface(x, y, z, color='g')
#
# # 绘制轨迹
# for i in range(traj.shape[1] - 1):
#     # 绘制轨迹点
#     ax.scatter(traj[0, i], traj[1, i], traj[2, i], c='r', marker='.')
#
# plt.show()



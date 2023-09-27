import numpy as np
from RRT_PathPlanner import PathPlanner
from TrajectoryGenerator import MinimumTrajPlanner
import matplotlib.pyplot as plt

import time

start = time.time()

startPoint = np.array([0, 0, 0]).reshape((3, 1))
endPoint = np.array([1, 1, 0]).reshape((3, 1))

searchRange = np.array([[-0.5, 1.5], [-0.5, 1.5], [0, 0]])

PathPlanner = PathPlanner(startPoint, endPoint, searchRange)
PathPlanner.addObstacle(np.array([0.4, 0.4, 0]).reshape((3, 1)), 0.3)

path = PathPlanner.RRT(True)

end = time.time()

print("pathPlanner: ", end - start)

# 可视化
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(startPoint[0], startPoint[1], startPoint[2], c='r', marker='o')
ax.scatter(endPoint[0], endPoint[1], endPoint[2], c='r', marker='o')

# 绘制障碍物
for i in range(len(PathPlanner.obstacle)):
    obstacle = PathPlanner.obstacle[i]
    ax.scatter(obstacle['center'][0], obstacle['center'][1], obstacle['center'][2], c='b', marker='o')
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    x = obstacle['radius'] * np.outer(np.cos(u), np.sin(v)) + obstacle['center'][0]
    y = obstacle['radius'] * np.outer(np.sin(u), np.sin(v)) + obstacle['center'][1]
    z = obstacle['radius'] * np.outer(np.ones(np.size(u)), np.cos(v)) + obstacle['center'][2]
    ax.plot_surface(x, y, z, color='g')

# # 绘制路径
for i in range(path.shape[1] - 1):
    # 绘制直线
    ax.plot([path[0, i], path[0, i + 1]], [path[1, i], path[1, i + 1]], [path[2, i], path[2, i + 1]], 'black', marker='.', markersize=3)

print(path)
# 轨迹规划

start = time.time()

v0 = np.array([0, 0, 0]).reshape((3, 1))
a0 = np.array([0, 0, 0]).reshape((3, 1))
vt = np.array([0, 0, 0]).reshape((3, 1))
at = np.array([0, 0, 0]).reshape((3, 1))
miniJerkTrajPlanner = MinimumTrajPlanner(path, 0.2, 100, v0, a0, vt, at, 4)
traj = miniJerkTrajPlanner.computeTraj()
trajAcc = miniJerkTrajPlanner.computeTrajAcc()

# print(traj.shape)

end = time.time()

# 绘制轨迹
# for i in range(traj.shape[1] - 1):
#     # 绘制轨迹点
#     ax.scatter(traj[0, i], traj[1, i], traj[2, i], c='r', marker='.')

plt.show()

# 绘制轨迹x，y，z三个方向的加速度曲线虚线
plt.figure()
plt.subplot(3, 1, 1)
plt.plot(traj[3, :], 'r')
# plt.plot(trajAcc[0, :], 'r--')
plt.subplot(3, 1, 2)
plt.plot(traj[4, :], 'g')
# plt.plot(trajAcc[1, :], 'g--')
plt.subplot(3, 1, 3)
plt.plot(traj[5, :], 'b')
# plt.plot(trajAcc[2, :], 'b--')
plt.show()

print("trajPlanner: ", end - start)

# # 可视化
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.scatter(startPoint[0], startPoint[1], startPoint[2], c='r', marker='o')
# ax.scatter(endPoint[0], endPoint[1], endPoint[2], c='r', marker='o')
#
# # 绘制障碍物
# for i in range(len(PathPlanner.obstacle)):
#     obstacle = PathPlanner.obstacle[i]
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



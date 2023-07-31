import numpy as np

def getSphericalPoints(pos, radius, distanceStep):
     # 生成球面的点
     sphericalPoints = None

     phi = np.arange(0, np.pi, distanceStep/radius)
     for phi_ in phi:
          radiusCircle = radius * np.sin(phi_)
          if radiusCircle == 0:
               circlePoints_ = np.array([pos[0], pos[1], radius * np.cos(phi_) + pos[2]]).reshape((3, 1))
          else:
               theta = np.arange(0, 2*np.pi, distanceStep/radiusCircle)
               x = radiusCircle * np.cos(theta) + pos[0]
               y = radiusCircle * np.sin(theta) + pos[1]
               z = np.ones(len(theta)) * radius * np.cos(phi_) + pos[2]
               circlePoints_ = np.vstack((x, y, z))
          if sphericalPoints is None:
               sphericalPoints = circlePoints_
          else:
               sphericalPoints = np.hstack((sphericalPoints, circlePoints_))
     
     return sphericalPoints

def getCuboidPoints(pos1, pos2, distanceStep):
     # 生成长方体表面的点
     cuboidPoints = None

     # 生成长方体的六个面
     x = np.linspace(pos1[0], pos2[0], int(abs(pos2[0] - pos1[0]) / distanceStep))
     y = np.linspace(pos1[1], pos2[1], int(abs(pos2[1] - pos1[1]) / distanceStep))
     z = np.linspace(pos1[2], pos2[2], int(abs(pos2[2] - pos1[2]) / distanceStep))
     # x-y平面
     x1, y1 = np.meshgrid(x, y)
     z1 = np.ones(x1.shape) * pos1[2]
     x2, y2 = np.meshgrid(x, y)
     z2 = np.ones(x2.shape) * pos2[2]
     # x-z平面
     # x3, z3 = np.meshgrid(x, z)
     # y3 = np.ones(x3.shape) * pos1[1]
     # x4, z4 = np.meshgrid(x, z)
     # y4 = np.ones(x4.shape) * pos2[1]
     # y-z平面
     y5, z5 = np.meshgrid(y, z)
     x5 = np.ones(y5.shape) * pos1[0]
     y6, z6 = np.meshgrid(y, z)
     x6 = np.ones(y6.shape) * pos2[0]

     # 合并
     cuboidPoints = np.vstack((x1.reshape((1, -1)), y1.reshape((1, -1)), z1.reshape((1, -1))))
     cuboidPoints = np.hstack((cuboidPoints, np.vstack((x2.reshape((1, -1)), y2.reshape((1, -1)), z2.reshape((1, -1))))))
     # cuboidPoints = np.hstack((cuboidPoints, np.vstack((x3.reshape((1, -1)), y3.reshape((1, -1)), z3.reshape((1, -1))))))
     # cuboidPoints = np.hstack((cuboidPoints, np.vstack((x4.reshape((1, -1)), y4.reshape((1, -1)), z4.reshape((1, -1))))))
     cuboidPoints = np.hstack((cuboidPoints, np.vstack((x5.reshape((1, -1)), y5.reshape((1, -1)), z5.reshape((1, -1))))))
     cuboidPoints = np.hstack((cuboidPoints, np.vstack((x6.reshape((1, -1)), y6.reshape((1, -1)), z6.reshape((1, -1))))))

     return cuboidPoints

# 测试getCuboidPoints函数
if __name__ == '__main__':
     import matplotlib.pyplot as plt

     fig = plt.figure()
     ax = fig.add_subplot(111, projection='3d')
     ax.set_xlabel('x')
     ax.set_ylabel('y')
     ax.set_zlabel('z')

     # 测试球面
     # pos = np.array([0, 0, 0]).reshape((3, 1))
     # radius = 1
     # distanceStep = 0.1
     # sphericalPoints = getSphericalPoints(pos, radius, distanceStep)
     # print(sphericalPoints.shape)
     # ax.scatter(sphericalPoints[0, :], sphericalPoints[1, :], sphericalPoints[2, :], c='r', marker='o')

     # 测试长方体
     initX = -0.46015
     initY = 0.11484
     initZ = 0.20458
     
     Xrange = [-0.26, 0.1]
     Yrange = [-1, 1]
     Zrange = [-1, 1]

     pos1 = np.array([initX + Xrange[0], initY + Yrange[0], initZ + Zrange[0]]).reshape((3, 1))
     pos2 = np.array([initX + Xrange[1], initY + Yrange[1], initZ + Zrange[1]]).reshape((3, 1))
     distanceStep = 0.1
     cuboidPoints = getCuboidPoints(pos1, pos2, distanceStep)
     ax.scatter(cuboidPoints[0, :], cuboidPoints[1, :], cuboidPoints[2, :], c='r', marker='o')

     plt.show()
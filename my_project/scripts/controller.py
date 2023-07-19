#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from iiwa_msgs.msg import CartesianPose
import numpy as np

# 初始化
firstFlag = True
initPos = PoseStamped()
currentPos = PoseStamped()
initX = -0.35
initY = -0.35
initZ = 0.13
initOrientationX = -0.3351
initOrientationY = 0.9418
initOrientationZ = -0.0253
initOrientationW = -0.0037

controllerFreq = 20         # Hz

def state_callBack(msg):
    # initalization msg
    global initPos, firstFlag, currentPos
    if firstFlag:
        print("initialization msg")
        initPos = msg.poseStamped
        firstFlag = False
    else:
        currentPos = msg.poseStamped

##########################################################################

# 指令输出
rospy.init_node('controller', anonymous=False)
pub = rospy.Publisher('referTrajectory', String, queue_size=10)
pubInit = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=10)
rospy.Subscriber('/iiwa/state/CartesianPose', CartesianPose, state_callBack, queue_size=1)
rate = rospy.Rate(controllerFreq)

# 等待初始化
while firstFlag:
    rospy.sleep(1)

# 发布初始位置
if firstFlag is False:
    print("go to initial position")
    initPos.pose.position.x = initX
    initPos.pose.position.y = initY
    initPos.pose.position.z = initZ
    initPos.pose.orientation.x = initOrientationX
    initPos.pose.orientation.y = initOrientationY
    initPos.pose.orientation.z = initOrientationZ
    while np.linalg.norm(np.array([currentPos.pose.position.x, currentPos.pose.position.y, currentPos.pose.position.z]) - np.array([initX, initY, initZ])) > 0.005:
        pubInit.publish(initPos)
        rospy.sleep(1)

print("start to publish referTrajectory")

##########################################################################

# 轨迹规划
# 轨迹生成
# 二维测试轨迹——圆形
R = 0.1
targetSpeed = 0.02         # m/s
stepDis = targetSpeed/controllerFreq
stepTheta = stepDis/R
theta = np.arange(0, 2*np.pi, stepTheta)

x = R*np.cos(theta) + initX
y = R*np.sin(theta) + initY
z = np.ones(len(theta)) * initZ

# 前向差分求速度
speedMatrix = np.eye(len(theta)) * (-1)
for i in range(len(theta)):
    speedMatrix[i][(i - 1) % len(theta)] = 1
speedMatrix = speedMatrix.T

vx = speedMatrix.dot(x) * controllerFreq
vy = speedMatrix.dot(y) * controllerFreq
vz = speedMatrix.dot(z) * controllerFreq

# 机械臂状态空间模型建立
Md = 5 * np.eye(3)
Cd = 75 * np.eye(3)
Md_inv = np.linalg.inv(Md)
A = np.zeros((6, 6))
A[0:3, 3:6] = np.eye(3)
A[3:6, 3:6] = -Md_inv.dot(Cd)
Br = np.zeros((6, 3))
Br[3:6, :] = Md_inv
Bh = np.zeros((6, 3))
Bh[3:6, :] = Md_inv
C = np.zeros((3, 6))
C[:, 0:3] = np.eye(3)

# 离散化
T = 1/controllerFreq
Ad = np.eye(6) + A*T
Brd = Br*T
Bhd = Bh*T

# 控制器参数设置
N1 = 50
phi = np.zeros((3*N1, 6))
theta_h = np.zeros((3*N1, 3*N1))
theta_r = np.zeros((3*N1, 3*N1))
phi_row = C.dot(Ad)
weight_r = 1
weight_h = 0.1
errorAmplitude = 10000
Qii_r = np.eye(3) * weight_r * errorAmplitude
Qii_h = np.eye(3) * weight_h * errorAmplitude
Qr = np.zeros((3*N1, 3*N1))
Qh = np.zeros((3*N1, 3*N1))

for i in range(N1):
    phi[(i*3+1):(3*i+3), :] = phi_row
    Qr[(i*3+1):(3*i+3), (i*3+1):(3*i+3)] = Qii_r
    Qh[(i*3+1):(3*i+3), (i*3+1):(3*i+3)] = Qii_h
    theta_h_row = np.eye(6)
    for j in range(i):
        theta_h[(i*3+1):(3*i+3),(3*(i-j)+1):(3*(i-j)+3)] = np.dot(C, theta_h_row.dot(Bhd))
        theta_r[(i*3+1):(3*i+3),(3*(i-j)+1):(3*(i-j)+3)] = np.dot(C, theta_h_row.dot(Brd))
        theta_h_row = theta_h_row.dot(Ad)
    phi_row = phi_row.dot(Ad)

theta_hg = np.cstack((theta_h, theta_h))
theta_rg = np.cstack((theta_r, theta_r))
phi_g = np.cstack((phi, phi))

# 初始化状态变量
x_cur = np.zeros((6, 1))
x_cur[0, 0] = x[0]
x_cur[1, 0] = y[0]
x_cur[2, 0] = z[0]

i = 0
while not rospy.is_shutdown():
    referTrajectory = str(x[i]) + ',' + str(y[i]) + ',' + str(z[i]) + ',' + str(vx[i]) + ',' + str(vy[i]) + ',' + str(vz[i])
    pub.publish(referTrajectory)
    i = (i + 1) % len(theta)
    rate.sleep()
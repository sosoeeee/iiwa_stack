import rospy
from std_msgs.msg import String
import numpy as np

from Jacobian import Jacobian

# 初始化

# 轨迹规划
# 二维测试轨迹——圆形
R = 0.1
controllerFreq = 20         # Hz
targetSpeed = 0.015         # m/s
stepDis = targetSpeed/controllerFreq
initX = 0
initY = 0
initZ = 0
stepTheta = stepDis/R
theta = np.arange(0, 2*np.pi, stepTheta)

x = R*np.cos(theta) + initX
y = R*np.sin(theta) + initY
z = np.ones(len(theta)) * initZ

# 控制器决策

# 笛卡尔空间到关节空间转换

# 指令输出
rospy.init_node('controller', anonymous=True)
pub = rospy.Publisher('referTrajectory', String, queue_size=10)
rate = rospy.Rate(controllerFreq)

while not rospy.is_shutdown():
    for i in range(len(theta)):
        referTrajectory = str(x[i]) + ',' + str(y[i]) + ',' + str(z[i])
        rospy.loginfo(referTrajectory)
        pub.publish(referTrajectory)
        rate.sleep()
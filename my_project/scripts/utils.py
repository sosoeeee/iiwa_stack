import numpy as np

# 将JointPositionVelocity消息转换为numpy数组
def toArray(msg):
     ans = np.zeros(7)
     ans[0] = msg.a1
     ans[1] = msg.a2
     ans[2] = msg.a3
     ans[3] = msg.a4
     ans[4] = msg.a5
     ans[5] = msg.a6
     ans[6] = msg.a7
     return ans
#!/usr/bin/env python3

import rospy
from pynput import keyboard
from iiwa_msgs.msg import CartesianPose

# def talker():
#      rospy.init_node('talker', anonymous=True)
#      pub = rospy.Publisher('/iiwa/command/CartesianPose', CartesianPose, queue_size=10)
#      rate = rospy.Rate(10) # 10hz

#      while not rospy.is_shutdown():
#            hello_str = CartesianPose()
#            hello_str.poseStamped.pose.position.x = 0.0
#            hello_str.poseStamped.pose.position.y = 0.0
#            hello_str.poseStamped.pose.position.z = 0.0
#            hello_str.poseStamped.pose.orientation.x = 0.0
#            hello_str.poseStamped.pose.orientation.y = 0.0
#            hello_str.poseStamped.pose.orientation.z = 0.0
#            hello_str.poseStamped.pose.orientation.w = 0.0
#            rospy.loginfo(hello_str)
#            pub.publish(hello_str)
#            rate.sleep()

def on_press(key):
    try:
        # 获取按键的字符串表示形式
        key_value = key.char
    except AttributeError:
        # 对于特殊按键（如回车、删除等），获取其特殊名称
        key_value = key.name

    print("Pressed key:", key_value)

def on_release(key):
    # 当按键释放时触发的回调函数
    pass

if __name__ == '__main__':
    # try:
    #     talker()
    # except rospy.ROSInterruptException:
    #     pass

    # 创建一个键盘监听器
    listener = keyboard.Listener(
        on_press=on_press,
        on_release=on_release)

    # 启动监听器
    listener.start()

    # 等待监听器线程结束（可以按Ctrl+C停止程序）
    listener.join()

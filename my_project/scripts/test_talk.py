#!/usr/bin/env python3

import rospy
from pynput import keyboard
from std_msgs.msg import String


def on_press(key):
    try:
        # 获取按键的字符串表示形式
        command_key = key.char

        if command_key =='w':
            pub.publish('0.05, 0.0, 0.0')
        if command_key == 's':
            pub.publish('-0.05, 0.0, 0.0')
        if command_key == 'a':
            pub.publish('0.0, 0.05, 0.0')
        if command_key == 'd':
            pub.publish('0.0, -0.05, 0.0')

    except AttributeError:
        # 对于特殊按键（如回车、删除等），获取其特殊名称
        command_key = key.name

    print("Pressed key:", command_key)
    rate.sleep()

def on_release(key):
    # 当按键释放时触发的回调函数
    pub.publish('0.0, 0.0, 0.0')
    pass

if __name__ == '__main__':
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('keyboardCmd', String, queue_size=1)
    rate = rospy.Rate(0.5)

    # 创建一个键盘监听器
    listener = keyboard.Listener(
        on_press=on_press,
        on_release=on_release)

    # 启动监听器
    listener.start()

    # 等待监听器线程结束（可以按Ctrl+C停止程序）
    listener.join()

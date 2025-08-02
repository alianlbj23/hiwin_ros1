#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import json
from std_msgs.msg import String

def publish_fixed_points(json_pub):
    points = [
        (-0.4992596209049225, 0.8769174218177795, 0.7969591021537781),
        (-0.4992596209049225, 0.5, 0.1),
        (-0.07023268938064575, 1.0, 0.9886242151260376)
    ]
    data = {"points": [{"x": x, "y": y, "z": z} for (x, y, z) in points]}
    json_str = json.dumps(data)
    msg = String(data=json_str)
    json_pub.publish(msg)
    rospy.loginfo("已送出 %d 個點 (固定座標，JSON)", len(points))

def publish_robot_signal(signal_pub, signal):
    msg = String(data=signal)
    signal_pub.publish(msg)
    rospy.loginfo("已發送 robot_signal: '%s'", signal)

if __name__ == "__main__":
    rospy.init_node('my_pointarray_json_publisher')
    json_pub = rospy.Publisher('/target_points_json', String, queue_size=10)
    signal_pub = rospy.Publisher('/robot_signal', String, queue_size=10)
    rospy.sleep(1.0)  # 等待 publisher 註冊完成

    try:
        while not rospy.is_shutdown():
            print("\n請選擇功能：")
            print("1. 發送固定範例座標點到 /target_points_json")
            print("2. 發送 robot_signal = 'plan' 到 /robot_signal")
            print("3. 發送 robot_signal = 'execute' 到 /robot_signal")
            print("4. 發送 robot_signal = 'stop' 到 /robot_signal")
            print("q. 離開 (quit)")
            func = input("請輸入功能編號 (1/2/3/4/q): ").strip()

            if func == "1":
                publish_fixed_points(json_pub)
            elif func == "2":
                publish_robot_signal(signal_pub, "plan")
            elif func == "3":
                publish_robot_signal(signal_pub, "execute")
            elif func == "4":
                publish_robot_signal(signal_pub, "stop")
            elif func.lower() == "q":
                print("結束程式")
                break
            else:
                print("輸入錯誤，請輸入 1、2、3、4 或 q")
            
            rospy.sleep(0.2)
    except KeyboardInterrupt:
        print("\n已收到 Ctrl+C，程式結束。")

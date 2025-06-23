#!/usr/bin/env python
# For auto publish position data to /target_points, simulate meta points
import rospy
from custom_msgs.msg import Point, PointArray

if __name__ == "__main__":
    rospy.init_node('my_pointarray_publisher')
    pub = rospy.Publisher('/target_points', PointArray, queue_size=10)
    rospy.sleep(1.0)  # 等待 publisher 註冊完成

    msg = PointArray()
    # 範例：送出三個座標
    points = [
        (-0.4992596209049225, 0.8769174218177795, 0.7969591021537781),
        (-0.4992596209049225, 0.8769174218177795, 0.7969591021537781),
        (-0.07023268938064575, 0.8089713454246521, 0.9886242151260376)
    ]
    for x, y, z in points:
        pt = Point()
        pt.x = x
        pt.y = y
        pt.z = z
        msg.points.append(pt)
    pub.publish(msg)
    rospy.loginfo("已送出 %d 個點", len(msg.points))
    rospy.sleep(0.5)  # 稍等確保訊息有送出去

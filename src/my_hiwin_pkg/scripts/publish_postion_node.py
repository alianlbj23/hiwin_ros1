#!/usr/bin/env python
# For auto publish position data to /target_points, simulate meta points
import rospy
from custom_msgs.msg import Point, PointArray

if __name__ == "__main__":
    rospy.init_node('my_pointarray_publisher')
    pub = rospy.Publisher('/target_points', PointArray, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        msg = PointArray()
        # 範例：送出三個隨機座標
        points = [
            (-0.5, 0.1, -0.8),
            (-0.6, 0.05, -0.9),
            (-0.7, 0.02, -0.7)
        ]
        for x, y, z in points:
            pt = Point()
            pt.x = x
            pt.y = y
            pt.z = z
            msg.points.append(pt)
        pub.publish(msg)
        rospy.loginfo("已送出 %d 個點", len(msg.points))
        rate.sleep()

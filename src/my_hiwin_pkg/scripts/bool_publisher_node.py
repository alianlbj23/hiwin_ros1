#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool

def bool_publisher_once():
    rospy.init_node('bool_publisher_node')
    pub = rospy.Publisher('bool_topic', Bool, queue_size=10)

    # 等待 Publisher 註冊完成（非常重要，否則可能發送失敗）
    rospy.sleep(1.0)

    msg = Bool()
    msg.data = True
    pub.publish(msg)
    rospy.loginfo("Published once: %s", msg.data)

if __name__ == '__main__':
    try:
        bool_publisher_once()
    except rospy.ROSInterruptException:
        pass

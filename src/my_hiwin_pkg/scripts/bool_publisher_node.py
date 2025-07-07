#!/usr/bin/env python
import rospy
import json
from std_msgs.msg import String

def json_bool_publisher_once():
    rospy.init_node('bool_json_publisher_node')
    # 改用 String topic
    pub = rospy.Publisher('/bool_topic_json', String, queue_size=10)

    # 等待 publisher 註冊完成
    rospy.sleep(1.0)

    # 把布林值包成 JSON，再轉成字串
    payload = { "data": True }
    json_str = json.dumps(payload)

    # 發佈 JSON string
    msg = String(data=json_str)
    pub.publish(msg)
    rospy.loginfo("Published JSON bool: %s", json_str)

if __name__ == '__main__':
    try:
        json_bool_publisher_once()
    except rospy.ROSInterruptException:
        pass

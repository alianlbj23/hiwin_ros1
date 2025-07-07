#!/usr/bin/env python
# For auto-publish position data as JSON to /target_points_json, simulate meta points
import rospy
import json
from std_msgs.msg import String

if __name__ == "__main__":
    rospy.init_node('my_pointarray_json_publisher')
    json_pub = rospy.Publisher('/target_points_json', String, queue_size=10)
    rospy.sleep(1.0)  # 等待 publisher 註冊完成

    # 範例：三個座標點
    points = [
        (-0.4992596209049225, 0.8769174218177795, 0.7969591021537781),
        (-0.4992596209049225, 0.8769174218177795, 0.7969591021537781),
        (-0.07023268938064575, 0.8089713454246521, 0.9886242151260376)
    ]

    # 組成 JSON 結構
    data = {
        "points": [
            {"x": x, "y": y, "z": z}
            for (x, y, z) in points
        ]
    }

    # 序列化成字串並發布
    json_str = json.dumps(data)
    msg = String(data=json_str)
    json_pub.publish(msg)
    rospy.loginfo("已送出 %d 個點 (JSON)", len(points))

    rospy.sleep(0.5)  # 稍等確保訊息送出

#!/usr/bin/env python
import rospy
from custom_msgs.msg import PointArray
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

ik_ready = False  # 全域變數，等待 bool_topic 的 True

def bool_callback(msg):
    global ik_ready
    ik_ready = msg.data
    rospy.loginfo("Received bool_topic: %s", ik_ready)

def compute_ik_and_get_joints(p, group='manipulator', frame_id='base_link', ik_link='tool0'):
    rospy.wait_for_service('/compute_ik')
    ik_srv = rospy.ServiceProxy('/compute_ik', GetPositionIK)
    req = GetPositionIKRequest()
    req.ik_request.group_name = group
    req.ik_request.ik_link_name = ik_link
    req.ik_request.pose_stamped.header.frame_id = frame_id
    req.ik_request.pose_stamped.pose.position.x = p.x
    req.ik_request.pose_stamped.pose.position.y = p.y
    req.ik_request.pose_stamped.pose.position.z = p.z
    req.ik_request.pose_stamped.pose.orientation.x = 0.0
    req.ik_request.pose_stamped.pose.orientation.y = 0.0
    req.ik_request.pose_stamped.pose.orientation.z = 0.0
    req.ik_request.pose_stamped.pose.orientation.w = 1.0
    req.ik_request.timeout = rospy.Duration(1.0)
    resp = ik_srv(req)
    if resp.error_code.val == 1:
        return resp.solution.joint_state
    else:
        return None

def callback(msg):
    global ik_ready
    joints_list = []

    for i, pt in enumerate(msg.points):
        js = compute_ik_and_get_joints(pt)
        if js is None:
            rospy.logwarn("第%d個點無解 (%.3f, %.3f, %.3f)", i, pt.x, pt.y, pt.z)
            success_pub.publish(False)  # 發布 IK 失敗
            return
        else:
            joints_list.append(js)

    rospy.loginfo("所有點都可達，等待 bool_topic 為 True...")

    while not ik_ready and not rospy.is_shutdown():
        rospy.sleep(0.1)

    rospy.loginfo("bool_topic 為 True，開始發佈 joint angles")
    for i, js in enumerate(joints_list):
        pub.publish(js)
        rospy.loginfo("已發佈第%d個點的 joint angles：%s", i, js.position)
        rospy.sleep(0.5)

    success_pub.publish(True)  # 發布 IK 成功

if __name__ == "__main__":
    rospy.init_node('ik_checker_and_publisher')
    pub = rospy.Publisher('/target_points_joints', JointState, queue_size=10)
    success_pub = rospy.Publisher('/ik_success', Bool, queue_size=10)
    rospy.Subscriber('/target_points', PointArray, callback)
    rospy.Subscriber('Anchor', PointArray, callback)
    rospy.Subscriber('/bool_topic', Bool, bool_callback)
    rospy.spin()

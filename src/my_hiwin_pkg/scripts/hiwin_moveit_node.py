#!/usr/bin/env python
import rospy
from custom_msgs.msg import PointArray
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from sensor_msgs.msg import JointState  # 用內建 JointState 來 publish 關節角
from geometry_msgs.msg import PoseStamped

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
    joints_list = []
    for i, pt in enumerate(msg.points):
        js = compute_ik_and_get_joints(pt, group='manipulator', frame_id='base_link', ik_link='tool0')
        if js is None:
            rospy.logwarn("第%d個點無解/不可達 (%.3f, %.3f, %.3f)，不 publish joints", i, pt.x, pt.y, pt.z)
            return  # 只要有一個點無法 IK，直接結束
        else:
            joints_list.append(js)

    rospy.loginfo("所有點都可達，依序發佈 joint angles")
    for i, js in enumerate(joints_list):
        pub.publish(js)
        rospy.loginfo("已發佈第%d個點的 joint angles：%s", i, js.position)
        rospy.sleep(0.5)  # 依序發送時，間隔 0.5 秒

if __name__ == "__main__":
    rospy.init_node('ik_checker_and_publisher')
    pub = rospy.Publisher('/target_points_joints', JointState, queue_size=10)
    rospy.Subscriber('/target_points', PointArray, callback)
    rospy.spin()

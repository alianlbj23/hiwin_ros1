#!/usr/bin/env python
import rospy
from custom_msgs.msg import PointArray
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from geometry_msgs.msg import PoseStamped

def check_ik(p, group='manipulator', frame_id='base_link', ik_link='tool0'):
    rospy.wait_for_service('/compute_ik')
    try:
        ik_srv = rospy.ServiceProxy('/compute_ik', GetPositionIK)
        req = GetPositionIKRequest()
        req.ik_request.group_name = group
        req.ik_request.ik_link_name = ik_link
        req.ik_request.pose_stamped.header.frame_id = frame_id
        req.ik_request.pose_stamped.pose.position.x = p.x
        req.ik_request.pose_stamped.pose.position.y = p.y
        req.ik_request.pose_stamped.pose.position.z = p.z
        # Orientation: 單位四元數（不指定末端方向可用）
        req.ik_request.pose_stamped.pose.orientation.x = 0.0
        req.ik_request.pose_stamped.pose.orientation.y = 0.0
        req.ik_request.pose_stamped.pose.orientation.z = 0.0
        req.ik_request.pose_stamped.pose.orientation.w = 1.0
        req.ik_request.timeout = rospy.Duration(1.0)
        resp = ik_srv(req)
        return resp.error_code.val == 1  # 1 代表成功
    except Exception as e:
        rospy.logwarn("IK service failed: %s", str(e))
        return False

def callback(msg):
    for i, pt in enumerate(msg.points):
        reachable = check_ik(pt, group='manipulator', frame_id='base_link', ik_link='tool0')
        if reachable:
            rospy.loginfo("第%d個點可達 (%.3f, %.3f, %.3f)", i, pt.x, pt.y, pt.z)
        else:
            rospy.logwarn("第%d個點無解/不可達 (%.3f, %.3f, %.3f)", i, pt.x, pt.y, pt.z)

if __name__ == "__main__":
    rospy.init_node('ik_checker')
    rospy.Subscriber('/target_points', PointArray, callback)
    rospy.spin()

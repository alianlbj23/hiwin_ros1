#!/usr/bin/env python
import rospy
import json
from std_msgs.msg import String
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from sensor_msgs.msg import JointState

ik_ready = False  # 等待 robot_signal 收到 'plan'

def robot_signal_callback(msg: String):
    global ik_ready
    rospy.loginfo("Received robot_signal: %s", msg.data)
    # 只有收到 'plan' 字串才觸發
    if msg.data.strip().lower() == 'plan':
        ik_ready = True
        rospy.loginfo("ik_ready set to True (plan received)")

def compute_ik_and_get_joints(p, group='manipulator', frame_id='base_link', ik_link='tool0'):
    try:
        rospy.loginfo("Waiting for /compute_ik service…")
        rospy.wait_for_service('/compute_ik', timeout=5.0)
    except rospy.ROSException:
        rospy.logerr("/compute_ik service not available after timeout")
        return None

    ik_srv = rospy.ServiceProxy('/compute_ik', GetPositionIK)
    req = GetPositionIKRequest()
    req.ik_request.group_name = group
    req.ik_request.ik_link_name = ik_link
    req.ik_request.pose_stamped.header.frame_id = frame_id
    req.ik_request.pose_stamped.pose.position.x = p['x']
    req.ik_request.pose_stamped.pose.position.y = p['y']
    req.ik_request.pose_stamped.pose.position.z = p['z']
    req.ik_request.pose_stamped.pose.orientation.w = 1.0
    req.ik_request.timeout = rospy.Duration(1.0)

    resp = ik_srv(req)
    if resp.error_code.val == 1:
        return resp.solution.joint_state
    else:
        return None

def json_callback(msg: String):
    global ik_ready
    try:
        data = json.loads(msg.data)
        pts = data.get('points', [])
    except json.JSONDecodeError as e:
        rospy.logerr("Cannot decode points JSON: %s", e)
        return

    for idx, p in enumerate(pts):
        rospy.loginfo("Target point %d → x: %.6f, y: %.6f, z: %.6f", idx, p['x'], p['y'], p['z'])

    joints_list = []
    for i, p in enumerate(pts):
        js = compute_ik_and_get_joints(p)
        if js is None:
            rospy.logwarn("Point %d IK failed: %s", i, p)
            json_pub.publish(json.dumps({'success': False, 'failed_index': i}))
            return
        joints_list.append({'name': js.name, 'position': js.position})

    rospy.loginfo("All points IK OK, waiting for robot_signal == 'plan' …")
    # 等待收到 plan 指令才發布
    while not ik_ready and not rospy.is_shutdown():
        rospy.sleep(0.1)

    reply = {'success': True, 'joints': joints_list}
    json_pub.publish(json.dumps(reply))
    rospy.loginfo("Published all joint angles JSON")
    ik_ready = False

if __name__ == "__main__":
    rospy.init_node('ik_json_interface')

    # 接收點陣列 JSON
    rospy.Subscriber('/target_points_json', String, json_callback)
    # 接收 robot_signal (直接 String 格式)
    rospy.Subscriber('/robot_signal', String, robot_signal_callback)

    # 回傳關節角度的 JSON
    json_pub = rospy.Publisher('/target_points_joints_json', String, queue_size=1)

    rospy.loginfo("IK JSON interface node started, waiting for /target_points_json …")
    rospy.spin()

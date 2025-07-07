#!/usr/bin/env python
import rospy
import json
from std_msgs.msg import String
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from sensor_msgs.msg import JointState

ik_ready = False  # 等待 bool_topic_json 變 True

def bool_json_callback(msg: String):
    global ik_ready
    rospy.loginfo("RAW bool JSON payload: %s", msg.data)
    try:
        payload = json.loads(msg.data)
        ik_ready = bool(payload.get('data', False))
        rospy.loginfo("Parsed ik_ready = %s", ik_ready)
    except json.JSONDecodeError as e:
        rospy.logerr("Cannot decode bool JSON: %s", e)

def compute_ik_and_get_joints(p, group='manipulator', frame_id='base_link', ik_link='tool0'):
    # 等待 IK 服務可用，5 秒 timeout
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
    # 解析進來的點陣列 JSON
    try:
        data = json.loads(msg.data)
        pts = data.get('points', [])
    except json.JSONDecodeError as e:
        rospy.logerr("Cannot decode points JSON: %s", e)
        return

    # 打印所有接收到的目標點
    for idx, p in enumerate(pts):
        rospy.loginfo("Target point %d → x: %.6f, y: %.6f, z: %.6f", idx, p['x'], p['y'], p['z'])

    joints_list = []
    # 為每個點計算 IK
    for i, p in enumerate(pts):
        js = compute_ik_and_get_joints(p)
        if js is None:
            rospy.logwarn("Point %d IK failed: %s", i, p)
            # 直接回報失敗給 Unity
            json_pub.publish(json.dumps({'success': False, 'failed_index': i}))
            return
        joints_list.append({'name': js.name, 'position': js.position})

    rospy.loginfo("All points IK OK, waiting for bool_topic_json=True…")
    # 等待外部布林信號
    while not ik_ready and not rospy.is_shutdown():
        rospy.sleep(0.1)

    # 回傳所有關節角度 JSON
    reply = {'success': True, 'joints': joints_list}
    json_pub.publish(json.dumps(reply))
    rospy.loginfo("Published all joint angles JSON")
    ik_ready = False

if __name__ == "__main__":
    rospy.init_node('ik_json_interface')

    # 接收點陣列 JSON
    rospy.Subscriber('/target_points_json', String, json_callback)
    # 接收布林 JSON
    rospy.Subscriber('/bool_topic_json', String, bool_json_callback)

    # 回傳關節角度的 JSON
    json_pub = rospy.Publisher('/target_points_joints_json', String, queue_size=1)

    rospy.loginfo("IK JSON interface node started, waiting for /target_points_json …")
    rospy.spin()

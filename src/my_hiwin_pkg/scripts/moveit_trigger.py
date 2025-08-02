#!/usr/bin/env python
import rospy
import json
from std_msgs.msg import String
import moveit_commander
import moveit_msgs.msg

def joints_callback(msg):
    rospy.loginfo("✅ 收到 /target_points_joints_json，準備執行 MoveIt 動作")

    try:
        data = json.loads(msg.data)
        if not data.get("success", False):
            rospy.logwarn("⚠️ joints_json 表示失敗，跳過")
            return

        joints_list = data.get("joints", [])
        if not joints_list:
            rospy.logwarn("⚠️ joints_list 為空，無法執行")
            return

        for i, joint_target in enumerate(joints_list):
            names = joint_target.get("name", [])
            positions = joint_target.get("position", [])

            if len(names) != len(positions):
                rospy.logwarn("⚠️ joint name 和 position 長度不一致")
                continue

            joint_goal = dict(zip(names, positions))
            rospy.loginfo(f"第 {i+1} 筆目標: {joint_goal}")

            group.set_joint_value_target(joint_goal)
            plan = group.plan()
            if plan and plan.joint_trajectory.points:
                rospy.loginfo("✅ 開始執行 MoveIt 路徑")
                group.go(wait=True)
            else:
                rospy.logwarn("❌ MoveIt 無法規劃路徑")

        group.stop()
        group.clear_pose_targets()
        rospy.loginfo("✅ 所有動作執行完成")

    except Exception as e:
        rospy.logerr(f"解析或執行失敗: {e}")

if __name__ == '__main__':
    moveit_commander.roscpp_initialize([])
    rospy.init_node('execute_joints_from_json_node', anonymous=True)

    group_name = "manipulator"  # 請根據你的 MoveIt 群組名稱修改
    group = moveit_commander.MoveGroupCommander(group_name)
    rospy.loginfo("🎯 MoveGroupCommander 已建立: %s", group_name)

    rospy.Subscriber("/target_points_joints_json", String, joints_callback)
    rospy.loginfo("🟢 等待 /target_points_joints_json 訊息...")
    rospy.spin()

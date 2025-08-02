#!/usr/bin/env python
import rospy
import actionlib
import json

from std_msgs.msg import String
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, Constraints, JointConstraint

class MoveGroupCommanderFromComplexJSON:
    def __init__(self):
        rospy.init_node('json_to_move_group_node')

        self.client = actionlib.SimpleActionClient('/move_group', MoveGroupAction)
        rospy.loginfo("等待 /move_group action server ...")
        self.client.wait_for_server()
        rospy.loginfo("/move_group 已連接")

        rospy.Subscriber("/target_points_joints_json", String, self.callback)
        rospy.loginfo("訂閱 /target_points_joints_json 中...")

        rospy.spin()

    def callback(self, msg):
        try:
            # 將字串解碼成 JSON 物件
            outer_data = json.loads(msg.data)
            joint_data = outer_data["joints"][0]  # 只取第一組 joints

            joint_names = joint_data["name"]
            joint_positions = joint_data["position"]

            if len(joint_names) != len(joint_positions):
                raise ValueError("joint 名稱數量與 position 數量不符")

            # 建立 MoveGroupGoal
            joint_constraints = []
            for name, pos in zip(joint_names, joint_positions):
                jc = JointConstraint()
                jc.joint_name = name
                jc.position = pos
                jc.tolerance_above = 0.01
                jc.tolerance_below = 0.01
                jc.weight = 1.0
                joint_constraints.append(jc)

            goal = MoveGroupGoal()
            goal.request.group_name = "manipulator"  # 替換為你的 MoveIt group 名稱
            goal.request.goal_constraints.append(Constraints(joint_constraints=joint_constraints))

            rospy.loginfo("發送角度到 /move_group ...")
            self.client.send_goal(goal)
            self.client.wait_for_result()
            rospy.loginfo("機械手臂已完成動作")

        except Exception as e:
            rospy.logerr("JSON 解析或 MoveIt 執行錯誤: {}".format(e))

if __name__ == '__main__':
    MoveGroupCommanderFromComplexJSON()

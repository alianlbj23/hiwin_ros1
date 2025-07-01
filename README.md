# HIWIN ROS MoveIt IK Demo

This package demonstrates a simple workflow for controlling a HIWIN robotic arm using ROS and MoveIt. It consists of two main nodes: one to publish a series of target 3D points, and another to calculate the Inverse Kinematics (IK) for these points and publish the corresponding joint angles **only when triggered**.

---

## 📦 Nodes

### `publish_postion_node.py`

This script acts as a simple test client. It publishes a hardcoded array of 3D points to the `/target_points` topic and then exits. This is used to simulate a stream of target coordinates for the robot arm.

- **Publishes to:** `/target_points`  
  Message type: [`custom_msgs/PointArray`](https://github.com/user/repo/blob/main/custom_msgs/msg/PointArray.msg)

---

### `hiwin_moveit_node.py`

This node handles IK computation and conditional joint publishing:

1. Subscribes to `/target_points` (or `Anchor`) to receive a list of 3D target points.
2. Computes inverse kinematics (IK) for all points via the `/compute_ik` service.
3. Waits for a signal from `/bool_topic` (`std_msgs/Bool`) to determine whether to publish the resulting joint angles.
4. If all points are reachable and the trigger signal is `True`, publishes the corresponding `JointState` messages to `/target_points_joints`.
5. Publishes an IK computation result (`True` or `False`) to `/ik_success`.

#### Behavior Summary

| Condition                                | Result                                                                 |
|------------------------------------------|------------------------------------------------------------------------|
| All points are reachable & `/bool_topic` is `True` | Publishes joint angles to `/target_points_joints`; sends `True` to `/ik_success` |
| Any point is unreachable (IK fails)      | Aborts processing; sends `False` to `/ik_success`                      |
| `/bool_topic` is `False`                 | Waits until it becomes `True` before proceeding                        |

- **Subscribes to:**
  - `/target_points` or `Anchor` (`custom_msgs/PointArray`)
  - `/bool_topic` (`std_msgs/Bool`)
- **Publishes to:**
  - `/target_points_joints` (`sensor_msgs/JointState`)
  - `/ik_success` (`std_msgs/Bool`)
- **Uses Service:**
  - `/compute_ik` (`moveit_msgs/GetPositionIK`)

---

## 🔧 Dependencies

Make sure the following packages are installed:

- `rospy`
- `moveit_ros_planning_interface`
- `sensor_msgs`
- `geometry_msgs`
- `std_msgs`
- `custom_msgs`  
  (This package requires a custom message definition for `Point` and `PointArray`.)

---

## ▶️ How to Run

1. **Launch your robot's MoveIt environment.**  
   This will start the necessary services, including `/compute_ik`.

    ```bash
    roslaunch my_hiwin_pkg demo.launch
    ```

2. **Run the IK solver node.**  
   This node will wait for target points and a trigger signal.

    ```bash
    rosrun my_hiwin_pkg hiwin_moveit_node.py
    ```

3. **Publish the target points.**

    ```bash
    rosrun my_hiwin_pkg publish_postion_node.py
    ```

4. **Trigger execution by publishing to `/bool_topic`:**

    ```bash
    rostopic pub /bool_topic std_msgs/Bool "data: true"
    ```

5. **Monitor the IK result feedback:**

    ```bash
    rostopic echo /ik_success
    ```

---

## 🧪 Message Format Examples

### `custom_msgs/PointArray.msg`

```text
geometry_msgs/Point[] points

# HIWIN ROS MoveIt IK Demo

This package demonstrates a simple workflow for controlling a HIWIN robotic arm using ROS and MoveIt. It consists of two main nodes: one to publish a series of target 3D points and another to calculate the Inverse Kinematics (IK) for these points and publish the corresponding joint angles.

## Nodes

### `publish_postion_node.py`

This script acts as a simple test client. It publishes a hardcoded array of 3D points to the `/target_points` topic and then exits. This is used to simulate a stream of target coordinates for the robot arm.

-   **Publishes to:** `/target_points` ([custom_msgs/PointArray](https://github.com/user/repo/blob/main/custom_msgs/msg/PointArray.msg))

### `hiwin_moveit_node.py`

This node subscribes to the `/target_points` topic. Upon receiving a list of points, it iterates through them and calls the `/compute_ik` service provided by MoveIt to find a valid joint configuration for each point.

-   If a valid IK solution is found for **all** points, it publishes the `sensor_msgs/JointState` for each point sequentially to the `/target_points_joints` topic.
-   If any point is unreachable (i.e., no IK solution is found), it logs a warning and aborts the process for the entire set of points.

-   **Subscribes to:** `/target_points` ([custom_msgs/PointArray](https://github.com/user/repo/blob/main/custom_msgs/msg/PointArray.msg))
-   **Publishes to:** `/target_points_joints` (`sensor_msgs/JointState`)
-   **Uses Service:** `/compute_ik` (`moveit_msgs/GetPositionIK`)

## Dependencies

-   `rospy`
-   `moveit_ros_planning_interface`
-   `sensor_msgs`
-   `geometry_msgs`
-   `custom_msgs` (This package requires a custom message definition for `Point` and `PointArray`).

## How to Run

1.  **Launch your robot's MoveIt environment.** This will start the necessary services, including `/compute_ik`.
    ```bash
    roslaunch my_hiwin_pkg demo.launch # Or your specific launch file
    ```

2.  **In a new terminal, run the IK solver node.** This node will wait for points to be published.
    ```bash
    rosrun my_hiwin_pkg hiwin_moveit_node.py
    ```

3.  **In another terminal, run the position publisher node.** This will send the target points to the IK solver.
    ```bash
    rosrun my_hiwin_pkg publish_postion_node.py
    ```

4.  Observe the terminal output from the `hiwin_moveit_node.py` to see the results of the IK calculation and the published joint angles.
# HIWIN ROS MoveIt IK JSON 介面

此 ROS package 提供一個透過 JSON 格式控制 HIWIN 機械手臂的流程。它包含兩個主要節點：一個是互動式命令列工具，用於發送目標點和控制信號；另一個是核心的 IK 運算節點，它根據收到的信號執行逆向運動學（IK）計算並發布關節角度。

此系統採用一個簡單的狀態機來管理流程：`IDLE` -> `GOT_POINTS` -> `PLANNED`。

---

## 📦 節點 (Nodes)

### `signal_publisher_node.py`

這是一個命令列介面的測試工具，讓使用者可以手動觸發流程中的各個步驟。

-   **功能:**
    1.  發送一組固定的 3D 座標點 (JSON 格式) 到 `/target_points_json` topic。
    2.  發送控制信號 (`plan`, `execute`, `stop`) 到 `/robot_signal` topic。
-   **發布 (Publishes to):**
    -   `/target_points_json` (`std_msgs/String`)
    -   `/robot_signal` (`std_msgs/String`)

---

### `hiwin_moveit_node_re.py`

這是處理 IK 運算和狀態管理的核心節點。

1.  **等待座標點:** 訂閱 `/target_points_json`。收到 JSON 格式的點位後，立即為所有點位計算 IK。
    -   若任一點位 IK 計算失敗，發布失敗訊息至 `/target_points_joints_json_tmp` 並重置流程。
    -   若全部成功，將計算結果 (關節角度) 暫存起來，並進入 `GOT_JSON` 狀態。
2.  **等待 Plan 指令:** 訂閱 `/robot_signal`。當收到 `plan` 指令時：
    -   發布暫存的關節角度到 `/target_points_joints_json_tmp` 作為預覽。
    -   進入 `PLANNED` 狀態。
3.  **等待 Execute 指令:** 當收到 `execute` 指令時：
    -   發布暫存的關節角度到 `/target_points_joints_json` 作為最終執行的目標。
    -   重置流程，回到初始狀態等待新的座標點。
4.  **Stop 指令:** 任何時候收到 `stop` 指令，都會立即中斷當前流程並重置狀態。

#### 行為摘要

| 目前狀態         | 收到信號 (`/robot_signal`) | 執行動作                                                               | 新狀態           |
| ---------------- | -------------------------- | ---------------------------------------------------------------------- | ---------------- |
| `IDLE`           | (收到 `/target_points_json`) | 計算所有點的 IK。若成功，暫存結果。                                    | `GOT_JSON`       |
| `GOT_JSON`       | `plan`                     | 發布關節角度到 `/target_points_joints_json_tmp`。                        | `PLANNED`        |
| `PLANNED`        | `execute`                  | 發布關節角度到 `/target_points_joints_json`，然後重置。                  | `IDLE`           |
| 任何狀態         | `stop`                     | 重置所有狀態與暫存資料。                                               | `IDLE`           |

-   **訂閱 (Subscribes to):**
    -   `/target_points_json` (`std_msgs/String`)
    -   `/robot_signal` (`std_msgs/String`)
-   **發布 (Publishes to):**
    -   `/target_points_joints_json_tmp` (`std_msgs/String`): IK 運算結果預覽。
    -   `/target_points_joints_json` (`std_msgs/String`): 最終確認要執行的關節目標。
-   **使用服務 (Uses Service):**
    -   `/compute_ik` (`moveit_msgs/GetPositionIK`)

---

## 🔧 依賴項 (Dependencies)

-   `rospy`
-   `moveit_ros_planning_interface`
-   `std_msgs`
-   `moveit_msgs`

*注意：此版本使用 `std_msgs/String` 傳遞 JSON，不再需要自定義訊息 (`custom_msgs`)。*

---

## ▶️ 如何執行

1.  **啟動機械手臂的 MoveIt 環境。**  
    這會啟動必要的服務，包含 `/compute_ik`。

    ```bash
    # 以您自己的啟動檔案取代
    roslaunch my_hiwin_pkg demo.launch
    ```

2.  **在一個終端機中，執行 IK 核心節點。**  
    此節點會開始等待來自 `/target_points_json` 的座標點。

    ```bash
    rosrun my_hiwin_pkg hiwin_moveit_node_re.py
    ```

3.  **在另一個終端機中，執行互動式命令節點。**

    ```bash
    rosrun my_hiwin_pkg signal_publisher_node.py
    ```

4.  **依照  的提示操作：**
    1.  輸入 `1` 發送目標點。
    2.  輸入 `2` 發送 `plan` 信號，觸發 IK 結果預覽。
    3.  輸入 `3` 發送 `execute` 信號，發布最終關節目標。
    4.  可隨時輸入 `4` (`stop`) 來重置流程。

5.  **(可選) 在其他終端機中監控 Topics：**

    ```bash
    # 監控 IK 預覽結果
    rostopic echo /target_points_joints_json_tmp

    # 監控最終執行的關節目標
    rostopic echo /target_points_joints_json
    ```

---

## 🧪 JSON 訊息格式範例

### `/target_points_json` (發送)

```json
{
  "points": [
    {"x": -0.499, "y": 0.876, "z": 0.796},
    {"x": -0.070, "y": 0.808, "z": 0.988}
  ]
}
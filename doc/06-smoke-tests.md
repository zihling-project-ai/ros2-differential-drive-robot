# 06 — ROS 2 程式環境與 GUI Smoke Test

> **狀態：** 2026-11-20 已於 WSL2 完成 **Turtlesim GUI／鍵盤控制**與 **rqt_graph 通訊圖**驗證（見 6.2、6.3 節）。Talker／Listener、Gazebo、實體 SLAM 尚未完成本輪驗收；不可直接標為 PASS。  
> **安全：** 先測純軟體 Node 與 Topic，不執行馬達、手臂或導航的實體控制。

## 6.1 測試前檢查

本機 WSL2：

```bash
whoami                       # zihling
hostname                     # LV333-06
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash  # 如果已經 colcon build
ros2 --help
ros2 pkg list | grep -E '^(ai_car|me333_pkg|turtlesim|rviz2)$'
```

AICAR 多主機測試沿用經驗證的設定（WSL2、Pi 每個新 Terminal 均一致）：

```bash
export ROS_DOMAIN_ID=6
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/192.168.123.2:7447"]'
```

先在 WSL2 保留**一個** `rmw_zenohd` Router，確認車端 Chrony 已同步，再跑以下測試。若以不同 RMW 做本機獨立實驗，應在隔離的 Shell 中明確記錄，不可直接拿不同 RMW 的 Topic 發現結果互相比較。

## 6.2 Turtlesim（小烏龜）

只需 WSL2 圖形桌面，無須讓實體車移動。

檢查是否已安裝：

```bash
ros2 pkg executables turtlesim
# 缺少且可連 Ubuntu APT 時才補裝
# sudo apt install ros-jazzy-turtlesim
```

WSL2 Terminal A：

```bash
ros2 run turtlesim turtlesim_node
```

WSL2 Terminal B（焦點在此 Terminal 才能使用方向鍵）：

```bash
ros2 run turtlesim turtle_teleop_key
```

WSL2 Terminal C：

```bash
ros2 node list --no-daemon
ros2 topic list -t --no-daemon
ros2 topic echo /turtle1/pose turtlesim/msg/Pose --once
```

**驗收證據（2026-11-20）：** Turtlesim GUI 已正常開啟；`turtle_teleop_key` 鍵盤操作已完成，小烏龜可以移動。`/turtle1/pose` 的單獨 CLI 讀取尚無本輪輸出紀錄，需另行驗證。視窗無法顯示時，先確認 WSLg／Windows 顯示環境與 `DISPLAY`，不要直接修改 ROS 2 Node。

## 6.3 rqt_graph：檢視 Node / Topic 連線

在 Turtlesim Node 與 Teleop 仍執行時，另開 WSL2 Terminal：

```bash
ros2 pkg prefix rqt_graph
ros2 run rqt_graph rqt_graph
```

### 2026-11-20 實測結果：PASS

在 WSL2 啟動 `turtlesim_node`、`turtle_teleop_key` 後，成功開啟 `rqt_graph`。畫面採用 **Nodes only** 檢視，確認兩個節點及其連線：

![Turtlesim 的 ROS 2 rqt_graph 實測畫面：teleop_turtle 和 turtlesim](assets/rqt-graph-turtlesim.png)

*圖 6-1：2026-11-20 WSL2 的 `rqt_graph` 實測截圖。*

截圖可直接辨識：

| 觀察項目 | 圖上內容 | 說明 |
|---|---|---|
| 發送端 Node | `/teleop_turtle` | 鍵盤 Teleop 節點 |
| 接收端 Node | `/turtlesim` | 小烏龜模擬器節點 |
| 速度 Topic | `/turtle1/cmd_vel`，由 `/teleop_turtle` 指向 `/turtlesim` | 驗證鍵盤速度命令的 Publisher → Subscriber 路徑 |
| 額外連線 | `/turtle1/rotate_absolute/_action/feedback`、`/turtle1/rotate_absolute/_action/status` | 圖中另顯示來自 Turtlesim 的 Action 回饋／狀態；**不是** `/cmd_vel` 控制路徑，也不代表本輪已測試旋轉 Action |

**驗收結論：** `rqt_graph` GUI 正常顯示，且預期的 `teleop_turtle → /turtle1/cmd_vel → turtlesim` 連線存在。此測試驗證本機 ROS 2 通訊關係與圖形介面；**不代表** Raspberry Pi 馬達、跨主機 Topic 或 SLAM 自身通過驗收。

如果畫面未顯示 Topic 名稱，可從上方檢視模式切換至 **Nodes/Topics (all)**，並點擊刷新。rqt_graph 是圖形化通訊圖，不是 SLAM 模擬器。

### Nodes/Topics (all)：顯示完整 Topic 與 Action 端點

將 `rqt_graph` 左上角由 **Nodes only** 切換為 **Nodes/Topics (all)** 後，可分辨 Topic 名稱、節點之間的資料方向，以及 Action 關聯的端點。

![rqt_graph — Nodes/Topics (all)：Turtlesim 的速度命令與 RotateAbsolute Action 相關連線](assets/rqt-graph-turtlesim-nodes-topics.png)

*圖 6-2：2026-11-20 WSL2 的 `rqt_graph` 完整 Topic 檢視（Nodes/Topics (all)）。*

| 圖上連線 | 方向與意義 | 驗證範圍 |
|---|---|---|
| `/turtle1/cmd_vel` | `/teleop_turtle` → `/turtlesim`，鍵盤控制的速度命令 | 已實測鍵盤移動及連線圖 |
| `/turtle1/rotate_absolute` | 提供旋轉至指定絕對角度的 Action 介面，圖上顯示節點之間相關通道 | 僅確認通訊圖可見 |
| `/turtle1/rotate_absolute/_action/feedback` | `/turtlesim` → `/teleop_turtle`，Action Feedback | 僅確認通訊圖可見 |
| `/turtle1/rotate_absolute/_action/status` | `/turtlesim` → `/teleop_turtle`，Action Goal 狀態 | 僅確認通訊圖可見 |

> **判讀限制：** `rqt_graph` 顯示通訊端點，不能單憑圖形認定旋轉 Action 已送出 Goal 或完成執行。實際完成的功能驗收是 Turtlesim GUI、方向鍵控制及 `/cmd_vel` 通訊路徑。

## 6.4 Demo Talker／Listener

兩個 WSL2 Terminal 分別執行：

```bash
# Terminal A
ros2 run demo_nodes_cpp talker
```

```bash
# Terminal B
ros2 run demo_nodes_py listener
```

額外用 CLI 查看：

```bash
ros2 topic list -t --no-daemon
ros2 topic echo /chatter std_msgs/msg/String --once
```

**預期：** Listener 收到 Talker 的文字。這確認基本 Publisher／Subscriber 與所選 RMW；但不代表跨主機 Zenoh 已通。

跨主機的實測方式請使用 [04 — 雙向 Topic 驗證](04-zenoh-firewall.md) 中 `/aicar_link_test`、`/aicar_reverse_test`，確認各方向都能接收。

## 6.5 `gz` / Gazebo（可能的「Z 開頭工具」；待確認）

教學中提及的另一個視覺化／模擬工具名稱尚未確認。若指的是 Gazebo（常見指令前綴為 `gz`），可**先查有無安裝**：

```bash
command -v gz
# 找到 gz 才查說明
# gz sim --help
```

此處不擅自宣稱原教學一定使用 Gazebo，也不因看不到 `gz` 就要求重裝。`gz`/Gazebo 偏向物理與感測器模擬；`rmw_zenohd` 則是 ROS 2 的通訊 Router，兩者作用不同。

## 6.6 專案 Package、Launch 與 RViz 檢查

本輪已確認 Workspace 內存在：

```text
src/me333_pkg/launch/turtlesim.launch.py
src/ai_car/launch/aicar1_launch.py
src/ai_car/launch/rviz2_launch.py
src/ai_car/launch/slam_launch.py
src/ai_car/launch/nav2_launch.py
src/ai_car/config/slam.yaml
src/ai_car/config/nav2.yaml
src/ai_car/urdf/ai_car.urdf
```

**檔案存在不代表執行成功。** 先確認套件與安裝資源：

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 pkg list | grep -E '^(ai_car|me333_pkg)$'
ros2 pkg executables ai_car
ros2 pkg prefix me333_pkg
ls src/me333_pkg/launch/
ls src/ai_car/launch/
```

在使用 `ros2 launch me333_pkg turtlesim.launch.py` 前，先閱讀該檔內容並確認 `setup.py` 已安裝其 launch 資源；不能只看到原始碼檔名就認為 `ros2 launch` 能找到它。

### RViz2、TF、SLAM 套件是否可用

```bash
ros2 pkg prefix rviz2
ros2 pkg prefix slam_toolbox
ros2 pkg prefix nav2_bringup
ros2 pkg prefix tf2_ros
```

如果只驗證 GUI，可先執行 `rviz2` 開啟空白介面，確認畫面可顯示。涉及 `/scan`、`/odom`、`map → odom`、`odom → base_footprint`、URDF 的檢查要在相應節點存在時進行，**不在本章自動啟動硬體控制 Node**。

## 6.7 進入實體車測試前的門檻

依序確認：

1. 兩主機 IP / SSH / Zenoh TCP 7447 正常；雙向 Topic PASS。
2. 車端 Chrony 正在同步，WSL2 與車端時間差可接受；無持續 Timestamp 警告。
3. 讀取 `aicar_base.py`、`dxl_wheel.py`、`dxl_arm.py` 的上電初始化與停機處理；確保沒有多個程序同時占用 U2D2。
4. 實際馬達／輪子測試必須安排實體隔離、緊急斷電與命令逾時停車措施；不要僅依賴 `Ctrl+C`。
5. 感測器、`/joint_states`、`/imu`、`/odom`、TF、`/scan` 按順序核對，再執行 SLAM；Nav2 放最後。

## 6.8 結果記錄表

| 驗收項目 | 預期證據 | 本輪狀態 |
|---|---|---|
| ROS 2 CLI / Package | `ros2 --help`、`ros2 pkg list` | 待本輪逐項查核 |
| Turtlesim GUI / Teleop | GUI 已開啟、鍵盤可控制移動 | **PASS（2026-11-20）**；`/turtle1/pose` CLI 待查 |
| rqt_graph | 兩個 Node、`/turtle1/cmd_vel` 路徑與截圖 | **PASS（2026-11-20）**，見圖 6-1、圖 6-2 |
| Talker / Listener | `/chatter` 接收 | 待測 |
| `gz` / Gazebo | 安裝與指令版本 | 工具名稱待確認 |
| Zenoh WSL2 ↔ car06 | 雙向 String Topic | **PASS** |
| Raspberry Pi NTP | `timedatectl synchronized=yes` | **PASS** |
| RViz / SLAM / Nav2 | 依專案驗證程序 | 尚未納入本輪驗收 |

## 後續文件（已新增）

後續程式學習可獨立拆分為：（1）ROS 2 通訊原理與 AICAR Topic/TF，（2）老師第 7 章《ai_car 套件》原文對照，（3）逐檔解析 `aicar_base.py`、IMU、Odometry、Wheel、Arm、SLAM、Nav2。原教材全文與實際原始碼未逐檔核對之前，不將推測當作程式實作事實。

**返回：** [README](../README.md)。


## 6.9 2026-11-20 補充驗收（以當日實驗輸出為準）

- ROS 2 Talker/Listener `/chatter`：**PASS**（本機測試）。
- Gazebo Harmonic 8.15.0：**PASS**（`empty.sdf` 開啟、AICAR URDF/SDF 車體及 Mesh 顯示、重力與地面接觸）。
- Gazebo JointStatePublisher：**PASS**（`arm_joint_1`、`arm_joint_2`、`tool_joint` 位置／速度取得）。
- JointPositionController：**NOT PASS**（末端關節速度約 ±50 rad/s 交替，底盤偏移；根因待查）。
- RViz2：**GUI PASS / RobotModel NOT PASS**。最後截圖只有 Grid、Fixed Frame=`map` 且出現警告；尚未完成模型與 TF 顯示。
- `ros_gz_bridge`：**NOT TESTED**。

詳細依據見 [07 — Gazebo 與 RViz2 實驗](07-gazebo-rviz2-aicar-experiment.md)。不可將「開啟軟體」誤列為整套資料流驗收通過。

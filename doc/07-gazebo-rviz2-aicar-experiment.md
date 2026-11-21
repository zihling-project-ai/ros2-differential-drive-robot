# 07｜AICAR Gazebo 與 RViz2 模擬實驗

- **日期**：2026-11-20（Asia/Taipei）
- **平台**：學校電腦 Windows 11／WSL2 Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic（Gazebo Sim 8.x）、RViz2
- **對象**：AICAR 四輪底盤＋三個手臂／工具關節
- **目的**：釐清 URDF、Gazebo 物理模擬、ROS 2 Topic／TF、RViz2 視覺化之間的責任分工
- **執行範圍**：本機 WSL2 的模擬實驗；**未驗證實體車的馬達控制**
- **驗證結論**：Gazebo 模型與關節狀態讀取完成；關節位置控制出現不穩定振盪及車體漂移；RViz2 視窗已啟動，但**模型顯示尚未通過驗收**。
- **可重現性界線**：以下 SDF 與 RViz 專用 URDF 是歷史本機實驗副本，尚未納入 Repository；目前可取得正式 URDF 與 Mesh，但不能只 Clone 就完整重現這些實驗。

## 1. 系統架構與角色

```text
AICAR URDF + Mesh
    ├─> Gazebo Sim（物理世界）
    │     ├─ Physics：重力／接觸／關節運動
    │     ├─ JointStatePublisher（Gazebo Transport）
    │     └─ JointPositionController（本次僅模擬 tool_joint）
    │
    └─> robot_state_publisher（ROS 2）
          ├─ /robot_description
          └─ TF（搭配 /joint_states）
                 └─> RViz2（顯示模型、TF 與 ROS 2 感測資料）

未完成的連接：
Gazebo joint_state --ros_gz_bridge--> ROS 2 /sim/joint_states
                                 --> 對應的 TF --> RViz2
```

**核心差別**：Gazebo 會求解物理運動，RViz2 不會。RViz2 根據 ROS 2 收到的機器人描述、TF 與關節狀態呈現模型。`/joint_states` 可來自實機、模擬器或手動測試，僅憑畫面不能判定資料來源。

## 2. 實作內容與證據

### 2.1 Gazebo 模型載入與物理測試 — PASS

- AICAR URDF 轉成 SDF；保留原始 `urdf/ai_car.urdf` 不修改。
- Mesh 已能在 Gazebo 內顯示。
- 模型生成後，Play 使其受重力與地面接觸作用；曾觀察到末端工具關節自然轉動。
- Gazebo 可查詢 `/world/empty/pose/info`、`/world/empty/dynamic_pose/info`。
- 最後截圖左側 Gazebo 可見 `aicar_hold_test` 模型、地面與車體。截圖當下介面呈暫停狀態，不能據此判斷模型當時是否正在移動。

歷史實驗指令（需已安裝相應套件及另行取得本機 SDF）：

```bash
export GZ_SIM_RESOURCE_PATH="$HOME/ros2_ws/src${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
gz sim -v 3 empty.sdf
```

另一個 WSL2 終端：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run ros_gz_sim create \
  -world empty \
  -file "$HOME/ros2_ws/src/ai_car/gazebo_test/aicar_jointstate.sdf" \
  -name aicar_jointstate_test \
  -z 0.05
```

### 2.2 Gazebo JointStatePublisher — PASS

由 `gz topic -i` 確認 Gazebo Topic 存在且發布訊息型態為 `gz.msgs.Model`；`No subscribers` 只代表查詢當下沒有訂閱者，並非發布失敗。

正式觀察到的三個關節數值（單次截取）：

| 關節 | Position（rad） | 約略角度 | Velocity（rad/s） | 解讀 |
|---|---:|---:|---:|---|
| `arm_joint_1` | -5.30414×10^-14 | 0° | -3.97725×10^-17 | 接近靜止 |
| `arm_joint_2` | -1.24400×10^-13 | 0° | -7.04688×10^-17 | 接近靜止 |
| `tool_joint` | 1.57 | 89.95° | -4.89164×10^-10 | 接近上限、靜止 |

**注意**：之前沒有顯示 position/velocity，後來在模擬進行時取得數值。這與「尚未更新或預設零值欄位不顯示」的情況相容，**未對底層原因做獨立驗證**。

查詢：

```bash
gz topic -e \
  -t /world/empty/model/aicar_jointstate_test/joint_state
```

### 2.3 關節位置控制（JointPositionController）— NOT PASS

建立獨立副本 `gazebo_test/aicar_position_control.sdf`，只針對 `tool_joint` 新增位置控制器，沒有修改原始 URDF。測試命令目標位置為 `0.0 rad`。

本次控制器參數：

```text
joint_name       = tool_joint
initial_position = 0.0
p_gain           = 5.0
i_gain           = 0.0
d_gain           = 0.2
cmd_max          = 2.0
cmd_min          = -2.0
```

`tool_joint` 原 URDF：

```text
axis     = (0, 0, 1)
lower    = 0.0 rad
upper    = 1.57 rad
velocity = 50.0 rad/s
effort   = 1000.0（旋轉關節之作用力矩上限宣告）
dynamics = 未定義
```

控制器啟用後，曾連續觀察：

| 樣本順序 | tool_joint Position（rad） | Velocity（rad/s） |
|---|---:|---:|
| 1 | 0.0751963 | -50.0000000 |
| 2 | 0.1251963 | +50.0000000 |
| 3 | 0.0751963 | -50.0000000 |

**結果**：未穩定收斂至 0 rad，觀察到反向高速振盪。樣本沒有時間戳間隔，因此不能由三筆資料計算振盪頻率或完整動態響應。`cmd_max` / `cmd_min` 為控制器 PID 輸出限制，不等於關節的 `velocity` 上限；URDF `effort=1000` 也不代表實際輸出達 1000 N·m。

### 2.4 底盤漂移 — OBSERVED / CAUSE UNCONFIRMED

在含控制器的 `aicar_hold_test` 模型上量到：

```text
model: aicar_hold_test
x = -0.049440894 m
y = +0.558517009 m
z =  0.000007150 m
```

相較於初始 (x, y) ≈ (0, 0)，表示底盤有顯著位移，尤其 Y 方向約 +55.9 cm。**這是位移觀測，並不自動表示「持續倒車」**；當次兩筆 Pose 一致，可能因模型已暫停。

優先假說：末端關節的振盪力矩、接觸摩擦、輪胎碰撞或物理求解共同造成非預期滑移。**尚未以等時長、相同初始條件的有／無控制器 A/B 實驗驗證，不能認定 PID 是唯一原因。**

### 2.5 RViz2 — WINDOW PASS / ROBOT MODEL NOT PASS

最後截圖的實際情況：

- RViz2 視窗已成功啟動。
- 右側只有 Grid；**沒有顯示 AICAR 車體**。
- `Global Options → Fixed Frame` 仍為 `map`，且 Global Status 為 Warn、Fixed Frame 有錯誤提示。
- 左側 Displays 未見 `RobotModel` 項目。
- 因此**不能把 RViz2 模型、TF 或 Mesh 顯示記成 PASS**；前面的實驗僅證明程式 GUI 能啟動。
- 截圖同時仍有 Gazebo 開啟，兩個視窗同時存在不代表兩套系統已橋接。

當天曾準備本機 RViz 專用 URDF 副本 `gazebo_test/aicar_rviz.urdf`，將 `model://ai_car/meshes/` 轉為本機 `file://` 路徑，並規劃透過 `robot_state_publisher` 加 `/joint_states` 手動值展示模型；**尚未從截圖確認完整資料流已生效**。

## 3. RViz2 觀測鏈驗收（待驗證）

此為後續觀測鏈驗收方案，尚未執行完畢。保持與實車隔離，不必執行 Gazebo 控制器。

1. 在相關 WSL2 終端使用相同且獨立的 ROS 2 環境，例如 `ROS_DOMAIN_ID=76`、`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`、`ROS_LOCALHOST_ONLY=1`，並清除 `ZENOH_CONFIG_OVERRIDE`。另確認本機 Fast DDS 套件可用。
2. 啟動使用 `aicar_rviz.urdf` 的 `robot_state_publisher`；確認 `/robot_description` 與 TF 有資料。
3. 以獨立測試 Publisher 發送包含四輪與三個手臂關節的 `sensor_msgs/msg/JointState`。
4. RViz2 的 Fixed Frame **改成 `base_footprint`**（不是 `map`）；`Add → RobotModel`，Description Source 使用 Topic，選 `/robot_description`。
5. 確認車身、四輪與手臂 Mesh 顯示完整，並更改 `tool_joint` 的手動數值（例如由 0 改為 1.57 rad）觀察視覺姿態更新。
6. 只有上述全部成功，才將 RViz2 模型顯示標示 PASS；再規劃 `ros_gz_bridge` 連接真正 Gazebo 的關節狀態，優先使用 `/sim/` 命名空間，避免與實體 `/joint_states` 混用。

可先檢查：

```bash
ros2 topic list | grep -E 'robot_description|joint_states|tf'
ros2 topic echo /joint_states --once
```

## 4. 後續控制實驗（未納入目前核心驗收）

- **B1：無控制器對照**：同場景、同起始姿態、同樣模擬時間，量測車體 x/y 變化，釐清是否只有加入控制器才偏移。
- **B2：受限閉環控制**：核對質量／轉動慣量、物理時間步長、力矩／速度約束、關節阻尼及接觸參數，再由低輸出、低增益開始整定；記錄目標、位置、速度及收斂結果。
- **B3：實車對映**：將 Gazebo 假設與實際 DYNAMIXEL 的扭矩啟用、Goal/Present Position、速度／負載能力分開驗證。未實測前不得將此模擬稱作經驗證的 Digital Twin。

## 5. 實驗驗收矩陣與工程判斷

| 主題 | 狀態 |
|---|---|
| URDF / Mesh 載入 Gazebo | PASS |
| Gazebo 重力、接觸及模型 Pose 觀察 | PASS（定性觀察） |
| Gazebo `JointStatePublisher` 三個關節讀數 | PASS |
| `tool_joint` 位置控制穩定收斂 | NOT PASS |
| 控制器導致底盤偏移的因果驗證 | NOT TESTED |
| RViz2 GUI 啟動 | PASS |
| RViz2 AICAR RobotModel／TF 顯示 | NOT PASS |
| Gazebo → ROS 2 Bridge → RViz2 | NOT TESTED |
| 實體 AICAR 位置控制／物理模型一致性 | NOT TESTED |

**設計判斷**：

> URDF 描述機械結構；Gazebo 在物理世界中計算運動；控制器按照目標與回饋產生控制輸出；ROS 2 傳遞狀態與命令；RViz2 視覺化 ROS 2 的模型與資料。能看到機器人，不等於物理模擬正確；能啟動控制器，不等於控制穩定；能開啟 RViz2，不等於機器人模型已載入。

**目前取捨**：暫不整定 PID 或宣稱數位分身已完成。先確立可觀測且可驗證的資料流，再決定是否擴充物理模型或橋接。模擬參數不能直接視為實車校正值。

## 6. 檔案與版本注意事項

- 原始：`~/ros2_ws/src/ai_car/urdf/ai_car.urdf`（不修改）。
- 測試：`gazebo_test/aicar_jointstate.sdf`、`gazebo_test/aicar_position_control.sdf`、`gazebo_test/aicar_rviz.urdf`（依先前操作產生；本紀錄**沒有重新連線查證本機檔案現在是否仍存在**）。
- Repository 收錄正式 URDF、Mesh 與本實驗紀錄；本機 SDF 副本不在版本控制內。若未來重現，應將可分發的實驗模型及版本一併管理。

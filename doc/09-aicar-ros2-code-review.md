# 09｜AICAR ROS2 程式碼模組教學與審查

更新日期：2026-11-21　｜　狀態：第一輪靜態審查／逐段學習中

> **重要：審查基準更新。** 下方第一輪行號指向先前獨立的 278 行硬體停用安全骨架，不是本 Repository 的正式 383 行 `src/ai_car/ai_car/aicar_base.py`。不得將舊版 `hardware_enabled=False` 特性誤認為正式版本已具備的安全功能。目前手臂驅動已合併 PR #5，車輪驅動重構位於草稿 PR #6；兩者均尚未完成實車驗收。

## 0. Repository 目錄與同名程式比較

### 目錄責任圖

```text
ros2-differential-drive-robot/    # 工程專案／ROS2 workspace
├── .github/workflows/           # 自動建置與驗證（CI）
├── .vscode/                     # 編輯器開發設定
├── docker/                      # 可重現的 Jazzy 執行環境
├── doc/                         # 設計決策、實驗與驗收證據
├── src/
│   └── ai_car/                  # ROS2 package
│       ├── ai_car/              # 正式 Python 模組：控制、驅動、感知、里程計
│       ├── config/              # SLAM / Nav2 參數
│       ├── launch/              # ROS2 nodes 的啟動與協調
│       ├── urdf/                # 機器人結構、關節與物理模型
│       ├── meshes/              # URDF 引用的 3D 幾何模型
│       ├── rviz/                # 可視化配置
│       ├── resource/            # Ament 套件索引標記
│       ├── share/               # 教師參考解答（暫保留，不作正式執行入口）
│       ├── package.xml          # 套件身分、建置類型與依賴
│       ├── setup.py / setup.cfg # 安裝規則、執行入口與位置
│       └── 99-aicar.rules       # Linux 裝置識別與存取規則
├── .gitignore / .dockerignore   # 排除建置產物與無關檔案
└── README.md                    # 專案入口、狀態與文件索引
```

**設計判斷：** 區分工程環境、機器人實作、部署配置與驗證紀錄。正式程式只從內層 `ai_car/` 演進；教師參考解答應逐項比較優缺點，確認功能與安全性後再統一為一套實作，避免長期維護兩套程式。

### 正式程式與參考解答

`src/ai_car/setup.py` 的 `console_scripts` 指向 `ai_car.aicar_base`、`aicar_imu`、`aicar_odom`、`aicar_teleop`；`find_packages()` 安裝 Python package。`data_files` 中的 `share/ai_car` 是安裝後資源目錄，不是來源中的 `src/ai_car/share/`。目前追蹤檔內未發現使用來源 `share/` 的引用。

| 同名 Python 檔 | `share/` / `ai_car/` 行數 | 比較 |
| --- | ---: | --- |
| `aicar_base.py` | 238 / 383 | 非同版；參考版有 `calc_diff()`，正式版展開回呼及 Encoder 程式 |
| `aicar_imu.py` | 88 / 88 | 位元組完全一致 |
| `aicar_odom.py` | 139 / 192 | 非同版；依賴匯入、回呼與實作不同 |
| `aicar_teleop.py` | 198 / 309 | 非同版；正式版鍵盤與手臂發布流程不同 |
| `ball_detector.py` | 156 / 128 | 非同版；影像回呼和偵測流程有差異 |
| `lidar_follow.py` | 88 / 29 | 正式版多處 `pass`；參考版有 `/scan` 訂閱與較完整內容 |
| `lidar_sub.py` | 97 / 87 | 正式版保留 TODO/pass，兩者不能視為等價 |

結論：來源 `share/` 目前並非正常 ROS2 套件執行入口；可以考慮在確認有用差異後移除，但目前保留，不直接將參考版本覆蓋正式程式。尤其 LiDAR 模組需先逐段比較與測試。`me333_pkg` 已移除，原始教材可由 Git 歷史找回。

## 1. 目的與審查範圍

先理解程式如何運作、每個模組的責任與介面，再確認問題、設計測試及修改。未確認原因前，不為了消除警告而隨意改動程式。

| 來源 | 狀態 |
| --- | --- |
| `aicar_base.py`（278 行） | 已逐行讀取；Python 語法檢查通過，尚未做 ROS2 執行測試 |
| 老師公開的 [ai_car 倉庫](https://github.com/zhl017/ai_car) | 公開首頁顯示 ROS1 Noetic；不能當作目前 ROS2 原始碼 |
| DYNAMIXEL 驅動、Odometry/IMU、URDF、Launch、YAML、Zenoh | 完整 ROS2 原始碼與配置待補 |

目前取得的是一份硬體停用版 `aicar_base.py`，不是已驗證的正式實車版本。以下行號以本次取得檔案為準。P0/P1/P2 表示修復或驗收優先級，而非事故發生機率。

## 2. ROS2 模組地圖

| 模組 | 主要責任 | 典型資料介面 |
| --- | --- | --- |
| Teleop / Planner | 產生運動需求 | `cmd_vel` (`Twist`) |
| Base Control | 解讀需求、協調底層驅動與狀態 | `cmd_vel`、`joint_states` |
| DYNAMIXEL Driver | 硬體專屬通訊與回饋驗證 | Controller API（待確認） |
| Odometry / IMU | 量測移動和姿態 | `odom`、`imu/data` |
| State Estimation | 感測器融合 | Pose、TF |
| URDF / State Publisher | 車體與關節座標關係 | `robot_description`、`tf`、`tf_static` |
| Gazebo | 物理與感測器模擬 | Gazebo Transport、`ros_gz` |
| RViz2 | 顯示 ROS2 資料 | RobotModel、TF、Odometry、感測器 |
| Zenoh | ROS2 節點間資料傳遞 | RMW、Router、QoS |

實車與模擬應共用明確的 ROS2 資料契約，但是各自獨立提供資料，不應同時發布互相競爭的控制或狀態來源。相同 Topic 並不代表動力學已一致。

### 底層驅動模組：Arm 與 Wheel

**模組目標：** `dxl_arm.py` 與 `dxl_wheel.py` 將設備通訊、型別轉換及有效資料回報封裝成 Python 介面；兩者不是 ROS2 Node。`aicar_base.py` 管理共用連線與操作時機，驅動層不負責 Topic、里程計積分或自主決策。硬體人工診斷另外放在 `diagnostics/`，離線測試放在 `test/`。

| `dxl_arm.py`（位置控制） | 介面與責任 |
| --- | --- |
| `angle_to_ticks(angle_deg, max_angle_deg)` | 驗證角度指令（度），轉為 Tick 增量 |
| `validate_position_tick(tick)` | 驗證 AX 原始位置數值的編碼範圍 |
| `__init__(port)` | 使用上層提供的 Port，設定既有速度；不自動下達復位位置命令 |
| `_check_result(...)` | 檢查 SDK 通訊與裝置錯誤 |
| `reset()` | 由呼叫者明確要求手臂回 Home Pose |
| `set_torque(set_data)` | 驗證 0/1，處理 Torque 要求並檢查每個結果 |
| `set_joint(set_data)` | 接收兩個關節角度（度），使用既有零點與方向換算位置 |
| `set_tool(set_data)` | 接收工具角度（度），使用同樣的驗證與換算原則 |
| `get_arm()` | 驗證並回傳三個實際關節位置（弧度） |

| `dxl_wheel.py`（速度控制） | 介面與責任 |
| --- | --- |
| `to_int32(value)` / `to_uint32(value)` | 驗證並互轉有號數值與固定 32-bit 位元表示；不改變物理單位 |
| `__init__(port)` | 使用共用 Port，確認既有速度控制模式、要求停止；不自動啟用 Torque |
| `_check_result(...)` | 檢查 SDK 通訊與裝置錯誤 |
| `_verify_velocity_mode()` | 唯讀確認馬達模式，不改寫已校正設定 |
| `reset()` | 要求四輪停止，嘗試全部停止並回報錯誤；不重設實體 Encoder 或控制模式 |
| `set_torque(set_data)` | 驗證並處理明確的 Torque 要求 |
| `set_wheel(set_data)` | 接收左右側有號速度原始值；驗證後依既有對應編碼與通訊 |
| `get_wheel()` | 驗證並回傳四輪有號 Encoder Tick，順序為左前、右前、左後、右後 |

**設計決策與驗證：** 保留原有硬體 ID、零點、方向、協定及換算參數，不新增配置框架。關節的度數與弧度、車輪的有號物理值與無號封包表示分開處理。PR #5（Arm）已合併，PR #6（Wheel）仍為草稿；目前 22 項假 SDK 離線測試通過。實際機械限制、模式與停止狀態須現場確認，Base 的明確啟用、故障處理及 Encoder 基準管理尚待整合。

## 3. `aicar_base.py` 逐段閱讀路線

| 順序 | 行號 | 學習內容 | 讀完應能解釋 |
| --- | --- | --- | --- |
| 01 | 1–36 | 檔案說明、import、常數、`constrain()` | 模組的責任、單位、速度限制 |
| 02 | 38–95 | Node 初始化、參數、Topic、Timer | 為什麼硬體預設停用；Publisher/Subscriber |
| 03 | 97–146 | Reset、Torque、Joint、Tool、Twist callbacks | 收到訊息和執行控制的區別 |
| 04 | 148–202 | Timer、逾時、故障、Reset 狀態 | 命令生命週期與失效處理 |
| 05 | 204–231 | 編碼器與 JointState | 角度、增量、基準、有效量測 |
| 06 | 233–278 | Shutdown 與 main | Node 啟動、spin、資源清理 |

學習流程：先讀原碼 → 用自己的話解釋 → 找出反例或測試 → 決定修復 → 重跑測試 → 更新本文。每次只改已理解的問題。

## 4. 第一輪程式碼審查

### 4.1 P0 — 實車部署阻擋條件

| ID | 檔案位置 | 觀察、影響與建議 |
| --- | --- | --- |
| P0-01 | L53–57、85–95 | `hardware_enabled=True` 會直接拒絕啟動，`_connect_hardware()` 尚未實作。這是刻意的保護，不是目前可運行的馬達故障。保留停用狀態，待完整驅動、硬體型號與安全驗收完成才能部署。 |
| P0-02 | L164–186 | 未來的硬體分支處理 torque 與一般命令時，未見明確的運行狀態互鎖。需要設計停用、待命、運行、故障的安全狀態規則並驗證。此分支目前不可達。 |

### 4.2 P1 — 高優先級

| ID | 檔案位置 | 觀察／修復方向 |
| --- | --- | --- |
| P1-01 | L80、144、148–156 | 逾時以 monotonic clock 衡量，卻由 ROS Timer 檢查；阻塞或模擬時間暫停可能推遲檢查。安全監控須與控制處理的失效情境一起設計、測試。 |
| P1-02 | L118–129 | 手臂與工具指令只驗證部分格式，未見機械限制；需從實際硬體規格與驅動確認限制，不能猜測。 |
| P1-03 | L102–109、172–174、198–202、221–227 | Reset 後的 Timer 再次清除 encoder baseline 狀態，可能使一筆增量被忽略；統一重設語義及基準建立時機。 |
| P1-04 | L222–226 | 小於等於 2 ticks 的差分被歸零，且仍更新 last encoder；反覆微小移動可能永久丟失。保留原始有效增量，區分量測雜訊與位移。 |
| P1-05 | L190–192、215–230 | 讀取四輪卻只發布前兩輪增量；這並非必然錯誤，但須核對 Odometry 下游採用的運動模型及後輪角色。 |
| P1-06 | L190–192 | `Float32MultiArray` 沒有 Header，無法在訊息中直接標記取樣時刻；後續融合需有一致的時間與單位契約。 |

### 4.3 P2 — 架構與維護

| ID | 檔案位置 | 觀察／修復方向 |
| --- | --- | --- |
| P2-01 | L26–31 | 幾何、通訊及換算常數寫死。建議用不同部署配置與經驗證的參數來源；不要把 URDF 宣告值當成實車量測。 |
| P2-02 | L68–71 | Joint 名稱寫死；需逐項與 URDF 比對。 |
| P2-03 | L72–79 | 絕對 Topic 名稱不易部署多車 Namespace；後續可設計相對名稱與 Remapping。 |
| P2-04 | L138–143 | 兩側命令獨立限制，飽和時可能改變預期曲率；先在無硬體的運動學測試中理解並確認需求。 |
| P2-05 | L207–230 | 已有長度和有限數檢查，但回饋是否新鮮、編碼器單位／方向是否正確取決於尚未取得的驅動與配置。 |

### 4.4 值得保留的設計

- 硬體預設停用，且沒有不實的感測器回饋。
- 非有限的 Twist 訊息不會被接受為有效運動要求。
- 資料驗證、命令逾時、故障狀態及資源清理已有清楚的初步概念。
- 驅動尚未驗證前拒絕連線，不假設 stub 回傳的零是真實量測。

## 5. 待完整原碼審查的模組

| 主題 | 應檢查的介面與證據 |
| --- | --- |
| DYNAMIXEL | 型號、協定、ID、量測單位、裝置錯誤處理、安全限制、故障與停用驗收 |
| Odometry | 輪徑與輪距、四輪模型、符號、時間差、Pose/Twist、covariance、重設語義 |
| IMU | 單位、座標方向、安裝姿態、校準、時間戳、covariance、是否具有可信 orientation |
| TF | `map → odom → base_link` 的發布者責任；URDF link/joint 名稱一致；每條 TF 不應重複發布 |
| ROS2 配置 | `package.xml`、依賴、Launch、參數預設、模擬／實車隔離、QoS |
| Zenoh | Router、Domain ID、RMW、跨主機配置、時鐘、QoS、隔離與失效驗收 |
| Gazebo | URDF/SDF、mesh、質量／慣量、關節、Bridge、`/clock`、模擬控制與資料一致性 |

RViz2 的 `Fixed Frame` 必須存在於 TF Tree。模型顯示階段可先使用 URDF 的根 frame（例如 `base_link`），不要假造 `map`。`robot_state_publisher` 根據 URDF 與 JointState 提供關節座標。

## 6. 測試與驗收計畫（待執行）

| 測試 | 預期結果 | 環境 |
| --- | --- | --- |
| Python 語法檢查 | 通過 | 已完成 |
| Node 硬體停用啟動 | 不連硬體、不發布實測回饋 | ROS2 模擬容器 |
| 合成資料單元測試 | 量測長度、單位、基準與積分行為符合契約 | ROS2 模擬容器／純運算測試 |
| URDF / TF | 關節名稱一致，無缺失的必要 frame | Gazebo / RViz2 |
| Bridge 與時鐘 | Topic 型別、方向、時間一致 | Gazebo / ROS2 |
| 跨主機資料測試 | 使用相容 middleware、QoS、正確時間 | 隔離的測試網路 |
| 實車驗收 | 由有經驗人員在安全條件下驗證、包括獨立保護與量測 | 學校 |

尚未執行的測試不能標示為通過。需要取得正式 ROS2 原始碼後，更新所有檔名、行號與結論。

## 7. 學習筆記與修訂紀錄

| 日期 | 閱讀段落 | 問題／理解 | 修訂狀態 |
| --- | --- | --- | --- |
| 2026-11-21 | L1–36 | 初始化前先理解模組責任、參數及單位 | 閱讀中；原碼未修改 |

## 8. 參考資料

- [ROS2 Jazzy robot_state_publisher](https://github.com/ros/robot_state_publisher/tree/jazzy)
- [ROS2 Jazzy rmw_zenoh](https://github.com/ros2/rmw_zenoh/tree/jazzy)
- [Gazebo 與 ROS 相容版本](https://gazebosim.org/docs/harmonic/ros_installation/)
- [10｜跨環境部署與 Docker](10-aicar-docker-deployment.md)

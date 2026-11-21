# 09｜AICAR ROS2 程式碼模組教學與審查

更新日期：2026-11-21　｜　狀態：第一輪靜態審查／逐段學習中

> **重要：審查基準更新。** 下方第一輪行號指向先前獨立的 278 行硬體停用安全骨架，不是本 Repository 的原始正式 `src/ai_car/ai_car/aicar_base.py`。不得將舊版 `hardware_enabled=False` 特性誤認為正式版本已具備的安全功能。目前手臂驅動已合併 PR #5，車輪驅動與 Base Node 重構位於草稿 PR #6；均尚未完成實車驗收。

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
| 早期 `aicar_base.py` 安全骨架（278 行） | 歷史靜態審查基準；不是目前正式程式 |
| 本 Repository `ai_car/ai_car/aicar_base.py` | 已逐段審查並於 PR #6 重構；離線單元測試通過，待實車驗收 |
| 老師公開的 [ai_car 倉庫](https://github.com/zhl017/ai_car) | 公開首頁顯示 ROS1 Noetic；不能當作目前 ROS2 原始碼 |
| DYNAMIXEL 驅動、Odometry/IMU、URDF、Launch、YAML | 已取得本 Repository 原始碼；驅動已重構，其餘模組尚待逐段審查 |

P0/P1/P2 表示修復或驗收優先級，而非事故發生機率。第 4 節保留先前安全骨架的歷史審查結果；目前實作以 PR #6 為準。

## 2. ROS2 模組地圖

| 模組 | 主要責任 | 典型資料介面 |
| --- | --- | --- |
| Teleop / Planner | 產生運動需求 | `cmd_vel` (`Twist`) |
| Base Control | 解讀需求、協調底層驅動與狀態 | `cmd_vel`、`joint_states` |
| DYNAMIXEL Driver | 硬體專屬通訊與回饋驗證 | Controller API（待確認） |
| Odometry / IMU | 量測旋轉速度、估算位姿 | `/odom`、`/imu`（本專案） |
| State Estimation | 感測器融合 | Pose、TF |
| URDF / State Publisher | 車體與關節座標關係 | `robot_description`、`tf`、`tf_static` |
| Gazebo | 物理與感測器模擬 | Gazebo Transport、`ros_gz` |
| RViz2 | 顯示 ROS2 資料 | RobotModel、TF、Odometry、感測器 |
| Zenoh | ROS2 節點間資料傳遞 | RMW、Router、QoS |

實車與模擬應共用明確的 ROS2 資料契約，但是各自獨立提供資料，不應同時發布互相競爭的控制或狀態來源。相同 Topic 並不代表動力學已一致。

### Base Control 的模組目標

`aicar_base.py` 依據機器人的需求，提供其**底層運動與硬體整合介面**。在 AICAR 中，Base 負責共用硬體連線、手臂與車輪協調、車體運動學、命令接收與有效回饋發布；不必容納所有機器人功能、所有 Topic 或所有控制限制。

不同機器人應依照其運動能力劃分控制模組：

| 機器人 | 主要控制模組 |
| --- | --- |
| 差速車 | 車輪速度控制 |
| 四足機器人 | 四隻腿的關節控制 |
| 人形機器人 | 全身關節、平衡與步態控制 |
| 機械手臂 | 關節位置、速度與軌跡控制 |

**設計判斷：** 大型機器人通常不會把所有控制功能集中在一支 `base.py` 裡，而是拆成不同控制器，再由更上層的控制模組協調。AICAR 規模較小且共用 DYNAMIXEL 通訊埠，因此目前保留單一 Base Node；各層只管理自己責任範圍的介面及限制，待規模增加再拆分，避免過度設計。

### Base Callback 與控制迴圈

`cb` 為 Callback（回呼函式）。Subscriber 的 `cb_reset()`、`cb_torque()`、`cb_wheel_torque()`、`cb_tool()`、`cb_joint()`、`cb_cmd()` 接收並驗證、暫存命令；`cb_timer()` 負責定期執行硬體操作及發布回饋。目前目標頻率為 30 Hz。絕對目標採用最新有效指令（Latest-wins），停止與停用要求優先；已過期的速度目標不能繼續使用。

草稿 PR #6 的控制順序（尚待實車驗收）：

```text
Timer 觸發
  ↓ 確認故障、命令逾時與停止要求（優先）
  ↓ 處理有效且允許的控制命令
  ↓ 讀取馬達實際回饋
  ↓ 驗證資料與更新 Encoder 基準
  ↓ 發布 JointState / Diff Rad
```

**設計理由：** 停止與故障優先於一般運動；目標指令不等於實際量測，應由有效回饋更新 ROS2 狀態。即使沒有新訊息，Timer 仍應檢查命令有效性。軟體 Timer 或馬達通訊可能延遲，不能取代獨立的硬體安全保護。

**介面約定：** 既有 `/torque` 仍為手臂控制；新增 `/wheel_torque` 作為獨立車輪啟用介面。軟體啟用權限預設關閉（不代表已測量實際扭矩狀態）；啟用後才接受新的運動目標。`/reset` 要求車輪停止並重建 Encoder 基準，不自動移動手臂或切換控制模式；回原位（Home Pose）屬於獨立的位置運動需求，而非數值歸零。`command_timeout_sec` 預設 0.5 秒，待 Teleop 及通訊驗收；關閉時不自動回原位或移除手臂支撐扭矩。此介面不是安全認證的急停系統。

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

**設計決策與驗證：** 保留原有硬體 ID、零點、方向、協定及換算參數，不新增配置框架。關節的度數與弧度、車輪的有號物理值與無號封包表示分開處理。PR #5（Arm）已合併，PR #6（Wheel／Base）仍為草稿；目前 36 項離線單元測試通過。實際機械限制、模式、Encoder 連續性及停止狀態仍須現場確認；Base 介面整合不等同於實車安全驗收。

## 3. `aicar_base.py` 重構介面與設計判斷（PR #6）

| 函式／區塊 | 責任與原因 |
| --- | --- |
| `constrain()`、`twist_to_wheel_values()` | 明確區分車體線速度、輪角速度與馬達數值；同比例限速保留理想運動學比例 |
| `encoder_delta()` | 計算有號 32-bit 回繞增量；硬體重新啟動造成的量測不連續仍需另外驗證 |
| `__init__()` | 共用一個 Port；驗證連線、建立 Topic 與 Timer，部分初始化失敗時清理資源 |
| `cb_reset()`、Torque、Tool、Joint、Twist callbacks | 將有效要求暫存，與 Timer 的硬體 I/O 分離；不把資料 Reset 等同於回原位 |
| `cb_timer()` | 停止優先、命令逾時、處理有效目標、讀取和發布回饋；故障時鎖定而非自動重啟 |
| `fn_update_states()` | 建立首次 Encoder 基準、保留低速增量並驗證所有回饋；暫保留前兩輪 `/diff_rad` 契約 |
| `fn_shutdown()`、`main()` | 停止請求與資源清理分開，不依賴等待時間或自動手臂移動證明安全 |

英文註解保留原教學步驟並補上設計原因。36 項假硬體離線測試已通過；ROS2 容器驗收與實車測試仍需分別追蹤。

### `aicar_imu.py`（MPU9250）：量測介面與定位估算分離

**模組目標：** 透過 I²C 取得陀螺儀 Z 軸角速度，以 ROS2 標準單位 rad/s 發布 `/imu` (`sensor_msgs/Imu`)。AICAR 為平面移動機器人，目前只需要 Yaw 角速度，不必為了擴充性立即實作完整九軸驅動。角度積分與位置估算由 `aicar_odom.py` 負責。

```text
MPU9250 (I²C, bus 1, address 0x68)
  ↓ 讀取 Z 軸高、低位元組 (0x47, 0x48)
  ↓ 合併 16-bit 並解碼二補數
  ↓ 除以 131 LSB/(°/s)，轉為 °/s，再轉為 rad/s
  ↓ 加上時間戳、imu_link 與量測有效性資訊
/imu → aicar_odom.py（結合 /diff_rad）→ /odom、TF
```

| 函式 | 責任 |
| --- | --- |
| `__init__()` | 建立 Node、I²C、初始化、Topic、訊息與 200 Hz 目標 Timer |
| `fn_init()` | 確認 MPU9250 裝置 ID、寫入並讀回電源與量程設定；並非校正或角度歸零 |
| `fn_read_gyro_z()` | 讀取、解碼 16-bit 原始值，回傳 °/s |
| `cb_timer()` | 轉為 rad/s、記錄量測時間、發布有效 IMU 訊息 |
| `cb_reset()`（原版） | 原版收到 `/reset` 後重寫感測器設定；重構後移除共享 Reset 訂閱 |
| `fn_shutdown()` / `main()` | 管理 ROS2 初始化、spin、關閉與 I²C 資源釋放 |

**物理單位與數位編碼：** `raw = (high << 8) | low`；若 `raw >= 32768`，減去 `65536`，還原 −32768 至 32767 的有號值。此運算只改變數值解讀，並未轉換物理單位。例如 `0xFAE2` = 64226（unsigned）→ −1310（signed）→ −10 °/s → −0.17453 rad/s。量程 ±250 °/s（約 ±4.36 rad/s），原有 131 LSB/(°/s) 與量程配對保留。

**積分與漂移：** `theta += angular_velocity * dt` 是 Odometry 的角度積分。假設靜止時陀螺儀有固定偏差 0.1 °/s，直接積分 60 秒便產生 6° 誤差；低通濾波可以降低高頻噪聲，卻不能保證消除固定偏差。現有 Odometry 在兩側 Encoder 增量皆零時以零角速度抑制靜止漂移，但輪子不轉不一定代表車體未受外力旋轉，且需另外檢查 IMU 資料時間與新鮮度。

| 操作 | 改變的狀態 | 不代表 |
| --- | --- | --- |
| Initialization | 喚醒硬體、設定量程；確認通訊與有效量測 | 偏差已校正、航向已歸零 |
| Calibration | 在已知靜止條件下估計角速度零點偏差，從後續量測扣除 | 車子回到起點或已消除全部漂移 |
| Odometry Reset | 使軟體估算的 x、y、theta 與積分時間基準重新起算 | 實體移動、感測器重新通電或校正 |

**設計決策：** 使用 `/reset` 不應無條件重新初始化 IMU；Odometry 歸零不需要改變量測硬體設定。若保留 IMU 的重初始化能力，應作為獨立的維護操作。校正值須由靜止實測取得，不在無硬體環境假造數值。200 Hz 是 ROS2 Timer 目標，不等於感測器實際取樣頻率。`orientation_covariance[0]` 與 `linear_acceleration_covariance[0]` 為 −1，表示未提供該類量測；不能把 X/Y 預設零誤認為實測值，通用融合前需明確限制使用 Z 軸並驗證 `imu_link` 與車體座標方向。

| 優先度 | 審查問題（原版） | 修改與驗證方向 |
| --- | --- | --- |
| 高 | `abs(gz) >= 2.0` 直接歸零，隱藏量程內的有效旋轉 | 保留有效量測；不得以零代替通訊失敗或異常資料 |
| 高 | I²C 初始化、讀取錯誤與關閉未完整處理 | 驗證初始化、讀取失敗不發布舊值／假值、確保 Bus 釋放 |
| 中 | 高低位元組分開讀取可能跨樣本 | 改為連續讀取，確認長度與測試有號邊界 |
| 中 | `/reset` 重寫硬體設定卻未校正 | 分離維護初始化、靜止校正與 Odometry 歸零 |
| 中 | 未確認資料方向、偏差、頻率、時間戳與 covariance | 離線驗證訊息契約；實車靜止及已知角度驗收，之後設定可信參數 |
| 低 | 轉換常數及外部依賴未明確說明 | `math.radians()`、英文註解；部署時確認 `smbus` 依賴 |

**狀態（草稿 PR #6）：** 已重構正式 IMU Node，保留 `share/` 參考原版及原始硬體參數。新增 `python3-smbus` 執行依賴；10 項 IMU 假硬體測試與既有 36 項測試全部通過（共 46 項）。ROS2 容器 IMU smoke test 已加入 CI，待本次執行結果。尚未實測裝置 ID、I²C 連續讀取、量測頻率、校正及座標；若校方板卡不是預期 MPU9250，應先辨識型號而非直接跳過 ID 檢查。先維持單一 Node，不引入 EKF 或新的驅動架構；IMU 讀取失敗時停止發布，但 Odometry 目前仍可能使用最後一次角速度，須在下階段修正資料過期處理。

## 4. 第一輪程式碼審查（歷史：278 行安全骨架）

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
| IMU | 重構及離線測試完成；待 ROS2 CI 與座標、校準、頻率與 covariance 的實車驗證 |
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
- [ROS2 Jazzy sensor_msgs/Imu 訊息定義](https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs/msg/Imu.msg)
- [REP-103：單位與座標規範](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst)
- [TDK MPU9250 Register Map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-9250-Register-Map.pdf)
- [Analog Devices：Gyroscope Calibration](https://www.analog.com/media/en/technical-documentation/technical-articles/gyrocalibration_edn_eu_7_2010.pdf)
- [Gazebo 與 ROS 相容版本](https://gazebosim.org/docs/harmonic/ros_installation/)
- [10｜跨環境部署與 Docker](10-aicar-docker-deployment.md)

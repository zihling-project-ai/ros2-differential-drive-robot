# 09｜AICAR ROS2 軟體工程設計與審查

更新日期：2026-11-21　｜　狀態：六個核心模組離線驗收完成，實車待驗證

> **審查基準：** ROS2 套件 v0.1.0，PR #5、#6、#7 已合併至 main。保留原教師 TODO 步驟及英文設計註解。79 項離線測試與四個 ROS2 smoke test（假硬體／合成資料）通過；尚未完成實車驗收。測試證據見 [06](06-smoke-tests.md)。

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
│       ├── diagnostics/         # 手動硬體診斷，不屬於離線 CI
│       ├── test/                # 離線單元測試與 ROS2 smoke test
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

| 同名 Python 檔 | 現行正式版與教師參考版的關係 |
| --- | --- |
| `aicar_base.py` | 正式版已重構命令、回饋、Reset、逾時與清理 |
| `aicar_imu.py` | 正式版新增裝置識別、連續讀取及異常處理 |
| `aicar_odom.py` | 正式版改用累計輪角與時間戳、Midpoint 積分 |
| `aicar_teleop.py` | 正式版增加回饋同步與命令期限 |
| `ball_detector.py`、`lidar_follow.py`、`lidar_sub.py` | 仍有實作差異，未納入六個核心模組驗收 |

結論：來源 `share/` 目前並非正常 ROS2 套件執行入口；可以考慮在確認有用差異後移除，但目前保留，不直接將參考版本覆蓋正式程式。尤其 LiDAR 模組需先逐段比較與測試。`me333_pkg` 已移除，原始教材可由 Git 歷史找回。

## 1. 目的與審查範圍

先理解程式如何運作、每個模組的責任與介面，再確認問題、設計測試及修改。未確認原因前，不為了消除警告而隨意改動程式。

| 來源 | 狀態 |
| --- | --- |
| 六個核心模組與 `constants.py` | 已審查、重構、整合；離線測試通過，待實車驗收 |
| 老師公開的 [ai_car 倉庫](https://github.com/zhl017/ai_car) | 公開首頁顯示 ROS1 Noetic；不能當作目前 ROS2 原始碼 |
| LiDAR、Ball Detector、Launch、Nav2、SLAM、Gazebo、RViz2 | 保留原有程式與資源；非本次重構及端到端驗收範圍 |

P0/P1/P2 表示修復或驗收優先級，而非事故發生機率。第 4 節記錄現行版本的待驗證條件。

## 2. ROS2 模組地圖

| 模組 | 主要責任 | 典型資料介面 |
| --- | --- | --- |
| Teleop / Planner | 產生運動需求 | `cmd_vel` (`Twist`) |
| Base Control | 解讀需求、協調底層驅動與狀態 | `cmd_vel`、`joint_states` |
| DYNAMIXEL Driver | 硬體專屬通訊與回饋驗證 | Arm/Wheel Controller API |
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

現行控制順序（尚待實車驗收）：

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

**設計決策與驗證：** 保留原有硬體 ID、零點、方向、協定及換算參數，不新增配置框架。關節的度數與弧度、車輪的有號物理值與無號封包表示分開處理。Arm 與 Wheel 分別有 9、13 項離線測試通過。實際機械限制、模式、Encoder 連續性及停止狀態仍須現場確認；Base 介面整合不等同於實車安全驗收。

**數值編碼與物理換算：** Tick 是裝置量測的離散數值，二補數是固定長度位元的有號表示，兩者與角度、角速度的物理單位各自獨立。32-bit 有號值的最高位權重是 −2³¹，其餘位權重為正；例如 `0xFFFFFFFF` 解讀為 −1，而非 4294967295 Tick。換算角度時仍需依馬達解析度、方向、零點偏移決定。Base 保留低速增量及回繞處理，不能以重新設定馬達位置代替 Odometry 歸零。

## 3. `aicar_base.py` 重構介面與設計判斷（PR #6）

| 函式／區塊 | 責任與原因 |
| --- | --- |
| `constrain()`、`twist_to_wheel_values()` | 明確區分車體線速度、輪角速度與馬達數值；同比例限速保留理想運動學比例 |
| `encoder_delta()` | 計算有號 32-bit 回繞增量；硬體重新啟動造成的量測不連續仍需另外驗證 |
| `__init__()` | 共用一個 Port；驗證連線、建立 Topic 與 Timer，部分初始化失敗時清理資源 |
| `cb_reset()`、Torque、Tool、Joint、Twist callbacks | 將有效要求暫存，與 Timer 的硬體 I/O 分離；不把資料 Reset 等同於回原位 |
| `cb_timer()` | 停止優先、命令逾時、處理有效目標、讀取和發布回饋；故障時鎖定而非自動重啟 |
| `fn_update_states()` | 建立 Encoder 基準、保留低速增量並驗證所有回饋；發布帶時間戳的累計角度，保留前兩輪 `/diff_rad` 教學介面 |
| `fn_shutdown()`、`main()` | 停止請求與資源清理分開，不依賴等待時間或自動手臂移動證明安全 |

英文註解保留原教學步驟並補上設計原因。14 項 Base 離線測試與 ROS2 Base smoke test 已通過；實車待驗證。Base 提供目標、回饋與通訊協調，並未實作馬達內部 PID；智慧伺服裝置內部閉環和 ROS2 上層控制週期是不同層次。

### `aicar_imu.py`（MPU9250）：量測介面與定位估算分離

**模組目標：** 透過 I²C 取得陀螺儀 Z 軸角速度，以 ROS2 標準單位 rad/s 發布 `/imu` (`sensor_msgs/Imu`)。AICAR 為平面移動機器人，目前只需要 Yaw 角速度，不必為了擴充性立即實作完整九軸驅動。角度積分與位置估算由 `aicar_odom.py` 負責。

```text
MPU9250 (I²C, bus 1, address 0x68)
  ↓ 讀取 Z 軸高、低位元組 (0x47, 0x48)
  ↓ 合併 16-bit 並解碼二補數
  ↓ 除以 131 LSB/(°/s)，轉為 °/s，再轉為 rad/s
  ↓ 加上時間戳、imu_link 與量測有效性資訊
/imu → aicar_odom.py（結合 /joint_states 的累計輪角與時間戳）→ /odom、TF
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

**驗收狀態：** 已重構正式 IMU Node，保留 `share/` 參考原版及原始硬體參數。新增 `python3-smbus` 執行依賴；10 項 IMU 假硬體測試與 ROS2 IMU smoke test 通過。尚未實測裝置 ID、I²C 連續讀取、量測頻率、校正及座標；若板卡不是預期 MPU9250，應先辨識型號而非直接跳過 ID 檢查。先維持單一 Node，不引入 EKF 或新的驅動架構；IMU 讀取失敗時停止發布，下游 Odometry 已新增資料過期檢查。

### `aicar_odom.py`：運動學、數值積分與 TF

**模組目標：** 不直接操作硬體；使用 `/joint_states`（依名稱讀取左、右前輪累計角度及 header 時間戳）及 `/imu`（Z 軸角速度）持續估算二維位姿 `(x, y, theta)` 和速度，發布 `/odom` 並廣播同一次估算的 `odom → base_footprint`。由車輪訊息觸發運算，目標更新率取決於 Base（30 Hz），不是 IMU（200 Hz）。Base 仍發布原有 `/diff_rad` 供教學對照，Odometry 不再訂閱它。

| 函式 | 責任與設計理由 |
| --- | --- |
| `__init__()` | 建立狀態、時間基準、訂閱、發布與 TF；第一筆有效輸入只建立基準 |
| `cb_reset()` | 清除軟體位姿、舊 IMU 與積分時間；不移動車體、不校正感測器 |
| `cb_imu()` | 檢查座標、有效數值及時間戳，僅接收最新有效角速度 |
| `_observe_clock()`、`_imu_is_fresh()` | 處理 ROS 時間倒退、量測時間與接收時間過期 |
| `cb_calc_odom()` | 驗證資料與時間間隔；換算行進距離，進行 Midpoint 積分及發布 |
| `main()` | 建立、spin、銷毀 Node，確保初始化失敗也關閉 ROS2 |

**差速正向運動學：** 車輪半徑 `r = 0.033 m`、輪距 `L = 0.16 m` 保留不變；與 Base 的逆向運動學互為對應。輪角度增量 `Δφ_L`、`Δφ_R`（rad）換算成行進距離 `s_L = r Δφ_L`、`s_R = r Δφ_R`（m）。車體中心位移 `s = (s_L+s_R)/2`，線速度 `v = s/Δt`。若以 Encoder 計算角度，理想差速模型為 `Δθ = (s_R − s_L)/L`；此版本仍使用 IMU 的 `Δθ = ω_z Δt`，不另外引入融合演算法。模型假設有效輪徑正確且輪子不打滑；四輪實際轉向與滑動需驗證。

**二維位置積分：** 朝向 `θ` 為弧度，`cos(θ)`、`sin(θ)` 將車體前進方向投影到 odom 的 X/Y 軸；原版 Euler 使用區間開始的朝向，新版 Midpoint 使用 `θ_mid = θ + Δθ/2`，`x += s cos(θ_mid)`、`y += s sin(θ_mid)`、`θ += Δθ`。Midpoint 減少曲線近似誤差，仍非精確圓弧；不能修正 IMU Bias、打滑或時間誤差。左右輪位移皆零時保留教師的靜止漂移抑制假設，需留意外力旋轉或打滑；不等同證明車體完全靜止。

**Quaternion：** Odometry 內部保留單一 Yaw；輸出標準 ROS2 Quaternion `(0, 0, sin(θ/2), cos(θ/2))`，滿足單位長度。90° 對應 `(0,0,0.7071,0.7071)`；`q` 與 `−q` 表示同一旋轉。三維 Euler 角在特定姿態可能出現萬向鎖（兩個旋轉自由度的參數方向重合，無法獨立區分），Quaternion 不具有該 Euler 表示奇異點。AICAR 平面 Yaw 不會遭遇此三維問題，使用 Quaternion 是為了遵循 ROS2 訊息契約。

| 介面 | 責任 | 座標或資料 |
| --- | --- | --- |
| `/odom` | 對外提供狀態估算 | Pose 在 `odom`；Twist 在 `base_footprint`；含 covariance 欄位 |
| `/tf` | 供 tf2 查詢座標與時間關係 | 同一估算的 `odom → base_footprint`，沒有速度或 covariance |
| `/tf_static` | URDF 固定關節座標關係 | `base_footprint → base_link → imu_link` 等；Robot State Publisher |

`/odom` 與 `/tf` 使用相同位姿與時間戳，前者供導航讀取狀態，後者供 RViz2、LiDAR、SLAM 將不同來源的資料轉換至共同座標系。`map → odom` 由定位或 SLAM 提供；不得由 Odometry 重複廣播定位變換。同一 TF 邊應有單一發布責任。`odom` 正常運作應連續，長期漂移由定位修正，而非週期性 Reset。

**時間與 Reset 的設計取捨：** IMU 同時檢查 ROS header 時間及單調接收時間（暫定 0.1 s），且與輪回饋時間差須在同一門檻內；輪回饋新鮮度及輪回饋間隔上限皆暫定 0.2 s。使用 `JointState.header.stamp` 的時間差而非接收時間，`/odom`、TF 共用該時間戳；前後累計角度之差可在短暫遺失訊息且資料仍有效時保留輪距離增量。超過時間上限、無效或 IMU 中斷的區間不補積分，恢復時重建基準。Base 的時間戳為該批硬體讀取完成後的主機時間，不是四個 Encoder 獨立的硬體時間；最新 IMU 角速度仍是區間近似，跨主機需同步時鐘。暫停估算不是停止實車。共享 `/reset` 不具跨 Node 原子同步；Base 停輪並重建 raw tick 基準，但保留累計角度，Odometry 清除位姿及差分基準、拒絕 Reset 前的 IMU 與 JointState 時戳，避免兩個 Node 處理順序不同造成假的反向位移。仍應在已停止的測試流程執行 Reset；Base 程序重新啟動會重新建立累計值，須協調重新啟動 Odometry 或重新歸零，不能假設所有中斷均可自動識別。`/joint_states` 必須只有可信的實車回饋來源，不得混入展示 GUI。Covariance 目前未校正，預設零不代表已知零誤差，不能直接當成可信融合量測。

**驗收狀態：** 保留 `/diff_rad`、原始幾何參數、Quaternion 與 TF 邏輯；Odometry 改用 `/joint_states`。19 項離線測試涵蓋累計角度、量測時間與接收時間差、短期遺失、重複及亂序、依名索引、兩種 Reset 處理順序、過期／未對齊資料；Base Reset 測試同時驗證累計角度連續與 `/diff_rad` 歸零。ROS2 Odometry/TF smoke test 通過。實車頻率、時間閾值、漂移、輪徑／輪距、covariance 及導航整合尚待驗收；不因通過測試而宣稱完成定位或安全驗證。

### `aicar_teleop.py`：操作目標、回饋及命令有效性

**模組目標：** 將終端機按鍵轉為 `/cmd_vel`（m/s、rad/s）、`/joint`（兩個關節角度，度）、`/tool`（工具角度，度），不直接操作硬體。以 `/joint_states` 建立起始目標，不把正在變化的實際位置當作使用者的目標。`/reset` 清除速度與待同步的目標，不等同 Home Pose。

| 介面或函式 | 責任與設計理由 |
| --- | --- |
| `__init__()` | 建立原有 Topic，新增對 `/torque`、`/wheel_torque` 要求的觀察以清除舊目標；不發布啟用要求 |
| `cb_joint_states()` | 依名稱找三個關節，驗證角度、時間與格式；初始化、恢復或重新啟用後同步目標 |
| `cb_reset()` | 清除速度並要求新回饋，不自動復歸手臂 |
| `fn_get_key()` | Raw Mode 讀取按鍵；區分等待逾時與 EOF，使用 `finally` 還原終端機 |
| `fn_wheel_pub()`、`fn_arm_pub()` | 車體速度與手臂／工具命令分開；僅明確 Home 同時發布 Joint 和 Tool |
| `_handle_key()`、`fn_run()` | 每次操作更新目標；短期有效命令定期發布，逾時、停止、退出清除速度 |
| `main()` | 單一背景 ROS Executor 與鍵盤主執行緒並行，以 Lock 保護共享狀態；確保關閉資源 |

**設計判斷：** 保留原始最大速度與角度範圍（尚未驗證為實車安全限制）；將正式版步進從 1 m/s、2 rad/s 恢復教師的 0.01 m/s、0.5 rad/s；手臂 2° 不變。暫定鍵盤目標期限 0.3 s、輪詢 0.1 s、手臂回饋期限 0.5 s；Base 的 0.5 s 逾時是獨立第二層保護。只有有效車輪按鍵更新期限，停止操作後不再重發非零目標。終端機不能可靠偵測按鍵放開或所有延遲按鍵；這不是硬體 dead-man switch，亦非安全認證急停。`/torque`、`/wheel_torque` 仍須明確啟用；收到其要求不是馬達狀態確認，Topic 也無跨 Node 原子同步。手臂位置命令未必即時達位；無效回饋、掉線或過期時不得創造新的角度目標。`h` 是使用者明確要求的 Home，`s` 是要求停止車輪，不會自動關閉手臂支撐扭矩。

**驗收狀態：** 11 項 Teleop 離線測試通過；包含步進與限制、命令期限、Reset、依名讀取回饋、分離 Joint/Tool、Home、Torque 要求造成的目標失效、EOF 清理等；ROS2 訊息 smoke test 通過。尚須實車確認鍵盤連續輸入時的行為、回饋與命令延遲、停止與獨立硬體保護；鍵盤與導航等不同控制來源不得同時競爭 `/cmd_vel`。LiDAR、Ball Detector 留待後續階段。

### 共用機器人參數：`constants.py`

將描述同一機器人規格或 ROS 介面的常數放在不依賴 ROS2、硬體 SDK 的純 Python 模組：車輪半徑 0.033 m、輪距 0.16 m、關節命令範圍 0～90°、工具命令範圍 0～45°、明確 Home Pose `(0, 0, 0)`、手臂關節名稱與 `imu_link`。Base 與 Odometry 必須使用同一幾何模型；Arm、Base 與 Teleop 必須使用一致的命令範圍；IMU 發布的 Frame 必須與 Odometry 驗證相容。硬體 ID、暫存器、零點校正、裝置通訊、操作步進、各層逾時等仍留在各自模組。這不是 YAML 或執行期配置框架；所有數值維持原樣，並非實測機械限制。3 項純常數／URDF 名稱一致性測試通過；實車校正仍待驗收。工具關節 URDF 與命令範圍差異見 [08](08-aicar-hardware-urdf-physical-model.md)。

## 4. 現行版本的工程風險與驗收條件

| 優先級 | 檔案／介面 | 現行狀態與待驗證事項 |
| --- | --- | --- |
| P0 | `dxl_arm.py`、`dxl_wheel.py`、實體設備 | 保留既有校正；設備型號、方向、模式、機械限位與獨立安全保護待確認 |
| P0 | `aicar_base.py`、`aicar_teleop.py` | 有軟體逾時、錯誤與停止處理；不等於通訊失效、阻塞或失電時已證實安全停止 |
| P1 | `aicar_base.py`、`aicar_odom.py` | Encoder 回繞、累計角度、Reset 順序已離線測試；硬體重啟、滑移、時間同步及實測精度待確認 |
| P1 | `aicar_imu.py` | 有識別、讀取失敗及資料過期處理；實際晶片、量程、方向與 Bias 校正待驗證 |
| P1 | `package.xml`、`setup.py`、Docker | ROS2 依賴已宣告、建置通過；正式硬體 SDK、I²C 與完整選配依賴仍須在目標環境驗收 |
| P1 | URDF／TF、Launch | XML 與核心名稱測試通過；完整 RobotModel、Bridge 及實機啟動流程待驗證 |
| P2 | `config/`、`launch/`、LiDAR／Camera | 保留現有資源；定位、導航、感知與多控制來源仲裁在擴充階段再驗證 |

停止相關設計應以有效回饋與獨立保護驗證，不能依賴軟體旗標或固定延遲來證明物理安全；離線測試未連接硬體，因此無法確認實際停止距離、支撐扭矩或機械限制。

## 5. 驗收範圍與版本界線

| 項目 | 狀態 |
| --- | --- |
| Python/XML、79 項離線測試 | PASS |
| AMD64 Jazzy 容器建置與四個假硬體 ROS2 smoke test | PASS |
| 過往跨主機測試 Topic、時鐘與 Zenoh | 歷史驗證，詳見 [03](03-network-ssh.md)～[06](06-smoke-tests.md) |
| 完整 RobotModel、Gazebo Bridge、實車動力學 | 尚未通過完整驗收，詳見 [07](07-gazebo-rviz2-aicar-experiment.md)、[08](08-aicar-hardware-urdf-physical-model.md) |
| Nav2／SLAM | 已有配置與 Launch，曾啟動；尚未完成實機及資料驗證 |
| 實車硬體、故障情境及定位精度 | 待驗證 |

正式 Base 沒有獨立的 `hardware_enabled` 參數可作為離線保護。CI 使用測試替身且不掛載裝置或連接外部網路；詳見 [10](10-aicar-docker-deployment.md)。依照目前單車、單套件需求維持此架構，不新增尚未需要的框架。

## 6. 參考資料

- [ROS2 Jazzy robot_state_publisher](https://github.com/ros/robot_state_publisher/tree/jazzy)
- [ROS2 Jazzy rmw_zenoh](https://github.com/ros2/rmw_zenoh/tree/jazzy)
- [ROS2 Jazzy sensor_msgs/Imu 訊息定義](https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs/msg/Imu.msg)
- [REP-103：單位與座標規範](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst)
- [REP-105：移動機器人座標規範](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst)
- [ROS2 Jazzy nav_msgs/Odometry](https://github.com/ros2/common_interfaces/blob/jazzy/nav_msgs/msg/Odometry.msg)
- [TDK MPU9250 Register Map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-9250-Register-Map.pdf)
- [Analog Devices：Gyroscope Calibration](https://www.analog.com/media/en/technical-documentation/technical-articles/gyrocalibration_edn_eu_7_2010.pdf)
- [Gazebo 與 ROS 相容版本](https://gazebosim.org/docs/harmonic/ros_installation/)
- [10｜跨環境部署與 Docker](10-aicar-docker-deployment.md)

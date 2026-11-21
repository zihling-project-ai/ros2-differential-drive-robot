# AICAR — ROS 2 Jazzy Differential-Drive Robot

<img width="753" height="660" alt="AICAR robot" src="https://github.com/user-attachments/assets/2ac8b5cf-3525-407a-b43e-1c973081a2ce" />

AICAR 是基於 ROS2 Jazzy 的四輪差速移動機器人專案，整合 DYNAMIXEL 馬達控制、IMU、Odometry、TF、鍵盤操作與 Zenoh 跨主機通訊，並透過 Docker 與 GitHub Actions 建立可重現的離線開發與驗證環境。

**工程目標：** 將設備驅動、運動控制、狀態估算與跨主機通訊的責任分開，保留原始硬體校正及教師教學步驟，以可驗證的介面改進既有 ROS2 專案。現行套件版本為 **0.1.0**；核心軟體已通過離線驗收，實車與完整模擬整合仍待驗證。

## 系統架構

```text
Teleop ──> /cmd_vel、/joint、/tool ──> Base
                                         ├── Arm / Wheel 驅動 ──> DYNAMIXEL
                                         └── /joint_states（累計輪角與時間戳）
                                                       │
IMU ──> /imu ────────────────────────────────────────> Odometry
                                                       ├── /odom
                                                       └── TF（odom → base_footprint）
URDF + /joint_states ──> robot_state_publisher ──> 機器人關節 TF（待完整整合）
```

Base 協調共用設備通訊及命令有效性；底層驅動封裝設備操作，Odometry 不直接連接硬體。數值 Reset、手臂 Home Pose 與設備初始化是不同操作。保留 `/diff_rad` 作為教學對照，Odometry 使用帶時間戳的累計輪角。

| 核心模組 | 工程責任 |
|---|---|
| `dxl_arm.py` | 手臂位置介面、角度換算、通訊與回饋驗證 |
| `dxl_wheel.py` | 車輪速度介面、有號 Encoder 資料與通訊檢查 |
| `aicar_base.py` | 差速運動學、命令與硬體整合、JointState 發布 |
| `aicar_imu.py` | 陀螺儀 Z 軸角速度量測與有效資料發布 |
| `aicar_odom.py` | 累計輪角與 IMU 的平面位姿積分、Odometry 與 TF |
| `aicar_teleop.py` | 鍵盤目標、回饋同步與命令期限 |

`constants.py` 統一共用幾何與介面參數；未引入不必要的配置框架。詳細設計取捨與限制見 [09](doc/09-aicar-ros2-code-review.md)。

## Repository 結構

```text
ros2-differential-drive-robot/
├── .github/workflows/      # CI：靜態檢查、測試與容器建置
├── docker/Dockerfile       # ROS2 Jazzy 開發環境
├── doc/                    # 工程設計、配置與驗證證據
├── src/ai_car/             # ROS2 套件
│   ├── ai_car/             # 正式運作的 Python 模組
│   ├── diagnostics/        # 手動硬體診斷
│   ├── test/               # 離線測試與 ROS2 smoke test
│   ├── share/              # 教師參考版本（非正式執行入口）
│   ├── config/、launch/    # 配置與 Launch
│   ├── urdf/、meshes/、rviz/ # 機器人模型與視覺化資源
│   └── package.xml、setup.py、setup.cfg
└── README.md
```

原始碼由 Git 管理；`build/`、`install/` 與 `log/` 應在目標環境重新產生。`package.xml` 是 ROS2 依賴宣告來源，Dockerfile 管理開發環境；目前不另設重複的 `requirements.txt`。`share/` 保留教師參考版本，與正式程式分開演進。

## 開發與部署環境

| 環境 | 責任 | 選擇原因 |
|---|---|---|
| macOS + Docker | 程式開發、離線測試 | 減少本機 ROS2 套件安裝；ARM64 本機部署待驗證 |
| Windows 11 + WSL2 Ubuntu 24.04 | ROS2 開發、模擬、跨主機通訊 | 利用桌上型電腦運算與 Linux 環境 |
| Raspberry Pi 4B Ubuntu 24.04 ARM64 | 實車控制及感測 | 直接連接馬達、IMU 與車端設備 |

```text
macOS / Docker                 Windows / WSL2 (dev-host)
離線開發與測試                 Zenoh Router / 時間同步 / 開發工具
       │                                     │
       │ Git 共用版本                        ⇅ Wi-Fi（移動部署目標）
       └───────────────────── Raspberry Pi (robot-host)
                                  底層控制與感測
```

Wi-Fi 是移動場域的部署目標；雙網卡與有線直連是無線無法取得 IP 時的獨立診斷方案，並非實車長期運行的必要條件。歷史有線路徑已驗證 SSH、Zenoh 雙向測試 Topic 與 Chrony 校時；無線方案仍須復驗。詳細拓撲與匿名位址角色見 [03](doc/03-network-ssh.md)～[05](doc/05-time-sync.md)。

## CI 與實際驗證結果

[AICAR CI](.github/workflows/ci.yml) 使用 GitHub Actions，執行 Python/XML 靜態檢查、79 項離線單元測試、AMD64 Jazzy Docker 建置、`colcon build`、套件辨識及四個 ROS2 smoke test。容器不映射實體裝置、不連接外部網路；測試使用假硬體及合成量測，不進行實車部署。

| 項目 | 驗證狀態 | 證據與限制 |
|---|---|---|
| 核心軟體 | PASS | 79 項離線測試與四個 ROS2 smoke test，見 [06](doc/06-smoke-tests.md) |
| 有線通訊與校時 | 歷史 PASS | 2026-11-20 SSH、雙向 Topic 與 NTP；無線待復驗 |
| Turtlesim / rqt_graph / Talker | 歷史 PASS | 本機資料流與測試 Topic |
| Gazebo 模型與關節回饋 | 部分 PASS | 模型、重力及三關節讀取成功；位置控制出現約 ±50 rad/s 振盪及漂移，根因未確認 |
| RViz2 / Gazebo Bridge | 待驗證 | RViz2 GUI 已啟動，RobotModel/TF 尚未通過；Bridge 未測試 |
| Nav2 / SLAM | 已有資源，待驗證 | 具備配置與 Launch，曾啟動；尚未完成實機及資料驗證 |
| Mac ARM64 與映像搬移 | 待驗證 | AMD64 CI 結果不能替代 Mac 實際部署 |
| 實車控制、安全及定位精度 | 待驗證 | 離線測試不能證明實體停止、精度或安全性 |

## 工程文件索引

| 文件 | 涵蓋內容 |
|---|---|
| [01｜系統環境與運算分工](doc/01-os-hosts.md) | macOS、Windows/WSL2、Pi ARM64 與環境角色 |
| [02｜ROS2 Jazzy 與工作區](doc/02-ros2-workspace.md) | ROS2 套件管理、建置、Workspace、遠端開發 |
| [03｜網路架構、無線部署與有線診斷](doc/03-network-ssh.md) | Wi-Fi 目標、有線備援、路由及 SSH |
| [04｜Zenoh Router 與防火牆](doc/04-zenoh-firewall.md) | 跨主機資料傳輸架構、配置及驗證 |
| [05｜Chrony 與跨主機校時](doc/05-time-sync.md) | 時間戳、時鐘同步與實測結果 |
| [06｜驗證策略與 Smoke Tests](doc/06-smoke-tests.md) | 分層驗收、79 項測試、ROS2 資料流程 |
| [07｜Gazebo 與 RViz2 模擬實驗](doc/07-gazebo-rviz2-aicar-experiment.md) | 實驗量測、振盪、漂移及未通過項目 |
| [08｜硬體與 URDF 物理模型](doc/08-aicar-hardware-urdf-physical-model.md) | BOM、模型質量、幾何、質心與實測界線 |
| [09｜ROS2 軟體工程設計與審查](doc/09-aicar-ros2-code-review.md) | 模組責任、運動學、回饋、Reset 與設計決策 |
| [10｜跨環境部署與 Docker](doc/10-aicar-docker-deployment.md) | AMD64/ARM64、映像搬移、CI 與實車隔離 |

## 已知限制與後續方向

- 現行容器為精簡開發環境，尚非完整實車、Gazebo、RViz2 或導航部署映像。
- URDF 宣告質量合計約 1.291 kg，實體重量、質心與模型動力學仍須量測；詳見 [08](doc/08-aicar-hardware-urdf-physical-model.md)。
- 優先驗證實車設備型號、校正、命令失效情境與獨立保護，再評估完整定位、導航及感知整合。
- 模擬與實車資料來源應隔離；不因 GUI 啟動、通訊成功或軟體測試通過而宣稱實車安全已驗證。

# 02 — ROS2 Jazzy 與可重現工作區

> **工程目標：** 讓相同 Git 版本能在不同主機建置、安裝與辨識 AICAR 核心套件，並清楚區分系統相依、Python 套件、硬體設備及驗收範圍。
>
> **驗證基準：** 2026-11-20 的兩端 ROS2 開發紀錄，以及 2026-11-21 的版本 `0.1.0` 核心軟體驗收。新電腦重新安裝與實車運行仍需獨立確認。

## 2.1 ROS2 環境選擇

以 ROS2 Jazzy 與 Ubuntu 24.04 LTS（Noble）為共同基準；官方提供 AMD64 與 ARM64 的 Debian 套件。macOS 使用 Linux 容器開發，Windows 使用 WSL2，車端使用 Raspberry Pi ARM64；環境分工見 [01](01-os-hosts.md)。

| 邏輯層 | 必要套件或環境 | 用途 |
| --- | --- | --- |
| ROS2 基礎 | `ros-jazzy-ros-base` | 核心通訊、訊息型別與 CLI |
| 開發工具 | `ros-dev-tools`、colcon、rosdep | 建置、安裝與解析相依 |
| AICAR 核心 | `ai_car`、DYNAMIXEL SDK、SMBus | 操作、硬體回饋、里程估算 |
| 跨主機通訊（依部署需求） | `rmw_zenoh_cpp` | 對應既有 Zenoh 通訊架構 |
| 圖形與自主功能（選配） | ROS2 Desktop、RViz2、Gazebo、Nav2、SLAM | 不列入本階段核心套件驗收 |

**設計判斷：** ROS2 Desktop 不是完整的硬體部署環境，不保證已安裝馬達或 I²C 驅動；車端亦不必為了使用核心 ROS2 程式而安裝全部 GUI 套件。Nav2、SLAM 已有配置並曾啟動，實機導航與資料尚未完成驗證，故不作為核心安裝的必要條件。

## 2.2 工作區、套件與 Git 的責任

目前 Repository 本身已符合 ROS2 工作區結構，以下以 `~/ros2_ws` 作為示意根目錄；各主機可選擇自己的名稱，不必複製原有使用者路徑。

```text
ros2_ws/                         # Repository / ROS2 workspace root
├── .github/workflows/           # CI
├── docker/                      # Container build environment
├── doc/                         # Engineering documentation
├── src/
│   └── ai_car/                  # ROS2 package
│       ├── ai_car/              # Python modules
│       ├── diagnostics/         # Manual hardware diagnostics
│       ├── test/                # Offline tests
│       ├── package.xml
│       ├── setup.py
│       └── setup.cfg
└── README.md
```

| 檔案／目錄 | 工程責任 |
| --- | --- |
| `package.xml` | ROS2 套件名稱、版本與相依宣告 |
| `setup.py` | Python 套件安裝、資源與四個 Node 執行入口 |
| `setup.cfg` | 將執行入口安裝到 `lib/ai_car`，供 ROS2 尋找 |
| `src/` | 跨主機共享的版本化原始碼 |
| `build/`、`install/`、`log/` | 由各主機自行建置，非跨架構共享的來源檔案 |

Git 同步的是來源版本；不是 ROS2 安裝、系統裝置權限、udev 規則、網路配置或實體校正。不同 CPU 架構的建置產物不應直接共用。

## 2.3 核心環境重建與相依管理

新建 Ubuntu 24.04 環境時，先依 [ROS2 Jazzy 官方 Ubuntu 安裝指引](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) 設定 locale、APT 來源及 ROS2 套件庫；APT 來源配置可能隨發行版本變動，因此不把下載最新套件的臨時腳本寫成固定的專案流程。已有可用環境不必重裝。

在已配置官方 ROS2 APT 來源、可連線且具有安裝權限的 Ubuntu Shell：

```bash
# Core development environment; Desktop is optional for GUI experiments.
sudo apt update
sudo apt install -y ros-jazzy-ros-base ros-dev-tools
source /opt/ros/jazzy/setup.bash
```

`package.xml` 已宣告 AICAR 核心相依，包括 `dynamixel_sdk` 與 `python3-smbus`；使用 rosdep 依目標平台解析安裝需求，不必額外維護重複的 `requirements.txt`。先確認工作區已取得正確的 Git 版本，再執行：

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
# Run only once if rosdep has not been initialized on this system.
# sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
colcon build --symlink-install --packages-select ai_car
source install/setup.bash
```

此為新環境重建參考，尚未在兩種架構的全新實機完成相同流程驗證。`rosdep` 解析已宣告的依賴；`colcon build` 不等於所有執行依賴、硬體或選配功能均已可用。官方相依管理參考：[ROS2 — Managing Dependencies with rosdep](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Rosdep.html)。若車端離線，需依 ARM64 及系統版本準備對應套件與相依性，不可將 AMD64 安裝檔直接移轉使用。

### 跨主機與其他選配套件

使用 Zenoh 的環境可按需安裝 `ros-jazzy-rmw-zenoh-cpp`，並以 `ros2 pkg prefix rmw_zenoh_cpp` 核對；通訊配置見 [04](04-zenoh-firewall.md)。RViz2、Gazebo、Nav2、SLAM 與相機／LiDAR 專用套件須依各自的選配環境確認，不隨核心 `rosdep` 流程自動視為完成。

## 2.4 建置後的驗收與執行界線

在來源及工作區環境已載入、且與實車隔離的終端執行基本核對：

```bash
ros2 pkg prefix ai_car
ros2 pkg executables ai_car
python3 -c 'import dynamixel_sdk, smbus; print("Hardware SDK imports OK")'
```

| ROS2 執行入口 | 主要責任 |
| --- | --- |
| `aicar_base` | 實體車輪與手臂控制、硬體回饋 |
| `aicar_imu` | 實體 IMU 讀取與發布 |
| `aicar_odom` | 利用 ROS2 資料估算位姿 |
| `aicar_teleop` | 終端機操作命令 |

`ros2 run ai_car <executable>` 用於單一 Node；`ros2 launch ai_car <launch-file>` 用於依啟動檔案整合節點。現有 `aicar1_launch.py` 會啟動 Base、IMU、Odometry **及 RPLIDAR 驅動**，並非無硬體 smoke test；不得以該 Launch 作為家用或 CI 驗收。實車啟動須有現場授權、獨立安全保護、設備及通訊驗收。

| 驗收項目 | 狀態 |
| --- | --- |
| Package 資訊、四個執行入口與相依宣告 | 已驗收 |
| ROS2 Jazzy Docker 建置、79 項離線測試、四個 Node smoke tests | 已驗收（假硬體／合成資料） |
| 乾淨 Ubuntu AMD64／ARM64 安裝及真實 SDK import | 待重新驗證 |
| 啟動實體硬體、設備權限及跨主機完整控制鏈 | 待現場驗證 |
| RViz2、Gazebo、Nav2、SLAM | 不列入本階段核心驗收 |

如只需重建軟體，優先使用隔離 Docker 及離線測試；見 [10](10-aicar-docker-deployment.md)。測試資料不可與實車控制資料混用，模擬的獨立時間與通訊配置留待 [07](07-gazebo-rviz2-aicar-experiment.md)、10 記錄。

## 2.5 遠端開發與環境維護

兩端本地工作區可使用 VS Code WSL、Remote-SSH 等介面；IDE 並不改變部署位置，也不會自動同步檔案。Git 才是兩端使用相同原始碼版本的依據。

| 開發視窗 | 連線方式 | 工作區示例 |
| --- | --- | --- |
| 開發端 | WSL: Ubuntu-24.04 | `~/ros2_ws` |
| 車端 | Remote-SSH: robot-ethernet | `~/ros2_ws` |

SSH 設定使用 [01](01-os-hosts.md) 定義的代稱（`IP2`、`robot-user` 需在授權的實際環境替換）：

```sshconfig
Host robot-ethernet
    HostName IP2
    User robot-user
```

Windows 和 WSL2 的 SSH 金鑰與 known_hosts 不一定共用。既有 Remote-SSH 問題曾來自 Shell 設定中的未閉合引號；遇到登入初始化異常，可先用 `bash -n ~/.bashrc` 驗證語法，勿直接修改或覆寫整份設定。Router 等設定可先在單一終端驗證，再決定是否持久化。

**設計原則：** 只將來源、配置模板及工程文件納入版本控制；不要提交 SSH 私鑰、`.env`、Wi-Fi 密碼、Token、個人或學校設備識別資料。授權與原始教師程式出處須保留並在公開前確認。

**接續：** [03 — 網路拓撲與 SSH](03-network-ssh.md)。

# 02 — ROS 2 Jazzy 與雙主機工作區

> **狀態：** 本輪已實際使用 ROS 2 Jazzy 與兩套 Workspace；此文件的初次安裝指令為**新環境建置參考**，不是本輪重灌紀錄。

## 2.1 安裝規劃

ROS 2 Jazzy 對應 Ubuntu 24.04（Noble）。本機使用具備圖形介面的 Desktop 套件較適合 RViz2；車端可依硬體驅動所需套件選擇 `ros-base` 或 Desktop。

| 主機 | CPU 架構 | 主要程式 |
|---|---|---|
| WSL2 Ubuntu | x86_64（新建置應驗證） | Zenoh Router、RViz2、SLAM Toolbox、Nav2、Teleop |
| Raspberry Pi | arm64 | Base、DYNAMIXEL、IMU、Odometry、LiDAR、Camera |

## 2.2 初次安裝 ROS 2 Jazzy（參考程序）

在**對應的 Ubuntu 24.04 Shell**，有 Internet 與安裝權限時執行；已有運行中的 ROS 2 則不需要重裝。

```bash
locale
sudo apt update
sudo apt install -y locales software-properties-common curl
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository universe
```

**ROS 2 APT 套件來源**依官方最新流程安裝 `ros2-apt-source`（官方可能更新下載位置與發行版本）：

```bash
sudo apt update
export ROS_APT_SOURCE_VERSION="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F 'tag_name' | awk -F\" '{print $4}')"
export UBUNTU_CODENAME="$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")"
curl -fL -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${UBUNTU_CODENAME}_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
```

安裝 ROS 2：

```bash
# WSL2 有 GUI 環境
sudo apt install -y ros-jazzy-desktop
# 車端精簡版（依硬體套件需求補裝）
# sudo apt install -y ros-jazzy-ros-base

sudo apt install -y ros-dev-tools python3-colcon-common-extensions
source /opt/ros/jazzy/setup.bash
ros2 --help
```

`ros2-apt-source` 的釋出版本或安裝方式有變更時，以 [ROS 2 Jazzy 官方安裝頁](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) 為準。當設備沒有網路時必須另行準備符合 `amd64`/`arm64` 的套件與相依性，不能直接複製另一種 CPU 架構的安裝檔。

## 2.3 Zenoh RMW 與 GUI 相關套件

視已安裝項目補充：

```bash
# 先查看是否安裝，避免重複安裝
ros2 pkg prefix rmw_zenoh_cpp
ros2 pkg prefix rviz2
ros2 pkg prefix slam_toolbox
ros2 pkg prefix nav2_bringup
ros2 pkg prefix turtlesim

# 若確定缺少且可連線 APT Repository，按需補裝
sudo apt install -y \
  ros-jazzy-rmw-zenoh-cpp \
  ros-jazzy-turtlesim \
  ros-jazzy-rqt-graph
```

此處是通用安裝參考；真實車端的 LiDAR、Camera、DYNAMIXEL SDK 等相依性必須以現有專案 `package.xml`、硬體驅動與廠商文件為準，不可假設 ROS Desktop 已包含全部硬體支援。

## 2.4 兩份 ROS 2 Workspace

```text
本機：/home/zihling/ros2_ws
├── src/
│   ├── ai_car/
│   └── me333_pkg/
├── build/     ← 編譯產物
├── install/   ← 安裝產物
└── log/       ← 建置紀錄

車端：/home/ubuntu/ros2_ws
└── src/ai_car/ ...（依現場實際套件為準）
```

在各自主機執行：

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 pkg list | grep -E '^(ai_car|me333_pkg)$'
ros2 pkg executables ai_car
```

編譯失敗時應先讀取建置錯誤、檢查 `setup.py` 的 `console_scripts`、`package.xml` 與 Python Import；不要直接刪除老師的原始資料夾。

### Python Package 與 Launch 的基本使用

```bash
# Node：以 setup.py console_scripts 定義的名稱啟動
ros2 run ai_car aicar_odom

# Launch：透過 launch 指令啟動，不是 ros2 run
ros2 launch ai_car aicar1_launch.py
```

`aicar1_launch.py` 在先前版本中啟動 Base、IMU、Odometry。**啟動前須檢查車輪是否離地、控制程序是否已存在以及停車保護；純環境驗證不執行此 Launch。**

## 2.5 VS Code 雙視窗分工

| 視窗 | 連線方式 | 專案根目錄 |
|---|---|---|
| 左側 | `WSL: Ubuntu-24.04` | `/home/zihling/ros2_ws` |
| 右側 | `Remote-SSH: aicar-ethernet` | `/home/ubuntu/ros2_ws` |

Windows 的 SSH 設定檔（範例）：

```sshconfig
Host aicar-ethernet
    HostName 192.168.123.1
    User ubuntu

Host aicar-campus
    HostName <當次確認的車端學校網路 IP>
    User ubuntu
```

在 Windows 版 VS Code 使用 `Remote-SSH: Connect to Host...`，選 Linux 平台，連線後開啟 `/home/ubuntu/ros2_ws`。Windows SSH Client 與 WSL2 `~/.ssh` 的金鑰/known_hosts 不一定共用。

### `.bashrc` 常見錯誤

本輪車端曾在 `ZENOH_CONFIG_OVERRIDE` 行尾多打一個 `"`，使 Bash 出現 `unexpected EOF while looking for matching '"'`，並影響 Remote-SSH 的初始化。

```bash
# 車端：先驗證語法
bash -n ~/.bashrc
nl -ba ~/.bashrc | tail -n 45
```

修正引號與 IP 後再次 `bash -n ~/.bashrc`，無輸出通常表示語法檢查通過。Router 位址切換可先在目前 Terminal 使用 `export`，不必每次更改 `.bashrc`。

## 2.6 Git 版本控制與重建邊界

`src/` 原始碼、Launch、YAML、URDF、RViz 設定應保留；`build/`、`install/`、`log/` 由本機重建。同步原始碼不等於同步 ROS 2 套件、udev 規則、硬體驅動與系統防火牆。

原始教學程式的授權與出處應保留。不要將 SSH 私鑰、`.env`、Wi-Fi 密碼、Token、巨型 VS Code 索引資料庫提交版本庫。

**接續：** [03 — 網路拓撲與 SSH](03-network-ssh.md)。


## 2.7 2026-11-20 模擬環境隔離補充

本機 RViz2 的獨立測試採 `ROS_DOMAIN_ID=76`、`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`、`ROS_LOCALHOST_ONLY=1`，且 `unset ZENOH_CONFIG_OVERRIDE`；實車採 `ROS_DOMAIN_ID=6`、`RMW_IMPLEMENTATION=rmw_zenoh_cpp`、獨立 Zenoh Router。**這只是當次隔離方法，跨電腦不能假設 DDS multicast 在所有 WSL2 設定均正常**。切換需對所有 Terminal 一致設定，且不要將模擬的運動控制命令導向實車。參見 [07](07-gazebo-rviz2-aicar-experiment.md)、[08](08-aicar-hardware-urdf-physical-model.md)。

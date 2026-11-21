# 01 — 作業系統、設備名稱與帳號

> **狀態：** 現有設備已實測；全新重灌步驟屬建置參考。  
> **範圍：** Windows 11、WSL2 Ubuntu 24.04、Raspberry Pi Ubuntu 24.04 ARM64、使用者與主機名稱。

## 1.1 執行環境

| 設備 | 作業系統 | Linux 使用者 | 主機名稱 | 主要用途 |
|---|---|---|---|---|
| 開發主機 | Windows 11 + WSL2 Ubuntu 24.04 | `zihling` | `LV333-06` | ROS 2 CLI、Zenoh Router、RViz2、SLAM、Nav2 |
| 車端 | Raspberry Pi 4B + Ubuntu 24.04.4 LTS ARM64 | `ubuntu` | `car06` | DYNAMIXEL、IMU、Odometry、LiDAR、Camera |

兩台 Ubuntu 位於**不同作業系統實例**，即使資料夾名稱都叫 `ros2_ws`，檔案也不會自動同步。

Terminal 提示符號 `zihling@LV333-06`、`ubuntu@car06` 的格式為 `<Linux user>@<hostname>`。主機名稱不是 IP，帳號亦不等於 Windows 登入使用者。`ubuntu` 是車端目前使用的 Linux 帳號；該名稱是否由原始映像檔或部署人員設定，現存記錄不足以確認。

### 兩台 Linux 都可以執行的識別指令

```bash
whoami
hostname
hostnamectl
cat /etc/os-release
uname -m
pwd
```

`uname -m` 預期 WSL2 為 `x86_64`、Pi 為 `aarch64`；以實際輸出為準。不同 CPU 架構的 `.deb` 套件不能直接互裝。

## 1.2 Windows 安裝 WSL2 Ubuntu 24.04（新環境參考）

在 **Windows PowerShell**（不是 Linux Shell）：

```powershell
wsl --list --verbose
wsl --list --online
# 僅未安裝時才執行
wsl --install -d Ubuntu-24.04
wsl --status
```

依提示完成 Windows 重新啟動；Ubuntu 首次開啟時建立 Linux 使用者及密碼。若目標帳號名稱需為 `zihling`，應於初次配置時設定；若已有使用者，不宜僅為了提示符號而無計畫改名。

官方指引：[Microsoft — Install WSL](https://learn.microsoft.com/windows/wsl/install)。

### 主機名稱設定（新環境選用）

Linux 一般環境：

```bash
hostnamectl
# 需有該設備管理授權，確有需要才變更
sudo hostnamectl set-hostname car06
```

WSL2 持久主機名稱可用 `/etc/wsl.conf`；此範例僅示意，不覆寫既有設定：

```ini
[network]
hostname=LV333-06
```

在 Windows PowerShell 執行 `wsl --shutdown` 後重啟發行版才可核對結果。停止 WSL 會中斷正在執行的 ROS 2、Zenoh、RViz 工作。

## 1.3 WSL2 Mirrored Networking（本輪環境已具備）

本輪 WSL2 可見：

```text
eth0  192.168.123.2/24  192.168.1.6/24
eth1  10.117.1.231/24
```

這與 Windows 兩張網卡映射進 WSL2 的 Mirrored Networking 配置一致。

Windows 使用者設定檔通常位於 `%UserProfile%\.wslconfig`；於 Windows PowerShell 開啟：

```powershell
notepad $env:USERPROFILE\.wslconfig
```

僅在新建置或已獲准修改時，檢查／設定：

```ini
[wsl2]
networkingMode=mirrored
firewall=true
```

修改後要在可中斷工作時執行 `wsl --shutdown`。Mirrored Networking 不代表入站流量會自動通過 Hyper-V Firewall；詳見 [04 — Zenoh 與防火牆](04-zenoh-firewall.md)。

## 1.4 Raspberry Pi 安裝 Ubuntu（重灌參考）

現有車端的已測資訊：

```text
Ubuntu 24.04.4 LTS
Linux 6.8.0-1051-raspi
Architecture: aarch64
hostname: car06
```

重建流程：

1. 備份 `src/` 專案與必要的 Netplan、udev、感測器設定、ROS 2 相依套件清單；移除備份中的密碼與金鑰。
2. 在 Raspberry Pi Imager 中選擇適用於 Raspberry Pi 4 的 **Ubuntu Server 24.04 LTS (64-bit)**，將系統燒錄至車端使用的儲存媒體。
3. 於 Imager 或首次開機流程配置合法管理帳號、主機名稱、SSH、網路；不可假設重灌後自帶原先的硬體驅動與控制套件。
4. 連線後驗證 `uname -m`、`hostname`、網卡、SSH 及 ROS 2 安裝需求。

實機目前由 `systemd-networkd` / Netplan 管理網路。網路設定檔 `50-cloud-init.yaml` 內含 Wi-Fi 認證資訊，**不得原封不動提交 Git**。

## 1.5 Ubuntu 基礎驗收

```bash
# Linux：唯讀檢查
hostname
whoami
ip -br link
ip -br -4 addr
date -Is
timedatectl status
```

### 完成條件

- 主機、帳號與實際執行位置一致。
- WSL2 與 Raspberry Pi 分別能取得各自工作區的存取權限。
- 系統時間與網路狀態有記錄；車端不可把「NTP 服務 active」等同「已同步」。

**接續：** [02 — ROS 2 Jazzy 與工作區](02-ros2-workspace.md)。

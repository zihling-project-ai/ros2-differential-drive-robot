# 01 — 系統環境與運算分工

> **工程目標：** 將開發、跨主機通訊與實車控制分開，在不同運算架構上重現 AICAR 核心軟體。
>
> **驗證基準：** 2026-11-20 的既有環境紀錄與 2026-11-21 的核心軟體離線驗收；重建環境及實車驗證需分別確認。

## 1.1 系統架構與環境選擇

| 執行環境 | 專案責任 | 選擇原因 |
| --- | --- | --- |
| macOS + Docker | 程式開發、離線測試 | 避免在 macOS 本機安裝大量 ROS2 套件 |
| Windows 11 + WSL2 | ROS2 開發、跨主機通訊 | 利用桌上型電腦的運算資源與 Linux 開發環境 |
| Raspberry Pi 4B（ARM64） | 實車控制及感測 | 直接連接 DYNAMIXEL 馬達與 IMU 硬體 |

```text
macOS / Docker           Windows / WSL2            Raspberry Pi / Ubuntu
離線開發與測試            ROS2 開發及通訊   ← ROS2 →   實車硬體及感測
       └──────────────────── Git：共用專案原始碼 ────────────────────┘
```

**設計判斷：** 開發環境不必直接連接實車；只有車端負責實體設備操作。各端共用 Git 版本與 ROS2 資料介面，而非共用硬體裝置或建置產物。兩個 Ubuntu 實例即使工作區都叫 `ros2_ws`，檔案也不會自動同步。跨架構的建置產物及套件必須在目標環境重新安裝或建置。

Nav2、SLAM 已有 Launch／YAML 配置，依既有操作回報曾成功啟動相關程式，但尚未完成實機及資料驗證；因此不列入本階段核心驗收，也不把啟動成功等同導航功能完成。RViz2、Gazebo、LiDAR 與相機整合暫保留為後續驗證項目。

## 1.2 作業系統、CPU 架構與代稱

| 環境 | 系統基準 | Linux 架構 | 公開文件代稱 |
| --- | --- | --- | --- |
| macOS 開發端 | macOS + Linux Docker 容器 | 依映像檔平台（ARM64／AMD64） | `mac-dev` |
| Windows 開發端 | Windows 11 + WSL2 Ubuntu 24.04 | `x86_64`（既有設備） | `dev-host` / `dev-user` |
| 車端 | Raspberry Pi 4B + Ubuntu Server 24.04 LTS（64-bit） | `aarch64` | `robot-host` / `robot-user` |

既有車端曾記錄 Ubuntu 24.04.4 LTS、Linux 6.8.0-1051-raspi 與 `aarch64`；這些是當時的環境觀測值，不代表重灌後須固定使用相同 Kernel 版本。

本文件的主機、使用者與網路位址均採通用代稱，不對外保留學校設備識別資訊。後續文件沿用下列網路角色：

| 代稱 | 角色 |
| --- | --- |
| `IP1` | 開發端連接車輛之專用網段位址 |
| `IP2` | Raspberry Pi 車端網段位址 |
| `IP3` | 開發端上網介面位址 |
| `IP4` | 其他實際存在的介面位址（如適用） |

以上為角色標記而非可直接使用的 IP，重建時需依現場授權網路與實際設定填入。終端提示符號例如 `dev-user@dev-host`，表示 Linux 帳號與主機名稱，並非 Windows 帳號或 IP。

### 環境識別（唯讀）

```bash
whoami
hostname
cat /etc/os-release
uname -m
pwd
```

若環境使用 systemd，可補充 `hostnamectl` 與 `timedatectl status`。`uname -m` 可區分 `x86_64` 與 `aarch64`；安裝 `.deb` 或使用 Docker 映像時必須留意架構相容性。Mac 容器的映像建置與跨平台限制見 [10 — 跨環境部署與 Docker](10-aicar-docker-deployment.md)。

## 1.3 環境重建參考

### Windows / WSL2

在 Windows PowerShell 檢查現有設定；只有未建立發行版且獲准時才安裝。

```powershell
wsl --version
wsl --list --verbose
wsl --list --online
# For a new environment only, after confirming the distribution name
wsl --install -d Ubuntu-24.04
```

Ubuntu 首次啟動時設定 Linux 帳號即可，不需要為符合文件代稱而重新命名既有使用者。WSL2 發行版可在 `/etc/wsl.conf` 設定主機名稱；新建置時選用即可，例如：

```ini
[network]
hostname=dev-host
```

官方參考：[Microsoft — Install WSL](https://learn.microsoft.com/windows/wsl/install)、[WSL 設定檔](https://learn.microsoft.com/windows/wsl/wsl-config)。

### Raspberry Pi

採用 Ubuntu Server 24.04 LTS (64-bit) 的 Raspberry Pi 映像是為了與 ROS2 Jazzy 的 Ubuntu 基準及 ARM64 架構相容。重建環境前應備份原始碼、必要 Netplan、udev、感測器配置及相依清單，含憑證的設備備份應另外安全保存，不提交至 Git。燒錄映像會覆寫選定儲存媒體的資料；未經授權或未確認備份不得重灌。

透過 Raspberry Pi Imager 選擇相容映像、完成合法帳號與網路初始配置後，再核對系統版本、ARM64 架構、SSH 及硬體裝置。重灌不會自動還原 AICAR 驅動、設備規則或通訊配置。

官方安裝參考：[Ubuntu — Install Ubuntu on the Raspberry Pi](https://ubuntu.com/hardware/docs/boards/how-to/ubuntu_supported/raspberry-pi/)。

## 1.4 網路設計取捨

既有 Windows 開發端使用 WSL2 Mirrored Networking，使 Windows 的網路介面可在 Linux 端使用；實際驗收仍要檢查網路路徑與防火牆，而不是只看介面是否存在。

此模式適用於支援該功能的 Windows 11 22H2 或更新版本及 WSL2。Windows 使用者目錄內的 `.wslconfig` 是全域 WSL2 配置，與每個發行版的 `/etc/wsl.conf` 不同。僅在新環境或獲准配置時使用下列參考：

```ini
[wsl2]
networkingMode=mirrored
firewall=true
```

變更後須安排可中斷工作時重新啟動 WSL（`wsl --shutdown` 會停止所有執行中的 WSL 發行版）。Mirrored Networking 並不保證入站通訊，保留防火牆並依授權驗證必要連線。詳見 [03 — 網路與 SSH](03-network-ssh.md)、[04 — Zenoh 與防火牆](04-zenoh-firewall.md) 及 [Microsoft — WSL Networking](https://learn.microsoft.com/windows/wsl/networking)。

## 1.5 驗收結果與限制

| 檢查項目 | 結果 | 證據與限制 |
| --- | --- | --- |
| Windows / WSL2 與 Raspberry Pi 系統、架構 | 已記錄（2026-11-20） | 既有系統及主機識別紀錄；未重新安裝驗證 |
| 兩端的基本通訊與時鐘同步 | 已驗證（2026-11-20） | 通訊與校時證據見 03～05；不代表實車控制完成 |
| 核心 ROS2 軟體 | 已離線驗收（2026-11-21） | 79 項單元測試與 ROS2 Jazzy 容器建置／Smoke Test 通過；見 09 |
| macOS 上的 Docker 實際部署 | 待驗證 | 採容器開發方向；目前 CI 成功不代表 Mac 已完成本機同版部署 |
| Nav2 / SLAM | 已有配置、曾啟動；尚待驗證 | 未完成實機、量測資料及導航功能驗收 |
| 實車馬達、IMU、設備權限及實際運動 | 待驗證 | 必須在有實體硬體的安全條件下進行 |

重建環境的最低驗收為：確認主機及架構、帳號與工作區存取權限、網路與時間狀態；不要將 NTP 服務啟動等同於已完成同步。ROS2 套件與硬體驅動的驗收各自獨立。

**接續：** [02 — ROS 2 Jazzy 與工作區](02-ros2-workspace.md)。

# 04 — Zenoh Router、ROS 2 跨主機通訊與 Hyper-V Firewall

> **狀態：** 方案 A（有線 `192.168.123.0/24`）已實測 TCP 7447 與 ROS 2 Topic 雙向 PASS。  
> **適用：** ROS 2 Jazzy、`rmw_zenoh_cpp`、WSL2 Mirrored Networking。

## 4.1 角色與端點

```text
WSL2 LV333-06                               Raspberry Pi car06
192.168.123.2                               192.168.123.1
  ├─ rmw_zenohd Router   ◄── TCP 7447 ───►  ├─ rmw_zenoh_cpp Client
  ├─ ROS 2 Node（Client）                    └─ ROS 2 Node（Client）
  └─ RViz2 / SLAM / Nav2
```

此處的 **Zenoh Router** 是 ROS 2 資料探索／傳輸使用的 Middleware 元件，不是 Windows/Linux 路由表中的 IP Gateway。兩種網路方案使用同一個 Router 程序，通常只改變 Client 指向的 Router 位址。

## 4.2 本機 WSL2 啟動 Router

在 **WSL2 專用 Terminal**：

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=6
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 run rmw_zenoh_cpp rmw_zenohd
```

保留此 Terminal；不要因啟動不同 Node 就再開一個相同 Router。另開 WSL2 Terminal：

```bash
ss -lntp | grep ':7447'
nc -vz -w 3 192.168.123.2 7447
```

本輪實測：`rmw_zenohd` 監聽 `*:7447`，且 WSL 本機 TCP 測試成功。**本機測通不代表遠端可連**，還需防火牆驗證。

## 4.3 雙方 ROS 2 Client 環境

兩端每個新的 ROS 2 Terminal 都執行（先確保 Router 已存在）：

```bash
source /opt/ros/jazzy/setup.bash
# 如該 ROS 2 Package 已建置，可再 source 對應工作區
source ~/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=6
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/192.168.123.2:7447"]'
```

當工作區尚未建置或尚無 `install/setup.bash`，先跳過那一行。上述 `ZENOH_CONFIG_OVERRIDE` 使用**單一候選端點**以便分辨實際路徑。

原方案 B 若復原，則依當次 WSL2 學校網卡 IP 更改 Endpoint，例如 `10.117.1.231:7447`，但該端點本輪**未驗證**。舊位址 `10.117.1.28` 不應照抄。

### `.bashrc` 管理

本輪車端曾因設定尾端多出 `"` 造成：

```text
-bash: /home/ubuntu/.bashrc: unexpected EOF while looking for matching '"'
```

建議不同網路實驗先在各 Terminal 使用 `export`，驗證後再決定是否寫入持久環境腳本。寫入後需使用 `bash -n ~/.bashrc` 驗證引號是否配對。

## 4.4 Windows Hyper-V Firewall 入站規則

WSL2 Mirrored 模式可能同時受 Windows 防火牆與 **Hyper-V Firewall** 管控。實測 WSL2 入站預設 `Block`，導致 Pi 對 TCP 7447 Timeout。

**Windows PowerShell（需管理員權限及學校授權）：**

```powershell
$wsl = '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}'
Get-NetFirewallHyperVVMSetting -PolicyStore ActiveStore -Name $wsl

New-NetFirewallHyperVRule `
  -Name 'AICAR-Zenoh-7447' `
  -DisplayName 'AICAR Zenoh TCP 7447' `
  -Direction Inbound `
  -VMCreatorId $wsl `
  -Protocol TCP `
  -LocalAddresses '192.168.123.2' `
  -LocalPorts 7447 `
  -RemoteAddresses '192.168.123.1' `
  -Action Allow
```

已存在時先查看，不要重複新增：

```powershell
Get-NetFirewallHyperVRule -Name 'AICAR-Zenoh-7447'
```

本輪首次建立規則遇到 `Access denied`；取得有權限的管理員 PowerShell 後才完成。不可用全面關閉防火牆替代最小入站規則。管理型電腦應遵守網管政策。

### 車端 TCP 驗收

```bash
# Raspberry Pi
nc -vz -w 3 192.168.123.2 7447
```

本輪輸出：`Connection to 192.168.123.2 7447 ... succeeded!`

## 4.5 ROS 2 雙向 Topic Smoke Test（本輪 PASS）

以下測試不會驅動馬達。

### A：WSL2 Publisher → 車端 Subscriber

WSL2（已載入上述 Client 環境）：

```bash
ros2 topic pub -r 1 /aicar_link_test \
  std_msgs/msg/String "{data: 'hello_after_ntp_sync'}"
```

car06（已載入相同 Client 環境）：

```bash
ros2 topic echo /aicar_link_test std_msgs/msg/String --once
```

本輪收到：

```yaml
data: hello_after_ntp_sync
---
```

### B：車端 Publisher → WSL2 Subscriber

WSL2：

```bash
ros2 topic echo /aicar_reverse_test std_msgs/msg/String --once
```

car06：

```bash
ros2 topic pub -r 1 /aicar_reverse_test \
  std_msgs/msg/String "{data: 'hello_from_car06'}"
```

本輪 WSL2 收到：

```yaml
data: hello_from_car06
---
```

完成後停止持續執行的 Publisher（`Ctrl+C`）。

## 4.6 Timestamp 異常與 ROS 2 驗收界線

在初次 Topic 測試中，車端時鐘落後 WSL2 約 23 天，Zenoh 出現：

```text
incoming timestamp ... exceeding delta 500ms is rejected
```

即使同時收到 Topic，也不能忽略時鐘錯誤。完成 Chrony 同步後，重新開啟 ROS 2 Terminal／測試節點，再次驗證 Topic，本輪未再觀察到該警告。詳見 [05 — Chrony 與時間同步](05-time-sync.md)。

Topic 雙向 PASS 只表示通訊層可用，**不等於**真實 `/odom`、TF、SLAM、Nav2 或 DYNAMIXEL 安全性已驗收。

## 4.7 問題與處理順序

| 現象 | 檢查方法 | 優先排查 |
|---|---|---|
| IP 無法互通 | `ip route get`、`ping` | 實體網路／子網／ARP |
| WSL 本機 7447 可連、Pi Timeout | `ss -lntp`、Pi `nc` | Windows / Hyper-V 入站防火牆 |
| `Unable to connect to a Zenoh router` | `echo "$ZENOH_CONFIG_OVERRIDE"` | IP 已改、Router 未啟動、設定未繼承 |
| `ros2 topic list` 沒有節點 | 確認 Domain、RMW、Router | Discovery／未啟動 Publisher |
| 收到 Topic 但時間戳告警 | `date -Is`、`chronyc tracking` | 兩主機時鐘偏差 |

**參考：** [ROS 2 rmw_zenoh](https://github.com/ros2/rmw_zenoh)、[Microsoft WSL Networking](https://learn.microsoft.com/windows/wsl/networking)。

**接續：** [05 — Chrony 與跨主機時間同步](05-time-sync.md)。

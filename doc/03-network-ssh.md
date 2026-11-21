# 03 — 網路拓撲、單／雙網卡與 SSH

> **狀態：** 雙網卡有線直連在 2026-11-20 **PASS**；原學校網路方案過去曾可用，但本輪**未復現**。  
> **前提：** 機器人已通電，網路線接妥；不修改正在承載 SSH 的 `br0`／`eth0`。

## 3.1 兩種連線模型

### 方案 A：開發主機雙網卡、有線直連（本輪 PASS）

```text
學校 LAN / Internet
10.117.1.0/24, gateway 10.117.1.1
    │
Windows 11 / WSL2 LV333-06
    ├── 乙太網路 / WSL eth1：10.117.1.231/24 → Internet
    └── 乙太網路 2 / WSL eth0：192.168.123.2/24（不設 gateway）
                                      │ Ethernet
                                      ▼
Raspberry Pi car06：br0 192.168.123.1/24（eth0 為 bridge 成員）
```

本機同時具備兩條直連網段路由。`192.168.123.1` 的封包經 `eth0`；其他外網流量由 default gateway `10.117.1.1` 經 `eth1` 送出。不需要把車端設成 Internet Gateway。

在 Windows 11 以 `Win + R` → `ncpa.cpl` 找到連車端的**第二張實體網卡**，配置 IPv4：

| 欄位 | 乙太網路 2 |
|---|---|
| IPv4 | `192.168.123.2` |
| Subnet mask | `255.255.255.0` |
| Default gateway | **留空** |
| DNS | **留空** |

學校網路原本的 IP/DHCP/Gateway 不要變更。本輪「乙太網路 2」另曾有 `192.168.1.6/24` 位址，是否保留應由原網路用途決定；不得未經核對任意刪除。

### 方案 B：原本共用學校網段（歷史可用）

```text
學校 LAN 10.117.1.0/24 ── Internet
       ├── Windows/WSL2 單張上網網卡：舊 10.117.1.28、現 10.117.1.231
       └── car06：舊 10.117.1.196（歷史紀錄，非目前有效 IP）
```

單一網卡同時連車端與 Internet 沒有衝突：同子網 IP 的流量透過本地鏈路；其他流量經預設閘道。方案 B 是否可用取決於 Raspberry Pi 是否能連到同一個允許互通的學校網路，以及 AP/交換設備與防火牆策略。**不要固定假設 DHCP 會永遠分配 `10.117.1.196`。**

## 3.2 IP 與路由檢查

Windows PowerShell：

```powershell
ipconfig
ping 192.168.123.1
Test-NetConnection 192.168.123.1 -Port 22
```

WSL2：

```bash
ip -br -4 addr
ip -4 route
ip -4 route get 192.168.123.1
ping -c 4 192.168.123.1
ip neigh show 192.168.123.1
```

本輪實測：

```text
192.168.123.1 dev eth0 src 192.168.123.2
4 packets transmitted, 4 received, 0% packet loss
TcpTestSucceeded : True
```

舊 `10.117.1.196` 曾出現 `Destination Host Unreachable`、鄰居解析 `FAILED`；Windows Ping 統計顯示「收到 4」但都是 ICMP 不可達訊息，**不代表 Ping 成功**。

## 3.3 Raspberry Pi 介面與 Netplan

車端唯讀檢查：

```bash
ip -br link
ip -br -4 addr
ip -4 route
networkctl list --no-pager
networkctl status wlan0 --no-pager
ls -1 /etc/netplan/
```

本輪狀態：

| 介面 | 觀察 |
|---|---|
| `eth0` | `UP`、`LOWER_UP`，`networkctl` 為 `enslaved` |
| `br0` | `192.168.123.1/24`，`routable` |
| `wlan0` | `NO-CARRIER`，無 IPv4 |
| IPv4 routes | 僅 `192.168.123.0/24 dev br0`，無 default route |

`/etc/netplan/50-cloud-init.yaml` 使用 `renderer: networkd`：`br0` 包含 `eth0` 並固定 `192.168.123.1/24`；`wlan0` 則設定 DHCP 連線到原教學 SSID `idm-wifi`。**此檔含認證資訊，不能直接貼到公開文件或 Git。**

`nmcli` 不存在並不代表網路管理故障；現場採 `systemd-networkd` 而非 NetworkManager。

## 3.4 原學校 Wi-Fi 排障紀錄（方案 B 未復現）

```bash
# Raspberry Pi，唯讀／不變更網路配置
sudo wpa_cli -i wlan0 status
sudo wpa_cli -i wlan0 list_networks
sudo wpa_cli -i wlan0 scan_results
systemctl status netplan-wpa-wlan0.service --no-pager -l
sudo journalctl -u netplan-wpa-wlan0.service -b -n 40 --no-pager
```

本輪觀察：`wpa_state=SCANNING`、`netplan-wpa-wlan0.service=active`，掃描能找到其他 2.4/5 GHz AP，但找不到原設定的 `idm-wifi`；Log 重複出現 `Failed to initiate sched scan`。此情況**不能直接證明**密碼錯誤、Wi-Fi 硬體損壞或驅動不相容。也可能是 AP 不在涵蓋範圍、SSID 已變更或被隱藏。

未獲學校網路管理員提供允許使用的 SSID 與連線方式前，不要改連附近其他 AP；不要為了測試單網卡而覆蓋可用的 `br0` 設定。

## 3.5 SSH 與 VS Code Remote-SSH

**WSL2 Terminal 連車端：**

```bash
ssh ubuntu@192.168.123.1
# 成功後應看到 ubuntu@car06:~$
whoami
hostname
hostname -I
```

SSH 初次連線需人工核對 **ED25519 Host Key Fingerprint** 與可信的車端資料後接受；登入密碼為車端 `ubuntu` 的密碼，並非 WSL `zihling` 的密碼。登入 `ubuntu@car06` 後再執行 `ssh ubuntu@192.168.123.1` 會形成車端連自己，無須這麼做。

**Windows VS Code 遠端連線：**

```sshconfig
# Windows: C:\Users\<WindowsUser>\.ssh\config
Host aicar-ethernet
    HostName 192.168.123.1
    User ubuntu

Host aicar-campus
    HostName <當次查證有效的車端學校網路 IP>
    User ubuntu
```

Windows VS Code `Ctrl+Shift+P` → `Remote-SSH: Connect to Host...` → 選擇 `aicar-ethernet` → Linux → Open Folder `/home/ubuntu/ros2_ws`。本機 VS Code 保留獨立 WSL 視窗開啟 `/home/zihling/ros2_ws`。

### SSH 能連但 VS Code 無法啟動

- 檢查 Windows 的 SSH Config 是否仍指向失效 IP。
- 由 WSL SSH 登入車端後檢查 `bash -n ~/.bashrc`。
- 本輪曾因 `.bashrc` 的 Zenoh `export` 行尾多一個雙引號而出現 `unexpected EOF`；修正後恢復遠端開發。
- Remote-SSH 連線後若卡在 VS Code Server 安裝，從 `Remote-SSH: Show Log` 辨認下載、遠端 Shell 或磁碟空間問題。
- WSL 與 Windows SSH Client 可能有不同的 `known_hosts`／金鑰，不可直接假設兩者共享登入狀態。

## 3.6 方案 B 的恢復驗收條件

1. 合法學校網路可供 car06 連線；`wlan0` 出現有效 `10.117.1.x` IP。
2. 從本機的學校網卡對車端新 IP 完成 Ping、SSH 驗證。
3. Zenoh Client 改指向本機**當次**學校網卡 IP 的 TCP 7447，重新檢查防火牆與雙向 Topic。
4. 跨主機時間同步來源亦須可達；不能因網路切換而保留已失效的固定 NTP 位址。

本輪沒有恢復 `idm-wifi`，所以方案 B 只能標記「歷史成功／待復驗」。

**接續：** [04 — Zenoh Router 與防火牆](04-zenoh-firewall.md)。

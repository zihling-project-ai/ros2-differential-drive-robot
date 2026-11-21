# 03 — 網路架構、無線部署與有線診斷

> **工程目標：** 建立可供移動機器人與開發端互通的網路，並在無線連線尚未建立時保留獨立的診斷路徑。
>
> **驗證基準：** 2026-11-20；無線方案曾可用，但此次未復現；雙網卡有線直連的網路與 SSH 已通過當輪測試，並非實地移動部署的驗收。

## 3.1 方案選擇：無線為主、有線為備援測試

| 方案 | 工程用途 | 設計取捨 | 驗證狀態 |
| --- | --- | --- | --- |
| A：無線 Wi-Fi | 實地移動與跨主機操作 | 車體不受網路線限制；需要有效 Wi-Fi、IP 配置、相互可達性及場域覆蓋 | 歷史可用，本輪未復現 |
| B：雙網卡、有線直連 | 建置、無線無 IP 時的診斷與隔離驗收 | 可保留開發端上網，同時建立獨立車端網段；網路線限制車體移動 | 2026-11-20 網路與 SSH 驗收通過 |

**設計判斷：** AICAR 是移動機器人，因此 Wi-Fi 是後續實地場域的目標方案。雙網卡不是正式運行的必要條件，也不代表必須讓車輛長期連接網路線。它的價值在於當 Wi-Fi 無法取得 IP、無法 SSH 或無法辨識故障時，仍能使用另一條已驗證的路徑檢查網路與軟體。此有線模式僅用於適當的靜止或隔離測試，不把連線成功當作實車運行驗收。

## 3.2 網路拓撲與路由

### 方案 A：Wi-Fi（目標部署）

```text
              授權的區域網路 / Wi-Fi AP
                  ├── 開發主機 dev-host（IP3）
                  └── Raspberry Pi robot-host（IP5，Wi-Fi）
```

開發主機可以經有線或無線加入同一個允許互通的網路；車端透過 `wlan0` 取得有效位址，不必與開發主機有實體線材連接。`IP5` 為本篇新增的車端 Wi-Fi 位址代稱，通常由 DHCP 分配；重新連線時須核對當前值，不將歷史位址視為永久設定。若場域允許，也可評估獨立的授權無線 AP，而非依賴共享網路。

無線網路適用於移動場域，但必須驗證 AP 覆蓋、設備互通策略、防火牆、通訊延遲及中斷行為；有效 IP 或成功 SSH 不代表移動時 ROS2 資料及控制延遲已符合要求。網路失聯也不應使實車依賴過期控制命令，獨立安全保護需另外驗收。

### 方案 B：有線診斷（已驗證的備援測試路徑）

```text
授權上網網路
       │
dev-host / Windows 11 + WSL2
       ├── 上網介面 IP3 → 原有預設閘道
       └── 專用 Ethernet IP1（不設預設閘道）
                              │ Ethernet
                              ▼
                     robot-host：br0 IP2
                     （eth0 為 bridge 成員）
```

`IP1` 與 `IP2` 遵循 [01](01-os-hosts.md) 所定義的專用車端網段角色，應位於同一個獲准且不與上網網段重疊的子網。開發主機原本上網介面的 DHCP、預設閘道及 DNS 保留；專用車端網卡僅負責該子網。

| 專用開發端網卡設定 | 原則 |
| --- | --- |
| IPv4 | `IP1`，與車端 `IP2` 同網段 |
| 子網遮罩 | 與車端網段相符；歷史測試使用 `/24` |
| Default gateway | 留空 |
| DNS | 不另設 |

這樣路由會將車端封包經專用 Ethernet 傳送，其他流量仍依開發端原有預設路由；不需要將車端設為 Internet Gateway。使用現有橋接介面時不得任意修改承載 SSH 的 `br0`／`eth0`；若還有其他介面位址 `IP4`，先確認其用途，不直接刪除。

## 3.3 連線驗收與故障判斷

依實際方案使用 `IP5`（無線）或 `IP2`（有線）替換目標位址；以下代稱不是可直接執行的實際 IP。只對獲准管理的車端設備進行檢查。

```bash
# WSL2 / Linux: inspect the selected network path
ip -br -4 addr
ip -4 route
ip -4 route get IP2
ping -c 4 IP2
```

```powershell
# Windows: check the vehicle endpoint and SSH service
ipconfig
Test-NetConnection IP2 -Port 22
```

| 檢查層級 | 判斷方式 |
| --- | --- |
| Link／Wi-Fi | 車端介面是否已連線 |
| IP／Routing | 有效 IP、子網及到達目標的網路路徑 |
| ICMP／TCP | Ping 及 SSH TCP 22 的可達性 |
| Application | SSH、ROS2 雙向 Topic、時鐘同步及資料品質 |

**已驗證證據（2026-11-20，有線路徑）：** 實測路由顯示車端目標由專用介面及開發端專用位址送出（代稱 `IP2 dev eth0 src IP1`）；Ping 4 次均成功、0% packet loss；Windows TCP 22 驗證為 `TcpTestSucceeded: True`，SSH 後確認登入車端。在共用網路路徑上，歷史車端位址曾出現 `Destination Host Unreachable`，鄰居解析 `FAILED`；收到 ICMP 不可達回覆不等於 Ping 成功。

## 3.4 Wi-Fi 未取得 IP 時的診斷

在車端或已建立的有線診斷連線中執行唯讀檢查：

```bash
ip -br link
ip -br -4 addr
ip -4 route
networkctl list --no-pager
networkctl status wlan0 --no-pager
```

若需要確認 Wi-Fi 連線狀態，在獲准管理的設備上查閱 `wpa_cli -i wlan0 status` 或相關 systemd 日誌；不要將包含 SSID、認證資料或其他設備識別資訊的完整輸出放入公開文件。

| 既有觀測 | 工程判斷 |
| --- | --- |
| `eth0` 已連線且為 bridge 成員、`br0` routable | 有線診斷路徑可用 |
| `wlan0` 為 `NO-CARRIER`，無 IPv4 | Wi-Fi 當時未建立有效連線，不是 ROS2 問題 |
| `wpa_state=SCANNING`，可見其他 AP 但找不到預期的授權 SSID | 尚無法確認當時的 Wi-Fi 失敗原因，不足以證明密碼錯誤或硬體損壞 |
| 車端僅有專用子網路由，無預設路由 | 有線診斷可達，但不能因此推定 Internet 可用 |

設備原先以 `systemd-networkd`／Netplan 管理網路，`nmcli` 不存在不代表網路管理故障。實際 Netplan 可能包含 Wi-Fi 認證，不能原封不動提交 Git。只有獲授權的無線環境及其管理資料確認後，才能重新驗證 Wi-Fi；不得以附近其他 AP 或歷史 DHCP 位址替代。

## 3.5 SSH 作為開發與診斷介面

SSH 能證明在目前網路及授權帳號下可進入車端 Linux，但不等於 ROS2 控制或導航已正常運作。

```bash
# Replace IP2 with the currently verified authorized vehicle address.
ssh robot-user@IP2
# On the remote host:
whoami
hostname
```

首次連線前應核對車端 Host Key Fingerprint；帳號為車端 Linux 帳號，與 Windows、WSL2 本機帳號各自獨立。VS Code 遠端開發設定見 [02](02-ros2-workspace.md)，不需在本篇重複操作流程。

## 3.6 實地無線部署的驗收計畫

| 驗收項目 | 目前狀態 |
| --- | --- |
| Wi-Fi 授權連線、有效 IP 與場域覆蓋 | 待復驗 |
| 無線路徑 Ping、SSH | 歷史可用，本輪未復現 |
| 無線 ROS2／Zenoh 雙向 Topic 與校時 | 待完整驗證；見 [04](04-zenoh-firewall.md)、[05](05-time-sync.md) |
| 場域內通訊延遲、失聯及獨立保護 | 待實際測量與安全驗收 |
| 無線無 IP 時的有線診斷 | 已驗證（2026-11-20 的網路與 SSH） |

最終目標是在無需長期網路線的情況下，取得可驗證的無線通訊；網路切換後必須重新核對實際 Router 與時鐘來源。此文件不將有線或無線 SSH 的成功宣稱為導航或馬達安全的通過。

**接續：** [04 — Zenoh Router 與防火牆](04-zenoh-firewall.md)。

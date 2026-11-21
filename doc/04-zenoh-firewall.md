# 04 — Zenoh 跨主機通訊與防火牆

> **工程目標：** 在已建立的網路路徑上，讓不同主機的 ROS2 Node 正確交換資料，並可分層驗證 Router、Middleware、網路權限與資料有效性。
>
> **驗證基準：** 2026-11-20 的有線診斷方案（03 的方案 B）已通過 TCP 7447 與 ROS2 雙向測試 Topic；實地無線方案（03 的方案 A）尚待復驗。

## 4.1 通訊架構與設計原因

```text
開發端 dev-host                             車端 robot-host
ROS2 Node（操作／開發）                      ROS2 Node（控制／感測）
       │                                            │
rmw_zenoh_cpp / Client                        rmw_zenoh_cpp / Client
       │                                            │
       └─────────── Zenoh Router ◄── TCP 7447 ───────┘
                   （WSL2）
```

Zenoh 是本專案採用的 ROS2 Middleware（RMW）實作之一。ROS2 Node 仍以相同 Topic、Publisher、Subscriber 與訊息型別交換資料；RMW 負責底層資料傳輸及探索。選擇 Router/client 架構是為了讓跨主機通訊有明確端點，便於在 WSL2 與車端之間分層驗收，而不是改寫機器人控制程式。

| 層級 | 職責 |
| --- | --- |
| ROS2 Node | 定義 Topic、資料型別與應用邏輯 |
| `rmw_zenoh_cpp` | ROS2 的 Middleware 實作 |
| Zenoh Router (`rmw_zenohd`) | 協助 Zenoh session 的探索及 client 資料轉送 |
| IP Gateway／路由表 | 決定網路封包到達目的 IP 的路徑，並非 Zenoh Router |

這是 AICAR 本階段採用的一種已測拓撲，不代表所有 Zenoh 部署都需要唯一 Router 或相同 session mode。Jazzy 的預設 session 與遠端 Router client 設定見 [ROS2 rmw_zenoh 官方文件](https://github.com/ros2/rmw_zenoh/tree/jazzy#configuration)。

## 4.2 無線主方案／有線診斷的端點

沿用 [01](01-os-hosts.md)、[03](03-network-ssh.md) 的網路代稱；以下並非實際可連線的 IP。

| 方案 | 開發端 Router 端點 | 車端 Client | 驗收 |
| --- | --- | --- | --- |
| A：Wi-Fi 目標部署 | `tcp/IP3:7447`（實際可達的開發端位址） | `IP5` | 待復驗；須確認場域及設備互通 |
| B：有線診斷 | `tcp/IP1:7447`（專用開發端位址） | `IP2` | 2026-11-20 已實測 |

兩種方案保留同一套 ROS2 程式與資料介面；切換網路時應重新核對 Router 監聽位址、Client 端點、路由及防火牆，而不是讓系統繼續使用歷史 IP。無線不需長期連接實體線材，有線僅作為無線缺少 IP 時的診斷路徑。

## 4.3 Router 與 Client 配置參考

在獲准且與實車控制隔離的 ROS2 Jazzy 測試環境，先確認已安裝 `rmw_zenoh_cpp`。Router 與 Client 使用獨立 Terminal，避免將 Client 的 `ZENOH_CONFIG_OVERRIDE` 意外套用到 Router 程序。

```bash
# Dedicated Router terminal (WSL2)
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
unset ZENOH_CONFIG_OVERRIDE
ros2 run rmw_zenoh_cpp rmw_zenohd
```

兩端 Client 使用相同的 RMW、Domain 與指定 Router 端點。下列 `42` 是示意 Domain ID，`IP1` 為有線測試 Router 位址代稱；必須按授權測試環境替換，無線時改用實際 Router 位址。

```bash
# Dedicated ROS2 client terminal on each test host
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42  # Example: use the same isolated test domain on both hosts
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/IP1:7447"]'
```

已建置工作區時可再載入 `install/setup.bash`。Domain ID 分隔 ROS2 通訊範圍，並非身份驗證或網路存取控制。測試環境不能只依賴 Domain ID 隔離實車運動；應維持授權網路、獨立安全保護及不啟動實車控制 Node 的驗收方式。切換 RMW 後若 CLI 顯示結果不一致，另核對 ROS2 daemon 與 Terminal 使用的環境是否相符。

## 4.4 Hyper-V Firewall 的工程取捨

WSL2 Mirrored Networking 的入站流量還可能受 Windows／Hyper-V Firewall 與管理政策過濾。本輪曾觀察 Router 在 WSL2 內已監聽，但車端 TCP 連線逾時，而有線 IP 路徑已可達；因此分層檢查了入站規則，而非重新修改 ROS2 程式或關閉防火牆。

| 項目 | 採用原則 |
| --- | --- |
| 協定與埠 | 僅允許該測試所需 TCP 7447 |
| 本機端點 | 已確認的開發端 Router 網路介面（有線為 `IP1`） |
| 遠端來源 | 已核對且獲授權的車端位址（有線為 `IP2`） |
| 權限 | 由具有設備及網路管理授權的人員核對有效規則 |
| 網路切換 | 無線位址變更後重新驗證範圍，不將規則擴大到所有來源或 Internet |

原測試曾因未具備管理權限而無法建立規則，改用具備權限的管理員 PowerShell 後才完成。保留原本防火牆及管理政策；不應以全面關閉防火牆或規避管理限制代替有範圍的規則。配置與管理指引參考 [Microsoft — WSL Networking](https://learn.microsoft.com/windows/wsl/networking)。

## 4.5 實際通訊驗收

以下為 2026-11-20 有線診斷路徑的實測結果，未將測試內容擴大解讀為無線或實車運行成功。

| 驗收層級 | 觀察結果 | 狀態 |
| --- | --- | --- |
| 網路可達性 | 有線專用網段路由正確、SSH 成功（見 03） | PASS |
| Router 監聽 | `rmw_zenohd` 監聽 TCP 7447；WSL2 本機連接成功 | PASS |
| 遠端 TCP | 車端連到指定 Router 端點，TCP 7447 測試成功 | PASS |
| 開發端 → 車端 | 發布 `/aicar_link_test`，車端成功接收 String 資料 | PASS |
| 車端 → 開發端 | 發布 `/aicar_reverse_test`，開發端成功接收 String 資料 | PASS |
| 實地 Wi-Fi 路徑 | 本輪無線未取得有效位址，未完成相同驗收 | 待復驗 |

確認 TCP 與 Topic 的最小重現方式（替換為已驗證且獲准的測試端點）如下。僅使用專用測試 Topic，不發布任何運動命令：

```bash
# Router host: confirm the listening service
ss -lntp | grep ':7447'
# Authorized vehicle host: validate the selected Router endpoint
nc -vz -w 3 IP1 7447
```

在已載入 Client 環境的兩端，分別執行下列發布與訂閱，再交換角色重測。執行前確認沒有實車控制 Node 參與測試：

```bash
ros2 topic pub -r 1 /aicar_link_test std_msgs/msg/String "{data: 'link_test'}"
```

```bash
ros2 topic echo /aicar_link_test std_msgs/msg/String --once
```

`link_test` 為通用示例，不是原始驗收訊息內容。完成後停止測試 Publisher。歷史雙向驗收證明基本 ROS2 資料可傳送，不證明即時性、丟包率、QoS、控制鏈、實機 `/odom`、TF 或導航皆已驗收。

## 4.6 時間戳與分層故障判斷

初次 Topic 測試曾觀察車端時鐘落後開發端約 23 天，Zenoh 記錄 timestamp 超過當時的 500 ms 門檻而被拒絕的警告。完成 Chrony 同步後，再重新開啟測試節點驗證 Topic，該次未再觀察到此警告。這是既有設備／版本的實測紀錄，不應將 500 ms 當成所有 Zenoh 版本的固定限制。詳細時間驗收保留於 [05](05-time-sync.md)。

| 現象 | 判斷方向 |
| --- | --- |
| 無法取得 IP 或路由不通 | 先處理網路路徑（見 03） |
| Router 主機可連，遠端 TCP 逾時 | 核對 Router 監聽、路由及獲准的防火牆規則 |
| TCP 可達，ROS2 Node 無法交換測試資料 | 核對 RMW、Domain、Client session 端點與 Topic 型別 |
| Topic 可收到但 timestamp 異常 | 核對兩端時鐘同步與量測時間 |

**設計結論：** 先證明底層網路，再驗證 Middleware 通訊與資料有效性。無線恢復後重複相同的分層測試，才進一步評估實地移動時的延遲、失聯與獨立安全保護。

**接續：** [05 — Chrony 與跨主機時間同步](05-time-sync.md)。

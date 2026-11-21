# 05 — 分散式時間基準與 Chrony 同步驗收

> **工程目標：** 建立開發端與車端之間可驗證的時間基準，使 ROS2 感測回饋、時間戳及跨主機資料可正確比較。
>
> **驗證基準：** 2026-11-20 的有線診斷方案（03 的方案 B）已完成 Chrony 同步及後續 Zenoh Topic 復驗；無線方案的時間來源與同步仍待驗證。

## 5.1 為什麼時間同步是機器人的必要條件？

| 時間概念 | AICAR 中的責任 |
| --- | --- |
| 系統／ROS 時間 | 為 `JointState`、IMU、Odometry、TF 記錄量測或估算時間；跨主機使用相容時鐘基準 |
| 單調時間（monotonic） | 判斷本機經過時間、控制命令及回饋逾時，不受系統時鐘跳變直接影響 |
| 量測時間與接收時間 | 量測時間用於計算運動積分間隔；接收時間輔助判斷通訊中斷 |

新版 AICAR 使用 `JointState.header.stamp` 的時間差估算輪角與線速度；IMU 也提供量測時間，並驗證資料新鮮度。因此跨主機時間不一致可能使有效資料被判為過期，或造成積分誤差及 TF 查詢問題。同步系統時鐘不代表網路沒有延遲，也不替代獨立的命令逾時機制。

模擬時間由模擬環境的 `/clock` 管理，不能直接與實車系統時間混用；本篇只處理實體主機時間基準。大幅校時可能使時間戳突然跳變，應先確保車體停止、硬體處於安全狀態，再停止依賴實體時間的控制、感測及定位程序，完成同步後重新建立其時間基準。

## 5.2 故障證據與原因判斷

| 既有觀測（2026-11-20） | 工程判斷 |
| --- | --- |
| 開發端日期為 2026-11-20，車端日期為 2026-10-28 | 車端時鐘相差約 23 天，不能直接比較跨主機 timestamp |
| 車端只有有線專用子網路由，無 default gateway／Internet | 無法依賴外部 NTP；需另規劃可達的本地時間來源 |
| `systemd-timesyncd` active，但系統未同步 | 服務啟動不等於已選用有效來源 |
| 車端顯示 `RTC time: n/a` | 當時無可用 RTC 資訊，不能假設斷電再啟動後時鐘正確 |
| Zenoh 出現 timestamp 超過 500 ms 而遭拒絕的警告，仍有 String Topic 到達 | 通訊可達不代表資料時間正確；500 ms 是該次版本／配置的觀察門檻 |

可使用 `date -Is`、`timedatectl status` 及 `ip -4 route` 分辨時鐘與網路狀態。上述證據不應解讀為 ROS2 或 Zenoh 本身損壞。

## 5.3 時間來源架構：可達性優先

```text
Windows / WSL2（dev-host）             Raspberry Pi（robot-host）
可用時間來源 → Chrony NTP Server   ◄── NTP / UDP ──  Chrony Client
                      IP1:1123                         IP2
                         └──── 有線診斷（已驗證） ─────┘
```

沿用 [01](01-os-hosts.md)、[03](03-network-ssh.md) 的代稱。當車端僅有直連網段又無 Internet 時，由可達的 WSL2 提供本地 NTP；無線恢復後應依獲准網路重新選擇可達且可信的時間來源，可以是可達的本地端點或核准的 NTP 服務，不必強制讓 NTP 與 Zenoh Router 同機。

### UDP 123 與 1123 的取捨

NTP 通常使用 UDP 123。既有 Windows 的 W32Time 已監聽 UDP 123；WSL2 Chrony 嘗試綁定專用位址的 123 埠時出現 `Could not open NTP socket`。本輪因此不停止 Windows 時間服務，改用 WSL2 UDP 1123 作為該拓撲的 NTP 端點。

| 端點 | 用途與限制 |
| --- | --- |
| `IP1:1123/UDP` | 本次有線 NTP Server，車端 Client 需明確指定非預設埠 |
| `IP3` 或其他獲准來源 | 無線方案的候選時間來源，尚未驗證，不直接照搬有線端點 |

此為當時的環境設計決策，UDP 1123 不是 ROS2 或 Chrony 的通用必要條件。

### WSL2 Chrony `-x` 模式

本輪 WSL2 的 `chronyd` 以 `-x` 運作，不直接校正 WSL2 系統時鐘；Chrony 仍能追蹤時間來源並維護提供給 NTP Client 的軟體時鐘。Windows 當時亦記錄 W32Time 來源為 Local CMOS Clock，未獨立證明 UTC 同步。

因此必須分辨三個對象：Windows／WSL2 系統時鐘、Chrony 提供的 NTP 時鐘、Pi 系統時鐘。歷史四時間戳 NTP 查詢顯示 WSL2 系統時鐘與其 NTP Server 的估算相對差約 `+0.433 ms`（單次量測），不是 Pi 與 WSL2 的長期誤差或 UTC 精度保證。若需要更嚴格精度，必須獨立驗證系統時間來源及網路延遲。

Chrony 官方說明：[chronyd `-x`](https://chrony-project.org/doc/4.5/chronyd.html)、[chronyc tracking](https://chrony-project.org/doc/4.5/chronyc.html#tracking)。

## 5.4 有線時間服務配置參考

下列為本輪設計的精簡配置模板，`IP1`、`IP2` 必須在獲准的實際測試網路替換為有效位址；不要直接覆寫既有 Chrony 設定。調整系統服務須在實車停止及管理授權的前提下進行。

```text
# WSL2 Chrony Server configuration (template)
bindaddress IP1
port 1123
allow IP2/32
```

```text
# Raspberry Pi Chrony Client configuration (template)
server IP1 port 1123 iburst
```

原測試以 `conf.d` 個別設定，車端預先存在 `makestep` 策略，無須重複加入；勿為了消除警告而強制移除 WSL2 的 `-x` 或在機器運動中手動調時。

Hyper-V Firewall 沿用 [04](04-zenoh-firewall.md) 的最小權限原則：本次有線測試僅針對獲准車端來源與專用開發端介面的 UDP 1123 配置入站規則。當時規則為 `Enabled=True`、`Action=Allow`、`EnforcementStatus=OK`；不關閉整個防火牆。重新連線或重灌後須核對規則與實際端點，不假設車端重灌會移除 Windows 規則。

## 5.5 車端離線安裝：可重現的跨架構依賴驗證

原有車端為 Ubuntu 24.04 ARM64，且當時無 Internet。先由目標車端 APT 索引、架構及模擬安裝確認需要的套件，再於可上網環境取得對應 `.deb` 及可信索引中的 SHA256 校驗資訊；不得把 AMD64 套件直接複製至 ARM64。

| 套件 | 歷史快照版本 | 架構 | 當時動作 |
| --- | --- | --- | --- |
| `chrony` | `4.5-1ubuntu4.2` | arm64 | 新裝 |
| `tzdata` | `2026e-0ubuntu0.24.04.1` | all | 升級 |
| `tzdata-legacy` | `2026e-0ubuntu0.24.04.1` | all | 新裝 |
| `systemd-timesyncd` | 當時版本 | — | APT 依相依性調整，由 Chrony 接手 |

以上僅為當時快照，不應在新設備固定下載相同版本。重建流程的工程關鍵是：確認架構與相依 → 取得可信套件及驗證資訊 → SHA256 檢查 → 模擬安裝 → 確認服務切換與安全狀態後安裝。可用以下指令進行必要檢查：

```bash
dpkg --print-architecture
apt-cache policy chrony
apt-get -s install chrony
# After transferring packages and the verified manifest to a dedicated directory:
sha256sum -c SHA256SUMS
apt-get -s install ./*.deb
```

**實際證據：** 三個下載套件均通過 SHA256 校驗。直接離線安裝曾因 APT 找不到對應 archives 而失敗，改依目標端仔細確認的版本及 APT cache 完成安裝；最終三個套件於 `dpkg -l` 均為 `ii`。這證明當時的離線相依處理成功，不代表所有 ARM64 主機會需要相同套件或版本。

## 5.6 時間同步驗收與量測解讀

先確認 NTP Server 可達、回覆格式有效，再驗證 Client 是否持續選用來源，最後檢查 ROS2 資料。收到 NTP 封包並不等於系統完成校時。

```bash
date -Is
timedatectl status
chronyc -n sources -v
chronyc tracking
```

| 既有實測結果（有線診斷） | 解讀 |
| --- | --- |
| 車端收到 NTP 回覆，Leap Indicator = 0、Stratum = 3 | NTP Server 在當時可達並返回有效時間資訊 |
| 車端日期由 2026-10-28 校正至 2026-11-20 | 實際系統時鐘發生大幅更新；該次初始偏差曾記錄約 −1,961,830 s |
| Client `Reference ID` 對應本地 NTP Server、Stratum = 4 | 來源選擇與拓撲相符 |
| `System time` 約慢 0.025 µs、`Last offset` 約 +23.89 µs | 特定時間點的 Chrony 狀態；不是跨主機絕對時間誤差 |
| `Leap status: Normal`、`System clock synchronized: yes` | 此次同步狀態符合基本驗收 |
| 重新驗證雙向 ROS2 Topic，未再觀察 timestamp 告警 | 同步後的通訊復驗通過（見 04） |

`System time` 是本機系統時鐘與 Chrony 維護的虛擬 NTP 時鐘之間的當前修正量；`Last offset` 是最近一次時鐘更新的估算偏差，兩者不可直接當成 Pi–WSL2 端到端時間差。長期驗收還須觀察來源被選用、Reach、LastRx、RMS offset、網路延遲及重啟後是否仍同步，不能用一次小數值宣稱微秒級實車精度。

## 5.7 移交與驗收界線

| 項目 | 狀態 |
| --- | --- |
| 有線 NTP Server、UDP 1123 及 Client 校時 | 已驗證（2026-11-20） |
| 有線同步後 ROS2 雙向 Topic 復驗 | 已驗證（見 04） |
| 無線正式場域的 NTP 來源、同步及重連 | 待復驗 |
| 跨主機相對時差的長期統計、UTC 絕對精度 | 尚未完整驗證 |
| IMU／JointState／Odometry／TF 實機端到端時間精度 | 待實機資料驗收 |

切換至無線網路後重新核對 NTP 來源及 Router 端點，完成時間同步再啟動控制與感測節點，避免沿用有線或舊時間戳。主機校時、Middleware 傳輸及應用層資料有效性必須分別驗收。

**參考：** [Chrony 4.5 Configuration Reference](https://chrony-project.org/doc/4.5/chrony.conf.html)、[Chrony 4.5 Monitoring](https://chrony-project.org/doc/4.5/chronyc.html)。

**接續：** [06 — 程式環境 Smoke Test](06-smoke-tests.md)。

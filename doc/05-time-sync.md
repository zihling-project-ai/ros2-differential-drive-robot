# 05 — Chrony、NTP 與 ROS 2 跨主機時間同步

> **狀態：** 2026-11-20 方案 A 已實測 PASS；車端使用 Chrony Client 向 WSL2 `192.168.123.2:1123/UDP` 校時。  
> **重要：** 在大幅調整 Raspberry Pi 系統時間前，須停止依賴時間戳的 ROS 2、TF、Odometry、SLAM 與實體控制程序。

## 5.1 故障背景

初次 ROS 2 Topic 雙主機測試時：

- WSL2：2026-11-20；可上學校網路。
- car06：2026-10-28；只有 `192.168.123.0/24` 本地路由，沒有 Default Gateway、無 Internet；`systemd-timesyncd` 為 active 但不同步。
- Zenoh：顯示 `incoming timestamp ... exceeding delta 500ms is rejected`，儘管 String Topic 仍能到達。

車端檢查：

```bash
ip -4 route
date -Is
timedatectl status
# 變更時間服務前曾使用
# timedatectl timesync-status
```

實測 `ip -4 route` 只有 `192.168.123.0/24 dev br0`，Ping 公網回傳 `Network is unreachable`。車端沒有可用 RTC（`RTC time: n/a`），無法假設斷電再開機後時鐘仍正確。

## 5.2 方案：WSL2 提供本地 NTP，Pi 作為 Client

```text
WSL2 Ubuntu 24.04
  192.168.123.2:1123 UDP ── Windows/Hyper-V Firewall ──► Pi car06
  Chrony Server                                      192.168.123.1
                                                     Chrony Client
```

Windows 上 W32Time 已在 UDP 123 監聽，且當時 `w32tm /query /status` 顯示 `Local CMOS Clock`、未同步。WSL2 Chrony 嘗試綁定 `192.168.123.2:123` 時回報 `Could not open NTP socket`。本輪避免停用 Windows W32Time，改以 WSL2 **UDP 1123** 提供 NTP。

**精度注意：** WSL2 裡 Chrony 因容器偵測以 `chronyd -x` 執行，不負責直接調整 WSL 系統時鐘。`chronyc tracking` 很小的修正量只描述與其選定來源的關係，不是 UTC 絕對精度保證。本輪另用四時間戳 NTP 查詢，驗證 NTP Server 與 WSL2 系統時鐘相對差約 `+0.433 ms`（單次量測）。若嚴格要求 UTC 精度，還需獨立驗證 Windows/Hypervisor 時間來源。

## 5.3 WSL2 安裝與設定 Chrony Server

**WSL2**（需要管理權限、學校政策允許）：

```bash
sudo apt update
sudo apt install -y chrony

sudo mkdir -p /etc/chrony/conf.d
sudo tee /etc/chrony/conf.d/aicar-ntp.conf >/dev/null <<'EOF_CONFIG'
allow 192.168.123.1/32
allow 192.168.123.2/32
bindaddress 192.168.123.2
port 1123
EOF_CONFIG

sudo systemctl restart chrony
systemctl is-active chrony
sudo ss -lunp | grep ':1123'
sudo chronyc accheck 192.168.123.1
```

其中對 `192.168.123.2/32` 的 `allow` 是提供 WSL2 **本機測試**使用；生產環境可按需要縮小。成功時應見：

```text
active
192.168.123.2:1123  ... chronyd
208 Access allowed
```

`chronyc accheck` 需要具備存取 Chrony 命令 Socket 的權限；一般使用者曾回報 `501 Not authorised`，改用 `sudo chronyc accheck ...` 後才取得正確的 ACL 判定。

### WSL2 服務檢查

```bash
chronyc tracking
chronyc sources -v
sudo journalctl -u chrony -n 50 --no-pager
ps -ef | grep '[c]hronyd'
```

如果日誌出現 `chronyd ... -x`、`Could not step system clock`，不要直接移除 `-x` 強制調時；先核對宿主系統時鐘與 NTP 來源。

## 5.4 Windows Hyper-V Firewall 開放 UDP 1123

**Windows PowerShell（管理員權限，需獲允許）：**

```powershell
$wsl = '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}'

New-NetFirewallHyperVRule `
  -Name 'AICAR-NTP-1123' `
  -DisplayName 'AICAR Chrony NTP UDP 1123' `
  -Direction Inbound `
  -VMCreatorId $wsl `
  -Protocol UDP `
  -LocalAddresses '192.168.123.2' `
  -LocalPorts 1123 `
  -RemoteAddresses '192.168.123.1' `
  -Action Allow

Get-NetFirewallHyperVRule -Name 'AICAR-NTP-1123'
```

本輪 `Enabled=True`、`Action=Allow`、`EnforcementStatus=OK`。不要為此關閉整個 Hyper-V Firewall。規則保存在 Windows，**重灌 Raspberry Pi 不會移除它**。

## 5.5 車端先驗證 NTP 封包，不更動時鐘

車端可用 Python 標準函式庫測試 UDP 1123：

```bash
python3 - <<'PY'
import socket
import struct
from datetime import datetime, timezone

request = b'\x23' + b'\x00' * 47
with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
    sock.settimeout(5)
    sock.sendto(request, ('192.168.123.2', 1123))
    data, address = sock.recvfrom(512)

assert len(data) >= 48, 'short NTP response'
leap = data[0] >> 6
stratum = data[1]
seconds, fraction = struct.unpack('!II', data[40:48])
unix_time = seconds - 2208988800 + fraction / (2**32)
print('NTP Server:', address)
print('Leap Indicator:', leap)
print('Stratum:', stratum)
print('Server UTC:', datetime.fromtimestamp(unix_time, timezone.utc).isoformat())
PY
```

本輪得到 `Leap Indicator: 0`、`Stratum: 3`、有效 UTC 時間。**收到封包不代表車端系統時鐘已經被校正。**

## 5.6 Raspberry Pi Chrony 離線安裝（本輪已實際完成）

車端 Ubuntu 24.04 ARM64 無 Internet；先在車端執行套件查核／模擬：

```bash
dpkg --print-architecture
apt-cache policy chrony
sudo apt-get -s install chrony
```

本輪需要：

| Package | 版本（當次快照） | 架構 | 安裝動作 |
|---|---|---|---|
| chrony | `4.5-1ubuntu4.2` | arm64 | 新裝 |
| tzdata | `2026e-0ubuntu0.24.04.1` | all | 升級 |
| tzdata-legacy | `2026e-0ubuntu0.24.04.1` | all | 新裝 |
| systemd-timesyncd | 原版本 | — | 由 APT 移除，Chrony 接手 |

在有網路的 **WSL2** 從車端 APT 索引中的 `Filename` 配合 `https://ports.ubuntu.com/ubuntu-ports/` 下載；**版本與 SHA256 必須按實際車端 APT 索引確認**。下列檔名為本輪記錄：

```bash
mkdir -p ~/aicar-chrony-debs
cd ~/aicar-chrony-debs
BASE='https://ports.ubuntu.com/ubuntu-ports'

curl -fL --retry 5 --retry-delay 3 --retry-all-errors -O \
  "$BASE/pool/main/c/chrony/chrony_4.5-1ubuntu4.2_arm64.deb"
curl -fL --retry 5 --retry-delay 3 --retry-all-errors -O \
  "$BASE/pool/main/t/tzdata/tzdata_2026e-0ubuntu0.24.04.1_all.deb"
curl -fL --retry 5 --retry-delay 3 --retry-all-errors -O \
  "$BASE/pool/main/t/tzdata/tzdata-legacy_2026e-0ubuntu0.24.04.1_all.deb"
```

建立校驗清單（雜湊來自**當次車端 `apt-cache show`**）：

```bash
cat > SHA256SUMS <<'EOF_HASH'
503d1f389dde1d540ae6f2c98bdac98040c70a2ce7d8da1458a8e04cd9dad1fd  chrony_4.5-1ubuntu4.2_arm64.deb
2f6794a7f31753fc947d9ac02ff03ea46510f34864f9d132a93d7b105e3c4447  tzdata_2026e-0ubuntu0.24.04.1_all.deb
da66b34322e7545f12bd97f2bd31c5f78406a9a9ec0ccfe284c3f3eb791647bf  tzdata-legacy_2026e-0ubuntu0.24.04.1_all.deb
EOF_HASH
sha256sum -c SHA256SUMS
```

本輪第一次下載 chrony 遇到 HTTPS `Connection reset by peer`，重試後三個檔案皆 `OK`。

經由 WSL2 複製到 Pi：

```bash
ssh ubuntu@192.168.123.1 'mkdir -p ~/aicar-chrony-debs'
scp *.deb SHA256SUMS ubuntu@192.168.123.1:~/aicar-chrony-debs/
```

車端：

```bash
cd ~/aicar-chrony-debs
sha256sum -c SHA256SUMS
sudo apt-get -s install ./*.deb

# 本輪直接 --no-download install ./*.deb 曾失敗：Unable to fetch archives。
# 將已校驗檔案置於 APT cache，再依確切版本安裝。
sudo install -m 0644 ./*.deb /var/cache/apt/archives/
sudo apt-get -s --no-download install \
  chrony=4.5-1ubuntu4.2 \
  tzdata=2026e-0ubuntu0.24.04.1 \
  tzdata-legacy=2026e-0ubuntu0.24.04.1

# 模擬結果符合預期才正式執行
sudo apt-get --no-download install \
  chrony=4.5-1ubuntu4.2 \
  tzdata=2026e-0ubuntu0.24.04.1 \
  tzdata-legacy=2026e-0ubuntu0.24.04.1
```

正式安裝前確認沒有控制／導航／感測時間戳程序需要持續運作。本輪完成後 `dpkg -l` 顯示三個套件狀態 `ii`。

## 5.7 車端 Chrony Client 設定與首次校時

Pi 的 `/etc/chrony/chrony.conf` 已有 `confdir /etc/chrony/conf.d` 與 `makestep 1 3`，**不需重複新增 `makestep`**。

```bash
sudo mkdir -p /etc/chrony/conf.d
printf '%s\n' 'server 192.168.123.2 port 1123 iburst' \
  | sudo tee /etc/chrony/conf.d/aicar-client.conf

# 停止依賴時間戳的 ROS 2 控制／導航程序後執行
sudo systemctl restart chrony
chronyc -n sources -v
chronyc tracking
date -Is
timedatectl status
```

首次同步會將車端日期從 2026-10-28 大幅校正至 2026-11-20。第一次 `Last offset` 曾記錄約 `-1,961,830 s`（歷史校時差），之後重新採樣結果：

```text
Reference ID : 192.168.123.2
Stratum      : 4
System time  : approximately 0.025 microseconds slow
Last offset  : approximately +23.89 microseconds
Leap status  : Normal
System clock synchronized: yes
```

不能只看一次大幅校時後的 `System time 0` 就判定穩定，應另看 `chronyc -n sources -v` 中的來源是否被選用、`Reach`、`LastRx`，以及後續 `tracking`。

## 5.8 驗收與環境移交

- 完成時鐘同步後**重新啟動**原先使用舊時間戳的 ROS 2 Node，避免 TF／Odometry 留有舊資料。
- 再驗證 [04 — Zenoh 雙向 Topic](04-zenoh-firewall.md)；本輪校時後未再看到 500 ms Timestamp 告警。
- Pi 若重灌，Chrony、其 Client 設定與 Netplan 都要重建；Windows 防火牆規則仍在。
- 切回學校 Wi-Fi 後，Pi 可能無法再到達 `192.168.123.2:1123`；應重新規劃可用時間來源，不要將此單一位址視為通用部署設定。
- Chrony 同步狀態與跨主機時間差，不等於嚴格的 UTC 絕對時間校驗。

**參考：** [Chrony 4.5 Reference](https://chrony-project.org/doc/4.5/chrony.conf.html)。

**接續：** [06 — 程式環境 Smoke Test](06-smoke-tests.md)。

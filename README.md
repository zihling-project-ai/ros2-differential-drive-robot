# AICAR — ROS 2 Jazzy 可重現環境、機器人規格與模擬驗證文件

> 適用環境：Windows 11 / WSL2 Ubuntu 24.04 / Raspberry Pi 4（Ubuntu 24.04 ARM64） / ROS 2 Jazzy。  
> 記錄基準：2026-11-20；連線狀態依現場網路與設備配置變動。

此儲存庫包含 AICAR ROS 2 專案及其環境部署紀錄。`doc/` 聚焦**安裝、網路連接、Zenoh、時鐘同步與功能驗證**；除環境操作外，收錄 Gazebo/RViz2 實驗、車輛硬體、URDF 模型規格、程式碼審查與跨環境部署。尚未取得的硬體型號、實測質心與物理校準值保持待確認。

## Repository 結構

```text
ros2-differential-drive-robot/
├── .github/workflows/ci.yml   # Python/XML 靜態檢查與 ROS2 套件建置
├── docker/Dockerfile          # ROS2 Jazzy 基礎容器環境
├── doc/                       # 00～10 文件與圖片
├── src/ai_car/                # AICAR ROS2 套件
├── .dockerignore
├── .gitignore
└── README.md
```

正式 Python 模組位於 `src/ai_car/ai_car/`。`src/ai_car/share/` 暫保留為參考版本，尚未合併差異；詳見第 09 份文件。課堂學習套件 `me333_pkg` 已自 `main` 移除，舊版本保留在 Git 歷史。

### CI 與相依套件

- GitHub Actions 只使用 `contents: read`，不需個人 PAT；執行語法/XML 與 `colcon build` 檢查，不啟動節點或實車、不發布 Docker Image。
- `package.xml` 是 ROS2 依賴的主要宣告；Dockerfile 管理系統環境。目前尚待盤點及補齊非 ROS2 Python 套件與硬體/圖形相依。僅當有需用 pip 管理的額外依賴時，再新增 `requirements.txt`，不要重複安裝 ROS2 套件。
- Docker Image 目前是基礎建置環境，不是完整 Gazebo/RViz2 或實車環境。請見第 10 份文件。

## 文件索引

| 文件 | 涵蓋內容 |
|---|---|
| [00 — 系統流程與兩端驗證](doc/0.系統流程.md) | 三種環境、八階段開發、跨主機驗證矩陣、SLAM 部署判斷 |
| [01 — 作業系統、設備名稱與帳號](doc/01-os-hosts.md) | Windows/WSL2、Raspberry Pi Ubuntu、`zihling@LV333-06`、`ubuntu@car06`、基本檢查 |
| [02 — ROS 2 Jazzy 與工作區](doc/02-ros2-workspace.md) | ROS 2 安裝參考、`colcon`、兩份 Workspace、VS Code 遠端開發 |
| [03 — 單／雙網卡與 SSH](doc/03-network-ssh.md) | 連線拓撲、介面 IP、Netplan、路由、Wi-Fi、SSH、故障排查 |
| [04 — Zenoh Router 與防火牆](doc/04-zenoh-firewall.md) | Router/Client 設定、Windows Hyper-V Firewall、7447/TCP、雙向 Topic |
| [05 — Chrony 與跨主機校時](doc/05-time-sync.md) | 時間戳、UDP 123 衝突、UDP 1123、Chrony 離線安裝與同步驗收 |
| [06 — 程式環境 Smoke Test](doc/06-smoke-tests.md) | Turtlesim、rqt_graph、Talker/Listener、`gz`/Gazebo 查核、RViz 與上實車前驗收 |
| [07 — Gazebo 與 RViz2 實驗](doc/07-gazebo-rviz2-aicar-experiment.md) | URDF/SDF、JointStatePublisher、控制振盪、底盤漂移與 RViz2 未完成項目 |
| [08 — AICAR 硬體與 URDF 物理資料](doc/08-aicar-hardware-urdf-physical-model.md) | BOM、DYNAMIXEL ID、質量/慣量/幾何、質心計算方法及尚未驗證項目 |
| [09 — ROS2 程式碼模組教學與審查](doc/09-aicar-ros2-code-review.md) | 模組與逐段學習、問題分級、參考/正式程式差異、修復與驗收 |
| [10 — AICAR 跨環境部署與 Docker](doc/10-aicar-docker-deployment.md) | AMD64/ARM64、Dockerfile、映像移轉、CI 與模擬隔離 |

## 網路拓撲與分工

```text
Windows 11 / LV333-06
├── 學校網路（上網）：10.117.1.231/24，gateway 10.117.1.1
└── 車端 Ethernet：192.168.123.2/24（無 default gateway）
     └── WSL2 Ubuntu 24.04（user: zihling）
          ├── rmw_zenohd Router → TCP 7447
          ├── chronyd NTP Server → UDP 1123
          └── RViz2 / SLAM Toolbox / Nav2（按測試需求啟動）
                    ⇅ Ethernet
Raspberry Pi 4 / car06
└── br0 = 192.168.123.1/24（eth0 bridge）
     ├── SSH / VS Code Remote-SSH（user: ubuntu）
     ├── rmw_zenoh_cpp Client
     ├── chronyd NTP Client
     └── ROS 2 Base / IMU / Odometry / LiDAR / Camera
```

上述 IP 是**2026-11-20 的現場觀測值**。`192.168.123.1` 為車端 Netplan 固定位址；本機 `192.168.123.2` 為第二張網卡設定。學校網路 IP 由環境決定，不應當作永久參數。

## 驗收狀態

| 子系統 | 狀態 | 證據／限制 |
|---|---|---|
| Ethernet、路由與 SSH | **PASS（本輪）** | 雙向可達，SSH 登入 `ubuntu@car06` |
| VS Code Remote-SSH | **PASS（本輪）** | 車端顯示 `SSH: aicar-ethernet` |
| Zenoh TCP 7447 | **PASS（本輪）** | 車端 `nc` 連線成功 |
| ROS 2 跨主機 Topic | **PASS（本輪）** | WSL2→車端與車端→WSL2 均收到測試 String |
| Chrony UDP 1123、車端校時 | **PASS（本輪）** | Pi `System clock synchronized: yes`、`Leap status: Normal` |
| 學校單網卡方案 | **歷史可用；本輪未復現** | 車端 Wi-Fi 原設定 SSID 未於掃描結果出現 |
| Turtlesim / rqt_graph | **PASS（本輪）** | 2026-11-20：Turtlesim GUI 與 Teleop 移動成功；rqt_graph 兩種檢視截圖確認 `/turtle1/cmd_vel` 與 RotateAbsolute Action 相關端點；見 [06](doc/06-smoke-tests.md#63-rqt_graph檢視-node--topic-連線) |
| Talker / Listener / Gazebo | **PASS（本輪）** | `/chatter` 本機測試；Gazebo AICAR 模型、重力、關節狀態均有現場證據，見 [06](doc/06-smoke-tests.md)、[07](doc/07-gazebo-rviz2-aicar-experiment.md) |
| Gazebo 位置控制 | **NOT PASS** | `tool_joint` 約 ±50 rad/s 交替，模型漂移；未確認根因 |
| RViz2 RobotModel | **尚未通過** | GUI 可啟動，但最終截圖只有 Grid，Fixed Frame=`map` 且有 Warning |
| Gazebo → ROS2 Bridge | **尚未驗證** | 模擬 Topic 尚未橋接到隔離命名空間 |
| 實車 SLAM / Nav2 | **尚未納入本輪驗收** | 軟體通訊正常不等於導航或馬達安全已通過 |

**標記定義：** `PASS（本輪）` = 有命令輸出直接佐證；`歷史可用` = 曾完成操作但本輪未重現；`待重新驗收` = 提供可執行步驟、尚無本輪結果；`建置參考` = 新設備重建流程，不表示本輪已重裝。

## 典型啟動順序

1. 確認 Windows 學校網路與獨立車端 Ethernet；以 `ping`、`ip route get` 驗證。
2. 用 VS Code `WSL: Ubuntu-24.04` 開啟本機 `~/ros2_ws`；用 `SSH: aicar-ethernet` 開啟車端 `~/ros2_ws`。
3. 確認 WSL2 Chrony UDP 1123、車端時鐘同步；**大幅校時應在 ROS 2 節點啟動之前**。
4. 本機啟動唯一的 Zenoh Router；雙方 ROS 2 Terminal 使用相同 `ROS_DOMAIN_ID`、RMW、Router Endpoint。
5. 用測試 Topic 做雙向驗證；確認無明顯時鐘錯誤，再執行 Turtlesim／CLI／RViz Smoke Test。
6. 最後才啟動涉及實體馬達的 AICAR 節點，並依設備安全程序驗收。

## 安全與重建注意

- 此處**不收錄**學校 Wi-Fi 密碼、SSH 私鑰、GitHub Token 或其他憑證。
- Windows Hyper-V Firewall 變更需有相應管理權限與設備管理者授權；不建議全面關閉防火牆。
- Raspberry Pi 重灌會移除 Pi 系統內的 Chrony、Netplan、Shell 設定；**不會**清掉 Windows 端的防火牆規則。
- 專案 `src/` 由 Git 保存；`build/`、`install/`、`log/` 應在目標機重新產生。保留原始教學程式的著作權與授權資訊。

## 技術參考

- [ROS 2 Jazzy：Ubuntu deb 安裝](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [ROS 2 Jazzy：Turtlesim 教學](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html)
- [ROS 2 `rmw_zenoh`](https://github.com/ros2/rmw_zenoh)
- [Microsoft：WSL Networking / Mirrored Mode](https://learn.microsoft.com/windows/wsl/networking)
- [Chrony 4.5 設定文件](https://chrony-project.org/doc/4.5/chrony.conf.html)



## 2026-11-21 文件重整與部署界線

- 先前 ZIP 是文件快照；目前 Repository 同時包含 `src/ai_car/`、`doc/0` 與 `doc/01`～`doc/10` 文件及原本 `doc/assets/` 截圖。完整的模擬/實車執行環境仍需重建及驗收。
- 原先 `01`～`06` 以已建立的 v1.2 文件為底稿，補入本次最新實驗結果；`07` 是當日實驗紀錄；`08` 是車體規格盤點。
- 教師公開 GitHub `zhl017/ai_car` 首頁目前顯示 ROS1 Noetic 版，與本次 ROS2 Jazzy 分支不同；切勿將 ROS1 `catkin_make/roslaunch` 指令當作這套工作區的操作步驟。
- **換電腦重現**不只 Clone：仍須重裝依賴、配置 WSL2/Zenoh/Chrony/Firewall/設備規則，並重新驗收 Topic、TF、物理模型和接觸運動。此版本尚未在第二台乾淨電腦完成端到端重現。
- 物理模型總質量 **1.291090686 kg 是既有 URDF 解析紀錄中的 Link 宣告加總**，並非實體車實測重量；整車 CoM 尚未計算。詳見 [08](doc/08-aicar-hardware-urdf-physical-model.md)。

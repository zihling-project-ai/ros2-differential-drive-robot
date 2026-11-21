# 10｜AICAR 跨環境部署與 Docker

更新日期：2026-11-21　｜　狀態：規劃／尚未建置或驗收

Repository 已加入 `docker/Dockerfile` 及 `.github/workflows/ci.yml` 的基礎版本。CI 目標為靜態檢查、建立最小 Jazzy Image、隔離容器內 `colcon build` 和 Package Discoverability，不執行實車、硬體測試或自動部署。此版本不保證所有硬體、視覺、導航與 Gazebo 相依已完整安裝。

## 1. 目標

在家用 Apple Silicon Mac 開發 AICAR ROS2，回學校可繼續相同版本的測試。避免在 macOS 分別安裝 ROS2、Gazebo、RViz2、Zenoh 等套件。以容器封裝環境、Git 管理程式與部署定義，不把實車環境直接當成家用模擬環境。

Docker 可降低套件版本差異，但不能解決所有 CPU、圖形加速、實體裝置與網路差異。Mac 仍需要可執行 Linux 容器的運行環境，例如 Docker Desktop；其背後會使用 Linux 虛擬機。

## 2. 環境矩陣

| 主機 | 用途 | 優先架構 | 注意事項 |
| --- | --- | --- | --- |
| 學校 Windows/WSL2 | Gazebo、RViz2、開發 | `linux/amd64`（以實際 `uname -m` 為準） | 完整圖形測試優先在此進行 |
| 家中 Apple Silicon Mac | 文件、程式碼、單元測試、無介面模擬 | `linux/arm64` | 不假設容器可直接存取原生 GPU 或顯示視窗 |
| Raspberry Pi | 實體裝置整合 | 以實際系統架構為準；64-bit OS 通常為 `linux/arm64` | 實車專用配置；不可用家中模擬設定誤啟動 |

統一基準：Ubuntu 24.04 Noble、ROS2 Jazzy、Gazebo Harmonic（選配）、RViz2（選配）、Zenoh（選配）。ROS 官方 `ros:jazzy-ros-base` 支援 AMD64 與 ARM64；Gazebo Harmonic 的 ARM 平台支援與圖形效能須單獨驗證。

## 3. 學會 Docker 的四個概念

| 名稱 | 意思 | AICAR 的例子 |
| --- | --- | --- |
| Dockerfile | 建立環境的配方 | 指定 ROS2 版本與依賴 |
| Image | 依配方建立的環境 | `aicar-dev:jazzy-arm64` |
| Container | Image 的運行實例 | 開啟 shell 執行 ROS2 測試 |
| Bind mount | 讓容器讀取主機資料夾 | 在 Mac 編輯程式，容器建置 |

建議把 Dockerfile、Compose 定義及源碼放 GitHub；映像檔是可匯入匯出的建置產物。只帶回 tar 映像並不等於帶回完整 Workspace：volume、bind mount、未提交的變更都需另行備份。

## 4. 建議資料夾結構（規劃）

```text
aicar_ws/
├── src/                 # 實際 ROS2 package（取得後放入）
├── docker/
│   ├── Dockerfile       # 基礎環境與依賴
│   └── compose.yaml     # 後續定義模擬服務
├── doc/
│   ├── 09-aicar-ros2-code-review.md
│   └── 10-aicar-docker-deployment.md
└── README.md
```

`src/` 中放 Package 的原始碼。不同 CPU 的 `build/`、`install/`、`log/` 不應直接共用或提交至 Git。實車與模擬的 Launch/parameters 應分離。

## 5. 第一階段：最小 ROS2 學習環境

以下是**尚未替 AICAR 建置驗收的示範配方**，需在學校或 Mac 的 Linux 容器環境實際測試。暫不安裝大型圖形套件，先驗證 ROS2 與跨平台基礎。

在未來的 `docker/Dockerfile` 內：

```dockerfile
FROM ros:jazzy-ros-base
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    ros-jazzy-demo-nodes-cpp \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /workspace
```

註：官方 ROS Image 已提供啟動用環境。互動 shell 可用 `source /opt/ros/jazzy/setup.bash` 明確載入 ROS2。實際 AICAR 依賴須從 `package.xml` 與正式原始碼確認；目前不能聲稱這個 Image 足以建置 AICAR。

在 Workspace 根目錄、已安裝 Docker 的主機執行：

```bash
docker buildx build --platform linux/arm64 --load \
  -f docker/Dockerfile -t aicar-dev:jazzy-arm64 .
```

在學校也可為自身環境建置 `linux/amd64`，使用 `aicar-dev:jazzy-amd64` 標籤。跨架構建置可能使用模擬，較慢；有條件時使用對應架構原生建置。

## 6. 在學校匯出，到家匯入

學校（確認 ARM64 映像已建置）：

```bash
docker image inspect --format '{{.Os}}/{{.Architecture}}' aicar-dev:jazzy-arm64
docker image save -o aicar-dev-jazzy-arm64.tar aicar-dev:jazzy-arm64
```

將 `.tar` 檔案與原始碼／配置一併帶回家。Mac（已安裝 Docker）：

```bash
docker image load -i aicar-dev-jazzy-arm64.tar
docker image inspect --format '{{.Os}}/{{.Architecture}}' aicar-dev:jazzy-arm64
```

`save/load` 保留 Image 與標籤，不保證會保存容器執行中的變更、volume、主機資料或外部設備。每次比較環境時，記錄 source commit、image ID/digest、Dockerfile 與執行測試結果。基礎 Image tag 會更新，穩定版可進一步固定 digest 與相依版本。

## 7. 在 Mac 開啟隔離的 ROS2 開發 Shell

在含 `src/` 的 Workspace 根目錄：

```bash
docker run --rm -it --network none --name aicar-dev-shell \
  --mount type=bind,source="$PWD/src",target=/workspace/src,readonly \
  aicar-dev:jazzy-arm64 bash
```

此示例只掛載原始碼為唯讀、不映射硬體設備、沒有外部網路連線。容器內的建置產物寫入 `/workspace` 的其他路徑，結束後將不保留。此環境適合先做隔離測試，不用於跨主機 Zenoh 或需要下載資源的工作。

```bash
source /opt/ros/jazzy/setup.bash
echo "$ROS_DISTRO"
ros2 pkg list
```

可使用 ROS2 官方 demo 在容器內做單機通訊驗收；AICAR 完整依賴取得後再加入建置與測試。不要啟動實體馬達驅動。容器中未來若要下載相依套件，應於 Image 建置階段完成，不要依賴每次臨時安裝。

## 8. 第二階段：Gazebo、RViz2 與 GUI

先對最小 Image 確認可用的架構與相依套件，再評估擴充：

| 套件 | 用途 |
| --- | --- |
| `ros-jazzy-ros-gz` | ROS2 與 Gazebo Harmonic 整合 |
| `ros-jazzy-rviz2` | ROS2 3D 視覺化 |
| `ros-jazzy-robot-state-publisher` | URDF + JointState → TF |
| `ros-jazzy-rmw-zenoh-cpp` | Zenoh RMW（待跨主機驗收） |

### 8.1 在家優先無介面模擬

Gazebo 安裝完成且 GUI 非必要時，可先在容器裡測試官方場景：

```bash
gz sim -s shapes.sdf
```

`-s` 表示只啟動 server，仍需檢查模型資源及渲染型感測器的支援情況。模擬 server 成功啟動不代表已完成 ROS2 Bridge。

### 8.2 圖形顯示方案

| 方案 | 優點 | 限制 |
| --- | --- | --- |
| 學校 WSL2/WSLg | 沿用已知環境 | 需在學校電腦使用 |
| Mac Docker 無介面 | 減少額外安裝，適合 Node/資料測試 | 沒有直接 3D 視窗；渲染感測器仍需驗證 |
| Linux 容器 + 瀏覽器遠端桌面（例如 noVNC） | Mac 不需安裝額外 X Server | 額外映像、顯示服務與資源；只應本機存取 |
| Ubuntu ARM64 虛擬機 | 完整 Linux 桌面 | 佔用更多磁碟、記憶體；3D 加速需測試 |

先驗證無介面模式，不必為追求 Gazebo 畫面同時安裝多套方案。

## 9. 模擬／實車隔離規則

- 家中開發 Image 不應配置 U2D2、GPIO、實車馬達或學校設備的連線；維持 `hardware_enabled=False`。
- 模擬工作先使用獨立的容器網路與 ROS Domain；與實車網路隔離，避免同名控制 Topic 混用。
- 不將 SSH key、Token、Wi-Fi 密碼或實車私有配置烘焙入 Image。
- 跨主機 Zenoh 在獨立測試網路中驗收，記錄 RMW、Router、QoS、時間同步；不把一般開發環境的網路設定直接沿用到實車。
- Docker 並不取代實車硬體限位、獨立安全監控與安全驗收。

## 10. 驗收清單

| 項目 | 通過標準 | 狀態 |
| --- | --- | --- |
| 建置 AMD64 Image | 學校可載入、啟動 | 待驗證 |
| 建置 ARM64 Image | Mac 可載入、啟動 | 待驗證 |
| Image 匯出／匯入 | 版本、架構、標籤正確 | 待驗證 |
| ROS2 CLI | ROS_DISTRO、Package 清單正常 | 待驗證 |
| ROS2 Node 測試 | 不使用真實設備，測試可重複 | 待驗證 |
| AICAR Build | 取得完整原碼後可重建 | 未取得原碼 |
| Gazebo headless | Server、模擬時間，實驗資料可輸出 | 待驗證 |
| RViz2 GUI | 顯示正確的 URDF/TF | 待驗證 |
| Bridge | 型別、方向、時間正確 | 待驗證 |
| 安全隔離 | 模擬不與實車控制通訊 | 待驗證 |

## 11. 參考資料

- [Docker 多架構建置](https://docs.docker.com/build/building/multi-platform/)
- [官方 ROS Image 架構與標籤](https://github.com/docker-library/official-images/blob/master/library/ros)
- [Docker image save](https://docs.docker.com/reference/cli/docker/image/save/)
- [Docker image load](https://docs.docker.com/reference/cli/docker/image/load/)
- [Docker Desktop 備份與還原](https://docs.docker.com/desktop/settings-and-maintenance/backup-and-restore/)
- [Gazebo Harmonic 與 ROS 安裝](https://gazebosim.org/docs/harmonic/ros_installation/)
- [Gazebo Harmonic 平台說明](https://gazebosim.org/docs/harmonic/install/)
- [09｜程式碼模組教學與審查](09-aicar-ros2-code-review.md)

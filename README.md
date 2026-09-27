# RecordHelper

给 Intel RealSense D435i 用的一键录制工具：把双目红外 + IMU（+ 深度、设备端位姿）录成
`unav_vio` 回放侧**可以直接吃**的 EuRoC 风格数据集，录完立刻自检，不用再做二次转换。

C++ 侧纯命令行，无 GUI 库、无 ROS、无 OpenCV、无 Ceres，依赖只有 `librealsense`、`Eigen`、`zlib`。
另有一个**可选**的 Python/tkinter 驱动层 `tools/rh_gui.py`：它不链接进库，只是把命令行按顺序调起来。

```
rh probe    # 读设备能力：内参/外参/IMU 速率/时间戳域/两目几何摘要（不写文件）
rh record   # 分阶段引导录制：预热 → 镜头朝上静止 → 温和激励 → 录制（手动停） → 尾部覆盖 → 出盘 → 自检
rh verify D # 按 unav_vio 的解析规则审查任意 mav0 数据集（含既有录制，例如 room_02）
```

## 为什么需要它

`/Users/mac/datasets/room_02` 是 Linux 上用 ROS 的 `realsense2_camera` 节点录的，导成 EuRoC
格式后 `unav_vio` 完全回放不了。`rh verify` 把它的问题钉成了可执行检查（8 项 FAIL）：

| 症状 | 根因 |
| --- | --- |
| `sensor.yaml` 根本解析不进来 | 导出器写的是 YAML block sequence（`- 640`），而当时 `euroc_dataset.cpp` 的最小 YAML 解析器只认 flow 列表（该子集已于 2026-09-21 放开，见 `docs/format-contract.md`） |
| 标定被拒 | `imu0/sensor.yaml` 没有四个噪声键，而 `validate_calibration` 要求它们严格 > 0 |
| 左右对不上 | 左右 image sequence 独立丢帧，`synchronize_stereo_stamps` 要求 ns 全等，`dropped_count=41` |
| IMU 是假数据 | ROS `unite_imu_method` 把 250 Hz 加表零阶保持到 400 Hz 陀螺栅格，38.3 % 的行在重复上一个加表值 |
| 初始化失败 | 开头没有静止段（最静窗 `accel_rms=0.51 / gyro_rms=0.17`，阈值 0.1/0.1），回放找不到静止→运动边界，anchor 无从确定 |
| 几何口径存疑 | 三个 `T_BS` 旋转都是单位阵。单位阵本身不必然错——它取决于「写进 `data.csv` 的加表数据在哪个系里」，而 room_02 那条路径（ROS `realsense2_camera` 重分发）和 SDK 直读并不保证同口径。所以这一条只能实测判定，不能照抄任何文档：见下面「静止段重力方向核对」 |
| 无指标 | 没有 `state_groundtruth_estimate0`，`integration.mh01_replay` 会在加载 GT 时早退 |

RecordHelper 在录制端逐条堵住这些坑：**只写 flow 列表**、**噪声从本次静止段实测**、
**一对双目共享同一个时间戳**、**IMU 统一到一条原始时间戳栅格且绝不零阶保持（`--imu-grid`）**、
**引导你完成静止 + 温和激励**、
**外参取设备标定并做正交化、再用静止段实测重力方向核对它与数据是否同一个系**、
**可选录设备端位姿流当伪 GT**。

## 构建

录制请用 **Linux**：当前 Homebrew librealsense 在 macOS 上不枚举 IMU。源码两边都能编，单测不需要相机。

### Linux

```bash
# 依赖：cmake / g++ / Eigen / zlib，外加一份能被 find_package 找到的 librealsense2
sudo apt install cmake g++ libeigen3-dev zlib1g-dev
# 让用户态能打开 D435i 的 USB 与 IMU（Linux 上 IMU 走内核 hid_sensor/IIO，
# 规则里 iio / hid_sensor 那两行是必需的，只有 usb+hidraw 会报 scan_element Permission denied）：
sudo cp pack/99-realsense.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
# 拔插一次相机后再：
cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release
cmake --build build-release --parallel
ctest --test-dir build-release --output-on-failure
```

`librealsense` 必须能被 `find_package(realsense2 CONFIG)` 找到。走 Intel 的 apt 源装
`librealsense2-dev` 时上面那条命令就够了；用的是 ROS 二进制包
（`sudo apt install ros-jazzy-librealsense2`，2.58.4，自带头文件和 CMake 配置）时，先
`source /opt/ros/jazzy/setup.bash`，或者显式给前缀：

```bash
cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/opt/ros/jazzy
```

不这么做会在 configure 阶段就停在 `realsense2_DIR` 找不到上头。

可执行文件是 `build-release/rh`，直接跑，一般不需要 sudo。

### macOS

```bash
brew install librealsense            # 2.58.x，arm64 有 bottle
cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release
cmake --build build-release --parallel
ctest --test-dir build-release --output-on-failure
```

`build-release/rh` 是指向 `build-release/rh.app/Contents/MacOS/rh` 的软链接。USB 相机接口要 TCC
摄像头权限，且 **IMU 在当前 SDK 里是关掉的**，所以这里只能 `verify` 已有数据集，不能录回放用的包。

第一次授权不要用 Cursor / VS Code / CLion 的内置终端，权限会记到 Helper 进程上。用系统「终端.app」：

```bash
open -a Terminal /Users/mac/src/record_helper
./build-release/rh probe
```

点允许后若仍是 `failed to set power state`，那是系统 UVC 占着 USB 接口。librealsense 从 macOS 12
起的官方做法是管理员权限；即便如此，Homebrew 2.58 也不会枚举 Motion Module。

## 录制

在 Linux 上、有控制终端的窗口里：

```bash
./build-release/rh probe
./build-release/rh record --out ~/datasets --seq room_03 --exposure-us 3000
./build-release/rh record --out ~/datasets --seq room_04 --still 5 --excite 4 --exposure-us 4000 --no-pose-gt
./build-release/rh record --out ~/datasets --seq room_05 --imu-grid accel   # 两列都要实测值，陀螺抽稀到 250 Hz
./build-release/rh record --out ~/datasets --seq room_06 --no-depth --exposure-us 4000 --duration 90
```

`--out` 默认是**当前目录**，不设就会把几百 MB 的包写进仓库里，示例里全部显式给。
`--no-depth` 不写 `depth0`（unav_vio 根本不读它，只留档），实测 94 s / 2828 对双目无深度的包是
**394 MB**；按 848×480 Z16 @ 30 fps 估算，带上深度大约再翻两三倍（这个数没实测过，只当量级看）。
`record` 里没有剩余空间检查，`df -h` 先看一眼。

| 阶段 | 谁推进 | 你做什么 |
| --- | --- | --- |
| 预热 | 自动 | 等出流（≥3 对双目 + 若干 IMU） |
| 静止标定 | 终端门控，`--still` 秒（默认 4） | **镜头竖直朝上**平放桌上、完全不动。不能跳。结束时核对重力方向，摆错或口径不一致都当场中止 |
| 温和激励 | 终端门控，`--excite` 秒（默认 3） | 缓慢平移 30–50 cm + 小幅偏航/俯仰。Enter 可提前结束激励 |
| 正式录制 | **只手动停** | 正常走、停、转弯都可以。停下超过 1.5 s **不会**改阶段，写盘也不会把前面的帧裁掉 |
| 尾部覆盖 | 手动结束后自动约 0.6 s | 停在原地，让 IMU 盖住最后一帧。再按一次 Enter / q / Ctrl-C 可立刻出盘 |

结束正式录制：`Enter` 或 `q` 或 Ctrl-C，然后自动收尾写盘并 `verify`。`x` 放弃。静止标定不能跳。
标定或激励阶段按 `q` / Ctrl-C 会放弃，不会跳过静止直接出盘。

`--duration` 只是状态行上的建议时长（IMU 要过 30000 行，`--imu-grid gyro` 约 75 s、
`accel` 约 120 s），**到点不会自动停**。

回放侧 `unav_vio` 仍取序列里**最后一个**静止→运动边界做初始化。本工具不再因为中途停下而改录制状态或删帧；若你希望回放从开头那次标定起步，正式录制里就不要再出现一段够长的静止后又起步。

### `--imu-grid gyro|accel`：IMU 栅格选哪条流

`imu0/data.csv` 一行只有一个时刻，而 D435i 的加表（250 Hz）和陀螺（400 Hz）各有各的节拍，
所以必须挑一条流的时间戳当栅格、把另一条流配上去。room_02 就是在这一步被 ROS 驱动的
`unite_imu_method=copy` 用零阶保持糊过去的（38 % 的行在重复上一个加值）。
RecordHelper 两种模式都**不做零阶保持**，配不上的时刻一律整行丢弃：

| `--imu-grid` | 栅格 | 另一条流怎么处理 | 代价 |
| --- | --- | --- | --- |
| `gyro`（默认） | 陀螺原始时刻，400 Hz | 加表**线性内插**到陀螺时刻 | 加表列是算出来的，设备在那个时刻并没有测过 |
| `accel` | 加表原始时刻，250 Hz | 陀螺**取时间最近的实测样本**（抽稀，不造数） | 陀螺少 150 Hz；每行 `w` 可能偏 ±1.25 ms；30000 行要录约 120 s |

要「一个样本都不造」就 `--imu-grid accel`；要陀螺速率就留着默认。选中的栅格、重采样方式、
行速率与 `max_source_skew_ms`（抽稀模式下栅格时刻与所取陀螺样本的实际最大偏移）都写进
`record_summary.yaml`，事后能从文件本身分辨两列哪些是实测、哪些是内插出来的。
`verify` 对两列各报一个重复比例（`accel_hold_ratio` / `gyro_hold_ratio`），防止抽稀退化。

### 静止段重力方向核对

静止标定阶段终端会要求把设备**镜头竖直朝上**平放。此时「世界上」就是光学系 +z，静止段平均比力
（重力反力，指向世界上）在 body 系里必须落在 `T_BS` 旋转的**第三列**上。静止段一结束就打印实测
夹角和它对应的摆法，超过 25° 当场中止、不出盘。**摆错姿态和口径不一致都会撞同一道中止**，
提示语里那行「实测是 (…) → 镜头水平朝前、画面正立」就是让你据此重摆；重跑不占磁盘。

这条查的是「`data.csv` 里的加表数据到底交在哪个系里」，和 `CheckGravity` 的模长检查正交：
设备外参表、librealsense 构建/后端、内核 `hid_sensor` 驱动，三者任何一个换了轴向口径，
模长都照样好看（9.81 附近），而这条会直接跳到 90°/180°。恒等外参本身不必然错——本机
SDK 直读实测下来，记录系与外参表同口径，单位阵成立（2026-09-21，librealsense 2.58.4 +
RSUSB 后端 + 内核 hid-sensor/IIO）。换机器、换构建都要重看这一行，夹角也写进
`record_summary.yaml` 的 `imu.static_gravity_vs_t_bs_col3_deg` 留痕。

录完退出码 0 表示契约通过。

## 回放

`unav_vio` 侧先构建一次（它自己的 `find_package(uav_nav CONFIG REQUIRED)` 要求把 CMake 前缀
指到已安装的 SDK；Ceres 要 2.2.0）：

```bash
cd <unav_vio>
cmake -S . -B build-replay -DCMAKE_BUILD_TYPE=Debug -DCMAKE_PREFIX_PATH=<uav_nav 的 sdk-install>
cmake --build build-replay --target vio_mh01_replay --parallel
```

新构建的可执行文件叫 `vio_mh01_replay`；旧目录里可能仍是 `vio_mh01_replay_test`。JSONL 在线播放还要
`vio_observation_playback`（只在找到 `uav_nav_rerun` 时才会编进来）。

然后指着我们录的目录跑：

```bash
env UNAV_VIO_EUROC_MH01=~/datasets/room_03 \
    UNAV_VIO_MH01_MAX_FRAMES=300 \
    ctest --test-dir build-replay -R '^integration.mh01_replay$' --output-on-failure
```

数据集根目录直接就是 `UNAV_VIO_EUROC_MH01` 该指的地方，不需要任何中转脚本。
`-R` 请写全名：`-R mh01_replay` 会连 `integration.mh01_replay_rerun` 一起跑，时间翻倍
（Debug 下 300 帧单次约 5 分钟）。`MAX_FRAMES` 不设时默认也是 300，想跑完整段写 `0`。

设备没有位姿流（`--no-pose-gt`，或本机这种根本不枚举 `RS2_STREAM_POSE` 的构建）时，包里就没有
`state_groundtruth_estimate0/`。加 `UNAV_VIO_MH01_REQUIRE_GT=0` 可以让上游回放测试只放弃「轨迹精度」
那一层判据（ATE/RPE），其余——解析、标定、双目硬同步、初始化及时性、状态发布完整性、求解器统计——
照常要求；默认不设该变量时仍然必须有 GT，结论行会印 `require_gt=0/1` 以便事后区分：

```bash
env UNAV_VIO_EUROC_MH01=~/datasets/room_03 UNAV_VIO_MH01_MAX_FRAMES=300 \
    UNAV_VIO_MH01_REQUIRE_GT=0 \
    ctest --test-dir build-replay -R '^integration.mh01_replay$' --output-on-failure
```

跑完想留可看的东西，再加两个变量：`UNAV_VIO_REPLAY_ARTIFACT_DIR=<空目录>` 落
`estimated.tum` / `trace.csv` / `manifest.json` / `metrics.json`（该目录必须为空，它拒绝覆盖），
`UNAV_VIO_RERUN_SAVE=<路径>.rrd` 录 Rerun 记录，事后用 Rerun viewer 打开。**SAVE 模式默认不写
相机画面**（只有轨迹/点云/坐标轴）——上游把画面留给 live 模式或显式开关，免得整段双目原图落盘；
要画面就得再加 `UNAV_VIO_RERUN_IMAGES=1`：

```bash
env UNAV_VIO_EUROC_MH01=~/datasets/room_03 UNAV_VIO_MH01_MAX_FRAMES=300 \
    UNAV_VIO_MH01_REQUIRE_GT=0 \
    UNAV_VIO_RERUN_SAVE=/tmp/room_03.rrd UNAV_VIO_REPLAY_ARTIFACT_DIR=/tmp/room_03-artifacts \
    UNAV_VIO_RERUN_IMAGES=1 \
    ./build-replay/vio_mh01_replay
<rerun 可执行> /tmp/room_03.rrd      # 或加 --serve-web 后用浏览器打开它打印的地址
```

画面是叠加了特征跟踪结果后渲染成 RGB8 的图，不是原始 PNG，实测约 **0.56 MB/帧对**（848x480 双目，
`d435i_20260924_164630` 跑 300 帧得 167,711,289 B、`rerun_image_records=600`）：整段五千多帧就是
3 GB 量级。结论行里的 `rerun_image_records=帧数×2` 就是"画面真的进去了"的证据，为 0 说明这个变量没生效。

标记本身是 unav_vio 提交 `0f38114`（09-24）才有的：**旧二进制写出的 `.rrd` 里相机面板是纯灰度图**，
每帧对只占 0.31 MB。所以"画面是干净的、没有点"通常意味着构建目录里的二进制比源码旧，判据很机械——
灰度图每个像素满足 R=G=B，抓一张截图统计通道偏离即可（标记色是绿 (40,255,40)、洋红 (255,0,255)、
黄 (255,220,0)，偏离都是满量程级）。早期帧里洋红（新检测）占多数，收敛之后才是绿点加位移箭头。

### 换到较新的 unav_vio 后回放会多出的几道旋钮

这些都在回放测试里，录制端管不着，但会直接让「昨天还能跑」的数据集今天跑不动：

1. **图像质量前置门**：`image_quality=fail ... 拉普拉斯方差中位数=1.9 < 下限=50` 会在喂帧之前就拒绝。
   D435i 的暗光红外素材本来就低对比（本机实测 `levels_median=54`、`lap_median=1.9`、`mean_grey=21.5`），
   所以默认下限基本必拒。用 `UNAV_VIO_MH01_IMAGE_LAP_MIN=1`（必要时配 `UNAV_VIO_MH01_IMAGE_LEVELS_MIN`）
   放宽——这是**观察侧旋钮**，放宽它不等于素材适合做 VIO。
2. **初始化等待预算**：上游默认 `max_bootstrap_frames=150`（EuRoC 20 Hz 约 7.5 s）。D435i 30 Hz
   还要先攒 0.5 s 静止，`d435i_20260924_164630` 的 Gate 1 留档 `first_state_frame=230`。不设
   `UNAV_VIO_MH01_BOOTSTRAP_FRAMES` 会在 150 帧处放弃，随后 `未及时初始化` / `states=0` /
   双目 ingress `status=5`（ComponentFailed）。这只加长「还允许再等多久」，不改任何判据。
   同一次成功留档还开了 `UNAV_VIO_MH01_FRONTEND_CONTRAST=1`，短窗 W1 用
   `UNAV_VIO_MH01_IMAGE_LEVELS_MIN=25`（出厂 32 不变）。
3. **相机时间偏移**：较新的回放会从 `sensor.yaml` 读 `timeshift_to_imu_ms`（结论行印
   `td_effective_ms=6 td_source=dataset_sensor_yaml`）。本仓库的录制端**不写这个键**，所以它只可能来自
   事后手改的数据集（2026-09-24 22:34 `estimate_td.py` 把实测 +6 ms 回写进了 `d435i_20260924_164630`）。
   同一份录制在 2026-09-26 用同一二进制做过全矩阵 A/B：td=6 时无论怎么配，全程回放都会中途
   NumericalFailure 停机——默认 bootstrap 下初始化被拖出预算（~200 帧处死、`states=0`），
   加长 `BOOTSTRAP_FRAMES` 后会在 461 帧处被健康门（连续 5 帧不健康）停机（300 帧短窗恰好
   死在窗口之前，所以 `states=71`/`states=219` 的短窗"通过"只说明还没跑到死点）；
   `UNAV_VIO_MH01_TD_MS=0`（或删掉 sensor.yaml 里那行键）后，配合 `FRONTEND_CONTRAST=1`
   与加长 bootstrap 能整段跑完（`first_state_frame=107`、5160 个状态连续发布到 5266 帧，
   与 Gate 1 全程留档一致）。症状上这表现为「回放播了几百帧之后轨迹/landmark/特征标记
   全部消失而画面照常」，极易误判成回放坏了。
   `rh verify` 现在会盯住这个键：录制产物（带 `record_summary.yaml`）里手写非零值判 FAIL，
   外来数据集给 WARN（`"0"` 必须显式传，空字符串表示不覆盖——对缺键数据集两者等价）。

Rerun 查看器不在 PATH 里，也不随本项目安装：它得是 **0.37.x**（`uav_nav_rerun 0.5.0` 传递依赖
`rerun_sdk 0.37.1 EXACT`），本机用的是 `~/桌面/rerun-cli-0.37.1-x86_64-unknown-linux-gnu`。
`.rrd` 是否完整可以用 `rerun rrd verify <文件>` 判。

live 推流（边跑边看）与 SAVE 互斥：先 `rerun --serve-web --bind 127.0.0.1 --port 9876`，
再用 `--connect` 跑，并且**不要**同时给 `UNAV_VIO_RERUN_SAVE`——回放侧的连接端点是写死的
`rerun+http://127.0.0.1:9876/proxy`。

### JSONL 在线播放（省内存）

整段带画面的 `.rrd` 会按时长涨到数 GB，Rerun 打开时还要把历史常驻在 Viewer 里。较新的 unav_vio
把观看拆成两步：回放时只写一份**不复制 PNG** 的观察留档（JSONL），再由 `vio_observation_playback`
按源曝光时间读 JSONL + 原始 `cam0`/`cam1` PNG，live 推给 Viewer。它不重跑 VIO，CLI 也不提供
SAVE。全段观察留档大约是一百多 MB（本机 5426 帧约 146 MiB），不是第二份 5 GB `.rrd`。

回放时设 `UNAV_VIO_OBSERVATION_ARCHIVE=<路径>.jsonl`；上游**拒绝覆盖**已有文件。可与
`UNAV_VIO_RERUN_SAVE` 并存（观察者挂在同一条回调上），但在线播放本身不要再走 SAVE：

```bash
env UNAV_VIO_EUROC_MH01=~/datasets/d435i_20260924_164630 \
    UNAV_VIO_MH01_MAX_FRAMES=300 \
    UNAV_VIO_MH01_REQUIRE_GT=0 \
    UNAV_VIO_MH01_BOOTSTRAP_FRAMES=300 \
    UNAV_VIO_MH01_FRONTEND_CONTRAST=1 \
    UNAV_VIO_MH01_IMAGE_LEVELS_MIN=25 \
    UNAV_VIO_MH01_IMAGE_LAP_MIN=1 \
    UNAV_VIO_OBSERVATION_ARCHIVE=/tmp/room_03-observation.jsonl \
    UNAV_VIO_REQUIRE_OBSERVATION_ARCHIVE=1 \
    ./build-rerun060-release/vio_euroc_replay
```

`observation_archive=pass` 和 `vio_result=pass` 是两件事：全段这份数据上 VIO 仍可能
`fail`（失锁），留档照样能播。上游拒绝覆盖已有 JSONL，坏档要先删再生成。

先起 live Viewer（`--memory-limit` 是 Viewer 保护，不能代替抽帧），再播。`--display-hz 5`
按墙钟抽 RGB（默认 `0` = 每帧都发，与源 30 Hz 对齐）；State / landmark 仍每帧推，不会因为
抽帧丢掉数值。观看路径不要加 `--repeat-shifted`（那是内存验收加倍样本用的）：

```bash
<rerun 可执行> --serve-web --bind 127.0.0.1 --port 9876 --memory-limit 4GB
./build-replay/vio_observation_playback \
    --archive /tmp/room_03-observation.jsonl \
    --dataset ~/datasets/room_03 \
    --connect --display-hz 5
```

`--dataset` 可省略，播放器会用留档头里的 `dataset_path`。没 Viewer 时 `--connect` 只记
`viewer=fail`、退出码仍为 0；要硬失败再加 `--require-viewer`。

### 把留档画成轨迹图

回放的 `artifacts/` 里有一份 `estimated.tum`（无 GT 时的估计轨迹）。出图用它，不再需要一次性脚本：

```bash
python3 tools/rh_report.py /tmp/room_03-artifacts        # 写 <产物目录>/../trajectory.png
python3 tools/rh_report.py <目录> --output /tmp/t.png --title "room_03"
python3 tools/rh_report.py --selftest                    # 合成方环自检，不需要 DISPLAY
```

左图是 XY 平面（首末状态各标一个点），右图是 XYZ 随时间；标题里给 `states / 时长 / XYZ 跨度 /
path（累计路径长）/ endpoint（首末距离）`。**path 和 endpoint 是两码事**：闭环判据看后者除以前者
（run_02 是 5.04 m / 155.9 m = 3.2%）。看形状、闭环、跳变可以，判断精度不行——没有 GT。它**拒绝
覆盖**已有图。依赖 numpy/matplotlib（本机 python3 自带 matplotlib 3.6.3），缺了会直接说装什么。

## 图形界面（可选）

```bash
python3 tools/rh_gui.py            # 需要 DISPLAY
python3 tools/rh_gui.py --selftest # 无界面检查参数拼装与磁盘估算，不需要 DISPLAY
python3 tools/rh_gui.py --smoke    # 构造界面 1.5 s 后自动关闭，验证布局代码不抛异常
```

它只是把上面几节的命令按顺序调起来，**不给 C++ 库引入任何依赖**（tkinter 是标准库自带的）。

- **设备**：跑 `rh probe`，输出进界面日志。
- **录制**：填参数 →「检查参数与磁盘」做离线预演（`--still/--excite/--tail` 的下限、
  `--duration` 在当前栅格下够不够 30001 行 IMU、目标盘余量对比估算体积）→
  「在终端里开始录制」用 `gnome-terminal`/`konsole`/`xterm` 开一个**真实终端**跑 `rh record`。
  阶段推进（Enter / q / x）和你拿相机走动都在那个窗口里——把键盘塞进管道会让"操作员看着
  终端决定何时停"这个前提失效，所以这里刻意不做内嵌伪终端。
- **数据集**：列 `<输出目录>/*/mav0`，标出大小与缺不缺 GT/深度；选中后跑 `rh verify`，
  刷新列表会保留你的选中项。
- **回放**：直接调 unav_vio 构建目录里的 `vio_mh01_replay`（找不到再试旧名 `vio_mh01_replay_test`），
  环境变量与上面「回放」小节完全一致（`UNAV_VIO_MH01_REQUIRE_GT`、`MAX_FRAMES`、
  `UNAV_VIO_MH01_BOOTSTRAP_FRAMES`、`UNAV_VIO_MH01_FRONTEND_CONTRAST`、
  `UNAV_VIO_RERUN_SAVE`、`UNAV_VIO_REPLAY_ARTIFACT_DIR`、`UNAV_VIO_RERUN_IMAGES`、
  `UNAV_VIO_OBSERVATION_ARCHIVE`、`UNAV_VIO_REQUIRE_OBSERVATION_ARCHIVE`）。默认构建目录优先选本机带 JSONL 播放器的
  `~/unav_vio/build-rerun060-release`。产物目录非空、JSONL 已存在时先自己拒绝，
  而不是等子进程报"拒绝覆盖"。
  「把双目画面写进 .rrd」默认**勾上**（界面是给人看东西的），日志会先按 0.56 MB/帧估一次体积并
  对比目标盘余量；取消勾选就回到上游 SAVE 的默认行为，只有轨迹和点云。
  「前端对比度归一化」默认勾上（与 Gate 1 成功 JSONL 同口径）；bootstrap 按
  `max(MAX_FRAMES, 240)` 自动设，整段（0）用 8192。
  「回放写 .rrd」走 SAVE；「生成 JSONL」另有一颗按钮，同样跑一遍 VIO，但只设
  `UNAV_VIO_OBSERVATION_ARCHIVE`、不写 `.rrd`。JSONL 路径默认 `/tmp/rh_gui-observation.jsonl`；
  上游拒绝覆盖，已有文件时「生成 JSONL」会拦下，「回放写 .rrd」则跳过观察留档只写 `.rrd`。
  日志出现 `observation_archive=pass` 即认为 JSONL 写完，即使退出码非 0（那只说明
  `vio_result=` 没过）。写出后再点「JSONL 在线播放」调 `vio_observation_playback --archive … --connect`；
  「display-hz」留空则不传该开关（CLI 默认 0=每帧都发；填 `5` 才抽帧）。
  在线播之前先点「打开 live Viewer」（`--serve-web` 监听 `127.0.0.1:9876`，带
  `--memory-limit 4GB`）；「关闭 live Viewer」只杀本界面拉起的那一组进程，终端里自己开的
  请在那边停。关窗口时也会顺手停掉它。
  「回放侧覆盖」四个小格对应图像质量门、`UNAV_VIO_MH01_TD_MS` 和停滞预算 `UNAV_VIO_MH01_WAIT_MS`。
  **lap 下限预填 1、levels 预填 25**（D435i 暗光红外过不了上游默认 50/32；短窗 Gate 1 的 W1
  抽样 `levels_median=25`）。**td 留空**，用数据集 `sensor.yaml` 里的值；不要把「未及时初始化」
  默认当成 timeshift 问题。**停滞 ms 留空**＝用上游默认 120000；只有 JSONL 留档写到一半被
  `TimedOut` 时才需要抬（留档每行 flush、跑在 Component 唯一的后台任务里，盘慢会节流消费）。
  三个开关另外单独列：`要求 GT`、`前端对比度归一化`（暗光红外建议开）、
  **`只准静止段起步`**（= `UNAV_VIO_MH01_ALLOW_MOTION_START=0`；允许运动段起步时初始化会在
  剧烈晃动的那一帧锁锚，run_01 曾因此在 0.9 s 内撞上 `max_speed` 健康门）。
  填了非数字会在日志里说明
  "这次按上游默认值"，而不是把坏值塞给子进程。
  路径框都带「浏览…」按钮，其中「输出 .rrd」和 JSONL 用的是**另存为**对话框——那个文件通常还没
  生成，拿目录选择器会逼你先手工建一个文件。
  「出 trajectory.png」调 `tools/rh_report.py`（见上一节），产物目录里没有 `estimated.tum` 时
  只提示、不空点。
  「打开 .rrd」先找 PATH 里的 `rerun`，找不到再找 `~/桌面/rerun-cli-*`。它是另开一个
  进程去**读**那个 `.rrd`，所以点它不会打断回放；真正的问题是时机——`.rrd` 的 footer 只在写入方
  关闭文件时才写上，回放途中打开只会得到 `Missing RRD footer / no RRD manifests`。因此日志出现
  「.rrd 已写完并关闭」之前，界面会拦下这一击。

子进程输出都在后台线程读管道、经队列回主线程刷新，所以回放跑五分钟界面不冻结。

## 产物结构

```
<out>/<seq>/mav0/
  cam0/{sensor.yaml, data.csv, data/<t>.png}       infra1，8-bit 灰度
  cam1/{sensor.yaml, data.csv, data/<t>.png}       infra2，与 cam0 逐行共享同一个 <t>
  imu0/{sensor.yaml, data.csv}                     7 列：ts, wx, wy, wz, ax, ay, az
  depth0/{data.csv, data/<t>.png}                  16-bit 毫米；unav_vio 不读取，仅留档
  state_groundtruth_estimate0/data.csv             设备端 HL-SLAM 位姿流，17 列，伪 GT
<out>/<seq>/record_summary.yaml                    溯源信息，不被解析
```

时间戳是**设备单调时钟纳秒**（非负、严格递增）。`unav_vio` 的 `TimePoint` 只做差值，
所以既不是 0 基也不需要 Unix epoch。

## 已知边界

- 只做录制，不做相机驱动、不做估计。本工具不改 `unav_vio`；上游判据要放宽还是收紧，是你在
  那个仓库里单独决定的事（下面 `REQUIRE_GT` 那条就是这么来的）。
- 每条格式约束对应的 `unav_vio` 判定点、room_02 的逐项诊断，以及「哪些东西根本无法用数据集
  文件表达」写在 [`docs/format-contract.md`](docs/format-contract.md)。改动上游解析器时按它复核。
- 伪 GT 来自设备内部 HLSL SLAM，是**另一个算法的输出**：指标只能当相对偏差看，
  不构成绝对精度声明。`--no-pose-gt` 可以完全不录 GT。上游 `integration.mh01_replay` 原先在
  缺 GT 时直接早退，连估计链都跑不到；现在它提供
  `UNAV_VIO_MH01_REQUIRE_GT=0`（默认仍是必须有 GT，只有精确写 `0` 才降级，结论行会印
  `require_gt=`），只放弃轨迹指标那一层——用法见上面「回放」小节。
- **IMU 必须逐帧取，不能按 frameset 取**：`pipeline::wait_for_frames()` 返回的 frameset
  每个流最多带一帧，用它取 IMU 会把 400/250 Hz 抽稀到相机帧率（30 Hz），栅格间隔立刻超
  上游的 10 ms 上限，而其余检查全都照过。`capture.cpp` 因此走带回调的 `start()`。
  `probe` 会按实测秒数报告两路 IMU 的真实采样率，任一低于请求值的 80 % 就打 `[FAIL]`
  并以退出码 1 结束——换 SDK 构建、换后端、换内核驱动之后先跑一次它。
- **位姿流不是想要就有的**：本机 `ros-jazzy-librealsense2` 2.58.4（RSUSB 后端）枚举到的六种流
  只有 `Infrared_1/2、Depth、Color、Accel、Gyro`，**没有 Pose**，所以 `--pose-gt auto` 会静默降级成
  不写 GT、`probe` 报「位姿流不可用」。要伪 GT 得先换一个会暴露该流的构建/固件，别指望改录制端。
- 加表逐轴标度不准：本机实测同一台设备镜头朝上时 z 轴读 10.10、水平姿态时 y 轴读 9.46
  （±3 % 上下）。两条都在 `verify` 的 0.5 m/s² 容差内，但静止初始化的零速假设会吃掉这部分余量；
  这是器件误差，不是录制端能修的，只在这里留个记录。
- D400 的红外是全局快门、电子快门；`--exposure-us` 用来压运动模糊。
- `calibration.version` 在 unav_vio 回放侧仍是硬编码常量（1）。`camera_time_offset_to_imu`
  原先也硬编码 0 ns，较新的回放改为从 `cam0/sensor.yaml` 的 `timeshift_to_imu_ms` 读取
  （缺键按 0）；录制端依旧不写这个键——D435i 的 IMU 与图像同用一个设备时钟域，共享曝光
  时间戳下 0 是自洽的，而实测非零 td 会让全程回放在中段停机（见「回放」一节的相机时间
  偏移），`rh verify` 会把手写该键的录制产物拦下。
- 单测不需要设备即可跑：合成数据走的是与真实录制完全相同的 writer/verify 代码路径。
  设备侧只有 `capture.cpp` 一个文件依赖 librealsense。

## 依赖与许可证

| 依赖 | 用途 | 许可证 |
| --- | --- | --- |
| librealsense 2.58.x | 设备访问、内参/外参、IMU 与位姿流 | Apache-2.0 |
| Eigen 3.4 | 旋转/四元数与 T_BS 组装 | MPL-2.0 |
| zlib | 自写的极简 PNG 编码（8-bit 灰度 + 16-bit 深度） | zlib |

本仓库不使用、也不复制 `unav_vio` 或 VINS 的任何源码；`verify`/`motion_gate` 里那些阈值和
解析规则是**按行为重新实现**的，判据出处逐条记在 `docs/format-contract.md`。

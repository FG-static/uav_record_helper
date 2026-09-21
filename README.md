# RecordHelper

给 Intel RealSense D435i 用的一键录制工具：把双目红外 + IMU（+ 深度、设备端位姿）录成
`unav_vio` 回放侧**可以直接吃**的 EuRoC 风格数据集，录完立刻自检，不用再做二次转换。

纯命令行，无 GUI，无 ROS，无 OpenCV，无 Ceres。依赖只有 `librealsense`、`Eigen`、`zlib`。

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
# 按 Intel 文档装 librealsense2（含 udev）和 Eigen
sudo apt install cmake g++ libeigen3-dev zlib1g-dev
# 若没走 Intel apt，至少让用户态能打开 D435i / hidraw：
sudo cp pack/99-realsense.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
# 拔插一次相机后再：
cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release
cmake --build build-release --parallel
ctest --test-dir build-release --output-on-failure
```

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
./build-release/rh record --seq room_04 --still 5 --excite 4 --exposure-us 4000 --no-pose-gt
./build-release/rh record --seq room_05 --imu-grid accel   # 两列都要实测值，陀螺抽稀到 250 Hz
```

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

```bash
env UNAV_VIO_EUROC_MH01=~/datasets/room_03 \
    UNAV_VIO_MH01_MAX_FRAMES=300 \
    ctest --test-dir <unav_vio build> -R mh01_replay --output-on-failure
```

数据集根目录直接就是 `UNAV_VIO_EUROC_MH01` 该指的地方，不需要任何中转脚本。

设备没有位姿流（`--no-pose-gt`，或本机这种根本不枚举 `RS2_STREAM_POSE` 的构建）时，包里就没有
`state_groundtruth_estimate0/`。加 `UNAV_VIO_MH01_REQUIRE_GT=0` 可以让上游回放测试只放弃「轨迹精度」
那一层判据（ATE/RPE），其余——解析、标定、双目硬同步、初始化及时性、状态发布完整性、求解器统计——
照常要求；默认不设该变量时仍然必须有 GT，结论行会印 `require_gt=0/1` 以便事后区分：

```bash
env UNAV_VIO_EUROC_MH01=~/datasets/room_03 UNAV_VIO_MH01_MAX_FRAMES=300 \
    UNAV_VIO_MH01_REQUIRE_GT=0 \
    ctest --test-dir <unav_vio build> -R mh01_replay --output-on-failure
```

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
- `camera_time_offset_to_imu` 与 `calibration.version` 在 unav_vio 回放侧是硬编码的
  （0 ns / 1），无法通过数据集文件表达；D435i 的 IMU 与图像同用一个设备时钟域，
  所以录出来的共享曝光时间戳是自洽的，但**不要**拿这份数据去测非零时移的行为。
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

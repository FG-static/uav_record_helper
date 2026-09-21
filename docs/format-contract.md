# 回放契约与 room_02 诊断

这份文档只回答一个问题：**RecordHelper 写出的每个字段，为什么长成这样。**
每条约束都指向 `unav_vio` 仓库里的具体判定点；上游一改，这里就要重新核对
（`tests/test_verify_room02.cpp` 会把「既有录制又变能放了」这种情况报出来）。

## 权威判定点（unav_vio @ main）

| 判定 | 位置 | 对录制的约束 |
| --- | --- | --- |
| YAML 只能按 flow 列表解析 | `tests/integration/euroc_dataset.cpp` 的 `read_yaml_block`（`key: [a, b]` 或 `cols/rows/data:`） | `sensor.yaml` 绝不能写 `- 640` 这种 block sequence |
| 数字必须被 `from_chars` 完整消费，拒绝 NaN/Inf | 同文件 `parse_number` / `parse_number_list` | 数值用最短可往返的十进制写法，时间戳用纯整数纳秒 |
| IMU CSV 恰好 7 列、时间戳非负且严格递增 | `euroc_dataset.cpp:413-431`、`parse_csv_time` | `ts,wx,wy,wz,ax,ay,az`；任何一行坏 → 整文件失败 |
| 相机 CSV 恰好 2 列 | `euroc_dataset.cpp:451-466` | `ts,filename`，文件名原样拼接进 `cam<N>/data/` |
| 左右只在 ns 完全相等时配对 | `euroc_dataset.cpp:473`（`synchronize_stereo_stamps`） | 一对双目共享同一个时间戳 |
| 回放断言 `dropped_count == 0` | `tests/integration/vio_mh01_replay_test.cpp:509` | 左右 CSV 的时间戳集合必须全等，不是「大致对齐」 |
| IMU 行数 > 30000 | `vio_mh01_replay_test.cpp:499` | 400 Hz 下至少要录约 76 s |
| IMU 间隔 ≤ 10 ms | `vio_mh01_replay_test.cpp:71`（覆盖 `AssemblerConfig::max_imu_gap`） | 陀螺掉拍即整段不可用；只裁窗，不造样本 |
| 帧间隔 ≤ 150 ms | `vio_mh01_replay_test.cpp:72` | 写盘背压只能整对丢，且要避开长空洞 |
| 双目同步误差 ≤ 100 µs | `include/unav_vio/initialization/initialization.hpp:81` | D435i 两目由同一 ASIC 触发，实测远小于该值 |
| `camera0` 是权威曝光时刻 | `src/runtime/vio_assembly.cpp:188`、`src/initialization/initialization.cpp:509` | 右目时间戳只用于统计偏斜，不取平均 |
| 标定合法性 | `src/data/data_contracts.cpp:151`（`validate_calibration`） | 正交性 1e-6、det=+1、`baseline>1e-9`、`cx<width`、四个噪声密度严格 > 0 |
| `T_BS` 按行主序 4×4 取用 | `euroc_dataset.cpp:353-358` | 旋转在 0,1,2/4,5,6/8,9,10，平移在 3,7,11 |
| radtan 只有 `k1,k2,p1,p2` | `src/geometry/geometry.cpp:92-102` | k3 非 0 就无法表达，必须显式处理而不是悄悄丢掉 |
| 静止→运动边界（取最后一个） | `euroc_dataset.cpp:592+`（`find_static_motion_boundary`） | 开头必须真静止。回放仍取最后一个边界；录制阶段不因此改状态，写盘也不裁到该边界 |
| anchor = 首个 ≥ `feed_start+60ms` 的帧，IMU 从 anchor 前 55 ms 起喂 | `vio_mh01_replay_test.cpp:522-530` | 帧密度要够（≥ ~18 Hz），IMU 必须在 anchor 之前就开始 |
| 每个曝光区间右端要被 IMU 覆盖 | `vio_mh01_replay_test.cpp:598` + `initialization.cpp:236` | 停止时 IMU 要比最后一帧再跑一会儿（`--tail`） |
| `g` 固定 9.81、`camera_time_offset_to_imu` 固定 0、`version` 固定 1 | `euroc_dataset.cpp:387-389`、`vio_mh01_replay_test.cpp:493` | 这三项**不能**由数据集文件表达，见下面的「无法表达的部分」 |

## RecordHelper 的对应做法

1. **格式编解码是上游解析器的对偶实现。** `src/record_helper/euroc_format.cpp` 逐条复刻
   `strip comment → trim → from_chars 必须整段消费 → flow 列表 → cols/rows/data 块`，
   所以 `rh verify` 通过 = 上游能解析，不需要靠肉眼比对 YAML。
2. **IMU 统一到陀螺原始时间戳栅格。** 加表（250 Hz）线性内插到陀螺（400 Hz）时刻；
   端点之外一律丢弃，不做零阶保持；先排序再去重时间戳，保证严格递增。
   `record_summary.yaml` 里的 `accel_hold_ratio` 就是这项健康度指标（room_02 = 0.383）。
3. **只裁窗，不造数。** `SelectReplayWindow` 在每个连续 IMU 段内挑最长的、
   同时满足「IMU 前置 ≥55 ms」「最后一个曝光右端 + `--tail` 有覆盖」
   「帧间隔 ≤150 ms」且「内部存在静止→运动边界」的双目区间。窗口从 IMU 覆盖到的第一帧起，
   **不**裁到最后一个静止边界（正式录制中途停下 1.5 s 再走是正常操作）。
   落在区间外的图像文件直接删除，不留孤儿引用。
4. **标定来自设备。** 内参取 `rs2_get_stream_profiles` 报告的 infra1/infra2（对应交付给主机的
   像素），外参取 `camera_infra* → accel` 的 `get_extrinsics_to`，再按 `R_BC/p_BC` 约定写成
   `T_BS`；设备给的是 float 精度，写出前做极分解正交化以稳定通过 1e-6 判据。
   加表系即 body 系；陀螺与加表的相对关系在 `record_summary.yaml` 里留档，
   因为回放契约只有一个 B 系。
5. **噪声参数实测。** 静止段用两延迟 Allan 分量估计：τ=dt 的一阶差分给白噪声密度
   （`N = sqrt(mean(Δ²)·dt)`，对常值 bias 天然免疫），τ=1~2 s 的块均值方差反解随机游走
   （`M = sqrt(3(Var - N²/2τ)/τ)`）。静止段太短则退回保守默认并在 manifest 与 `verify` 里
   标注 `fallback`，绝不假装是实测值。
6. **伪 GT 可选。** D435i 的设备端 HL-SLAM 位姿流（`RS2_STREAM_POSE`）经 `camera0→body`
   外参换算后写成 EuRoC 17 列。它是**另一个算法的输出**，只能用来给 ATE/RPE 一个相对参照，
   不是精度声明；`record_summary.yaml` 与 `verify` 输出都会这样标注。
7. **外参的系口径实测，不抄文档。** 静止标定阶段设备平放、镜头朝上，世界上即光学系 +z，
   因此静止段平均比力在 body 系里必须沿 `T_BS` 的第三列；夹角超过 25° 当场中止。
   上游没有任何一条判据能替代它：`validate_calibration` 只查正交性/det/基线/噪声正数，
   单位阵和任何置换都照样通过，`SummarizeStereoGeometry` 看的是两目**相对**关系、两目同错即抵消。
   换 librealsense 构建、换后端、换内核驱动都可能换掉加表数据的系（本机内核 IIO 原始系与
   SDK 交付系之间就差一个 `diag(-1,+1,-1)`），所以这条只能每次录的时候当场量。

## room_02 的逐项结论

`rh verify /Users/mac/datasets/room_02` 现在会给出（2026-09-20 复核）：

```
[FAIL] camera.sensor_yaml      cam0 resolution: missing data or mismatched rows/cols（block sequence）
[FAIL] imu.noise_density       四个噪声键全部 key not found
[PASS] imu.gap                 最大相邻间隔 7.489 ms（≤10 ms，这条其实是过的）
[WARN] imu.zero_order_hold     相邻行加表完全相同的比例 38.3 %
[FAIL] stereo.hard_sync        可配对 4096 对，dropped_count=41
[PASS] camera.frame_gap        最大帧间隔 100.03 ms
[FAIL] ground_truth            缺少 state_groundtruth_estimate0/data.csv
[FAIL] initialization.static_window  最静窗 accel_rms=0.505 / gyro_rms=0.168（阈值 0.1/0.1）
结论: 不满足回放契约（fail=8 warn=2）
```

注意 `camera.sensor_yaml` 在第一个键 `resolution` 就失败，所以后面的标定项（含「`T_BS` 旋转是
单位阵」这一条）根本读不进来 —— 「外参错了」不是它的首要故障，**首要故障是格式压根没被上游解析器
接受**，其次是没有静止段。至于单位阵本身算不算错，取决于 room_02 那条 ROS 重分发路径把加表数据交在
哪个系里，而这份数据连解析都过不了、没法实测核对，所以本文档不把它记成已定案的 FAIL。对照实验：
本机 SDK 直读路径下，静止平放镜头朝上时平均比力与 `T_BS` 第三列同向，单位阵成立。

想让 room_02 变得能用，只能重录：41 个不重叠的时间戳、被零阶保持污染过的 IMU、
以及不存在的开头静止窗，都不是事后转换能补回来的（补出来的是假数据）。

## 无法用数据集文件表达的三件事

| 项 | 上游现状 | 影响 |
| --- | --- | --- |
| `camera_time_offset_to_imu` | `euroc_dataset.cpp:389` 硬编码 0 ns | D435i 的 IMU 与图像共用同一设备时钟域，且我们写的是共享曝光时间戳，因此 offset=0 是自洽的；但这意味着**不要**用这份数据去验证非零时移路径 |
| `imu_noise.g` | `euroc_dataset.cpp:387` 硬编码 9.81 | 录制端只能要求静止段 `‖a‖` 与 9.81 相符（`verify` 的 `initialization.gravity`，容差 0.5 m/s² 取自 `InitializerConfig::max_static_accel_norm_error_mps2`） |
| `calibration.version` | `vio_mh01_replay_test.cpp:493` 传入常量 1 | 无影响，录制端不写这个字段 |

## 「仅回放、不评估」的那道开关

`--no-pose-gt` 录出来的数据集刻意没有 `state_groundtruth_estimate0/`。上游
`tests/integration/vio_mh01_replay_test.cpp` 原先在 `expect(gt.ok)` 失败后立刻 `return`，
连静止→运动边界都测不到。现在（2026-09-21，经用户同意后改的是 `unav_vio` 一侧）该判据可由
`UNAV_VIO_MH01_REQUIRE_GT=0` 显式降级：只放弃「轨迹指标可用 + ATE/RPE 健康界」这一层，
解析、标定、硬同步、初始化及时性、状态发布完整性、求解器统计照常要求。

- **默认值不变**：不设该变量时仍然必须有 GT，MH_01_easy 的回归判据一字未改。
- 只有精确字符串 `0` 才降级，变量名或值写错都不会悄悄放宽。
- 结论行会印 `require_gt=0/1`，所以「降级跑过」在事后能被区分出来。

RecordHelper 自己不碰 `unav_vio` 的判据；`rh verify --no-gt-required` 只在自己的报告里把该项
降为 WARN，用来先确认「除了 GT 之外全部满足契约」。

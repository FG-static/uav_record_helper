# RK3588 CSI 双目 + MAVLink IMU 录制

后端 `csi-mavlink` 面向现有 RK3588 双路 OV8858 和飞控 HIGHRES_IMU，采集研究/回放数据。它通过 Linux V4L2/GStreamer appsink 获取原始灰度像素，通过 pyserial/pymavlink 读取 IMU。原 RealSense 原生后端保持默认；CSI 后端无需安装 librealsense、ROS、OpenCV 或 NumPy。启动前检查当前用户可见的设备占用，遇到其他录制进程拒绝继续；串口同时使用独占锁，勿与不遵守锁的程序同时读取。采集不会发送解锁、模式、参数或设定点，发送 GCS HEARTBEAT、TIMESYNC，以及只针对 HIGHRES_IMU 的 MAV_CMD_SET_MESSAGE_INTERVAL（511）请求。

## 构建与环境

```bash
# Debian：GStreamer v4l2src/videoconvert/appsink 和对应 GI typelib 必须存在。
sudo apt-get install python3-gi gir1.2-gstreamer-1.0 gir1.2-gst-plugins-base-1.0 \
  gstreamer1.0-plugins-base gstreamer1.0-plugins-good python3-yaml python3-venv
python3 -m venv --system-site-packages .venv-csi
.venv-csi/bin/pip install -r requirements-csi.txt
cmake -S . -B build-csi -DRH_WITH_REALSENSE=OFF -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE="$PWD/.venv-csi/bin/python"
cmake --build build-csi --parallel 3
ctest --test-dir build-csi --output-on-failure
export RH_CSI_PYTHON="$PWD/.venv-csi/bin/python"
```

`rh` 原生入口通过 argv 直接执行仓库内 Python helper，不经过 shell。`RH_CSI_PYTHON` 选择有 GI、yaml、pyserial、pymavlink 的解释器。也可直接运行 `python tools/rh_csi_mavlink.py ...`。Python 3.9+。不支持非 Linux 的 CSI 采集；纯时序、写盘和导出测试可跨平台运行。发行构建必须保留源码目录中的 tools/，当前项目没有安装打包规则。

本轮实际板子提供的独立解释器是 `/home/cat/.local/state/record-helper/venv/bin/python`（system site packages 提供 GI/YAML，复用本机已验证版本的协议依赖）。现有 `/home/cat/projects/record_helper/config/rk3588-csi-mavlink.json` 是当前设备配置模板。设备节点可能随设备树/枚举顺序变化，启动前核对 `v4l2-ctl --list-devices` 和 `/dev/serial/by-id/`，不要按编号猜左右。

## 使用

```bash
# 无落盘探测：先发心跳，协商软件时钟偏移，然后打开两路相机。
RH_CSI_PYTHON=.venv-csi/bin/python build-csi/rh probe --backend csi-mavlink \
  --config config/rk3588-csi-mavlink.json --duration 5

# 录制：duration 从相机启动开始计时，另有启动同步及250ms IMU尾部。
RH_CSI_PYTHON=.venv-csi/bin/python build-csi/rh record --backend csi-mavlink \
  --config config/rk3588-csi-mavlink.json --out datasets --seq desk01 --duration 30
```

CSI 的 `--duration` 是实际自动停止时长；0 表示持续至 Ctrl-C/SIGTERM。它与原 D435i 引导录制的建议时长语义不同。Ctrl-C/SIGTERM 进入正常收尾；SIGKILL/断电则 `complete=false`。目标序列目录必须不存在，包含失败的旧序列也拒绝覆盖。此后端不强制摆放镜头朝上、不要求静止→激励后才写盘；请自行先静置几秒，再做温和、充分的平移/旋转，供后续 VIO 初始化。短时冒烟样本不代表满足长回放所需样本量/运动条件。

相机配置显式指定两个节点、输入格式、宽高、帧率和 `readout_ns`；当前模板中首末行扫描差 32749000ns 是此前依据当前寄存器/帧周期估算的值，不是新做的光学标定。改变 sensor mode 后要重新核对；未知可填 null。此值只留档，不做去卷帘补偿。`pair_tolerance_ms` 是软件配对阈值，不是曝光同步保证。

MAVLink 启动先发心跳，随后发送 COMMAND_LONG(command=511)，param1=105（HIGHRES_IMU），param2=5000微秒（默认请求200Hz），其他参数为0。配置 mavlink.imu_rate_hz 可改请求频率，旧配置缺此键时默认200；ACK未到每秒重试、最多3次，拒绝或最终超时会停止并留失败状态。等待选定飞控的接受ACK和有效软件时间映射后才开始相机采集。ACK仅确认命令接受，不证明频率已达到；manifest同时保留请求值、ACK、实测IMU频率/最大间隔，低于请求80%会给出明确提示。默认USB CDC链路的波特率字段不等同于实际带宽；其他UART链路须核对其吞吐。录制结束不另发511恢复默认频率，也不修改持久参数。

MAVLink 仅选择显式 system_id/component_id/sensor_id；HIGHRES_IMU 六轴更新位完整且有限才进入规范 CSV。配置 `body_frame=frd` 时，y/z 各翻转一次导出 FLU。角速度 rad/s、加速度比力 m/s²；不去重力，不从姿态推 IMU，不用 packet sequence 当传感器序号。实际轴定义必须在实物上核对，校准数据的 body/IMU 原点与轴必须匹配导出后的 FLU。

## 目录与真实时序

```
<out>/<seq>/
  capture_manifest.json          完整配置、时钟估计、统计、完成/失败、适用限制
  record_summary.yaml            后端/卷帘契约标志，rh verify 能识别
  raw/mavlink-rx.bin              未重编码的接收字节，包含非IMU消息
  raw/mavlink-tx.jsonl            HEARTBEAT/TIMESYNC/511 的发送记录
  raw/command-acks.jsonl          511请求的飞控ACK（含拒绝/进行中）
  raw/imu.csv                    飞控原时刻、主机接收时刻、来源、原轴值、接纳情况
  raw/cameras.csv                每路 buffer/接收时刻、独立序号、driver offset、PTS
  raw/pairs.csv                  已写出帧对的两路原始时刻、偏差、共享CSV索引
  raw/timesync.jsonl              匹配响应与往返时延、候选偏移
  mav0/cam{0,1}/data/<ns>.png     GRAY8，无损，保留原始尺寸/畸变，不是视频抽帧
  mav0/cam{0,1}/data.csv          整数相对ns,文件名；共享cam0索引以兼容EuRoC读取器
  mav0/imu0/data.csv              整数相对ns,wx,wy,wz,ax,ay,az；原测量栅格，不插值补频
```

两个 pipeline 使用同一个 GstSystemClock。由 segment running time + pipeline base time 转为共同主机时钟；buffer timestamp 的 SOE/EOF/曝光参考未认证，不能当“全局曝光中点”。采集保留原始 PTS，图像 stride/padding 按 VideoInfo/VideoMeta 逐行拷贝。

飞控 boot clock 映射使用发起方匹配的 TIMESYNC 响应：偏移候选 `(send_host+receive_host)/2 - remote`。只接受选中飞控、匹配本次 ts1、RTT≤10ms 的响应，至少8次后取低延迟子集的中位数。随后按每秒最低RTT观测估计时钟速率比（限制±2000ppm），用连续仿射映射跟踪速率差，每次响应相位修正不超过50微秒；模型残差超过2ms停止。每个更新的基准和速率比都随原始交换一起记录，可离线重拟合。没有可用软件映射时15秒报错，不拿到达时刻伪造测量时刻。飞控时间回退立即停止，重复样本拒绝。软件映射仍含链路不对称、采样与遥测延迟等不确定性，不能代替相机/IMU硬同步。原始时间都保留，后期可重估。

左右按单调时刻在容差内做一对一软件配对，保留各自真实时刻，不取平均。PNG写盘队列有界，满了整对丢；最多6对并行编码/写盘，CSV仍按采集顺序提交，两张PNG均写成功才登记CSV。输入队列8对加在途6对，内存不会随录制时长增长。每路图像或IMU超过3秒无输入停止。结束时检查IMU覆盖图像区间及55ms前置；失败数据保留但complete=false。计数同时记录配对丢弃、队列丢弃、可用driver序号空洞和实际IMU最大间隔。

## 标定与 EuRoC 回放

相机不能自动读出这套实际安装的内外参，因此录制时默认不生成 sensor.yaml，也不生成 GT。先保存真实数据，获得标定后：

```bash
# calibration/{cam0,cam1,imu0}/sensor.yaml 必须来自该安装状态的实际标定。
build-csi/rh export-csi datasets/desk01 --calibration calibration
# 或录制时 --calibration calibration，在正常收尾后导入。
build-csi/rh verify datasets/desk01 --no-gt-required
```

标定采用 EuRoC pinhole + radial-tangential 四参数；两相机独立分辨率/内参/畸变、4×4行主序T_BS，非零基线；IMU四个噪声密度/随机游走均为正。T_BS 把相机光学系变换到规范CSV的 FLU body/IMU系。实际测得的 `timeshift_to_imu_ms` 可以随标定提供，不人为填0冒充已同步。导入前校验全部文件，拒绝覆盖既有 sensor.yaml。清单留存标定来源与哈希。

数据CSV/PNG与现有 unav_vio EuRoC loader 兼容；补齐真实标定后可以用于研究回放。但当前 unav_vio 仍按全局快门/硬同步模型工作，本后端的 `replay_ready` 始终false，`rh verify` 的 `capture.physical_contract` 明确FAIL；这与“文件格式可读”是两层结论。49Hz IMU 的约20ms间隔也超过当前回放默认10ms门限。不能为过门而补出200Hz假样本，或只复制相同左右时间戳就声称硬同步。后续回放须显式评估卷帘、非同步和采样率限制，校验标定/运动/缺GT选项，不把离线能运行当作估计精度或实飞认证。

参考接口：[MAVLink TIMESYNC](https://mavlink.io/en/services/timesync.html)、[PX4 1.14.3应答实现](https://github.com/PX4/PX4-Autopilot/blob/v1.14.3/src/modules/mavlink/mavlink_timesync.cpp)、[GStreamer appsink](https://gstreamer.freedesktop.org/documentation/app/appsink.html)、[GstClock](https://gstreamer.freedesktop.org/documentation/gstreamer/gstclock.html)。后端没有复制这些项目的实现代码；协议由安装的pymavlink处理。


## 2026-11-05 板上离线写盘压测

使用已有标定录制中的真实1632×1224灰度图，不新开相机/串口。初版单线程在30对/秒送入90对时仅写出34对，故改为最多6对并行编码、单序提交CSV；并发顺序和写盘失败收尾都有回归测试。

更新后按30对/秒送入300对，300对全部写出、队列丢弃0；600张PNG解码像素与源PGM逐字相同。送入耗时9.968秒，关闭队列耗时合计10.093秒，再逐文件fsync耗时4.926秒。10秒缓存内吞吐通过，不能据此宣称当前根分区能长时间持续30fps；720MB级输出的存储刷盘余量需要单独验收，选择更快输出盘或降低保存数据率后再测。压测保留原始图片时间戳，只加速送入编码器，不是新录制的10秒传感器数据，也不是VIO精度测试。


## ace_unav 标定录制入口

`ace_unav/tools/stereo_imu_calib_record.py` 复用本采集核心：默认每目20fps独立时间戳保存，避免自由运行相机的相位使严格配对丢光数据。该模式不是上面面向现有VIO loader的共享时间键配对模式，不能不经检查直接当作同步EuRoC输入；manifest的independent_camera_timestamps明确区分两者。

两种模式均先写PNG临时文件并fsync后rename；CSV每10对等价帧数checkpoint并fsync，随后原子更新durable-pairs.json。IMU/raw日志和complete=false的manifest后台周期同步；正常退出等待队列和同步完成，再原子写complete=true。突发断电无法保证最后未checkpoint的数据，重启后不完整数据不得直接通过标定验收。

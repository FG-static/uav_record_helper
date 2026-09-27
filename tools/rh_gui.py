#!/usr/bin/env python3
"""RecordHelper 的驱动界面：探测 → 录制 → 校验 → 回放（.rrd 全量 / JSONL 在线）。

它不链接任何图形库，也不给 C++ 库引入新依赖：界面里每个动作都是在调用已经构建好的命令行
程序，参数与环境变量和 README 里手敲的那几条完全一致。录制本身仍然开在**真实终端窗口**里
——阶段靠键盘推进，而且需要人拿着相机走动，把键盘塞进管道会丢掉这个前提。

用法：
    python3 tools/rh_gui.py                 # 打开界面
    python3 tools/rh_gui.py --selftest      # 只跑无界面的纯函数检查（不需要 DISPLAY）
    python3 tools/rh_gui.py --smoke         # 构造界面后自动关闭，验证布局代码不抛异常
"""

from __future__ import annotations

import argparse
import os
import queue
import shlex
import shutil
import signal
import subprocess
import sys
import threading
from pathlib import Path

# 实测：94.3 s / 2828 对、无深度 = 394 MB → 约 4.2 MB/s。带深度那一档只是按 Z16 848x480@30
# 估的量级，界面里必须标成估算，别当测量值用。
BYTES_PER_SECOND_NO_DEPTH = 4.2e6
DEPTH_SIZE_FACTOR = 2.5

# unav_vio 的回放判据：IMU 行数要 > 30000（vio_mh01_replay_test.cpp:499）。
REQUIRED_IMU_ROWS = 30001
GYRO_HZ = 400
ACCEL_HZ = 250


# --------------------------------------------------------------------------- 纯函数


def repo_root(start: Path) -> Path:
    """向上找到 record_helper 仓库根（有 CMakeLists.txt 且有 src/record_helper）。"""
    for candidate in (start, *start.parents):
        if ((candidate / "CMakeLists.txt").is_file()
                and (candidate / "src" / "record_helper").is_dir()):
            return candidate
    return start.parent if start.name == "tools" else start


def find_rh(root: Path) -> Path | None:
    """README 承诺的产物就是 build-release/rh；build/ 只是给开发时留的余地。"""
    for candidate in (root / "build-release" / "rh", root / "build" / "rh"):
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return candidate
    return None


def _executable(path: Path) -> Path | None:
    return path if path.is_file() and os.access(path, os.X_OK) else None


def find_replay_binary(build_dir: Path) -> Path | None:
    """新 unav_vio 把可执行文件改名为 vio_mh01_replay；旧构建目录仍是 *_test。"""
    for name in ("vio_mh01_replay", "vio_mh01_replay_test"):
        if found := _executable(build_dir / name):
            return found
    return None


def find_playback_binary(build_dir: Path) -> Path | None:
    return _executable(build_dir / "vio_observation_playback")


def default_unav_build() -> Path:
    """优先选带 JSONL 播放器的构建目录，没有再退回 GUI 原先默认的 ace_unav 路径。"""
    home = Path.home()
    candidates = (
        home / "unav_vio" / "build-rerun060-release",
        home / "unav_vio" / "build-replay",
        home / "ace_unav" / "modules" / "unav_vio" / "build-replay",
    )
    for path in candidates:
        if find_replay_binary(path) is not None:
            return path
    return candidates[-1]


def rerun_viewer() -> Path | None:
    """Rerun 查看器不随本项目安装，也常常不在 PATH 里：先查 PATH，再查 ~/桌面 的 rerun-cli-*。"""
    if found := shutil.which("rerun"):
        return Path(found)
    home = Path.home()
    for base in (home / "桌面", home / "Desktop", home / "下载"):
        for match in sorted(base.glob("rerun-cli-*")) if base.is_dir() else ():
            if match.is_file() and os.access(match, os.X_OK):
                return match
    return None


def build_record_command(rh: Path, cfg: dict) -> list[str]:
    argv = [str(rh), "record"]
    seq = str(cfg.get("seq", "")).strip()
    if seq:
        argv += ["--seq", seq]  # 留空就不传，让 rh 用它自己的时间戳默认名
    argv += ["--out", str(cfg["out"])]
    argv += ["--still", str(cfg["still"]), "--excite", str(cfg["excite"])]
    argv += ["--exposure-us", str(cfg["exposure_us"]), "--tail", str(cfg["tail"])]
    argv += ["--duration", str(cfg["duration"]), "--imu-grid", str(cfg["imu_grid"])]
    for key, flag in (("no_depth", "--no-depth"), ("no_pose_gt", "--no-pose-gt"),
                      ("zero_distortion", "--zero-distortion"), ("overwrite", "--force")):
        if cfg.get(key):
            argv.append(flag)
    return argv


def build_verify_command(rh: Path, dataset: Path, no_gt_required: bool, deep: bool) -> list[str]:
    argv = [str(rh), "verify", str(dataset)]
    if no_gt_required:
        argv.append("--no-gt-required")
    if deep:
        argv.append("--deep")
    return argv


# 实测（d435i_20260924_164630，MAX_FRAMES=300 → .rrd 167,711,289 B，image_records=600）：画面是
# 叠加特征后渲染成 RGB8 的图，不是原始 PNG，所以每帧对约 0.56 MB；整段 5400 帧就是 3 GB 量级。
# 没画标记的旧二进制是 0.31 MB/帧对，别拿那个数当预期。
RRD_IMAGE_BYTES_PER_FRAME = 5.6e5


def read_timeshift_ms(dataset: Path) -> float | None:
    """读 mav0/cam0/sensor.yaml 的 timeshift_to_imu_ms；缺键、读不了或值坏都返回 None。

    录制端从不写这个键（D435i 图像与 IMU 同一时钟域，共享曝光时间戳下 td=0 自洽），
    所以非 None 就意味着数据集被事后改过（例如 estimate_td.py 实测后回写）。
    实测（2026-09-26，同一二进制 A/B）非零 td 会让全程回放在中段停机：轨迹/landmark/
    特征标记从那一刻全部消失而画面照常；td 框填 0（或删掉那行键）即恢复整段回放。
    """
    path = dataset / "mav0" / "cam0" / "sensor.yaml"
    try:
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            stripped = line.split("#", 1)[0].strip()
            if stripped.startswith("timeshift_to_imu_ms:"):
                return float(stripped.split(":", 1)[1].strip())
    except (OSError, ValueError):
        return None
    return None


def default_bootstrap_frames(max_frames: int) -> int:
    """只加长「初始化还允许再等多久」，不改任何判据。

    上游默认 150 帧（EuRoC 20 Hz 约 7.5 s）。D435i 30 Hz 还要先攒 0.5 s 静止，
    `d435i_20260924_164630` 的 Gate 1 留档 first_state_frame=230，150 帧预算会
    `未及时初始化` / `states=0`，随后双目 ingress `status=5`（ComponentFailed）。
    整段（max_frames=0）给一个够用的上限；短窗至少放到 240（上游「及时」判据）。
    """
    if max_frames <= 0:
        return 8192
    return max(int(max_frames), 240)


def replay_env(dataset: Path, max_frames: int, require_gt: bool, rrd: Path | None,
               artifacts: Path | None, rerun_images: bool = False,
               image_levels_min: str = "", image_lap_min: str = "",
               td_ms: str = "", observation_archive: Path | None = None,
               frontend_contrast: bool = False,
               bootstrap_frames: int | None = None,
               allow_motion_start: bool | None = None,
               wait_ms: str = "") -> dict[str, str]:
    env = {
        "UNAV_VIO_EUROC_MH01": str(dataset),
        "UNAV_VIO_MH01_MAX_FRAMES": str(max_frames),
        # 结论行会印 require_gt=0/1：降级跑过必须能被事后区分，所以永远显式设。
        "UNAV_VIO_MH01_REQUIRE_GT": "1" if require_gt else "0",
        "UNAV_VIO_MH01_BOOTSTRAP_FRAMES": str(
            default_bootstrap_frames(max_frames) if bootstrap_frames is None
            else bootstrap_frames),
    }
    if rrd is not None:
        env["UNAV_VIO_RERUN_SAVE"] = str(rrd)
        # 上游把画面写在 live 请求或显式开关上，SAVE 模式默认不落图，所以这里必须显式打开。
        if rerun_images:
            env["UNAV_VIO_RERUN_IMAGES"] = "1"
    if artifacts is not None:
        env["UNAV_VIO_REPLAY_ARTIFACT_DIR"] = str(artifacts)
    # JSONL 观察留档：不复制 PNG，给 vio_observation_playback 按源时间在线播。上游拒绝覆盖。
    if observation_archive is not None:
        env["UNAV_VIO_OBSERVATION_ARCHIVE"] = str(observation_archive)
        # 观察失败单独进退出码，不把 VIO 结论改写成 pass。
        env["UNAV_VIO_REQUIRE_OBSERVATION_ARCHIVE"] = "1"
    if frontend_contrast:
        # Gate 1 同数据集成功留档开了对比度归一化；默认关会让暗光红外更难攒视差。
        env["UNAV_VIO_MH01_FRONTEND_CONTRAST"] = "1"
    if allow_motion_start is False:
        # 只准在静止段起步。允许运动段起步时，初始化会在"这一批不安静"的当前帧锁锚，
        # 没有静止窗可平均：run_01 曾在 imu_excitation=105.6 的猛晃里起步，首个发布态的
        # |b_a| 就到 1.52 m/s²，0.9 s 内速度冲到 7.7 m/s 撞 max_speed 健康门。
        env["UNAV_VIO_MH01_ALLOW_MOTION_START"] = "0"
    if wait_ms:
        # 停滞预算（ms，上游默认 120000）：只统计"已消费帧数不变"的时长，不是总运行时间。
        # JSONL 留档每行都 flush 且跑在 Component 唯一的后台任务里，盘慢会直接节流消费。
        env["UNAV_VIO_MH01_WAIT_MS"] = wait_ms
    # 留空 = 用上游默认；填了才覆盖。这几个都是回放侧旋钮，不是录制端契约。
    # 注意 "0" 是有效覆盖（把数据集里写的相机时间偏移清零），所以判据是"非空"而非"非零"。
    for key, value in (("UNAV_VIO_MH01_IMAGE_LEVELS_MIN", image_levels_min),
                       ("UNAV_VIO_MH01_IMAGE_LAP_MIN", image_lap_min),
                       ("UNAV_VIO_MH01_TD_MS", td_ms)):
        if value:
            env[key] = value
    return env


def build_report_command(root_dir: Path, artifacts: Path) -> list[str]:
    """出图走 tools/rh_report.py：它只读 artifacts/estimated.tum，不接受重复运行覆盖旧图。"""
    return [sys.executable, str(root_dir / "tools" / "rh_report.py"), str(artifacts)]


def build_playback_command(binary: Path, archive: Path, dataset: Path | None,
                           display_hz: str, rate: str = "1.0") -> list[str]:
    """拼 vio_observation_playback：读 JSONL + 原始 PNG，不重跑 VIO。

    display_hz 空或省略 = 每帧都发（CLI 默认 0）；>0 按墙钟抽 RGB，省 Viewer 内存。
    """
    argv = [str(binary), "--archive", str(archive), "--connect"]
    if dataset is not None:
        argv += ["--dataset", str(dataset)]
    hz = display_hz.strip()
    if hz:
        argv += ["--display-hz", hz]
    rate = rate.strip()
    if rate and rate != "1.0":
        argv += ["--rate", rate]
    return argv


def live_viewer_command(viewer: Path) -> list[str]:
    """JSONL 在线播用的 live Viewer：端点写死 rerun+http://127.0.0.1:9876/proxy。"""
    return [str(viewer), "--serve-web", "--bind", "127.0.0.1", "--port", "9876",
            "--memory-limit", "4GB"]


def kill_process_group(proc: subprocess.Popen) -> None:
    """先 SIGTERM 整组，3 s 后再 SIGKILL。live Viewer 会再拉 web 子进程。"""
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except (ProcessLookupError, PermissionError, OSError):
        try:
            proc.terminate()
        except OSError:
            pass
    try:
        proc.wait(timeout=3)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except (ProcessLookupError, PermissionError, OSError):
        try:
            proc.kill()
        except OSError:
            pass
    try:
        proc.wait(timeout=2)
    except (subprocess.TimeoutExpired, OSError):
        pass


def terminal_launcher(argv: list[str], cwd: Path, keep_open: bool = True) -> list[str] | None:
    """把一条命令包进可见终端。每个参数单独 shell 引用，不做字符串拼接式注入。"""
    script = " ".join(shlex.quote(part) for part in argv)
    if keep_open:
        script += '; echo; echo "[退出码 $?] 按回车关闭本窗口"; read -r _'
    launcher = ["bash", "-lc", script]
    for name, prefix in (("gnome-terminal", ["--working-directory", str(cwd), "--"]),
                         ("konsole", ["--cwd", str(cwd), "-e"]),
                         ("xterm", ["-e"]),
                         ("x-terminal-emulator", ["-e"])):
        if path := shutil.which(name):
            return [path, *prefix, *launcher]
    return None


def disk_free(path: Path) -> int | None:
    target = path
    while not target.exists() and target != target.parent:
        target = target.parent
    return shutil.disk_usage(target).free if target.exists() else None


def estimate_record_bytes(cfg: dict) -> float:
    rate = BYTES_PER_SECOND_NO_DEPTH * (1.0 if cfg.get("no_depth") else DEPTH_SIZE_FACTOR)
    # 静止、激励、正式录制和尾部覆盖都在出流，按总时长估。
    return rate * (float(cfg["duration"]) + float(cfg["still"]) + float(cfg["excite"]) + 2.0)


def grid_hz(cfg: dict) -> int:
    return GYRO_HZ if str(cfg.get("imu_grid", "gyro")) == "gyro" else ACCEL_HZ


def imu_rows(cfg: dict) -> int:
    return int(float(cfg["duration"]) * grid_hz(cfg))


def record_problems(cfg: dict, out_dir: Path) -> list[str]:
    """离线阶段能判出来的问题。全部只提示，不拦停——真正的门在 rh 自己手里。"""
    problems: list[str] = []
    seq = str(cfg.get("seq", "")).strip()
    if seq and (len(seq) > 64 or any(char in seq for char in "/ \t")):
        problems.append(f"序列名 {seq!r} 不合法：不能含空格或斜杠，长度 ≤ 64。留空则用时间戳默认名。")
    if float(cfg["still"]) < 1.5:
        problems.append("--still 至少 1.5 s（3 个 0.5 s 窗），否则 rh 直接拒绝启动。")
    if float(cfg["excite"]) < 2.5:
        problems.append("--excite 至少 2.5 s（5 个 0.5 s 窗），否则 rh 直接拒绝启动。")
    if float(cfg["tail"]) < 0.1:
        problems.append("--tail 至少 0.1 s。")
    rows = imu_rows(cfg)
    if rows < REQUIRED_IMU_ROWS:
        need = (REQUIRED_IMU_ROWS - rows) / grid_hz(cfg)
        problems.append(
            f"--duration {cfg['duration']} s 在 {cfg['imu_grid']} 栅格下约 {rows} 行 IMU，"
            f"过不了回放的 {REQUIRED_IMU_ROWS} 行下限；还要多录约 {need:.0f} s。")
    free = disk_free(out_dir)
    if free is not None:
        need = estimate_record_bytes(cfg)
        if free < need * 1.5:
            problems.append(
                f"目标盘剩余 {free / 1e9:.1f} GB，这条录制估算约 {need / 1e6:.0f} MB"
                "（含深度时只是量级估算）。rh 没有剩余空间检查，写到一半满盘整段报废——先腾地方。")
    return problems


def nearest_existing_dir(path: Path) -> Path:
    """文件选择框的 initialdir 必须真实存在，否则 tkinter 会悄悄退回上次的目录。

    界面里的默认值经常还没被创建（unav_vio 的 build-replay、产物目录），所以逐级向上找；
    绝对路径的链条最坏到根目录，一定能停下。
    """
    candidate = path if path.is_absolute() else Path.cwd() / path
    while not candidate.is_dir() and candidate != candidate.parent:
        candidate = candidate.parent
    return candidate


def dir_size(path: Path) -> int:
    total = 0
    for root, _, files in os.walk(path):
        for name in files:
            try:
                total += (Path(root) / name).stat().st_size
            except OSError:
                pass
    return total


def list_datasets(out_dir: Path) -> list[tuple[str, int, str]]:
    """列出 <out>/*/mav0 形态的包：(路径, 字节, 备注)。"""
    rows: list[tuple[str, int, str]] = []
    if not out_dir.is_dir():
        return rows
    for child in sorted(out_dir.iterdir()):
        mav0 = child / "mav0"
        if not mav0.is_dir():
            continue
        notes = []
        if not (mav0 / "state_groundtruth_estimate0" / "data.csv").is_file():
            notes.append("无GT")
        if not (mav0 / "depth0" / "data.csv").is_file():
            notes.append("无深度")
        rows.append((str(child), dir_size(child), ",".join(notes) or "完整"))
    return rows


# ------------------------------------------------------------------------------ UI


class App:
    def __init__(self, root_dir: Path) -> None:
        import tkinter as tk
        from tkinter import ttk

        self.tk, self.ttk = tk, ttk
        self.root_dir = root_dir
        self.rh = find_rh(root_dir)
        self.log_queue: queue.Queue[str] = queue.Queue()
        self.process: subprocess.Popen | None = None
        # 只有回放会往 .rrd 里写；探测/校验跑着的时候开 viewer 没有顾虑，所以单独记这一点。
        self.writing_rrd: Path | None = None
        self.writing_jsonl: Path | None = None
        self.reader_thread: threading.Thread | None = None
        self.live_viewer: subprocess.Popen | None = None

        self.root = tk.Tk()
        self.root.title("RecordHelper")
        self.root.geometry("1080x820")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._build_widgets()
        self.refresh_datasets()
        self.log(f"仓库根：{root_dir}")
        self.log(f"rh：{self.rh or '未构建 —— 先按 README 构建 build-release/rh'}")
        self.log("录制会开一个新的终端窗口，阶段推进（Enter / q / x）在那边按。")
        if os.environ.get("RH_GUI_SMOKE") == "1":
            self.root.after(1500, self.root.destroy)
        self._pump()

    # --- 日志与子进程 -----------------------------------------------------

    def log(self, text: str) -> None:
        self.log_text.configure(state="normal")
        self.log_text.insert("end", text.rstrip("\n") + "\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _pump(self) -> None:
        try:
            while True:
                self.log(self.log_queue.get_nowait())
        except queue.Empty:
            pass
        if self.process is not None and self.process.poll() is not None:
            code = self.process.returncode
            reader = self.reader_thread
            self.reader_thread = None
            if reader is not None:
                reader.join(timeout=1.0)
            try:
                while True:
                    self.log(self.log_queue.get_nowait())
            except queue.Empty:
                pass
            self.log(f"[退出码 {code}]")
            self.process = None
            if self.writing_rrd is not None:
                self.log(f".rrd 已写完并关闭：{self.writing_rrd}（现在可以打开 viewer 了）")
                self.writing_rrd = None
            if self.writing_jsonl is not None:
                jsonl = self.writing_jsonl
                log_dump = self.log_text.get("1.0", "end")
                archive_ok = "observation_archive=pass" in log_dump
                if archive_ok:
                    self.log(f"JSONL 已写完：{jsonl}（现在可以「JSONL 在线播放」）")
                    if code != 0:
                        self.log("[提示] 退出码非 0 只说明 VIO 验收没过（看 vio_result=），"
                                 "观察留档成功不等于估计通过。")
                else:
                    hint = ("回放结束但没看到 observation_archive=pass"
                            if code == 0 else
                            "回放非 0 退出且没看到 observation_archive=pass")
                    self.log(f"[提示] {hint}，检查 {jsonl} 是否完整"
                             "（上游拒绝覆盖，坏档要先删再生成）")
                self.writing_jsonl = None
            self._set_replay_busy(False)
        if self.live_viewer is not None and self.live_viewer.poll() is not None:
            self.log(f"live Viewer 已退出（码 {self.live_viewer.returncode}）")
            self.live_viewer = None
            self._set_live_viewer_running(False)
        self.root.after(120, self._pump)

    def spawn(self, argv: list[str], env_extra: dict[str, str] | None = None) -> bool:
        if self.process is not None:
            self.log("[提示] 还有子进程在跑，等日志出现退出码再点")
            return False
        self.log("$ " + " ".join(shlex.quote(part) for part in argv))
        env = dict(os.environ)
        if env_extra:
            env.update(env_extra)
            shown = " ".join(f"{key}={shlex.quote(value)}"
                             for key, value in sorted(env_extra.items())
                             if key.startswith("UNAV_VIO_"))
            if shown:
                self.log("  env " + shown)
        try:
            self.process = subprocess.Popen(
                argv, cwd=str(self.root_dir), stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, text=True, bufsize=1, env=env)
        except OSError as error:
            self.log(f"[启动失败] {error}")
            return False
        pipe = self.process.stdout

        def reader() -> None:
            assert pipe is not None
            for line in pipe:
                self.log_queue.put(line)
            pipe.close()

        self.reader_thread = threading.Thread(target=reader, daemon=True)
        self.reader_thread.start()
        return True

    def _set_replay_busy(self, busy: bool) -> None:
        state = "disabled" if busy else "normal"
        for name in ("replay_button", "archive_button", "playback_button"):
            button = getattr(self, name, None)
            if button is not None:
                button.configure(state=state)

    def _set_live_viewer_running(self, running: bool) -> None:
        if getattr(self, "live_open_button", None) is None:
            return
        self.live_open_button.configure(state="disabled" if running else "normal")
        self.live_close_button.configure(state="normal" if running else "disabled")

    def _on_close(self) -> None:
        if os.environ.get("RH_GUI_SMOKE") != "1":
            self.close_live_viewer(quiet=True)
        self.root.destroy()

    def require_rh(self) -> bool:
        if self.rh is not None:
            return True
        from tkinter import messagebox
        messagebox.showerror("缺 rh", "先按 README 构建 build-release/rh")
        return False

    # --- 控件 -------------------------------------------------------------

    def _build_widgets(self) -> None:
        tk, ttk = self.tk, self.ttk
        defaults = {
            "out": str(Path.home() / "datasets"), "seq": "room_gui_01",
            "still": "4", "excite": "3", "exposure_us": "4000", "tail": "0.2",
            "duration": "95", "imu_grid": "gyro",
        }
        self.fields = {key: tk.StringVar(value=value) for key, value in defaults.items()}
        self.checks = {key: tk.BooleanVar(value=(key == "no_depth"))
                       for key in ("no_depth", "no_pose_gt", "zero_distortion", "overwrite")}

        self._device_frame()
        self._record_frame()
        self._dataset_frame()
        self._replay_frame()

        log_frame = ttk.LabelFrame(self.root, text="输出")
        log_frame.pack(fill="both", expand=True, padx=10, pady=6)
        self.log_text = tk.Text(log_frame, height=18, state="disabled", wrap="none")
        scroll = ttk.Scrollbar(log_frame, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scroll.set)
        scroll.pack(side="right", fill="y")
        self.log_text.pack(fill="both", expand=True)

    def _device_frame(self) -> None:
        ttk = self.ttk
        frame = ttk.LabelFrame(self.root, text="设备")
        frame.pack(fill="x", padx=10, pady=6)
        ttk.Button(frame, text="探测（rh probe）", command=self.probe).pack(side="left", padx=8, pady=4)
        ttk.Label(frame, text="镜头朝上平放时静止 ‖a‖ 应接近 9.81；实测采样率必须接近 400/250 Hz"
                  ).pack(side="left")

    def _record_frame(self) -> None:
        tk, ttk = self.tk, self.ttk
        frame = ttk.LabelFrame(self.root, text="录制")
        frame.pack(fill="x", padx=10, pady=6)
        grid = ttk.Frame(frame)
        grid.pack(fill="x", padx=8, pady=4)

        def entry(row: int, col: int, label: str, key: str, width: int) -> None:
            ttk.Label(grid, text=label).grid(row=row, column=col * 2, sticky="e", padx=(0, 3))
            ttk.Entry(grid, textvariable=self.fields[key], width=width).grid(
                row=row, column=col * 2 + 1, sticky="w", padx=(0, 16))

        entry(0, 0, "输出目录", "out", 26)
        entry(0, 1, "序列名（留空=时间戳名）", "seq", 18)
        entry(1, 0, "--still 秒", "still", 6)
        entry(1, 1, "--excite 秒", "excite", 6)
        entry(2, 0, "--exposure-us", "exposure_us", 8)
        entry(2, 1, "--tail 秒", "tail", 5)
        entry(3, 0, "--duration 秒", "duration", 8)
        ttk.Label(grid, text="IMU 栅格").grid(row=3, column=2, sticky="e", padx=(0, 3))
        ttk.Combobox(grid, textvariable=self.fields["imu_grid"], values=("gyro", "accel"),
                     state="readonly", width=7).grid(row=3, column=3, sticky="w")

        checks = ttk.Frame(frame)
        checks.pack(fill="x", padx=8)
        labels = {"no_depth": "不录深度", "no_pose_gt": "不录伪 GT",
                  "zero_distortion": "写零畸变", "overwrite": "--force 覆盖已有"}
        for key, label in labels.items():
            ttk.Checkbutton(checks, text=label, variable=self.checks[key]).pack(side="left", padx=6)

        buttons = ttk.Frame(frame)
        buttons.pack(fill="x", padx=8, pady=6)
        ttk.Button(buttons, text="检查参数与磁盘", command=self.check_record).pack(side="left", padx=4)
        ttk.Button(buttons, text="在终端里开始录制", command=self.start_record).pack(side="left", padx=4)
        ttk.Button(buttons, text="浏览…", command=self.pick_out_dir).pack(side="left", padx=4)

    def _dataset_frame(self) -> None:
        tk, ttk = self.tk, self.ttk
        frame = ttk.LabelFrame(self.root, text="数据集与校验")
        frame.pack(fill="x", padx=10, pady=6)
        self.dataset_list = tk.Listbox(frame, height=6, exportselection=False)
        self.dataset_paths: list[str] = []
        self.dataset_list.pack(fill="x", expand=True, padx=8, pady=4)
        row = ttk.Frame(frame)
        row.pack(fill="x", padx=8, pady=4)
        ttk.Button(row, text="刷新列表", command=self.refresh_datasets).pack(side="left")
        self.verify_no_gt = tk.BooleanVar(value=True)
        self.verify_deep = tk.BooleanVar(value=False)
        ttk.Checkbutton(row, text="--no-gt-required", variable=self.verify_no_gt).pack(side="left", padx=8)
        ttk.Checkbutton(row, text="--deep", variable=self.verify_deep).pack(side="left")
        ttk.Button(row, text="校验选中数据集", command=self.verify_selected).pack(side="left", padx=8)

    def _replay_frame(self) -> None:
        tk, ttk = self.tk, self.ttk
        frame = ttk.LabelFrame(self.root, text="回放（unav_vio：.rrd 全量 / JSONL 在线）")
        frame.pack(fill="x", padx=10, pady=6)
        self.replay_fields = {
            "build": tk.StringVar(value=str(default_unav_build())),
            "max_frames": tk.StringVar(value="300"),
            "rrd": tk.StringVar(value="/tmp/rh_gui.rrd"),
            "artifacts": tk.StringVar(value="/tmp/rh_gui-artifacts"),
            "jsonl": tk.StringVar(value="/tmp/rh_gui-observation.jsonl"),
            "display_hz": tk.StringVar(value=""),
            # 300 帧短窗 Gate 1 的 W1 抽样 levels_median=25，出厂下限 32 会拒。
            "image_levels": tk.StringVar(value="25"),
            # D435i 暗光红外 lap 中位数经常 <10，上游默认下限 50 会在喂帧前整段拒绝。
            "image_lap": tk.StringVar(value="1"),
            "td_ms": tk.StringVar(value=""),
            # 上游默认停滞预算 120000 ms；JSONL 每行 flush 会节流消费，盘慢时按需要抬。
            "wait_ms": tk.StringVar(value=""),
        }
        self.require_gt = tk.BooleanVar(value=False)
        self.rerun_images = tk.BooleanVar(value=True)
        # Gate 1 同数据集成功 JSONL 开了对比度归一化。
        self.frontend_contrast = tk.BooleanVar(value=True)
        # 允许运动段起步时，初始化可能在一帧猛晃里锁锚（run_01 的 105.6），勾上就只等安静窗。
        self.static_start = tk.BooleanVar(value=False)
        # 最后一步是选法：dir 选已有目录，save 给还没写出来的文件，None 是数字输入不给按钮。
        labels = (("unav_vio 构建目录", "build", 44, "dir"),
                  ("MAX_FRAMES（0=整段）", "max_frames", 8, None),
                  ("输出 .rrd", "rrd", 44, "save"),
                  ("产物目录（必须为空）", "artifacts", 44, "dir"),
                  ("JSONL 观察留档", "jsonl", 44, "jsonl"),
                  ("display-hz（空=每帧）", "display_hz", 8, None))
        for row, (label, key, width, style) in enumerate(labels):
            ttk.Label(frame, text=label).grid(row=row, column=0, sticky="e", padx=6, pady=2)
            ttk.Entry(frame, textvariable=self.replay_fields[key], width=width).grid(
                row=row, column=1, sticky="w")
            if style:
                ttk.Button(frame, text="浏览…", width=7,
                           command=lambda k=key, s=style: self.pick_replay_path(k, s)).grid(
                    row=row, column=2, sticky="w", padx=(6, 0))
        overrides = ttk.Frame(frame)
        overrides.grid(row=len(labels), column=1, columnspan=2, sticky="w")
        ttk.Label(frame, text="回放侧覆盖（空=不改）").grid(row=len(labels), column=0,
                                                     sticky="e", padx=6, pady=2)
        for key, caption in (("image_levels", "levels 下限"), ("image_lap", "lap 下限"),
                             ("td_ms", "td ms"), ("wait_ms", "停滞 ms")):
            ttk.Label(overrides, text=caption).pack(side="left")
            ttk.Entry(overrides, textvariable=self.replay_fields[key], width=5).pack(
                side="left", padx=(2, 8))
        ttk.Label(overrides, text="lap 已预填 1、levels 25：D435i 暗光红外过不了上游默认 50/32；"
                                  "td 留空用 sensor.yaml（Gate 1 成功跑是 6 ms，不要一上来填 0）；"
                                  "停滞 ms 只在留档写到一半被 TimedOut 时才需要抬")\
            .pack(side="left", padx=6)
        ttk.Checkbutton(frame, text="要求 GT（本机没有位姿流，默认关）",
                        variable=self.require_gt).grid(row=len(labels) + 1, column=1, sticky="w")
        ttk.Checkbutton(frame, text="前端对比度归一化（D435i 暗光红外建议开）",
                        variable=self.frontend_contrast).grid(
            row=len(labels) + 1, column=2, sticky="w")
        ttk.Checkbutton(frame, text="只准静止段起步（等安静窗，防锁错锚点）",
                        variable=self.static_start).grid(row=len(labels) + 2, column=1, sticky="w")
        ttk.Checkbutton(frame, text="把双目画面写进 .rrd（约 0.56 MB/帧）",
                        variable=self.rerun_images).grid(row=len(labels) + 2, column=2, sticky="w")
        buttons = ttk.Frame(frame)
        buttons.grid(row=len(labels) + 3, column=1, columnspan=2, sticky="w", pady=4)
        self.replay_button = ttk.Button(buttons, text="回放写 .rrd", command=self.run_replay)
        self.replay_button.pack(side="left", padx=4)
        self.archive_button = ttk.Button(buttons, text="生成 JSONL", command=self.run_generate_jsonl)
        self.archive_button.pack(side="left", padx=4)
        self.playback_button = ttk.Button(buttons, text="JSONL 在线播放", command=self.run_jsonl_playback)
        self.playback_button.pack(side="left", padx=4)
        ttk.Button(buttons, text="出 trajectory.png", command=self.run_report).pack(side="left", padx=4)
        viewers = ttk.Frame(frame)
        viewers.grid(row=len(labels) + 4, column=1, columnspan=2, sticky="w", pady=(0, 4))
        self.live_open_button = ttk.Button(viewers, text="打开 live Viewer", command=self.open_live_viewer)
        self.live_open_button.pack(side="left", padx=4)
        self.live_close_button = ttk.Button(viewers, text="关闭 live Viewer", command=self.close_live_viewer)
        self.live_close_button.pack(side="left", padx=4)
        ttk.Button(viewers, text="打开 .rrd", command=self.open_viewer).pack(side="left", padx=4)
        ttk.Label(viewers, text="JSONL 要先生成再播；关闭只杀本界面拉起的 Viewer").pack(
            side="left", padx=6)
        self._set_live_viewer_running(False)

    # --- 动作 -------------------------------------------------------------

    def record_cfg(self) -> dict:
        cfg = {key: var.get() for key, var in self.fields.items()}
        cfg.update({key: bool(var.get()) for key, var in self.checks.items()})
        return cfg

    def probe(self) -> None:
        if self.require_rh():
            self.spawn([str(self.rh), "probe"])

    def pick_out_dir(self) -> None:
        from tkinter import filedialog
        start = Path(self.fields["out"].get() or str(Path.home()))
        chosen = filedialog.askdirectory(initialdir=str(nearest_existing_dir(start)))
        if chosen:
            self.fields["out"].set(chosen)

    def pick_replay_path(self, key: str, style: str) -> None:
        """dir 只能选存在的目录；.rrd / JSONL 往往还没生成，所以走 save 对话框。"""
        from tkinter import filedialog

        base = Path(self.replay_fields[key].get() or str(Path.home()))
        if style == "save":
            chosen = filedialog.asksaveasfilename(
                initialdir=str(nearest_existing_dir(base.parent if base.suffix else base)),
                initialfile=base.name if base.suffix else "",
                defaultextension=".rrd", filetypes=[("Rerun 文档", "*.rrd"), ("所有文件", "*")])
        elif style == "jsonl":
            chosen = filedialog.asksaveasfilename(
                initialdir=str(nearest_existing_dir(base.parent if base.suffix else base)),
                initialfile=base.name if base.suffix else "observation.jsonl",
                defaultextension=".jsonl",
                filetypes=[("观察留档", "*.jsonl"), ("所有文件", "*")])
        else:
            chosen = filedialog.askdirectory(initialdir=str(nearest_existing_dir(base)))
        if chosen:
            self.replay_fields[key].set(chosen)

    def check_record(self) -> None:
        problems = record_problems(self.record_cfg(), Path(self.fields["out"].get()))
        for problem in problems:
            self.log("[提示] " + problem)
        if not problems:
            self.log("[OK] 参数、时长与磁盘余量都没问题")

    def start_record(self) -> None:
        if not self.require_rh():
            return
        cfg = self.record_cfg()
        for problem in record_problems(cfg, Path(cfg["out"])):
            self.log("[提示] " + problem)
        argv = build_record_command(self.rh, cfg)
        launcher = terminal_launcher(argv, self.root_dir)
        if launcher is None:
            self.log("[错误] 没找到 gnome-terminal / konsole / xterm，开不了可见终端。")
            return
        self.log("$ " + " ".join(shlex.quote(part) for part in argv) + "   （新终端窗口）")
        subprocess.Popen(launcher, cwd=str(self.root_dir))

    def selected_dataset(self) -> Path | None:
        """取选中项的路径。绝不从显示文本里反推——那串里有大小和备注，分隔空格数一变就拼错路径。"""
        selection = self.dataset_list.curselection()
        if not selection or selection[0] >= len(self.dataset_paths):
            return None
        return Path(self.dataset_paths[selection[0]])

    def refresh_datasets(self) -> None:
        # delete() 会清掉选中项，所以先记住再按序号恢复，否则跑完一次校验就得重新点一遍。
        previous = self.dataset_list.curselection()
        index = previous[0] if previous else None
        self.dataset_list.delete(0, "end")
        self.dataset_paths = []
        for path, size, note in list_datasets(Path(self.fields["out"].get())):
            self.dataset_paths.append(path)
            self.dataset_list.insert("end", f"{size / 1e6:7.0f} MB  {note:<8} {path}")
        if index is not None and index < len(self.dataset_paths):
            self.dataset_list.selection_set(index)
            self.dataset_list.see(index)

    def verify_selected(self) -> None:
        if not self.require_rh():
            return
        dataset = self.selected_dataset()
        if dataset is None:
            self.log("[提示] 先在列表里点一个数据集")
            return
        self.refresh_datasets()
        self.spawn(build_verify_command(self.rh, dataset, self.verify_no_gt.get(),
                                       self.verify_deep.get()))

    def numeric_field(self, key: str, label: str) -> str:
        """喂给上游的浮点旋钮：非数字会被子进程静默忽略，所以在这里挡掉并说明。"""
        raw = self.replay_fields[key].get().strip()
        if not raw:
            return ""
        try:
            float(raw)
        except ValueError:
            self.log(f"[提示] {label}「{raw}」不是数字，这次按上游默认值")
            return ""
        return raw

    def _replay_binary(self) -> Path | None:
        build = Path(self.replay_fields["build"].get())
        binary = find_replay_binary(build)
        if binary is None:
            self.log(f"[错误] {build} 里没有 vio_mh01_replay / vio_mh01_replay_test。构建它：")
            self.log(f"  cmake -S {build.parent} -B {build} -DCMAKE_BUILD_TYPE=Debug "
                     "-DCMAKE_PREFIX_PATH=<uav_nav 的 sdk-install>")
            self.log(f"  cmake --build {build} --target vio_mh01_replay --parallel")
        return binary

    def _start_mh01_replay(self, rrd: Path | None, artifacts: Path | None,
                           jsonl: Path | None) -> None:
        dataset = self.selected_dataset()
        if dataset is None:
            self.log("[提示] 先在列表里点一个数据集")
            return
        binary = self._replay_binary()
        if binary is None:
            return
        if artifacts is not None and artifacts.exists() and any(artifacts.iterdir()):
            self.log(f"[错误] 产物目录非空，回放侧拒绝覆盖：{artifacts}")
            return
        if jsonl is not None and jsonl.exists():
            self.log(f"[错误] JSONL 已存在，上游拒绝覆盖：{jsonl}。换路径或先删掉再生成。")
            return
        frames = int(self.replay_fields["max_frames"].get() or 0)
        td_raw = self.numeric_field("td_ms", "相机时间偏移")
        if not td_raw:
            # 覆盖框留空 = 按数据集 sensor.yaml 的 td 跑。录制产物本不该有这个键；
            # 它一旦带非零值，实测全程回放会在中段停机（轨迹/标记全消失，画面照常）。
            td_in_dataset = read_timeshift_ms(dataset)
            if td_in_dataset is not None and td_in_dataset != 0.0:
                self.log(f"[提示] {dataset.name} 的 cam0/sensor.yaml 手写了 timeshift_to_imu_ms="
                         f"{td_in_dataset:g} ms（录制端从不写这个键）。实测非零 td 会让全程回放在"
                         "中段停机：那之后轨迹/landmark/特征标记全部消失而画面照常。在「回放侧"
                         "覆盖」的 td 框填 0 即可恢复整段回放（根治是删掉那行键）；rh verify 也"
                         "会报这一条。")
            if self.frontend_contrast.get():
                # 对比度归一化会把近常量的暗光红外压到 levels 下限之下：这是 Gate 1 实测过的组合，
                # 提前说清，免得把「前置门拒绝」误读成素材不可用。
                self.log("[提示] 勾了对比度归一化时，暗光素材可能连「灰度级下限」都过不了："
                         "若前置门以「图像接近常量」拒绝，在 levels 框填 1 放宽（观察侧旋钮）。")
        if self.spawn([str(binary)], replay_env(
                dataset, frames, self.require_gt.get(), rrd, artifacts, self.rerun_images.get(),
                self.numeric_field("image_levels", "levels 下限"),
                self.numeric_field("image_lap", "lap 下限"),
                td_raw,
                jsonl, self.frontend_contrast.get(),
                allow_motion_start=False if self.static_start.get() else None,
                wait_ms=self.numeric_field("wait_ms", "停滞预算"))):
            self._set_replay_busy(True)
            self.writing_rrd = rrd
            self.writing_jsonl = jsonl

    def run_report(self) -> None:
        """出 trajectory.png。产物目录里没有 estimated.tum 时点空是常态，所以先说清缺什么。"""
        raw = self.replay_fields["artifacts"].get().strip()
        artifacts = Path(raw) if raw else None
        if artifacts is None or not (artifacts / "estimated.tum").is_file():
            self.log(f"[提示] {raw or '产物目录'} 里没有 estimated.tum：先跑一次带产物目录的回放")
            return
        self.spawn(build_report_command(self.root_dir, artifacts))

    def run_replay(self) -> None:
        if self.selected_dataset() is None:
            self.log("[提示] 先在列表里点一个数据集")
            return
        rrd_raw = self.replay_fields["rrd"].get().strip()
        art_raw = self.replay_fields["artifacts"].get().strip()
        rrd = Path(rrd_raw) if rrd_raw else None
        artifacts = Path(art_raw) if art_raw else None
        if rrd is not None and rrd.exists():
            self.log(f"[提示] 将覆盖已有 .rrd：{rrd}")
        jsonl_raw = self.replay_fields["jsonl"].get().strip()
        jsonl = Path(jsonl_raw) if jsonl_raw else None
        if jsonl is not None and jsonl.exists():
            self.log(f"[提示] JSONL 已存在，这次只写 .rrd，不改观察留档：{jsonl}。"
                     "要重写请换路径或先删掉再点「生成 JSONL」。")
            jsonl = None
        frames = int(self.replay_fields["max_frames"].get() or 0)
        with_images = self.rerun_images.get() and rrd is not None
        if self.rerun_images.get() and rrd is None:
            self.log("[提示] 勾了「把画面写进 .rrd」但 .rrd 是空的：没有文件可落，画面也就没了")
        if with_images:
            need = frames * RRD_IMAGE_BYTES_PER_FRAME
            picture = (f"含双目画面（约 {need / 1e6:.0f} MB）" if frames
                       else "含双目画面（整段，帧数未知）")
            free = disk_free(rrd.parent) if frames else None
            if free is not None and free < need * 1.5:
                self.log(f"[提示] {rrd.parent} 只剩 {free / 1e6:.0f} MB，含画面的 .rrd 估算 "
                         f"{need / 1e6:.0f} MB，写满会整段报废")
        else:
            picture = "只有轨迹/点云，没有画面"
        dataset = self.selected_dataset()
        name = dataset.name if dataset is not None else "?"
        archive_note = f"，顺带 JSONL → {jsonl}" if jsonl else "，不写 JSONL"
        wait_raw = self.replay_fields["wait_ms"].get().strip()
        wait_note = f"，停滞预算 {wait_raw} ms" if wait_raw else ""
        self.log(f"回放写 .rrd {name}：MAX_FRAMES={frames}，"
                 f"require_gt={'1' if self.require_gt.get() else '0'}，"
                 f"BOOTSTRAP_FRAMES={default_bootstrap_frames(frames)}，"
                 f"FRONTEND_CONTRAST={'1' if self.frontend_contrast.get() else '0'}，"
                 f"静止起步={'1' if self.static_start.get() else '0'}，"
                 f"{'.rrd → ' + str(rrd) if rrd else '不存 .rrd（字段留空了）'}，{picture}"
                 f"{archive_note}{wait_note}（Debug 下 300 帧约 5 分钟）")
        self._start_mh01_replay(rrd, artifacts, jsonl)

    def run_generate_jsonl(self) -> None:
        """跑一遍 VIO，只写观察留档，不落 .rrd。"""
        if self.selected_dataset() is None:
            self.log("[提示] 先在列表里点一个数据集")
            return
        jsonl_raw = self.replay_fields["jsonl"].get().strip()
        if not jsonl_raw:
            self.log("[提示] 「JSONL 观察留档」是空的：填好要写出的路径再生成")
            return
        jsonl = Path(jsonl_raw)
        frames = int(self.replay_fields["max_frames"].get() or 0)
        dataset = self.selected_dataset()
        name = dataset.name if dataset is not None else "?"
        self.log(f"生成 JSONL {name} → {jsonl}：MAX_FRAMES={frames}，"
                 f"require_gt={'1' if self.require_gt.get() else '0'}，"
                 f"BOOTSTRAP_FRAMES={default_bootstrap_frames(frames)}，"
                 f"FRONTEND_CONTRAST={'1' if self.frontend_contrast.get() else '0'}，"
                 f"IMAGE_LEVELS_MIN={self.replay_fields['image_levels'].get().strip() or '未设→上游默认32'}，"
                 f"IMAGE_LAP_MIN={self.replay_fields['image_lap'].get().strip() or '未设→上游默认50'}，"
                 "不写 .rrd（仍是一遍 VIO，Debug 下 300 帧约 5 分钟）")
        self._start_mh01_replay(None, None, jsonl)

    def run_jsonl_playback(self) -> None:
        """按源时间把 JSONL 观察留档推给 live Viewer，不重跑 VIO。"""
        build = Path(self.replay_fields["build"].get())
        binary = find_playback_binary(build)
        if binary is None:
            self.log(f"[错误] {build} 里没有 vio_observation_playback。"
                     "指向含 uav_nav_rerun 的新 unav_vio 构建（例如 build-rerun060-release）。")
            return
        jsonl_raw = self.replay_fields["jsonl"].get().strip()
        if not jsonl_raw:
            self.log("[提示] 「JSONL 观察留档」是空的：先回放写出，或填已有 observation.jsonl")
            return
        archive = Path(jsonl_raw)
        if not archive.is_file():
            self.log(f"[提示] {archive} 还不存在。先点「生成 JSONL」（跑一遍 VIO，"
                     "环境变量 UNAV_VIO_OBSERVATION_ARCHIVE）。")
            return
        hz = self.numeric_field("display_hz", "display-hz")
        dataset = self.selected_dataset()
        argv = build_playback_command(binary, archive, dataset, hz)
        self.log(f"JSONL 在线播放 {archive.name}：display-hz="
                 f"{hz if hz else '0（每帧）'}；先确认 live Viewer 已在 9876 监听")
        if self.spawn(argv):
            self._set_replay_busy(True)

    def open_live_viewer(self) -> None:
        """live 推流端点写死 rerun+http://127.0.0.1:9876/proxy，必须先有监听者。"""
        if self.live_viewer is not None and self.live_viewer.poll() is None:
            self.log(f"[提示] live Viewer 已由本界面拉起（pid {self.live_viewer.pid}），先关再开")
            return
        viewer = rerun_viewer()
        if viewer is None:
            self.log("[错误] 没找到 Rerun 查看器（PATH 与 ~/桌面 都没有 rerun-cli-*）。"
                     "它必须是 0.37.x，且不随本项目安装。")
            return
        argv = live_viewer_command(viewer)
        self.log("$ " + " ".join(shlex.quote(part) for part in argv) + "   （新窗口，JSONL 在线播用）")
        try:
            self.live_viewer = subprocess.Popen(argv, start_new_session=True)
        except OSError as error:
            self.log(f"[启动失败] {error}")
            self.live_viewer = None
            return
        self._set_live_viewer_running(True)

    def close_live_viewer(self, quiet: bool = False) -> None:
        proc = self.live_viewer
        if proc is None:
            if not quiet:
                self.log("[提示] 没有由本界面拉起的 live Viewer。"
                         "你在终端里开的请在那边停（Ctrl+C）。")
            return
        if proc.poll() is not None:
            if not quiet:
                self.log("live Viewer 已经不在了")
            self.live_viewer = None
            self._set_live_viewer_running(False)
            return
        if not quiet:
            self.log(f"结束 live Viewer（pid {proc.pid}）")
        kill_process_group(proc)
        self.live_viewer = None
        self._set_live_viewer_running(False)

    def open_viewer(self) -> None:
        viewer = rerun_viewer()
        rrd_raw = self.replay_fields["rrd"].get().strip()
        if viewer is None:
            self.log("[错误] 没找到 Rerun 查看器（PATH 与 ~/桌面 都没有 rerun-cli-*）。"
                     "它必须是 0.37.x，且不随本项目安装。")
            return
        if not rrd_raw:
            self.log("[提示] 「输出 .rrd」是空的：填好路径再回放一次才会有文件可开")
            return
        rrd = Path(rrd_raw)
        if self.writing_rrd is not None:
            self.log(f"[提示] 回放还在写 {self.writing_rrd}：.rrd 的 footer 只在写入方关闭文件时才写上，"
                     "这时打开只会得到 “Missing RRD footer / no RRD manifests”。"
                     "等日志出现「已写完并关闭」再点。")
            return
        if not rrd.is_file():
            self.log(f"[提示] {rrd} 还不存在，先跑一次回放")
            return
        self.log(f"$ {viewer} {rrd}   （新窗口）")
        subprocess.Popen([str(viewer), str(rrd)])

    def run(self) -> int:
        self.root.mainloop()
        return 0


# ---------------------------------------------------------------------- selftest


def selftest(root_dir: Path) -> int:
    """不依赖 DISPLAY 的检查：命令拼装、参数校验、磁盘估算、终端包装。"""
    failures: list[str] = []

    def check(condition: bool, label: str) -> None:
        print(("[ ok ] " if condition else "[FAIL] ") + label)
        if not condition:
            failures.append(label)

    rh = Path("/x/build-release/rh")
    cfg = {"out": "/home/u/datasets", "seq": "room_07", "still": "4", "excite": "3",
           "exposure_us": "4000", "tail": "0.2", "duration": "95", "imu_grid": "gyro",
           "no_depth": True, "no_pose_gt": False, "zero_distortion": False, "overwrite": False}
    argv = build_record_command(rh, cfg)
    check(argv[:2] == ["/x/build-release/rh", "record"], "record 子命令")
    check("--no-depth" in argv and "--no-pose-gt" not in argv, "复选框映射到开关")
    check(argv[argv.index("--imu-grid") + 1] == "gyro", "--imu-grid 透传")
    check("--seq" in argv and argv[argv.index("--seq") + 1] == "room_07", "--seq 透传")
    check("--seq" not in build_record_command(rh, dict(cfg, seq="  ")),
          "序列名留空时不传 --seq，交给 rh 的默认名")
    check(all(part.strip() for part in argv), "没有把空白参数传给子进程")

    env = replay_env(Path("/d/room_07"), 300, False, Path("/tmp/a.rrd"), None)
    check(env["UNAV_VIO_MH01_REQUIRE_GT"] == "0", "require_gt 显式落到环境")
    check(env["UNAV_VIO_MH01_MAX_FRAMES"] == "300", "MAX_FRAMES 落到环境")
    check(env["UNAV_VIO_MH01_BOOTSTRAP_FRAMES"] == "300",
          "300 帧短窗 bootstrap=max(300,240)，避免上游默认 150 提前放弃")
    check("UNAV_VIO_REPLAY_ARTIFACT_DIR" not in env and env["UNAV_VIO_RERUN_SAVE"] == "/tmp/a.rrd",
          "只设给过路径的那两个产物变量")
    bare = replay_env(Path("/d/room_07"), 0, True, None, None)
    check("UNAV_VIO_RERUN_SAVE" not in bare and "UNAV_VIO_REPLAY_ARTIFACT_DIR" not in bare,
          ".rrd/产物字段留空就是不存，不会把 '.' 传给子进程")
    check(bare["UNAV_VIO_MH01_BOOTSTRAP_FRAMES"] == "8192",
          "整段（MAX_FRAMES=0）bootstrap 用够用的上限")
    check(replay_env(Path("/d"), 80, False, None, None)["UNAV_VIO_MH01_BOOTSTRAP_FRAMES"] == "240",
          "短于 240 的窗也至少等到上游「及时初始化」判据")
    check(default_bootstrap_frames(300) == 300 and default_bootstrap_frames(0) == 8192,
          "bootstrap 纯函数与 replay_env 同口径")
    check("UNAV_VIO_MH01_FRONTEND_CONTRAST" not in env,
          "对比度归一化默认不设（GUI 勾了才开）")
    check(replay_env(Path("/d"), 300, False, None, None,
                     frontend_contrast=True)["UNAV_VIO_MH01_FRONTEND_CONTRAST"] == "1",
          "勾了对比度就设 UNAV_VIO_MH01_FRONTEND_CONTRAST=1")
    # SAVE 模式默认不落图（上游 vio_mh01_replay_test.cpp 的 stereo_images 判定），必须显式开。
    check("UNAV_VIO_RERUN_IMAGES" not in replay_env(Path("/d"), 300, False, Path("/t/a.rrd"), None),
          "不开开关时 .rrd 里只有轨迹，行为与上游默认一致")
    check(replay_env(Path("/d"), 300, False, Path("/t/a.rrd"), None, True)["UNAV_VIO_RERUN_IMAGES"] == "1",
          "勾了画面就设 UNAV_VIO_RERUN_IMAGES=1")
    check("UNAV_VIO_RERUN_IMAGES" not in replay_env(Path("/d"), 300, False, None, None, True),
          "没有 .rrd 就没有落图的地方，不该设这个变量")
    # 图像质量下限是上游回放前置门的旋钮：留空必须真的什么都不设，让默认值生效。
    check(replay_env(Path("/d"), 300, False, None, None, False, "", "1")["UNAV_VIO_MH01_IMAGE_LAP_MIN"]
          == "1", "lap 下限填了才透传")
    check("UNAV_VIO_MH01_IMAGE_LEVELS_MIN" not in replay_env(Path("/d"), 300, False, None, None),
          "下限留空时交给上游默认，不设空字符串")
    check(replay_env(Path("/d"), 300, False, None, None, td_ms="0")["UNAV_VIO_MH01_TD_MS"] == "0",
          "td 覆盖成 0 也要传（数据集里写了 timeshift 时这是唯一清零办法）")
    check("UNAV_VIO_MH01_TD_MS" not in replay_env(Path("/d"), 300, False, None, None),
          "td 留空时不覆盖，交给数据集的 sensor.yaml")
    check(replay_env(Path("/d"), 300, False, None, None,
                     allow_motion_start=False)["UNAV_VIO_MH01_ALLOW_MOTION_START"] == "0",
          "勾了「只准静止段起步」就设 ALLOW_MOTION_START=0")
    check("UNAV_VIO_MH01_ALLOW_MOTION_START" not in replay_env(Path("/d"), 300, False, None, None),
          "不勾时不设：上游默认就允许运动段起步，没必要显式写 1")
    check(replay_env(Path("/d"), 300, False, None, None,
                     wait_ms="600000")["UNAV_VIO_MH01_WAIT_MS"] == "600000",
          "停滞预算填了才透传")
    check("UNAV_VIO_MH01_WAIT_MS" not in replay_env(Path("/d"), 300, False, None, None),
          "停滞预算留空时用上游默认 120000")
    report = build_report_command(Path("/repo"), Path("/d/artifacts"))
    check(report[1] == "/repo/tools/rh_report.py" and report[-1] == "/d/artifacts",
          f"出图命令指向仓库里的 rh_report.py: {report}")
    archive_env = replay_env(Path("/d"), 300, False, None, None,
                             observation_archive=Path("/tmp/o.jsonl"))
    check(archive_env["UNAV_VIO_OBSERVATION_ARCHIVE"] == "/tmp/o.jsonl",
          "填了 JSONL 路径就设 UNAV_VIO_OBSERVATION_ARCHIVE")
    check(archive_env["UNAV_VIO_REQUIRE_OBSERVATION_ARCHIVE"] == "1",
          "写 JSONL 时观察失败单独进退出码，不改写 vio_result")
    check("UNAV_VIO_OBSERVATION_ARCHIVE" not in replay_env(Path("/d"), 300, False, None, None),
          "JSONL 留空就不设，避免把 '.' 传给子进程")
    play = build_playback_command(Path("/b/vio_observation_playback"), Path("/t/o.jsonl"),
                                  Path("/d/room_07"), "5")
    check(play[:4] == ["/b/vio_observation_playback", "--archive", "/t/o.jsonl", "--connect"],
          "在线播放固定 --archive 与 --connect")
    check(play[play.index("--dataset") + 1] == "/d/room_07", "--dataset 透传选中数据集")
    check(play[play.index("--display-hz") + 1] == "5", "--display-hz 5 抽帧")
    check("--rate" not in build_playback_command(Path("/b/p"), Path("/t/o.jsonl"), None, ""),
          "display-hz 留空且 rate=1 时不传多余开关（每帧都发）")
    check("--dataset" not in build_playback_command(Path("/b/p"), Path("/t/o.jsonl"), None, "5"),
          "没选数据集时不传 --dataset，交给留档头的 dataset_path")
    live = live_viewer_command(Path("/v/rerun"))
    check(live[:4] == ["/v/rerun", "--serve-web", "--bind", "127.0.0.1"],
          "live Viewer 走 --serve-web 绑定本机")
    check(live[live.index("--port") + 1] == "9876", "live 端点端口 9876")
    check(live[live.index("--memory-limit") + 1] == "4GB", "live Viewer 带 4GB 限额")

    check(record_problems(cfg, Path("/")) == [], "合法参数不该报警")
    bad = dict(cfg, seq="has space", still="1", excite="1", tail="0", duration="10")
    joined = " | ".join(record_problems(bad, Path("/")))
    for needle in ("不合法", "1.5 s", "2.5 s", "0.1 s", "30001"):
        check(needle in joined, f"参数校验报出「{needle}」")

    accel = dict(cfg, imu_grid="accel")
    check(imu_rows(accel) == 23750, "accel 栅格按 250 Hz 算行数")
    check(any("还要多录约" in w for w in record_problems(accel, Path("/"))),
          "accel 栅格 95 s 会被提醒不够 30001 行")
    check(not any("还要多录" in w for w in record_problems(dict(cfg, duration="120"), Path("/"))),
          "gyro 栅格 120 s 足够，不该提醒")
    check(estimate_record_bytes(dict(cfg, no_depth=False)) > estimate_record_bytes(cfg),
          "带深度的估算更大")

    launcher = terminal_launcher(["printf", "a b c"], Path("/tmp"))
    if launcher is None:
        print("[info] 本机没有可用终端程序，terminal_launcher 返回 None")
    else:
        check(launcher[-3:-1] == ["bash", "-lc"], "终端命令包装成 bash -lc")
        check("printf 'a b c'" in launcher[-1], "参数按 shell 规则引用，不靠字符串拼接歧义")

    import tempfile
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp)
        good = out / "room_09"
        (good / "mav0" / "cam0" / "data").mkdir(parents=True)
        (good / "mav0" / "imu0").mkdir(parents=True)
        (good / "mav0" / "cam0" / "data.csv").write_text("1,x.png\n")
        (good / "record_summary.yaml").write_text("seq: room_09\n")
        (out / "not_a_dataset").mkdir()
        rows = list_datasets(out)
        check(len(rows) == 1 and rows[0][0] == str(good),
              "list_datasets 给出的是原始路径，不是显示串（选中项靠序号而不是解析文本）")
        check(rows[0][2] == "无GT,无深度", "备注同时认出缺 GT 和缺深度")
        check(dir_size(good) > 0, "dir_size 统计到内容")
        # 选择框的 initialdir 必须是真实目录，否则 tkinter 会退回上次的目录，用户看不出为什么。
        check(nearest_existing_dir(good / "mav0") == good / "mav0", "已存在的目录原样给出")
        check(nearest_existing_dir(good / "mav0" / "nope" / "deeper") == good / "mav0",
              "还没建出来的默认路径向上退到存在的父目录")
        check(nearest_existing_dir(Path("/no/such/place")).is_dir(),
              "任意绝对路径都能停在一个真实目录（最坏是根）")

    with tempfile.TemporaryDirectory() as tmp:
        ds = Path(tmp) / "seq_a"
        (ds / "mav0" / "cam0").mkdir(parents=True)
        check(read_timeshift_ms(ds) is None, "sensor.yaml 不存在时 td 视为未写")
        sensor = ds / "mav0" / "cam0" / "sensor.yaml"
        sensor.write_text("distortion_coefficients: [0, 0, 0, 0]\n"
                          "timeshift_to_imu_ms: 6.0  # estimate_td 实测\n")
        check(read_timeshift_ms(ds) == 6.0, "td 读数剥掉行内注释")
        sensor.write_text("timeshift_to_imu_ms: 0\n")
        check(read_timeshift_ms(ds) == 0.0, "手写 0 也读得出来")
        sensor.write_text("rate_hz: 30\n")
        check(read_timeshift_ms(ds) is None, "缺键返回 None")
        sensor.write_text("timeshift_to_imu_ms: bad\n")
        check(read_timeshift_ms(ds) is None, "td 值坏时返回 None 而不是抛异常")

    print(f"[info] rh：{find_rh(root_dir) or '未构建'}")
    print(f"[info] Rerun viewer：{rerun_viewer() or '未找到'}")
    print(f"[info] 默认 unav_vio 构建：{default_unav_build()}")
    print(f"[info] 回放二进制：{find_replay_binary(default_unav_build()) or '未找到'}")
    print(f"[info] JSONL 播放器：{find_playback_binary(default_unav_build()) or '未找到'}")
    print(f"\nselftest：{'全部通过' if not failures else f'{len(failures)} 项失败'}")
    return 1 if failures else 0


def main() -> int:
    parser = argparse.ArgumentParser(description="RecordHelper 的驱动界面")
    parser.add_argument("--selftest", action="store_true", help="只跑无界面检查")
    parser.add_argument("--smoke", action="store_true", help="构造界面 1.5 s 后自动关闭")
    parser.add_argument("--root", default=str(repo_root(Path(__file__).resolve())),
                        help="record_helper 仓库根（默认从脚本位置向上找）")
    args = parser.parse_args()
    root_dir = Path(args.root).resolve()
    if args.selftest:
        return selftest(root_dir)
    if args.smoke:
        os.environ["RH_GUI_SMOKE"] = "1"
    return App(root_dir).run()


if __name__ == "__main__":
    sys.exit(main())

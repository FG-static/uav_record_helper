#!/usr/bin/env python3
"""RecordHelper 的驱动界面：探测 → 录制 → 校验 → 回放（含 .rrd）。

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


def find_replay_binary(build_dir: Path) -> Path | None:
    candidate = build_dir / "vio_mh01_replay_test"
    if candidate.is_file() and os.access(candidate, os.X_OK):
        return candidate
    return None


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


def replay_env(dataset: Path, max_frames: int, require_gt: bool, rrd: Path | None,
               artifacts: Path | None, rerun_images: bool = False,
               image_levels_min: str = "", image_lap_min: str = "",
               td_ms: str = "") -> dict[str, str]:
    env = {
        "UNAV_VIO_EUROC_MH01": str(dataset),
        "UNAV_VIO_MH01_MAX_FRAMES": str(max_frames),
        # 结论行会印 require_gt=0/1：降级跑过必须能被事后区分，所以永远显式设。
        "UNAV_VIO_MH01_REQUIRE_GT": "1" if require_gt else "0",
    }
    if rrd is not None:
        env["UNAV_VIO_RERUN_SAVE"] = str(rrd)
        # 上游把画面写在 live 请求或显式开关上，SAVE 模式默认不落图，所以这里必须显式打开。
        if rerun_images:
            env["UNAV_VIO_RERUN_IMAGES"] = "1"
    if artifacts is not None:
        env["UNAV_VIO_REPLAY_ARTIFACT_DIR"] = str(artifacts)
    # 留空 = 用上游默认；填了才覆盖。这几个都是回放侧旋钮，不是录制端契约。
    # 注意 "0" 是有效覆盖（把数据集里写的相机时间偏移清零），所以判据是"非空"而非"非零"。
    for key, value in (("UNAV_VIO_MH01_IMAGE_LEVELS_MIN", image_levels_min),
                       ("UNAV_VIO_MH01_IMAGE_LAP_MIN", image_lap_min),
                       ("UNAV_VIO_MH01_TD_MS", td_ms)):
        if value:
            env[key] = value
    return env


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

        self.root = tk.Tk()
        self.root.title("RecordHelper")
        self.root.geometry("1000x780")
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
            self.log(f"[退出码 {self.process.returncode}]")
            self.process = None
            if self.writing_rrd is not None:
                self.log(f".rrd 已写完并关闭：{self.writing_rrd}（现在可以打开 viewer 了）")
                self.writing_rrd = None
            self.replay_button.configure(state="normal")
        self.root.after(120, self._pump)

    def spawn(self, argv: list[str], env_extra: dict[str, str] | None = None) -> None:
        self.log("$ " + " ".join(shlex.quote(part) for part in argv))
        env = dict(os.environ)
        if env_extra:
            env.update(env_extra)
        try:
            self.process = subprocess.Popen(
                argv, cwd=str(self.root_dir), stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, text=True, bufsize=1, env=env)
        except OSError as error:
            self.log(f"[启动失败] {error}")
            return
        pipe = self.process.stdout

        def reader() -> None:
            assert pipe is not None
            for line in pipe:
                self.log_queue.put(line)
            pipe.close()

        threading.Thread(target=reader, daemon=True).start()

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
        frame = ttk.LabelFrame(self.root, text="回放（直接调 unav_vio 的 vio_mh01_replay_test）")
        frame.pack(fill="x", padx=10, pady=6)
        self.replay_fields = {
            "build": tk.StringVar(value=str(Path.home() / "ace_unav" / "modules" / "unav_vio" / "build-replay")),
            "max_frames": tk.StringVar(value="300"),
            "rrd": tk.StringVar(value="/tmp/rh_gui.rrd"),
            "artifacts": tk.StringVar(value="/tmp/rh_gui-artifacts"),
            # 上游回放侧旋钮，留空就用它自带的默认值。
            "image_levels": tk.StringVar(value=""), "image_lap": tk.StringVar(value=""),
            "td_ms": tk.StringVar(value=""),
        }
        self.require_gt = tk.BooleanVar(value=False)
        self.rerun_images = tk.BooleanVar(value=True)
        # 最后一步是选法：dir 选已有目录，save 给还没写出来的 .rrd，None 是数字输入不给按钮。
        labels = (("unav_vio 构建目录", "build", 44, "dir"),
                  ("MAX_FRAMES（0=整段）", "max_frames", 8, None),
                  ("输出 .rrd", "rrd", 44, "save"),
                  ("产物目录（必须为空）", "artifacts", 44, "dir"))
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
        ttk.Label(frame, text="回放侧覆盖（空=默认）").grid(row=len(labels), column=0,
                                                     sticky="e", padx=6, pady=2)
        for key, caption in (("image_levels", "levels 下限"), ("image_lap", "lap 下限"),
                             ("td_ms", "td ms")):
            ttk.Label(overrides, text=caption).pack(side="left")
            ttk.Entry(overrides, textvariable=self.replay_fields[key], width=5).pack(
                side="left", padx=(2, 8))
        ttk.Label(overrides, text="D435i 暗光红外的 lap 中位数实测 ~1.9，默认 50 会拒；"
                                  "sensor.yaml 里的 timeshift 让初始化失败时填 0")\
            .pack(side="left", padx=6)
        ttk.Checkbutton(frame, text="要求 GT（本机没有位姿流，默认关）",
                        variable=self.require_gt).grid(row=len(labels) + 1, column=1, sticky="w")
        ttk.Checkbutton(frame, text="把双目画面写进 .rrd（约 0.56 MB/帧）",
                        variable=self.rerun_images).grid(row=len(labels) + 1, column=2, sticky="w")
        buttons = ttk.Frame(frame)
        buttons.grid(row=len(labels) + 2, column=1, columnspan=2, sticky="w", pady=4)
        self.replay_button = ttk.Button(buttons, text="回放选中数据集", command=self.run_replay)
        self.replay_button.pack(side="left", padx=4)
        ttk.Button(buttons, text="打开 Rerun viewer", command=self.open_viewer).pack(side="left", padx=4)
        ttk.Label(buttons, text="viewer 打开的是文件，不会打断回放").pack(side="left", padx=6)

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
        """dir 只能选存在的目录；.rrd 往往还没生成，所以它走 save 对话框，别拿 askdirectory 逼用户先建文件。"""
        from tkinter import filedialog

        base = Path(self.replay_fields[key].get() or str(Path.home()))
        if style == "save":
            chosen = filedialog.asksaveasfilename(
                initialdir=str(nearest_existing_dir(base.parent if base.suffix else base)),
                initialfile=base.name if base.suffix else "",
                defaultextension=".rrd", filetypes=[("Rerun 文档", "*.rrd"), ("所有文件", "*")])
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

    def run_replay(self) -> None:
        dataset = self.selected_dataset()
        if dataset is None:
            self.log("[提示] 先在列表里点一个数据集")
            return
        build = Path(self.replay_fields["build"].get())
        binary = find_replay_binary(build)
        if binary is None:
            self.log(f"[错误] {build} 里没有 vio_mh01_replay_test。构建它：")
            self.log(f"  cmake -S {build.parent} -B {build} -DCMAKE_BUILD_TYPE=Debug "
                     "-DCMAKE_PREFIX_PATH=<uav_nav 的 sdk-install>")
            self.log(f"  cmake --build {build} --target vio_mh01_replay_test --parallel")
            return
        # 留空 = 不要那份产物：replay_env 收到 None 就不设对应环境变量，
        # 省得 Path("") 变成 "." 传给子进程当输出路径。
        rrd_raw = self.replay_fields["rrd"].get().strip()
        art_raw = self.replay_fields["artifacts"].get().strip()
        rrd = Path(rrd_raw) if rrd_raw else None
        artifacts = Path(art_raw) if art_raw else None
        if artifacts is not None and artifacts.exists() and any(artifacts.iterdir()):
            self.log(f"[错误] 产物目录非空，回放侧拒绝覆盖：{artifacts}")
            return
        if rrd is not None and rrd.exists():
            self.log(f"[提示] 将覆盖已有 .rrd：{rrd}")
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
        self.log(f"回放 {dataset.name}：MAX_FRAMES={frames}，"
                 f"require_gt={'1' if self.require_gt.get() else '0'}，"
                 f"{'.rrd → ' + str(rrd) if rrd else '不存 .rrd（字段留空了）'}，{picture}"
                 f"（Debug 下 300 帧约 5 分钟）")
        self.spawn([str(binary)], replay_env(
            dataset, frames, self.require_gt.get(), rrd, artifacts, self.rerun_images.get(),
            self.numeric_field("image_levels", "levels 下限"),
            self.numeric_field("image_lap", "lap 下限"),
            self.numeric_field("td_ms", "相机时间偏移")))
        # 启动失败时 spawn 只留一行日志、self.process 仍为 None：这时别把按钮锁死。
        if self.process is not None:
            self.replay_button.configure(state="disabled")
            self.writing_rrd = rrd

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
    check("UNAV_VIO_REPLAY_ARTIFACT_DIR" not in env and env["UNAV_VIO_RERUN_SAVE"] == "/tmp/a.rrd",
          "只设给过路径的那两个产物变量")
    bare = replay_env(Path("/d/room_07"), 0, True, None, None)
    check("UNAV_VIO_RERUN_SAVE" not in bare and "UNAV_VIO_REPLAY_ARTIFACT_DIR" not in bare,
          ".rrd/产物字段留空就是不存，不会把 '.' 传给子进程")
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

    print(f"[info] rh：{find_rh(root_dir) or '未构建'}")
    print(f"[info] Rerun viewer：{rerun_viewer() or '未找到'}")
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

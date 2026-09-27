#!/usr/bin/env python3
"""把一次回放的 artifacts 画成 trajectory.png：左边 XY 平面、右边 XYZ 随时间。

数据只来自 `estimated.tum`。没有 GT 就没有绝对真值，这张图回答的是"形状、闭环、有没有跳变"，
不是"准不准"。它取代了此前散在 datasets 树里的一次性脚本；依赖 numpy/matplotlib，属于离线工具，
不参与 C++ 构建，也不被 `rh` 子命令调用。

用法：
    python3 tools/rh_report.py <artifacts目录> [--title …] [--output …]
    python3 tools/rh_report.py --selftest      # 用合成轨迹跑一遍，不需要 DISPLAY
"""

from __future__ import annotations

import argparse
import json
import sys
import tempfile
from pathlib import Path

try:
    import numpy as np
except ImportError:  # 出图是可选工具：缺依赖时说清装什么，而不是甩一段 traceback
    raise SystemExit("rh_report 需要 numpy：pip3 install numpy matplotlib")


def load_tum(path: Path) -> tuple[np.ndarray, np.ndarray]:
    timestamps: list[int] = []
    positions: list[list[float]] = []
    with path.open() as stream:
        for line in stream:
            values = line.split()
            if not values or values[0].startswith("#"):
                continue
            seconds, fraction = values[0].split(".")
            timestamps.append(int(seconds) * 10**9 + int(fraction.ljust(9, "0")))
            positions.append([float(v) for v in values[1:4]])
    if not timestamps:
        raise SystemExit(f"estimated.tum 为空: {path}")
    return np.array(timestamps, dtype=np.int64), np.array(positions)


def path_length(positions: np.ndarray) -> float:
    if len(positions) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(positions, axis=0), axis=1).sum())


def default_title(directory: Path) -> str:
    manifest = directory / "manifest.json"
    if manifest.exists():
        return str(json.loads(manifest.read_text()).get("dataset_path", directory.parent.name))
    return directory.parent.name


def render(directory: Path, output: Path, title: str) -> dict[str, float]:
    try:
        import matplotlib

        matplotlib.use("Agg")  # 无 DISPLAY 也要能跑：CI 与 ssh 里都可能用
        import matplotlib.pyplot as plt
    except ImportError:
        raise SystemExit("rh_report 需要 matplotlib：pip3 install numpy matplotlib")

    t, p = load_tum(directory / "estimated.tum")
    seconds = (t - t[0]) / 1e9
    travelled = path_length(p)
    endpoint = float(np.linalg.norm(p[-1] - p[0]))
    span = p.max(axis=0) - p.min(axis=0)

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.7), constrained_layout=True)
    axes[0].plot(p[:, 0], p[:, 1], color="#007f8b", linewidth=1.1, label="VIO estimated")
    axes[0].scatter(p[0, 0], p[0, 1], color="#c05a25", s=35, zorder=3, label="First published state")
    axes[0].scatter(p[-1, 0], p[-1, 1], color="#263444", s=35, zorder=3, label="Last published state")
    axes[0].set(xlabel="X [m]", ylabel="Y [m]", title="XY (no GT; VIO world)")
    axes[0].set_aspect("equal", adjustable="datalim")
    axes[0].legend(fontsize=8)
    axes[0].grid(alpha=0.2)

    axes[1].plot(seconds, p[:, 0], label="X", linewidth=1.0)
    axes[1].plot(seconds, p[:, 1], label="Y", linewidth=1.0)
    axes[1].plot(seconds, p[:, 2], label="Z", linewidth=1.0)
    axes[1].set(
        xlabel="Time since first published state [s]",
        ylabel="Position [m]",
        title=(
            f"{len(p)} states / {seconds[-1]:.1f}s\n"
            f"span XYZ=({span[0]:.2f}, {span[1]:.2f}, {span[2]:.2f}) m  "
            f"path={travelled:.2f} m  endpoint={endpoint:.2f} m"
        ),
    )
    axes[1].legend(fontsize=8)
    axes[1].grid(alpha=0.2)
    fig.suptitle(title, fontsize=11)
    fig.savefig(output, dpi=140)
    plt.close(fig)
    return {
        "states": float(len(p)),
        "duration_s": float(seconds[-1]),
        "path_m": travelled,
        "endpoint_m": endpoint,
    }


def selftest() -> int:
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        run = root / "run"
        run.mkdir()
        # 一个 10 m 边长的方环，首末重合：路径 40 m、首末距离 0，正好检验两个量没有互换。
        corners = [(0, 0), (10, 0), (10, 10), (0, 10), (0, 0)]
        lines = []
        for index, (x, y) in enumerate(corners):
            lines.append(f"{1790000000 + index}.000000000 {x} {y} 0")
        (run / "estimated.tum").write_text("\n".join(lines) + "\n")
        out = root / "trajectory.png"
        stats = render(run, out, "selftest")
        ok = True

        def check(condition: bool, label: str) -> None:
            nonlocal ok
            print(("[ ok ] " if condition else "[FAIL] ") + label)
            ok = ok and condition

        check(out.is_file() and out.stat().st_size > 1000, "写出 PNG")
        check(stats["states"] == 5, f"读到 5 个状态: {stats['states']}")
        check(40.0 <= stats["path_m"] <= 41.0, f"路径长 ≈40 m（不是首末距离）: {stats['path_m']:.2f}")
        check(stats["endpoint_m"] < 1e-9, f"首末距离 =0（闭环）: {stats['endpoint_m']:.3f}")
    print(f"\nselftest：{'全部通过' if ok else '有失败'}")
    return 0 if ok else 1


def main() -> int:
    parser = argparse.ArgumentParser(description="把回放 artifacts 画成 trajectory.png")
    parser.add_argument("directory", nargs="?", type=Path, help="含 estimated.tum 的 artifacts 目录")
    parser.add_argument("--title", default="", help="图标题；默认取 manifest.json 的 dataset_path")
    parser.add_argument("--output", type=Path, default=None,
                        help="输出 PNG；默认写到 artifacts 目录的上一级 trajectory.png")
    parser.add_argument("--selftest", action="store_true", help="用合成轨迹自检，不需要输入目录")
    args = parser.parse_args()
    if args.selftest:
        return selftest()
    if args.directory is None:
        parser.error("需要一个含 estimated.tum 的目录（或 --selftest）")
    if not (args.directory / "estimated.tum").is_file():
        parser.error(f"{args.directory} 里没有 estimated.tum；先跑一次带 UNAV_VIO_REPLAY_ARTIFACT_DIR 的回放")
    output = args.output if args.output is not None else args.directory.parent / "trajectory.png"
    if output.exists():
        parser.error(f"拒绝覆盖已有图: {output}")
    stats = render(args.directory, output, args.title or default_title(args.directory))
    print(f"wrote {output} states={int(stats['states'])} duration_s={stats['duration_s']:.3f} "
          f"path_m={stats['path_m']:.2f} endpoint_m={stats['endpoint_m']:.2f} "
          f"endpoint_ratio={stats['endpoint_m'] / max(stats['path_m'], 1e-9):.4f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

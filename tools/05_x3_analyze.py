#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
05_x3_analyze.py — 接入 YDLIDAR X3 的真实数据

用途：读 E:\\工作\\激光雷达\\dataset 下自己录的 CSV（frame,angle_deg,distance_mm），
      做单帧质量分析并转成标准 npz，让后续手写 ICP 能用同一套代码吃两种数据源。

用法：
    D:/slam/env/Scripts/python.exe D:/slam/tools/05_x3_analyze.py
    D:/slam/env/Scripts/python.exe D:/slam/tools/05_x3_analyze.py "E:/工作/激光雷达/dataset/test_scan.csv"

输出：
    docs/figs/x3_scan.png
    docs/X3数据画像.md
    datasets/processed/x3_<原文件名>.npz

注意 X3 的角度约定（和你那份上手手册一致）：
    顺时针为正，0° 在电机正前方，个体差异 ±3°。
    本脚本按 x = d·sin(θ), y = d·cos(θ) 画图，所以 0° 指向正上方（正前方）。
"""
import sys
import argparse
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.stdout.reconfigure(encoding="utf-8", errors="replace")
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

ROOT = Path(r"D:/slam")
X3_DIR = Path(r"E:/工作/激光雷达/dataset")
FIGS = ROOT / "docs" / "figs"
PROC = ROOT / "datasets" / "processed"


def load_x3_csv(path: Path):
    """读 X3 CSV -> dict(frame_id, angle_deg, dist_m)"""
    raw = np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding="utf-8")
    frame = np.atleast_1d(raw["frame"]).astype(np.int32)
    ang = np.atleast_1d(raw["angle_deg"]).astype(np.float64)
    dist_mm = np.atleast_1d(raw["distance_mm"]).astype(np.float64)
    dist = dist_mm / 1000.0
    return frame, ang, dist


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="?", default=str(X3_DIR / "scan_20261019_094002.csv"))
    ap.add_argument("--min", type=float, default=0.12, help="最小有效距离 m")
    ap.add_argument("--max", type=float, default=8.0, help="最大有效距离 m")
    args = ap.parse_args()

    p = Path(args.csv)
    if not p.exists():
        print(f"[错误] 找不到 {p}")
        print(f"       可用的：{[x.name for x in X3_DIR.glob('*.csv')]}")
        return 1

    frame, ang_deg, dist = load_x3_csv(p)
    valid = np.isfinite(dist) & (dist >= args.min) & (dist <= args.max)

    frames = np.unique(frame)
    print(f"文件：{p.name}")
    print(f"  帧数：{len(frames)}，原始点：{len(dist)}，有效点：{valid.sum()}（{valid.mean()*100:.1f}%）")

    stats = []
    for f in frames:
        m = (frame == f)
        mv = m & valid
        a, d = ang_deg[mv], dist[mv]
        # 角度覆盖：把 360° 分成 72 个 5° 格子，看有几格有数据
        bins = np.histogram(a, bins=np.arange(0, 361, 5))[0]
        cover = (bins > 0).mean() * 100
        stats.append(dict(frame=int(f), n_raw=int(m.sum()), n_valid=int(mv.sum()),
                          cover=cover, dmin=float(d.min()) if len(d) else 0,
                          dmed=float(np.median(d)) if len(d) else 0,
                          dmax=float(d.max()) if len(d) else 0,
                          amin=float(a.min()) if len(a) else 0,
                          amax=float(a.max()) if len(a) else 0))
        print(f"  帧 {f}: 有效 {mv.sum():4d}/{m.sum():4d} 点，角度覆盖 {cover:5.1f}%，"
              f"距离 {stats[-1]['dmin']:.2f}~{stats[-1]['dmax']:.2f} m（中位 {stats[-1]['dmed']:.2f}）")

    # 用有效点最多的那一帧作图
    best = max(stats, key=lambda s: s["n_valid"])["frame"]
    m = (frame == best) & valid
    a = np.radians(ang_deg[m])
    d = dist[m]
    x, y = d * np.sin(a), d * np.cos(a)

    FIGS.mkdir(parents=True, exist_ok=True)
    fig = plt.figure(figsize=(13.5, 6.2))
    ax = fig.add_subplot(1, 2, 1)
    ax.scatter(x, y, s=6, c="#9A3412", alpha=0.85)
    ax.plot(0, 0, "^", color="k", ms=10)
    lim = max(np.percentile(np.hypot(x, y), 98) * 1.2, 2.0)
    ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim)
    ax.set_aspect("equal")
    ax.axhline(0, lw=0.5, color="#D3D1C7"); ax.axvline(0, lw=0.5, color="#D3D1C7")
    ax.text(0, lim * 0.92, "0° 正前", ha="center", fontsize=11, color="#57534E")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title(f"X3 实测点云（帧 {best}，{int(m.sum())} 点）")
    ax.grid(alpha=0.25, lw=0.5)

    ax2 = fig.add_subplot(1, 2, 2)
    bins = np.arange(0, 361, 5)
    cov = np.histogram(ang_deg[m], bins=bins)[0]
    ax2.bar(bins[:-1], cov, width=4, align="edge", color="#9A3412", alpha=0.85)
    ax2.set_xlabel("角度 [deg]（0=正前，顺时针）"); ax2.set_ylabel("点数 / 5°")
    ax2.set_title(f"角度覆盖（{stats[0]['cover']:.0f}%，缺口=超量程或吸光）")
    ax2.grid(alpha=0.25, lw=0.5, axis="y")

    fig.tight_layout()
    out_png = FIGS / "x3_scan.png"
    fig.savefig(out_png, dpi=140, facecolor="white")

    # 转标准 npz
    PROC.mkdir(parents=True, exist_ok=True)
    out_npz = PROC / f"x3_{p.stem}.npz"
    np.savez_compressed(out_npz,
                        frame=frame.astype(np.int16),
                        angle_deg=ang_deg.astype(np.float32),
                        dist=dist.astype(np.float32),
                        valid=valid)
    print(f"\n已保存 {out_png}")
    print(f"已导出 {out_npz}（{out_npz.stat().st_size/1024:.0f} KB）")

    # 画像 markdown
    md = [f"# X3 数据画像：{p.name}\n",
          f"- 帧数：{len(frames)}",
          f"- 原始点：{len(dist)}，有效 {valid.sum()}（{valid.mean()*100:.1f}%）",
          f"- 有效判据：{args.min} m ≤ d ≤ {args.max} m\n",
          "| 帧 | 原始点 | 有效点 | 角度覆盖 | 距离范围 m | 中位距离 m |",
          "|---|---|---|---|---|---|"]
    md += [f"| {s['frame']} | {s['n_raw']} | {s['n_valid']} | {s['cover']:.0f}% | "
           f"{s['dmin']:.2f} ~ {s['dmax']:.2f} | {s['dmed']:.2f} |" for s in stats]
    md.append("\n## 和 Cartographer bag 的对比\n")
    md.append("| 项 | X3 实测 | Deutsches Museum bag |")
    md.append("|---|---|---|")
    md.append("| 原理 | 三角测距（无强度） | 多回波（取首回波） |")
    md.append("| 点数/帧 | 约 2700（8Hz） | 1079（40Hz） |")
    md.append("| 视场 | 360°（实测 348°） | 270° |")
    md.append("| 角分辨率 | 0.96° | 0.25° |")
    md.append("| 量程 | 0.12~8 m | 0.02~60 m |")
    md.append("\n> 结论：X3 是**室内小场景**雷达。做标定和前端匹配够用，但做长距离回环实验会受量程限制。\n")
    md.append("> 想跑长序列，仍需 Cartographer bag；想练标定，你自己的 X3 更真实。")
    o = ROOT / "docs" / "X3数据画像.md"
    o.write_text("\n".join(md), encoding="utf-8")
    print(f"已写入 {o}")

    if len(frames) < 100:
        print("\n⚠️ 当前只有 %d 帧。做 SLAM 需要一个**带运动的连续序列**（建议 200 帧以上）。" % len(frames))
        print("   插上雷达（CH340 → COMx），然后：")
        print("   python E:/工作/激光雷达/tools/ydlidar_raw.py --port COMx --frames 300 --save D:/slam/datasets/x3/run01.csv")
    return 0


if __name__ == "__main__":
    sys.exit(main())

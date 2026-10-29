#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
02_plot_scan.py — 看一眼扫描长什么样（L1「看图」）

用途：从 bag 里取一帧（或几帧）LaserScan，画成俯视散点图。
      第一遍看数据就干这个：确认量程到哪、哪边是墙、有没有空洞。

用法：
    D:/slam/env/Scripts/python.exe D:/slam/tools/02_plot_scan.py
    D:/slam/env/Scripts/python.exe D:/slam/tools/02_plot_scan.py <bag> --topic /horizontal_laser_2d --idx 200

输出：docs/figs/scan_frame.png
"""
import sys
import argparse
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.stdout.reconfigure(encoding="utf-8", errors="replace")
sys.path.insert(0, str(Path(__file__).parent))
from slamlib import read_laser, beam_angles, clean_mask  # noqa: E402

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

ROOT = Path(r"D:/slam")
DEFAULT_BAG = ROOT / "datasets/cartographer_2d/cartographer_paper_deutsches_museum.bag"
FIGS = ROOT / "docs" / "figs"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", nargs="?", default=str(DEFAULT_BAG))
    ap.add_argument("--topic", default=None, help="不给则自动选第一个 LaserScan 话题")
    ap.add_argument("--idx", type=int, default=100, help="取第几帧（默认 100）")
    args = ap.parse_args()

    bag = Path(args.bag)
    if not bag.exists():
        print(f"[错误] 找不到 {bag}"); return 1

    from rosbags.typesys import Stores, get_typestore
    from rosbags.rosbag1 import Reader
    ts = get_typestore(Stores.ROS1_NOETIC)

    with Reader(str(bag)) as reader:
        topic = args.topic
        if topic is None:
            cands = [c.topic for c in reader.connections if "LaserScan" in c.msgtype]
            topic = next((t for t in cands if "horizontal" in t.lower()), cands[0] if cands else None)
        if topic is None:
            print("[错误] 这个 bag 里没有 LaserScan 话题"); return 1
        print(f"使用话题：{topic}，取第 {args.idx} 帧")

        msg, cnt = None, 0
        for conn, t, raw in reader.messages(connections=[c for c in reader.connections if c.topic == topic]):
            if cnt == args.idx:
                msg = ts.deserialize_ros1(raw, conn.msgtype)
                break
            cnt += 1

    if msg is None:
        print(f"[错误] 帧号 {args.idx} 超出范围（共 {cnt} 帧）"); return 1

    r = read_laser(msg)
    a0, a1 = msg.angle_min, msg.angle_max
    ang = beam_angles(msg, len(r))

    valid = clean_mask(r, msg.range_min, msg.range_max)
    rv, av = r[valid], ang[valid]
    x, y = rv * np.cos(av), rv * np.sin(av)

    print(f"  点数 {len(r)}，有效 {valid.sum()}（{valid.mean()*100:.1f}%）")
    print(f"  量程 [{msg.range_min:.2f}, {msg.range_max:.2f}] m，实测 [{rv.min():.2f}, {rv.max():.2f}] m")
    print(f"  角度 [{np.degrees(a0):.1f}°, {np.degrees(a1):.1f}°]，角增量 {np.degrees(msg.angle_increment):.3f}°")
    print(f"  扫描周期 {msg.scan_time*1000:.0f} ms，单点时间增量 {msg.time_increment*1e6:.1f} us")

    FIGS.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(1, 2, figsize=(13, 6.2))

    ax[0].scatter(x, y, s=4, c="#9A3412", alpha=0.85)
    ax[0].plot(0, 0, "k^", ms=10)
    # 按实测点自适应缩放：标称量程 60 m 但场景可能只有 25 m，用 range_max 会把图压扁
    lim = max(float(np.percentile(rv, 98)) * 1.2, 4.0)
    ax[0].add_patch(plt.Circle((0, 0), min(lim, msg.range_max), fill=False, ls="--", lw=0.8, color="#A8A29E"))
    ax[0].set_xlim(-lim, lim); ax[0].set_ylim(-lim, lim)
    ax[0].set_aspect("equal")
    ax[0].set_xlabel("x [m]"); ax[0].set_ylabel("y [m]")
    ax[0].set_title(f"扫描俯视图（第 {args.idx} 帧，{valid.sum()} 个有效点）")
    ax[0].grid(alpha=0.25, lw=0.5)

    ax[1].plot(np.degrees(av), rv, lw=0.7, color="#9A3412")
    ax[1].axhline(msg.range_max, ls="--", lw=0.8, color="#A8A29E", label="range_max")
    ax[1].set_xlabel("角度 [deg]"); ax[1].set_ylabel("距离 [m]")
    ax[1].set_title("距离-角度原始曲线")
    ax[1].legend(fontsize=9); ax[1].grid(alpha=0.25, lw=0.5)

    fig.tight_layout()
    out = FIGS / "scan_frame.png"
    fig.savefig(out, dpi=140, facecolor="white")
    print(f"\n已保存 {out}")
    print("看图问三个问题：① 墙是直线还是锯齿？② 有没有空洞段？③ 远处点是不是明显变稀？")
    return 0


if __name__ == "__main__":
    sys.exit(main())

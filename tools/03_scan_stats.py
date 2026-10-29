#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
03_scan_stats.py — 把数据变成数字（L2「看量」）

用途：统计整段数据的量程分布、有效点数、真实帧率，以及
      「测距噪声 σ 随距离怎么变」——这个数直接就是后端信息矩阵的对角元，
      拍脑袋填 1.0 是新手最常见的错误。

用法：
    D:/slam/env/Scripts/python.exe D:/slam/tools/03_scan_stats.py
    D:/slam/env/Scripts/python.exe D:/slam/tools/03_scan_stats.py <bag> --topic /horizontal_laser_2d

输出：
    docs/figs/scan_stats.png
    docs/扫描统计_<bagname>.md
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
from slamlib import read_laser, clean_mask  # noqa: E402

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

ROOT = Path(r"D:/slam")
DEFAULT_BAG = ROOT / "datasets/cartographer_2d/cartographer_paper_deutsches_museum.bag"
FIGS = ROOT / "docs" / "figs"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", nargs="?", default=str(DEFAULT_BAG))
    ap.add_argument("--topic", default=None)
    ap.add_argument("--max", type=int, default=1200, help="最多处理多少帧（默认1200，够统计）")
    args = ap.parse_args()

    bag = Path(args.bag)
    if not bag.exists():
        print(f"[错误] 找不到 {bag}"); return 1

    from rosbags.typesys import Stores, get_typestore
    from rosbags.rosbag1 import Reader
    ts = get_typestore(Stores.ROS1_NOETIC)

    stamps, npts, nvalid, rmin, rmed, rmax = [], [], [], [], [], []
    rough_r, rough_d = [], []
    info = {}

    with Reader(str(bag)) as reader:
        topic = args.topic
        if topic is None:
            cands = [c.topic for c in reader.connections if "LaserScan" in c.msgtype]
            topic = next((t for t in cands if "horizontal" in t.lower()), cands[0] if cands else None)
        if topic is None:
            print("[错误] 没有 LaserScan"); return 1
        print(f"话题：{topic}")
        conns = [c for c in reader.connections if c.topic == topic]

        k = 0
        for conn, t, raw in reader.messages(connections=conns):
            if k >= args.max:
                break
            msg = ts.deserialize_ros1(raw, conn.msgtype)
            r = read_laser(msg)
            valid = clean_mask(r, msg.range_min, msg.range_max)
            if not valid.any():
                k += 1; continue
            rv = r[valid]

            if not info:
                info = dict(range_min=msg.range_min, range_max=msg.range_max,
                            angle_min=np.degrees(msg.angle_min), angle_max=np.degrees(msg.angle_max),
                            ang_inc=np.degrees(msg.angle_increment), scan_time=msg.scan_time,
                            time_inc=msg.time_increment, n=int(len(r)))

            stamps.append(t / 1e9)
            npts.append(len(r)); nvalid.append(int(valid.sum()))
            rmin.append(float(rv.min())); rmed.append(float(np.median(rv))); rmax.append(float(rv.max()))

            # 局部粗糙度：相邻两点的距离差，用于估计测距噪声
            d = np.abs(np.diff(np.where(valid, r, np.nan)))
            a = r[:-1]
            m = np.isfinite(d) & (d < 0.3)
            rough_r.append(a[m]); rough_d.append(d[m])
            k += 1
            if k % 200 == 0:
                print(f"  已处理 {k} 帧", end="\r")

    print(f"  完成，共 {k} 帧" + " " * 20)
    if not stamps:
        print("[错误] 没读到有效帧"); return 1

    stamps = np.array(stamps); nvalid = np.array(nvalid)
    dt = np.diff(stamps)
    dt = dt[(dt > 0) & (dt < 2)]
    hz = 1.0 / np.median(dt) if len(dt) else 0

    # 噪声：需要按欧氏距离算，这里用 range 近似（近场误差小，可接受）
    rr = np.concatenate(rough_r); dd = np.concatenate(rough_d)
    bins = np.arange(1.0, min(info["range_max"], 28), 2.0)
    idx = np.digitize(rr, bins)
    bx, bsig = [], []
    for i in range(1, len(bins)):
        sel = dd[idx == i]
        if len(sel) > 200:
            bx.append((bins[i-1] + bins[i]) / 2)
            bsig.append(sel.std() / np.sqrt(2))   # 相邻点各自独立 -> /sqrt(2)

    print(f"\n基本参数：{info['n']} 点/帧，角度 {info['angle_min']:.0f}° ~ {info['angle_max']:.0f}°，"
          f"角增量 {info['ang_inc']:.3f}°，量程 {info['range_min']:.2f}~{info['range_max']:.1f} m")
    print(f"真实帧率：{hz:.2f} Hz（标称 {1/info['scan_time']:.2f} Hz）")
    print(f"帧间隔：中位 {np.median(dt)*1000:.1f} ms，最大 {dt.max()*1000:.1f} ms，抖动 std {dt.std()*1000:.1f} ms")
    print(f"有效点比例：{nvalid.mean() / info['n'] * 100:.1f}%")
    if bx:
        print("\n测距噪声估计（局部粗糙度法）：")
        for a_, s_ in zip(bx, bsig):
            print(f"  {a_:4.1f} m 处  σ ≈ {s_*100:4.1f} cm")

    FIGS.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(2, 2, figsize=(13, 8))

    ax[0, 0].plot(stamps - stamps[0], nvalid, lw=0.5, color="#9A3412")
    ax[0, 0].set_xlabel("时间 [s]"); ax[0, 0].set_ylabel("有效点数")
    ax[0, 0].set_title("有效点数随时间（掉点说明遇到玻璃/吸光材质）")
    ax[0, 0].grid(alpha=0.25, lw=0.5)

    ax[0, 1].hist(dt * 1000, bins=60, color="#9A3412", alpha=0.8)
    ax[0, 1].axvline(np.median(dt) * 1000, ls="--", color="k", lw=0.9,
                     label=f"中位 {np.median(dt)*1000:.1f} ms")
    ax[0, 1].set_xlabel("帧间隔 [ms]"); ax[0, 1].set_ylabel("帧数")
    ax[0, 1].set_title(f"帧间隔分布（中位对应 {hz:.2f} Hz）")
    ax[0, 1].legend(fontsize=9); ax[0, 1].grid(alpha=0.25, lw=0.5)

    ax[1, 0].hist(np.concatenate([np.array([v]) for v in rmed]), bins=60,
                  color="#9A3412", alpha=0.8)
    ax[1, 0].set_xlabel("每帧距离中位数 [m]"); ax[1, 0].set_ylabel("帧数")
    ax[1, 0].set_title("场景尺度分布（你在什么样的空间里跑）")
    ax[1, 0].grid(alpha=0.25, lw=0.5)

    if bx:
        ax[1, 1].plot(bx, np.array(bsig) * 100, "o-", color="#9A3412")
        ax[1, 1].set_xlabel("距离 [m]"); ax[1, 1].set_ylabel("σ [cm]")
        ax[1, 1].set_title("测距噪声随距离增长（后端信息矩阵就填这个）")
        ax[1, 1].grid(alpha=0.25, lw=0.5)

    fig.tight_layout()
    out = FIGS / "scan_stats.png"
    fig.savefig(out, dpi=140, facecolor="white")
    print(f"\n已保存 {out}")

    md = [f"# 扫描统计：{bag.stem}\n", f"- 话题：`{topic}`", f"- 样本帧数：{k}",
          f"- 点数/帧：{info['n']}", f"- 角分辨率：{info['ang_inc']:.3f}°",
          f"- 量程：{info['range_min']:.2f} ~ {info['range_max']:.1f} m",
          f"- 实测帧率：{hz:.2f} Hz",
          f"- 帧间隔抖动：{dt.std()*1000:.1f} ms",
          f"- 有效点比例：{nvalid.mean()/info['n']*100:.1f}%\n",
          "## 测距噪声\n", "| 距离 [m] | σ [cm] |", "|---|---|"]
    md += [f"| {a_:.1f} | {s_*100:.1f} |" for a_, s_ in zip(bx, bsig)]
    md.append("\n> 用途：后端位姿图里每条边的信息矩阵，对角线应正比于 1/σ²。距离越远越不可信。")
    o2 = ROOT / "docs" / f"扫描统计_{bag.stem}.md"
    o2.write_text("\n".join(md), encoding="utf-8")
    print(f"统计已写入 {o2}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

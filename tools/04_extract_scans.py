#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
04_extract_scans.py — 把 bag 里的扫描导成 numpy（W4 前置）

用途：rosbag 不适合反复读，导成 .npz 后手写 ICP 时读一次只要毫秒级。
      导出的就是「点集 + 时间戳」，没有 ROS 依赖，Windows 原生就能跑。

用法：
    D:/slam/env/Scripts/python.exe D:/slam/tools/04_extract_scans.py
    D:/slam/env/Scripts/python.exe D:/slam/tools/04_extract_scans.py <bag> --max 300 --stride 1

输出：
    datasets/processed/<bagname>_scans.npz
      ranges  : (N, P) float32   每帧各角度上的距离，无效值为 nan
      angles  : (P,)   float32   角度轴 [rad]
      stamps  : (N,)   float64   时间戳 [s]
      meta    : 文字说明

用法示例（手写 ICP 时）：
    d = np.load("...npz")
    r0, r1 = d["ranges"][100], d["ranges"][101]
    a = d["angles"]
    m = np.isfinite(r0)
    P0 = np.stack([r0[m]*np.cos(a[m]), r0[m]*np.sin(a[m])], axis=1)
"""
import sys
import argparse
from pathlib import Path

import numpy as np

sys.stdout.reconfigure(encoding="utf-8", errors="replace")
sys.path.insert(0, str(Path(__file__).parent))
from slamlib import read_laser, beam_angles  # noqa: E402

ROOT = Path(r"D:/slam")
DEFAULT_BAG = ROOT / "datasets/cartographer_2d/cartographer_paper_deutsches_museum.bag"
OUTDIR = ROOT / "datasets" / "processed"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", nargs="?", default=str(DEFAULT_BAG))
    ap.add_argument("--topic", default=None)
    ap.add_argument("--max", type=int, default=400, help="最多导出多少帧（默认400，约40秒）")
    ap.add_argument("--stride", type=int, default=1, help="抽帧间隔，2=每两帧取一帧")
    args = ap.parse_args()

    bag = Path(args.bag)
    if not bag.exists():
        print(f"[错误] 找不到 {bag}"); return 1

    from rosbags.typesys import Stores, get_typestore
    from rosbags.rosbag1 import Reader
    ts = get_typestore(Stores.ROS1_NOETIC)

    frames, stamps, angles = [], [], None
    with Reader(str(bag)) as reader:
        topic = args.topic
        if topic is None:
            cands = [c.topic for c in reader.connections if "LaserScan" in c.msgtype]
            topic = next((t for t in cands if "horizontal" in t.lower()), cands[0] if cands else None)
        if topic is None:
            print("[错误] 没有 LaserScan"); return 1
        conns = [c for c in reader.connections if c.topic == topic]
        print(f"话题：{topic}")

        k = 0
        for conn, t, raw in reader.messages(connections=conns):
            if k % args.stride:
                k += 1; continue
            if len(frames) >= args.max:
                break
            msg = ts.deserialize_ros1(raw, conn.msgtype)
            r = read_laser(msg)
            r[~np.isfinite(r)] = np.nan
            r[r <= msg.range_min] = np.nan
            r[r >= msg.range_max] = np.nan
            frames.append(r)
            stamps.append(t / 1e9)
            if angles is None:
                angles = beam_angles(msg, len(r))
            k += 1

    if not frames:
        print("[错误] 未读到帧"); return 1

    R = np.stack(frames)
    S = np.array(stamps)
    OUTDIR.mkdir(parents=True, exist_ok=True)
    out = OUTDIR / f"{bag.stem}_scans.npz"
    np.savez_compressed(out,
                        ranges=R, angles=angles, stamps=S,
                        meta=np.array([f"topic={topic}", f"frames={len(R)}",
                                       f"stride={args.stride}", f"source={bag.name}"]))

    dt = np.diff(S)
    print(f"\n导出 {len(R)} 帧 x {R.shape[1]} 点")
    print(f"  时长 {S[-1]-S[0]:.1f} s，平均间隔 {np.median(dt)*1000:.0f} ms（{1/np.median(dt):.1f} Hz）")
    print(f"  每帧有效点：中位 {int(np.nanmedian(np.sum(np.isfinite(R), axis=1)))}")
    print(f"  文件 {out}  ({out.stat().st_size/1024/1024:.1f} MB)")
    print("\n下一步：读它，取相邻两帧，手写 ICP 求相对位姿。")
    print("        验收标准：平移误差 <3 cm、角度误差 <1°（真值从 /tf 或 KITTI poses 取）")
    return 0


if __name__ == "__main__":
    sys.exit(main())

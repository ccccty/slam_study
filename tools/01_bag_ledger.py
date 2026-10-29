#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
01_bag_ledger.py — 数据台账（W1 产出）

用途：读一个 rosbag，把里面有什么、多长、多少条、多少赫兹全部列清楚。
      这是流程里 W1 的交付物，也是后面所有分析的基准。

用法：
    D:/slam/env/Scripts/python.exe D:/slam/tools/01_bag_ledger.py <bag路径>
    # 不给参数时默认用 Deutsches Museum 那个 bag

输出：
    终端表格 + docs/数据台账_<bagname>.md
"""
import sys
import io
from pathlib import Path
from collections import defaultdict

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

ROOT = Path(r"D:/slam")
DEFAULT_BAG = ROOT / "datasets/cartographer_2d/cartographer_paper_deutsches_museum.bag"


def open_reader(path: Path):
    """兼容 rosbag2 与 rosbag1。返回 (reader, typestore)"""
    from rosbags.typesys import Stores, get_typestore
    is_bag2 = path.is_dir() or path.suffix == ".db3"
    if is_bag2:
        from rosbags.rosbag2 import Reader
        store = Stores.ROS2_HUMBLE
    else:
        from rosbags.rosbag1 import Reader
        store = Stores.ROS1_NOETIC
    return Reader(str(path)), get_typestore(store)


def deser(typestore, rawdata, msgtype):
    for fn in ("deserialize_ros1", "deserialize"):
        f = getattr(typestore, fn, None)
        if f is None:
            continue
        try:
            return f(rawdata, msgtype)
        except Exception:
            continue
    raise RuntimeError(f"无法反序列化 {msgtype}")


def main():
    bag = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_BAG

    if not bag.exists():
        print(f"[错误] 找不到 {bag}")
        print("       先跑 tools/download_all.sh 下载数据集")
        return 1

    print(f"数据文件：{bag}")
    print(f"大小：{bag.stat().st_size / 1024 / 1024:.1f} MB" if bag.is_file() else "目录形式")
    print("=" * 78)

    reader, typestore = open_reader(bag)
    with reader:
        topics = {}
        for c in reader.connections:
            topics[c.topic] = {"msgtype": c.msgtype, "count": 0, "first": None, "last": None}

        n = 0
        for conn, ts, raw in reader.messages():
            t = topics[conn.topic]
            t["count"] += 1
            if t["first"] is None:
                t["first"] = ts
            t["last"] = ts
            n += 1
            if n % 20000 == 0:
                print(f"  ...已扫描 {n} 条消息", end="\r")

    print(f"  ...扫描完成，共 {n} 条消息" + " " * 20)

    t0 = min(v["first"] for v in topics.values() if v["first"] is not None)
    t1 = max(v["last"] for v in topics.values() if v["last"] is not None)
    dur = (t1 - t0) / 1e9

    print(f"\n总时长：{dur:.1f} s（{dur/60:.1f} min）")
    print(f"话题数：{len(topics)}\n")
    print(f"{'话题':<38} {'类型':<34} {'条数':>7} {'Hz':>7}")
    print("-" * 90)
    rows = []
    for name in sorted(topics):
        v = topics[name]
        dt = (v["last"] - v["first"]) / 1e9 if v["count"] > 1 else 0
        hz = (v["count"] - 1) / dt if dt > 0 else 0
        st = v["msgtype"].replace("/msg/", "/")
        print(f"{name:<38} {st:<34} {v['count']:>7} {hz:>7.1f}")
        rows.append((name, st, v["count"], hz))

    # 关键结论：哪些话题能直接用
    print("\n" + "=" * 78)
    scans = [r for r in rows if "LaserScan" in r[1]]
    pcds = [r for r in rows if "PointCloud2" in r[1]]
    imus = [r for r in rows if "/Imu" in r[1]]
    odoms = [r for r in rows if "Odometry" in r[1]]
    tfs = [r for r in rows if r[1].endswith("/tf2_msgs/TFMessage") or "/TFMessage" in r[1]]

    print("可用性判断：")
    print(f"  激光扫描 LaserScan : {len(scans)} 个 -> {[s[0] for s in scans]}")
    print(f"  点云 PointCloud2   : {len(pcds)} 个 -> {[s[0] for s in pcds]}")
    print(f"  IMU                : {len(imus)} 个 -> {[s[0] for s in imus]}")
    print(f"  里程计 Odometry    : {len(odoms)} 个 -> {[s[0] for s in odoms]}")
    print(f"  TF 变换            : {len(tfs)} 个 -> {[s[0] for s in tfs]}")
    if not imus:
        print("\n  [注意] 这个 bag 没有 IMU。W2 的 lidar-IMU 标定要换 M2DGR / KITTI。")
    if scans:
        print(f"\n  下一步：跑 02_plot_scan.py {scans[0][0]} 看一眼扫描长什么样")

    # 写 markdown
    md = [f"# 数据台账：{bag.name}\n",
          f"- 路径：`{bag}`",
          f"- 大小：{bag.stat().st_size/1024/1024:.1f} MB" if bag.is_file() else "- 目录形式",
          f"- 总时长：{dur:.1f} s",
          f"- 话题数：{len(topics)}",
          f"- 消息总数：{n}\n",
          "| 话题 | 类型 | 条数 | 频率 Hz |", "|---|---|---|---|"]
    md += [f"| `{r[0]}` | `{r[1]}` | {r[2]} | {r[3]:.1f} |" for r in rows]
    md.append("\n## 可用性\n")
    md.append(f"- LaserScan：{len(scans)} 个 {[s[0] for s in scans]}")
    md.append(f"- PointCloud2：{len(pcds)} 个 {[s[0] for s in pcds]}")
    md.append(f"- IMU：{len(imus)} 个 {[s[0] for s in imus]}")
    md.append(f"- Odometry：{len(odoms)} 个 {[s[0] for s in odoms]}")
    md.append(f"- TF：{len(tfs)} 个 {[s[0] for s in tfs]}")

    out = ROOT / "docs" / f"数据台账_{bag.stem}.md"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text("\n".join(md), encoding="utf-8")
    print(f"\n台账已写入 {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

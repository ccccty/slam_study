#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
00_env_check.py — 环境自检

用途：确认 D:/slam 下的环境是否完整、数据集是否就位。
输出：终端打印 + docs/环境自检报告.md

用法：
    D:/slam/env/Scripts/python.exe D:/slam/tools/00_env_check.py
"""
import os
import sys
import platform
from pathlib import Path

ROOT = Path(r"D:/slam")
OUT = ROOT / "docs" / "环境自检报告.md"

PKGS = [
    ("numpy", "数值计算"), ("scipy", "科学计算/优化"),
    ("matplotlib", "画图"), ("pandas", "表格处理"),
    ("tqdm", "进度条"), ("rosbags", "纯 Python 读 rosbag"),
    ("evo", "轨迹评估 ATE/RPE"),
]

DATASETS = [
    ("datasets/cartographer_2d/cartographer_paper_deutsches_museum.bag", 400, "2D 激光 + IMU 主数据集"),
    ("datasets/kitti_odometry/dataset/sequences", 0, "KITTI 标定参数（22 个序列）"),
    ("datasets/kitti_odometry/dataset/poses", 0, "KITTI 真值轨迹（11 个序列）"),
    ("datasets/processed/cartographer_paper_deutsches_museum_scans.npz", 0, "已导出的点集"),
]

CODE = [
    ("code/kiss-icp", "前端匹配范本"),
    ("code/scancontext_tro", "回环描述子"),
    ("code/FAST_LIO", "紧耦合 LIO"),
]


def human(n):
    for u in ["B", "KB", "MB", "GB"]:
        if n < 1024:
            return f"{n:.1f}{u}"
        n /= 1024
    return f"{n:.1f}TB"


def main():
    lines, ok_all = [], True
    lines.append("# 环境自检报告\n")
    lines.append(f"- 生成时间：{__import__('datetime').datetime.now():%Y-%m-%d %H:%M:%S}")
    lines.append(f"- Python：{sys.version.split()[0]} ({platform.architecture()[0]})")
    lines.append(f"- 平台：{platform.system()} {platform.release()}")
    lines.append(f"- 解释器：{sys.executable}\n")

    lines.append("## 一、Python 依赖\n")
    lines.append("| 包 | 用途 | 状态 | 版本 |")
    lines.append("|---|---|---|---|")
    for name, use in PKGS:
        try:
            mod = __import__(name)
            ver = getattr(mod, "__version__", "ok")
            lines.append(f"| {name} | {use} | 已安装 | {ver} |")
        except Exception as e:
            ok_all = False
            lines.append(f"| {name} | {use} | **缺失** | {type(e).__name__} |")

    lines.append("\n## 二、数据集\n")
    lines.append("| 路径 | 说明 | 大小 | 状态 |")
    lines.append("|---|---|---|---|")
    for rel, min_mb, desc in DATASETS:
        p = ROOT / rel
        if not p.exists():
            ok_all = False
            lines.append(f"| `{rel}` | {desc} | - | **缺失** |")
            continue
        if p.is_file():
            mb = p.stat().st_size / 1024 / 1024
            flag = "就位" if mb >= min_mb else f"**不完整** ({mb:.0f}MB < {min_mb}MB)"
            if mb < min_mb:
                ok_all = False
            lines.append(f"| `{rel}` | {desc} | {human(p.stat().st_size)} | {flag} |")
        else:
            n = sum(1 for _ in p.rglob("*") if _.is_file())
            lines.append(f"| `{rel}` | {desc} | {n} 个文件 | 就位 |")

    lines.append("\n## 三、代码仓库\n")
    lines.append("| 路径 | 用途 | 状态 |")
    lines.append("|---|---|---|")
    for rel, desc in CODE:
        p = ROOT / rel
        if (p / ".git").exists() or (p.exists() and any(p.iterdir())):
            lines.append(f"| `{rel}` | {desc} | 就位 |")
        else:
            lines.append(f"| `{rel}` | {desc} | 未克隆（不影响本周进度） |")

    lines.append("\n## 四、结论\n")
    if ok_all:
        lines.append("**环境就绪**。可以直接运行 `01_bag_ledger.py` 开始 W1。")
    else:
        lines.append("**有缺失项**，见上表标红部分。数据集未下完就重跑 `tools/download_all.sh`。")

    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text("\n".join(lines), encoding="utf-8")
    print("\n".join(lines))
    print(f"\n已写入 {OUT}")


if __name__ == "__main__":
    main()

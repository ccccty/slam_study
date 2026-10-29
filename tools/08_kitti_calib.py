#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
08_kitti_calib.py — 读 KITTI 的标定文件，看清「标定」到底标了什么

做什么：
  1. 解析 sequences/00/calib.txt：P0..P3（相机投影矩阵）、Tr（雷达到相机外参）
     注意：KITTI odometry 数据集的 calib.txt 不含 R0_rect（那是 raw 数据集才有的），
     本脚本如实报告这一点，并按单位阵处理。
  2. 把外参拆成 平移（米）+ 旋转（转轴-角，度）。
  3. 把 4 个相机的位置反算回雷达坐标系，画俯视传感器布局图。

用法：
    D:/slam/env/Scripts/python.exe D:/slam/tools/08_kitti_calib.py

输出：
    docs/figs/kitti_sensor_layout.png
"""
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.stdout.reconfigure(encoding="utf-8", errors="replace")
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

ROOT = Path(r"D:/slam")
CALIB = ROOT / "datasets/kitti_odometry/dataset/sequences/00/calib.txt"
FIGS = ROOT / "docs" / "figs"

CAM_LABEL = {0: "相机0（灰·左）", 1: "相机1（灰·右）", 2: "相机2（彩·左）", 3: "相机3（彩·右）"}
CAM_COLOR = {0: "#185FA5", 1: "#185FA5", 2: "#9A3412", 3: "#9A3412"}


def parse_calib(path):
    """calib.txt 每行是 '名字: 12 或 9 个浮点数'。返回 {名字: ndarray}"""
    out = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or ":" not in line:
            continue
        key, vals = line.split(":", 1)
        arr = np.array([float(v) for v in vals.split()])
        out[key.strip()] = arr
    return out


def rot_to_axis_angle(R):
    """旋转矩阵 -> (转轴单位向量, 角度[度])"""
    cos_a = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    ang = np.arccos(cos_a)
    if ang < 1e-12:
        return np.array([0.0, 0.0, 1.0]), 0.0
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    axis /= (2.0 * np.sin(ang))
    return axis, np.degrees(ang)


def fmt_mat(M, name, w=11, p=4):
    lines = [f"{name} ="]
    for row in M:
        lines.append("  [" + "  ".join(f"{v:{w}.{p}f}" for v in row) + "]")
    return "\n".join(lines)


def main():
    if not CALIB.exists():
        print(f"[错误] 找不到 {CALIB}")
        return 1
    raw = parse_calib(CALIB)
    print(f"解析 {CALIB}\ncalib.txt 里的条目：{sorted(raw.keys())}")

    # ---- P0..P3：3x4 投影矩阵 = K [I | t]，像素 = P · 相机系下齐次点 ----
    P = {i: raw[f"P{i}"].reshape(3, 4) for i in range(4)}
    for i in range(4):
        print("\n" + fmt_mat(P[i], f"P{i}（{CAM_LABEL[i]} 的投影矩阵）"))

    K0 = P[0][:, :3]
    fx, fy, cx, cy = K0[0, 0], K0[1, 1], K0[0, 2], K0[1, 2]
    print(f"\n相机内参（从 P0 读出）：焦距 fx=fy={fx:.3f} 像素，主点 (cx, cy)=({cx:.2f}, {cy:.2f}) 像素")
    print("内参回答的问题是：相机自己怎么看世界——一个 3D 点投到图像哪个像素。")

    # ---- R0_rect：odometry 数据集没有这个条目 ----
    if "R0_rect" in raw:
        R0 = raw["R0_rect"].reshape(3, 3)
        print("\nR0_rect 存在，内容如下：")
        print(fmt_mat(R0, "R0_rect"))
    else:
        R0 = np.eye(3)
        print("\n[说明] 这份 calib.txt 里没有 R0_rect。"
              "KITTI odometry 数据集已经把图像做了畸变校正，R0_rect 隐含为单位阵；"
              "它只出现在 KITTI raw 数据集里。下面按单位阵处理。")

    # ---- Tr：velodyne -> camera 外参（3x4）----
    key_tr = "Tr_velo_to_cam" if "Tr_velo_to_cam" in raw else "Tr"
    Tr = raw[key_tr].reshape(3, 4)
    print(f"\n{fmt_mat(Tr, key_tr + '（velodyne → camera0 外参，3×4）')}")
    R, t = Tr[:, :3], Tr[:, 3]
    axis, ang = rot_to_axis_angle(R)
    print(f"\n外参拆解（回答「雷达装在相机系的哪里、朝哪」）：")
    print(f"  平移 t = [{t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f}] m，|t| = {np.linalg.norm(t):.4f} m")
    print(f"  旋转角 = {ang:.2f}°，转轴 = [{axis[0]:.3f}, {axis[1]:.3f}, {axis[2]:.3f}]")
    print("  解读：转轴 ≈ (1,-1,1)/√3、转角 ≈ 120°，本质是坐标轴换序——")
    print("  雷达系 x 前/y 左/z 上，相机系 z 前/x 右/y 下，两者相差约 120° 的轴置换。")

    # ---- 把相机中心反算到雷达坐标系 ----
    # P_i = K [R0_rect | t_rect]；相机中心在 rect 系下 c = -K^{-1} P[:,3]
    # 再到雷达系：p_velo = R^T (R0^T c - t)
    cam_velo, cam_dir = {}, {}
    print("\n各相机在雷达坐标系下的位置（x 前 / y 左 / z 上）：")
    for i in range(4):
        Ki = P[i][:, :3]
        c_rect = -np.linalg.inv(Ki) @ P[i][:, 3]
        p = R.T @ (np.linalg.inv(R0) @ c_rect - t)
        d = R.T @ (np.linalg.inv(R0) @ np.array([0.0, 0.0, 1.0]))  # 光轴在雷达系的方向
        cam_velo[i], cam_dir[i] = p, d
        print(f"  {CAM_LABEL[i]}: 位置 [{p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f}] m，"
              f"光轴方向 [{d[0]:+.3f}, {d[1]:+.3f}, {d[2]:+.3f}]")
    b01 = np.linalg.norm(cam_velo[1] - cam_velo[0])
    b23 = np.linalg.norm(cam_velo[3] - cam_velo[2])
    print(f"  双目基线：灰度对 |cam1-cam0| = {b01*100:.1f} cm，彩色对 |cam3-cam2| = {b23*100:.1f} cm")

    # ---- 俯视布局图（雷达系：x 前 → 图上向右，y 左 → 图上向上）----
    FIGS.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(9.5, 7.5))
    ax.scatter([0], [0], s=220, marker="o", color="#44403C", zorder=5)
    ax.annotate("Velodyne 激光雷达\n(原点)", (0, 0), textcoords="offset points",
                xytext=(10, -26), fontsize=11, color="#44403C")
    L = 0.45  # 箭头长度
    ax.annotate("", xy=(L, 0), xytext=(0, 0),
                arrowprops=dict(arrowstyle="-|>", color="#44403C", lw=2))
    ax.text(L + 0.03, 0.0, "雷达前向 x", fontsize=10, color="#44403C", va="center")
    for i in range(4):
        p, d = cam_velo[i], cam_dir[i]
        ax.scatter([p[0]], [p[1]], s=140, marker="^", color=CAM_COLOR[i], zorder=5)
        ax.annotate("", xy=(p[0] + 0.3 * d[0], p[1] + 0.3 * d[1]), xytext=(p[0], p[1]),
                    arrowprops=dict(arrowstyle="-|>", color=CAM_COLOR[i], lw=1.8))
        ax.annotate(CAM_LABEL[i], (p[0], p[1]), textcoords="offset points",
                    xytext=(8, 10), fontsize=11, color=CAM_COLOR[i])
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]（前）")
    ax.set_ylabel("y [m]（左）")
    ax.set_title("KITTI 传感器布局俯视图（雷达坐标系，由 calib.txt 反算）\n"
                 f"雷达→相机0：平移 |t|={np.linalg.norm(t):.3f} m，旋转 {ang:.1f}°（≈120° 轴置换）")
    ax.grid(alpha=0.25, lw=0.5)
    fig.tight_layout()
    out = FIGS / "kitti_sensor_layout.png"
    fig.savefig(out, dpi=140, facecolor="white")
    print(f"\n已保存 {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

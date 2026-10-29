#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
10_odometry.py — W3 手写前端里程计：400 帧 scan-to-scan ICP 里程计 + 回放数据导出

在 06 的单帧 ICP 实验基础上往前走一步：不再有人工合成的真值，
而是把 400 帧真实扫描一帧一帧「贴」起来，链出一条完整轨迹和一张累积地图。

核心做法（全部复用 06 的点对点 ICP）：
  - 每帧：最近邻对应（cKDTree）+ Kabsch/SVD 闭式解 + 1.0 m 对应门限
  - 初值用「恒速模型」：把上一帧求出的相对运动当作本帧初值。
    为什么这样更稳：机器人 10~20 Hz 采样，相邻两帧的真实相对运动通常只有
    几厘米 / 零点几度，而上一次的运动是最好的免费预测。06 已证明初值偏差
    超过临界值（约 22°）ICP 就会收敛到错误解——零初值等于每帧都赌
    「这一帧没动」，恒速初值则赌「这一帧和上一帧动得差不多」，后者赌赢的
    概率大得多，迭代次数也更少。本脚本后半部分用对照实验量化这件事。
  - 降采样：ICP 用每帧均匀抽 350 点，保证 400 帧几分钟内跑完；
    导出给前端展示的扫描点每帧 ≤140 点、地图每帧增量 ≤100 点。

用法：
    D:/slam/env/Scripts/python.exe D:/slam/tools/10_odometry.py

输出：
    docs/assets/data/slam_replay.json   （严格按 CONTRACT.md v2 第 4 节）
    docs/figs/odom_map.png              轨迹 + 最终累积地图
    docs/figs/odom_quality.png          每帧 rmse / 内点数曲线
"""
import sys
import json
import time
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.stdout.reconfigure(encoding="utf-8", errors="replace")
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

ROOT = Path(r"D:/slam")
NPZ = ROOT / "datasets/processed/cartographer_paper_deutsches_museum_scans.npz"
FIGS = ROOT / "docs" / "figs"
OUT_JSON = ROOT / "docs" / "assets" / "data" / "slam_replay.json"

RMIN, RMAX = 0.3, 25.0      # 有效点量程（与 06 同口径）
N_ICP = 350                 # ICP 每帧降采样点数
GATE = 1.0                  # 对应点最大距离门限 [m]
MAX_ITER = 60               # 单次 ICP 最大迭代
SCAN_OUT = 140              # 导出 JSON 每帧扫描点上限（契约 ≤140）
MAP_INC = 100               # 导出 JSON 每帧地图增量上限（契约 ≤100）
JSON_BUDGET = 1_500_000     # 契约体积上限 1.5MB


# ---------- 基础变换工具（与 06 相同）----------
def apply(T, P):
    """把点集 P (N,2) 用 2D 齐次变换 T 搬过去"""
    return (T[:2, :2] @ P.T).T + T[:2, 2]


def kabsch(A, B):
    """给定对应点 A[i] ↔ B[i]，求最优刚体变换 A -> B（闭式解，SVD）"""
    ca, cb = A.mean(0), B.mean(0)
    H = (A - ca).T @ (B - cb)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1.0, d]) @ U.T
    T = np.eye(3)
    T[:2, :2] = R
    T[:2, 2] = cb - R @ ca
    return T


def icp(P, Q, T0, max_iter=MAX_ITER, tol=1e-7, gate=GATE):
    """点对点 ICP：把当前帧 P 对齐到参考帧 Q（均为各自雷达系局部坐标）。

    返回 (T_rel, 迭代次数, rmse, 内点数)：
      T_rel 是 P 坐标系 -> Q 坐标系 的刚体变换；
      rmse 为收敛时内点对应距离的均方根 [m]。
    """
    tree = cKDTree(Q)
    T = T0.copy()
    prev = np.inf
    n_in = 0
    rmse = np.inf
    it = 0
    for it in range(max_iter):
        Pw = apply(T, P)
        d, idx = tree.query(Pw)
        m = d < gate
        if m.sum() < 10:            # 对应太少，认为失败，保留当前估计
            break
        Pw_m, Qm = Pw[m], Q[idx[m]]
        T = kabsch(Pw_m, Qm) @ T    # 累加增量变换
        dm = d[m]
        err = float(np.mean(dm ** 2))
        n_in = int(m.sum())
        rmse = float(np.sqrt(err))
        if abs(prev - err) < tol * max(1.0, err):
            break
        prev = err
    return T, it + 1, rmse, n_in


# ---------- 数据装载 ----------
def load_scans():
    d = np.load(NPZ)
    R, A, S = d["ranges"], d["angles"], d["stamps"]
    n = R.shape[0]
    scans = []        # 全量有效点（局部系），供导出展示
    scans_icp = []    # 均匀降采样点（局部系），供 ICP
    for i in range(n):
        r = R[i]
        m = np.isfinite(r) & (r > RMIN) & (r < RMAX)
        pts = np.stack([r[m] * np.cos(A[m]), r[m] * np.sin(A[m])], axis=1)
        scans.append(pts)
        if len(pts) > N_ICP:
            pts = pts[np.linspace(0, len(pts) - 1, N_ICP).astype(int)]
        scans_icp.append(pts)
    stamps = (S - S[0]).astype(float)
    return scans, scans_icp, stamps


# ---------- 里程计主链 ----------
def run_odometry(scans_icp, use_motion_model=True):
    """逐帧 scan-to-scan ICP 链。

    use_motion_model=True  : 初值 = 上一帧的相对运动（恒速模型）
    use_motion_model=False : 初值 = 单位阵（零初值，赌每帧都没动）
    返回 dict：poses (N,3)，每帧 rmse / n_inlier / n_iter / T_rel
    """
    n = len(scans_icp)
    W = np.eye(3)                 # 当前帧的全局位姿（局部系 -> 全局系）
    T_prev_rel = np.eye(3)        # 上一帧求出的相对运动
    poses = np.zeros((n, 3))
    rmses = np.zeros(n)
    inliers = np.zeros(n, dtype=int)
    iters = np.zeros(n, dtype=int)

    for i in range(1, n):
        # 恒速初值：预测「这一帧和上一帧动得一样」。
        # 06 的实验说明 ICP 是局部收敛的，初值离真值越近，收敛越快越不容易跑飞。
        # 相邻帧真实运动只有厘米级，上一次的相对运动是零成本能拿到的最好预测。
        T0 = T_prev_rel if use_motion_model else np.eye(3)
        T_rel, n_it, rmse, n_in = icp(scans_icp[i], scans_icp[i - 1], T0)
        W = W @ T_rel
        T_prev_rel = T_rel
        poses[i, 0], poses[i, 1] = W[0, 2], W[1, 2]
        poses[i, 2] = np.arctan2(W[1, 0], W[0, 0])
        rmses[i], inliers[i], iters[i] = rmse, n_in, n_it
    return {"poses": poses, "rmse": rmses, "inliers": inliers,
            "iters": iters, "W_final": W}


def path_length(poses):
    d = np.diff(poses[:, :2], axis=0)
    return float(np.sum(np.hypot(d[:, 0], d[:, 1])))


# ---------- 导出 slam_replay.json ----------
def r4(x):
    return round(float(x), 4)


def build_json(frames_data, map_pts, meta):
    return {"schema": "slam_replay/1",
            "source": "cartographer_paper_deutsches_museum.bag / horizontal_laser_2d",
            "frames": frames_data, "map": map_pts, "meta": meta}


def export_replay(scans, stamps, odom):
    """按契约 v2 第 4 节导出；为控体积，scan/地图增量在上限内自适应缩减。"""
    n = len(scans)
    poses = odom["poses"]

    def build(scan_n, inc_n):
        frames, map_pts = [], []
        for i in range(n):
            W = np.eye(3)
            th = poses[i, 2]
            W[0, 0], W[0, 1] = np.cos(th), -np.sin(th)
            W[1, 0], W[1, 1] = np.sin(th), np.cos(th)
            W[0, 2], W[1, 2] = poses[i, 0], poses[i, 1]
            pts = scans[i]
            if len(pts) > scan_n:
                pts = pts[np.linspace(0, len(pts) - 1, scan_n).astype(int)]
            gw = apply(W, pts)
            scan_out = [[r4(x), r4(y)] for x, y in gw]
            # 地图增量：从本帧 scan 里再均匀取子集（契约允许的做法）
            if len(gw) > inc_n:
                inc = gw[np.linspace(0, len(gw) - 1, inc_n).astype(int)]
            else:
                inc = gw
            map_pts += [[r4(x), r4(y)] for x, y in inc]
            frames.append({
                "i": i, "t": r4(stamps[i]),
                "pose": [r4(poses[i, 0]), r4(poses[i, 1]), r4(poses[i, 2])],
                "rmse": round(float(odom["rmse"][i]), 6),
                "n_inlier": int(odom["inliers"][i]),
                "scan": scan_out, "map_upto": len(map_pts)})
        meta = {"frames_total": n,
                "path_length_m": r4(path_length(poses)),
                "odom_final": [r4(poses[-1, 0]), r4(poses[-1, 1]), r4(poses[-1, 2])],
                "method": "scan-to-scan ICP 链（无回环校正）"}
        return build_json(frames, map_pts, meta)

    scan_n, inc_n = SCAN_OUT, MAP_INC
    while True:
        obj = build(scan_n, inc_n)
        payload = json.dumps(obj, ensure_ascii=False, separators=(",", ":"))
        nbytes = len(payload.encode("utf-8"))
        if nbytes <= JSON_BUDGET or (scan_n <= 40 and inc_n <= 30):
            return payload, nbytes, scan_n, inc_n
        # 超预算：按比例缩减（保持 scan : inc ≈ 140:100）
        scale = 0.92 * JSON_BUDGET / nbytes
        scan_n = max(40, int(scan_n * scale))
        inc_n = max(30, int(inc_n * scale))


# ---------- 出图（rviz 风格深色底）----------
DARK = "#12161c"
GRID = "#2a323d"


def style_ax(ax):
    ax.set_facecolor(DARK)
    ax.figure.set_facecolor(DARK)
    for s in ax.spines.values():
        s.set_color(GRID)
    ax.tick_params(colors="#9aa4b2", labelsize=9)
    ax.xaxis.label.set_color("#c8d0da")
    ax.yaxis.label.set_color("#c8d0da")
    ax.title.set_color("#e6ebf2")
    ax.grid(color=GRID, lw=0.5, alpha=0.8)


def fig_map(poses, map_pts, out):
    fig, ax = plt.subplots(figsize=(10.5, 9))
    style_ax(ax)
    mp = np.asarray(map_pts)
    ax.scatter(mp[:, 0], mp[:, 1], s=0.3, c="#5b8dd9", alpha=0.35,
               linewidths=0, label="累积地图点")
    ax.plot(poses[:, 0], poses[:, 1], "-", color="#e8b339", lw=1.6,
            label="里程计轨迹")
    ax.scatter(*poses[0, :2], marker="o", s=70, c="#3ecf6f", zorder=5,
               label="起点")
    ax.scatter(*poses[-1, :2], marker="s", s=70, c="#e5534b", zorder=5,
               label="终点")
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title("400 帧 scan-to-scan ICP 里程计：轨迹 + 累积地图（无回环校正）")
    leg = ax.legend(fontsize=10, loc="upper right", facecolor="#1c232d",
                    edgecolor=GRID, labelcolor="#c8d0da")
    fig.tight_layout()
    fig.savefig(out, dpi=150, facecolor=DARK)


def fig_quality(odom, out):
    n = len(odom["rmse"])
    xs = np.arange(n)
    fig, ax = plt.subplots(2, 1, figsize=(11, 6.4), sharex=True)
    for a in ax:
        style_ax(a)
    ax[0].plot(xs, odom["rmse"] * 100, color="#5b8dd9", lw=1.1)
    ax[0].set_ylabel("收敛 RMSE [cm]")
    ax[0].set_title("每帧 ICP 收敛质量（第 0 帧为原点，无匹配）")
    ax[1].plot(xs, odom["inliers"], color="#3ecf6f", lw=1.1)
    ax[1].set_ylabel("内点数")
    ax[1].set_xlabel("帧号")
    ax[1].set_ylim(0, N_ICP * 1.05)
    fig.tight_layout()
    fig.savefig(out, dpi=150, facecolor=DARK)


def main():
    t_start = time.time()
    if not NPZ.exists():
        print(f"[错误] 找不到 {NPZ}")
        return 1

    scans, scans_icp_, stamps = load_scans()
    n = len(scans)
    print(f"装载 {n} 帧；ICP 每帧 {N_ICP} 点，门限 {GATE} m，max_iter={MAX_ITER}")

    # 主链：恒速初值
    t0 = time.time()
    odom = run_odometry(scans_icp_, use_motion_model=True)
    t_main = time.time() - t0
    poses = odom["poses"]
    print(f"\n[主链·恒速初值] 400 帧耗时 {t_main:.1f}s")
    print(f"  最终位姿 x={poses[-1,0]:.2f} m, y={poses[-1,1]:.2f} m, "
          f"theta={np.degrees(poses[-1,2]):.1f}°")
    print(f"  路径长度 {path_length(poses):.1f} m")
    print(f"  平均迭代 {odom['iters'][1:].mean():.1f} 次/帧，"
          f"RMSE 中位 {np.median(odom['rmse'][1:])*100:.2f} cm，"
          f"内点数中位 {int(np.median(odom['inliers'][1:]))}")

    # 对照链：零初值
    t0 = time.time()
    odom0 = run_odometry(scans_icp_, use_motion_model=False)
    t_zero = time.time() - t0
    poses0 = odom0["poses"]
    dev = np.hypot(*(poses[:, :2] - poses0[:, :2]).T)
    print(f"\n[对照·零初值] 耗时 {t_zero:.1f}s，平均迭代 {odom0['iters'][1:].mean():.1f} 次/帧")
    print(f"  两条轨迹最大偏差 {dev.max():.3f} m（第 {int(np.argmax(dev))} 帧），"
          f"末端偏差 {dev[-1]:.3f} m")
    print(f"  恒速 vs 零初值：平均迭代 {odom['iters'][1:].mean():.1f} vs "
          f"{odom0['iters'][1:].mean():.1f} 次/帧")

    # 导出 JSON
    OUT_JSON.parent.mkdir(parents=True, exist_ok=True)
    payload, nbytes, scan_n, inc_n = export_replay(scans, stamps, odom)
    OUT_JSON.write_text(payload, encoding="utf-8")
    print(f"\n已写入 {OUT_JSON}（{nbytes/1e6:.3f} MB，scan {scan_n} 点/帧，"
          f"地图增量 {inc_n} 点/帧）")

    # 出图
    FIGS.mkdir(parents=True, exist_ok=True)
    obj = json.loads(payload)
    fig_map(poses, obj["map"], FIGS / "odom_map.png")
    print(f"已保存 {FIGS / 'odom_map.png'}")
    fig_quality(odom, FIGS / "odom_quality.png")
    print(f"已保存 {FIGS / 'odom_quality.png'}")

    print(f"\n总耗时 {time.time() - t_start:.1f}s")
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
09_time_sync.py — 激光与 IMU 的时间关系：帧间隔抖动、IMU 密度、yaw 互相关

三个实验：
  ① 激光帧间隔直方图 —— 双峰分布是「帧率不稳定」的直接证据
  ② 相邻两帧激光之间有多少个 IMU 样本（均值/最小/最大）
  ③ 挑战项：相邻激光帧做 ICP 估计帧间 yaw 角速度，与 IMU gyro z 的平均角速度
     做互相关，在 ±200 ms 内扫滞后量，报最优滞后与相关系数。

时间戳说明：统一用 bag 录制时间（rosbags 的 message time），与 03_scan_stats 一致。

用法：
    D:/slam/env/Scripts/python.exe D:/slam/tools/09_time_sync.py [--max-frames 2000]

输出：
    docs/figs/time_sync_intervals.png
    docs/figs/time_sync_yawcorr.png
"""
import sys
import argparse
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.stdout.reconfigure(encoding="utf-8", errors="replace")
sys.path.insert(0, str(Path(__file__).parent))
from slamlib import read_laser, clean_mask, beam_angles, polar_to_xy  # noqa: E402

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

ROOT = Path(r"D:/slam")
BAG = ROOT / "datasets/cartographer_2d/cartographer_paper_deutsches_museum.bag"
FIGS = ROOT / "docs" / "figs"


# ---------- 2D ICP（与 06_icp_minimal 同思路：KD 树找对应 + SVD 闭式解）----------
def kabsch_2d(A, B):
    ca, cb = A.mean(0), B.mean(0)
    H = (A - ca).T @ (B - cb)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1.0, d]) @ U.T
    return R, cb - R @ ca


def icp_yaw(src, dst, max_iter=8, gate=0.8, min_inliers=80):
    """估计 src 帧 -> dst 帧的 yaw 增量 [deg]。失败返回 None。"""
    tree = cKDTree(dst)
    R = np.eye(2); t = np.zeros(2)
    for _ in range(max_iter):
        Pw = (R @ src.T).T + t
        d, idx = tree.query(Pw)
        m = d < gate
        if m.sum() < min_inliers:
            return None
        dR, dt_ = kabsch_2d(Pw[m], dst[idx[m]])
        R = dR @ R; t = dR @ t + dt_
    yaw = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
    resid = float(np.mean(d[m] ** 2))
    return yaw, resid, int(m.sum())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--max-frames", type=int, default=2000,
                    help="处理前多少帧激光（默认 2000，约 53 s）")
    ap.add_argument("--lag-ms", type=float, default=200.0, help="互相关滞后扫描范围 ±ms")
    ap.add_argument("--lag-step", type=float, default=1.0, help="滞后步长 ms")
    args = ap.parse_args()

    if not BAG.exists():
        print(f"[错误] 找不到 {BAG}")
        return 1

    # ---------- 读 bag：激光帧 + IMU ----------
    from rosbags.typesys import Stores, get_typestore
    from rosbags.rosbag1 import Reader
    ts = get_typestore(Stores.ROS1_NOETIC)

    laser_t, laser_xy = [], []
    imu_t, imu_gz = [], []
    with Reader(str(BAG)) as reader:
        conn_laser = [c for c in reader.connections if c.topic == "horizontal_laser_2d"]
        conn_imu = [c for c in reader.connections if c.topic == "imu"]
        if not conn_laser or not conn_imu:
            print("[错误] 缺话题 horizontal_laser_2d 或 imu"); return 1
        print(f"话题：{conn_laser[0].topic}（{conn_laser[0].msgtype}）+ {conn_imu[0].topic}（{conn_imu[0].msgtype}）")
        for conn, t, raw in reader.messages(connections=conn_laser + conn_imu):
            if conn.topic == "imu":
                msg = ts.deserialize_ros1(raw, conn.msgtype)
                imu_t.append(t / 1e9)
                imu_gz.append(msg.angular_velocity.z)
            else:
                if len(laser_t) >= args.max_frames and len(imu_t) > 0 and imu_t[-1] > laser_t[-1]:
                    break
                msg = ts.deserialize_ros1(raw, conn.msgtype)
                r = read_laser(msg)
                m = clean_mask(r, msg.range_min, msg.range_max)
                ang = beam_angles(msg, len(r))
                xy = polar_to_xy(r[m], ang[m])
                # 降采样到 ≤360 点，ICP 更快
                if len(xy) > 360:
                    xy = xy[:: len(xy) // 360 + 1]
                laser_t.append(t / 1e9)
                laser_xy.append(xy)
                if len(laser_t) % 400 == 0:
                    print(f"  已读 {len(laser_t)} 帧激光，{len(imu_t)} 条 IMU", end="\r")

    laser_t = np.array(laser_t); imu_t = np.array(imu_t)
    imu_gz = np.array(imu_gz)
    print(f"\n读完：激光 {len(laser_t)} 帧，IMU {len(imu_t)} 条，"
          f"窗口 {laser_t[-1] - laser_t[0]:.1f} s")

    # ---------- ① 帧间隔 ----------
    dt = np.diff(laser_t) * 1000.0  # ms
    med = np.median(dt)
    main_cl = np.mean((dt >= 26) & (dt < 30)) * 100
    second_cl = np.mean((dt >= 33) & (dt < 38)) * 100
    print(f"\n[实验①] 帧间隔（共 {len(dt)} 个）：")
    print(f"  中位 {np.median(dt):.2f} ms，均值 {dt.mean():.2f} ms，"
          f"最小 {dt.min():.2f} ms，最大 {dt.max():.2f} ms，std {dt.std():.2f} ms")
    print(f"  主簇 26~30 ms 占 {main_cl:.1f}%，次簇 33~38 ms 占 {second_cl:.1f}% → 双峰")
    print(f"  实测帧率（1/中位间隔）= {1000/med:.2f} Hz")
    # 若假设固定 26.7 ms 打时间戳，窗口末尾累积的系统偏差：
    N = len(dt)
    fixed = 26.7
    drift_ms = (laser_t[-1] - (laser_t[0] + N * fixed / 1000.0)) * 1000.0
    print(f"  若按固定 {fixed} ms 赋时间戳：本窗口（{laser_t[-1]-laser_t[0]:.1f} s / {N} 帧）"
          f"末尾累积偏差 {drift_ms:+.1f} ms（平均每帧 {drift_ms/N:+.3f} ms）")

    FIGS.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(1, 2, figsize=(13, 4.6))
    ax[0].hist(dt, bins=np.arange(24, 44, 0.25), color="#185FA5", alpha=0.85)
    ax[0].axvline(med, color="#9A3412", ls="--", lw=1.5, label=f"中位 {med:.1f} ms")
    ax[0].set_xlabel("相邻帧间隔 [ms]"); ax[0].set_ylabel("次数")
    ax[0].set_title("激光帧间隔直方图：主簇 26.7 ms + 次簇 34–36 ms（双峰）")
    ax[0].legend(); ax[0].grid(alpha=0.25, lw=0.5)
    ax[1].plot((laser_t[1:] - laser_t[0]), dt, lw=0.4, color="#185FA5")
    ax[1].set_xlabel("时间 [s]"); ax[1].set_ylabel("帧间隔 [ms]")
    ax[1].set_title("帧间隔随时间：抖动不是偶然，是持续存在")
    ax[1].grid(alpha=0.25, lw=0.5)
    fig.tight_layout()
    out1 = FIGS / "time_sync_intervals.png"
    fig.savefig(out1, dpi=140, facecolor="white")
    plt.close(fig)
    print(f"已保存 {out1}")

    # ---------- ② 每对相邻激光帧之间的 IMU 样本数 ----------
    idx = np.searchsorted(imu_t, laser_t)
    counts = np.diff(idx)  # 第 i 段 (t_i, t_{i+1}] 内的 IMU 条数
    imu_dt = np.diff(imu_t) * 1000
    imu_rate = (len(imu_t) - 1) / (imu_t[-1] - imu_t[0])
    print(f"\n[实验②] 相邻激光帧之间的 IMU 样本数（共 {len(counts)} 段）：")
    print(f"  均值 {counts.mean():.2f}，最小 {counts.min()}，最大 {counts.max()}")
    print(f"  IMU（按 bag 录制时间）：平均速率 {imu_rate:.0f} Hz，"
          f"但间隔中位 {np.median(imu_dt):.2f} ms、最大 {imu_dt.max():.2f} ms ——"
          f"间隔不均匀，是成批到达的")
    print(f"  结论：每个激光周期内有 {counts.min()}~{counts.max()} 个 IMU 样本，"
          f"做插值/积分绰绰有余——IMU 约快 {imu_rate/(1000/np.median(np.diff(laser_t)*1000)):.0f} 倍。")

    # ---------- ③ 挑战项：ICP yaw 角速度 vs IMU gyro z 互相关 ----------
    print(f"\n[实验③] ICP 帧间 yaw 与 IMU gyro z 的互相关（滞后 ±{args.lag_ms:.0f} ms）：")
    mid_t, yaw_rate_laser, ok_flags = [], [], []
    n_fail = 0
    for i in range(len(laser_t) - 1):
        dt_s = laser_t[i + 1] - laser_t[i]
        if not (0.015 < dt_s < 0.06) or len(laser_xy[i]) < 80 or len(laser_xy[i + 1]) < 80:
            n_fail += 1
            continue
        res = icp_yaw(laser_xy[i + 1], laser_xy[i])   # 后一帧 -> 前一帧
        if res is None or res[1] > 0.05:
            n_fail += 1
            continue
        yaw_deg = res[0]
        mid_t.append((laser_t[i] + laser_t[i + 1]) / 2)
        yaw_rate_laser.append(yaw_deg / dt_s)         # deg/s
        ok_flags.append(i)
    mid_t = np.array(mid_t); yaw_rate_laser = np.array(yaw_rate_laser)
    print(f"  有效 ICP 配对 {len(mid_t)}/{len(laser_t)-1} 对（剔除 {n_fail} 对：点太少/匹配发散）")

    if len(mid_t) < 50:
        print("  [放弃] 有效配对不足 50，互相关没有意义；原因如实记录，不编造结果。")
        return 0

    # IMU gyro z 换成 deg/s，并对每段激光间隔求平均（与 ICP 的 yaw/dt 同物理量）
    imu_gz_deg = np.degrees(imu_gz)
    yaw_rate_imu = []
    for i in ok_flags:
        m = (imu_t >= laser_t[i]) & (imu_t <= laser_t[i + 1])
        if m.sum() >= 2:
            # 梯形积分 / 区间长 = 平均角速度
            inc = np.trapezoid(imu_gz_deg[m], imu_t[m])
            yaw_rate_imu.append(inc / (imu_t[m][-1] - imu_t[m][0]))
        else:
            yaw_rate_imu.append(np.nan)
    yaw_rate_imu = np.array(yaw_rate_imu)
    valid = np.isfinite(yaw_rate_imu)
    print(f"  IMU 侧有效 {valid.sum()} 段；"
          f"ICP yaw 速率范围 [{np.min(yaw_rate_laser):.1f}, {np.max(yaw_rate_laser):.1f}] deg/s，"
          f"IMU 平均角速度范围 [{np.nanmin(yaw_rate_imu):.1f}, {np.nanmax(yaw_rate_imu):.1f}] deg/s")

    lags = np.arange(-args.lag_ms, args.lag_ms + args.lag_step / 2, args.lag_step)
    corrs = np.full(len(lags), np.nan)
    a = yaw_rate_laser[valid]
    mt = mid_t[valid]
    for j, lag in enumerate(lags):
        tq = mt + lag / 1000.0
        m2 = (tq >= imu_t[0]) & (tq <= imu_t[-1])
        if m2.sum() < 50:
            continue
        b = np.interp(tq[m2], imu_t, imu_gz_deg)
        aa = a[m2]
        if np.std(b) < 1e-9 or np.std(aa) < 1e-9:
            continue
        corrs[j] = np.corrcoef(aa, b)[0, 1]
    j_best = int(np.nanargmax(np.abs(corrs)))
    best_lag, best_r = lags[j_best], corrs[j_best]
    r0 = corrs[np.argmin(np.abs(lags))]
    print(f"  滞后扫描：{len(lags)} 个滞后点，步长 {args.lag_step:.0f} ms")
    print(f"  零滞后相关系数 r(0) = {r0:+.3f}")
    print(f"  最优滞后 = {best_lag:+.0f} ms，|r| 最大处相关系数 r = {best_r:+.3f}")
    if abs(best_r) < 0.3:
        print("  [注意] 相关系数弱（|r|<0.3）：说明这套简化 ICP 的 yaw 估计噪声大，"
              "或两传感器坐标轴方向约定相反/不一致。结论按弱相关如实记录，不夸大。")

    # 图：上=时间序列（IMU 平移最优滞后后），下=r(滞后)
    fig, ax = plt.subplots(2, 1, figsize=(13, 7.5), height_ratios=[2, 1])
    win = mt <= mt[0] + 40  # 前 40 s，免得线挤在一起
    ax[0].plot(mt[win] - mt[0], a[win], lw=1.0, color="#185FA5",
               label="激光 ICP 帧间 yaw 角速度")
    b_best = np.interp(mt + best_lag / 1000.0, imu_t, imu_gz_deg)
    ax[0].plot(mt[win] - mt[0], b_best[win], lw=1.0, color="#9A3412", alpha=0.85,
               label=f"IMU gyro z（平移 {best_lag:+.0f} ms 后）")
    ax[0].set_xlabel("时间 [s]"); ax[0].set_ylabel("yaw 角速度 [deg/s]")
    ax[0].set_title(f"两条独立传感器测到的「转多快」：最优滞后 {best_lag:+.0f} ms，r = {best_r:+.3f}")
    ax[0].legend(); ax[0].grid(alpha=0.25, lw=0.5)
    ax[1].plot(lags, corrs, lw=1.2, color="#44403C")
    ax[1].axvline(best_lag, color="#9A3412", ls="--", lw=1.5,
                  label=f"最优滞后 {best_lag:+.0f} ms")
    ax[1].axvline(0, color="#185FA5", ls=":", lw=1)
    ax[1].set_xlabel("IMU 相对激光的滞后 [ms]"); ax[1].set_ylabel("Pearson 相关系数 r")
    ax[1].set_title("互相关-滞后曲线：峰值位置就是两路时间戳的偏差")
    ax[1].legend(); ax[1].grid(alpha=0.25, lw=0.5)
    fig.tight_layout()
    out2 = FIGS / "time_sync_yawcorr.png"
    fig.savefig(out2, dpi=140, facecolor="white")
    plt.close(fig)
    print(f"已保存 {out2}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

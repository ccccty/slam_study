#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
07_export_web_data.py — 导出网站前端用的三份 JSON 数据

契约：docs/assets/data/CONTRACT.md（schema 以它为准，不许私改）

产出（docs/assets/data/）：
  1. scans_seq.json   扫描序列播放器：第 20–79 帧，极坐标转直角坐标，每帧 ≤600 点
  2. icp_trace.json   ICP 收敛动画：与 06_icp_minimal.py 同口径（第 27 帧、400 点、
                      真值 6.0°/26.9cm、σ=1cm、对应门限 1.0m），初值 0/8/16/22/24/28°，
                      记录每一轮的 R、t、rmse（iterations[0] 是未迭代的初值位姿）
  3. kitti_traj.json  KITTI 真值轨迹：00/05/08，取平移 x,z，每序列 ≤2000 点

用法：
    D:/slam/env/Scripts/python.exe D:/slam/tools/07_export_web_data.py
    D:/slam/env/Scripts/python.exe D:/slam/tools/07_export_web_data.py --selfcheck
"""
import sys
import json
import argparse
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

ROOT = Path(r"D:/slam")
NPZ = ROOT / "datasets/processed/cartographer_paper_deutsches_museum_scans.npz"
POSES_DIR = ROOT / "datasets/kitti_odometry/dataset/poses"
OUT_DIR = ROOT / "docs" / "assets" / "data"

FRAME_START, FRAME_END = 20, 79          # scans_seq：连续 60 帧
MAX_SCAN_POINTS = 600
ICP_INIT_DEGS = [0.0, 8.0, 16.0, 22.0, 24.0, 28.0]
KITTI_SEQS = ["00", "05", "08"]
MAX_TRAJ_POINTS = 2000


# ---------- 与 06_icp_minimal.py 同口径的 2D 刚体变换工具 ----------
def T_from(deg, tx=0.0, ty=0.0):
    th = np.radians(deg)
    T = np.eye(3)
    T[0, 0], T[0, 1] = np.cos(th), -np.sin(th)
    T[1, 0], T[1, 1] = np.sin(th), np.cos(th)
    T[0, 2], T[1, 2] = tx, ty
    return T


def apply(T, P):
    return (T[:2, :2] @ P.T).T + T[:2, 2]


def yaw_of(T):
    return np.degrees(np.arctan2(T[1, 0], T[0, 0]))


def pose_err(T_est, T_gt):
    dt = np.linalg.norm(T_est[:2, 2] - T_gt[:2, 2])
    da = abs((yaw_of(T_est) - yaw_of(T_gt) + 180) % 360 - 180)
    return dt, da


def kabsch(A, B):
    ca, cb = A.mean(0), B.mean(0)
    H = (A - ca).T @ (B - cb)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1.0, d]) @ U.T
    T = np.eye(3)
    T[:2, :2] = R
    T[:2, 2] = cb - R @ ca
    return T


def icp_trace(P, Q, T0, max_iter=60, tol=1e-7, max_corr_dist=1.0):
    """与 06 的 icp() 同参数同终止条件，但记录每一轮的位姿和残差。

    iterations[0] 是初值位姿本身（未迭代状态）；
    循环结束后追加最终位姿，使 iterations[-1] 是收敛/终止状态。
    rmse 沿用 06 的口径：全部最近邻距离平方的均值（mean(d^2)，不做门限裁剪）。
    """
    tree = cKDTree(Q)
    T = T0.copy()
    prev = np.inf
    iters = []

    def record(Tcur):
        Pw = apply(Tcur, P)
        d, _ = tree.query(Pw)
        iters.append((Tcur.copy(), float(np.mean(d ** 2))))

    record(T)                                   # iterations[0] = 初值位姿
    n_it = 0
    for it in range(max_iter):
        Pw = apply(T, P)
        d, idx = tree.query(Pw)
        m = d < max_corr_dist
        n_it = it + 1
        if m.sum() < 10:
            break
        T = kabsch(Pw[m], Q[idx[m]]) @ T
        err = float(np.mean(d ** 2))
        record(T)                               # 本轮更新后的位姿
        if abs(prev - err) < tol * max(1.0, err):
            break
        prev = err
    return T, n_it, iters


# ---------- 1. scans_seq.json ----------
def export_scans_seq(d):
    R, A, stamps = d["ranges"], d["angles"], d["stamps"]
    t0 = float(stamps[FRAME_START])
    frames = []
    strides = []
    for i in range(FRAME_START, FRAME_END + 1):
        r = R[i].astype(np.float64)
        m = np.isfinite(r) & (r > 0.3) & (r < 25)      # 过滤无效点，与 06 同口径
        xy = np.stack([r[m] * np.cos(A[m]), r[m] * np.sin(A[m])], axis=1)
        stride = max(1, int(np.ceil(len(xy) / MAX_SCAN_POINTS)))
        strides.append(stride)
        xy = xy[::stride]
        frames.append({
            "i": int(i),
            "t": round(float(stamps[i]) - t0, 4),
            "xy": np.round(xy, 4).tolist(),
        })
    out = {
        "schema": "scans_seq/1",
        "source": "cartographer_paper_deutsches_museum.bag / horizontal_laser_2d",
        "frame_start": FRAME_START,
        "frames": frames,
        "meta": {
            "point_count_raw": int(R.shape[1]),
            "downsample": int(max(strides)),
            "frames_total": int(R.shape[0]),
        },
    }
    npts = [len(f["xy"]) for f in frames]
    print(f"scans_seq: {len(frames)} 帧, 每帧点数 min={min(npts)} max={max(npts)}, "
          f"stride 集合={sorted(set(strides))}")
    return out


# ---------- 2. icp_trace.json ----------
def export_icp_trace(d):
    R, A = d["ranges"], d["angles"]
    valid_cnt = np.sum(np.isfinite(R), axis=1)
    f = int(np.argmax(valid_cnt))                       # 与 06 相同：取有效点最多的一帧
    r = R[f]
    m = np.isfinite(r) & (r > 0.3) & (r < 25)
    P = np.stack([r[m] * np.cos(A[m]), r[m] * np.sin(A[m])], axis=1)

    rng = np.random.default_rng(0)                      # 与 06 相同：先 choice 后 normal
    if len(P) > 400:
        P = P[rng.choice(len(P), 400, replace=False)]
    T_gt = T_from(6.0, 0.25, 0.10)
    Q = apply(T_gt, P) + rng.normal(0, 0.01, P.shape)
    print(f"icp_trace: 第 {f} 帧, {len(P)} 点, 真值 yaw={yaw_of(T_gt):.1f}° "
          f"t=({T_gt[0,2]:.2f},{T_gt[1,2]:.2f}) |t|={np.linalg.norm(T_gt[:2,2]):.4f} m")

    runs = []
    for dd in ICP_INIT_DEGS:
        T0 = T_from(dd) @ T_gt
        T_est, n_it, iters = icp_trace(P, Q, T0, max_iter=60, max_corr_dist=1.0)
        dt, da = pose_err(T_est, T_gt)
        converged = not (dt > 0.1 or da > 3)            # 与 06 的「跑飞了」判据一致
        runs.append({
            "label": f"{dd:g}°",
            "init_deg": float(dd),
            "converged": bool(converged),
            "iterations": [
                {
                    "R": np.round(Tk[:2, :2], 4).tolist(),
                    "t": np.round(Tk[:2, 2], 4).tolist(),
                    "rmse": round(err, 6),   # 契约 v1.1：rmse 例外保留 6 位小数
                } for Tk, err in iters
            ],
        })
        print(f"  init={dd:4.1f}°  iter={n_it:2d}  记录{len(iters)}轮  "
              f"平移误差={dt*100:6.2f}cm  角度误差={da:6.3f}°  "
              f"末轮rmse={iters[-1][1]:.6f}  converged={converged}")
    out = {
        "schema": "icp_trace/1",
        "source_frame": int(f),
        "truth": {"deg": 6.0, "t": [round(float(T_gt[0, 2]), 4), round(float(T_gt[1, 2]), 4)]},
        "target": np.round(Q, 4).tolist(),
        "source": np.round(P, 4).tolist(),
        "runs": runs,
    }
    return out


# ---------- 3. kitti_traj.json ----------
def export_kitti_traj():
    seqs = {}
    for s in KITTI_SEQS:
        path = POSES_DIR / f"{s}.txt"
        M = np.loadtxt(path).reshape(-1, 3, 4)
        xz = M[:, [0, 2], 3]                            # 平移列的 x、z（俯视）
        length = float(np.sum(np.linalg.norm(np.diff(xz, axis=0), axis=1)))
        n_raw = len(xz)
        if n_raw > MAX_TRAJ_POINTS:
            idx = np.unique(np.round(np.linspace(0, n_raw - 1, MAX_TRAJ_POINTS)).astype(int))
            xz_ds = xz[idx]
        else:
            xz_ds = xz
        seqs[s] = {
            "poses": np.round(xz_ds, 4).tolist(),
            "poses_raw": int(n_raw),
            "length_m": round(length, 4),
        }
        print(f"kitti_traj: 序列 {s}: {n_raw} -> {len(xz_ds)} 点, length_m={length:.1f}")
    return {"schema": "kitti_traj/1", "sequences": seqs}


def write_json(name, obj):
    path = OUT_DIR / name
    path.write_text(json.dumps(obj, ensure_ascii=False, separators=(",", ":")),
                    encoding="utf-8")
    print(f"已写入 {path} ({path.stat().st_size} 字节)")


# ---------- 自检：重新 json.load 三份文件，核对契约关键点 ----------
def selfcheck():
    ok = True

    def check(cond, msg):
        nonlocal ok
        print(f"  [{'OK' if cond else 'FAIL'}] {msg}")
        ok = ok and bool(cond)

    for name in ["scans_seq.json", "icp_trace.json", "kitti_traj.json"]:
        path = OUT_DIR / name
        raw = path.read_bytes()
        obj = json.loads(raw.decode("utf-8"))
        print(f"\n== {name}: {len(raw)} 字节, schema={obj.get('schema')!r} ==")
        if name == "scans_seq.json":
            fr = obj["frames"]
            max_pts = max(len(f["xy"]) for f in fr)
            print(f"  帧数={len(fr)}, 每帧最大点数={max_pts}, "
                  f"frame 范围={fr[0]['i']}..{fr[-1]['i']}, meta={obj['meta']}")
            check(len(raw) <= 2 * 1024 * 1024, "体积 ≤ 2MB")
            check(len(fr) == 60 and fr[0]["i"] == 20 and fr[-1]["i"] == 79, "60 帧连续 (20..79)")
            check(max_pts <= 600, "每帧 ≤ 600 点")
        elif name == "icp_trace.json":
            runs = obj["runs"]
            print(f"  source_frame={obj['source_frame']}, truth={obj['truth']}, "
                  f"target={len(obj['target'])} 点, source={len(obj['source'])} 点, 运行数={len(runs)}")
            by_label = {r["label"]: r for r in runs}
            for lb in ["0°", "8°", "16°", "22°", "24°", "28°"]:
                r = by_label[lb]
                print(f"    {lb}: converged={r['converged']}, 轮数={len(r['iterations'])}, "
                      f"末轮 rmse={r['iterations'][-1]['rmse']}, "
                      f"iterations[0].t={r['iterations'][0]['t']}")
            r0 = by_label["0°"]
            rmse0 = r0["iterations"][-1]["rmse"]
            check(r0["converged"] is True, "0° run converged=true")
            check(abs(rmse0 - 0.000125) < 1e-5,
                  f"0° 末轮 rmse={rmse0} 与文档值 0.000125 一致 (契约 v1.1 保留 6 位)")
            check(by_label["24°"]["converged"] is False, "24° run converged=false")
            check(by_label["28°"]["converged"] is False, "28° run converged=false")
            check(obj["source_frame"] == 27, "source_frame=27 与实验文档一致")
            check(len(obj["target"]) == 400 and len(obj["source"]) == 400, "target/source 各 400 点")
        else:
            seqs = obj["sequences"]
            for s, v in seqs.items():
                print(f"  序列 {s}: poses={len(v['poses'])} 点 (raw={v['poses_raw']}), "
                      f"length_m={v['length_m']}")
            check(sorted(seqs.keys()) == ["00", "05", "08"], "序列 00/05/08 齐全")
            check(all(len(v["poses"]) <= 2000 for v in seqs.values()), "每序列 ≤ 2000 点")
            check(all(v["length_m"] > 100 for v in seqs.values()), "length_m 合理 (>100m)")
    print("\n自检" + ("全部通过" if ok else "存在失败项"))
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description="导出网站数据 JSON（契约见 docs/assets/data/CONTRACT.md）")
    ap.add_argument("--selfcheck", action="store_true", help="只重新加载已生成的 JSON 做自检")
    args = ap.parse_args()
    if args.selfcheck:
        return selfcheck()

    if not NPZ.exists():
        print(f"[错误] 找不到 {NPZ}")
        return 1
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    d = np.load(NPZ)
    write_json("scans_seq.json", export_scans_seq(d))
    write_json("icp_trace.json", export_icp_trace(d))
    write_json("kitti_traj.json", export_kitti_traj())
    print()
    return selfcheck()


if __name__ == "__main__":
    sys.exit(main())

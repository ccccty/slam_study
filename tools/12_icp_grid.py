#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
12_icp_grid.py — ICP 收敛域地形图（契约 v3 第 7 节，schema icp_grid/1）

网格：13 个角度偏差 deg ∈ {-30,-25,…,30} × 6 个平移偏差 dx ∈ {0,0.1,…,0.5} = 78 格。
初值 T0 = T_from(deg, dx) @ T_gt —— 与 06/07 的「T_from(dd) @ T_gt」同族，
(0°, 0) 格退化为 T0 = T_gt，与 icp_trace.json 的 0° run 完全同口径。

点集生成与 icp_trace.json 同种子同流程（保证 (0°,0) 格可复算 icp_trace 的 0° run）：
  npz 中有效点最多的一帧（第 27 帧）→ 过滤 (0.3, 25) m → default_rng(0) 先
  choice 400 点后 normal(σ=1cm) 加噪；ICP 直接复用 07_export_web_data.icp_trace。

用法：
    D:/slam/env/Scripts/python.exe D:/slam/tools/12_icp_grid.py
输出：
    docs/assets/data/icp_grid.json   （≤1.5MB；超限则每格 iterations 只留前 60 条）
"""
import sys
import json
import importlib
from pathlib import Path

import numpy as np

sys.stdout.reconfigure(encoding="utf-8", errors="replace")
sys.path.insert(0, str(Path(__file__).parent))
m07 = importlib.import_module("07_export_web_data")   # 复用 T_from/apply/pose_err/icp_trace

NPZ = m07.NPZ
OUT_DIR = m07.OUT_DIR
TRACE_JSON = OUT_DIR / "icp_trace.json"
OUT_JSON = OUT_DIR / "icp_grid.json"

DEGS = list(range(-30, 31, 5))          # 13 个角度偏差
DXS = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5]    # 6 个沿 x 平移偏差 [m]
JSON_BUDGET = 1_500_000
MAX_ITER_KEEP = 60                      # 超限时每格 iterations 最多保留条数（契约 §7）


def make_points():
    """与 07_export_web_data.export_icp_trace 完全同口径的点集生成（同种子同顺序）。"""
    d = np.load(NPZ)
    R, A = d["ranges"], d["angles"]
    valid_cnt = np.sum(np.isfinite(R), axis=1)
    f = int(np.argmax(valid_cnt))
    r = R[f]
    m = np.isfinite(r) & (r > 0.3) & (r < 25)
    P = np.stack([r[m] * np.cos(A[m]), r[m] * np.sin(A[m])], axis=1)
    rng = np.random.default_rng(0)
    if len(P) > 400:
        P = P[rng.choice(len(P), 400, replace=False)]
    T_gt = m07.T_from(6.0, 0.25, 0.10)
    Q = m07.apply(T_gt, P) + rng.normal(0, 0.01, P.shape)
    return f, P, Q, T_gt


def run_cell(deg, dx, P, Q, T_gt):
    T0 = m07.T_from(deg, dx, 0.0) @ T_gt
    T_est, n_it, iters = m07.icp_trace(P, Q, T0, max_iter=60, max_corr_dist=1.0)
    dt, da = m07.pose_err(T_est, T_gt)
    converged = not (dt > 0.1 or da > 3)          # 与 06/07 的「跑飞了」判据一致
    return {"deg": deg, "dx": dx,
            "converged": bool(converged), "iters": int(n_it),
            "final_err": {"t_cm": round(dt * 100, 4), "deg": round(da, 4)},
            "final_rmse": round(float(iters[-1][1]), 6),
            "iterations": [{"R": np.round(Tk[:2, :2], 4).tolist(),
                            "t": np.round(Tk[:2, 2], 4).tolist(),
                            "rmse": round(err, 6)} for Tk, err in iters]}


def selfcheck():
    ok = True

    def check(cond, msg):
        nonlocal ok
        print(f"  [{'OK' if cond else 'FAIL'}] {msg}")
        ok = ok and bool(cond)

    print("\n========== 自检 ==========")
    raw = OUT_JSON.read_bytes()
    loaded = json.loads(raw.decode("utf-8"))
    print(f"== icp_grid.json: {len(raw)} 字节, schema={loaded.get('schema')!r} ==")
    check(len(raw) <= JSON_BUDGET, "体积 ≤ 1.5MB")
    check(loaded.get("schema") == "icp_grid/1", "schema == icp_grid/1")
    check(loaded["source_frame"] == 27, "source_frame == 27")
    check(loaded["grid"]["degs"] == DEGS and loaded["grid"]["dxs"] == DXS,
          "grid.degs/dxs 与契约一致")
    cells = loaded["cells"]
    check(len(cells) == 78, f"78 格齐全（实际 {len(cells)}）")
    combo = {(c["deg"], c["dx"]) for c in cells}
    check(combo == {(d, x) for d in DEGS for x in DXS}, "degs × dxs 全覆盖且无重复")
    check(len(loaded["target"]) == 400 and len(loaded["source"]) == 400,
          "target/source 各 400 点")
    check(all(len(c["iterations"]) >= 2 for c in cells),
          "每格 iterations ≥ 2 条（iterations[0]=初值位姿）")

    # 关键一致性：(0°, 0) 格必须复算 icp_trace.json 的 0° run
    trace = json.loads(TRACE_JSON.read_text(encoding="utf-8"))
    run0 = next(r for r in trace["runs"] if r["label"] == "0°")
    cell00 = next(c for c in cells if c["deg"] == 0 and c["dx"] == 0.0)
    it_cell, it_ref = cell00["iterations"], run0["iterations"]
    print("\n== (0°,0) 格 vs icp_trace 0° run ==")
    print(f"  网格: 轮数={len(it_cell)} 末轮 rmse={it_cell[-1]['rmse']} "
          f"converged={cell00['converged']}")
    print(f"  参照: 轮数={len(it_ref)} 末轮 rmse={it_ref[-1]['rmse']} "
          f"converged={run0['converged']}")
    check(len(it_cell) == len(it_ref), "iterations 轮数相同")
    check(it_cell[-1]["rmse"] == it_ref[-1]["rmse"], "末轮 rmse 相同")
    check(it_cell == it_ref, "逐轮 R/t/rmse 完全一致")
    check(cell00["converged"] == run0["converged"], "converged 判定一致")

    print("\n自检" + ("全部通过" if ok else "存在失败项"))
    return ok


def main():
    f, P, Q, T_gt = make_points()
    print(f"点集：第 {f} 帧，{len(P)} 点，真值 yaw=6.0° t=(0.25, 0.10)，σ=1cm"
          f"（与 icp_trace 同种子同流程）")
    if f != 27:
        print(f"[错误] 有效点最多的帧是第 {f} 帧，应为 27")
        return 1

    cells = []
    for deg in DEGS:
        row = []
        for dx in DXS:
            c = run_cell(deg, dx, P, Q, T_gt)
            cells.append(c)
            row.append("√" if c["converged"] else "×")
        print(f"  deg={deg:+4d}: " + " ".join(row) + "   (√=收敛, 列 dx=0.0..0.5)")

    def build(cell_list):
        return {"schema": "icp_grid/1", "source_frame": int(f),
                "truth": {"deg": 6.0, "t": [round(float(T_gt[0, 2]), 4),
                                            round(float(T_gt[1, 2]), 4)]},
                "target": np.round(Q, 4).tolist(), "source": np.round(P, 4).tolist(),
                "grid": {"degs": DEGS, "dxs": DXS}, "cells": cell_list}

    obj = build(cells)
    payload = json.dumps(obj, ensure_ascii=False, separators=(",", ":"))
    nbytes = len(payload.encode("utf-8"))
    if nbytes > JSON_BUDGET:
        # 契约 §7：超限时 iterations 只保留每格前 60 条
        trimmed = []
        for c in cells:
            c2 = dict(c)
            c2["iterations"] = c["iterations"][:MAX_ITER_KEEP]
            trimmed.append(c2)
        obj = build(trimmed)
        payload = json.dumps(obj, ensure_ascii=False, separators=(",", ":"))
        nbytes = len(payload.encode("utf-8"))
        print(f"超预算，iterations 已按契约截断到每格前 {MAX_ITER_KEEP} 条")
    OUT_JSON.write_text(payload, encoding="utf-8")
    print(f"\n已写入 {OUT_JSON}（{nbytes/1e6:.3f} MB，78 格）")

    nconv = sum(1 for c in cells if c["converged"])
    print(f"收敛 {nconv}/78 格")

    ok = selfcheck()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

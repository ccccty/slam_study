#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
11_export_segments.py — 多场景段导出：走廊段 / 旋转段 + 零初值对照链 + 段索引

做什么：
  1. 通读 bag 的 horizontal_laser_2d 全部帧（约 7 万帧）与 imu 话题陀螺仪 z 轴；
  2. 用两条确定规则各选一个 400 帧连续新段（不与 main 段 = bag 第 0–399 帧重叠）：
       corridor 走廊段：窗口内逐帧「PCA 主轴特征值比 λmax/λmin」的均值最大
                        （点云各向异性最强 → 长走廊/规则结构）；
       rotation 旋转段：窗口内相邻帧 yaw 变化（陀螺仪 z 积分在帧时刻取差）绝对值
                        累计最大，且不与 corridor 段重叠；
       候选窗口还要求每帧有效点 ≥ MIN_VALID（保证 ICP 可用）。
       平局取起始帧最小者（稳定排序），全程无随机数，可复现。
  3. 对每段（含 main）用 10_odometry 的链式里程计各跑两条链：
       主链 = 恒速初值（写 frames/map），对照链 = 零初值（写 alt.poses）。
  4. 按契约 v3 第 5/6 节导出：
       docs/assets/data/replay_index.json          （replay_index/1）
       docs/assets/data/slam_replay.json           （main 段升级为 slam_replay/2：
                                                    数据与现有 v1 一致，仅加 alt）
       docs/assets/data/slam_replay_corridor.json  （slam_replay/2）
       docs/assets/data/slam_replay_rotation.json  （slam_replay/2）

用法：
    D:/slam/env/Scripts/python.exe D:/slam/tools/11_export_segments.py
"""
import sys
import json
import time
import subprocess
import importlib
from pathlib import Path

import numpy as np

sys.stdout.reconfigure(encoding="utf-8", errors="replace")
sys.path.insert(0, str(Path(__file__).parent))
odo = importlib.import_module("10_odometry")      # 复用 ICP / 里程计链 / 自适应降采样参数
from slamlib import read_laser, beam_angles       # noqa: E402

ROOT = Path(r"D:/slam")
BAG = ROOT / "datasets/cartographer_2d/cartographer_paper_deutsches_museum.bag"
OUT_DIR = ROOT / "docs" / "assets" / "data"

SEG_LEN = 400          # 每段帧数（与 main 段一致）
MIN_VALID = 200        # 段内每帧最少有效点
MAIN_END = 400         # main 段 = bag 第 0..399 帧（即现有 npz 切片），新段从 400 起选
RMIN, RMAX = odo.RMIN, odo.RMAX
N_ICP = odo.N_ICP

SEGMENTS_META = {
    "main":     {"label": "大厅段", "file": "slam_replay.json",
                 "desc": "开阔大厅，混合运动", "tags": ["开阔", "混合"]},
    "corridor": {"label": "长廊段", "file": "slam_replay_corridor.json",
                 "desc": "长走廊，点云各向异性强", "tags": ["走廊", "各向异性"]},
    "rotation": {"label": "旋转段", "file": "slam_replay_rotation.json",
                 "desc": "连续旋转多，偏航变化大", "tags": ["旋转", "大转角"]},
}


# ---------- 1. 读 bag：全部激光帧 + IMU 陀螺仪 z ----------
def read_bag():
    from rosbags.typesys import Stores, get_typestore
    from rosbags.rosbag1 import Reader
    ts = get_typestore(Stores.ROS1_NOETIC)
    frames, stamps = [], []
    imu_t, imu_wz = [], []
    angles = None
    t0 = time.time()
    with Reader(str(BAG)) as reader:
        conns = [c for c in reader.connections
                 if c.topic in ("horizontal_laser_2d", "imu")]
        n_laser = sum(c.msgcount for c in conns if c.topic == "horizontal_laser_2d")
        print(f"读取 bag：horizontal_laser_2d 预期 {n_laser} 帧 + imu …", flush=True)
        last = 0
        for conn, t, raw in reader.messages(connections=conns):
            if conn.topic == "horizontal_laser_2d":
                msg = ts.deserialize_ros1(raw, conn.msgtype)
                r = read_laser(msg).astype(np.float32)
                frames.append(r)
                stamps.append(t / 1e9)
                if angles is None:
                    angles = beam_angles(msg, len(r))
                if len(frames) - last >= 20000:
                    last = len(frames)
                    print(f"  …已读 {last} 帧（{time.time()-t0:.0f}s）", flush=True)
            else:
                msg = ts.deserialize_ros1(raw, conn.msgtype)
                imu_t.append(t / 1e9)
                imu_wz.append(float(msg.angular_velocity.z))
    R = np.stack(frames)
    print(f"读完：{R.shape[0]} 帧 × {R.shape[1]} 束，imu {len(imu_t)} 条，"
          f"耗时 {time.time()-t0:.0f}s", flush=True)
    return R, angles, np.array(stamps), np.array(imu_t), np.array(imu_wz)


# ---------- 2. 选段指标 ----------
def frame_metrics(R, A):
    """逐帧：有效点数 + PCA 主轴特征值比（各向异性强度）。"""
    N = R.shape[0]
    nvalid = np.zeros(N, dtype=int)
    aniso = np.full(N, np.nan)
    for i in range(N):
        r = R[i]
        m = np.isfinite(r) & (r > RMIN) & (r < RMAX)
        nvalid[i] = int(m.sum())
        if nvalid[i] >= 50:
            p = np.stack([r[m] * np.cos(A[m]), r[m] * np.sin(A[m])], axis=1)
            p -= p.mean(axis=0)
            w = np.linalg.eigvalsh((p.T @ p) / len(p))
            aniso[i] = w[1] / max(w[0], 1e-12)
    return nvalid, aniso


def frame_dyaw(stamps, imu_t, imu_wz):
    """相邻激光帧之间的 yaw 变化 [rad]：陀螺仪 z 轴梯形积分，再在帧时刻取值取差。"""
    yaw = np.zeros(len(imu_t))
    np.cumsum(0.5 * (imu_wz[1:] + imu_wz[:-1]) * np.diff(imu_t), out=yaw[1:])
    yaw_at = np.interp(stamps, imu_t, yaw)
    return np.diff(yaw_at)          # dyaw[i] = 帧 i -> i+1 的 yaw 变化


def select_segments(nvalid, aniso, dyaw):
    """按 docstring 里的确定规则选 corridor / rotation 两段的起始帧。"""
    N = len(nvalid)
    bad = (nvalid < MIN_VALID).astype(int)
    cbad = np.concatenate([[0], np.cumsum(bad)])
    ca = np.concatenate([[0], np.cumsum(np.nan_to_num(aniso))])
    cy = np.concatenate([[0], np.cumsum(np.abs(dyaw))])

    starts = np.arange(MAIN_END, N - SEG_LEN + 1)
    ok = (cbad[starts + SEG_LEN] - cbad[starts]) == 0     # 窗口内每帧有效点都达标
    starts = starts[ok]
    anis_score = (ca[starts + SEG_LEN] - ca[starts]) / SEG_LEN
    yaw_score = cy[starts + SEG_LEN - 1] - cy[starts]     # 窗口内 399 个 |dyaw| 之和

    print("\n选段规则（可复现，无随机数，平局取起始帧最小者）：")
    print(f"  候选窗口：起始帧 ∈ [{MAIN_END}, {N - SEG_LEN}]，窗长 {SEG_LEN} 帧，"
          f"要求窗内每帧有效点 ≥ {MIN_VALID} → 合格窗口 {len(starts)} 个")
    print("  corridor：窗内逐帧 PCA 特征值比 λmax/λmin 的均值最大")
    print("  rotation：窗内相邻帧 |Δyaw|（陀螺仪 z 积分）累计最大，且不与 corridor 重叠")

    order = np.argsort(-anis_score, kind="stable")
    c_start = int(starts[order[0]])
    print("\n  corridor 候选 Top5（按各向异性均值）：")
    for k in range(5):
        s = int(starts[order[k]])
        print(f"    #{k+1} 帧 {s:>6}–{s+SEG_LEN-1:>6}  各向异性均值 "
              f"{anis_score[order[k]]:8.2f}  |Δyaw|累计 {np.degrees(yaw_score[order[k]]):7.1f}°")
    print(f"  → corridor 选定帧 {c_start}–{c_start+SEG_LEN-1}")

    keep = (starts + SEG_LEN <= c_start) | (starts >= c_start + SEG_LEN)
    rs, ry = starts[keep], yaw_score[keep]
    order2 = np.argsort(-ry, kind="stable")
    r_start = int(rs[order2[0]])
    print("\n  rotation 候选 Top5（按 |Δyaw| 累计，已排除与 corridor 重叠的窗口）：")
    for k in range(5):
        s = int(rs[order2[k]])
        print(f"    #{k+1} 帧 {s:>6}–{s+SEG_LEN-1:>6}  |Δyaw|累计 "
              f"{np.degrees(ry[order2[k]]):7.1f}°  各向异性均值 "
              f"{(ca[s+SEG_LEN]-ca[s])/SEG_LEN:8.2f}")
    print(f"  → rotation 选定帧 {r_start}–{r_start+SEG_LEN-1}")
    return c_start, r_start


# ---------- 3. 段切片（与 10 号 load_scans 同口径）----------
def slice_scans(R, A, S, s0):
    scans, scans_icp = [], []
    for i in range(s0, s0 + SEG_LEN):
        r = R[i]
        m = np.isfinite(r) & (r > RMIN) & (r < RMAX)
        pts = np.stack([r[m] * np.cos(A[m]), r[m] * np.sin(A[m])], axis=1)
        scans.append(pts)
        p = pts
        if len(p) > N_ICP:
            p = p[np.linspace(0, len(p) - 1, N_ICP).astype(int)]
        scans_icp.append(p)
    st = (S[s0:s0 + SEG_LEN] - S[s0]).astype(float)
    return scans, scans_icp, st


# ---------- 4. 导出 slam_replay/2（在 10 号 build 逻辑上加 alt）----------
def export_replay_v2(scans, stamps, odom_main, odom_alt, scan_n0=None, inc_n0=None):
    """契约 v3 第 6 节；自适应降采样策略与 10 号完全一致（同上限、同缩放、同下限）。

    scan_n0/inc_n0：起始降采样档位。main 段传入旧 v1 文件实际使用的档位
    （旧文件本身经过自适应缩减；从同一档位出发 + alt 仍在预算内 → 数据与 v1
    逐字节一致）。新段用默认 140/100，与 10 号一致。
    """
    n = len(scans)
    poses = odom_main["poses"]
    alt_poses = odom_alt["poses"]

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
            gw = odo.apply(W, pts)
            scan_out = [[odo.r4(x), odo.r4(y)] for x, y in gw]
            if len(gw) > inc_n:
                inc = gw[np.linspace(0, len(gw) - 1, inc_n).astype(int)]
            else:
                inc = gw
            map_pts += [[odo.r4(x), odo.r4(y)] for x, y in inc]
            frames.append({
                "i": i, "t": odo.r4(stamps[i]),
                "pose": [odo.r4(poses[i, 0]), odo.r4(poses[i, 1]), odo.r4(poses[i, 2])],
                "rmse": round(float(odom_main["rmse"][i]), 6),
                "n_inlier": int(odom_main["inliers"][i]),
                "scan": scan_out, "map_upto": len(map_pts)})
        return {"schema": "slam_replay/2",
                "source": "cartographer_paper_deutsches_museum.bag / horizontal_laser_2d",
                "frames": frames, "map": map_pts,
                "alt": {"label": "零初值对照链",
                        "poses": [[odo.r4(p[0]), odo.r4(p[1]), odo.r4(p[2])]
                                  for p in alt_poses]},
                "meta": {"frames_total": n,
                         "path_length_m": odo.r4(odo.path_length(poses)),
                         "odom_final": [odo.r4(poses[-1, 0]), odo.r4(poses[-1, 1]),
                                        odo.r4(poses[-1, 2])],
                         "method": "scan-to-scan ICP 链（无回环校正）"}}

    scan_n = scan_n0 if scan_n0 is not None else odo.SCAN_OUT
    inc_n = inc_n0 if inc_n0 is not None else odo.MAP_INC
    while True:
        obj = build(scan_n, inc_n)
        payload = json.dumps(obj, ensure_ascii=False, separators=(",", ":"))
        nbytes = len(payload.encode("utf-8"))
        if nbytes <= odo.JSON_BUDGET or (scan_n <= 40 and inc_n <= 30):
            return obj, payload, nbytes, scan_n, inc_n
        scale = 0.92 * odo.JSON_BUDGET / nbytes
        scan_n = max(40, int(scan_n * scale))
        inc_n = max(30, int(inc_n * scale))


def seg_stats(odom_main, odom_alt):
    poses, poses0 = odom_main["poses"], odom_alt["poses"]
    dev = np.hypot(poses[:, 0] - poses0[:, 0], poses[:, 1] - poses0[:, 1])
    return {"path_length": odo.path_length(poses),
            "rmse_med": float(np.median(odom_main["rmse"][1:])),
            "dev_max": float(dev.max()), "dev_argmax": int(np.argmax(dev)),
            "dev_end": float(dev[-1]),
            "iters_mean": float(odom_main["iters"][1:].mean()),
            "iters0_mean": float(odom_alt["iters"][1:].mean()),
            "final": [float(v) for v in poses[-1]]}


def load_old_main():
    """取 main 段的旧 v1 基线：优先 git HEAD（真正的 slam_replay/1）。"""
    try:
        out = subprocess.run(["git", "show", "HEAD:docs/assets/data/slam_replay.json"],
                             capture_output=True, cwd=ROOT, check=True).stdout
        obj = json.loads(out.decode("utf-8"))
        if obj.get("schema") == "slam_replay/1":
            print("旧 v1 基线取自 git HEAD（slam_replay/1）")
            return obj
        print(f"[警告] git HEAD 的 schema 是 {obj.get('schema')!r}，不是 v1")
    except Exception as e:
        print(f"[警告] 无法从 git HEAD 取 v1 基线：{e}")
    return json.loads((OUT_DIR / "slam_replay.json").read_text(encoding="utf-8"))


# ---------- 5. 自检 ----------
def selfcheck(old_main, stats):
    ok = True

    def check(cond, msg):
        nonlocal ok
        print(f"  [{'OK' if cond else 'FAIL'}] {msg}")
        ok = ok and bool(cond)

    print("\n========== 自检 ==========")
    loaded = {}
    for sid, meta in SEGMENTS_META.items():
        path = OUT_DIR / meta["file"]
        raw = path.read_bytes()
        obj = json.loads(raw.decode("utf-8"))
        loaded[sid] = obj
        print(f"\n== {meta['file']}: {len(raw)} 字节, schema={obj.get('schema')!r} ==")
        check(len(raw) <= 1_500_000, "体积 ≤ 1.5MB")
        check(obj.get("schema") == "slam_replay/2", "schema == slam_replay/2")
        check(len(obj["frames"]) == SEG_LEN, f"frames == {SEG_LEN}")
        check(len(obj["alt"]["poses"]) == len(obj["frames"]),
              "alt.poses 与 frames 等长")
        check(len(obj["map"]) > 0 and obj["frames"][-1]["map_upto"] == len(obj["map"]),
              "map 非空且 map_upto 收尾一致")
        check(obj["frames"][0]["rmse"] == 0 and obj["frames"][0]["n_inlier"] == 0,
              "第 0 帧 rmse=0 / n_inlier=0")
        check(obj["alt"]["label"] == "零初值对照链", "alt.label 正确")

    idx_raw = (OUT_DIR / "replay_index.json").read_bytes()
    idx = json.loads(idx_raw.decode("utf-8"))
    print(f"\n== replay_index.json: {len(idx_raw)} 字节, schema={idx.get('schema')!r} ==")
    check(idx.get("schema") == "replay_index/1", "schema == replay_index/1")
    ids = [s["id"] for s in idx["segments"]]
    check(ids == ["main", "corridor", "rotation"], f"三段齐全且 main 为默认段: {ids}")
    for s in idx["segments"]:
        check((OUT_DIR / s["file"]).exists(), f"段文件存在: {s['file']}")
        check(s["frames"] == SEG_LEN, f"{s['id']}.frames == {SEG_LEN}")

    print("\n== main 段 v2 vs 旧 v1 一致性 ==")
    new_main = loaded["main"]
    check(new_main["frames"] == old_main["frames"],
          "frames 完全一致（pose/rmse/n_inlier/scan/map_upto）")
    check(new_main["map"] == old_main["map"], "map 完全一致")
    check(new_main["meta"] == old_main["meta"], "meta 完全一致")
    expect_new = {"alt"} if "alt" not in old_main else set()
    check(set(new_main.keys()) - set(old_main.keys()) == expect_new,
          "相对 v1 仅新增 alt 字段")
    check(new_main.get("source") == old_main.get("source"), "source 字段一致")

    print("\n== 段关键统计 ==")
    for sid in ["main", "corridor", "rotation"]:
        st = stats[sid]
        print(f"  {sid:>9}: 路径长 {st['path_length']:.2f} m | rmse 中位 "
              f"{st['rmse_med']*100:.2f} cm | 恒速vs零初值最大偏差 {st['dev_max']:.3f} m "
              f"(第 {st['dev_argmax']} 帧, 末端 {st['dev_end']:.3f} m) | 平均迭代 "
              f"{st['iters_mean']:.1f} vs {st['iters0_mean']:.1f} 次/帧")

    print("\n自检" + ("全部通过" if ok else "存在失败项"))
    return ok


def main():
    t_start = time.time()
    if not BAG.exists():
        print(f"[错误] 找不到 {BAG}")
        return 1
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    # 先取旧 v1 基线（稍后要覆盖重写 slam_replay.json，一致性对照全靠它）。
    # 注意必须取 git HEAD 里的 slam_replay/1 —— 若上次运行已把文件写成 /2，
    # 从当前文件推导降采样档位会偏离 v1 实际档位（114/81），数据就不再一致。
    old_main = load_old_main()
    print(f"旧 slam_replay 基线: schema={old_main.get('schema')!r}, "
          f"frames={len(old_main['frames'])}, map={len(old_main['map'])}")

    R, A, S, imu_t, imu_wz = read_bag()
    dt = np.diff(S)
    print(f"激光时长 {S[-1]-S[0]:.1f} s，中位间隔 {np.median(dt)*1000:.1f} ms "
          f"({1/np.median(dt):.1f} Hz)")

    print("\n计算逐帧指标（有效点数 / PCA 各向异性 / 陀螺仪 Δyaw）…", flush=True)
    nvalid, aniso = frame_metrics(R, A)
    dyaw = frame_dyaw(S, imu_t, imu_wz)
    print(f"  有效点中位 {int(np.median(nvalid))}，最少 {int(nvalid.min())}；"
          f"|Δyaw| 全包中位 {np.degrees(np.median(np.abs(dyaw))):.3f}°/帧")

    c_start, r_start = select_segments(nvalid, aniso, dyaw)

    # 三段：main 沿用 npz（= bag 第 0–399 帧，与 10 号同一代码路径）
    scans_m, scans_icp_m, st_m = odo.load_scans()
    assert len(scans_m) == SEG_LEN
    segments = {"main": (scans_m, scans_icp_m, st_m)}
    for sid, s0 in [("corridor", c_start), ("rotation", r_start)]:
        segments[sid] = slice_scans(R, A, S, s0)
        print(f"\n{sid}: bag 帧 {s0}–{s0+SEG_LEN-1}，时长 {segments[sid][2][-1]:.2f} s，"
              f"t0={S[s0]:.2f}")

    # main 段沿用旧 v1 的实际降采样档位，保证「数据与 v1 一致仅加 alt」
    main_scan_n = max(len(f["scan"]) for f in old_main["frames"])
    main_inc_n = old_main["frames"][0]["map_upto"]
    print(f"\n旧 v1 降采样档位：scan {main_scan_n} 点/帧，地图增量 {main_inc_n} 点/帧"
          f"（main 段从该档位出发）")

    stats = {}
    for sid in ["main", "corridor", "rotation"]:
        scans, scans_icp, st = segments[sid]
        kw = {"scan_n0": main_scan_n, "inc_n0": main_inc_n} if sid == "main" else {}
        print(f"\n[{sid}] 主链（恒速初值）…", flush=True)
        t0 = time.time()
        om = odo.run_odometry(scans_icp, use_motion_model=True)
        print(f"  耗时 {time.time()-t0:.1f}s；最终位姿 x={om['poses'][-1,0]:.2f} "
              f"y={om['poses'][-1,1]:.2f} θ={np.degrees(om['poses'][-1,2]):.1f}°")
        print(f"[{sid}] 对照链（零初值）…", flush=True)
        t0 = time.time()
        o0 = odo.run_odometry(scans_icp, use_motion_model=False)
        print(f"  耗时 {time.time()-t0:.1f}s")
        stats[sid] = seg_stats(om, o0)
        obj, payload, nbytes, scan_n, inc_n = export_replay_v2(scans, st, om, o0, **kw)
        (OUT_DIR / SEGMENTS_META[sid]["file"]).write_text(payload, encoding="utf-8")
        print(f"  已写入 {SEGMENTS_META[sid]['file']}（{nbytes/1e6:.3f} MB，"
              f"scan {scan_n} 点/帧，地图增量 {inc_n} 点/帧）")

    index = {"schema": "replay_index/1",
             "segments": [{"id": sid, "label": SEGMENTS_META[sid]["label"],
                           "file": SEGMENTS_META[sid]["file"], "frames": SEG_LEN,
                           "desc": SEGMENTS_META[sid]["desc"],
                           "tags": SEGMENTS_META[sid]["tags"]}
                          for sid in ["main", "corridor", "rotation"]]}
    (OUT_DIR / "replay_index.json").write_text(
        json.dumps(index, ensure_ascii=False, separators=(",", ":")), encoding="utf-8")
    print("\n已写入 replay_index.json")

    ok = selfcheck(old_main, stats)
    print(f"\n总耗时 {time.time()-t_start:.1f}s")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

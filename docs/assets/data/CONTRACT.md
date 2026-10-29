# 网站数据契约 v1（docs/assets/data/）

本文件是「数据导出」（tools/07_export_web_data.py）与「网站前端」（docs/assets/app.js）之间的唯一契约。
双方都以此为准，不得私自改动格式；改动必须同步更新本文件版本号。

## 通用约定
- 单位：米（m）、弧度（rad）、秒（s，Unix 时间戳可相对归零）。
- 扫描点坐标系：激光雷达坐标系，x 前、y 左，已过滤无效点（NaN/Inf/超量程）。
- 所有 JSON UTF-8、无 BOM；浮点保留 4 位小数以控制体积。
- 前端加载顺序：先取正式文件 `assets/data/<name>.json`，404 时回退 `assets/data/__fixture__/<name>.json`，仍失败则显示「数据尚未生成」占位。

## 1. scans_seq.json —— 扫描序列播放器
```json
{
  "schema": "scans_seq/1",
  "source": "cartographer_paper_deutsches_museum.bag / horizontal_laser_2d",
  "frame_start": 20,
  "frames": [
    { "i": 20, "t": 12.34, "xy": [[0.42, 1.03], [0.43, 1.01]] }
  ],
  "meta": { "point_count_raw": 1079, "downsample": 2, "frames_total": 400 }
}
```
- 60 帧连续帧（建议第 20–79 帧，保证播放连续性），每帧降采样到 ≤600 点。
- 体积目标 ≤ 2 MB。

## 2. icp_trace.json —— ICP 收敛动画
```json
{
  "schema": "icp_trace/1",
  "source_frame": 27,
  "truth": { "deg": 6.0, "t": [0.269, 0.0] },
  "target": [[0.42, 1.03]],
  "source": [[0.45, 1.0]],
  "runs": [
    {
      "label": "0°", "init_deg": 0.0, "converged": true,
      "iterations": [
        { "R": [[1, 0], [0, 1]], "t": [0.0, 0.0], "rmse": 0.0123 }
      ]
    }
  ]
}
```
- target/source 为降采样后点集（与 docs/ICP误差-初值实验.md 同口径：第 27 帧、400 点、合成真值 6.0°/26.9cm、噪声 σ=1cm）。
- runs 覆盖初值 0/8/16/22/24/28°（含收敛与失败两类），iterations 记录每轮位姿与残差，前端用 R,t 自行变换 source 点做动画。

## 3. kitti_traj.json —— 真值轨迹对比器
```json
{
  "schema": "kitti_traj/1",
  "sequences": {
    "00": { "poses": [[12.3, 4.5]], "poses_raw": 4541, "length_m": 3724.2 }
  }
}
```
- 来源 datasets/kitti_odometry/dataset/poses/{00,05,08}.txt（每行 12 个数 = 3×4 位姿矩阵，取平移的 x、z 做俯视）。
- 每序列均匀降采样到 ≤2000 点，保留整体形状；length_m 用原始全量点计算。

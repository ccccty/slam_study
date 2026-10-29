# slam_study — 激光 SLAM 数据驱动学习

> **一句话：激光 SLAM 数据驱动学习——从读真实数据到建图，全程公开记录。**

🌐 **在线演示站**：https://ccccty.github.io/slam_study/ （每周的学习笔记、图表、交互演示都在这里）

这是一个面向初学者的公开学习仓库：不先啃论文、不先装 ROS，而是从一份真实的激光雷达数据（493 MB 的博物馆 rosbag）入手，用纯 Python 把「读数据 → 看扫描 → 算噪声 → 手写 ICP → 建图 → 算误差」这条链路一步步跑通，每周留下可复现的图、表和笔记。

---

## 12 周路线概览

| 阶段 | 周 | 主题 | 交付物 |
|---|---|---|---|
| 数据入门 | W1 | 读懂 rosbag：台账、单帧可视化、噪声统计 | 数据台账 + 统计图 |
| 数据入门 | W2 | 标定与外参：雷达-IMU 时间同步、坐标系 | 标定实验报告 |
| 前端 | W3 | 点云预处理：滤波、降采样、运动畸变 | 预处理脚本 + 对比图 |
| 前端 | W4 | 手写 ICP：从 npz 取相邻帧做配准 | 可运行的 ICP + 收敛曲线 |
| 前端 | W5 | 扫描匹配进阶：NDT / 相关匹配，读 KISS-ICP | 匹配方法对比 |
| 建图 | W6 | 2D 占据栅格地图：从扫描叠出第一张图 | 博物馆栅格地图 |
| 建图 | W7 | 位姿图与子图：Cartographer 思路复现 | 子图拼接结果 |
| 回环 | W8 | 回环检测：读 ScanContext，退化场景实验 | 回环实验记录 |
| 后端 | W9 | 图优化入门：位姿图优化、ATE/RPE 评估 | evo 评估报告 |
| 融合 | W10 | 紧耦合 LIO：读 FAST-LIO，IMU 进前端 | LIO 阅读笔记 + 实验 |
| 综合 | W11 | 实物雷达（YDLIDAR X3）数据采集与建图 | 自录数据集 + 建图结果 |
| 综合 | W12 | 总结与开源整理：全流程复盘 | 完整在线文档站 |

每一周的具体执行细节见在线演示站的周记。

---

## 目录结构速览

```
├── docs/                    所有公开产出：周记、实验报告、图（GitHub Pages 源）
│   ├── assets/data/         供网页用的小型导出数据（JSON/CSV，含数据契约）
│   ├── figs/                生成的图表
│   └── 本地环境指南.md       本地工作区的完整环境说明
├── tools/                   纯 Python 工具脚本（读 bag、画图、统计、导出）
│   └── slamlib.py           统一处理 MultiEchoLaserScan / LaserScan 差异
├── datasets/
│   ├── kitti_odometry/      KITTI 标定参数与真值轨迹（小 txt，已入库）
│   └── processed/           脚本导出的轻量中间结果（npz，已入库）
└── README.md                本文件
```

> 原始 bag、第三方源码、Python 虚拟环境等大体量/本机内容按 `.gitignore` 排除，不进仓库。

---

## 数据集来源与致谢

本仓库的一切分析都建立在以下公开数据集之上，感谢原作者与机构：

- **Cartographer 2D · Deutsches Museum bag**（主数据集）：Google Cartographer 论文配套的德国博物馆 2D 激光数据（水平+垂直激光 + 250 Hz IMU，约 31.9 分钟）。
  来源：https://google-cartographer-ros.readthedocs.io/ 
- **KITTI Odometry**：22 个序列的相机内参 + 雷达外参（calib.txt），以及 11 个序列的真值轨迹（poses/），用于学标定与算 ATE。
  来源：https://www.cvlibs.net/datasets/kitti/eval_odometry.php 
- **M2DGR**（上海交大 SJTU-ViSYS，多模态地面机器人数据集）：作为第二数据集的场景多样性补充。
  来源：https://github.com/SJTU-ViSYS/M2DGR 

数据集各自的许可证归原发布方所有；本仓库不重新分发原始 bag，只公开由脚本计算得到的统计结果与图表。

---

## 本地复现快速上手

本项目的完整本地环境（Python venv、依赖版本、脚本用法、坑位记录）都写在：

📖 **[docs/本地环境指南.md](docs/本地环境指南.md)**

三分钟版（Windows，需自备 Python 3.13 venv，依赖：numpy / scipy / matplotlib / pandas / rosbags / evo）：

```bash
# 环境自检
python tools/00_env_check.py
# 看清 bag 里有什么（数据台账）
python tools/01_bag_ledger.py
# 画一帧激光扫描
python tools/02_plot_scan.py
# 统计噪声与真实帧率
python tools/03_scan_stats.py
```

注意：主数据集的扫描消息是 `MultiEchoLaserScan`（字段为 `LaserEcho[]` 而非 `float32[]`），直接 `np.asarray(msg.ranges)` 会报错——读扫描请复用 `tools/slamlib.py` 里的函数。

---

## License

本仓库的原创文字、图表与脚本建议采用 **MIT**（代码）+ **CC BY 4.0**（文档与图表）双许可；正式 License 文件将在首次内容发布时补齐。所引用数据集的许可证以原发布方为准。

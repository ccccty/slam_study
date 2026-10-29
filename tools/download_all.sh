#!/usr/bin/env bash
# download_all.sh — 数据集一键下载（可重复执行，已下完的会跳过）
#
# 用法：  bash D:/slam/tools/download_all.sh
# 单独下某个：  bash D:/slam/tools/download_all.sh kitti
#
# 说明：curl -C - 支持断点续传，中断了重跑即可，不会从头开始。

set -u
ROOT="D:/slam"
DS="$ROOT/datasets"
ONLY="${1:-all}"

say() { printf '\n\033[1m== %s ==\033[0m\n' "$1"; }

# ---------- 1. Cartographer 2D（主数据集，493 MB，已验证可下）----------
if [ "$ONLY" = "all" ] || [ "$ONLY" = "cartographer" ]; then
  say "Cartographer 2D · Deutsches Museum（493 MB）"
  D="$DS/cartographer_2d"; mkdir -p "$D"
  U="https://storage.googleapis.com/cartographer-public-data/bags/backpack_2d/cartographer_paper_deutsches_museum.bag"
  if [ -f "$D/cartographer_paper_deutsches_museum.bag" ] && \
     [ "$(stat -c%s "$D/cartographer_paper_deutsches_museum.bag" 2>/dev/null || echo 0)" -ge 493377445 ]; then
    echo "  已完整，跳过"
  else
    curl -L -C - --retry 5 --retry-delay 3 -o "$D/cartographer_paper_deutsches_museum.bag" "$U"
  fi
fi

# ---------- 2. KITTI odometry 的标定与真值（各 0.6 / 1.3 MB，必下）----------
if [ "$ONLY" = "all" ] || [ "$ONLY" = "kitti" ]; then
  say "KITTI odometry · calib + poses（约 2 MB）"
  D="$DS/kitti_odometry"; mkdir -p "$D"; cd "$D" || exit 1
  for f in data_odometry_calib.zip data_odometry_poses.zip; do
    if [ -f "${f%.zip}" ] || [ -d "${f%.zip}" ]; then echo "  $f 已解压，跳过"; continue; fi
    curl -L -C - --retry 5 -O "https://s3.eu-central-1.amazonaws.com/avg-kitti/$f" && unzip -oq "$f" -d .
  done
  echo "  点云包 data_odometry_velodyne.zip 有 80 GB，按需手动下载："
  echo "  https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_velodyne.zip"
fi

# ---------- 3. Radish 经典 2D（小，但要转格式）----------
if [ "$ONLY" = "all" ] || [ "$ONLY" = "radish" ]; then
  say "Radish 2D 经典数据集"
  mkdir -p "$DS/radish"
  echo "  需要手动从网页挑：https://radish.sourceforge.net/"
  echo "  推荐：Intel Research Lab（最小）、MIT Killian Court（带回环）"
  echo "  注意：格式为 CARMEN .clf，不能直接 rosbag play，需用 tools/05_clf_to_npz.py 转换"
fi

# ---------- 4. M2DGR（多模态，含 IMU，用于 lidar-IMU 标定）----------
if [ "$ONLY" = "all" ] || [ "$ONLY" = "m2dgr" ]; then
  say "M2DGR（上海交大，多模态）"
  echo "  数据不提供直链，需从以下入口下载（选一个序列即可，不要全下）："
  echo "    项目主页  https://github.com/SJTU-ViSYS/M2DGR"
  echo "    百度网盘  见仓库 README 内的链接（国内速度快）"
  echo "  W2 做 lidar-IMU 标定时只需要 1 个含 IMU 的序列。"
fi

say "完成"
echo "校验环境： D:/slam/env/Scripts/python.exe D:/slam/tools/00_env_check.py"

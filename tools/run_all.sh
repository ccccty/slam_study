#!/usr/bin/env bash
# run_all.sh — 一键跑完 W1 的全部产出
#
# 用法：  bash D:/slam/tools/run_all.sh
#
# 跑完会在 docs/ 下得到：数据台账、扫描统计、两张图、一份点集 npz

set -e
PY="D:/slam/env/Scripts/python.exe"
cd "D:/slam" || exit 1

echo "=============================================="
echo " W1 数据台账 · 一键执行"
echo "=============================================="

echo ""
echo "[1/4] 数据台账（扫描整个 bag，约 1 分钟）"
"$PY" tools/01_bag_ledger.py 2>&1 | grep -v "已扫描"

echo ""
echo "[2/4] 画一帧扫描"
"$PY" tools/02_plot_scan.py

echo ""
echo "[3/4] 统计噪声与帧率"
"$PY" tools/03_scan_stats.py --max 800 2>&1 | grep -v "已处理"

echo ""
echo "[4/4] 导出点集"
"$PY" tools/04_extract_scans.py --max 400

echo ""
echo "=============================================="
echo " 产出清单"
echo "=============================================="
ls -lh docs/*.md docs/figs/*.png datasets/processed/*.npz 2>/dev/null | awk '{printf "  %-8s %s\n", $5, $9}'
echo ""
echo "W1 完成。把 docs/数据台账_*.md 存好，那是你第一份交付物。"
echo "下一步（W2）：在这个 bag 上做 lidar-IMU 标定 —— 它有 250 Hz 的 IMU。"

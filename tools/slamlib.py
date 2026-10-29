#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
slamlib.py — 三个脚本共用的小工具

存在的理由：真实数据不会按教科书的样子出现。
Deutsches Museum 这个 bag 用的就是 MultiEchoLaserScan（多回波）而不是 LaserScan，
字段是 LaserEcho[] 而不是 float32[]。这里统一掉，上层脚本就不用关心了。
"""
import numpy as np


def read_laser(msg):
    """把 LaserScan / MultiEchoLaserScan 统一成 float32 数组。

    MultiEchoLaserScan 每束可能有多个回波（穿玻璃、穿雨雾时会有），
    这里取第一个回波 —— 也就是最近的那个表面，与 LaserScan 语义一致。
    """
    r = getattr(msg, "ranges", None)
    if r is None:
        raise TypeError(f"不支持的消息类型 {type(msg).__name__}")
    if len(r) > 0 and hasattr(r[0], "echoes"):
        return np.array([e.echoes[0] if len(e.echoes) else np.nan for e in r], dtype=np.float32)
    return np.asarray(r, dtype=np.float32)


def beam_angles(msg, n):
    """生成每束激光的角度轴 [rad]。"""
    return np.linspace(msg.angle_min, msg.angle_max, n, dtype=np.float32)


def clean_mask(r, rmin, rmax):
    """有效点掩码：排除 nan / inf / 超出量程。"""
    return np.isfinite(r) & (r > rmin) & (r < rmax)


def polar_to_xy(r, ang):
    """极坐标 -> 平面点集。"""
    return np.stack([r * np.cos(ang), r * np.sin(ang)], axis=1)


def is_laser_msg(msgtype):
    """是否是某种激光扫描（含多回波变体）。"""
    return "LaserScan" in msgtype

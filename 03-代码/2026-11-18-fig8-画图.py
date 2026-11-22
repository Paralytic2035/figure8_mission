#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 figure8 跟踪 CSV 画成俯视图 + 误差图。
用法：python3 2026-11-18-fig8-画图.py <csv> [输出png]
"""
import sys, math
import numpy as np
from PIL import Image, ImageDraw, ImageFont

A, K = 0.5, 1.41          # 默认值；可用第 4 个参数覆盖 a
L_TARGET_LAPS = 3

def ref_curve(n=2000):
    t = np.linspace(0, 2 * math.pi, n, endpoint=False)
    d = 1.0 + np.sin(t) ** 2
    return A * np.cos(t) / d, K * A * np.sin(t) * np.cos(t) / d

def font(size):
    for p in ("/System/Library/Fonts/PingFang.ttc",
              "/System/Library/Fonts/Hiragino Sans GB.ttc",
              "/System/Library/Fonts/Supplemental/Songti.ttc",
              "/Library/Fonts/Arial Unicode.ttf"):
        try:
            return ImageFont.truetype(p, size)
        except Exception:
            continue
    return ImageFont.load_default()

def main():
    global A
    csv = sys.argv[1] if len(sys.argv) > 1 else "fig8.csv"
    out = sys.argv[2] if len(sys.argv) > 2 else "fig8_topview.png"
    if len(sys.argv) > 3:
        A = float(sys.argv[3])

    d = np.genfromtxt(csv, delimiter=",", names=True)
    px, py = d["px"], d["py"]
    e, t, sa, sc = d["e_cross"], d["t"], d["s_act"], d["s_cmd"]
    rx, ry = ref_curve()

    W, H = 1500, 980
    PV = 860                      # 左侧俯视图边长
    img = Image.new("RGB", (W, H), "white")
    dr = ImageDraw.Draw(img)
    f14, f18, f22 = font(14), font(18), font(22)

    lim = 0.9
    cx, cy = PV / 2, PV / 2 + 20
    sc_px = (PV - 110) / (2 * lim)     # 米 → 像素

    def P(x, y):
        return (cx + x * sc_px, cy - y * sc_px)

    # 网格
    for g in np.arange(-lim, lim + 1e-9, 0.1):
        col = (200, 200, 200) if abs(g * 10 - round(g * 10)) < 1e-6 and abs(round(g * 10) % 5) else (232, 232, 232)
        dr.line([P(g, -lim), P(g, lim)], fill=col)
        dr.line([P(-lim, g), P(lim, g)], fill=col)
    # 目标包络 ±0.5 / ±0.25
    _a, _b = P(-A, -A*K/2), P(A, A*K/2)
    dr.rectangle([min(_a[0], _b[0]), min(_a[1], _b[1]), max(_a[0], _b[0]), max(_a[1], _b[1])],
                 outline=(255, 190, 190))
    # 坐标轴
    dr.line([P(-lim, 0), P(lim, 0)], fill=(140, 140, 140))
    dr.line([P(0, -lim), P(0, lim)], fill=(140, 140, 140))
    dr.text(P(lim - 0.06, 0.03), "x [m]", fill=(80, 80, 80), font=f14)
    dr.text(P(0.02, lim - 0.06), "y [m]", fill=(80, 80, 80), font=f14)

    # 理论 ∞
    dr.line([P(x, y) for x, y in zip(rx, ry)], fill=(150, 150, 150), width=3)
    # 实际轨迹
    dr.line([P(x, y) for x, y in zip(px, py)], fill=(20, 90, 235), width=3)

    def dot(x, y, r, c):
        dr.ellipse([P(x, y)[0] - r, P(x, y)[1] - r, P(x, y)[0] + r, P(x, y)[1] + r], fill=c)

    dot(0, 0, 9, (20, 160, 60))       # 起飞/原点
    dot(px[0], py[0], 7, (0, 200, 200))
    dot(px[-1], py[-1], 8, (220, 40, 40))

    # 图例
    lg = ["俯视图（等比例）",
          "灰：理论 ∞   蓝：实际轨迹",
          "绿：原点/起飞点   青：入轨   红：结束"]
    for i, s in enumerate(lg):
        dr.text((30, 20 + i * 26), s, fill=(30, 30, 30), font=f18 if i == 0 else f14)

    # 右侧：误差随弧长
    x0, y0, w, h = PV + 40, 90, W - PV - 80, 360
    emax = 0.30
    dr.rectangle([x0, y0, x0 + w, y0 + h], outline=(120, 120, 120))
    dr.line([x0, y0 + h / 2, x0 + w, y0 + h / 2], fill=(180, 180, 180))
    for sgn in (-1, 1):
        yy = y0 + h / 2 - sgn * (0.15 / emax) * (h / 2)
        dr.line([x0, yy, x0 + w, yy], fill=(255, 170, 170))
    spt = (sa - sa[0]) / max(sa[-1] - sa[0], 1e-9)
    pts = [(x0 + u * w, y0 + h / 2 - (v / emax) * (h / 2)) for u, v in zip(spt, e)]
    dr.line(pts, fill=(20, 90, 235), width=2)
    dr.text((x0, y0 - 30), "横向偏差 e_cross ≤ ±0.15 m（红线）", fill=(30, 30, 30), font=f18)

    # 右侧下：进度门控
    y1 = y0 + h + 90
    dr.rectangle([x0, y1, x0 + w, y1 + h], outline=(120, 120, 120))
    lead = sc - sa
    top = max(0.2, float(np.abs(lead).max()) * 1.1)
    pts = [(x0 + u * w, y1 + h - (v / top) * h) for u, v in zip(spt, lead)]
    dr.line(pts, fill=(230, 120, 20), width=2)
    yy = y1 + h - (0.15 / top) * h
    dr.line([x0, yy, x0 + w, yy], fill=(255, 170, 170))
    dr.text((x0, y1 - 30), "门控量 s_cmd - s_act [m]（红线 = 0.15 冻结线）", fill=(30, 30, 30), font=f18)

    # 右下角数据
    rmse = float(np.sqrt((e ** 2).mean()))
    info = [
        "样本 %d，时长 %.1f s" % (len(t), t[-1]),
        "RMSE %.3f m   max|e| %.3f m" % (rmse, np.abs(e).max()),
        "x ∈ [%.2f, %.2f]   y ∈ [%.2f, %.2f]" % (px.min(), px.max(), py.min(), py.max()),
        "实际速度 均值 %.2f / 最大 %.2f m/s" % (
            np.hypot(np.diff(px), np.diff(py)).sum() / t[-1], np.hypot(np.diff(px), np.diff(py)).max() / np.diff(t).mean()),
        "闭合误差 %.3f m" % math.hypot(px[-1], py[-1]),
    ]
    for i, s in enumerate(info):
        dr.text((x0, y1 + h + 40 + i * 24), s, fill=(30, 30, 30), font=f14)

    img.save(out)
    print("saved:", out)
    print("RMSE %.3f  max|e| %.3f  x[%.2f,%.2f] y[%.2f,%.2f]" %
          (rmse, np.abs(e).max(), px.min(), px.max(), py.min(), py.max()))

if __name__ == "__main__":
    main()

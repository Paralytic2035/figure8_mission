#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""四轮仿真对照图：轨迹叠加 + e_cross + 幅值柱状。
用法：python3 2026-11-18-fig8-对比图.py <csv1> <csv2> <csv3> <csv4> [输出png]
"""
import sys, math
import numpy as np
from PIL import Image, ImageDraw, ImageFont

A, K = 0.5, 1.41
L = 3.04033
LAPS = 3

RUNS = [
    dict(tag="fig8-20261118-200705.csv", name="① pos_ff + progress", color=(220, 60, 60),  w=3),
    dict(tag="fig8-20261118-203321.csv", name="② pos + progress (k_gate 0.15)", color=(240, 150, 40), w=3),
    dict(tag="fig8-20261118-204524.csv", name="③ pos_ff + timed", color=(150, 80, 200), w=3),
    dict(tag="fig8-20261118-205822.csv", name="④ pos + progress (k_gate 0.45)", color=(20, 90, 235), w=5),
]

def font(size):
    for p in ("/System/Library/Fonts/PingFang.ttc",
              "/System/Library/Fonts/Hiragino Sans GB.ttc",
              "/System/Library/Fonts/Supplemental/Songti.ttc"):
        try:
            return ImageFont.truetype(p, size)
        except Exception:
            continue
    return ImageFont.load_default()

def ref_curve(n=2000):
    t = np.linspace(0, 2 * math.pi, n, endpoint=False)
    d = 1.0 + np.sin(t) ** 2
    return A * np.cos(t) / d, K * A * np.sin(t) * np.cos(t) / d

def main():
    base = sys.argv[1]
    out = sys.argv[-1] if sys.argv[-1].endswith(".png") else "fig8_compare.png"

    W, H = 1620, 1020
    img = Image.new("RGB", (W, H), "white")
    dr = ImageDraw.Draw(img)
    f12, f14, f16, f18, f24 = font(12), font(14), font(16), font(18), font(24)

    # ---------------- 左：轨迹叠加 ----------------
    lim = 0.80
    x0, y0, side = 40, 90, 880
    sc = (side - 60) / (2 * lim)
    cx, cy = x0 + side / 2, y0 + side / 2
    P = lambda x, y: (cx + x * sc, cy - y * sc)

    for g in np.arange(-lim, lim + 1e-9, 0.1):
        c = (205, 205, 205) if abs(round(g * 10) % 5) == 0 else (235, 235, 235)
        dr.line([P(g, -lim), P(g, lim)], fill=c)
        dr.line([P(-lim, g), P(lim, g)], fill=c)
    a, b = P(-0.5, -0.25), P(0.5, 0.25)
    dr.rectangle([min(a[0], b[0]), min(a[1], b[1]), max(a[0], b[0]), max(a[1], b[1])],
                 outline=(255, 190, 190), width=2)
    dr.line([P(-lim, 0), P(lim, 0)], fill=(150, 150, 150))
    dr.line([P(0, -lim), P(0, lim)], fill=(150, 150, 150))
    dr.text((x0 + 24, y0 - 42), "四轮轨迹叠加（等比例俯视图）", fill=(20, 20, 20), font=f24)
    dr.text((x0 + 24, y0 - 10), "灰：理论 ∞ / 粉框：目标包络 ±0.5 × ±0.25", fill=(90, 90, 90), font=f14)

    rx, ry = ref_curve()
    dr.line([P(x, y) for x, y in zip(rx, ry)], fill=(165, 165, 165), width=4)

    stats = []
    for r in RUNS:
        d = np.genfromtxt(base + "/" + r["tag"], delimiter=",", names=True)
        px, py = d["px"], d["py"]
        dr.line([P(x, y) for x, y in zip(px, py)], fill=r["color"], width=r["w"])
        stats.append((r, d, float(np.sqrt((d["e_cross"] ** 2).mean())),
                      float(np.abs(d["e_cross"]).max()),
                      (px.max() - px.min()) / 2, (py.max() - py.min()) / 2,
                      d["t"][-1]))

    # 起点标记
    for dx, dy in [(0, 0)]:
        px_, py_ = P(dx, dy)
        dr.ellipse([px_ - 8, py_ - 8, px_ + 8, py_ + 8], fill=(20, 160, 70))

    # 图例
    ly = y0 + side - 120
    for i, (r, *_rest) in enumerate(stats):
        yy = ly + i * 26
        dr.line([x0 + 24, yy + 8, x0 + 66, yy + 8], fill=r["color"], width=4)
        dr.text((x0 + 78, yy), r["name"], fill=(30, 30, 30), font=f14)

    # ---------------- 右上：e_cross ----------------
    gx, gy, gw, gh = 980, 140, 600, 315
    dr.text((gx, gy - 76), "横向偏差 e_cross（红线 = ±0.15 m 目标线）", fill=(20, 20, 20), font=f18)
    total = LAPS * L
    for sgn in (-1, 1):
        yy = gy + gh / 2 - sgn * (0.15 / 0.30) * (gh / 2)
        dr.line([gx, yy, gx + gw, yy], fill=(255, 170, 170), width=2)
    dr.rectangle([gx, gy, gx + gw, gy + gh], outline=(140, 140, 140))
    dr.line([gx, gy + gh / 2, gx + gw, gy + gh / 2], fill=(190, 190, 190))
    for i, (r, d, *_x) in enumerate(stats):
        u = np.clip(d["s_act"] / total, 0, 1)
        pts = [(gx + uu * gw, gy + gh / 2 - vv / 0.30 * (gh / 2)) for uu, vv in zip(u, d["e_cross"])]
        dr.line(pts, fill=r["color"], width=2)
    for frac in (0, 1 / 3, 2 / 3, 1):
        dr.line([gx + frac * gw, gy + gh, gx + frac * gw, gy + gh + 6], fill=(120, 120, 120))
        dr.text((gx + frac * gw - 10, gy + gh + 10), "%.0f 圈" % (frac * 3), fill=(90, 90, 90), font=f12)

    # ---------------- 右下：幅值柱状 ----------------
    bx0, by0, bw, bh = 980, 660, 600, 240
    dr.text((bx0, by0 - 46), "幅值对比（灰虚线 = 目标 0.500 / 0.249）", fill=(20, 20, 20), font=f18)
    dr.line([bx0, by0 + bh, bx0 + bw, by0 + bh], fill=(120, 120, 120))
    amax = 0.75
    for tgt, col in ((0.5, (200, 200, 200)), (0.249, (200, 200, 200))):
        yy = by0 + bh - tgt / amax * bh
        for xx in range(bx0, bx0 + bw, 12):
            dr.line([xx, yy, xx + 6, yy], fill=col)
    slot = bw / len(stats)
    for i, (r, d, rmse, emax, ax, ay, tt) in enumerate(stats):
        cxb = bx0 + slot * (i + 0.5)
        for j, (val, shade) in enumerate(((ax, 1.0), (ay, 0.55))):
            h = val / amax * bh
            xa = cxb - 34 + j * 38
            col = tuple(int(c * shade + 255 * (1 - shade)) for c in r["color"])
            dr.rectangle([xa, by0 + bh - h, xa + 30, by0 + bh], fill=col)
            dr.text((xa + 4, by0 + bh - h - 20), "%.2f" % val, fill=(40, 40, 40), font=f12)
        dr.text((cxb - 44, by0 + bh + 8), "第 %d 轮" % (i + 1), fill=(60, 60, 60), font=f14)

    # 右上角数字表
    tx = gx
    for i, (r, d, rmse, emax, ax, ay, tt) in enumerate(stats):
        dr.text((tx, 505 + i * 23),
                "%s   RMSE %.3f   max|e| %.3f" % (r["name"], rmse, emax),
                fill=r["color"] if i == 3 else (70, 70, 70), font=f12)

    img.save(out)
    print("saved:", out)
    for r, d, rmse, emax, ax, ay, tt in stats:
        print("%-36s RMSE %.3f  max %.3f  x %.3f  y %.3f  %.1fs" % (r["name"], rmse, emax, ax, ay, tt))

if __name__ == "__main__":
    main()

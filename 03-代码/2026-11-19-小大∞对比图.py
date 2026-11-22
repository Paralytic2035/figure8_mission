import math, numpy as np
from PIL import Image, ImageDraw, ImageFont

K = 1.41
def ref(a, n=3000):
    t = np.linspace(0, 2*math.pi, n, endpoint=False)
    d = 1.0 + np.sin(t)**2
    return a*np.cos(t)/d, K*a*np.sin(t)*np.cos(t)/d

def font(s):
    for p in ("/System/Library/Fonts/PingFang.ttc", "/System/Library/Fonts/Hiragino Sans GB.ttc"):
        try: return ImageFont.truetype(p, s)
        except Exception: pass
    return ImageFont.load_default()

W,H = 1500, 900
img = Image.new("RGB",(W,H),"white"); dr = ImageDraw.Draw(img)
f12,f14,f16,f20,f26 = font(12),font(14),font(16),font(20),font(26)

# 画布
lim = 0.95; x0,y0,side = 40,100,820
sc = (side-60)/(2*lim); cx,cy = x0+side/2, y0+side/2
P = lambda x,y:(cx+x*sc, cy-y*sc)
for g in np.arange(-lim, lim+1e-9, 0.1):
    c = (205,205,205) if abs(round(g*10)%5)==0 else (238,238,238)
    dr.line([P(g,-lim),P(g,lim)],fill=c); dr.line([P(-lim,g),P(lim,g)],fill=c)
dr.line([P(-lim,0),P(lim,0)],fill=(150,150,150)); dr.line([P(0,-lim),P(0,lim)],fill=(150,150,150))

def box(a, col):
    w = a; h = a*K/2
    aa,bb = P(-w,-h), P(w,h)
    dr.rectangle([min(aa[0],bb[0]),min(aa[1],bb[1]),max(aa[0],bb[0]),max(aa[1],bb[1])], outline=col, width=2)

box(0.75,(200,200,255)); box(0.45,(255,205,160))

sx, sy = ref(0.45); dr.line([P(x,y) for x,y in zip(sx,sy)], fill=(240,140,30), width=4)
bx, by = ref(0.75); dr.line([P(x,y) for x,y in zip(bx,by)], fill=(150,150,150), width=4)

d = np.genfromtxt("02-记录/fig8-20261119-212135.csv", delimiter=",", names=True)
dr.line([P(x,y) for x,y in zip(d["px"],d["py"])], fill=(20,90,235), width=2)

px_,py_ = P(0,0); dr.ellipse([px_-8,py_-8,px_+8,py_+8], fill=(20,160,70))

dr.text((x0+16,y0-72),"小 ∞ 与 大 ∞ 的实际尺寸对比（等比例）",fill=(20,20,20),font=f26)
dr.text((x0+16,y0-32),"橙：小 ∞(a=0.45)  灰：大 ∞(a=0.75)  蓝：大 ∞ 的仿真实际轨迹",fill=(90,90,90),font=f14)

# 标注
dr.text((P(0.18,-0.13)[0], P(0.18,-0.13)[1]), "小 ∞", fill=(240,140,30), font=f16)
dr.text((P(0.10,-0.55)[0], P(0.10,-0.55)[1]), "大 ∞", fill=(120,120,120), font=f16)
dr.text((P(-0.44,0.30)[0], P(-0.44,0.30)[1]), "0.9 × 0.45 m", fill=(240,140,30), font=f14)
dr.text((P(-0.74,0.42)[0], P(-0.74,0.42)[1]), "1.5 × 0.75 m", fill=(120,120,120), font=f14)

# 右侧表格
tx = 900
rows = [
    ("", "小 ∞", "大 ∞"),
    ("半长轴 a", "0.45 m", "0.75 m"),
    ("外框尺寸", "0.90 × 0.45 m", "1.50 × 0.75 m"),
    ("弧长 L", "2.736 m", "4.561 m"),
    ("最小曲率半径", "0.14 m", "0.24 m"),
    ("单圈用时", "13 s (prog)", "14 s (timed)"),
    ("3 圈用时", "—", "44 s"),
    ("实机表现", "√ 形状正确", "× 1/4 圈卡死*"),
]
dr.text((tx,120), "两种尺寸对照", fill=(20,20,20), font=f20)
y = 170
for i,r in enumerate(rows):
    col = (20,20,20) if i==0 else (50,50,50)
    dr.text((tx, y), r[0], fill=col, font=f14)
    dr.text((tx+190, y), r[1], fill=(200,110,20) if i else col, font=f14)
    dr.text((tx+360, y), r[2], fill=(20,90,235) if i else col, font=f14)
    if i==0: dr.line([tx,y+20,tx+470,y+20],fill=(150,150,150))
    y += 34

notes = [
    "* 卡死那次是 a=0.75 + progress 模式（门控死锁）；",
    "  换成 timed 后仿真 3 圈 44.2 s、max|e| 5.8 cm。",
    "",
    "为什么大的更容易卡：曲线越大越平缓，",
    "飞机越容易“追上胡萝卜”，于是门控把它钉住。",
    "小的更紧凑，飞机一直在追，反而不卡。",
]
y = 170 + 34*len(rows) + 40
for s in notes:
    dr.text((tx, y), s, fill=(90,90,90), font=f14); y += 26

img.save("02-记录/2026-11-19-小∞与big∞尺寸对比.png")
print("saved")

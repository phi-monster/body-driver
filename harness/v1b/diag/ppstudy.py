#!/usr/bin/env python3
# 不动的眼的焦距为什么一贯偏大 0.35%:两只手的点(按各自的真值相似变换搬到仿真)配进不动的眼的像素,按板解相机 —— 主点钉 (320,240) / 放开,比
import sys, os, math
import numpy as np
from scipy.optimize import least_squares
sys.argv_saved = list(sys.argv)
os.environ.setdefault("KDIR", sys.argv[2])
exec(open('/root/diag/fxstudy.py').read().split("# 真相机投第一只手的点")[0])
s1, Rg1, tg1 = F[1]['s'], F[1]['Rg'], F[1]['tg']
X1m = a1[:, 6:9]; uv1 = a1[:, 3:5]; X1s = s1 * (X1m @ Rg1.T) + tg1
z1 = X1s[:, 2]; on1 = np.abs(z1 - np.median(z1)) < 5 * 1.4826 * np.median(np.abs(z1 - np.median(z1)))
Xa = np.vstack([X0s[on], X1s[on1]]); uva = np.vstack([uv0[on], uv1[on1]])
print("点:第一只手 %d、第二只手 %d(桌面上)· 像素 u %.0f..%.0f v %.0f..%.0f" % (on.sum(), on1.sum(), uva[:, 0].min(), uva[:, 0].max(), uva[:, 1].min(), uva[:, 1].max()))
def projc(R, p, f, cx, cy, X):
    Xc = (X - p) @ R; z = -Xc[:, 2]
    return np.stack([f * Xc[:, 0] / z + cx, -f * Xc[:, 1] / z + cy], axis=1)
for nm, free_pp in (("主点钉 (320,240)", False), ("主点放开", True)):
    def res(x):
        R = rv(x[:3]) @ Rtrue; p = ptrue + x[3:6]; f = ftrue * math.exp(x[6])
        cx, cy = (320.0 + x[7], 240.0 + x[8]) if free_pp else (320.0, 240.0)
        return (projc(R, p, f, cx, cy, Xa) - uva).ravel()
    r = least_squares(res, np.zeros(9 if free_pp else 7), loss="soft_l1", f_scale=1.0)
    x = r.x; R = rv(x[:3]) @ Rtrue; p = ptrue + x[3:6]; f = ftrue * math.exp(x[6])
    e = np.hypot(*(projc(R, p, f, (320 + x[7]) if free_pp else 320, (240 + x[8]) if free_pp else 240, Xa) - uva).T)
    print("  %s:焦距 %.2f · 位置差 (%.1f, %.1f, %.1f) mm · 转差 %.3f° · 主点 %s · 残差中位 %.3f px" % (nm, f, *(1000 * (p - ptrue)), math.degrees(np.linalg.norm(logR(R.T @ Rtrue))),
          "(%.2f, %.2f)" % (320 + x[7], 240 + x[8]) if free_pp else "(320, 240)", np.median(e)))
# 真相机、主点挪半个像素的残差
for cx, cy in ((320.0, 240.0), (319.5, 239.5), (320.5, 240.5)):
    e = np.hypot(*(projc(Rtrue, ptrue, ftrue, cx, cy, Xa) - uva).T)
    print("  真相机 主点 (%.1f, %.1f):残差中位 %.3f px、平均 (%.3f, %.3f)" % (cx, cy, np.median(e), *(projc(Rtrue, ptrue, ftrue, cx, cy, Xa) - uva).mean(axis=0)))

#!/usr/bin/env python3
# 不动的眼的位姿偏在哪:第一只手三角出、配进不动的眼的点 vs 仿真真值(真相机:pos (0,-0.41,1.308)、绕 x 转 30°、f 288.12)。真值只打分。
import sys, os, math
import numpy as np
from scipy.optimize import least_squares
sys.argv_saved = list(sys.argv)
exec(open('/root/diag/alignstudy.py').read().split("for RUN in sys.argv[1:]:")[0])
RUN = sys.argv_saved[1]; AD = sys.argv_saved[2]
def fits(RUN, KDIR):
    rows = [l for l in open(os.path.join(RUN, "look", "sweep.txt")) if "||" in l]
    F = {}
    for arm in range(2):
        L = open(os.path.join(KDIR, "kinem_arm%d.txt" % arm)).read().split("\n")
        q0 = np.array([float(x) for x in L[1].split()[1:]])
        W, Pp = [], []
        for l in L[2:]:
            if l.startswith("axis"):
                v = [float(x) for x in l.split()[2:]]; W.append(v[:3]); Pp.append(v[3:])
        W = np.array(W); Pp = np.array(Pp)
        Rf, Tf, Tp, Rt = [], [], [], []
        for l in rows:
            left, right = l.split("||", 1); parts = left.strip().split("|"); h = parts[0].split()
            if int(h[1]) != arm: continue
            ee = [float(x) for x in right.split()]
            if len(ee) != 7: continue
            q = np.array([float(x) for x in parts[1 + arm].split()])
            R, t = kinem_fk(W, Pp, q0, q)
            Rf.append(R); Tf.append(t); Tp.append(np.array(ee[:3])); Rt.append(qR(np.array(ee[3:])))
        Rf = np.array(Rf); Tf = np.array(Tf); Tp = np.array(Tp); Rt = np.array(Rt)
        idx = np.arange(len(Tf))
        def res2(x):
            Rg = rv(x[0:3]); Rx = rv(x[3:6]); s = x[6]; tg = x[7:10]; tx = x[10:13]
            pe = (s * (Tf[idx] @ Rg.T) + tg) - (Tp[idx] + np.einsum('nab,b->na', Rt[idx], tx))
            re = np.array([logR((Rt[i] @ Rx).T @ (Rg @ Rf[i])) for i in idx])
            return np.concatenate([pe.ravel(), 0.05 * re.ravel()])
        best = None; rng = np.random.default_rng(0)
        for k in range(12):
            x0 = np.concatenate([rng.normal(size=3) * 1.5, rng.normal(size=3) * 1.5, [0.05], np.zeros(6)])
            r = least_squares(res2, x0, method="lm", max_nfev=4000)
            if best is None or r.cost < best.cost: best = r
        x = best.x
        F[arm] = dict(s=x[6], Rg=rv(x[0:3]), tg=x[7:10], Tf=Tf, Tp=Tp)
    return F
F = fits(RUN, os.environ.get("KDIR", os.path.join(RUN, "look")))
s0, Rg0, tg0 = F[0]['s'], F[0]['Rg'], F[0]['tg']
s1, Rg1, tg1 = F[1]['s'], F[1]['Rg'], F[1]['tg']
c30, s30 = math.cos(math.radians(30)), math.sin(math.radians(30))
Rtrue = np.array([[1, 0, 0], [0, c30, -s30], [0, s30, c30]]); ptrue = np.array([0.0, -0.41, 1.308]); ftrue = 10.0 / 22.212 * 640
def proj(R, p, f, X):   # R: 相机 → 这个系;-z 朝前;v 向下
    Xc = (X - p) @ R
    z = -Xc[:, 2]
    return np.stack([f * Xc[:, 0] / z + 320.0, -f * Xc[:, 1] / z + 240.0], axis=1)
def ray(R, p, f, uv):
    d = np.stack([(uv[:, 0] - 320.0) / f, -(uv[:, 1] - 240.0) / f, -np.ones(len(uv))], axis=1) @ R.T
    return d / np.linalg.norm(d, axis=1)[:, None]
O = np.loadtxt(os.path.join(AD, "cam_obs.txt"))
fe = open(os.path.join(AD, "fixed_eye.txt")).read().split()
fd = float(fe[1]); pd = np.array([float(v) for v in fe[fe.index("pos") + 1: fe.index("pos") + 4]]); Rd = np.array([float(v) for v in fe[fe.index("R") + 1: fe.index("R") + 10]]).reshape(3, 3)
a0 = O[O[:, 0] == 0]; a1 = O[O[:, 0] == 1]
print("配进不动的眼:第一只手 %d 笔、第二只手 %d 笔" % (len(a0), len(a1)))
uv0 = a0[:, 3:5]; X0m = a0[:, 6:9]
X0s = s0 * (X0m @ Rg0.T) + tg0          # 第一只手的点按它的相似变换搬到仿真(米)
# 真桌面高:第一只手的点在仿真里的 z 的中位(按离中位 3 倍中位差内的)
z = X0s[:, 2]; zt = np.median(z); md = np.median(np.abs(z - zt)); on = np.abs(z - zt) < 5 * 1.4826 * md
print("第一只手的点在仿真里的高:中位 %.4f m、离散(1.4826×中位差)%.2f mm、算在桌面上的 %d / %d" % (zt, 1000 * 1.4826 * md, on.sum(), len(z)))
A = np.c_[X0s[on, 0], X0s[on, 1], np.ones(on.sum())]; cz = np.linalg.lstsq(A, z[on], rcond=None)[0]
print("   桌面点的高随位置:dz/dx %.4f、dz/dy %.4f(= 倾 %.3f° / %.3f°)· x 范围 %.3f..%.3f、y 范围 %.3f..%.3f m" % (cz[0], cz[1], math.degrees(math.atan(cz[0])), math.degrees(math.atan(cz[1])), X0s[on, 0].min(), X0s[on, 0].max(), X0s[on, 1].min(), X0s[on, 1].max()))
# 真相机投第一只手的点
ut = proj(Rtrue, ptrue, ftrue, X0s); rt = uv0 - ut
# 驱动的相机(第一只手系里)投
ud = proj(Rd, pd, fd, X0m); rd = uv0 - ud
print("第一只手的点投进不动的眼:按真相机 残差中位 %.2f px、平均 (%.2f, %.2f) px · 按驱动解的相机 中位 %.2f px、平均 (%.2f, %.2f)" %
      (np.median(np.linalg.norm(rt, axis=1)), *rt.mean(axis=0), np.median(np.linalg.norm(rd, axis=1)), *rd.mean(axis=0)))
# 按真相机的残差随像素位置怎么变(一阶):r = a + B (uv - 中心)
C = np.c_[np.ones(len(uv0)), (uv0[:, 0] - 320) / 100, (uv0[:, 1] - 240) / 100]
for k, nm in ((0, "u"), (1, "v")):
    c = np.linalg.lstsq(C[on], rt[on, k], rcond=None)[0]
    print("   按真相机的残差 %s = %.2f + %.3f·(u-320)/100 + %.3f·(v-240)/100 px" % (nm, *c))
print("   第一只手的点在不动的眼里的像素范围:u %.0f..%.0f、v %.0f..%.0f(画幅 640×480)" % (uv0[:, 0].min(), uv0[:, 0].max(), uv0[:, 1].min(), uv0[:, 1].max()))
# 真相机的视线交真桌面(z = zt)= 这一笔配点指的真位置;和第一只手三角出来的比(仿真米)
d = ray(Rtrue, ptrue, ftrue, uv0); lam = (zt - ptrue[2]) / d[:, 2]; Xt = ptrue + lam[:, None] * d
e = (X0s - Xt) * 1000
print("第一只手的点 − 真视线交真桌面(mm,只算桌面上的):平均 (%.2f, %.2f, %.2f)、中位 |e| %.2f" % (*e[on].mean(axis=0), np.median(np.linalg.norm(e[on], axis=1))))
# 按离第一只手眼(扫描起点)的远近分
eye0 = tg0   # 模型原点 = 扫描起点那一格的眼
dist = np.linalg.norm(X0s - eye0, axis=1)
for lo, hi in ((0, 0.3), (0.3, 0.4), (0.4, 0.5), (0.5, 0.7), (0.7, 2)):
    m = on & (dist >= lo) & (dist < hi)
    if m.sum() > 5:
        print("   离第一只手起点的眼 %.1f–%.1f m:%4d 点 · 平均误差 (%.2f, %.2f, %.2f) mm" % (lo, hi, m.sum(), *e[m].mean(axis=0)))
# 用第一只手的点(桌面上的)按像素残差解不动的眼的位姿 + 焦距(同驱动:抗野点)⇒ 和真值比;再用"真视线交真桌面"那批点解一次(对照)
def pnp(X, uv, R0, p0, f0):
    def res(x):
        R = rv(x[:3]) @ R0; p = p0 + x[3:6]; f = f0 * math.exp(x[6])
        return (proj(R, p, f, X) - uv).ravel()
    r = least_squares(res, np.zeros(7), loss="soft_l1", f_scale=1.0)
    x = r.x; return rv(x[:3]) @ R0, p0 + x[3:6], f0 * math.exp(x[6])
Rp, pp, fp = pnp(X0s[on], uv0[on], Rtrue, ptrue, ftrue)
print("按第一只手的点解不动的眼(仿真系):焦距 %.1f · 位置差 (%.1f, %.1f, %.1f) mm · 转差 %.3f°" % (fp, *(1000 * (pp - ptrue)), math.degrees(np.linalg.norm(logR(Rp.T @ Rtrue)))))
Rq, pq, fq = pnp(Xt[on], uv0[on], Rtrue @ rv(np.array([0.01, 0.0, 0.0])), ptrue + 0.01, ftrue * 1.01)
print("   对照:拿真视线交真桌面的点解:焦距 %.1f · 位置差 (%.1f, %.1f, %.1f) mm" % (fq, *(1000 * (pq - ptrue))))
# 驱动解的不动的眼搬到仿真
pdw = s0 * (Rg0 @ pd) + tg0; Rdw = Rg0 @ Rd
print("驱动解的不动的眼(搬到仿真):焦距 %.1f · 位置差 (%.1f, %.1f, %.1f) mm · 转差 %.3f°" % (fd, *(1000 * (pdw - ptrue)), math.degrees(np.linalg.norm(logR(Rdw.T @ Rtrue)))))

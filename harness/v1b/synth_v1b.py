# 合成考试:一条已知的 6 关节胳膊(像 x5:肩两根、肘一根、腕三根,手上的眼在腕子前面几厘米),
# 造"各停之间相机转了多少、往哪挪"的读数(带噪声),看 fit_arm 能不能只凭关节读数把考试停的眼算回来。
import sys, math, numpy as np
import v1b_fit as V

rng = np.random.default_rng(7)
# 世界系(底座,z 朝上)里的 6 根轴:方向 + 过的一点(米)
axes_w = [((0, 0, 1), (0, 0, 0.05)), ((0, 1, 0), (0, 0, 0.12)), ((0, 1, 0), (0.25, 0, 0.12)),
          ((0, 1, 0), (0.45, 0, 0.16)), ((0, 0, 1), (0.50, 0, 0.16)), ((1, 0, 0), (0.55, 0, 0.16))]
# 参照停时眼的位姿:在 (0.60, 0, 0.22),朝前下方看(光轴 z 朝前下,x 朝右,y 朝下 —— 相机约定)
c0 = np.array([0.60, 0.0, 0.22])
fwd = np.array([0.5, 0, -0.87]); fwd /= np.linalg.norm(fwd)
right = np.array([0, -1.0, 0]); down = np.cross(fwd, right)
R0 = np.c_[right, down, fwd]            # 眼系 → 世界
W_true = np.array([R0.T @ np.array(w, float) for w, p in axes_w])
P_true = np.array([R0.T @ (np.array(p, float) - c0) for w, p in axes_w])

def run(range_deg, nstops, rot_noise_deg, px_noise, f=397.0, depth=0.35, inl=400, seed=0):
    r = np.random.default_rng(seed)
    n = 6
    dQ = r.uniform(-1, 1, (nstops, n)) * math.radians(range_deg); dQ[0] = 0
    poses = [V.fk(W_true, P_true, dq) for dq in dQ]
    train = [i for i in range(nstops) if i % 3 != 2]; test = [i for i in range(nstops) if i % 3 == 2]
    meas = []
    for a, i in enumerate(train):
        for j in train[a + 1:]:
            Ri, ti = poses[i]; Rj, tj = poses[j]
            R = Rj.T @ Ri; t = Rj.T @ (ti - tj)
            base = np.linalg.norm(t)
            par_rad = base / depth
            w = r.normal(size=3); w /= np.linalg.norm(w)
            Rn = V.rot(w, math.radians(r.normal() * rot_noise_deg)) @ R
            # 平移方向的误差 ≈ 像素噪声 / 焦距 / 视差 / √内点数(弧度)
            sd = (px_noise / f) / max(par_rad, 1e-4) / math.sqrt(inl)
            d = t / base + r.normal(size=3) * sd
            d /= np.linalg.norm(d)
            meas.append(dict(i=i, j=j, R=Rn, t=d, ninl=inl, par=math.degrees(par_rad)))
    # 世界真值:眼的世界位姿 = G·T(米,倍数 1);手(末端)= 眼 · X⁻¹
    Rx = V.rot(np.array([0.3, -0.2, 1.0]), 0.7); tx = np.array([0.0, -0.085, 0.051])
    Pw = []
    for Rk, tk in poses:
        Rc = R0 @ Rk; pc = R0 @ tk + c0
        Ree = Rc @ Rx.T; pee = pc - Ree @ tx
        Pw.append((pee, Ree))
    print("\n##### 合成:关节范围 ±%g°、%d 停、转动噪声 %.3f°、像素噪声 %.2f px" % (range_deg, nstops, rot_noise_deg, px_noise))
    return V.fit_arm(dQ, meas, train, test, Pw, n, r)

for rg in (10, 20, 30):
    res = run(rg, 30, 0.02, 0.5)
    print("=> 考试停眼的位置误差 中位 %.2f mm / 最大 %.2f mm" % (res["test_med_mm"], res["test_max_mm"]))

//! **自图 —— 每个像素一个"我一动它跑多少"(稠密图像雅可比)。**
//!
//! 照 `DIJE`(arXiv 2507.00446,东大 JSK)那篇实现,四个部件一个不缺。
//! 我第一版只抄了"每像素一个雅可比"这个**形**,漏掉了让它成立的三个**机制**,
//! 于是诊断链条永远死在最后一关(`解释得掉的 0`)。这一版把四个都补上。
//!
//! # 判据只有一句
//!
//! **我一动,它同比例地跟着动 ⇒ 它是我;时灵时不灵、或者我没动它自己动 ⇒ 它是世界。**
//! 这句话里没有相机内参、没有运动学、没有标记点。换身体换相机,图会自己重新长。
//!
//! # 四个部件(缺一个都不成立)
//!
//! ① **卡尔曼滤波**(式 9/10/11):状态是每像素的雅可比 `j ∈ R^{2Nj}`,
//!    协方差**近似成对角**(否则每像素 O(Nj²),论文算过:320×240、5 关节就要 61 MB)。
//!    再进一步:横向和纵向那两半共用同一组方差(式 6),于是只存 `p ∈ R^{Nj}`。
//!
//! ② **沿【预测】光流搬**(式 14,论文明说这是它自己的核心贡献):
//!    像素钉在画面上不动,而身体是从上面**滑过去**的 —— 同一个像素这一秒是小臂、
//!    下一秒是手腕、再下一秒是地板,它的雅可比根本不是一个固定的东西。
//!    所以每一帧要把整张图**搬到"这一点上的东西上一帧在哪儿"**。
//!    🔴 搬的时候必须用 **`J·q̇`(预测流)**,不能用实测光流 ——
//!       实测光流里含着**别人**的运动,用它搬会把外部运动"漏"进我的估计里
//!       (论文图 4 就是这个对照:用实测流,背景有人走过,自体标签当场漏到背景上)。
//!
//! ③ **k-means 聚类 + 逐簇时间一致性**(算法 1/2):把每像素的雅可比当成一个向量去聚类,
//!    **簇心在时间上稳不稳定**就是"是不是我"的判据。我的身体和我的命令之间关系固定;
//!    别人走过、东西被碰倒,那关系每帧都在变、当场露馅。
//!
//! ④ **被跟踪的点是【跟着身体跑的】**(式 15):`p_self ← p_self + 该处的光流`。
//!    论文原话:*用 p 而不是 x,是为了强调这是随实际运动变化的坐标,
//!    而不是每个像素的固定坐标(拉格朗日 vs 欧拉参考系)*。
//!    这一条一次性替掉模板匹配 —— 不需要模板、没有孔径问题。
//!
//! # 和论文的差别(照实记)
//!
//! * 光流:论文用 Farnebäck,这里用金字塔 Horn–Schunck(见 `flow.rs`)。作用相同。
//! * 论文**不用深度**(它把深度列为自己的局限);这里也先不用,与它对齐。
//! * 论文在 320×240 上跑;这里同样把图**缩一半**再算。

/// 一张稠密图像雅可比 + 它的自我识别。
pub struct 自图 {
    w: usize,
    h: usize,
    nj: usize,
    /// 每像素的雅可比,长度 `w*h*2*nj`:像素 i 的 `[jx(0..nj), jy(0..nj)]`。
    j: Vec<f32>,
    /// 对角协方差,长度 `w*h*nj`(横纵共用,式 6)。
    p: Vec<f32>,
    q过程: f32,
    /// 观测噪声方差 —— **每帧从这幅图自己的光流里量**,不是抄来的数。
    /// 论文里它是一个固定值 0.034(他们那台相机、那个场景上的),而抄一个别人机器上的数
    /// 正是本仓最不许有的东西:换一台相机、换一个分辨率、换一个帧率,它就错了。
    /// 这里取**全图光流平方的中位数** —— 画面绝大多数是没动的背景,它们的光流就是噪声本身。
    r观测: f32,
    /// 最近一帧的光流(供"跟着身体跑的点"用)。
    上流: Option<crate::flow::流>,
    簇心: Vec<Vec<f32>>,
    簇评: Vec<f32>,
    自簇: Vec<bool>,
    标: Vec<u8>,
    pub 帧: u32,
}

fn 缩半(src: &[u8], w: usize, h: usize) -> (Vec<u8>, usize, usize) {
    let (nw, nh) = (w / 2, h / 2);
    let mut out = vec![0u8; nw * nh];
    for y in 0..nh {
        for x in 0..nw {
            let (a, b) = (2 * x, 2 * y);
            let s = src[b * w + a] as u16 + src[b * w + a + 1] as u16
                + src[(b + 1) * w + a] as u16 + src[(b + 1) * w + a + 1] as u16;
            out[y * nw + x] = (s / 4) as u8;
        }
    }
    (out, nw, nh)
}

impl 自图 {
    /// `w0,h0` 是原图尺寸(内部按论文缩一半再算);`nj` 是这具身体能下命令的自由度个数。
    pub fn 新(w0: usize, h0: usize, nj: usize) -> 自图 {
        let (w, h) = (w0 / 2, h0 / 2);
        let nj = nj.max(1);
        自图 {
            w, h, nj,
            // 论文:雅可比初值全零,协方差初值 1。
            j: vec![0.0; w * h * 2 * nj],
            p: vec![1.0; w * h * nj],
            q过程: 1e-3,
            r观测: 1.0,   // 开机占位,第一帧就会被量到的值替掉
            上流: None,
            簇心: Vec::new(),
            簇评: Vec::new(),
            自簇: Vec::new(),
            标: vec![0; w * h],
            帧: 0,
        }
    }

    pub fn 宽(&self) -> usize { self.w }
    pub fn 高(&self) -> usize { self.h }

    /// 双线性取一个点的雅可比(工作分辨率坐标)。
    fn 取雅(&self, x: f32, y: f32) -> Vec<f32> {
        let x = x.clamp(0.0, (self.w - 1) as f32);
        let y = y.clamp(0.0, (self.h - 1) as f32);
        let (x0, y0) = (x.floor() as usize, y.floor() as usize);
        let (x1, y1) = ((x0 + 1).min(self.w - 1), (y0 + 1).min(self.h - 1));
        let (fx, fy) = (x - x0 as f32, y - y0 as f32);
        let n = 2 * self.nj;
        let mut out = vec![0.0f32; n];
        for k in 0..n {
            let a = self.j[(y0 * self.w + x0) * n + k] * (1.0 - fx) + self.j[(y0 * self.w + x1) * n + k] * fx;
            let b = self.j[(y1 * self.w + x0) * n + k] * (1.0 - fx) + self.j[(y1 * self.w + x1) * n + k] * fx;
            out[k] = a * (1.0 - fy) + b * fy;
        }
        out
    }

    /// 一次观测:发了 `qd` 这一下(**实际动了多少**,不是命令),画面从 `前` 变成 `后`。
    pub fn 喂(&mut self, 前: &[u8], 后: &[u8], w0: usize, h0: usize, qd: &[f64]) {
        if 前.len() < w0 * h0 || 后.len() < w0 * h0 || qd.len() < self.nj { return }
        let (a, w, h) = 缩半(前, w0, h0);
        let (b, _, _) = 缩半(后, w0, h0);
        if w != self.w || h != self.h { return }
        let Some(u) = crate::flow::算(&a, &b, w, h, 4, 40) else { return };
        if u.w != w || u.h != h { return }
        // 🔴 观测噪声**现量**:全图光流平方的中位数(绝大多数像素是没动的背景 ⇒ 那就是噪声底)。
        {
            let mut m: Vec<f32> = (0..w * h).step_by(7).map(|i| u.u[i] * u.u[i] + u.v[i] * u.v[i]).collect();
            if !m.is_empty() {
                m.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
                self.r观测 = m[m.len() / 2].max(1e-4);
            }
        }
        let qd: Vec<f32> = qd[..self.nj].iter().map(|v| *v as f32).collect();
        let n = 2 * self.nj;

        // ── ① 预测步:沿**预测光流** J·q̇ 把整张图搬过去(式 14);协方差加过程噪声(式 9)
        let mut 新j = vec![0.0f32; self.j.len()];
        for y in 0..h {
            for x in 0..w {
                let i = y * w + x;
                let (mut fu, mut fv) = (0.0f32, 0.0f32);
                for k in 0..self.nj {
                    fu += self.j[i * n + k] * qd[k];
                    fv += self.j[i * n + self.nj + k] * qd[k];
                }
                let src = self.取雅(x as f32 - fu, y as f32 - fv);
                新j[i * n..i * n + n].copy_from_slice(&src);
            }
        }
        self.j = 新j;
        for v in self.p.iter_mut() { *v += self.q过程 }

        // ── ② 更新步(式 10、11)
        let qq: Vec<f32> = qd.iter().map(|v| v * v).collect();
        for i in 0..w * h {
            let s: f32 = (0..self.nj).map(|k| self.p[i * self.nj + k] * qq[k]).sum();
            let 分母 = s + self.r观测;
            if !(分母.abs() > 1e-12) { continue }
            let (mut pu, mut pv) = (0.0f32, 0.0f32);
            for k in 0..self.nj {
                pu += self.j[i * n + k] * qd[k];
                pv += self.j[i * n + self.nj + k] * qd[k];
            }
            let (eu, ev) = (u.u[i] - pu, u.v[i] - pv);
            for k in 0..self.nj {
                let g = self.p[i * self.nj + k] * qd[k] / 分母;
                self.j[i * n + k] += eu * g;
                self.j[i * n + self.nj + k] += ev * g;
            }
            for k in 0..self.nj {
                let f = self.p[i * self.nj + k] * qq[k] / 分母;
                self.p[i * self.nj + k] *= 1.0 - f;
            }
        }
        self.上流 = Some(u);
        self.帧 = self.帧.saturating_add(1);
    }

    /// **算法 1 + 2:聚类 + 逐簇时间一致性 ⇒ 哪些像素是我。**
    ///
    /// `nk` 是簇数(论文用 5:理论上 2 就够 —— 机器人和背景 —— 但多分几簇能把背景运动
    /// 造出来的各种假雅可比分开)。`门` 是评分门槛(论文 0.2)。两个都是**个数/比值**,无量纲。
    pub fn 聚类并判我(&mut self, nk: usize, 门: f32) {
        let n = 2 * self.nj;
        let np = self.w * self.h;
        if np == 0 || nk == 0 { return }
        let mut 心: Vec<Vec<f32>> = if self.簇心.len() == nk {
            self.簇心.clone()
        } else {
            (0..nk).map(|c| {
                let i = (c * np / nk).min(np - 1);
                self.j[i * n..i * n + n].to_vec()
            }).collect()
        };
        let mut 标 = vec![0u8; np];
        for _ in 0..6 {   // 6 轮:**次数**,无量纲
            for i in 0..np {
                let v = &self.j[i * n..i * n + n];
                let mut best = (f32::INFINITY, 0u8);
                for (c, ce) in 心.iter().enumerate() {
                    let d: f32 = v.iter().zip(ce.iter()).map(|(a, b)| (a - b) * (a - b)).sum();
                    if d < best.0 { best = (d, c as u8) }
                }
                标[i] = best.1;
            }
            let mut 和 = vec![vec![0.0f32; n]; nk];
            let mut 数 = vec![0.0f32; nk];
            for i in 0..np {
                let c = 标[i] as usize;
                for k in 0..n { 和[c][k] += self.j[i * n + k] }
                数[c] += 1.0;
            }
            for c in 0..nk {
                if 数[c] > 0.0 { for k in 0..n { 心[c][k] = 和[c][k] / 数[c] } }
            }
        }
        // 算法 1 的评分:每个新簇心去上一轮里找**最近的**那个,继承它的分。
        // 越靠近(= 这个簇在时间上越稳)分越高 —— **这就是"是不是我"的全部判据**。
        let 旧心 = self.簇心.clone();
        let 旧评 = self.簇评.clone();
        let mut 新评 = vec![1.0f32; nk];
        if !旧心.is_empty() {
            for c in 0..nk {
                let mut best = (f32::INFINITY, 0usize);
                for (d, oc) in 旧心.iter().enumerate() {
                    let dd: f32 = 心[c].iter().zip(oc.iter()).map(|(a, b)| (a - b) * (a - b)).sum();
                    if dd < best.0 { best = (dd, d) }
                }
                let 距 = best.0.sqrt();
                let 模: f32 = 心[c].iter().map(|v| v * v).sum::<f32>().sqrt().max(1e-6);
                // 论文:Consistency = 1/sqrt(‖dist‖/‖Center‖ + 0.1)。
                // 括号里那个 0.1 只是防止除零的**无量纲**小量(距离已经除以了簇心自己的模,
                // 所以整个式子是尺度无关的);外面那个 0.1 是评分的**学习率**,也是无量纲。
                let 一致 = 1.0 / (距 / 模 + 0.1).sqrt();
                let 上 = 旧评.get(best.1).copied().unwrap_or(1.0);
                // 论文:Eval ← Eval × 0.1(Consistency − 1) + 1
                新评[c] = 上 * (0.1 * (一致 - 1.0) + 1.0);
            }
            let m = 新评.iter().cloned().fold(0.0f32, f32::max).max(1e-6);
            for v in 新评.iter_mut() { *v /= m }
        }
        // 🔴 **一动没动的簇不算我。** 论文靠"背景那些簇的雅可比在时间上不稳"把它们排掉,
        // 而一片**完全静止**的背景,它的雅可比恒等于零 —— 零在时间上是最稳的,反而拿满分。
        // ⇒ 再加一句:簇心的模必须够大(至少是最大那一簇的十分之一)。
        // 十分之一是**比值**,无量纲。
        let 模们: Vec<f32> = 心.iter().map(|c| c.iter().map(|v| v * v).sum::<f32>().sqrt()).collect();
        let 最大模 = 模们.iter().cloned().fold(0.0f32, f32::max).max(1e-9);
        self.自簇 = (0..nk).map(|c| 新评[c] > 门 && 模们[c] >= 最大模 * 0.1).collect();
        self.簇心 = 心;
        self.簇评 = 新评;
        self.标 = 标;
    }

    /// 这个像素(工作分辨率坐标)是不是我。
    pub fn 是我(&self, x: usize, y: usize) -> bool {
        if x >= self.w || y >= self.h { return false }
        let c = self.标[y * self.w + x] as usize;
        self.自簇.get(c).copied().unwrap_or(false)
    }

    /// 有多少像素被判成"是我"。
    pub fn 我有几个(&self) -> usize {
        (0..self.w * self.h)
            .filter(|i| self.自簇.get(self.标[*i] as usize).copied().unwrap_or(false))
            .count()
    }

    /// **式 15:让一个"长在身上的点"跟着身体跑。** 传归一化画幅坐标,返回新的归一化坐标。
    ///
    /// 它不是固定像素、也不是"最近的自体格子" —— 它每一帧沿**实测**光流走一步,
    /// 于是永远贴在身体的同一处。这一条一次性替掉模板匹配:没有模板、没有孔径问题。
    pub fn 跟着走(&self, u: f64, v: f64) -> (f64, f64) {
        let Some(f) = self.上流.as_ref() else { return (u, v) };
        let x = (u * self.w as f64) as f32;
        let y = (v * self.h as f64) as f32;
        let xc = x.clamp(0.0, (self.w - 1) as f32) as usize;
        let yc = y.clamp(0.0, (self.h - 1) as f32) as usize;
        let i = yc * self.w + xc;
        (
            (((x + f.u[i]) as f64) / self.w as f64).clamp(0.0, 1.0),
            (((y + f.v[i]) as f64) / self.h as f64).clamp(0.0, 1.0),
        )
    }

    /// **式 16:某一点处的雅可比**(归一化画幅坐标进)。
    /// 返回两行 × nj 列(行优先),单位是**画幅 / 命令**(已把工作分辨率换算掉)。
    pub fn 雅可比(&self, u: f64, v: f64) -> Vec<f64> {
        let j = self.取雅(u as f32 * self.w as f32, v as f32 * self.h as f32);
        let mut out = vec![0.0f64; 2 * self.nj];
        for k in 0..self.nj {
            out[k] = j[k] as f64 / self.w as f64;
            out[self.nj + k] = j[self.nj + k] as f64 / self.h as f64;
        }
        out
    }

    /// **我身上离某个画面位置最近的那一点**(归一化坐标进出)。伺服拿它当 `p_self` 的初值。
    ///
    /// 用"离目标最近"而不是整片形心:要送到球上的是指尖,而形心在胳膊中段
    ///(仓里 `hand.rs` N45:形心在指身中段 ⇒ 指尖越过物体,合爪咬边挤出)。
    pub fn 我离目标最近的一点(&self, 目u: f64, 目v: f64) -> Option<(f64, f64)> {
        let (tx, ty) = (目u * self.w as f64, 目v * self.h as f64);
        let mut best: Option<((usize, usize), f64)> = None;
        for y in 0..self.h {
            for x in 0..self.w {
                if !self.是我(x, y) { continue }
                let d = (x as f64 - tx).powi(2) + (y as f64 - ty).powi(2);
                if best.map(|(_, b)| d < b).unwrap_or(true) { best = Some(((x, y), d)) }
            }
        }
        best.map(|((x, y), _)| ((x as f64 + 0.5) / self.w as f64, (y as f64 + 0.5) / self.h as f64))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn 画(w: usize, h: usize, x: i64) -> Vec<u8> {
        let mut g = vec![40u8; w * h];
        for i in 0..w * h { if (i / w + i % w) % 7 == 0 { g[i] = 90 } }
        for dy in 0..32i64 { for dx in 0..32i64 {
            let (px, py) = (x + dx, 32 + dy);
            if px >= 0 && py >= 0 && (px as usize) < w && (py as usize) < h {
                let hh = ((dx as u64).wrapping_mul(2654435761) ^ (dy as u64).wrapping_mul(40503)) as u8;
                g[py as usize * w + px as usize] = 60u8.saturating_add(hh / 2);
            }
        }}
        g
    }

    /// 一块跟着命令来回动的方块该被判成"我",不动的背景不该。
    /// **这条测试守的是这一层唯一的那句判据。**
    #[test]
    fn 跟着我动的那一片才算我() {
        let (w, h) = (128usize, 96usize);
        let mut 自 = 自图::新(w, h, 1);
        let mut x = 40i64;
        let mut 前 = 画(w, h, x);
        for k in 0..14 {
            let c: f64 = if k % 2 == 0 { 1.0 } else { -1.0 };
            x += (4.0 * c) as i64;
            let 后 = 画(w, h, x);
            自.喂(&前, &后, w, h, &[c * 4.0]);
            自.聚类并判我(3, 0.2);
            前 = 后;
        }
        assert!(自.我有几个() > 0, "该认出一些像素是我");
        let (cx, cy) = (((x + 16) / 2) as usize, (32 + 16) / 2);
        assert!(自.是我(cx, cy), "跟着命令动的方块该被判成我(工作坐标 {cx},{cy})");
        assert!(!自.是我(3, 3), "不跟我动的背景不该被判成我");
    }

    /// **长在身上的那个点会跟着身体跑**(式 15)。
    #[test]
    fn 长在身上的点会跟着身体跑() {
        let (w, h) = (128usize, 96usize);
        let mut 自 = 自图::新(w, h, 1);
        自.喂(&画(w, h, 40), &画(w, h, 46), w, h, &[6.0]);
        let (u0, v0) = ((40.0 + 16.0) / w as f64, 48.0 / h as f64);
        let (u1, _) = 自.跟着走(u0, v0);
        assert!(u1 > u0, "点该跟着方块往右跑:{u0:.4} → {u1:.4}");
    }
}

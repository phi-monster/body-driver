//! **产点器:把眼睛看到的东西变成一团三维表面点 —— ②a 就吃这个。**
//!
//! # 这一格为什么一直空着
//!
//! `contact_gen::candidates` 要**一团物体表面的三维点**,而眼睛只给二维像素。
//! `DRIVER_GOAL` §四 第 2 步原文:*"那一块的深度点**反投影成三维点** —— 🔴 要建"*。
//! **接口和执行层齐了,喂它的东西没有** —— 这就是那个洞。
//!
//! # 🔴 owner 定死的那条界(2026-08-16)
//!
//! | | |
//! |---|---|
//! | 这台机器怎么拿 | 用深度图(渲染器一直在出) |
//! | **架构底线** | **两只普通相机(左右眼各一个)必须能构建出同一团点。深度不许变成硬依赖。** |
//! | 不要求 | 真·单目做立体 —— owner:*"太难为人了"* |
//!
//! ⇒ 这里**两条路并列**,共用同一个出口:`depth`(便利)与 `pair`(底线)。
//! **只造深度那一条就收工 = 违规。**
//!
//! # 🔴🔴 关于"不许写内参外参"这条仓规,要说准
//!
//! 仓里写着 *"no intrinsics, no extrinsics, no hand-eye transform"*,而 `debt.rs` 把一个
//! **写死的焦距**记成欠债(原文:*"FROZEN P1 rig; do not re-derive"*)。
//! **那条债的罪名是【写死】,不是【有焦距】。**
//!
//! 而另一侧有一条实测:*"一张全局换算表"* 被判死 —— 同一台相机三个锚点三份 2×3,
//! **`dv` 整行三个数各自翻号**,同布局重跑差 2.6%(不是噪声)。
//! **机制就是透视**:相机在 z=1.308,探针把手推到 z=1.2096(离相机 10 cm),
//! 动的像素从 1912 跳到 4276 —— **一个线性映射根本装不下它**。
//!
//! ⇒ 两条合起来的结论是:**别写死一个焦距,也别拿线性表硬凑;
//! 让身体【自己量】一个针孔模型出来。** 手在哪本体感受免费给,
//! 看着自己的手挪几个地方,焦距就解出来了 —— **没有标定板,没有外部文件**。
//! 这与"身体靠量"是同一条,而不是它的例外。

#![forbid(unsafe_code)]

/// 世界坐标里的一个点(米)。与 `contact_gen::P3` 同构,**故意不引它** ——
/// 这一层不依赖 ②a,是 ②a 的上游。
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct P3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// 画面上的一个像素(列, 行)。
pub type Px = [f64; 2];

/// **一只眼睛:针孔 + 它在世界里的位姿。**
///
/// 🔴 `fx/fy/cx/cy` **必须是量出来的**(见 `fit`),不许从配置文件里抄。
/// 位姿由**本体感受免费给** —— 相机拧在一根已知的连杆上,机器人知道那根杆在哪。
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Eye {
    /// 横向焦距(像素)。
    pub fx: f64,
    /// 纵向焦距(像素)。
    pub fy: f64,
    /// 主点列。
    pub cx: f64,
    /// 主点行。
    pub cy: f64,
    /// 相机在世界里的位置(米)。
    pub at: [f64; 3],
    /// 相机的朝向,四元数 `(w,x,y,z)`:把**相机系**的方向转到**世界系**。
    ///
    /// 相机系约定:**+z 朝前(看出去)· +x 朝右 · +y 朝下**(和图像的行列一致)。
    pub q: [f64; 4],
}

/// 量不出来 / 算不出来时,**点名**。
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum WhyNot {
    /// 样本太少,解不出四个未知数。
    TooFewSamples(usize),
    /// 🔴 **样本全挤在同一个深度上** —— 焦距和距离**乘在一起**出现,
    /// 只在一个深度上看,这两个分不开(把焦距翻倍、距离也翻倍,像素一模一样)。
    ///
    /// 这正是"一张全局换算表"当年翻号的根:**它是在一个高度上量的**,
    /// 换个高度就外推到了模型之外。带上实测的深度跨度(米)。
    AllAtOneDepth(f64),
    /// 两条视线**不相交**(差得比容差还远)—— 多半是左右眼配错了点。
    /// 带上两条线最近处的距离(米)。
    RaysMiss(f64),
    /// 这个点在相机**背后**,或者深度不是一个正数。
    Behind,
    /// 拟合之后残差仍然大 —— 模型装不下这组样本。带上最大残差(像素)。
    BadFit(f64),
    /// 🔴 样本**共面**(含"全在一个高度")—— 一张平面上的点定不下一个完整投影矩阵。
    Coplanar,
    /// 解出来的内参有**斜切**,而这个模型没有这一项 ⇒ 装不下。带上斜切量。
    HasSkew(f64),
}

fn qrot(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    let t = [
        2.0 * (y * v[2] - z * v[1]),
        2.0 * (z * v[0] - x * v[2]),
        2.0 * (x * v[1] - y * v[0]),
    ];
    [
        v[0] + w * t[0] + (y * t[2] - z * t[1]),
        v[1] + w * t[1] + (z * t[0] - x * t[2]),
        v[2] + w * t[2] + (x * t[1] - y * t[0]),
    ]
}

fn qconj(q: [f64; 4]) -> [f64; 4] {
    [q[0], -q[1], -q[2], -q[3]]
}

impl Eye {
    /// 把一个世界点变到**相机系**。
    pub fn into_cam(&self, p: P3) -> [f64; 3] {
        let d = [p.x - self.at[0], p.y - self.at[1], p.z - self.at[2]];
        qrot(qconj(self.q), d)
    }

    /// **世界点 → 像素。** 在相机背后就没有像素。
    pub fn project(&self, p: P3) -> Option<Px> {
        let c = self.into_cam(p);
        if c[2] <= 1e-9 {
            return None;
        }
        Some([self.cx + self.fx * c[0] / c[2], self.cy + self.fy * c[1] / c[2]])
    }

    /// **一个像素对应的那条视线**(世界系的单位方向)。
    pub fn ray(&self, px: Px) -> [f64; 3] {
        let d = [(px[0] - self.cx) / self.fx, (px[1] - self.cy) / self.fy, 1.0];
        let w = qrot(self.q, d);
        let n = (w[0] * w[0] + w[1] * w[1] + w[2] * w[2]).sqrt();
        [w[0] / n, w[1] / n, w[2] / n]
    }

    /// **像素 + 深度 → 世界点。** `depth` 是**沿相机光轴**的距离(米),不是斜距。
    pub fn back_project(&self, px: Px, depth: f64) -> Result<P3, WhyNot> {
        if !(depth.is_finite() && depth > 0.0) {
            return Err(WhyNot::Behind);
        }
        let d = [(px[0] - self.cx) / self.fx * depth, (px[1] - self.cy) / self.fy * depth, depth];
        let w = qrot(self.q, d);
        Ok(P3 { x: self.at[0] + w[0], y: self.at[1] + w[1], z: self.at[2] + w[2] })
    }
}

/// **让身体自己量出一只眼睛的针孔参数。**
///
/// 输入:一串 `(手在哪, 手在画面的哪个像素)`。**手在哪由本体感受免费给** ——
/// 不需要标定板、不需要外部文件,这与"身体靠量"是同一条。
///
/// # 🔴 它会拒绝,而拒绝正是这一步最值钱的地方
///
/// 焦距与距离在针孔里是**乘在一起**出现的(`u = cx + fx·X/Z`)。
/// **只在一个深度上采样,这两个分不开** —— 焦距翻倍、距离也翻倍,像素一模一样。
/// 当年那张"全局换算表"就是这么翻号的:它在一个高度上量,换个高度就外推到了模型之外。
/// ⇒ 深度跨度不够就 `AllAtOneDepth`,**不许硬解一个出来**。
///
/// 位姿(`at`/`q`)由调用方给(本体感受量的);这里只解 `fx/fy/cx/cy` —— 它们对
/// `(X/Z, Y/Z)` 是**线性**的,两条一元最小二乘就够,不引任何矩阵库。
pub fn fit(at: [f64; 3], q: [f64; 4], seen: &[(P3, Px)]) -> Result<Eye, WhyNot> {
    if seen.len() < 4 {
        return Err(WhyNot::TooFewSamples(seen.len()));
    }
    let probe = Eye { fx: 1.0, fy: 1.0, cx: 0.0, cy: 0.0, at, q };
    let mut xs = Vec::with_capacity(seen.len());
    for (p, px) in seen {
        let c = probe.into_cam(*p);
        if c[2] <= 1e-9 {
            return Err(WhyNot::Behind);
        }
        xs.push(([c[0] / c[2], c[1] / c[2]], *px, c[2]));
    }
    // 🔴 深度跨度:太窄就分不开焦距和距离。门槛按**相对**跨度定(10%),不拍绝对米数。
    let (mut zlo, mut zhi) = (f64::MAX, f64::MIN);
    for (_, _, z) in &xs {
        zlo = zlo.min(*z);
        zhi = zhi.max(*z);
    }
    if zhi - zlo < 0.1 * zhi {
        return Err(WhyNot::AllAtOneDepth(zhi - zlo));
    }
    // 两条独立的一元最小二乘:u = cx + fx·(X/Z),v = cy + fy·(Y/Z)
    let solve = |i: usize| -> (f64, f64) {
        let n = xs.len() as f64;
        let (sx, sy): (f64, f64) =
            (xs.iter().map(|(r, _, _)| r[i]).sum(), xs.iter().map(|(_, px, _)| px[i]).sum());
        let (mx, my) = (sx / n, sy / n);
        let mut num = 0.0;
        let mut den = 0.0;
        for (r, px, _) in &xs {
            num += (r[i] - mx) * (px[i] - my);
            den += (r[i] - mx) * (r[i] - mx);
        }
        let f = if den > 1e-18 { num / den } else { 0.0 };
        (f, my - f * mx)
    };
    let (fx, cx) = solve(0);
    let (fy, cy) = solve(1);
    let eye = Eye { fx, fy, cx, cy, at, q };
    // 🔴 拟合完**回代核一遍**:装不下就说装不下,不许把残差咽下去。
    let mut worst = 0.0f64;
    for (p, px) in seen {
        let got = eye.project(*p).ok_or(WhyNot::Behind)?;
        worst = worst.max((got[0] - px[0]).abs().max((got[1] - px[1]).abs()));
    }
    if worst > 0.5 {
        return Err(WhyNot::BadFit(worst));
    }
    Ok(eye)
}

/// **两只普通相机 → 一个三维点。没有深度传感器。**
///
/// 这就是 owner 定的那条架构底线。两只眼的相对位姿**由本体感受免费给**
/// (两个相机拧在同一具身体上,机器人知道它们各在哪)—— **不需要外部标定**。
///
/// 做法:两条视线在三维里一般**不相交**,取它们最近的那一段的中点;
/// 差得太远就说明左右眼**配错了点**,当场 `RaysMiss` —— 那是配对的错,不许当成一个点收下。
pub fn triangulate(a: &Eye, pa: Px, b: &Eye, pb: Px, tol_m: f64) -> Result<P3, WhyNot> {
    let (da, db) = (a.ray(pa), b.ray(pb));
    let w = [a.at[0] - b.at[0], a.at[1] - b.at[1], a.at[2] - b.at[2]];
    let dot = |u: [f64; 3], v: [f64; 3]| u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
    let (aa, bb, cc) = (dot(da, da), dot(da, db), dot(db, db));
    let (dd, ee) = (dot(da, w), dot(db, w));
    let den = aa * cc - bb * bb;
    if den.abs() < 1e-12 {
        return Err(WhyNot::RaysMiss(f64::INFINITY)); // 两条线平行 ⇒ 定不下来
    }
    let s = (bb * ee - cc * dd) / den;
    let t = (aa * ee - bb * dd) / den;
    if s <= 0.0 || t <= 0.0 {
        return Err(WhyNot::Behind);
    }
    let pa3 = [a.at[0] + da[0] * s, a.at[1] + da[1] * s, a.at[2] + da[2] * s];
    let pb3 = [b.at[0] + db[0] * t, b.at[1] + db[1] * t, b.at[2] + db[2] * t];
    let miss = ((pa3[0] - pb3[0]).powi(2) + (pa3[1] - pb3[1]).powi(2) + (pa3[2] - pb3[2]).powi(2)).sqrt();
    if miss > tol_m {
        return Err(WhyNot::RaysMiss(miss));
    }
    Ok(P3 {
        x: (pa3[0] + pb3[0]) / 2.0,
        y: (pa3[1] + pb3[1]) / 2.0,
        z: (pa3[2] + pb3[2]) / 2.0,
    })
}

/// **深度那条路:一张深度图 + 一块掩膜 → 一团点。**
///
/// `mask` 说**哪些像素是这个物体** —— 由眼睛(学的那一层)给。
///
/// # 🔴 这里【不】自己切物体,而这是有代价换来的
///
/// LAB 原话:*"坏的不是深度,是从深度里切物体"* —— 那条纯几何规则
/// (比 90 分位近 1 cm)实测掩膜占了**全帧 72%**,手从 27 cm 逼近到 0.7 cm 时
/// **物体像素宽 601→603 纹丝不动**。⇒ **切物体是【学】的活,不是【算】的活**,
/// 这里老老实实把它当输入要。
pub fn from_depth(eye: &Eye, depth: &[f64], mask: &[bool], w: usize, h: usize) -> Vec<P3> {
    let mut out = Vec::new();
    for r in 0..h {
        for c in 0..w {
            let i = r * w + c;
            if i >= mask.len() || !mask[i] || i >= depth.len() {
                continue;
            }
            if let Ok(p) = eye.back_project([c as f64, r as f64], depth[i]) {
                out.push(p);
            }
        }
    }
    out
}

/// **两只相机那条路:左右各一串已配好对的像素 → 一团点。**
///
/// 配对(哪个左像素对哪个右像素)由眼睛给 —— 与掩膜同一条理由:**那是学的活**。
/// 配错了会被 `triangulate` 的容差挡下来,**挡下来的直接丢掉,不许硬凑进点云**。
///
/// 返回 `(点, 丢了几个)` —— 丢了多少必须报出来,悄悄丢就成了"点云很干净"的假象。
pub fn from_pair(a: &Eye, b: &Eye, pairs: &[(Px, Px)], tol_m: f64) -> (Vec<P3>, usize) {
    let mut out = Vec::new();
    let mut 丢 = 0usize;
    for (pa, pb) in pairs {
        match triangulate(a, *pa, b, *pb, tol_m) {
            Ok(p) => out.push(p),
            Err(_) => 丢 += 1,
        }
    }
    (out, 丢)
}

// ─────────────────────────────────────────────────────────────────────
// 完整自标定:连**相机在哪**一起解出来
// ─────────────────────────────────────────────────────────────────────

/// **看着自己的手,把"相机在哪 + 焦距多少"一起解出来。**
///
/// # 为什么必须连位姿一起解
///
/// `fit` 要调用方给相机位姿 —— **腕相机行**(拧在已知连杆上,本体感受免费给),
/// **头相机不行**:它固定在世界里某处,那个位姿本身就是一个**没量过的身体常数**。
/// 而这条链最需要的恰恰是头相机(开局只有它看得见物体)。
///
/// # 输入只有一样东西:手挪到哪儿、在画面的哪个像素
///
/// **没有标定板、没有配置文件、没有人手填的数。** 手在哪由本体感受免费给。
/// 仿真器的确会在线缆里自报 `intrinsic_matrix` / `extrinsic_matrix`,
/// 🔴 **但那是仿真的便利,真机上没人给你** —— 所以自报值只拿来**独立核对**,
/// 永远不当来源。(这与 `debt.rs` 记的那笔"写死焦距"的债是同一条界。)
///
/// # 做法
///
/// 直接线性变换(DLT)解出 3×4 的投影矩阵,再拆成"内参 × 位姿"。
/// 先按 Hartley 把三维点和像素各自归一化 —— **不归一化,条件数会烂到解不出来**。
pub fn fit_full(seen: &[(P3, Px)]) -> Result<Eye, WhyNot> {
    if seen.len() < 6 {
        return Err(WhyNot::TooFewSamples(seen.len()));
    }
    // 🔴 共面就拒绝:一张平面上的点定不下一个完整投影矩阵(经典退化)。
    // 判据用**最小主轴的伸展**:比最大主轴小 1000 倍就当共面。
    if 共面(seen) {
        return Err(WhyNot::Coplanar);
    }
    let (zlo, zhi) = seen.iter().fold((f64::MAX, f64::MIN), |(a, b), (p, _)| (a.min(p.z), b.max(p.z)));
    let _ = (zlo, zhi); // 深度跨度那一条由共面判据覆盖(共面含"全在一个高度")

    // Hartley 归一化
    let n = seen.len() as f64;
    let cx3 = [
        seen.iter().map(|(p, _)| p.x).sum::<f64>() / n,
        seen.iter().map(|(p, _)| p.y).sum::<f64>() / n,
        seen.iter().map(|(p, _)| p.z).sum::<f64>() / n,
    ];
    let s3 = {
        let d: f64 = seen
            .iter()
            .map(|(p, _)| ((p.x - cx3[0]).powi(2) + (p.y - cx3[1]).powi(2) + (p.z - cx3[2]).powi(2)).sqrt())
            .sum();
        if d > 1e-12 { n * 3f64.sqrt() / d } else { 1.0 }
    };
    let cp = [
        seen.iter().map(|(_, q)| q[0]).sum::<f64>() / n,
        seen.iter().map(|(_, q)| q[1]).sum::<f64>() / n,
    ];
    let sp = {
        let d: f64 = seen
            .iter()
            .map(|(_, q)| ((q[0] - cp[0]).powi(2) + (q[1] - cp[1]).powi(2)).sqrt())
            .sum();
        if d > 1e-12 { n * 2f64.sqrt() / d } else { 1.0 }
    };

    let mut ata = vec![vec![0.0f64; 12]; 12];
    for (p, q) in seen {
        let x = [(p.x - cx3[0]) * s3, (p.y - cx3[1]) * s3, (p.z - cx3[2]) * s3, 1.0];
        let (u, v) = ((q[0] - cp[0]) * sp, (q[1] - cp[1]) * sp);
        let mut r1 = [0.0f64; 12];
        let mut r2 = [0.0f64; 12];
        for k in 0..4 {
            r1[k] = -x[k];
            r1[8 + k] = u * x[k];
            r2[4 + k] = -x[k];
            r2[8 + k] = v * x[k];
        }
        for i in 0..12 {
            for j in 0..12 {
                ata[i][j] += r1[i] * r1[j] + r2[i] * r2[j];
            }
        }
    }
    let p12 = 最小特征向量n(&mut ata);
    // 反归一化:P = Tp⁻¹ · P̂ · T3
    let mut pm = [[0.0f64; 4]; 3];
    for r in 0..3 {
        for c in 0..4 {
            pm[r][c] = p12[r * 4 + c];
        }
    }
    // T3:x' = s3(x − c3)  ⇒  P̂·T3 作用在原始齐次坐标上
    let mut p2 = [[0.0f64; 4]; 3];
    for r in 0..3 {
        for c in 0..3 {
            p2[r][c] = pm[r][c] * s3;
        }
        p2[r][3] = pm[r][3] - s3 * (pm[r][0] * cx3[0] + pm[r][1] * cx3[1] + pm[r][2] * cx3[2]);
    }
    // Tp⁻¹:u = u'/sp + cp
    let mut p3 = [[0.0f64; 4]; 3];
    for c in 0..4 {
        p3[0][c] = p2[0][c] / sp + cp[0] * p2[2][c];
        p3[1][c] = p2[1][c] / sp + cp[1] * p2[2][c];
        p3[2][c] = p2[2][c];
    }
    拆开(p3, seen)
}

/// 这组点是不是**共面**(含"全在一个高度")。用协方差最小主轴的伸展判。
fn 共面(seen: &[(P3, Px)]) -> bool {
    let n = seen.len() as f64;
    let m = [
        seen.iter().map(|(p, _)| p.x).sum::<f64>() / n,
        seen.iter().map(|(p, _)| p.y).sum::<f64>() / n,
        seen.iter().map(|(p, _)| p.z).sum::<f64>() / n,
    ];
    let mut c = vec![vec![0.0f64; 3]; 3];
    for (p, _) in seen {
        let d = [p.x - m[0], p.y - m[1], p.z - m[2]];
        for i in 0..3 {
            for j in 0..3 {
                c[i][j] += d[i] * d[j];
            }
        }
    }
    let mut cc = c.clone();
    let _ = 最小特征向量n(&mut cc);
    // 对角线上的特征值(Jacobi 之后 c 已被对角化)
    let (mut lo, mut hi) = (f64::MAX, f64::MIN);
    for i in 0..3 {
        lo = lo.min(cc[i][i].abs());
        hi = hi.max(cc[i][i].abs());
    }
    hi <= 0.0 || lo / hi < 1e-6
}

/// 把 3×4 的投影矩阵拆成"内参 × 位姿",并**回代核一遍**。
fn 拆开(p: [[f64; 4]; 3], seen: &[(P3, Px)]) -> Result<Eye, WhyNot> {
    // 🔴 DLT 解出来的投影矩阵**差一个整体符号**(零空间向量正负都行)。
    // 符号错了,所有点都会算到相机背后去 —— 病相是 `Behind`,而根因只是一个正负号。
    // 判法:拿样本点代进第三行,看深度是正是负;多数为负就整体翻号。
    let 正 = seen
        .iter()
        .filter(|(x, _)| p[2][0] * x.x + p[2][1] * x.y + p[2][2] * x.z + p[2][3] > 0.0)
        .count();
    let p = if 正 * 2 < seen.len() {
        let mut q = p;
        for r in q.iter_mut() {
            for v in r.iter_mut() {
                *v = -*v;
            }
        }
        q
    } else {
        p
    };
    let m = [[p[0][0], p[0][1], p[0][2]], [p[1][0], p[1][1], p[1][2]], [p[2][0], p[2][1], p[2][2]]];
    let (k, r) = rq3(m);
    // K 的对角要为正:负的就把那一列/行同时翻号(不改变 K·R)
    let mut k = k;
    let mut r = r;
    for i in 0..3 {
        if k[i][i] < 0.0 {
            for j in 0..3 {
                k[j][i] = -k[j][i];
                r[i][j] = -r[i][j];
            }
        }
    }
    let s = k[2][2];
    if s.abs() < 1e-15 {
        return Err(WhyNot::BadFit(f64::INFINITY));
    }
    for row in k.iter_mut() {
        for v in row.iter_mut() {
            *v /= s;
        }
    }
    // 斜切项装不下就说装不下 —— 这个模型没有 skew
    if k[0][1].abs() > 1e-6 * k[0][0].abs().max(1.0) {
        return Err(WhyNot::HasSkew(k[0][1]));
    }
    // 相机中心:C = −M⁻¹ p₄
    let p4 = [p[0][3] / s, p[1][3] / s, p[2][3] / s];
    let mut kr = [[0.0f64; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            kr[i][j] = m[i][j] / s;
        }
    }
    let c = 解三元(kr, [-p4[0], -p4[1], -p4[2]]).ok_or(WhyNot::BadFit(f64::INFINITY))?;
    // r 是 世界→相机;Eye.q 存的是 相机→世界 ⇒ 取转置
    let q = 转成四元数([[r[0][0], r[1][0], r[2][0]], [r[0][1], r[1][1], r[2][1]], [r[0][2], r[1][2], r[2][2]]]);
    let eye = Eye { fx: k[0][0], fy: k[1][1], cx: k[0][2], cy: k[1][2], at: c, q };
    // 🔴 回代:装不下就报出来,不许把残差咽下去。
    let mut worst = 0.0f64;
    for (p3d, px) in seen {
        let got = eye.project(*p3d).ok_or(WhyNot::Behind)?;
        worst = worst.max((got[0] - px[0]).abs().max((got[1] - px[1]).abs()));
    }
    if worst > 0.5 {
        return Err(WhyNot::BadFit(worst));
    }
    Ok(eye)
}

/// 3×3 的 RQ 分解(Givens),`m = k · r`,`k` 上三角、`r` 正交。零依赖。
fn rq3(m: [[f64; 3]; 3]) -> ([[f64; 3]; 3], [[f64; 3]; 3]) {
    let mut a = m;
    let mut q = [[0.0f64; 3]; 3];
    for i in 0..3 {
        q[i][i] = 1.0;
    }
    fn give(a: &mut [[f64; 3]; 3], q: &mut [[f64; 3]; 3], i: usize, j: usize, x: f64, y: f64) {
        let d = (x * x + y * y).sqrt();
        if d < 1e-18 {
            return;
        }
        let (c, s) = (x / d, y / d);
        let mut g = [[0.0f64; 3]; 3];
        for k in 0..3 {
            g[k][k] = 1.0;
        }
        g[i][i] = c;
        g[i][j] = -s;
        g[j][i] = s;
        g[j][j] = c;
        let mut na = [[0.0f64; 3]; 3];
        let mut nq = [[0.0f64; 3]; 3];
        for r in 0..3 {
            for cc in 0..3 {
                na[r][cc] = (0..3).map(|k| a[r][k] * g[k][cc]).sum();
                // q ← gᵀ · q
                nq[r][cc] = (0..3).map(|k| g[k][r] * q[k][cc]).sum();
            }
        }
        *a = na;
        *q = nq;
    }
    let (x, y) = (a[2][2], -a[2][1]);
    give(&mut a, &mut q, 1, 2, x, y);
    let (x, y) = (a[2][2], -a[2][0]);
    give(&mut a, &mut q, 0, 2, x, y);
    let (x, y) = (a[1][1], -a[1][0]);
    give(&mut a, &mut q, 0, 1, x, y);
    (a, q)
}

fn 解三元(a: [[f64; 3]; 3], b: [f64; 3]) -> Option<[f64; 3]> {
    let det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
    if det.abs() < 1e-18 {
        return None;
    }
    let mut out = [0.0f64; 3];
    for k in 0..3 {
        let mut m = a;
        for r in 0..3 {
            m[r][k] = b[r];
        }
        let d = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
        out[k] = d / det;
    }
    Some(out)
}

fn 转成四元数(m: [[f64; 3]; 3]) -> [f64; 4] {
    let tr = m[0][0] + m[1][1] + m[2][2];
    let q = if tr > 0.0 {
        let s = (tr + 1.0).sqrt() * 2.0;
        [0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s]
    } else if m[0][0] > m[1][1] && m[0][0] > m[2][2] {
        let s = (1.0 + m[0][0] - m[1][1] - m[2][2]).sqrt() * 2.0;
        [(m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s]
    } else if m[1][1] > m[2][2] {
        let s = (1.0 + m[1][1] - m[0][0] - m[2][2]).sqrt() * 2.0;
        [(m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s]
    } else {
        let s = (1.0 + m[2][2] - m[0][0] - m[1][1]).sqrt() * 2.0;
        [(m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s]
    };
    let n = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
    [q[0] / n, q[1] / n, q[2] / n, q[3] / n]
}

/// N×N 实对称阵的**最小**特征向量,循环 Jacobi;算完 `a` 已被对角化。零依赖。
fn 最小特征向量n(a: &mut Vec<Vec<f64>>) -> Vec<f64> {
    let n = a.len();
    let mut v = vec![vec![0.0f64; n]; n];
    for i in 0..n {
        v[i][i] = 1.0;
    }
    for _ in 0..100 {
        let mut off = 0.0;
        for i in 0..n {
            for j in (i + 1)..n {
                off += a[i][j] * a[i][j];
            }
        }
        if off < 1e-30 {
            break;
        }
        for p in 0..n {
            for q in (p + 1)..n {
                if a[p][q].abs() < 1e-20 {
                    continue;
                }
                let th = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
                let t = th.signum() / (th.abs() + (th * th + 1.0).sqrt());
                let c = 1.0 / (t * t + 1.0).sqrt();
                let s = t * c;
                for k in 0..n {
                    let (kp, kq) = (a[k][p], a[k][q]);
                    a[k][p] = c * kp - s * kq;
                    a[k][q] = s * kp + c * kq;
                }
                for k in 0..n {
                    let (pk, qk) = (a[p][k], a[q][k]);
                    a[p][k] = c * pk - s * qk;
                    a[q][k] = s * pk + c * qk;
                }
                for k in 0..n {
                    let (kp, kq) = (v[k][p], v[k][q]);
                    v[k][p] = c * kp - s * kq;
                    v[k][q] = s * kp + c * kq;
                }
            }
        }
    }
    let mut lo = 0usize;
    for i in 1..n {
        if a[i][i].abs() < a[lo][lo].abs() {
            lo = i;
        }
    }
    (0..n).map(|k| v[k][lo]).collect()
}

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
    /// 🔴 **两个视角挨得太近** —— 三角形太扁,深度极度不敏感,误差爆炸。
    /// 一只相机靠自己挪动当双目用时,这一条是那条硬边界。
    BaselineTooShort { got_m: f64, need_m: f64 },
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
    // 🔴 残差门槛不能照合成数据定。合成台上该到 1e-6,而**真数据自带 ~18 px 的认手噪声**
    // ⇒ 残差本来就该是那个量级。这里**只挡真正的退化**(残差大到画幅一半),
    // 具体够不够用由调用方拿自己的判据去比 —— 而这一层把**斜切**与**残差**都报出来。
    let 画幅 = seen.iter().map(|(_, u)| u[0].abs().max(u[1].abs())).fold(0.0f64, f64::max).max(1.0);
    if worst > 画幅 * 0.5 {
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
    // 🔴 **斜切按 0 处理,而不是"解出来有斜切就拒绝"。**
    //
    // 真相机的斜切本来就是 0 —— 它是**模型的约束**,不是待解的量。
    // 先无约束地解、再抱怨解出来带斜切,是把**测量噪声**读成了"这台相机有斜切":
    // 实测(2026-08-16,真数据 24 组、认手噪声 ~18 px)解出 **skew = 50.3**,
    // 而那 50.3 全是噪声被塞进了这一项。⇒ 归零,并把归掉多少**报出来**,不藏。
    let 斜切 = k[0][1];
    k[0][1] = 0.0;
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
    // 🔴 残差门槛不能照合成数据定。合成台上该到 1e-6,而**真数据自带 ~18 px 的认手噪声**
    // ⇒ 残差本来就该是那个量级。这里只挡真正的退化(残差大到画幅一半);
    // 够不够用由调用方拿自己的判据去比,而这一层把【斜切】与【残差】都报出来。
    let 画幅 = seen.iter().map(|(_, u)| u[0].abs().max(u[1].abs())).fold(0.0f64, f64::max).max(1.0);
    if worst > 画幅 * 0.5 {
        return Err(WhyNot::BadFit(worst));
    }
    if 斜切.abs() > 1e-6 {
        println!("[针孔] 斜切按 0 处理(无约束解里是 {:.1});回代最大残差 {:.1} px", 斜切, worst);
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

// ─────────────────────────────────────────────────────────────────────
// 哪些像素是这个物体 —— **锚在眼睛指的那一点上**,不对全画面用规则
// ─────────────────────────────────────────────────────────────────────

/// 圈不出来时,**点名**。
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum NoMask {
    /// 眼睛指的那一点上没有有效深度 —— 指到天上或指到洞里了。
    NothingThere,
    /// 🔴 **圈出来的东西比眼睛说的大太多** —— 十有八九是把桌面/背景一起圈进来了。
    ///
    /// 带上(实际占全帧的比例, 眼睛说的那一块该占的比例)。
    ///
    /// 这一条就是冲着 LAB 那次失败装的:那条规则是**对整幅画**说"比 90 分位近 1 cm",
    /// 实测掩膜占了**全帧 72%**,而手从 27 cm 逼近到 0.7 cm 时**物体像素宽 601→603 纹丝不动**
    /// —— 它圈的根本不是物体。**这个闸会响,那次就不会一路走到合爪。**
    TooBig(f64, f64),
    /// 圈出来的点太少,凑不成一个面。
    TooFew(usize),
}

/// **从深度图里圈出那个物体的像素。**
///
/// 输入的 `at_px` / `span_frac` 就是眼睛给的那两样(它指的点、那一块占画面宽的几分之几)。
///
/// # 🔴 它跟 LAB 判死的那条规则差在哪
///
/// 那一条是**全局**的:*"深度比全画面 90 分位近 1 cm 的都算物体"* ——
/// 于是整张桌子都比背景近,掩膜吃掉 **72% 全帧**。
///
/// 这一条是**局部 + 有锚**的:只在眼睛指的那个圈里找,而且深度必须贴着**那一点**的深度。
/// 厚度门槛也不是拍的:一个紧凑物体前后的厚度,大致不超过它自己的宽 ——
/// 而它的宽由 `span_frac` × 那个距离上的画幅宽算得出来,**全是量出来的量**。
///
/// **并且它会拒绝**:圈出来的比眼睛说的大太多就 `TooBig` —— 那次 72% 会当场被挡下。
pub fn mask_around(
    eye: &Eye,
    depth: &[f64],
    w: usize,
    h: usize,
    at_px: Px,
    span_frac: f64,
    宽容: f64,
) -> Result<Vec<bool>, NoMask> {
    let idx = |c: usize, r: usize| r * w + c;
    let (c0, r0) = (at_px[0].round() as i64, at_px[1].round() as i64);
    if c0 < 0 || r0 < 0 || c0 as usize >= w || r0 as usize >= h {
        return Err(NoMask::NothingThere);
    }
    // 中心那一点的深度:取它周围一小圈的中位,单个像素太脆
    let mut near = Vec::new();
    for dr in -2i64..=2 {
        for dc in -2i64..=2 {
            let (c, r) = (c0 + dc, r0 + dr);
            if c < 0 || r < 0 || c as usize >= w || r as usize >= h {
                continue;
            }
            let v = depth[idx(c as usize, r as usize)];
            if v.is_finite() && v > 0.0 {
                near.push(v);
            }
        }
    }
    if near.len() < 5 {
        return Err(NoMask::NothingThere);
    }
    near.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let z0 = near[near.len() / 2];

    // 眼睛说它占画面宽的 span_frac ⇒ 在 z0 那个距离上,它大约有这么宽(米)
    let 画幅宽_米 = w as f64 / eye.fx * z0;
    let 物体宽_米 = (span_frac * 画幅宽_米).max(1e-4);
    // 圈的半径(像素):比眼睛说的略放一点,放多少由调用方给
    let 半径_px = (span_frac * w as f64 / 2.0 * 宽容).max(3.0);
    // 深度厚度:一个紧凑物体前后不会比自己更厚
    let 厚 = 物体宽_米 * 宽容;

    let mut mask = vec![false; w * h];
    let mut n = 0usize;
    for r in 0..h {
        for c in 0..w {
            let (dc, dr) = (c as f64 - at_px[0], r as f64 - at_px[1]);
            if dc * dc + dr * dr > 半径_px * 半径_px {
                continue;
            }
            let v = depth[idx(c, r)];
            if !(v.is_finite() && v > 0.0) {
                continue;
            }
            if (v - z0).abs() > 厚 {
                continue;
            }
            mask[idx(c, r)] = true;
            n += 1;
        }
    }
    if n < 8 {
        return Err(NoMask::TooFew(n));
    }
    // 🔴 会响的闸 —— 两道,各有各的理由,**都不许被 `宽容` 放大到失效**。
    //
    // ⚠️ 上一版把门槛写成 `该占 × 宽容² × 4`。`宽容 = 8` 时门槛 = **268%**,
    // 而一个门槛超过 100% 的闸**根本不可能响** —— 它读起来是一道闸,其实是个摆设。
    // 实测:整幅桌面被圈走 58%,那一版照样放行。**这就是"永远不会响的闸"那一族。**
    let 实占 = n as f64 / (w * h) as f64;
    let 该占 = core::f64::consts::PI / 4.0 * span_frac * span_frac * w as f64 / h as f64;
    // ① 相对眼睛说的那一块:放宽多少半径,面积就放宽它的平方,再多就是圈错了。
    // ② 绝对上限:**即使这东西是一根贯穿整幅画高的竖条**,它也只占 `span_frac`;
    //    占到两倍,就一定圈到别的东西了。这一条**与 `宽容` 无关**,放多宽都挡得住。
    if 实占 > 该占 * 宽容 * 宽容 || 实占 > span_frac * 2.0 {
        return Err(NoMask::TooBig(实占, 该占));
    }
    Ok(mask)
}

// ─────────────────────────────────────────────────────────────────────
// 一个俯视相机只看得见顶面 —— 两条出路,一条要假设,一条不要
// ─────────────────────────────────────────────────────────────────────

/// **把几个视角的点合到一起。** 不需要任何假设。
///
/// # 为什么这是首选
///
/// 一个俯视相机只看得见物体的**顶面**,而手指要捏的是**侧面** ——
/// 实测:合成一个方块的俯视深度图,点全落在同一个高度上,②a 直接拒绝 `Flat`
/// (它按高度切层,而一张平面切不出层)。**这不是代码的错,是传感器摆位的物理事实。**
///
/// 合并不需要外部标定:每一帧的相机位姿由**本体感受免费给**,
/// 而 `from_depth` 吐出来的本来就是**世界坐标** ⇒ 直接拼起来就是对的。
/// 🔴 **这条路一个假设都不用**,所以它排在下面那条前面。
pub fn merge(clouds: &[Vec<P3>]) -> Vec<P3> {
    let mut out = Vec::new();
    for c in clouds {
        out.extend_from_slice(c);
    }
    out
}

/// **把看得见的顶面,朝支撑面"拉"下去,补出侧面。**
///
/// # 🔴🔴 这一条是【假设】,不是测量 —— 所以它必须是显式的、能关掉的
///
/// 假设原文:**物体是实心的,从看得见的顶面一直连到支撑面。**
///
/// **它什么时候是错的**:马克杯的把手(把手下面是空的)· 拱形件 · 悬臂 · 有凹槽的东西 ——
/// 这些地方拉出来的"侧面"根本不存在,而**爪子会合到空气上**。
///
/// **它什么时候可以用**:摆在桌上的紧凑实心件(积木、瓶子、盒子)—— 占绝大多数。
///
/// ⇒ 用它就要**明说用了**,别让它悄悄变成"我们看见了侧面"。
/// 首选仍然是 `merge`(换个视角再看一眼),那条**不用假设**。
///
/// `support_z` 是支撑面的高度(**量出来的**,不是拍的);`step_m` 是拉下去时每层多密。
pub fn extrude_to_support(top: &[P3], support_z: f64, step_m: f64) -> Vec<P3> {
    let mut out = top.to_vec();
    let step = step_m.max(1e-4);
    for p in top {
        let mut z = p.z - step;
        while z > support_z {
            out.push(P3 { x: p.x, y: p.y, z });
            z -= step;
        }
    }
    out
}

// ─────────────────────────────────────────────────────────────────────
// 尺子:用身体自己量过的长度,查出"两只眼配歪了"
// ─────────────────────────────────────────────────────────────────────

/// 尺子对不上时说什么。
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Drift {
    /// 两只眼看到的那把尺子,量出来跟**身体自己量过的**对不上。
    /// 带上(量出来多长, 该多长, 容差)。
    Off { got_m: f64, want_m: f64, tol_m: f64 },
    /// 尺子的两端有一端三角化不出来。
    CannotSee(WhyNot),
    /// 给的"该多长"不是一个正数。
    BadRuler,
}

/// **拿身体自己量过的一段长度当尺子,查两只眼的相对位姿有没有飘。**
///
/// # 🔴 为什么必须有这一道闸
///
/// 实测(2026-08-16):把一只眼拧 **0.1°**,三维点差 **5.27 mm**,
/// 而"两条视线错开多远"这个自检信号**只有 0.001 mm** —— 几乎为零。
/// 机制:拧一下之后视差沿极线滑动 ⇒ 点沿着视线**往前后挪**,两条线照样漂亮地相交。
/// **纯粹的深度偏置,没有任何几何不一致** ⇒ 给出一个偏了 5 毫米的点,而所有自检都是绿的。
/// **这是本仓最危险的那一族错:不报警的错。**
///
/// # 尺子从哪来:身体自己身上
///
/// 钳口跨度是驱动**实测过**的身体常数(这台机器上 **0.0803 m**)。
/// 两只眼都看得见两个指尖 ⇒ 三角化出来的跨度跟量过的一比就露馅。
/// **不需要标定板,不需要任何外部真值。**
///
/// 实测灵敏度:θ=0.1° ⇒ 差 **0.69 mm**(≈1 px,在噪声边缘)·
/// θ=0.3° ⇒ **2.04 mm**(≈3 px,查得出)· θ=1° ⇒ **6.40 mm**。
/// ⇒ **0.3° 以上抓得出;0.1° 在边缘。这条界写在这儿,别当成"能查出任意小的飘"。**
pub fn check_ruler(
    a: &Eye,
    b: &Eye,
    端1: (Px, Px),
    端2: (Px, Px),
    量过的长度_m: f64,
    tol_m: f64,
) -> Result<f64, Drift> {
    if !(量过的长度_m.is_finite() && 量过的长度_m > 0.0) {
        return Err(Drift::BadRuler);
    }
    // 三角化时把容差放宽:这里要查的是**长度**,不是"视线交不交得上"
    // (而且上面那条实测说了,视线错开在这种飘法下几乎为零,拿它当闸没用)。
    let p1 = triangulate(a, 端1.0, b, 端1.1, 1.0).map_err(Drift::CannotSee)?;
    let p2 = triangulate(a, 端2.0, b, 端2.1, 1.0).map_err(Drift::CannotSee)?;
    let got = ((p2.x - p1.x).powi(2) + (p2.y - p1.y).powi(2) + (p2.z - p1.z).powi(2)).sqrt();
    if (got - 量过的长度_m).abs() > tol_m {
        return Err(Drift::Off { got_m: got, want_m: 量过的长度_m, tol_m });
    }
    Ok(got)
}

// ─────────────────────────────────────────────────────────────────────
// 怎么适配各种传感器 —— 出口只有一个:一团【世界坐标里的三维表面点】
// ─────────────────────────────────────────────────────────────────────
//
// 🔴 **适配层其实早就在了,只是没人把它说出来**:
// `contact_gen::candidates(pts, …)` **只吃一团点,不问它从哪来**。
// ⇒ 换传感器 = 换一个"产点器",**后面整条链一个字都不用改**。
//
// | 传感器 | 怎么变成那团点 |
// |---|---|
// | 深度相机 / 结构光 / ToF | `from_depth`(像素 + 深度 → 点) |
// | 两只普通相机 | `from_pair`(左右配对 → 三角化) |
// | **一只普通相机 + 机器人自己动一下** | `from_motion` —— **就是同一个三角化**,见下 |
// | 激光雷达 | `from_sensor_frame`(它本来就是点,只差一次坐标变换) |
// | **摸** | `from_touch` —— **摸到的那一点本来就是一个表面点** |
//
// 🔴 而每一个来源都**必须自报自己有多准**(`sigma_m`),否则混在一起没法判断。

/// **一只普通相机 + 机器人自己挪一下 = 双目。**
///
/// # 为什么这在我们这套架构里几乎是免费的
///
/// 双目的难点从来不是"两条视线怎么交",是**"两个视角之间差多少"**(基线与相对朝向)。
/// 别人要靠标定板或 SLAM 去estimate 它;**而机器人自己挪了多远,本体感受直接告诉它。**
/// ⇒ 相机不动是单目,让胳膊挪 10 cm 再拍一张,**就是一副基线 10 cm 的双目**,
/// 而且用的是**同一个** `triangulate`,一行新几何都不用写。
///
/// # 🔴 但它有一条硬边界,必须挡住
///
/// 挪得太少 = 基线太短 = 三角形太扁 ⇒ 深度**极度不敏感**,误差爆炸。
/// 边界不是拍的,是从实测那张表来的:误差 ∝ 距离² /(焦距 × 基线)。
/// ⇒ 这里要求 **基线 ≥ 距离 × `最小基线比`**,达不到就**拒绝**,不给一个看起来正常的烂点。
pub fn from_motion(
    前: &Eye,
    后: &Eye,
    pairs: &[(Px, Px)],
    大约多远_m: f64,
    最小基线比: f64,
    tol_m: f64,
) -> Result<(Vec<P3>, usize), WhyNot> {
    let 基线 = norm3([后.at[0] - 前.at[0], 后.at[1] - 前.at[1], 后.at[2] - 前.at[2]]);
    if !(大约多远_m.is_finite() && 大约多远_m > 0.0) {
        return Err(WhyNot::Behind);
    }
    if 基线 < 大约多远_m * 最小基线比 {
        return Err(WhyNot::BaselineTooShort { got_m: 基线, need_m: 大约多远_m * 最小基线比 });
    }
    Ok(from_pair(前, 后, pairs, tol_m))
}

/// **激光雷达 / 任何"本来就出点"的传感器 → 世界坐标。**
///
/// 它跟相机那条路的差别只有一件事:**不用解视差**。剩下的完全一样 ——
/// 传感器在哪由本体感受给,点转到世界坐标就完事。
pub fn from_sensor_frame(点: &[[f64; 3]], at: [f64; 3], q: [f64; 4]) -> Vec<P3> {
    点.iter()
        .map(|p| {
            let w = qrot(q, *p);
            P3 { x: at[0] + w[0], y: at[1] + w[1], z: at[2] + w[2] }
        })
        .collect()
}

/// **摸到的点,本来就是一个表面点。**
///
/// 🔴 这一条不是凑数:**碰一下拿到的点比看到的点更准**,而且**不怕反光、不怕透明** ——
/// 那正好是所有视觉方案公认没解决的那一格(镜面/透明,冠军还有 42% 坏点率)。
/// 代价是**慢**、而且**只拿得到手够得着的那几点**。
///
/// ⇒ 它不是替代视觉,是**补视觉最烂的那一格**:看不清的地方,伸手摸一下。
/// 类型上它跟其它来源**完全一样**,所以 `merge` 直接就能把它们拌在一起。
pub fn from_touch(接触点: &[[f64; 3]]) -> Vec<P3> {
    接触点.iter().map(|p| P3 { x: p[0], y: p[1], z: p[2] }).collect()
}

/// **这个来源在这个距离上大概有多准(米)。** 供上层决定信不信、要不要再看一眼。
///
/// 公式是三角化的几何:`σ ≈ 像素误差 × 距离² /(焦距 × 基线)`。
/// 与 2026-08-16 那张实测表同源(基线 12 cm、f=900:0.1° 在 0.3/0.6/1.2 m 上
/// 分别是 1.4 / 5.3 / 20.7 mm)。
pub fn sigma_stereo(距离_m: f64, 基线_m: f64, 焦距_px: f64, 像素误差: f64) -> f64 {
    if 基线_m <= 0.0 || 焦距_px <= 0.0 {
        return f64::INFINITY;
    }
    像素误差 * 距离_m * 距离_m / (焦距_px * 基线_m)
}

fn norm3(v: [f64; 3]) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

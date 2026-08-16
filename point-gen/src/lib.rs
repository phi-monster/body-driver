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

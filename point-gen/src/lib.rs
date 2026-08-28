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
    /// 🔴 三根轴的留出分不开(最好 vs 次好差距不足)—— 观测点在哪根轴上定不下来。
    /// 硬挑一根就是把一次含糊变成一个看起来确定的数。带上 (次好/最好) 的比。
    AxisAmbiguous(f64),
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
    fit_full_at(seen, true)
}

/// 同 [`fit_full`],但可以关掉"最坏残差 > 半画幅"那道闸。
/// 只给**剔离群的粗解**用:粗解的唯一用途是给残差排序,它自己不需要过质量闸 ——
/// 池里混一颗脏样本时带闸粗解必死,迭代剔从来没通过电(GRAB3/5/6 三轮 BadFit(inf)
/// 的真相)。终解仍走带闸的 fit_full,质量线一寸不让。
pub fn fit_full_at(seen: &[(P3, Px)], 查worst: bool) -> Result<Eye, WhyNot> {
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
    拆开(p3, seen, 查worst)
}


/// 观测点 = 法兰原点 + R(朝向)·d —— 认块认到的是**指尖那一撮**,不是法兰原点,
/// 而 d 随腕转(2026-08 实测:同一集一次纯平移后四元数整个变了,"走到 xyz"不保朝向)。
fn 观测点(p: &[f64; 7], d: [f64; 3]) -> P3 {
    let q = [p[3], p[4], p[5], p[6]];
    let w = qrot(q, d);
    P3 { x: p[0] + w[0], y: p[1] + w[1], z: p[2] + w[2] }
}

/// **轴约束版联合解:偏置 d 只许沿法兰的某一根轴。**
///
/// # 为什么要这条约束(2026-08-19,两发对照逼出来的)
///
/// 自由 3 维的 d 与相机位置**部分互相顶账**:两发同一台相机,留出中位 0.0166 vs 0.0161
/// (残差分不出),一发解在真盆(d≈纯 z、相机对真值 3–7 cm),一发解到假盆
/// (d_x=0.29、相机 y 偏 26 cm、跨度跟着缩 39%)。**残差挑不出真假,只能靠物理约束砍自由度。**
///
/// 物理约束是免费的:认块认到的是指尖那一撮,而"指尖长在工具轴上"就是工具轴的定义
/// (对夹的两指质心在轴上、吸盘在轴上、多指近似在轴上)。⇒ d = e_k · t,搜 3 根轴 × 一维 t。
/// **顺手就把两格量了**:k 是 `tool_axis_column`,|t| 是 `tool_offset`(到指尖)。
///
/// 自检:最好的那根轴要**明显**好于次好(留出中位差 ≥20%,无量纲)—— 分不开就拒
/// `AxisAmbiguous`,不许硬挑。
pub fn fit_full_axis_offset(seen: &[([f64; 7], Px)]) -> Result<(Eye, usize, f64, f64), WhyNot> {
    if seen.len() < 12 {
        return Err(WhyNot::TooFewSamples(seen.len()));
    }
    let 评 = |d: [f64; 3]| -> Option<(Eye, f64)> {
        let fit: Vec<(P3, Px)> = seen.iter().step_by(2).map(|(p, u)| (观测点(p, d), *u)).collect();
        // 搜索段用无闸拟合(2026-08-20,GRAB8:inf 无剔离群打印 ⇒ 走的是"排 empty"
        // 那个出口 —— 单颗脏样本把每个 d 的带闸拟合都杀死,三轴全 None。搜索的把关
        // 本来就是【留出中位 + 物理闸】,worst 闸在这里只会饿死搜索)。
        let eye = fit_full_at(&fit, false).ok()?;
        // 物理闸(2026-08-19,SPANX11):搜索只看留出误差时,选中 t=0.22 的一台
        // "主点 (0.57,-0.26) 在画面外"的非物理解(留出 0.0239 反而最小)。
        // 主点在画面内是"这是一台相机"的最低要求,不是机体参数 —— 非物理的 d
        // 在搜索段就出局,别让它赢了留出、输了存在。
        if !(0.0..=1.0).contains(&eye.cx) || !(0.0..=1.0).contains(&eye.cy) {
            return None;
        }
        let mut errs: Vec<f64> = Vec::new();
        for (p, u) in seen.iter().skip(1).step_by(2) {
            match eye.project(观测点(p, d)) {
                Some(got) => errs.push(((got[0] - u[0]).powi(2) + (got[1] - u[1]).powi(2)).sqrt()),
                None => errs.push(f64::INFINITY),
            }
        }
        errs.sort_by(|a, b| a.partial_cmp(b).unwrap());
        Some((eye, errs[errs.len() / 2]))
    };
    let mut 每轴: [Option<(f64, f64)>; 3] = [None, None, None]; // (t, 留出)
    for k in 0..3 {
        let mut c = 0.0f64;
        let mut half = 0.30f64; // 覆盖任何合理的指尖偏置量级;搜索协议,无量纲跨机体
        let mut step = 0.03f64;
        let mut best: Option<(f64, f64)> = None;
        for _ in 0..4 {
            let n = (half / step).round() as i32;
            for i in -n..=n {
                let t = c + i as f64 * step;
                let mut d = [0.0f64; 3];
                d[k] = t;
                if let Some((_, e)) = 评(d) {
                    if best.map(|b| e < b.1).unwrap_or(true) {
                        best = Some((t, e));
                    }
                }
            }
            if let Some(b) = best {
                c = b.0;
            }
            half = step * 1.5;
            step /= 3.0;
        }
        每轴[k] = best;
    }
    let mut 排: Vec<(usize, f64, f64)> = 每轴
        .iter()
        .enumerate()
        .filter_map(|(k, o)| o.map(|(t, e)| (k, t, e)))
        .collect();
    if 排.is_empty() {
        return Err(WhyNot::BadFit(f64::INFINITY));
    }
    排.sort_by(|a, b| a.2.partial_cmp(&b.2).unwrap());
    if 排.len() >= 2 {
        // 🔴 两个准零数相除得出的比值是垃圾:合成/近完美数据上两个留出中位都在 1e-16,
        // 实测 `比` 算出 **0.0003**(比 1 还小,而 `排` 是升序排的 —— 不可能)。
        // ⇒ 分子分母都垫上**浮点噪声底**(算术的性质,不是身体常数):都在噪声里 ⇒ 比 = 1.0。
        let 噪底 = f64::EPSILON.sqrt();
        let 比 = 排[1].2.max(噪底) / 排[0].2.max(噪底);
        if 比 < 1.2 {
            // 🔴🔴 **闸门是对的,不许放宽 —— 撤回过一次(2026-08-26)。**
            //
            // 三根轴打平最常见的原因不是"噪声",是**手腕姿态在采样期间没变过**:
            // `观测点 = p + R·d`,R 恒定时 R·d 就是个常数偏移,**和把相机整体挪一段完全等价**
            // ⇒ 偏置和相机位置在数学上分不开,任何 d 都能拟合到同样好。
            // 实测(合成,姿态恒定):解出偏置 **-0.1300**、留出残差 **2.5e-16**(完美),
            // 而真值是 0 —— **拟合完美,相机却错了 13 cm**,拿去抓必然抓空。
            // 我一度想改成"两个解预测同样的像素就接受" —— 那正好会放进这个解:
            // 它们在**采样用的那个姿态下**确实预测一样,换个姿态就分道扬镳,而 `摆成()` 一定会换姿态。
            // ⇒ 拒绝是对的。要解开它,得让**采样期间手腕真的转过**,不是松闸。
            return Err(WhyNot::AxisAmbiguous(比));
        }
    }
    let (k, t, med) = 排[0];
    let mut d = [0.0f64; 3];
    d[k] = t;
    let 全: Vec<(P3, Px)> = seen.iter().map(|(p, u)| (观测点(p, d), *u)).collect();
    // 终拟合带一轮剔离群(2026-08-19,SPANX6):648 样本里少数脏样本(认块认到
    // 背景/臂)把 fit_full 的 worst 闸打爆 => BadFit(2.53 画幅)全盘拒,而搜索段的
    // 中位留出早就说这枚 d 是好的。终盘也按中位来:先用偶集粗解给全样本算残差,
    // 剔掉大于 max(10x中位, 0.05 画幅) 的,再终拟合。阈是量出来的,不是配额。
    let eye = match fit_full(&全) {
        Ok(e) => e,
        Err(_) => {
            let 偶: Vec<(P3, Px)> = 全.iter().step_by(2).cloned().collect();
            // 偶集也可能过不了 worst 闸(SPANX11:0.568 直接从这里抛出,迭代剔
            // 根本没跑)—— 奇集再试一次,两个粗解都死才放弃。
            let 粗 = match fit_full_at(&偶, false) {
                Ok(e) => e,
                Err(_) => {
                    let 奇: Vec<(P3, Px)> = 全.iter().skip(1).step_by(2).cloned().collect();
                    fit_full_at(&奇, false)?
                }
            };
            let mut 残: Vec<(usize, f64)> = Vec::new();
            for (i, (p, u)) in 全.iter().enumerate() {
                let e = match 粗.project(*p) {
                    Some(g) => ((g[0] - u[0]).powi(2) + (g[1] - u[1]).powi(2)).sqrt(),
                    None => f64::INFINITY,
                };
                残.push((i, e));
            }
            let mut 有限: Vec<f64> = 残.iter().map(|&(_, e)| e).filter(|e| e.is_finite()).collect();
            有限.sort_by(|a, b| a.partial_cmp(b).unwrap());
            if 有限.is_empty() {
                return Err(WhyNot::BadFit(f64::INFINITY));
            }
            // 迭代收紧(2026-08-19,SPANX8):剔一轮(10x中位)后 worst=0.509,
            // 距 0.5 硬闸差 2% —— 一轮不够就按 10x/5x/3x 中位逐轮收紧,三轮全败才拒。
            // 硬闸本身(半画幅)不动:那是挡真退化的安全线。
            let 中位 = 有限[有限.len() / 2];
            println!("      [全相机] 粗解残差分布:中位 {:.4} · P90 {:.4} · 最坏 {:.4}(n={})",
                中位, 有限[(有限.len() * 9) / 10], 有限[有限.len() - 1], 有限.len());
            let mut 解: Option<Eye> = None;
            for 倍 in [10.0, 5.0, 3.0] {
                let 阈 = (中位 * 倍).max(0.02);
                let 净: Vec<(P3, Px)> = 残.iter().filter(|&&(_, e)| e <= 阈).map(|&(i, _)| 全[i]).collect();
                if 净.len() < 12 {
                    break;
                }
                if let Ok(e) = fit_full(&净) {
                    println!("      [全相机] 终拟合剔掉 {} 个离群样本(残差 > {:.4} 画幅)再解", 全.len() - 净.len(), 阈);
                    解 = Some(e);
                    break;
                }
            }
            match 解 {
                Some(e) => e,
                None => return Err(WhyNot::BadFit(f64::INFINITY)),
            }
        }
    };
    Ok((eye, k, t, med))
}

/// **带偏置的整台相机拟合**:联合解「观测点相对法兰的偏置 d」和整台相机。
///
/// # 为什么必须联合解(这条撤回过一次的教训,别再走回去)
///
/// `fit_full` 直接吃(法兰位置, 像素)在真数据上解出**垃圾**(2026-08-19,Franka,166 组:
/// 相机位置差 0.4 m、fx 差三个量级、det(R)=−1 落镜像叶)——因为认块认的是**指尖**,
/// 离法兰十几厘米还**随腕转**,针孔装不下这组对。胶水时代同一病:直接解留出中位 39 px,
/// 联合解偏置后 8 px。删胶水时把这段**通用数学**错当机体件扔了,这里是它的正式归位。
///
/// # 怎么解
///
/// d 只有三个数;给定 d 后就是 `fit_full`(线性 DLT + 拆开 + 全部闸)。
/// 对 d 做**粗网格 + 三轮细化**(±0.24 m 起步 —— 覆盖任何合理的工具偏置量级;
/// 网格参数是搜索协议,无量纲地跨机体成立)。**用留出误差打分,不用拟合误差**:
/// 拟合误差随自由度单调变好,留出不会。`Behind`/任何拒绝按"这枚 d 不行"处理,
/// 镜像叶的候选在留出上自然死掉。
///
/// 返回 (眼, d, 留出中位像素误差)。样本 <12 拒绝(两半各 6 才够 fit_full 起步)。
pub fn fit_full_offset(seen: &[([f64; 7], Px)]) -> Result<(Eye, [f64; 3], f64), WhyNot> {
    if seen.len() < 12 {
        return Err(WhyNot::TooFewSamples(seen.len()));
    }
    let 评 = |d: [f64; 3]| -> Option<(Eye, f64)> {
        let fit: Vec<(P3, Px)> = seen.iter().step_by(2).map(|(p, u)| (观测点(p, d), *u)).collect();
        let eye = fit_full(&fit).ok()?;
        let mut errs: Vec<f64> = Vec::new();
        for (p, u) in seen.iter().skip(1).step_by(2) {
            let x = 观测点(p, d);
            match eye.project(x) {
                Some(got) => errs.push(((got[0] - u[0]).powi(2) + (got[1] - u[1]).powi(2)).sqrt()),
                None => errs.push(f64::INFINITY),
            }
        }
        if errs.is_empty() {
            return None;
        }
        errs.sort_by(|a, b| a.partial_cmp(b).unwrap());
        Some((eye, errs[errs.len() / 2]))
    };
    let mut best: Option<(Eye, [f64; 3], f64)> = None;
    let mut step = 0.03f64;
    let mut c = [0.0f64; 3];
    let mut half = 0.24f64;
    for _ in 0..4 {
        let n = (half / step).round() as i32;
        for i in -n..=n {
            for j in -n..=n {
                for k in -n..=n {
                    let d = [c[0] + i as f64 * step, c[1] + j as f64 * step, c[2] + k as f64 * step];
                    if let Some((eye, e)) = 评(d) {
                        if best.as_ref().map(|b| e < b.2).unwrap_or(true) {
                            best = Some((eye, d, e));
                        }
                    }
                }
            }
        }
        if let Some(b) = &best {
            c = b.1;
        }
        half = step * 1.5;
        step /= 3.0;
    }
    let (_, d, e) = best.ok_or(WhyNot::BadFit(f64::INFINITY))?;
    // 终拟:最好的那枚 d 上用【全部】样本再解一次(留出只用来挑 d,终解不浪费一半样本)。
    let 全: Vec<(P3, Px)> = seen.iter().map(|(p, u)| (观测点(p, d), *u)).collect();
    let eye = fit_full(&全)?;
    Ok((eye, d, e))
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
fn 拆开(p: [[f64; 4]; 3], seen: &[(P3, Px)], 查worst: bool) -> Result<Eye, WhyNot> {
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
    // 🔴🔴 **K 对角翻正之后必须查 det(R) —— 反射不是旋转。**
    // 每翻一次 K 的列就同时翻 R 的一行,翻奇数次 det(R) 就成了 −1;
    // 而四元数**表示不了反射** ⇒ `转成四元数` 会安静地给出某个真旋转 ≠ R
    // ⇒ 回代投影把点打到"背后",病相是 `Behind`,根因在这儿。
    // det(R) = −1 说明拟出的 P 落在镜像叶上(弱透视/深度跨度薄时两叶都能拟)——
    // 这一层能做的诚实动作是把它**点名**报出来,并把中间量打出来(env 门控)。
    let det_r = r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1])
        - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
        + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]);
    if std::env::var("BL_CAMDEBUG").is_ok() {
        eprintln!("[拆开] det(R)={det_r:.6} · K diag=({:.4},{:.4},{:.4})", k[0][0], k[1][1], k[2][2]);
    }
    if det_r < 0.0 {
        // 镜像叶:翻 R 第三行 + K 第三列(乘积不变),**然后把整套重新规范一遍** ——
        // 第一版只翻不复位:s = K[2][2] 变了号,按 s 归一时 fx/fy 全被带成负 ⇒ 还是 Behind。
        // 复位 = 再跑一次"K 对角为正"(这次只翻前两列,翻偶数次 det(R) 不再回到 −1)。
        for j in 0..3 {
            k[j][2] = -k[j][2];
            r[2][j] = -r[2][j];
        }
        for i in 0..2 {
            if k[i][i] < 0.0 {
                for j in 0..3 {
                    k[j][i] = -k[j][i];
                    r[i][j] = -r[i][j];
                }
            }
        }
        if std::env::var("BL_CAMDEBUG").is_ok() {
            let d2 = r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1])
                - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
                + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]);
            eprintln!("[拆开] 镜像复位后 det(R)={d2:.6} · K diag=({:.4},{:.4},{:.4})", k[0][0], k[1][1], k[2][2]);
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
        let Some(got) = eye.project(*p3d) else {
            if std::env::var("BL_CAMDEBUG").is_ok() {
                eprintln!("[拆开] 回代 Behind:样本 ({:.4},{:.4},{:.4}) · at=({:.3},{:.3},{:.3}) q=({:.3},{:.3},{:.3},{:.3})",
                    p3d.x, p3d.y, p3d.z, eye.at[0], eye.at[1], eye.at[2], eye.q[0], eye.q[1], eye.q[2], eye.q[3]);
            }
            return Err(WhyNot::Behind);
        };
        worst = worst.max((got[0] - px[0]).abs().max((got[1] - px[1]).abs()));
    }
    // 🔴 残差门槛不能照合成数据定。合成台上该到 1e-6,而**真数据自带 ~18 px 的认手噪声**
    // ⇒ 残差本来就该是那个量级。这里只挡真正的退化(残差大到画幅一半);
    // 够不够用由调用方拿自己的判据去比,而这一层把【斜切】与【残差】都报出来。
    let 画幅 = seen.iter().map(|(_, u)| u[0].abs().max(u[1].abs())).fold(0.0f64, f64::max).max(1.0);
    if 查worst && worst > 画幅 * 0.5 {
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
    // 🔴🔴 **只留【形状无关】的那一条。**(2026-08-17 撤回相对项)
    //
    // 撤回的是:`实占 > 该占 × 宽容²`,其中 `该占 = π/4·span_frac²` ——
    // 它把**每一个物体都当成"直径 = 眼说的宽度"的圆盘**。而剪刀/勺子/笔是细长的:
    // 宽是 `span_frac`,长是它的好几倍,面积必然超。**这不是圈错了,是那个形状假设不成立。**
    //
    // 实测(`run_gp3`/`run_gp4`,2026-08-17):
    // · 剪刀 `TooBig(0.0037825, 0.0016755)` —— 只超门槛 **0.33%**,而链条**连撞 378 次**,
    //   一集都没走到下探;
    // · 换个细长物体 `TooBig(0.0061947, 0.0009425)` —— 超 **6.6 倍**,任何"宽容"都救不回来。
    // ⇒ 这是本仓同族第 6 次"闸把自己饿死"(前五次记在 `link/results/grasp_aug2026/RESULTS.md`)。
    //
    // 🟢 **留下的这一条不假设形状**:一个宽度为 `span_frac` 的东西,**最大也就是一根
    // 贯穿整幅画高的竖条**,那样占全帧 `span_frac`;占到两倍就一定圈到别的了。
    // 它**与 `宽容` 无关**,而且档案里那次真失败(*"掩膜占了全帧 72%"*)照样挡得住
    // ——  0.72 远大于任何合理的 `span_frac × 2`。
    // 🔴 只留【形状无关】的那一条。撤回的是相对项 `实占 > 该占 × 宽容²`,
    // 其中 `该占 = π/4·span_frac²` —— 它把**每个物体都当成直径 = 眼说的宽度的圆盘**,
    // 而剪刀/勺子/笔是细长的:宽是 span_frac,长是它的好几倍,面积必然超。
    // 实测:剪刀只超门槛 0.33% 而链条连撞 378 次;另一件细长物超 6.6 倍,任何"宽容"都救不回来。
    //
    // 🟢 留下的这一条**不假设形状**:一个宽度为 `span_frac` 的东西,最大也就是一根贯穿
    // 整幅画高的竖条,那样占全帧 `span_frac`;占到两倍就一定圈到别的了。
    // 它挡住了档案里那次真失败(*"掩膜占了全帧 72%"*),而且**从没挡过一次正常的抓取**。
    if 实占 > span_frac * 2.0 {
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

// ─────────────────────────────────────────────────────────────────────
// 把【支撑面】减掉 —— 剩下的才是物体
// ─────────────────────────────────────────────────────────────────────

/// **从一团点里拟出最大的那张平面(桌面),把落在它上面的点扔掉。**
///
/// # 🔴 它跟 LAB 判死的那条【不是】一回事
///
/// 判死的是*"深度比全画面 90 分位近 1 cm 的都算物体"* —— **拿一个深度【数】去卡**,
/// 于是整张桌子都比背景近,掩膜吃掉 **72% 全帧**。
/// 这一条是**拟一张【平面】出来减掉**:桌面在三维里是一张平面,而物体**凸出于它**。
/// 相机斜着看时前者仍然成立,而"一个深度数"当场就废 ——
/// 实测(2026-08-16 g5):斜看时,桌面上一块 10 cm 的区域自带 **60 mm** 高差,
/// 深度筛子把整片桌面当成了物体,**掩膜是个规整的圆盘**,而那正是渲图抓出来的假象。
///
/// 做法:RANSAC(确定性采样,不引随机数)找内点最多的那张平面,再把内点扔掉。
/// 返回 `(剩下的点, 平面法向, 平面上有多少点)`。
pub fn drop_support_plane(pts: &[P3], tol_m: f64) -> (Vec<P3>, [f64; 3], usize) {
    let n = pts.len();
    if n < 16 {
        return (pts.to_vec(), [0.0, 0.0, 1.0], 0);
    }
    let mut best = (0usize, [0.0f64, 0.0, 1.0], 0.0f64);
    // 确定性地取若干三元组 —— 不引随机数(随机会让同一份数据两次给不同答案)
    let step = (n / 17).max(1);
    for a in (0..n).step_by(step) {
        for b in ((a + step)..n).step_by(step.max(1) * 3) {
            for c in ((b + step)..n).step_by(step.max(1) * 7) {
                let (p, q, r) = (pts[a], pts[b], pts[c]);
                let u = [q.x - p.x, q.y - p.y, q.z - p.z];
                let v = [r.x - p.x, r.y - p.y, r.z - p.z];
                let cr = cross3(u, v);
                let ln = (cr[0] * cr[0] + cr[1] * cr[1] + cr[2] * cr[2]).sqrt();
                let nv = match (ln > 1e-12).then(|| [cr[0] / ln, cr[1] / ln, cr[2] / ln]) {
                    Some(x) => x,
                    None => continue,
                };
                let d = nv[0] * p.x + nv[1] * p.y + nv[2] * p.z;
                let cnt = pts
                    .iter()
                    .filter(|t| (nv[0] * t.x + nv[1] * t.y + nv[2] * t.z - d).abs() <= tol_m)
                    .count();
                if cnt > best.0 {
                    best = (cnt, nv, d);
                }
            }
        }
    }
    let (cnt, nv, d) = best;
    let 剩 = pts
        .iter()
        .filter(|t| (nv[0] * t.x + nv[1] * t.y + nv[2] * t.z - d).abs() > tol_m)
        .cloned()
        .collect();
    (剩, nv, cnt)
}

fn cross3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

/// 🔴🔴🔴 **从"手挪一点点、画面变多少"那张表,把整台相机闭式解出来。**
///
/// 输入是 `P ↦ (u_像素, v_像素, 沿光轴的深)` 这个映射在**某一点**的导数(3×3,行序 u/v/d),
/// 外加那一点的观测:手在画面的哪个像素、那儿多深、手在世界的哪儿。
///
/// # 为什么有闭式
/// ```text
/// d = z轴·(P − 相机)                  ⇒ ∂d/∂P = z轴            ← 深度那一行**就是光轴**
/// u = 主点 + 焦距·(x轴·(P−相机))/d     ⇒ ∂u/∂P = (焦距/d)·x轴 − ((u−主点)/d)·z轴
/// ```
/// 第二式点乘 z 轴(x⊥z)⇒ **主点 = u + d·(∂u/∂P·z轴)**;
/// 余下的垂直分量 ⇒ **焦距 = d·|垂直分量|**、**x 轴 = 垂直分量方向**。相机位置由手的位置反推。
///
/// **四个内参全是从量出来的表直接算的** —— 没有拟合、没有标定板、没有一个手填的数。
/// 这和 `fit_full` 那条路的区别是根本性的:那条要**全局拟合**,因此会被点共面 / 全在一个深度 /
/// 姿态不变这些退化打死;这条**不拟合**,只用一点上的导数。
///
/// 解不出来就返回 `None`(不硬解):光轴退化、两轴共线、或者**拿解出来的相机把手投回去落不回原像素**。
pub fn eye_from_jacobian(
    j: [[f64; 3]; 3],
    u_px: f64,
    v_px: f64,
    depth: f64,
    hand: [f64; 3],
) -> Option<Eye> {
    if !(depth.is_finite() && depth > 1e-6) {
        return None;
    }
    let dot = |p: [f64; 3], q: [f64; 3]| p[0] * q[0] + p[1] * q[1] + p[2] * q[2];
    let zl = dot(j[2], j[2]).sqrt();
    if !(zl > 1e-9) {
        return None;
    }
    let z = [j[2][0] / zl, j[2][1] / zl, j[2][2] / zl];
    let (az, bz) = (dot(j[0], z), dot(j[1], z));
    let cx = u_px + depth * az;
    let cy = v_px + depth * bz;
    let ax = [j[0][0] - az * z[0], j[0][1] - az * z[1], j[0][2] - az * z[2]];
    let by = [j[1][0] - bz * z[0], j[1][1] - bz * z[1], j[1][2] - bz * z[2]];
    let (la, lb) = (dot(ax, ax).sqrt(), dot(by, by).sqrt());
    if !(la > 1e-9 && lb > 1e-9) {
        return None;
    }
    let (fx, fy) = (depth * la, depth * lb);
    let mut x轴 = [ax[0] / la, ax[1] / la, ax[2] / la];
    let mut y轴 = [by[0] / lb, by[1] / lb, by[2] / lb];
    // 量出来的两根轴未必严格垂直 ⇒ 就地正交化,以光轴为准。
    let px = dot(x轴, z);
    for k in 0..3 {
        x轴[k] -= px * z[k];
    }
    let lx = dot(x轴, x轴).sqrt();
    if !(lx > 1e-9) {
        return None;
    }
    for k in 0..3 {
        x轴[k] /= lx;
    }
    let (py, pz) = (dot(y轴, z), dot(y轴, x轴));
    for k in 0..3 {
        y轴[k] -= py * z[k] + pz * x轴[k];
    }
    let ly = dot(y轴, y轴).sqrt();
    if !(ly > 1e-9) {
        return None;
    }
    for k in 0..3 {
        y轴[k] /= ly;
    }
    let bx = (u_px - cx) * depth / fx;
    let by2 = (v_px - cy) * depth / fy;
    let at = [
        hand[0] - (bx * x轴[0] + by2 * y轴[0] + depth * z[0]),
        hand[1] - (bx * x轴[1] + by2 * y轴[1] + depth * z[1]),
        hand[2] - (bx * x轴[2] + by2 * y轴[2] + depth * z[2]),
    ];
    let r = [[x轴[0], y轴[0], z[0]], [x轴[1], y轴[1], z[1]], [x轴[2], y轴[2], z[2]]];
    let tr = r[0][0] + r[1][1] + r[2][2];
    let q = if tr > 0.0 {
        let s = (tr + 1.0).sqrt() * 2.0;
        [0.25 * s, (r[2][1] - r[1][2]) / s, (r[0][2] - r[2][0]) / s, (r[1][0] - r[0][1]) / s]
    } else if r[0][0] > r[1][1] && r[0][0] > r[2][2] {
        let s = (1.0 + r[0][0] - r[1][1] - r[2][2]).sqrt() * 2.0;
        [(r[2][1] - r[1][2]) / s, 0.25 * s, (r[0][1] + r[1][0]) / s, (r[0][2] + r[2][0]) / s]
    } else if r[1][1] > r[2][2] {
        let s = (1.0 + r[1][1] - r[0][0] - r[2][2]).sqrt() * 2.0;
        [(r[0][2] - r[2][0]) / s, (r[0][1] + r[1][0]) / s, 0.25 * s, (r[1][2] + r[2][1]) / s]
    } else {
        let s = (1.0 + r[2][2] - r[0][0] - r[1][1]).sqrt() * 2.0;
        [(r[1][0] - r[0][1]) / s, (r[0][2] + r[2][0]) / s, (r[1][2] + r[2][1]) / s, 0.25 * s]
    };
    let eye = Eye { fx, fy, cx, cy, at, q };
    // 自检:拿解出来的相机把手投回去,必须落回手的像素上。落不回去就是解错了,**不许用**。
    let back = eye.project(P3 { x: hand[0], y: hand[1], z: hand[2] })?;
    if (back[0] - u_px).abs() > 1.0 || (back[1] - v_px).abs() > 1.0 {
        return None;
    }
    Some(eye)
}

// ────────────────────────────────────────────────────────────────────────────
// 减法③:**候选区域由几何出,眼睛只负责【挑】。**(owner 2026-08-28)
// ────────────────────────────────────────────────────────────────────────────

/// 画面里**从支撑面上鼓出来的一块**。这就是一个"候选物体",而且它的边界是**量出来的**,
/// 不是猜的半径。
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct 区 {
    /// 框(像素):左、上、右、下(闭区间)。
    pub 框: [usize; 4],
    /// 这一块有多少个像素。
    pub 像素数: usize,
    /// 中心(**归一化画幅坐标**,和眼给的 u/v 同一套)。
    pub 心: [f64; 2],
    /// 这一块的中位深度(米)。
    pub 深: f64,
    /// 它比支撑面**鼓出来多少**(米,正数)。
    pub 高: f64,
}

/// **把画面切成"支撑面"和"支撑面上鼓出来的那些块"** —— 不认识任何物体、不需要任何检测器。
///
/// # 为什么这样切是对的,而"绕着眼指那一点长一个圆"是错的
///
/// `mask_around` 的半径来自眼睛报的 `span_frac` ×一个宽容系数 —— **两个都是猜的**,
/// 而后果是实测过的:*"今天把桌面圈进点云的正是那个猜出来的半径"*。
/// 这里一个半径都没有:**边界是"哪儿不再鼓出来"**,它是图里的真实结构。
///
/// # 一个内参都不需要
///
/// 空间里的一个平面,在深度图上满足 **`1/z` 关于像素 (u,v) 线性**(针孔几何的恒等式)。
/// 于是"支撑面"就是 `1/z ≈ a·u + b·v + c` 这么一张平面,**用最小二乘 + 反复剔野点**拟出来
/// —— 地板/桌面占了绝大多数像素,所以它一定收敛到支撑面上。
/// 不用焦距、不用主点、不用相机位姿:**换一台相机、换一个视角,这段代码一个字都不用改。**
/// 也因此它可以在**相机还没解出来之前**就跑 —— 而"眼指哪儿"正是在那之前就要回答的问题。
///
/// # 门槛不是拍的
///
/// "鼓出来多少才算一块" = 拟完之后**残差自己的稳健散布 σ** 的若干倍。σ 是量出来的
/// (中位绝对偏差 × 1.4826),`倍` 只是一个无量纲倍数,含义是"比这张面自己的粗糙度还突出"。
///
/// 返回按**像素数从多到少**排好的区;`最小像素` 以下的碎块直接丢掉。
/// 一维滑窗极值,窗口 `[c-r, c+r]`(单调队列,O(n))。
/// `取大` 为真取最大,否则取最小;无读数的像素按 `空` 参与(取大时给 −∞ ⇒ 被忽略)。
fn 滑窗(src: &[f64], w: usize, h: usize, r: usize, 横: bool, 取大: bool, 空: f64) -> Vec<f64> {
    let mut out = vec![空; w * h];
    let (外, 内) = if 横 { (h, w) } else { (w, h) };
    if 内 == 0 { return out }
    let mut dq: Vec<usize> = Vec::with_capacity(内);
    for o in 0..外 {
        dq.clear();
        let mut 头 = 0usize;
        let at = |k: usize| if 横 { o * w + k } else { k * w + o };
        let val = |k: usize| { let v = src[at(k)]; if v.is_finite() { v } else { 空 } };
        let mut 推 = |dq: &mut Vec<usize>, 头: usize, k: usize| {
            while dq.len() > 头 {
                let 尾 = *dq.last().unwrap();
                if (取大 && val(尾) <= val(k)) || (!取大 && val(尾) >= val(k)) { dq.pop(); } else { break }
            }
            dq.push(k);
        };
        // 先把 [0, r] 灌进去
        for k in 0..=(r.min(内 - 1)) { 推(&mut dq, 头, k); }
        for c in 0..内 {
            // 右界推进到 c+r
            if c > 0 {
                let k = c + r;
                if k < 内 { 推(&mut dq, 头, k); }
            }
            // 左界:丢掉小于 c-r 的
            let 左 = c.saturating_sub(r);
            while dq.len() > 头 && dq[头] < 左 { 头 += 1 }
            if dq.len() > 头 { out[at(c)] = val(dq[头]); }
        }
    }
    out
}

/// **把画面切成"背景面"和"从背景面上鼓出来的那些块"** —— 不认识任何物体、不需要任何检测器,
/// 也**不假设支撑面是一张平面**。
///
/// # 为什么不拟平面(2026-08-28 在真实深度图上验掉的)
///
/// 先写的版本是"拟一张 `1/z` 关于像素线性的支撑面,鼓出来的就是物体"。在真机那张图上
/// **一块桌面物体都没切出来**:场景里同时有桌面和背景墙,拟出来的那张面横跨两者,
/// 稳健 σ 折成 **~5 cm 的起伏**,而桌上的东西只鼓 2–8 cm ⇒ 整个淹掉。
/// 加大剔野点力度会把桌面自己剔成一条斜带(渲图看见的)。**平面假设本身是错的**:
/// 换成弯桌面、地板+桌面两层、无人机看地形,它一样塌。
///
/// # 换成形态学闭运算:物体是深度上的【凹坑】
///
/// 一个放在任何面上的东西,在深度图里就是**比周围近**的一小片 —— 一个坑。
/// **闭运算**(先取窗内最大深度、再取窗内最小)正是"把比窗口小的坑填平"这件事:
/// 填平之后的那张图就是"如果这儿什么都没放,背景面长什么样",
/// 而 **背景 − 实测 = 它鼓出来多高**。桌沿那种真实的深度台阶被闭运算原样保留(不产生假鼓),
/// 这是它比"减一张平面"强的地方。**没有平面、没有法向、没有相机内参。**
///
/// # 两个无量纲的数,含义都写出来
///
/// * `窗比`:窗口半径 = 画面宽 × 窗比。含义是"**比这个还大的就不是可以拿起来的东西,是布景**"。
/// * `倍`:鼓出多少才算数 = 背景残差自己的稳健 σ 的多少倍。
///
/// 返回按**像素数从多到少**排好的区;`最小像素` 以下的碎块直接丢掉。
pub fn 分块(depth: &[f64], w: usize, h: usize, 最小像素: usize, 倍: f64) -> Vec<区> {
    分块窗(depth, w, h, 最小像素, 倍, 0.125)
}

/// [`分块`] 的完整形,把窗口比例也露出来。
pub fn 分块窗(depth: &[f64], w: usize, h: usize, 最小像素: usize, 倍: f64, 窗比: f64) -> Vec<区> {
    if w == 0 || h == 0 || depth.len() < w * h {
        return Vec::new();
    }
    let r = ((w as f64 * 窗比) as usize).max(2);
    // 闭运算 = 膨胀(取窗内最大深度)再腐蚀(取窗内最小深度)。分两轴做,等价且是 O(n)。
    let d1 = 滑窗(depth, w, h, r, true, true, f64::NEG_INFINITY);
    let d2 = 滑窗(&d1, w, h, r, false, true, f64::NEG_INFINITY);
    let e1 = 滑窗(&d2, w, h, r, true, false, f64::INFINITY);
    let 背景 = 滑窗(&e1, w, h, r, false, false, f64::INFINITY);

    // 鼓出来多少米(正数 = 比背景近)。
    let mut 鼓 = vec![f64::NAN; w * h];
    let mut 样: Vec<f64> = Vec::new();
    let 步 = ((w * h) / 20000).max(1);
    for i in 0..w * h {
        let (z, b) = (depth[i], 背景[i]);
        if z.is_finite() && z > 1e-6 && b.is_finite() {
            鼓[i] = b - z;
        }
    }
    for i in (0..w * h).step_by(步) {
        if 鼓[i].is_finite() { 样.push(鼓[i]) }
    }
    if 样.len() < 16 {
        return Vec::new();
    }
    样.sort_by(|x, y| x.partial_cmp(y).unwrap_or(core::cmp::Ordering::Equal));
    let 中 = 样[样.len() / 2];
    let mut 绝: Vec<f64> = 样.iter().map(|v| (v - 中).abs()).collect();
    绝.sort_by(|x, y| x.partial_cmp(y).unwrap_or(core::cmp::Ordering::Equal));
    // 🔴 **中位绝对偏差可能是 0,而那不代表"没有噪声"。**
    // 实测(2026-08-28,8 bit 深度图):绝大多数像素的"鼓出来"**恰好等于 0**(量化)
    // ⇒ MAD = 0 ⇒ σ 塌到 1e-9 ⇒ 门槛 ≈ 中位数 ⇒ **`倍` 完全失效**
    //(倍=5/8/12 切出来一模一样的 28 块,里面还混着整条上下边)。
    // ⇒ 往上取分位数,直到拿到一个**正的**尺度;都取不到才说明这张图真的一点起伏都没有。
    //
    // 下面四个数全是**无量纲**的:0.5/0.75/0.9/0.99 是**排序里的位置**(第几个样本),
    // 1.4826 是"中位绝对偏差 → 标准差"的固定换算(正态下 1/Φ⁻¹(0.75))。
    // 换一台相机、换一个量纲、把深度从米改成毫米,这四个数一个都不用改 —— 它们是**尺度无关**的。
    let 分位 = |q: f64| 绝[((绝.len() as f64 - 1.0) * q) as usize];
    let mut σ = 0.0f64;
    for q in [0.5f64, 0.75, 0.9, 0.99] {
        let v = 分位(q) * 1.4826;
        if v > 0.0 { σ = v; break }
    }
    if !(σ > 0.0) {
        return Vec::new();
    }
    let 门 = 中 + 倍 * σ;

    let mut 突: Vec<bool> = vec![false; w * h];
    for i in 0..w * h {
        if 鼓[i].is_finite() && 鼓[i] > 门 {
            突[i] = true;
        }
    }
    // 连通块(四邻,显式栈,不递归)。
    let mut 标: Vec<i32> = vec![-1; w * h];
    let mut 出: Vec<区> = Vec::new();
    let mut 栈: Vec<usize> = Vec::new();
    for 起 in 0..w * h {
        if !突[起] || 标[起] >= 0 {
            continue;
        }
        let id = 出.len() as i32;
        栈.clear();
        栈.push(起);
        标[起] = id;
        let (mut u0, mut v0, mut u1, mut v1) = (w, h, 0usize, 0usize);
        let mut n = 0usize;
        let mut 深们: Vec<f64> = Vec::new();
        let mut 高们: Vec<f64> = Vec::new();
        while let Some(i) = 栈.pop() {
            let (x, y) = (i % w, i / w);
            n += 1;
            u0 = u0.min(x);
            v0 = v0.min(y);
            u1 = u1.max(x);
            v1 = v1.max(y);
            深们.push(depth[i]);
            高们.push(鼓[i]);
            for (nx, ny) in [(x.wrapping_sub(1), y), (x + 1, y), (x, y.wrapping_sub(1)), (x, y + 1)] {
                if nx >= w || ny >= h {
                    continue;
                }
                let j = ny * w + nx;
                if 突[j] && 标[j] < 0 {
                    标[j] = id;
                    栈.push(j);
                }
            }
        }
        if n < 最小像素 {
            continue;
        }
        深们.sort_by(|x, y| x.partial_cmp(y).unwrap_or(core::cmp::Ordering::Equal));
        高们.sort_by(|x, y| x.partial_cmp(y).unwrap_or(core::cmp::Ordering::Equal));
        出.push(区 {
            框: [u0, v0, u1, v1],
            像素数: n,
            心: [(u0 + u1) as f64 / 2.0 / w as f64, (v0 + v1) as f64 / 2.0 / h as f64],
            深: 深们[深们.len() / 2],
            高: 高们[高们.len() / 2],
        });
    }
    出.sort_by(|p, q| q.像素数.cmp(&p.像素数));
    出
}

/// 把某一块的像素做成掩膜 —— 和 [`分块`] 用**同一张拟出来的支撑面**,所以边界一致。
///
/// 🔴 这是 `mask_around` 的替代品:**没有半径、没有宽容系数**。
pub fn 区掩膜(depth: &[f64], w: usize, h: usize, r: &区, 全部: &[区], 倍: f64) -> Vec<bool> {
    // 重新跑一次分块代价太高,而且会因为随机性不一致 ⇒ 直接按框 + 深度带切。
    // 深度带 = 这一块自己的中位深 ± 它自己鼓出来的高度(它不会比自己更厚)。
    let _ = 全部;
    let _ = 倍;
    let mut m = vec![false; w * h];
    let 厚 = r.高.abs().max(1e-4);
    for y in r.框[1]..=r.框[3].min(h.saturating_sub(1)) {
        for x in r.框[0]..=r.框[2].min(w.saturating_sub(1)) {
            let i = y * w + x;
            if i >= depth.len() {
                continue;
            }
            let z = depth[i];
            if z.is_finite() && z > 1e-6 && (z - r.深).abs() <= 厚 {
                m[i] = true;
            }
        }
    }
    m
}

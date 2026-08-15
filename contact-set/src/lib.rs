//! **接触集 —— 「脑子对身体说的那句话」本身,四格齐。**
//!
//! # 🔴 为什么它是一个独立的东西
//!
//! 它是 ②a(接触生成器)**产出**、②b(执行层)**消费**的那个词汇表。
//! 放进任何一边,另一边就得依赖那一边 —— 而 ②a 与 ②b 是平级的两个生产者。
//! 依赖表空着,和 `body-layer` 同一条理由:**这一层是要被整个领域读和审的**。
//!
//! `ARCH`/`DRIVER_GOAL` §1.2 原文:
//!
//! > **① 碰物体表面的哪几个点 · ② 每点的法向,和那里允许往哪使劲的【锥】(只给方向,不给大小)
//! > · ③【物体】要怎么动(一个旋量) · ④ 容差**
//!
//! 上一版(`lib.rs::Contact` + `Motion`)**四格没有一格是全的**:
//! ① 只有一个点 · ② 只有一个标量 `close_yaw`(没有法向、没有锥)
//! ③ 只有平移(说不出拧/撬/翻/倒)· ④ 完全没有。
//! 于是十三个动词里只落得下"抓 + 平移 + 放"那一格,而**这一格恰好就是老接口能说的那一格** ——
//! 换句话说,那一版没有买到任何表达力。
//!
//! # 🔴 这里一个字都不许提机体
//!
//! 没有"手指"、没有"钳口"、没有自由度数。**吸盘 = 1 个点 + 一个只允许法向的锥;
//! 三指 = 3 个点;五指 = 5 个点。** 谁来执行是身体层的事。
//!
//! # 🔴 力只给方向,不给大小(§1.3)
//!
//! *"一个苹果要多少牛顿 = 世界属性,可以记住;我这条胳膊命令一下产出多少牛顿 = 身体属性,
//! 随电量/磨损/温度漂。存成积一换电池就作废。"* ⇒ 第②格只有**锥**,没有牛顿。

/// 三维向量(米,或无量纲方向)。
pub type V3 = [f64; 3];

pub fn norm(v: V3) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

pub fn unit(v: V3) -> Option<V3> {
    let n = norm(v);
    if n.is_finite() && n > 1e-12 {
        Some([v[0] / n, v[1] / n, v[2] / n])
    } else {
        None
    }
}

pub fn cross(a: V3, b: V3) -> V3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

pub fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// **②的后一半:这一点上允许往哪使劲。**
///
/// 只给方向 —— 一个轴 + 一个半张角。**没有牛顿。**
///
/// - `half_angle = 0` ⇒ 只准**沿轴**使劲(吸盘就是这个:只能沿法向吸,不能侧推)
/// - `half_angle = π/2` ⇒ 半空间(任何不把物体推离表面的方向都行)
/// - 摩擦锥的半张角就是 `atan(μ)`,而 **μ 不写在这里** —— 谁量到了谁填。
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Cone {
    /// 锥轴,单位向量,世界系。
    pub axis: V3,
    /// 半张角,弧度。
    pub half_angle: f64,
}

impl Cone {
    /// 这个方向在不在锥里。**判据是角度,不是力** —— 与 `contact_gen::face_tilt_rad` 同一条:
    /// *"不需要 μ:μ 只决定'多大算过线',而排序只需要'谁更小'"*。
    pub fn admits(&self, dir: V3) -> bool {
        match (unit(self.axis), unit(dir)) {
            (Some(a), Some(d)) => dot(a, d).clamp(-1.0, 1.0).acos() <= self.half_angle + 1e-9,
            _ => false,
        }
    }
}

/// **①②④ 合起来:一个接触点。**
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Point {
    /// ① 碰物体表面的哪儿(世界系,米)。
    pub at: V3,
    /// ② 那一点的表面法向,**指向物体外侧**(单位向量)。
    pub normal: V3,
    /// ② 那一点允许往哪使劲。
    pub cone: Cone,
    /// ④ **这一个点**的容差(米)。
    ///
    /// 🔴 容差是**每个接触点各一个**,不是每个计划一个 —— 原文:
    /// *"碰到的地方要毫米级,只是路过的地方厘米级"*。老代码定义过却从没用过,
    /// 代价照记:渲图看见爪子已经张着骑在物体上,而代码因为一个**悬停**航点差 5 cm 在无限重规划。
    pub tol_m: f64,
}

/// **③ 物体要怎么动 —— 一个旋量。**
///
/// 🔴 **上一版只有平移,于是拧/撬/翻/倒四个动词【结构上说不出】。**
/// 旋量 = 平移 + 绕某轴转;而"绕轴转"必须说清**绕哪一点**转:
/// 撬是绕**物体贴着支撑面的那条边**,拧是绕**物体自己的轴** —— 同样的角速度,支点不同,结果完全不同。
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Twist {
    /// 平移,米。
    pub lin: V3,
    /// 轴角:方向 = 转轴,**模长 = 转多少弧度**。零向量 = 不转。
    pub ang: V3,
    /// 绕哪一点转(世界系,米)。`ang` 为零时它不起作用。
    pub pivot: V3,
}

impl Twist {
    /// 什么都不动。压/敲/抓的第③格就是它 —— **"物体不动"是一个合法的答案,不是缺省值。**
    pub fn still(pivot: V3) -> Twist {
        Twist { lin: [0.0; 3], ang: [0.0; 3], pivot }
    }
    /// 纯平移。
    pub fn slide(lin: V3) -> Twist {
        Twist { lin, ang: [0.0; 3], pivot: [0.0; 3] }
    }
    /// 绕 `pivot` 的一条轴转 `rad`。
    pub fn turn(axis: V3, rad: f64, pivot: V3) -> Option<Twist> {
        let a = unit(axis)?;
        Some(Twist { lin: [0.0; 3], ang: [a[0] * rad, a[1] * rad, a[2] * rad], pivot })
    }
    /// 转多少弧度。
    pub fn angle(&self) -> f64 {
        norm(self.ang)
    }
    /// 把一个世界点按这个旋量搬过去。**旋转绕 `pivot`,然后整体平移。**
    /// 用罗德里格斯公式,不引矩阵库。
    pub fn apply(&self, p: V3) -> V3 {
        let th = self.angle();
        let rotated = if th < 1e-12 {
            p
        } else {
            let k = [self.ang[0] / th, self.ang[1] / th, self.ang[2] / th];
            let r = [p[0] - self.pivot[0], p[1] - self.pivot[1], p[2] - self.pivot[2]];
            let (c, s) = (th.cos(), th.sin());
            let kxr = cross(k, r);
            let kdr = dot(k, r);
            [
                self.pivot[0] + r[0] * c + kxr[0] * s + k[0] * kdr * (1.0 - c),
                self.pivot[1] + r[1] * c + kxr[1] * s + k[1] * kdr * (1.0 - c),
                self.pivot[2] + r[2] * c + kxr[2] * s + k[2] * kdr * (1.0 - c),
            ]
        };
        [
            rotated[0] + self.lin[0],
            rotated[1] + self.lin[1],
            rotated[2] + self.lin[2],
        ]
    }
}

/// **一个接触集 —— 四格齐。**
///
/// 这就是"脑子对身体说的那句话"。里面**只有物体,没有身体**:
/// 碰这几个点、每点往这边使劲、于是物体这样动、每点差多少还算数。
#[derive(Clone, Debug, PartialEq)]
pub struct ContactSet {
    /// ①②④ —— **≥1 个**。吸盘 1 个,两指 2 个,三指 3 个,五指 5 个。
    ///
    /// 🔴 `≥2` 那句话是**对"抓"这个动词**说的(要有相对的两点才夹得住),
    /// 不是对这个结构说的 —— 推/压/撬都只要一个点,吸盘也只要一个点。
    pub points: Vec<Point>,
    /// ③ 物体要怎么动。
    pub motion: Twist,
    /// 🔴🔴 **四格【定不下来】的那一个自由度:手从哪个方向进场。**
    ///
    /// 这不是第五格,是四格的**补**。四格说的是"要发生什么",这一项说的是"手从哪儿来"。
    ///
    /// # 为什么四格定不下它(测试逼出来的,2026-08-16)
    ///
    /// - **单点接触**(推/压/吸盘):锥轴就是进场方向 ⇒ 四格**定得下**,这里填 `None` 即可。
    /// - **对夹**(抓/三指/五指):两个锥正好相反、两个法向也正好相反 ⇒ **合成恰好为零**。
    ///   剩下的约束只有"进场方向 ⊥ 接触点连线",而那是**一整圈**方向 —— 还剩一个自由度。
    ///   实测:不填时 `plan::steps` 返回 `NoFrame`,而不是随便挑一个。
    ///
    /// # 🔴 由此要订正一句我自己说过的话
    ///
    /// 我把老接口的 `close_yaw` 骂成"只有一个标量所以窄"。**骂错了一半** ——
    /// 它携带的恰好就是四格定不下来的**那一个**自由度,一个不多一个不少。
    /// 它真正的毛病不是窄,是它**只有这一个** ⇒ 法向、锥、旋量、容差全没地方放。
    ///
    /// 谁来填:看得见空隙的那一层(②a 从点云里知道哪边伸得进去)。填不了就**拒绝**,
    /// 不许在执行层里瞎挑一个 —— 挑错了物体会被爪子侧面撞飞,而没有任何一个环节会不一致。
    pub approach: Option<V3>,
}

/// 一个接触集哪儿填不下去。**必须点名是哪一格,不许含糊。**
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Gap {
    /// ① 一个接触点都没有。
    NoPoints,
    /// ② 某一点的法向不是一个方向(零向量 / NaN)。
    BadNormal(usize),
    /// ② 某一点的锥轴不是一个方向,或半张角不在 [0, π]。
    BadCone(usize),
    /// ② 某一点**允许的用力方向里,没有一个能推动物体按③走** ——
    /// 接触集自相矛盾:你说只能这样使劲,又说物体要那样动。
    ConeCannotDrive(usize),
    /// ③ 旋量既不平移也不转,而这个动词要求物体动。
    MotionStill,
    /// ③ 要转,但没说绕哪一点(pivot 落在物体外很远处通常是没填)。
    NoPivot,
    /// ④ 某一点的容差不是一个正数。
    BadTolerance(usize),
    /// 🔴 **这一格根本不是"一个接触集"能表达的** —— 见 `WhyNotOneSet`。
    NotOneSet(WhyNotOneSet),
}

/// 为什么这件事不是"一个接触集"。**这些不是缺陷,是接口的边界,必须说得出口。**
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum WhyNotOneSet {
    /// **没有刚体在动** —— 擦/扫/舀:被作用的是一片区域、一堆散料,不是一个有位姿的物体。
    /// 第③格(物体要怎么动)**无从填起**。
    /// ⇒ 它要的是**一串接触集**(闭式弓字形那一类),不是一个。
    NoRigidObject,
    /// **两件事同时成立** —— 握住 + 扣扳机:一个接触集在维持,另一个在动。
    /// ⇒ 它要的是**并存的多个接触集**。
    Concurrent,
    /// **它不作用在物体上** —— 够(Reach):手要去某处,而物体不参与。
    /// ⇒ 它是两个接触集**之间的过渡**,由执行层自己产生,不该出现在接口里。
    NoObjectInvolved,
}

impl ContactSet {
    /// 逐格自检。**过不了就点名是哪一格**,不许静默放行。
    ///
    /// `must_move`:这个动词要不要求物体动(压/敲/抓不要求,推/撬/拧要求)。
    pub fn check(&self, must_move: bool) -> Result<(), Gap> {
        if self.points.is_empty() {
            return Err(Gap::NoPoints);
        }
        for (i, p) in self.points.iter().enumerate() {
            if unit(p.normal).is_none() {
                return Err(Gap::BadNormal(i));
            }
            if unit(p.cone.axis).is_none()
                || !(0.0..=core::f64::consts::PI).contains(&p.cone.half_angle)
            {
                return Err(Gap::BadCone(i));
            }
            if !(p.tol_m.is_finite() && p.tol_m > 0.0) {
                return Err(Gap::BadTolerance(i));
            }
        }
        let moving = norm(self.motion.lin) > 1e-9 || self.motion.angle() > 1e-9;
        if must_move && !moving {
            return Err(Gap::MotionStill);
        }
        if self.motion.angle() > 1e-9 {
            // 绕轴转必须说清绕哪一点。这里只查它是不是一个数;"填得对不对"是几何层的事。
            if !self.motion.pivot.iter().all(|v| v.is_finite()) {
                return Err(Gap::NoPivot);
            }
        }
        // 🔴 **自洽性:每一点允许的用力方向里,至少要有一个能把物体往③推。**
        // 这一条挡的是"你说只能沿法向压,又要求物体横着走"这种自相矛盾的接触集 ——
        // 而那正是老接口无法表达、因此也无法自检的东西。
        if moving {
            for (i, p) in self.points.iter().enumerate() {
                // 这一点被③带着走的方向
                let after = self.motion.apply(p.at);
                let want = [after[0] - p.at[0], after[1] - p.at[1], after[2] - p.at[2]];
                match unit(want) {
                    // 这一点恰好落在转轴上 ⇒ 它不动,不构成矛盾(拧的时候轴心那一点就是这样)
                    None => continue,
                    Some(w) => {
                        if !p.cone.admits(w) {
                            return Err(Gap::ConeCannotDrive(i));
                        }
                    }
                }
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod 十三个动词填表 {
    use super::*;
    use core::f64::consts::{FRAC_PI_2, PI};

    const MM: f64 = 0.002; // 碰到的地方:毫米级
    const CM: f64 = 0.02; // 路过的地方:厘米级

    fn cone(axis: V3, half: f64) -> Cone {
        Cone { axis, half_angle: half }
    }
    fn pt(at: V3, normal: V3, c: Cone, tol: f64) -> Point {
        Point { at, normal, cone: c, tol_m: tol }
    }
    /// 两个相对的点:抓的最小形状。物体在原点、宽 `w`。
    fn 对置两点(w: f64, half: f64) -> Vec<Point> {
        vec![
            pt([-w / 2.0, 0.0, 0.10], [-1.0, 0.0, 0.0], cone([1.0, 0.0, 0.0], half), MM),
            pt([w / 2.0, 0.0, 0.10], [1.0, 0.0, 0.0], cone([-1.0, 0.0, 0.0], half), MM),
        ]
    }
    /// 握着的那几点跟物体刚性同动 ⇒ 锥取各自真实的位移方向。
    fn 跟着走(pts: Vec<Point>, m: &Twist) -> Vec<Point> {
        pts.into_iter()
            .map(|p| {
                let a = m.apply(p.at);
                match unit([a[0] - p.at[0], a[1] - p.at[1], a[2] - p.at[2]]) {
                    Some(d) => Point { cone: cone(d, FRAC_PI_2), ..p },
                    None => p,
                }
            })
            .collect()
    }

    #[test]
    fn 抓_两个相对的点向内使劲() {
        let cs = ContactSet { points: 对置两点(0.05, 0.5), motion: Twist::still([0.0, 0.0, 0.10]) , approach: None };
        assert_eq!(cs.check(false), Ok(()));
        assert_eq!(cs.points.len(), 2);
    }

    #[test]
    fn 松_同样的点物体不动() {
        let cs = ContactSet { points: 对置两点(0.05, 0.5), motion: Twist::still([0.0, 0.0, 0.10]) , approach: None };
        assert_eq!(cs.check(false), Ok(()));
    }

    #[test]
    fn 压_一个点沿法向使劲物体不动() {
        let cs = ContactSet {
            points: vec![pt([0.0, 0.0, 0.10], [0.0, 0.0, 1.0], cone([0.0, 0.0, -1.0], 0.2), MM)],
            motion: Twist::still([0.0, 0.0, 0.10]), approach: None };
        assert_eq!(cs.check(false), Ok(()), "压:物体不动是一个合法答案");
    }

    #[test]
    fn 推_一个点横向力物体在支撑面上平移() {
        let cs = ContactSet {
            points: vec![pt([0.03, 0.0, 0.05], [1.0, 0.0, 0.0], cone([-1.0, 0.0, 0.0], 0.6), MM)],
            motion: Twist::slide([-0.10, 0.0, 0.0]), approach: None };
        assert_eq!(cs.check(true), Ok(()));
    }

    #[test]
    fn 撬_边缘上一个点物体绕那条边转() {
        let pivot = [0.05, 0.0, 0.0];
        let at = [-0.04, 0.0, 0.02];
        let m = Twist::turn([0.0, 1.0, 0.0], -0.4, pivot).expect("轴非零");
        let cs = ContactSet { points: 跟着走(vec![pt(at, [0.0, 0.0, 1.0], cone([0.0, 0.0, 1.0], 0.5), MM)], &m), motion: m , approach: None };
        assert_eq!(cs.check(true), Ok(()));
        assert!(cs.motion.angle() > 0.0, "撬必须有转,而上一版的 Motion 说不出转");
    }

    #[test]
    fn 翻_同一形状更大的角() {
        let pivot = [0.05, 0.0, 0.0];
        let at = [-0.04, 0.0, 0.02];
        let m = Twist::turn([0.0, 1.0, 0.0], -PI * 0.9, pivot).expect("轴非零");
        let cs = ContactSet { points: 跟着走(vec![pt(at, [0.0, 0.0, 1.0], cone([0.0, 0.0, 1.0], 0.9), MM)], &m), motion: m , approach: None };
        assert_eq!(cs.check(true), Ok(()));
    }

    #[test]
    fn 倒_握着绕一条水平轴转() {
        let m = Twist::turn([0.0, 1.0, 0.0], 1.8, [0.0, 0.0, 0.10]).expect("轴非零");
        let cs = ContactSet { points: 跟着走(对置两点(0.05, 0.5), &m), motion: m , approach: None };
        assert_eq!(cs.check(true), Ok(()));
    }

    #[test]
    fn 拧_握着绕物体自己的轴转() {
        let m = Twist::turn([0.0, 0.0, 1.0], 1.5, [0.0, 0.0, 0.10]).expect("轴非零");
        let cs = ContactSet { points: 跟着走(对置两点(0.05, 0.5), &m), motion: m , approach: None };
        assert_eq!(cs.check(true), Ok(()));
        // 🔴 与"倒"的差别【只在第③格的轴】,四格结构一个字没变 —— 这就是"塌成一张模板"。
    }

    #[test]
    fn 插_握着沿一条轴往里走() {
        let m = Twist::slide([0.0, 0.06, 0.0]);
        let cs = ContactSet { points: 跟着走(对置两点(0.05, 0.5), &m), motion: m , approach: None };
        assert_eq!(cs.check(true), Ok(()));
    }

    #[test]
    fn 放_握着搬到目标位姿() {
        let m = Twist::slide([0.20, 0.30, -0.10]);
        let cs = ContactSet { points: 跟着走(对置两点(0.05, 0.5), &m), motion: m , approach: None };
        assert_eq!(cs.check(true), Ok(()), "放 = 把物体搬过去;松手是【下一个】接触集");
    }

    #[test]
    fn 吸盘_一个点加一个只允许法向的锥() {
        let cs = ContactSet {
            points: vec![pt([0.0, 0.0, 0.10], [0.0, 0.0, 1.0], cone([0.0, 0.0, 1.0], 0.0), MM)],
            motion: Twist::slide([0.0, 0.0, 0.05]), approach: None };
        assert_eq!(cs.check(true), Ok(()));
        assert_eq!(cs.points.len(), 1, "吸盘 1 个点填同一张表");
    }

    #[test]
    fn 三指与五指_只是点数不同() {
        for n in [3usize, 5] {
            let pts: Vec<Point> = (0..n)
                .map(|i| {
                    let a = i as f64 / n as f64 * 2.0 * PI;
                    let (c, s) = (a.cos(), a.sin());
                    pt([0.03 * c, 0.03 * s, 0.10], [c, s, 0.0], cone([-c, -s, 0.0], 0.5), MM)
                })
                .collect();
            let cs = ContactSet { points: pts, motion: Twist::still([0.0, 0.0, 0.10]) , approach: None };
            assert_eq!(cs.check(false), Ok(()), "{n} 指必须填得满同一张表");
            assert_eq!(cs.points.len(), n);
        }
    }

    #[test]
    fn 锥与物体运动矛盾时当场点名() {
        let cs = ContactSet {
            points: vec![pt([0.0, 0.0, 0.10], [0.0, 0.0, 1.0], cone([0.0, 0.0, -1.0], 0.1), MM)],
            motion: Twist::slide([0.10, 0.0, 0.0]), approach: None };
        assert_eq!(cs.check(true), Err(Gap::ConeCannotDrive(0)));
    }

    #[test]
    fn 四格各自缺失都点得出名() {
        let good = pt([0.0, 0.0, 0.1], [0.0, 0.0, 1.0], cone([0.0, 0.0, -1.0], 0.3), MM);
        assert_eq!(ContactSet { points: vec![], motion: Twist::still([0.0; 3]) , approach: None }.check(false), Err(Gap::NoPoints));
        assert_eq!(
            ContactSet { points: vec![Point { normal: [0.0; 3], ..good }], motion: Twist::still([0.0; 3]) , approach: None }.check(false),
            Err(Gap::BadNormal(0))
        );
        assert_eq!(
            ContactSet { points: vec![Point { cone: cone([0.0; 3], 0.3), ..good }], motion: Twist::still([0.0; 3]) , approach: None }.check(false),
            Err(Gap::BadCone(0))
        );
        assert_eq!(
            ContactSet { points: vec![Point { tol_m: 0.0, ..good }], motion: Twist::still([0.0; 3]) , approach: None }.check(false),
            Err(Gap::BadTolerance(0))
        );
        assert_eq!(ContactSet { points: vec![good], motion: Twist::still([0.0; 3]) , approach: None }.check(true), Err(Gap::MotionStill));
        // ④ 容差是【每点】各一个:碰到的毫米级,路过的厘米级
        let mixed = ContactSet {
            points: vec![good, Point { at: [0.0, 0.0, 0.3], tol_m: CM, ..good }],
            motion: Twist::still([0.0; 3]), approach: None };
        assert_eq!(mixed.check(false), Ok(()));
        assert!(mixed.points[0].tol_m < mixed.points[1].tol_m);
    }

    #[test]
    fn 够_不作用在物体上() {
        // Reach:手要去某处,而**物体不参与** ⇒ 第①格无从填起。
        // 它不是一个接触集,是两个接触集**之间的过渡**,由执行层自己产生。
        assert_eq!(Gap::NotOneSet(WhyNotOneSet::NoObjectInvolved), Gap::NotOneSet(WhyNotOneSet::NoObjectInvolved));
    }

    #[test]
    fn 擦与舀_没有刚体在动() {
        // Wipe / Scoop:被作用的是一片区域 / 一堆散料,没有一个有位姿的物体 ⇒ 第③格无从填起。
        // ①②④ 都填得出来,唯独 ③ 不行 ⇒ 要的是**一串**接触集(闭式弓字形那一类),不是一个。
        assert_eq!(Gap::NotOneSet(WhyNotOneSet::NoRigidObject), Gap::NotOneSet(WhyNotOneSet::NoRigidObject));
    }

    #[test]
    fn 握着扣扳机_两件事同时成立() {
        let 握 = ContactSet { points: 对置两点(0.06, 0.5), motion: Twist::still([0.0, 0.0, 0.1]) , approach: None };
        let 扣 = ContactSet {
            points: vec![pt([0.0, 0.02, 0.10], [0.0, 1.0, 0.0], cone([0.0, -1.0, 0.0], 0.3), MM)],
            motion: Twist::slide([0.0, -0.01, 0.0]), approach: None };
        assert_eq!(握.check(false), Ok(()), "握:四格填得满");
        assert_eq!(扣.check(true), Ok(()), "扣扳机:四格也填得满");
        // 🔴 缺的**不是表达力**,是"同时落两个接触集" —— 点名在这里。
        assert_eq!(Gap::NotOneSet(WhyNotOneSet::Concurrent), Gap::NotOneSet(WhyNotOneSet::Concurrent));
    }
}

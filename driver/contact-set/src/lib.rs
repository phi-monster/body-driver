
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

pub mod many;
pub mod replay;

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

/// **这个接触是谁跟物体之间的。**
///
/// # 🔴 为什么①必须记这一维(测试逼出来的,2026-08-16)
///
/// 撬:物体绕它压在桌面上的那条边转。手只有**一个**接触点,而单个接触力
/// `F = f ≠ 0`,产生不出③要的**纯力矩** ⇒ 集合级判据当场判死 `CannotDrive`。
/// 判得对 —— **少的那一个接触是桌子给的反力**,它一直都在,只是①没地方记。
///
/// 少了这一维,撬/翻/舀/靠着墙推**全都填不满**,而且会被判成"接触集自相矛盾",
/// 把一条完全正确的接触集冤枉掉。
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Who {
    /// 手(或吸盘/工具)给的接触 —— **执行层要去访问它**,它变成航点。
    ///
    /// 🔴 **带着"是【哪一只】"**(2026-08-16 补)。一只五指手的五个点都是同一只
    /// ⇒ 同一个手腕、同一个朝向;而**双臂抱一个箱子是两只**,两个手腕各有各的朝向,
    /// 合成一个没有意义。上一版这里没有编号,于是**人形直接说不出口**。
    ///
    /// 编号本身**不带语义**(0 不必是"左手")—— 它只说"这些点归同一个执行器管",
    /// 谁是谁由身体层自己对号。**接口仍然不知道有几只手,只知道点分几堆。**
    Hand(u8),
    /// 世界给的接触:桌面、墙、卡具、另一只手按住的地方。
    /// **执行层不访问它**(手够不到桌子底下那条边),但它参与"能不能驱动"的计算。
    World,
}

/// **①②④ 合起来:一个接触点。**
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Point {
    /// ① 这个接触是谁的。
    pub by: Who,
    /// ① 碰物体表面的哪儿(世界系,米)。
    pub at: V3,
    /// ② 那一点的表面法向,**指向物体外侧**(单位向量)。
    pub normal: V3,
    /// ② 那一点允许往哪使劲。
    pub cone: Cone,
    /// ② **这个接触【拉】不拉得动**(能不能沿法向把物体往自己这边拽)。
    ///
    /// # 🔴 为什么必须有这一项(全链测试逼出来的,2026-08-16)
    ///
    /// 普通接触是**单向**的:只推得动,拉不动(手指一拉就离开表面了)。
    /// 而**真空吸盘/电磁/胶带的全部意义就是能拉** —— 少了这一项,
    /// **吸盘吸住了也抬不起任何东西**:判据会说"你只能往下压,却要求物体往上走" ⇒ `CannotDrive`。
    /// 实测:点云 → 吸盘 → 抬起 5 cm 这条链就断在这儿。
    ///
    /// 🔴 它同样是**要量的身体属性**(能拉多少是真空度/磁力的事),
    /// 但这里只记"能不能",不记"多大" —— 与"力只给方向不给大小"同一条。
    pub pull: bool,
    /// ② **这个接触扭不扭得动**(能不能绕自己的法向传力矩)。
    ///
    /// # 🔴 为什么必须有这一项(测试逼出来的,2026-08-16)
    ///
    /// 纯**点**接触传不了扭矩。于是"两指捏着勺子把它翻过来""两指捏着螺丝刀拧"
    /// 在静力学上**直接判死** —— 而那个转轴恰好就是两个接触点的连线,
    /// 和 `replay` 说"绕连线的自转定不下来"**是同一件事的两面**(一个说产生不出,一个说测不出来)。
    ///
    /// 真手做得到,是因为**指腹是一片面,不是一个点**(Murray-Li-Sastry 的 soft-finger contact)。
    /// ⇒ 这一项就是"这个接触是点还是面"。**吸盘 = true**(吸盘吸住了是拧得动的),
    /// 硬钢针尖 = false。
    ///
    /// 🔴 它是**身体属性,要量**:指腹多软、贴片多大,决定了扭得动扭不动。
    /// 现在没有任何一处量过它 —— 谁填谁负责,别默认 true 混过去。
    pub torsion: bool,
    /// ② **这个接触【掰】不掰得动**(能不能绕**切向**轴传力矩,也就是抗不抗"剥离")。
    ///
    /// # 🔴 为什么它和 `torsion` 是两件事(验收台逼出来的,2026-08-16)
    ///
    /// `torsion` 只管**绕自己法向**那一根轴。于是一个吸盘吸在物体顶上,
    /// **只转得动、翻不动** —— 撬/翻/倒/舀 全判死(验收台上吸盘那九格就是它)。
    /// 而真空吸盘搬面板时**天天在把面板立起来**:密封面是一片有半径的面,
    /// 它扛得住剥离力矩(扛到多大是真空度 × 半径³的事)。
    ///
    /// ⇒ **点接触 false · 指尖 false · 吸盘/胶垫/大贴片 true。**
    /// 与 `pull`、`torsion` 合起来才是第②格的真身:**这个接触能传哪几种东西**,
    /// 而不是"一个锥"。今天连着三次在②里发现少一维,病根都是把②读窄了。
    pub peel: bool,
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
    /// ② **所有接触加在一起**都产生不出③要的那个力旋量 ——
    /// 接触集自相矛盾:你说只能这样使劲,又说物体要那样动。
    ///
    /// 🔴 判据在【集合】上,不在单点上(见 `can_drive` 的头注)。
    CannotDrive,
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
        // 🔴 **自洽性:所有接触【加在一起】,能不能产生③要的那个力旋量。**
        // 这一条挡的是"你说只能这样使劲,又说物体要那样动"的自相矛盾接触集。
        if moving && !self.can_drive() {
            return Err(Gap::CannotDrive);
        }
        Ok(())
    }

    /// **所有接触的摩擦锥张成的集合,包不包含③要的那个力旋量方向。**
    ///
    /// # 🔴 为什么判据必须在【集合】上,不在单点上
    ///
    /// §1.1 那条刀口写的是 `G · F接触 = F物体` —— 左边是**对所有接触求和**。
    /// 逐点判"这一点自己能不能把物体推过去"是**错的层级**,而且会把对的接触集判死:
    /// 两指捏着横向搬运,任何**单**指都做不到(切向力出了自己的摩擦锥),
    /// 但两指**一起**可以 —— 内部的对夹力互相抵消,切向的摩擦力叠加。
    /// 本仓实测:逐点判据把「放」「反例」两条合法接触集判成 `ConeCannotDrive`(2026-08-16)。
    ///
    /// # 建模假设(写明,不藏)
    ///
    /// - 准静态:阻力与运动方向相反 ⇒ 需要的力旋量方向 ∝ ③的旋量。
    /// - 力与力矩单位不同,用**特征长度** L(各接触到参考点的平均距离)配平:
    ///   `ŵ = (v, ω·L)`。L 是这个问题里真实存在的长度,不是调出来的常数。
    /// - 只判**方向**在不在锥里,不判**大小** —— 大小是"捏多紧",归执行层。
    pub fn can_drive(&self) -> bool {
        // 🔴 **参考点取接触点质心(物体质心的代理),【不是】第③格的 pivot。**
        //
        // 我一开始拿 pivot 当参考点,于是"绕一个偏在下面的支点转"被写成了
        // **纯力矩**的需求 —— 而纯力矩要求合力为零,两指对夹根本给不出
        // (实测:握着绕下方 10 cm 的支点转,被判 `CannotDrive`,而这件事天天在做)。
        // 错在**建模**:第③格的 pivot 说的是"转轴在哪",不是"反力作用在哪"。
        // 拿着东西挥的时候,物体的合力**本来就不为零** —— 那个力是胳膊给的。
        //
        // 正确写法(牛顿-欧拉方向):参考点取质心,需求 = (质心的速度, 角速度)。
        // 质心未知(世界属性,要学),这里用**接触点质心**当代理,并把这句话写在这儿。
        let refp = {
            let k = self.points.len() as f64;
            [
                self.points.iter().map(|p| p.at[0]).sum::<f64>() / k,
                self.points.iter().map(|p| p.at[1]).sum::<f64>() / k,
                self.points.iter().map(|p| p.at[2]).sum::<f64>() / k,
            ]
        };
        let l: f64 = {
            let k = self.points.len() as f64;
            let s: f64 = self
                .points
                .iter()
                .map(|p| norm([p.at[0] - refp[0], p.at[1] - refp[1], p.at[2] - refp[2]]))
                .sum();
            let m = s / k;
            if m > 1e-9 {
                m
            } else {
                1.0
            }
        };
        // 想要的力旋量方向:力 ∝ **参考点自己的速度**(平移 + 绕支点转带出来的那一份),
        // 力矩 ∝ 角速度。绕支点转会把质心也甩出去 —— 那一份必须算进来,不算就是要求纯力矩。
        let m = self.motion;
        let rp = [refp[0] - m.pivot[0], refp[1] - m.pivot[1], refp[2] - m.pivot[2]];
        let vspin = cross(m.ang, rp);
        let mut wd = [
            m.lin[0] + vspin[0],
            m.lin[1] + vspin[1],
            m.lin[2] + vspin[2],
            m.ang[0] * l,
            m.ang[1] * l,
            m.ang[2] * l,
        ];
        let nd = (wd.iter().map(|x| x * x).sum::<f64>()).sqrt();
        if nd < 1e-12 {
            return true;
        }
        for x in wd.iter_mut() {
            *x /= nd;
        }
        // 每个接触的摩擦锥,取 8 条棱 + 轴心,各生成一条力旋量
        let mut gen: Vec<[f64; 6]> = Vec::new();
        for p in &self.points {
            let n = match unit(p.cone.axis) {
                Some(v) => v,
                None => return false,
            };
            let t1 = {
                let a = if n[0].abs() < 0.9 { [1.0, 0.0, 0.0] } else { [0.0, 1.0, 0.0] };
                match unit(cross(n, a)) {
                    Some(v) => v,
                    None => return false,
                }
            };
            let t2 = cross(n, t1);
            let (c, s) = (p.cone.half_angle.cos(), p.cone.half_angle.sin());
            for k in 0..9 {
                let f = if k == 8 {
                    n
                } else {
                    let phi = core::f64::consts::TAU * (k as f64) / 8.0;
                    let (cp, sp) = (phi.cos(), phi.sin());
                    [
                        c * n[0] + s * (cp * t1[0] + sp * t2[0]),
                        c * n[1] + s * (cp * t1[1] + sp * t2[1]),
                        c * n[2] + s * (cp * t1[2] + sp * t2[2]),
                    ]
                };
                let r = [p.at[0] - refp[0], p.at[1] - refp[1], p.at[2] - refp[2]];
                let t = cross(r, f);
                gen.push([f[0], f[1], f[2], t[0] / l, t[1] / l, t[2] / l]);
            }
            // 能拉的接触(真空/磁/胶):沿法向**反着**也使得上劲。把锥镜像过来。
            if p.pull {
                for k in 0..9 {
                    let f = if k == 8 {
                        [-n[0], -n[1], -n[2]]
                    } else {
                        let phi = core::f64::consts::TAU * (k as f64) / 8.0;
                        let (cp, sp) = (phi.cos(), phi.sin());
                        let t1 = {
                            let a = if n[0].abs() < 0.9 { [1.0, 0.0, 0.0] } else { [0.0, 1.0, 0.0] };
                            unit(cross(n, a)).unwrap_or([0.0, 1.0, 0.0])
                        };
                        let t2 = cross(n, t1);
                        [
                            -c * n[0] + s * (cp * t1[0] + sp * t2[0]),
                            -c * n[1] + s * (cp * t1[1] + sp * t2[1]),
                            -c * n[2] + s * (cp * t1[2] + sp * t2[2]),
                        ]
                    };
                    let r = [p.at[0] - refp[0], p.at[1] - refp[1], p.at[2] - refp[2]];
                    let t = cross(r, f);
                    gen.push([f[0], f[1], f[2], t[0] / l, t[1] / l, t[2] / l]);
                }
            }
            // 抗剥离的接触(吸盘/胶垫):还能绕**切向**两根轴传力矩,四个方向都给。
            if p.peel {
                let t1 = {
                    let a = if n[0].abs() < 0.9 { [1.0, 0.0, 0.0] } else { [0.0, 1.0, 0.0] };
                    unit(cross(n, a)).unwrap_or([0.0, 1.0, 0.0])
                };
                let t2 = cross(n, t1);
                for ax in [t1, t2] {
                    gen.push([0.0, 0.0, 0.0, ax[0] / l, ax[1] / l, ax[2] / l]);
                    gen.push([0.0, 0.0, 0.0, -ax[0] / l, -ax[1] / l, -ax[2] / l]);
                }
            }
            // 面接触:还能绕自己的法向拧。**两个方向都给** —— 扭矩可正可负。
            if p.torsion {
                gen.push([0.0, 0.0, 0.0, n[0] / l, n[1] / l, n[2] / l]);
                gen.push([0.0, 0.0, 0.0, -n[0] / l, -n[1] / l, -n[2] / l]);
            }
        }
        in_cone(&gen, wd)
    }
}

/// `wd` 在不在 `gen` 张成的**凸锥**里 —— 非负最小二乘,投影梯度,零依赖。
fn in_cone(gen: &[[f64; 6]], wd: [f64; 6]) -> bool {
    if gen.is_empty() {
        return false;
    }
    // 🔴 **先把每条棱归一化。** 凸锥对每条棱各自的正倍数不变,所以这不改变答案,
    // 但**不做就会算错**:力矩那三列除以特征长度 l,l 小到 1.2 cm 时棱的模差 ~80 倍,
    // 投影梯度的步长被最大的那条棱锁死,4000 步都收敛不到 ⇒ **一条可行的接触集被判成不可行**。
    // 实测:两指捏着勺子往下插(明明做得到)因此被判 `CannotDrive`。
    let gen: Vec<[f64; 6]> = gen
        .iter()
        .filter_map(|g| {
            let m = (g.iter().map(|x| x * x).sum::<f64>()).sqrt();
            (m > 1e-12).then(|| [g[0] / m, g[1] / m, g[2] / m, g[3] / m, g[4] / m, g[5] / m])
        })
        .collect();
    let gen = &gen[..];
    let n = gen.len();
    if n == 0 {
        return false;
    }
    // 步长取 1/L(L = 最大特征值上界,用 Frobenius 范数代替,保守但稳)
    let mut lip = 0.0f64;
    for g in gen {
        lip += g.iter().map(|x| x * x).sum::<f64>();
    }
    let step = if lip > 1e-12 { 1.0 / lip } else { 1.0 };

    // 🔴🔴 **加速(FISTA)+ 一条真的停机判据。**
    //
    // 上一版是定步长投影梯度,跑满 4000 步就下结论。**它会在还没算完的时候回答"做不到"** ——
    // 而"做不到"是这条链上最重的一句话。实测(2026-08-16,验收台诊断):
    // 薄板的三指/五指「绕 x 转」「绕 y 转」在 4000 步下判死;把步数放到 20 万,**全部变成可行**
    // ⇒ 那几格根本不是物理上做不到,**是判据没算完就下了结论**,而且它长得和真结论一模一样。
    //
    // ⇒ 两条都要:① 加速,把收敛率从 O(1/k) 提到 O(1/k²);
    //   ② **停机看"还在不在进步",不看跑了多少步** —— 进步停了才允许说"做不到"。
    let 残差 = |a: &[f64]| -> ([f64; 6], f64) {
        let mut r = [0.0f64; 6];
        for (j, g) in gen.iter().enumerate() {
            for d in 0..6 {
                r[d] += a[j] * g[d];
            }
        }
        for d in 0..6 {
            r[d] -= wd[d];
        }
        (r, (r.iter().map(|x| x * x).sum::<f64>()).sqrt())
    };
    let mut a = vec![0.0f64; n];
    let mut y = a.clone();
    let mut t = 1.0f64;
    let mut 上次 = f64::MAX;
    for _ in 0..20000 {
        let (r, _) = 残差(&y);
        let mut next = vec![0.0f64; n];
        for (j, g) in gen.iter().enumerate() {
            let grad: f64 = (0..6).map(|d| g[d] * r[d]).sum();
            next[j] = (y[j] - step * grad).max(0.0);
        }
        let t2 = (1.0 + (1.0 + 4.0 * t * t).sqrt()) / 2.0;
        let w = (t - 1.0) / t2;
        for j in 0..n {
            y[j] = next[j] + w * (next[j] - a[j]);
        }
        a = next;
        t = t2;
        let res = 残差(&a).1;
        if res < 1e-7 {
            return true;
        }
        // 🔴 停机看进步,不看步数。还在往下掉就接着算,掉不动了才敢说"做不到"。
        if (上次 - res).abs() < 1e-14 * 上次.max(1.0) {
            break;
        }
        上次 = res;
    }
    残差(&a).1 < 1e-4
}

#[cfg(test)]
mod 十三个动词填表 {
    use super::*;
    use crate::many::{ManyGap, Move};
    use core::f64::consts::{FRAC_PI_2, PI};

    const MM: f64 = 0.002; // 碰到的地方:毫米级
    const CM: f64 = 0.02; // 路过的地方:厘米级

    fn cone(axis: V3, half: f64) -> Cone {
        Cone { axis, half_angle: half }
    }
    fn pt(at: V3, normal: V3, c: Cone, tol: f64) -> Point {
        Point { by: Who::Hand(0), at, normal, cone: c, pull: false, torsion: false, peel: false, tol_m: tol }
    }
    /// 两个相对的点:抓的最小形状。物体在原点、宽 `w`。
    fn 对置两点(w: f64, half: f64) -> Vec<Point> {
        vec![
            pt([-w / 2.0, 0.0, 0.10], [-1.0, 0.0, 0.0], cone([1.0, 0.0, 0.0], half), MM),
            pt([w / 2.0, 0.0, 0.10], [1.0, 0.0, 0.0], cone([-1.0, 0.0, 0.0], half), MM),
        ]
    }
    /// 握着的那几点跟物体刚性同动 ⇒ 锥取各自真实的位移方向。

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

    /// 桌子在支点那儿顶着物体的那个接触。**它一直都在,只是①以前没地方记。**
    fn 桌子顶着(pivot: V3, mu_atan: f64) -> Point {
        Point {
            by: Who::World,
            at: pivot,
            normal: [0.0, 0.0, -1.0], // 物体朝下的那个面,法向朝外 = 朝下
            cone: cone([0.0, 0.0, 1.0], mu_atan), // 桌子只能往上顶
            pull: false,
            torsion: false,
            peel: false,
            tol_m: MM,
        }
    }

    #[test]
    fn 撬_边缘上一个点物体绕那条边转() {
        let pivot = [0.05, 0.0, 0.0];
        let at = [-0.04, 0.0, 0.02];
        let m = Twist::turn([0.0, 1.0, 0.0], -0.4, pivot).expect("轴非零");
        // 🔴 手一个点 + 桌子那条边。少了后者,`can_drive` 判 `CannotDrive` —— 而且判得对:
        // 单个接触力产生不出③要的纯力矩,支反力本来就是这件事的一部分。
        let mut pts = vec![pt(at, [0.0, 0.0, 1.0], cone([0.0, 0.0, -1.0], 0.4636), MM)];
        pts.push(桌子顶着(pivot, 0.46));
        let cs = ContactSet { points: pts, motion: m, approach: None };
        assert_eq!(cs.check(true), Ok(()));
        assert!(cs.motion.angle() > 0.0, "撬必须有转,而上一版的 Motion 说不出转");
    }

    #[test]
    fn 撬_不给支反力就该判死() {
        let pivot = [0.05, 0.0, 0.0];
        let at = [-0.04, 0.0, 0.02];
        let m = Twist::turn([0.0, 1.0, 0.0], -0.4, pivot).expect("轴非零");
        let cs = ContactSet {
            points: vec![pt(at, [0.0, 0.0, 1.0], cone([0.0, 0.0, -1.0], 0.4636), MM)],
            motion: m,
            approach: None,
        };
        // **反例:台子有没有牙。** 只有手那一个点时必须判死,否则集合级判据是摆设。
        assert_eq!(cs.check(true), Err(Gap::CannotDrive));
    }

    #[test]
    fn 翻_同一形状更大的角() {
        let pivot = [0.05, 0.0, 0.0];
        let at = [-0.04, 0.0, 0.02];
        let m = Twist::turn([0.0, 1.0, 0.0], -PI * 0.9, pivot).expect("轴非零");
        let mut pts = vec![pt(at, [0.0, 0.0, 1.0], cone([0.0, 0.0, -1.0], 0.4636), MM)];
        pts.push(桌子顶着(pivot, 0.46));
        let cs = ContactSet { points: pts, motion: m, approach: None };
        assert_eq!(cs.check(true), Ok(()));
    }

    #[test]
    fn 倒_握着绕一条水平轴转() {
        let m = Twist::turn([0.0, 1.0, 0.0], 1.8, [0.0, 0.0, 0.10]).expect("轴非零");
        let cs = ContactSet { points: 对置两点(0.05, 0.4636), motion: m , approach: None };
        assert_eq!(cs.check(true), Ok(()));
    }

    #[test]
    fn 拧_握着绕物体自己的轴转() {
        let m = Twist::turn([0.0, 0.0, 1.0], 1.5, [0.0, 0.0, 0.10]).expect("轴非零");
        let cs = ContactSet { points: 对置两点(0.05, 0.4636), motion: m , approach: None };
        assert_eq!(cs.check(true), Ok(()));
        // 🔴 与"倒"的差别【只在第③格的轴】,四格结构一个字没变 —— 这就是"塌成一张模板"。
    }

    #[test]
    fn 插_握着沿一条轴往里走() {
        let m = Twist::slide([0.0, 0.06, 0.0]);
        let cs = ContactSet { points: 对置两点(0.05, 0.4636), motion: m , approach: None };
        assert_eq!(cs.check(true), Ok(()));
    }

    #[test]
    fn 放_握着搬到目标位姿() {
        let m = Twist::slide([0.20, 0.30, -0.10]);
        let cs = ContactSet { points: 对置两点(0.05, 0.4636), motion: m , approach: None };
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
        assert_eq!(cs.check(true), Err(Gap::CannotDrive));
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
        // 🔴 这一条**在接口里没有条目**,而且是故意的:它由执行层在两段之间自己产生
        //(每段开头那个"悬停"就是它)。这里验的是"一个接触点都没有时会点名",
        //     而不是从前那句 `assert_eq!(X, X)` —— 那种断言**永远不可能失败**。
        let 空 = ContactSet { points: vec![], motion: Twist::slide([0.1, 0.0, 0.0]), approach: None };
        assert_eq!(空.check(true), Err(Gap::NoPoints));
    }

    /// 握着抹布来回擦 —— **一串,而且全程不松手**。
    ///
    /// 🔴 **订正我自己写过的一句话**:从前把这一格记成"没有刚体在动"。**错的** ——
    /// 握着的抹布本身就是刚体、它的运动就是旋量,第③格填得满。
    /// 真正说不出的是"那片地方擦干净了没有",而那是**任务的判据**,从来不是接触集的活。
    fn 擦(道: &[(V3, V3)]) -> Move {
        Move::Keep(
            道.iter()
                .map(|(from, d)| {
                    let pts = vec![
                        pt([from[0], from[1] - 0.02, from[2]], [0.0, -1.0, 0.0], cone([0.0, 1.0, 0.0], 0.4636), MM),
                        pt([from[0], from[1] + 0.02, from[2]], [0.0, 1.0, 0.0], cone([0.0, -1.0, 0.0], 0.4636), MM),
                    ];
                    Move::One(ContactSet {
                        points: pts,
                        motion: Twist::slide(*d),
                        approach: Some([0.0, 0.0, -1.0]),
                    })
                })
                .collect(),
        )
    }

    #[test]
    fn 擦_一串不松手的接触集_每段都填得满() {
        let m = 擦(&[
            ([0.0, 0.0, 0.02], [0.20, 0.0, 0.0]),
            ([0.20, 0.0, 0.02], [0.0, 0.05, 0.0]),
            ([0.20, 0.05, 0.02], [-0.20, 0.0, 0.0]),
        ]);
        assert_eq!(m.check(true), Ok(()));
        assert_eq!(m.flatten().len(), 3);
    }

    #[test]
    fn 擦_接不上的一串必须点名是第几段_差多少() {
        // 第 0 段末了在 x=0.20,第 1 段却说从 x=0.30 开始 ⇒ 手上还握着东西却跳了 10 cm。
        let m = 擦(&[([0.0, 0.0, 0.02], [0.20, 0.0, 0.0]), ([0.30, 0.0, 0.02], [0.0, 0.05, 0.0])]);
        match m.check(true) {
            Err(ManyGap::KeepBreaksContact(_, i, d)) => {
                assert_eq!(i, 1);
                assert!((d - 0.10).abs() < 1e-9, "差的就是那 10 cm,实得 {d}");
            }
            other => panic!("说了不松手却接不上,必须点名;实得 {other:?}"),
        }
    }

    #[test]
    fn 舀_插进去_兜起来_抬出来() {
        // 勺子是刚体:插进去(平移)→ 兜起来(绕勺口转)→ 抬出来(平移)。**全程不松手。**
        // 🔴 `torsion: true` = **指腹是一片面**。兜起来那一下的转轴恰好就是两指连线,
        //    纯点接触在静力学上产生不出它 —— 见下面那条反例。
        let 握 = |at: V3| {
            let pad = |p: Point| Point { pull: false, torsion: true, peel: false, ..p };
            vec![
                pad(pt([at[0], at[1] - 0.012, at[2]], [0.0, -1.0, 0.0], cone([0.0, 1.0, 0.0], 0.4636), MM)),
                pad(pt([at[0], at[1] + 0.012, at[2]], [0.0, 1.0, 0.0], cone([0.0, -1.0, 0.0], 0.4636), MM)),
            ]
        };
        let 插 = Move::One(ContactSet {
            points: 握([0.0, 0.0, 0.10]),
            motion: Twist::slide([0.0, 0.0, -0.04]),
            approach: Some([0.0, 0.0, -1.0]),
        });
        let 兜 = Move::One(ContactSet {
            points: 握([0.0, 0.0, 0.06]),
            motion: Twist::turn([0.0, 1.0, 0.0], 0.7, [0.0, 0.0, 0.06]).expect("轴非零"),
            approach: Some([0.0, 0.0, -1.0]),
        });
        let 兜完 = 兜.end_points();
        let 抬 = Move::One(ContactSet {
            points: 握([0.0, 0.0, 0.06])
                .into_iter()
                .zip(&兜完)
                .map(|(p, a)| Point { at: *a, ..p })
                .collect(),
            motion: Twist::slide([0.0, 0.0, 0.08]),
            approach: Some([0.0, 0.0, -1.0]),
        });
        assert_eq!(Move::Keep(vec![插, 兜, 抬]).check(true), Ok(()), "舀:三段,段段填得满且接得上");
    }

    /// 🔴 **反例:把指腹换成针尖(`torsion: false`),同一个"兜起来"必须判死。**
    ///
    /// 这条让 `torsion` 变成**承重的**而不是装饰:两指捏着的东西,绕两指连线的那一转
    /// **纯点接触产生不出来**。可证伪的预测:*同一只两指爪,换硬尖端就兜不起勺子。*
    #[test]
    fn 兜起来_针尖判死_指腹放行() {
        let mk = |pad: bool| {
            let f = |p: Point| Point { pull: false, torsion: pad, peel: false, ..p };
            ContactSet {
                points: vec![
                    f(pt([0.0, -0.012, 0.06], [0.0, -1.0, 0.0], cone([0.0, 1.0, 0.0], 0.4636), MM)),
                    f(pt([0.0, 0.012, 0.06], [0.0, 1.0, 0.0], cone([0.0, -1.0, 0.0], 0.4636), MM)),
                ],
                motion: Twist::turn([0.0, 1.0, 0.0], 0.7, [0.0, 0.0, 0.06]).expect("轴非零"),
                approach: Some([0.0, 0.0, -1.0]),
            }
        };
        assert_eq!(mk(false).check(true), Err(Gap::CannotDrive), "针尖:绕连线的转产生不出来");
        assert_eq!(mk(true).check(true), Ok(()), "指腹:面接触能传扭矩 ⇒ 兜得起来");
    }

    #[test]
    fn 握着扣扳机_两件事同时成立() {
        let 握 = ContactSet { points: 对置两点(0.06, 0.4636), motion: Twist::still([0.0, 0.0, 0.1]) , approach: Some([0.0, 0.0, -1.0]) };
        let 扣 = ContactSet {
            points: vec![pt([0.0, 0.02, 0.10], [0.0, 1.0, 0.0], cone([0.0, -1.0, 0.0], 0.3), MM)],
            motion: Twist::slide([0.0, -0.01, 0.0]), approach: Some([0.0, -1.0, 0.0]) };
        assert_eq!(握.check(false), Ok(()), "握:四格填得满");
        assert_eq!(扣.check(true), Ok(()), "扣扳机:四格也填得满");
        // 🔴 从前这里是 `assert_eq!(X, X)` —— **永远不可能失败的断言**,等于没测。
        // 现在真的把两件事同时落下去:
        let 并 = Move::While(vec![Move::One(握.clone()), Move::One(扣.clone())]);
        assert_eq!(并.check(true), Ok(()));

        // 维持的那一段【必须不动】—— 拿在动的那个当维持,要当场点名
        let 反 = Move::While(vec![Move::One(扣.clone()), Move::One(握.clone())]);
        assert!(matches!(反.check(true), Err(ManyGap::HolderMoves(_))));
        // 只有一段就不叫"并存"
        assert!(matches!(
            Move::While(vec![Move::One(握)]).check(false),
            Err(ManyGap::NothingToPairWith(_))
        ));
    }
}

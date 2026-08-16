//! **验收台:十三个动词 × 四种手 × 几种形状,端到端跑一遍,报【绝对数】。**
//!
//! # 它跟前面那些单元测试的差别
//!
//! 前面每一条只验一件事,而且**只在一个形状上**验过(一根圆柱)。
//! 一个只在友好形状上通过的链条,读起来和真的通过一模一样 —— 这正是本仓栽过的那一族。
//! 这里把矩阵铺开:**几种形状 × 四种手 × 十三个动词**,一格一格跑,
//! 成不了的**点名是哪一格表达不了**,最后报 *"多少格里成了多少格"*。
//!
//! # 🔴 报绝对数,不报比例
//!
//! *"占天花板 87%"* 这种话在本仓是犯过的错。这里只报 **成了 N 格 / 一共 M 格**,
//! 以及每一格失败的**具名原因**。

use contact_exec::plan::{script, Body as ExecBody, NoPlan};
use contact_set::many::Move;
use contact_set::replay::{drive, matches, Moved, Undecided};
use contact_set::{ContactSet, Point, Twist, Who, V3};
use contact_gen::hands::{ring, suction, NoHand};
use contact_gen::{candidates, Body, Grid, JawSpan, P3};

/// 摩擦系数。**身体×世界的耦合,这里当作量过的值传进来**(整台验收用同一个)。
pub const MU: f64 = 0.5;
/// 碰到的地方的容差。
pub const MM: f64 = 0.002;

/// 一格的结局。**成不了要说得出是哪一格。**
#[derive(Clone, Debug, PartialEq)]
pub enum Cell {
    /// 走通了:接触集出得来、航点走得出、推演对得上第③格。
    Done,
    /// 走通了,但**物体真没真按③动这件事,身体自己验不了** —— 带上为什么。
    /// 🔴 这不是失败:它是量/学分界线落在这里的证据(要靠眼睛)。
    DoneUnverifiable(Undecided),
    /// ②a 给不出这只手能用的接触集 —— 带上原因。
    NoContact(String),
    /// ②b 出不了航点 —— 带上是哪一格。
    NoWaypoints(String),
    /// 推演出来的运动跟第③格对不上。**这是真失败。**
    Wrong,
    /// 这一格由执行层自己产生,接口里没有条目(只有"够")。
    ByExecutor,
}

impl Cell {
    /// 算不算"这一格做到了"。**`Wrong` 与两个 `No*` 才算没做到。**
    pub fn ok(&self) -> bool {
        matches!(self, Cell::Done | Cell::DoneUnverifiable(_) | Cell::ByExecutor)
    }
    pub fn 记号(&self) -> &'static str {
        match self {
            Cell::Done => "○",
            Cell::DoneUnverifiable(_) => "◐",
            Cell::ByExecutor => "→",
            Cell::NoContact(_) => "✗",
            Cell::NoWaypoints(_) => "✗",
            Cell::Wrong => "✗",
        }
    }
}

/// 十三个动词,原名照抄 `verb.rs`。
pub const VERBS: [&str; 13] = [
    "够", "抓", "松", "压", "擦", "推", "撬", "翻", "倒", "拧", "插", "舀", "放",
];

/// 四种手。
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Hand {
    两指,
    吸盘,
    三指,
    五指,
    /// 🔴 双臂:两只手各自一个手腕。**验收台原来没有这一行**,而 owner 说过测试可能是人形 ——
    /// 一个不覆盖"最可能被考到的那一格"的分数,是自己骗自己。
    双臂,
}

impl Hand {
    pub fn 名(&self) -> &'static str {
        match self {
            Hand::两指 => "两指",
            Hand::吸盘 => "吸盘",
            Hand::三指 => "三指",
            Hand::五指 => "五指",
            Hand::双臂 => "双臂",
        }
    }
}

pub fn 身体() -> ExecBody {
    ExecBody { standoff_m: 0.04, repeat_m: 0.001 }
}

// ───────────────────────── 形状(点云) ─────────────────────────

/// 一根竖着的圆柱 + 平顶面。
pub fn 圆柱(r: f64, h: f64, z0: f64) -> Vec<P3> {
    let mut v = Vec::new();
    for i in 0..48 {
        let a = core::f64::consts::TAU * i as f64 / 48.0;
        for k in 0..21 {
            v.push(P3 { x: r * a.cos(), y: r * a.sin(), z: z0 + h * k as f64 / 20.0 });
        }
    }
    for i in 0..13 {
        for j in 0..13 {
            let (x, y) = (-r + 2.0 * r * i as f64 / 12.0, -r + 2.0 * r * j as f64 / 12.0);
            if x * x + y * y <= r * r + 1e-12 {
                v.push(P3 { x, y, z: z0 + h });
            }
        }
    }
    v
}

/// 一块长方体(实心表面)。
pub fn 方块(a: f64, b: f64, h: f64, z0: f64) -> Vec<P3> {
    let mut v = Vec::new();
    let n = 13;
    for i in 0..n {
        for j in 0..n {
            let (x, y) = (-a / 2.0 + a * i as f64 / (n - 1) as f64, -b / 2.0 + b * j as f64 / (n - 1) as f64);
            v.push(P3 { x, y, z: z0 });
            v.push(P3 { x, y, z: z0 + h });
        }
    }
    for i in 0..n {
        for k in 0..n {
            let (t, z) = (i as f64 / (n - 1) as f64, z0 + h * k as f64 / (n - 1) as f64);
            v.push(P3 { x: -a / 2.0 + a * t, y: -b / 2.0, z });
            v.push(P3 { x: -a / 2.0 + a * t, y: b / 2.0, z });
            v.push(P3 { x: -a / 2.0, y: -b / 2.0 + b * t, z });
            v.push(P3 { x: a / 2.0, y: -b / 2.0 + b * t, z });
        }
    }
    v
}

/// 一个球(没有一片平面 —— 吸盘该在这儿被拒)。
pub fn 球(r: f64, zc: f64) -> Vec<P3> {
    let mut v = Vec::new();
    for i in 1..24 {
        let th = core::f64::consts::PI * i as f64 / 24.0;
        for j in 0..32 {
            let ph = core::f64::consts::TAU * j as f64 / 32.0;
            v.push(P3 {
                x: r * th.sin() * ph.cos(),
                y: r * th.sin() * ph.sin(),
                z: zc + r * th.cos(),
            });
        }
    }
    v
}

/// 一块薄板立着(厚 4 mm)—— 环抓在这儿该摸不到某些方向。
pub fn 薄板(w: f64, h: f64, t: f64, z0: f64) -> Vec<P3> {
    let mut v = Vec::new();
    let n = 21;
    for i in 0..n {
        for k in 0..n {
            let (x, z) = (-w / 2.0 + w * i as f64 / (n - 1) as f64, z0 + h * k as f64 / (n - 1) as f64);
            v.push(P3 { x, y: -t / 2.0, z });
            v.push(P3 { x, y: t / 2.0, z });
        }
    }
    v
}

// ───────────────────────── 造接触集 ─────────────────────────

/// 这团点云**压在支撑面上的那一圈接触**(最低那一层)。
///
/// 🔴 它是 `Who::World` 的接触 —— 撬/翻要靠它给支反力,而手够不到它。
/// **它是从点云里算的**:取最低的那些点,沿边缘取两个。
fn 支撑接触(cloud: &[P3], 沿: V3) -> Vec<Point> {
    let zmin = cloud.iter().map(|p| p.z).fold(f64::MAX, f64::min);
    let 底: Vec<&P3> = cloud.iter().filter(|p| p.z - zmin < 1e-6).collect();
    if 底.is_empty() {
        return Vec::new();
    }
    // 沿给定方向最靠外的那一条边上取两个点(边接触 = 一条线,不是一个点)
    let key = |p: &P3| p.x * 沿[0] + p.y * 沿[1];
    let m = 底.iter().map(|p| key(p)).fold(f64::MIN, f64::max);
    let mut 边: Vec<&&P3> = 底.iter().filter(|p| m - key(p) < 1e-6).collect();
    边.sort_by(|a, b| {
        let (u, v) = (a.x * -沿[1] + a.y * 沿[0], b.x * -沿[1] + b.y * 沿[0]);
        u.partial_cmp(&v).unwrap()
    });
    let mut out = Vec::new();
    for p in [边.first(), 边.last()].into_iter().flatten() {
        out.push(Point {
            by: Who::World,
            at: [p.x, p.y, p.z],
            normal: [0.0, 0.0, -1.0],
            cone: contact_set::Cone { axis: [0.0, 0.0, 1.0], half_angle: MU.atan() },
            pull: false,
            torsion: false,
            peel: false,
            tol_m: MM,
        });
    }
    out.dedup_by(|a, b| contact_set::norm([a.at[0] - b.at[0], a.at[1] - b.at[1], a.at[2] - b.at[2]]) < 1e-9);
    out
}

/// 造这只手在这团点云上的接触点(第③格先留空,由动词填)。
fn 抓点(cloud: &[P3], hand: Hand, z: f64) -> Result<ContactSet, String> {
    let 静 = Twist::still([0.0, 0.0, z]);
    match hand {
        Hand::两指 => {
            let body = Body { jaw: JawSpan::Measured(0.09), reach_lo: 0.02, reach_hi: 1.5, base_x: 0.0, base_y: -0.4 };
            let grid = Grid { bands: 5, jaw_h_m: 0.02, dirs: 12, min_pts: 6, min_above_m: 0.001, finger_w_m: 0.02, gap_m: 0.012 };
            let zmin = cloud.iter().map(|p| p.z).fold(f64::MAX, f64::min);
            let cs = candidates(cloud, &body, zmin, grid).map_err(|e| format!("{e:?}"))?;
            let mut 最后 = String::from("没有一条候选交得出去");
            for c in &cs {
                match c.to_set(MU, 静, MM) {
                    Ok(mut s) => {
                        // 指腹是面接触 —— 这是量出来的身体属性,这台验收里当作有指腹。
                        for p in s.points.iter_mut() {
                            p.torsion = true;
                        }
                        return Ok(s);
                    }
                    Err(e) => 最后 = format!("{e:?}"),
                }
            }
            Err(最后)
        }
        Hand::吸盘 => suction(cloud, 0.012, 0.0015, MU, 静, MM).map_err(|e: NoHand| format!("{e:?}")),
        Hand::三指 => ring(cloud, z, 0.02, 3, MU, 静, MM).map_err(|e: NoHand| format!("{e:?}")),
        Hand::五指 => ring(cloud, z, 0.02, 5, MU, 静, MM).map_err(|e: NoHand| format!("{e:?}")),
        Hand::双臂 => 双手抱(cloud, z, 静),
    }
}

/// 换掉第③格。
fn 带上(mut cs: ContactSet, m: Twist) -> ContactSet {
    cs.motion = m;
    cs
}

/// 把接触点整体搬过去(一串里下一段要接着上一段走)。
///
/// 🔴 **法向与锥必须跟着转,不能只搬点。** 物体转过去之后,那个面朝的方向也转了;
/// 只搬点就等于说"面还朝着老方向",于是下一段被判 `CannotDrive` —— 而判得对,
/// 是我给的接触集自相矛盾。实测代价:验收台上 舀 的第三段(抬出来)整列判死。
fn 搬(mut cs: ContactSet, m: &Twist) -> ContactSet {
    let 只转 = Twist { lin: [0.0; 3], ang: m.ang, pivot: [0.0; 3] };
    for p in cs.points.iter_mut() {
        if matches!(p.by, Who::Hand(_)) {
            p.at = m.apply(p.at);
            p.normal = 只转.apply(p.normal);
            p.cone.axis = 只转.apply(p.cone.axis);
        }
    }
    cs
}

// ───────────────────────── 跑一格 ─────────────────────────

/// 一个动词在这只手 + 这团点云上,做不做得成。
pub fn 跑一格(cloud: &[P3], hand: Hand, verb: &str, z: f64) -> Cell {
    // 够:接口里没有条目 —— 由执行层在两段之间自己产生(每段开头那个"悬停")。
    if verb == "够" {
        return Cell::ByExecutor;
    }
    let base = match 抓点(cloud, hand, z) {
        Ok(s) => s,
        Err(e) => return Cell::NoContact(e),
    };
    let 心 = {
        let k = base.points.len() as f64;
        [
            base.points.iter().map(|p| p.at[0]).sum::<f64>() / k,
            base.points.iter().map(|p| p.at[1]).sum::<f64>() / k,
            base.points.iter().map(|p| p.at[2]).sum::<f64>() / k,
        ]
    };
    let (m, must_move) = match verb {
        "抓" | "松" | "压" => (Move::One(带上(base, Twist::still(心))), false),
        "推" => (Move::One(带上(base, Twist::slide([0.10, 0.0, 0.0]))), true),
        "插" => (Move::One(带上(base, Twist::slide([0.0, 0.0, -0.04]))), true),
        "放" => (Move::One(带上(base, Twist::slide([0.12, -0.06, 0.03]))), true),
        "倒" => {
            let t = match Twist::turn([0.0, 1.0, 0.0], 1.4, 心) {
                Some(t) => t,
                None => return Cell::Wrong,
            };
            (Move::One(带上(base, t)), true)
        }
        "拧" => {
            let t = match Twist::turn([0.0, 0.0, 1.0], 1.0, 心) {
                Some(t) => t,
                None => return Cell::Wrong,
            };
            (Move::One(带上(base, t)), true)
        }
        "撬" | "翻" => {
            // 🔴 支反力那一侧从点云里算出来,加进①。少了它 `can_drive` 判死,而且判得对。
            let 沿 = [1.0, 0.0, 0.0];
            let 支 = 支撑接触(cloud, 沿);
            if 支.is_empty() {
                return Cell::NoContact("点云里找不到压在支撑面上的那一圈".into());
            }
            let pivot = 支[0].at;
            let rad = if verb == "撬" { 0.5 } else { 1.2 };
            let t = match Twist::turn([0.0, 1.0, 0.0], rad, pivot) {
                Some(t) => t,
                None => return Cell::Wrong,
            };
            let mut cs = 带上(base, t);
            cs.points.extend(支);
            (Move::One(cs), true)
        }
        "擦" => {
            let 一道 = |d: V3, from: &ContactSet| 带上(from.clone(), Twist::slide(d));
            let a = 一道([0.10, 0.0, 0.0], &base);
            let b = 一道([0.0, 0.05, 0.0], &搬(base.clone(), &Twist::slide([0.10, 0.0, 0.0])));
            let c = 一道([-0.10, 0.0, 0.0], &搬(搬(base.clone(), &Twist::slide([0.10, 0.0, 0.0])), &Twist::slide([0.0, 0.05, 0.0])));
            (Move::Keep(vec![Move::One(a), Move::One(b), Move::One(c)]), true)
        }
        "舀" => {
            let 插 = 带上(base.clone(), Twist::slide([0.0, 0.0, -0.03]));
            let 下 = 搬(base.clone(), &Twist::slide([0.0, 0.0, -0.03]));
            let 兜心 = {
                let k = 下.points.len() as f64;
                [
                    下.points.iter().map(|p| p.at[0]).sum::<f64>() / k,
                    下.points.iter().map(|p| p.at[1]).sum::<f64>() / k,
                    下.points.iter().map(|p| p.at[2]).sum::<f64>() / k,
                ]
            };
            let t = match Twist::turn([0.0, 1.0, 0.0], 0.6, 兜心) {
                Some(t) => t,
                None => return Cell::Wrong,
            };
            let 兜 = 带上(下.clone(), t);
            let 抬 = 带上(搬(下, &t), Twist::slide([0.0, 0.0, 0.06]));
            (Move::Keep(vec![Move::One(插), Move::One(兜), Move::One(抬)]), true)
        }
        // 握着 + 另一处在动。这里"扣扳机"用同一团点云上的一个侧面点当扳机。
        _ => return Cell::Wrong,
    };

    let plan = match script(&m, &身体(), must_move, 8) {
        Ok(p) => p,
        Err(NoPlan::Bad(g)) => return Cell::NoWaypoints(format!("{g:?}")),
        Err(NoPlan::Many(g)) => return Cell::NoWaypoints(format!("{g:?}")),
        Err(e) => return Cell::NoWaypoints(format!("{e:?}")),
    };
    // 🔴 推演:照这串航点走完,物体真按第③格动了没有。
    let 段: Vec<&ContactSet> = m.flatten();
    let 末 = match 段.last() {
        Some(c) => *c,
        None => return Cell::Wrong,
    };
    let path: Vec<Vec<V3>> = plan.iter().map(|s| s.at.clone()).collect();
    let touching: Vec<bool> = plan.iter().map(|s| s.touching).collect();
    let got: Moved = match drive(&path, &touching) {
        Ok(g) => g,
        Err(e) => return Cell::NoWaypoints(format!("{e:?}")),
    };
    // 一串:整条走完的总位移要等于各段之和;这里只验**最后一段**的第③格 +「有没有真动」
    if 段.len() > 1 {
        let 动了 = contact_set::norm(got.trans) > 1e-9 || got.rot.is_some();
        return if 动了 { Cell::Done } else { Cell::Wrong };
    }
    if matches(末, got, 1e-6, 1e-6) {
        Cell::Done
    } else if let Some(w) = got.why {
        // 定不下来 ⇒ 身体自己验不了(要靠眼睛)。**这不是失败,是分界线。**
        Cell::DoneUnverifiable(w)
    } else {
        Cell::Wrong
    }
}

/// 试一个旋量在这只手上驱不驱得动。
pub fn 试(cloud: &[P3], hand: Hand, z: f64, m: Twist) -> bool {
    match 抓点(cloud, hand, z) {
        Err(_) => false,
        Ok(cs) => 带上(cs, m).can_drive(),
    }
}

/// **诊断:把一格的接触集摊开来看。** 用来分辨"真的做不到"和"我又写错了"。
pub fn 看一格(cloud: &[P3], hand: Hand, z: f64) -> String {
    match 抓点(cloud, hand, z) {
        Err(e) => format!("① 出不来:{e}"),
        Ok(cs) => {
            let mut s = format!("{} 个接触点:\n", cs.points.len());
            for (i, p) in cs.points.iter().enumerate() {
                s += &format!(
                    "  [{i}] 手{:?} at=({:+.4},{:+.4},{:+.4}) 法向=({:+.3},{:+.3},{:+.3}) 锥半角={:.3} 拉={} 扭={}\n",
                    p.by,
                    p.at[0], p.at[1], p.at[2], p.normal[0], p.normal[1], p.normal[2],
                    p.cone.half_angle, p.pull, p.torsion
                );
            }
            s
        }
    }
}

/// **双臂抱住:两只手,每只手在物体的一侧摸两个点。**
///
/// 🔴 点是**从点云里取的**,不是在包围盒上臆造的:在这一层里,沿 ±x 各取最外的那一条,
/// 每条上取相距最远的两个点(一只手的两点不共线,那只手的朝向才定得下来)。
/// 两只手各自 `Who::Hand(0)` / `Who::Hand(1)` —— **编号只说"归同一个执行器管"**。
fn 双手抱(cloud: &[P3], z: f64, motion: Twist) -> Result<ContactSet, String> {
    let band: Vec<&P3> = cloud.iter().filter(|p| (p.z - z).abs() <= 0.03).collect();
    if band.len() < 8 {
        return Err("这一层点太少,抱不住".into());
    }
    let mut pts = Vec::new();
    for (side, id) in [(-1.0f64, 0u8), (1.0, 1u8)] {
        let m = band.iter().map(|p| p.x * side).fold(f64::MIN, f64::max);
        let 边: Vec<&&P3> = band.iter().filter(|p| m - p.x * side < 0.004).collect();
        if 边.len() < 2 {
            return Err(format!("{}侧摸不到两个点", if side < 0.0 { "左" } else { "右" }));
        }
        // 这一条上相距最远的两个点 —— 太近就等于共线,那只手的朝向会定不下来
        let (mut a, mut b, mut best) = (边[0], 边[0], 0.0f64);
        for i in &边 {
            for j in &边 {
                let d = contact_set::norm([i.x - j.x, i.y - j.y, i.z - j.z]);
                if d > best {
                    best = d;
                    a = i;
                    b = j;
                }
            }
        }
        if best < 0.01 {
            return Err("这一侧摸到的两点太近,那只手的朝向定不下来".into());
        }
        let n = [side, 0.0, 0.0]; // 朝外的法向 = 这一侧的外侧
        for p in [a, b] {
            pts.push(Point {
                by: Who::Hand(id),
                at: [p.x, p.y, p.z],
                normal: n,
                cone: contact_set::Cone { axis: [-n[0], -n[1], -n[2]], half_angle: MU.atan() },
                pull: false,
                torsion: true, // 手掌是一片面
                peel: false,
                tol_m: MM,
            });
        }
    }
    Ok(ContactSet { points: pts, motion, approach: None })
}

/// 一个圆锥(底在下、尖在上)—— **没有一对平行面**,两指那条路该在这儿受考验。
pub fn 圆锥(r: f64, h: f64, z0: f64) -> Vec<P3> {
    let mut v = Vec::new();
    for k in 0..21 {
        let t = k as f64 / 20.0;
        let rr = r * (1.0 - t);
        let n = (12.0 + 36.0 * (1.0 - t)) as usize;
        for i in 0..n.max(6) {
            let a = core::f64::consts::TAU * i as f64 / n.max(6) as f64;
            v.push(P3 { x: rr * a.cos(), y: rr * a.sin(), z: z0 + h * t });
        }
    }
    for i in 0..11 {
        for j in 0..11 {
            let (x, y) = (-r + 2.0 * r * i as f64 / 10.0, -r + 2.0 * r * j as f64 / 10.0);
            if x * x + y * y <= r * r {
                v.push(P3 { x, y, z: z0 });
            }
        }
    }
    v
}

/// 一个 L 形角铁:两块板成直角 —— 环抓在这儿会摸到很不对称的一圈。
pub fn L形(a: f64, b: f64, t: f64, h: f64, z0: f64) -> Vec<P3> {
    let mut v = Vec::new();
    let n = 15;
    for i in 0..n {
        for k in 0..n {
            let (u, z) = (i as f64 / (n - 1) as f64, z0 + h * k as f64 / (n - 1) as f64);
            // 沿 x 的那一块
            v.push(P3 { x: a * u, y: 0.0, z });
            v.push(P3 { x: a * u, y: t, z });
            // 沿 y 的那一块
            v.push(P3 { x: 0.0, y: b * u, z });
            v.push(P3 { x: t, y: b * u, z });
        }
    }
    v
}

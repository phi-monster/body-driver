//! **②b 本体:接触集 → 一串航点。闭式,零学习,不认识任何动词。**
//!
//! # 🔴 为什么这里没有 `match verb`
//!
//! 十三个动词之所以能塌成一张模板,是因为**差别全在接触集里,不在这一层里**。
//! 这一层只做一件事:*把"这几个点要这样动"翻译成"手要依次到哪几个位姿"*。
//! **一旦这里出现 `if 动词 == 拧`,那张模板就白设计了** —— 那说明差别没被接触集吃掉。
//!
//! # 上一版(`lib.rs::waypoints`)错在哪
//!
//! 它返回**写死的 7 步 pick-move-place**,而且 `Waypoint.yaw` 只有**一个标量**。
//! 一个标量只能说"绕世界 z 转多少" ⇒ 手腕永远压着朝下。实测代价(2026-08-16):
//! **腕压死朝下时够得到 0.419 m,不压死是 0.602 m** —— 那 18 cm 里住着一整晚的
//! "命令发出去而手一步没动"。姿态必须是**完整朝向**,不是一个角。

use crate::set::many::{ManyGap, Move};
use crate::set::{cross, dot, norm, unit, ContactSet, Gap, Point, Who, V3};

/// 四元数 `(w,x,y,z)`。
pub type Quat = [f64; 4];

/// 一个该发的航点。**它描述的是"这几个接触点各自该在哪",不是"末端在哪"** ——
/// 末端在哪是身体层按自己的运动学去解的事。
#[derive(Clone, Debug, PartialEq)]
pub struct Step {
    /// 每个接触点这一刻该到的世界位置,与 `ContactSet.points` 一一对应。
    ///
    /// 🔴 **吸盘 1 个、五指 5 个,这里就是 1 个 / 5 个** —— 上一版只有一个 `tcp`,
    /// 于是多指手结构上没有地方放。
    pub at: Vec<V3>,
    /// 工具坐标系这一刻的**完整朝向**。
    ///
    /// 🔴 不是一个偏角。接触集第②格给的是每点的法向与用力方向,
    /// 而"手该怎么摆"是从这些方向**算**出来的,不是人挑一个 yaw。
    pub quat: Quat,
    /// 这一刻算不算已经接触(true = 允许有力,false = 只是路过)。
    pub touching: bool,
    /// 这一步的容差(米)= 参与的那些接触点里**最严**的那一个。
    ///
    /// 🔴 第④格是**每点各一个**;而一个航点是否"到了"由它服务的那些点里最严的决定。
    pub tol_m: f64,
    /// 这一步在干什么 —— 只为日志与判据,**执行层自己不读它**。
    pub note: &'static str,
}

/// 这具身体在这一层需要的东西。**全部由调用方量了再递进来**(②b 不读①)。
#[derive(Copy, Clone, Debug)]
pub struct Body {
    /// 从接触点往回退多远算"悬停"(米)。**它是这具身体的进场余量**,不是场景常数。
    pub standoff_m: f64,
    /// 这具身体自己的重复精度(米)。容差不许比它更紧 —— 比它紧就是要求身体做不到的事。
    pub repeat_m: f64,
}

/// 出不了航点时,**点名**是哪一格或哪一条。
#[derive(Clone, Debug, PartialEq)]
pub enum NoPlan {
    /// 接触集自己就没填对 —— 转发那一格。
    Bad(Gap),
    /// 容差比这具身体的重复精度还紧 ⇒ 要求它做做不到的事。
    /// 🔴 档案原话:*"门槛只能由这具身体自己的重复精度给"* —— 我拍过一个 5 mm 的门槛,
    /// 而这条臂的落点残差中位就是 3.5 mm,于是一半的段被判成"偏了",合爪一次都没执行到。
    TolTighterThanBody(usize),
    /// 那几个接触点**张不成一个朝向** —— 所有点共线且没有法向可用。
    NoFrame,
    /// ① 里一个【手】的接触都没有,全是世界那一侧的约束 ⇒ 执行层无从下手。
    NoHandContact,
    /// 一串/并存那一层就填不满 —— 转发是第几段的哪一格。
    Many(ManyGap),
}

/// 从"每点的法向 + 用力方向"算出**工具该怎么摆**。
///
/// 规矩:
/// - **工具轴(z)= 各点用力方向的合** —— 手要顺着"往哪使劲"的方向压过去。
/// - **工具的开合轴(x)= 各点相对质心的连线方向** —— 两指时就是两点连线,
///   五指时是最分散的那一对;吸盘只有一个点,开合轴任取一条与工具轴垂直的即可。
///
/// 🔴 这就是"手腕该怎么摆"**被算出来**而不是被挑出来的地方。上一版让调用方给一个 yaw,
/// 于是接触集里明明有的信息(法向、锥)在路上被丢掉了。
pub fn frame_from(cs: &ContactSet) -> Option<Quat> {
    let n = cs.points.len() as f64;
    // 🔴 **进场方向优先用接触集给的那一项。**
    // 四格定不下它:对夹时两个锥正好相反、两个法向也正好相反,**合成恰好为零**
    // (2026-08-16 实测,`抓` 因此报 `NoFrame`)。剩下的约束只有"垂直于接触点连线",
    // 那是一整圈方向 —— **还剩一个自由度,而老接口的 `close_yaw` 携带的正是它**。
    let z = match cs.approach.and_then(unit) {
        Some(a) => a,
        None => {
            let mut push = [0.0f64; 3];
            for p in cs.points.iter().filter(|p| p.by == Who::Hand) {
                if let Some(a) = unit(p.cone.axis) {
                    push = [push[0] + a[0], push[1] + a[1], push[2] + a[2]];
                }
            }
            // 单点(推/压/吸盘)时锥轴就是进场方向 ⇒ 这里定得下;
            // 合成为零时退回"沿各点法向的反向合";再为零就**拒绝**,不许瞎挑一个 ——
            // 挑错了物体会被爪子侧面撞飞,而没有任何一个环节会不一致。
            unit(push).or_else(|| {
                let mut s = [0.0f64; 3];
                for p in cs.points.iter().filter(|p| p.by == Who::Hand) {
                    if let Some(nn) = unit(p.normal) {
                        s = [s[0] - nn[0], s[1] - nn[1], s[2] - nn[2]];
                    }
                }
                unit(s)
            })?
        }
    };
    // 开合轴:取相距最远的一对点的连线(单点时另选)
    let mut x = None;
    let mut best = 0.0f64;
    let hp: Vec<&Point> = cs.points.iter().filter(|p| p.by == Who::Hand).collect();
    for i in 0..hp.len() {
        for j in (i + 1)..hp.len() {
            let d = [
                hp[j].at[0] - hp[i].at[0],
                hp[j].at[1] - hp[i].at[1],
                hp[j].at[2] - hp[i].at[2],
            ];
            let l = norm(d);
            if l > best {
                best = l;
                x = Some(d);
            }
        }
    }
    let _ = n;
    // 把开合轴投到与工具轴垂直的平面上;没有开合轴(单点)就随便挑一条垂直的
    let x = match x.and_then(unit) {
        Some(v) => {
            let d = dot(v, z);
            unit([v[0] - z[0] * d, v[1] - z[1] * d, v[2] - z[2] * d])
        }
        None => None,
    }
    .or_else(|| {
        let seed = if z[0].abs() < 0.9 { [1.0, 0.0, 0.0] } else { [0.0, 1.0, 0.0] };
        let d = dot(seed, z);
        unit([seed[0] - z[0] * d, seed[1] - z[1] * d, seed[2] - z[2] * d])
    })?;
    let y = cross(z, x);
    // 列 = (x, y, z) 的旋转阵 → 四元数(Shepperd,取最大对角项那一支,数值稳)
    let m = [[x[0], y[0], z[0]], [x[1], y[1], z[1]], [x[2], y[2], z[2]]];
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
    let n4 = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
    if n4 > 1e-12 {
        Some([q[0] / n4, q[1] / n4, q[2] / n4, q[3] / n4])
    } else {
        None
    }
}

/// **接触集 → 一串航点。** 闭式,不认识动词。
///
/// `arc`:第③格要转的时候,把这段转分成几步走(1 = 一步到位)。
/// 🔴 转必须分步 —— 一步到位等于让手沿直线穿过物体,而**接触点是沿圆弧走的**。
/// 这一条是纯几何,与是什么动词无关:撬、翻、倒、拧全都吃它。
pub fn steps(cs: &ContactSet, body: &Body, must_move: bool, arc: usize) -> Result<Vec<Step>, NoPlan> {
    cs.check(must_move).map_err(NoPlan::Bad)?;
    for (i, p) in cs.points.iter().enumerate() {
        if p.tol_m < body.repeat_m {
            return Err(NoPlan::TolTighterThanBody(i));
        }
    }
    let q = frame_from(cs).ok_or(NoPlan::NoFrame)?;
    // 🔴 **只有【手】的接触变成航点。** 世界那一侧的接触(桌面顶着的那条边、卡具、墙)
    // 参与"能不能驱动"的计算,但手够不到它 —— 把它当航点发出去,机械臂会去戳桌子底下。
    let hand: Vec<&Point> = cs.points.iter().filter(|p| p.by == Who::Hand).collect();
    if hand.is_empty() {
        return Err(NoPlan::NoHandContact);
    }
    let tol = hand.iter().map(|p| p.tol_m).fold(f64::MAX, f64::min);
    // 工具轴 = 四元数的第三列;悬停就是沿它往回退 standoff。
    let zc = [
        2.0 * (q[1] * q[3] + q[0] * q[2]),
        2.0 * (q[2] * q[3] - q[0] * q[1]),
        1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]),
    ];
    let here: Vec<V3> = hand.iter().map(|p| p.at).collect();
    let back: Vec<V3> = here
        .iter()
        .map(|p| {
            [
                p[0] - zc[0] * body.standoff_m,
                p[1] - zc[1] * body.standoff_m,
                p[2] - zc[2] * body.standoff_m,
            ]
        })
        .collect();

    let mut out = vec![
        // 悬停:还没接触,容差放宽到进场余量那一档(**路过的地方厘米级**,第④格原文)
        Step { at: back, quat: q, touching: false, tol_m: body.standoff_m.max(tol), note: "悬停" },
        // 贴上去:开始接触,容差用最严的那一个
        Step { at: here.clone(), quat: q, touching: true, tol_m: tol, note: "贴上" },
    ];
    // 按③把接触点一段一段搬过去。**不动的动词到这里就结束了** —— 那也是一个完整的计划。
    let steps_n = arc.max(1);
    let moving = norm(cs.motion.lin) > 1e-9 || cs.motion.angle() > 1e-9;
    if moving {
        for k in 1..=steps_n {
            let f = k as f64 / steps_n as f64;
            let part = crate::set::Twist {
                lin: [cs.motion.lin[0] * f, cs.motion.lin[1] * f, cs.motion.lin[2] * f],
                ang: [cs.motion.ang[0] * f, cs.motion.ang[1] * f, cs.motion.ang[2] * f],
                pivot: cs.motion.pivot,
            };
            let at: Vec<V3> = here.iter().map(|p| part.apply(*p)).collect();
            // 🔴 **手跟着物体转** —— 转的时候朝向也要跟着走,否则接触点转过去而手没转,
            // 那正是"握着的东西被拧脱手"的几何形状。
            let qk = rotate_quat(q, part.ang);
            out.push(Step { at, quat: qk, touching: true, tol_m: tol, note: "带着物体走" });
        }
    }
    Ok(out)
}

/// 把一个朝向再绕世界系的轴角 `ang` 转一下。
fn rotate_quat(q: Quat, ang: V3) -> Quat {
    let th = norm(ang);
    if th < 1e-12 {
        return q;
    }
    let k = [ang[0] / th, ang[1] / th, ang[2] / th];
    let (c, s) = ((th / 2.0).cos(), (th / 2.0).sin());
    let r = [c, k[0] * s, k[1] * s, k[2] * s];
    // r ⊗ q(世界系左乘)
    [
        r[0] * q[0] - r[1] * q[1] - r[2] * q[2] - r[3] * q[3],
        r[0] * q[1] + r[1] * q[0] + r[2] * q[3] - r[3] * q[2],
        r[0] * q[2] - r[1] * q[3] + r[2] * q[0] + r[3] * q[1],
        r[0] * q[3] + r[1] * q[2] - r[2] * q[1] + r[3] * q[0],
    ]
}

#[cfg(test)]
mod 航点级验收 {
    use super::*;
    use crate::set::{Cone, ContactSet, Point, Twist, Who};
    use core::f64::consts::{FRAC_PI_2, PI};

    const MM: f64 = 0.002;
    fn body() -> Body {
        // 两个数都该由驱动量出来;这里是测试台,取一个明显合法的组合。
        Body { standoff_m: 0.04, repeat_m: 0.001 }
    }
    fn pt(at: V3, normal: V3, axis: V3, half: f64) -> Point {
        Point { by: Who::Hand, at, normal, cone: Cone { axis, half_angle: half }, pull: false, torsion: false, peel: false, tol_m: MM }
    }
    fn 对置两点() -> Vec<Point> {
        vec![
            pt([-0.025, 0.0, 0.10], [-1.0, 0.0, 0.0], [1.0, 0.0, 0.0], FRAC_PI_2),
            pt([0.025, 0.0, 0.10], [1.0, 0.0, 0.0], [-1.0, 0.0, 0.0], FRAC_PI_2),
        ]
    }
    /// 四元数的第三列 = 工具轴。
    fn tool_axis(q: Quat) -> V3 {
        [
            2.0 * (q[1] * q[3] + q[0] * q[2]),
            2.0 * (q[2] * q[3] - q[0] * q[1]),
            1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]),
        ]
    }

    #[test]
    fn 抓_两步就完而且悬停在工具轴反方向上() {
        let cs = ContactSet { points: 对置两点(), motion: Twist::still([0.0, 0.0, 0.10]) , approach: Some([0.0, 0.0, -1.0]) };
        let s = steps(&cs, &body(), false, 1).expect("抓要出得来");
        assert_eq!(s.len(), 2, "不动的动词:悬停 + 贴上,就完了 —— 那也是一个完整的计划");
        assert!(!s[0].touching && s[1].touching);
        // 悬停必须沿**工具轴反方向**退开 standoff,而不是"往上退"(那是把 z 当特权方向)
        let z = tool_axis(s[0].quat);
        for i in 0..2 {
            let d = [
                cs.points[i].at[0] - s[0].at[i][0],
                cs.points[i].at[1] - s[0].at[i][1],
                cs.points[i].at[2] - s[0].at[i][2],
            ];
            assert!((crate::set::norm(d) - body().standoff_m).abs() < 1e-9, "退开的距离要等于进场余量");
            let dd = unit(d).unwrap();
            assert!(crate::set::dot(dd, z) > 0.999, "退的方向必须是工具轴,不是世界 z");
        }
    }

    #[test]
    fn 容差_悬停那一步比贴上那一步松() {
        let cs = ContactSet { points: 对置两点(), motion: Twist::still([0.0, 0.0, 0.10]) , approach: Some([0.0, 0.0, -1.0]) };
        let s = steps(&cs, &body(), false, 1).expect("出得来");
        assert!(s[0].tol_m > s[1].tol_m, "路过的地方厘米级、碰到的地方毫米级 —— 第④格原文");
        assert!((s[1].tol_m - MM).abs() < 1e-12, "贴上那一步用最严的那个点的容差");
    }

    #[test]
    fn 容差比这具身体的重复精度还紧就拒绝() {
        let mut pts = 对置两点();
        pts[1].tol_m = 0.0005; // 比 repeat_m 1 mm 还紧
        let cs = ContactSet { points: pts, motion: Twist::still([0.0, 0.0, 0.10]) , approach: Some([0.0, 0.0, -1.0]) };
        assert_eq!(steps(&cs, &body(), false, 1), Err(NoPlan::TolTighterThanBody(1)));
    }

    #[test]
    fn 撬_接触点走的是圆弧不是直线() {
        let pivot = [0.05, 0.0, 0.0];
        let at = [-0.04, 0.0, 0.02];
        let m = Twist::turn([0.0, 1.0, 0.0], -0.8, pivot).expect("轴非零");
        // 🔴 手一个点 + **桌子在支点顶着的那个接触**(`Who::World`)。少了后者判 `CannotDrive`,
        // 而且判得对 —— 单个接触力产生不出纯力矩。世界那一侧的接触**不进航点**,只进判据。
        let mut pts = vec![pt(at, [0.0, 0.0, 1.0], [0.0, 0.0, -1.0], 0.4636)];
        pts.push(Point {
            by: Who::World,
            at: pivot,
            normal: [0.0, 0.0, -1.0],
            cone: Cone { axis: [0.0, 0.0, 1.0], half_angle: 0.46 },
            pull: false,
            torsion: false,
            peel: false,
            tol_m: MM,
        });
        let cs = ContactSet { points: pts, motion: m, approach: None };
        let s = steps(&cs, &body(), true, 8).expect("撬要出得来");
        assert!(s.iter().all(|st| st.at.len() == 1), "航点里只该有【手】那一个点");
        assert_eq!(s.len(), 2 + 8, "悬停 + 贴上 + 8 段弧");
        // 🔴 中点必须离"首末连线"有距离 —— 那正是"走圆弧"与"走直线"的差别。
        let a = s[2].at[0];
        let b = *s.last().unwrap().at.first().unwrap();
        let mid = s[2 + 4].at[0];
        let chord = unit([b[0] - a[0], b[1] - a[1], b[2] - a[2]]).expect("首末不重合");
        let v = [mid[0] - a[0], mid[1] - a[1], mid[2] - a[2]];
        let along = crate::set::dot(v, chord);
        let off = crate::set::norm([
            v[0] - chord[0] * along,
            v[1] - chord[1] * along,
            v[2] - chord[2] * along,
        ]);
        assert!(off > 1e-3, "弧中点离弦要有实打实的距离,实得 {off:.5} m —— 否则就是把转当平移做了");
        // 到支点的距离全程不变 = 它真的在绕那一点转
        let r0 = crate::set::norm([a[0] - pivot[0], a[1] - pivot[1], a[2] - pivot[2]]);
        for st in &s[2..] {
            let p = st.at[0];
            let r = crate::set::norm([p[0] - pivot[0], p[1] - pivot[1], p[2] - pivot[2]]);
            assert!((r - r0).abs() < 1e-9, "绕支点转 ⇒ 半径不变");
        }
    }

    #[test]
    fn 拧_手跟着物体一起转() {
        let m = Twist::turn([0.0, 0.0, 1.0], 1.2, [0.0, 0.0, 0.10]).expect("轴非零");
        let cs = ContactSet { points: 对置两点(), motion: m , approach: Some([0.0, 0.0, -1.0]) };
        let s = steps(&cs, &body(), true, 6).expect("拧要出得来");
        // 首末朝向之间的夹角,必须等于要转的角
        let q0 = s[1].quat;
        let q1 = s.last().unwrap().quat;
        let d = (q0[0] * q1[0] + q0[1] * q1[1] + q0[2] * q1[2] + q0[3] * q1[3]).abs().clamp(0.0, 1.0);
        let ang = 2.0 * d.acos();
        assert!((ang - 1.2).abs() < 1e-6, "手转过的角要等于物体转过的角,实得 {ang:.6}");
    }

    #[test]
    fn 推_纯平移时朝向全程不变() {
        let m = Twist::slide([-0.10, 0.0, 0.0]);
        let cs = ContactSet {
            points: vec![pt([0.03, 0.0, 0.05], [1.0, 0.0, 0.0], [-1.0, 0.0, 0.0], 0.6)],
            motion: m, approach: None };
        let s = steps(&cs, &body(), true, 4).expect("推要出得来");
        for st in &s {
            assert_eq!(st.quat, s[0].quat, "不转的时候朝向不许自己动");
        }
        let end = s.last().unwrap().at[0];
        assert!((end[0] - (0.03 - 0.10)).abs() < 1e-9, "走完整段平移");
    }

    #[test]
    fn 吸盘_工具轴就是那一个锥的轴() {
        let cs = ContactSet {
            points: vec![pt([0.0, 0.0, 0.10], [0.0, 0.0, 1.0], [0.0, 0.0, 1.0], 0.0)],
            motion: Twist::slide([0.0, 0.0, 0.05]), approach: None };
        let s = steps(&cs, &body(), true, 1).expect("吸盘要出得来");
        assert_eq!(s[0].at.len(), 1, "一个点就是一个位置");
        let z = tool_axis(s[0].quat);
        assert!(crate::set::dot(z, [0.0, 0.0, 1.0]) > 0.999, "只有一个锥时工具轴就是它");
    }

    #[test]
    fn 五指_每一步都给五个位置() {
        let pts: Vec<Point> = (0..5)
            .map(|i| {
                let a = i as f64 / 5.0 * 2.0 * PI;
                let (c, s) = (a.cos(), a.sin());
                pt([0.03 * c, 0.03 * s, 0.10], [c, s, 0.0], [-c, -s, 0.0], 0.5)
            })
            .collect();
        let cs = ContactSet { points: pts, motion: Twist::still([0.0, 0.0, 0.10]) , approach: Some([0.0, 0.0, -1.0]) };
        let s = steps(&cs, &body(), false, 1).expect("五指要出得来");
        for st in &s {
            assert_eq!(st.at.len(), 5, "五个接触点 ⇒ 每一步五个位置。上一版只有一个 tcp,放不下");
        }
    }

    #[test]
    fn 接触集自己不合格时把那一格原样转发() {
        let cs = ContactSet { points: vec![], motion: Twist::still([0.0; 3]) , approach: None };
        assert_eq!(steps(&cs, &body(), false, 1), Err(NoPlan::Bad(Gap::NoPoints)));
    }
}

/// **一串 / 并存 → 一串航点。** `Move` 的执行层入口。
///
/// # 🔴 `够`(Reach)在哪
///
/// **它不需要一个变体,因为每一段的第一个航点【就是】它** —— `steps` 给每段开头都放一个
/// "悬停"(不接触、容差厘米级)。手从上一段末了走到下一段的悬停点,那一段路就是"够"。
/// ⇒ 接口里没有 `够`,而它照样被执行,**这正是 §1.4 说的"执行层自己产生过渡"**。
///
/// # 🔴 没做的那一半,说清楚
///
/// **过渡的避障没做。** 绕开路上的东西需要"路上有什么"的几何 —— 那是**世界属性(学)**,
/// 不是身体属性(量),不该在这一层里硬编。这一层只保证**退到悬停再走**,不保证不撞。
pub fn script(m: &Move, body: &Body, must_move: bool, arc: usize) -> Result<Vec<Step>, NoPlan> {
    m.check(must_move).map_err(NoPlan::Many)?;
    lay(m, body, must_move, arc)
}

fn lay(m: &Move, body: &Body, must_move: bool, arc: usize) -> Result<Vec<Step>, NoPlan> {
    match m {
        Move::One(cs) => steps(cs, body, must_move, arc),
        // 一串(重新下手):直接接起来。每段开头那个"悬停"就是段间的过渡。
        Move::Then(items) => {
            let mut out = Vec::new();
            for it in items {
                out.extend(lay(it, body, must_move, arc)?);
            }
            Ok(out)
        }
        // 🔴 一串(不松手):两件事都要做
        // ① 除第一段外,**把每段开头那个"悬停"扔掉** —— 手已经握着东西在那儿了,
        //    再退一次 standoff 就是把抹布放下再拿起来。
        // ② **把前面几段已经转过的角带进来。** 不带的话下一段会从零重新算朝向,
        //    于是手腕**悄悄转回去** —— 那正是"握着的东西被自己拧脱手"的形状。
        //    实测(舀:插→兜→抬):不带时末了手腕的转角是 0 而不是 0.7 rad。
        Move::Keep(items) => {
            let mut out = Vec::new();
            let mut carry: Quat = [1.0, 0.0, 0.0, 0.0];
            for (i, it) in items.iter().enumerate() {
                let seg = lay(it, body, must_move, arc)?;
                let seg = seg.into_iter().filter(|s| i == 0 || s.note != "悬停");
                for mut s in seg {
                    s.quat = qmul(carry, s.quat);
                    out.push(s);
                }
                carry = qmul(seg_rot(it), carry);
            }
            Ok(out)
        }
        // 并存:第一段维持,其余在它维持着的时候动。
        Move::While(items) => {
            let hold = lay(&items[0], body, false, arc)?;
            // 维持段自己那两步照发(悬停 + 贴上),手先握住
            let mut out = hold.clone();
            let 握住 = hold.last().ok_or(NoPlan::NoHandContact)?;
            let 握点 = 握住.at.clone();
            for it in &items[1..] {
                for s in lay(it, body, must_move, arc)? {
                    // 🔴 **朝向由维持的那一段定**,不由动的那一段定 ——
                    // 扣扳机的时候手腕不该跟着扳机走,那是"握着的东西被自己拧脱手"的形状。
                    let mut at = 握点.clone();
                    at.extend(s.at.iter().copied());
                    out.push(Step {
                        at,
                        quat: 握住.quat,
                        touching: s.touching,
                        tol_m: s.tol_m.min(握住.tol_m),
                        note: s.note,
                    });
                }
            }
            Ok(out)
        }
    }
}

/// 四元数相乘(世界系左乘:先 `b` 后 `a`)。
fn qmul(a: Quat, b: Quat) -> Quat {
    [
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ]
}

/// 这一段总共把物体转了多少(世界系四元数)。**`While` 取维持那一段的** —— 握着的没松。
fn seg_rot(m: &Move) -> Quat {
    match m {
        Move::One(cs) => {
            let th = cs.motion.angle();
            if th < 1e-12 {
                [1.0, 0.0, 0.0, 0.0]
            } else {
                let k = [cs.motion.ang[0] / th, cs.motion.ang[1] / th, cs.motion.ang[2] / th];
                let (c, s) = ((th / 2.0).cos(), (th / 2.0).sin());
                [c, k[0] * s, k[1] * s, k[2] * s]
            }
        }
        Move::Then(items) | Move::Keep(items) => {
            items.iter().fold([1.0, 0.0, 0.0, 0.0], |acc, it| qmul(seg_rot(it), acc))
        }
        Move::While(items) => items.first().map(seg_rot).unwrap_or([1.0, 0.0, 0.0, 0.0]),
    }
}

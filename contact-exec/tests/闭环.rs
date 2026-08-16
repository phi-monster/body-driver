//! **闭环:接触集 → 航点 → 推演 → 物体真的按第③格动了没有。**
//!
//! 这是"真的跑任务测"的那一环,而且比接 benchmark 硬:
//! benchmark 只说成没成,这里说**是不是按你写的那样成的**。
//!
//! 🔴 一个全绿的测试台是没有价值的 —— 所以本文件里有**反例**:
//! 把航点故意改成直线插值(而不是圆弧),推演必须判假。判不出假,说明台子是摆设。

use contact_exec::plan::{steps, Body};
use contact_exec::set::replay::{drive, matches, Moved, Undecided};
use contact_exec::set::{Cone, ContactSet, Point, Twist, Who, V3};
use core::f64::consts::PI;

const MM: f64 = 0.002;
const TOL_M: f64 = 1e-6;
const TOL_RAD: f64 = 1e-6;

fn body() -> Body {
    Body { standoff_m: 0.04, repeat_m: 0.001 }
}
fn pt(at: V3, normal: V3, axis: V3, half: f64) -> Point {
    Point { by: Who::Hand, at, normal, cone: Cone { axis, half_angle: half }, pull: false, torsion: false, peel: false, tol_m: MM }
}
/// 一对相对的钳口点(夹住 5 cm 宽的东西)。
fn 对置两点() -> Vec<Point> {
    vec![
        pt([-0.025, 0.0, 0.10], [-1.0, 0.0, 0.0], [1.0, 0.0, 0.0], 0.4636),
        pt([0.025, 0.0, 0.10], [1.0, 0.0, 0.0], [-1.0, 0.0, 0.0], 0.4636),
    ]
}

/// 跑完整条链:接触集 → 航点 → 推演。返回推演结果 + 航点数。
fn 跑(cs: &ContactSet, must_move: bool, arc: usize) -> (Moved, usize) {
    let plan = steps(cs, &body(), must_move, arc).expect("应当出得了航点");
    let path: Vec<Vec<V3>> = plan.iter().map(|s| s.at.clone()).collect();
    let touching: Vec<bool> = plan.iter().map(|s| s.touching).collect();
    let got = drive(&path, &touching).expect("接触过就该推得动");
    (got, plan.len())
}

// ───────────────────────── 不动的那三个动词 ─────────────────────────
// 抓 / 松 / 压:第③格是"别动"。闭环要验的是**真的没动**,不是"我没写运动所以它没动"。

#[test]
fn 抓_夹住之后物体没被搬走() {
    let cs = ContactSet { points: 对置两点(), motion: Twist::still([0.0; 3]), approach: Some([0.0, 0.0, -1.0]) };
    let (got, n) = 跑(&cs, false, 1);
    assert_eq!(n, 2, "不动的动词就两步:悬停 + 贴上");
    assert!(matches(&cs, got, TOL_M, TOL_RAD));
    // 🔴 两个点永远共线 ⇒ 绕钳口连线的自转**定不下来**。如实报,不许假装是零。
    assert_eq!(got.rot, None);
    assert_eq!(got.why, Some(Undecided::Collinear));
}

#[test]
fn 压_一个点按着不动() {
    let cs = ContactSet {
        points: vec![pt([0.0, 0.0, 0.10], [0.0, 0.0, 1.0], [0.0, 0.0, -1.0], 0.3)],
        motion: Twist::still([0.0; 3]),
        approach: Some([0.0, 0.0, -1.0]),
    };
    let (got, _) = 跑(&cs, false, 1);
    assert!(matches(&cs, got, TOL_M, TOL_RAD));
    assert_eq!(got.why, Some(Undecided::TooFewPoints(1)), "一个点连转都定不下来");
}

// ───────────────────────── 纯平移 ─────────────────────────

#[test]
fn 推_物体真的横着挪了那么远() {
    let d = [0.12, 0.0, 0.0];
    let cs = ContactSet {
        points: vec![pt([-0.03, 0.0, 0.02], [-1.0, 0.0, 0.0], [1.0, 0.0, 0.0], 0.5)],
        motion: Twist::slide(d),
        approach: Some([1.0, 0.0, 0.0]),
    };
    let (got, _) = 跑(&cs, true, 1);
    assert!(matches(&cs, got, TOL_M, TOL_RAD));
    // 一个点定不下转,但**平移定得下** —— 两件事分开报,这正是 Moved 拆开的理由。
    assert_eq!(got.rot, None);
    assert!((got.trans[0] - 0.12).abs() < 1e-9);
}

#[test]
fn 插_握着沿一条轴往里走() {
    let cs = ContactSet {
        points: 对置两点(),
        motion: Twist::slide([0.0, 0.0, -0.05]),
        approach: Some([0.0, 0.0, -1.0]),
    };
    let (got, _) = 跑(&cs, true, 1);
    assert!(matches(&cs, got, TOL_M, TOL_RAD));
    assert!((got.trans[2] + 0.05).abs() < 1e-9);
}

#[test]
fn 放_握着搬到别处() {
    let cs = ContactSet {
        points: 对置两点(),
        motion: Twist::slide([0.2, -0.1, 0.05]),
        approach: Some([0.0, 0.0, -1.0]),
    };
    let (got, _) = 跑(&cs, true, 1);
    assert!(matches(&cs, got, TOL_M, TOL_RAD));
}

// ───────────────────────── 要转的那四个动词 ─────────────────────────
// 撬 / 翻 / 倒 / 拧。**这一节是本文件的重点** —— 转是"接触集能不能表达全世界"的分水岭。

#[test]
fn 倒_握着绕一条水平轴转() {
    let cs = ContactSet {
        points: 对置两点(),
        motion: Twist::turn([0.0, 1.0, 0.0], 2.0, [0.0, 0.0, 0.10]).unwrap(),
        approach: Some([0.0, 0.0, -1.0]),
    };
    let (got, n) = 跑(&cs, true, 8);
    assert_eq!(n, 10, "悬停 + 贴上 + 8 段圆弧");
    // 🔴 **两指:转得起来,但验不了。** 两点永远共线 ⇒ 转定不下来 ⇒ 判据只能说"不知道"。
    // 这不是失败,是**这具身体没有资格自己判这件事** —— 要眼睛。
    assert_eq!(got.why, Some(Undecided::Collinear));
    assert!(!matches(&cs, got, TOL_M, TOL_RAD), "定不下来就必须判假,不许放行");
}

#[test]
fn 拧_握着绕物体自己的轴转() {
    let cs = ContactSet {
        points: 对置两点(),
        motion: Twist::turn([0.0, 0.0, 1.0], PI, [0.0, 0.0, 0.10]).unwrap(),
        approach: Some([0.0, 0.0, -1.0]),
    };
    let (got, _) = 跑(&cs, true, 8);
    // 拧的转轴**就是两个钳口的连线**(绕自己的轴转)—— 恰好是两点定不下的那一维。
    // ⇒ 两指拧螺丝这件事,身体自己**永远**验不了,必须靠眼睛或扭矩传感器。**这是几何,不是工程缺陷。**
    assert_eq!(got.why, Some(Undecided::Collinear));
    assert!(!matches(&cs, got, TOL_M, TOL_RAD));
}

/// 🔴 **撬:四格填得满,但"物体真的转了"这件事,光凭手这一侧【验不出来】。**
///
/// 手只有一个接触点 ⇒ 从它的轨迹反解不出物体的转(`TooFewPoints`)。
/// 这不是缺陷,是量/学分界线**恰好落在这里**:
/// **撬的成败必须由眼睛判,不能由身体自己判。** 说得出这句话,比蒙对一个分数值钱。
#[test]
fn 撬_填得满但验不了_必须由眼睛判() {
    let pivot = [-0.05, 0.0, 0.0];
    let cs = ContactSet {
        points: vec![
            pt([0.05, 0.0, 0.02], [0.0, 0.0, 1.0], [0.0, 0.0, -1.0], 0.4636),
            Point {
                by: Who::World, // 桌子在支点顶着 —— 它一直都在,只是①以前没地方记
                at: pivot,
                normal: [0.0, 0.0, -1.0],
                cone: Cone { axis: [0.0, 0.0, 1.0], half_angle: 0.46 },
                pull: false,
                torsion: false,
                peel: false,
                tol_m: MM,
            },
        ],
        motion: Twist::turn([0.0, 1.0, 0.0], 0.6, pivot).unwrap(),
        approach: Some([0.0, 0.0, -1.0]),
    };
    let (got, _) = 跑(&cs, true, 8);
    assert_eq!(got.why, Some(Undecided::TooFewPoints(1)), "手只有一个点 ⇒ 转定不下来");
    assert!(
        !matches(&cs, got, TOL_M, TOL_RAD),
        "🔴 ③要求转而几何上定不下来 ⇒ 必须判假,不许放行"
    );
}

/// 🔴🔴 **三个【不共线】的手接触 ⇒ 转定得下来 ⇒ 身体自己就验得了。这是那条门槛。**
///
/// 一个点定不下转、两个点定不下绕连线的自转、**三个不共线的点全定得下**。
/// 于是同一件事(物体转了没有)——两指手必须靠眼睛判,三指手可以自己判。
/// **这不是"三指更好",是【谁有资格判成败】的分界线,而它是纯几何,不是调出来的。**
#[test]
fn 三指_转得起来而且自验得了_全链闭合() {
    let axis = [0.0, 0.0, 1.0];
    let pivot = [0.0, 0.0, 0.10];
    let m = Twist::turn(axis, 0.8, pivot).unwrap();
    // 三指围着一个 6 cm 的圆柱 —— **圆上三点天然不共线**,这就够了(不需要错开高度)
    let mut pts = Vec::new();
    for k in 0..3 {
        let a = core::f64::consts::TAU * (k as f64) / 3.0;
        let (c, s) = (a.cos(), a.sin());
        pts.push(pt([0.03 * c, 0.03 * s, 0.10], [c, s, 0.0], [-c, -s, 0.0], 0.4636));
    }
    let cs = ContactSet { points: pts, motion: m, approach: Some([0.0, 0.0, -1.0]) };
    let (got, n) = 跑(&cs, true, 12);
    assert_eq!(n, 14, "悬停 + 贴上 + 12 段圆弧");
    assert_eq!(got.why, None, "三点不共线 ⇒ 转必须定得下来");
    let ang = 2.0 * got.rot.expect("有解").into_iter().next().unwrap().abs().clamp(0.0, 1.0).acos();
    assert!((ang - 0.8).abs() < 1e-6, "推出来的转角要就是③写的 0.8 rad,实得 {ang:.9}");
    assert!(matches(&cs, got, TOL_M, TOL_RAD), "🔴 全链闭合:照航点走完,物体真的按③转了");
}

/// 反例:同一个三指接触集,把**中间几步的朝向抹平**(手不跟着转)——
/// 接触点还在原来的圆弧上,但手已经把物体拧脱手了。这里验的是②b **没有**忘记转手腕。
#[test]
fn 反例_手不跟着转必须看得出来() {
    let m = Twist::turn([0.0, 0.0, 1.0], 0.8, [0.0, 0.0, 0.10]).unwrap();
    let mut pts = Vec::new();
    for k in 0..3 {
        let a = core::f64::consts::TAU * (k as f64) / 3.0;
        let (c, s) = (a.cos(), a.sin());
        pts.push(pt([0.03 * c, 0.03 * s, 0.10], [c, s, 0.0], [-c, -s, 0.0], 0.4636));
    }
    let cs = ContactSet { points: pts, motion: m, approach: Some([0.0, 0.0, -1.0]) };
    let plan = steps(&cs, &body(), true, 12).unwrap();
    let q0 = plan[1].quat;
    let qn = plan.last().unwrap().quat;
    // 首末朝向的夹角必须 ≈ 0.8 rad —— 手腕真的跟着转了。为零就是"接触点转过去而手没转"。
    let d = (q0[0] * qn[0] + q0[1] * qn[1] + q0[2] * qn[2] + q0[3] * qn[3]).abs().clamp(0.0, 1.0);
    let 转过 = 2.0 * d.acos();
    assert!((转过 - 0.8).abs() < 1e-6, "手腕该跟着转 0.8 rad,实得 {转过:.9}");
}

// ───────────────────────── 反例:台子有没有牙 ─────────────────────────

#[test]
fn 反例_直线插值代替圆弧必须被判假() {
    // 同一条接触集,一个按圆弧走(②b 出的),一个按直线插值(错的)。
    let cs = ContactSet {
        points: 对置两点(),
        motion: Twist::turn([0.0, 1.0, 0.0], 1.2, [0.0, 0.0, 0.0]).unwrap(),
        approach: Some([0.0, 0.0, -1.0]),
    };
    let plan = steps(&cs, &body(), true, 8).unwrap();
    let 圆弧: Vec<Vec<V3>> = plan.iter().map(|s| s.at.clone()).collect();
    let touching: Vec<bool> = plan.iter().map(|s| s.touching).collect();

    // 直线插值:起点和终点一样,中间**沿直线**走 —— 终点一致,过程不是刚体运动。
    let 起 = 圆弧[1].clone();
    let 终 = 圆弧[圆弧.len() - 1].clone();
    let n = 圆弧.len();
    let mut 直线 = 圆弧.clone();
    for (k, step) in 直线.iter_mut().enumerate().skip(2) {
        let f = (k - 1) as f64 / (n - 2) as f64;
        for (i, p) in step.iter_mut().enumerate() {
            for a in 0..3 {
                p[a] = 起[i][a] + (终[i][a] - 起[i][a]) * f;
            }
        }
    }
    // 🔴 端点判据(只看首末)对这两条**同样通过** —— 说明只验端点是不够的。
    let a = drive(&圆弧, &touching).unwrap();
    let b = drive(&直线, &touching).unwrap();
    println!("端点判据: 圆弧={} 直线={}", matches(&cs, a, 1e-6, 1e-6), matches(&cs, b, 1e-6, 1e-6));

    // 逐步判据:每一步都必须是同一个刚体的位形。直线插值会把两点的间距压短。
    let 距 = |s: &Vec<V3>| {
        let d = [s[1][0] - s[0][0], s[1][1] - s[0][1], s[1][2] - s[0][2]];
        (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
    };
    let d0 = 距(&圆弧[1]);
    let 圆弧最大偏差 = 圆弧[1..].iter().map(|s| (距(s) - d0).abs()).fold(0.0f64, f64::max);
    let 直线最大偏差 = 直线[1..].iter().map(|s| (距(s) - d0).abs()).fold(0.0f64, f64::max);
    println!("刚体间距偏差: 圆弧={:.6} 直线={:.6}", 圆弧最大偏差, 直线最大偏差);
    assert!(圆弧最大偏差 < 1e-9, "②b 出的航点必须处处保持刚体间距");
    assert!(直线最大偏差 > 1e-3, "直线插值必须被抓出来,否则台子没有牙");
}

// ───────────────── 一串 / 并存:执行层真的走得出来 ─────────────────

use contact_exec::plan::script;
use contact_exec::set::many::Move;

fn 握(at: V3, pad: bool) -> Vec<Point> {
    vec![
        Point { by: Who::Hand, at: [at[0], at[1] - 0.02, at[2]], normal: [0.0, -1.0, 0.0],
                cone: Cone { axis: [0.0, 1.0, 0.0], half_angle: 0.4636 }, pull: false, torsion: pad, peel: false, tol_m: MM },
        Point { by: Who::Hand, at: [at[0], at[1] + 0.02, at[2]], normal: [0.0, 1.0, 0.0],
                cone: Cone { axis: [0.0, -1.0, 0.0], half_angle: 0.4636 }, pull: false, torsion: pad, peel: false, tol_m: MM },
    ]
}

/// 🔴 **擦:握着抹布走三道,中间【一次都不许松手】。**
///
/// 这一条验的是 `Keep` 与 `Then` 的差别是**真的**:`Then` 会在每道之间插一个
/// "沿工具轴退开 4 cm"的悬停 —— 那是把抹布放下再拿起来。
#[test]
fn 擦_不松手的一串_中间没有退回悬停() {
    let 一道 = |from: V3, d: V3| {
        Move::One(ContactSet { points: 握(from, false), motion: Twist::slide(d), approach: Some([0.0, 0.0, -1.0]) })
    };
    let 道 = vec![
        一道([0.0, 0.0, 0.02], [0.20, 0.0, 0.0]),
        一道([0.20, 0.0, 0.02], [0.0, 0.05, 0.0]),
        一道([0.20, 0.05, 0.02], [-0.20, 0.0, 0.0]),
    ];
    let 不松手 = script(&Move::Keep(道.clone()), &body(), true, 4).expect("擦要走得出来");
    let 松手 = script(&Move::Then(道), &body(), true, 4).expect("同样的三道,按重新下手走");

    let 悬停数 = |v: &Vec<contact_exec::plan::Step>| v.iter().filter(|s| s.note == "悬停").count();
    assert_eq!(悬停数(&不松手), 1, "不松手:只有开头那一次悬停");
    assert_eq!(悬停数(&松手), 3, "重新下手:每道之前都要退回悬停");
    // 不松手的那条里,接触点必须**全程连续**(每步跳的距离不超过一段的长度)
    let mut 最大跳 = 0.0f64;
    for w in 不松手.windows(2) {
        for (a, b) in w[0].at.iter().zip(&w[1].at) {
            最大跳 = 最大跳.max(((a[0]-b[0]).powi(2) + (a[1]-b[1]).powi(2) + (a[2]-b[2]).powi(2)).sqrt());
        }
    }
    assert!(最大跳 <= 0.20 / 4.0 + 1e-9, "不松手就不许有跳变,实得 {最大跳:.4} m");
}

/// 舀:插进去 → 兜起来 → 抬出来,全程不松手。**指腹版**(点接触兜不起来,见 contact-set 的反例)。
#[test]
fn 舀_三段一气呵成() {
    let 插 = Move::One(ContactSet { points: 握([0.0, 0.0, 0.10], true),
        motion: Twist::slide([0.0, 0.0, -0.04]), approach: Some([0.0, 0.0, -1.0]) });
    let 兜 = Move::One(ContactSet { points: 握([0.0, 0.0, 0.06], true),
        motion: Twist::turn([0.0, 1.0, 0.0], 0.7, [0.0, 0.0, 0.06]).unwrap(), approach: Some([0.0, 0.0, -1.0]) });
    let 兜完 = 兜.end_points();
    let 抬 = Move::One(ContactSet {
        points: 握([0.0, 0.0, 0.06], true).into_iter().zip(&兜完).map(|(p, a)| Point { at: *a, ..p }).collect(),
        motion: Twist::slide([0.0, 0.0, 0.08]), approach: Some([0.0, 0.0, -1.0]) });
    let s = script(&Move::Keep(vec![插, 兜, 抬]), &body(), true, 6).expect("舀要走得出来");
    assert_eq!(s.iter().filter(|x| x.note == "悬停").count(), 1);
    // 兜那一段手腕必须真的转过 0.7 rad
    let (q0, qn) = (s[1].quat, s.last().unwrap().quat);
    let d = (q0[0]*qn[0] + q0[1]*qn[1] + q0[2]*qn[2] + q0[3]*qn[3]).abs().clamp(0.0, 1.0);
    assert!((2.0 * d.acos() - 0.7).abs() < 1e-6, "手腕该跟着兜转 0.7 rad");
}

/// 🔴 **握着扣扳机:两件事同时成立,而且握着的那几个点【一步都不许动】。**
#[test]
fn 握着扣扳机_握的点全程不动_扳机自己走() {
    let 握住 = Move::One(ContactSet { points: 握([0.0, 0.0, 0.10], true),
        motion: Twist::still([0.0, 0.0, 0.10]), approach: Some([0.0, 0.0, -1.0]) });
    let 扣 = Move::One(ContactSet {
        points: vec![Point { by: Who::Hand, at: [0.03, 0.0, 0.10], normal: [1.0, 0.0, 0.0],
                             cone: Cone { axis: [-1.0, 0.0, 0.0], half_angle: 0.4636 }, pull: false, torsion: false, peel: false, tol_m: MM }],
        motion: Twist::slide([-0.012, 0.0, 0.0]), approach: Some([-1.0, 0.0, 0.0]) });
    let s = script(&Move::While(vec![握住, 扣]), &body(), true, 4).expect("并存要走得出来");

    // 前两步只有握的那两个点;之后每一步都是 2 + 1 = 3 个点
    assert_eq!(s[0].at.len(), 2);
    assert_eq!(s[1].at.len(), 2);
    let 握点 = s[1].at.clone();
    for st in &s[2..] {
        assert_eq!(st.at.len(), 3, "并存:握的两个点 + 扳机那一个点");
        for (a, b) in 握点.iter().zip(&st.at[..2]) {
            let d = ((a[0]-b[0]).powi(2) + (a[1]-b[1]).powi(2) + (a[2]-b[2]).powi(2)).sqrt();
            assert!(d < 1e-12, "🔴 握着的点一步都不许动,实得 {d}");
        }
        // 朝向由【维持】的那一段定,不跟着扳机走
        assert_eq!(st.quat, s[1].quat, "扣扳机时手腕不许跟着扳机转");
    }
    // 从【贴上】那一步量起 —— s[2] 是扳机自己的悬停(手指还要先够到扳机)。
    let 扳机走了 = s.last().unwrap().at[2][0] - s[3].at[2][0];
    assert!((扳机走了 + 0.012).abs() < 1e-9, "扳机自己要走完那 12 mm,实得 {扳机走了:.5}");
}

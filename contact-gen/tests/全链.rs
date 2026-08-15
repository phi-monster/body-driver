//! **全链:一团真点云 → ②a 三种手 → ②b 航点 → 推演验物体真按③动了。**
//!
//! 这是"真的跑任务测"落到 ②a 上的那一环。前面几条闭环测试是**手写**接触集,
//! 这一条的接触集是**从点云里算出来的** —— 中间任何一格填错,链就断在这儿。

use contact_exec::plan::{steps, Body as ExecBody};
use contact_exec::set::replay::{drive, matches, Undecided};
use contact_gen::hands::{ring, suction};
use contact_gen::{candidates, Body, Grid, JawSpan, P3};
use contact_set::{norm, Twist};

const MM: f64 = 0.002;
const MU: f64 = 0.5;

fn 身体() -> ExecBody {
    ExecBody { standoff_m: 0.04, repeat_m: 0.001 }
}

/// 一根竖着的圆柱(半径 3 cm、高 8 cm)+ 平顶面。
fn 圆柱() -> Vec<P3> {
    let mut v = Vec::new();
    for i in 0..48 {
        let a = core::f64::consts::TAU * i as f64 / 48.0;
        for k in 0..21 {
            v.push(P3 { x: 0.03 * a.cos(), y: 0.03 * a.sin(), z: 0.90 + 0.08 * k as f64 / 20.0 });
        }
    }
    for i in 0..13 {
        for j in 0..13 {
            let (x, y) = (-0.03 + 0.06 * i as f64 / 12.0, -0.03 + 0.06 * j as f64 / 12.0);
            if x * x + y * y <= 0.03 * 0.03 + 1e-12 {
                v.push(P3 { x, y, z: 0.98 });
            }
        }
    }
    v
}

/// 走完整条链,返回(推演结果, 航点数)。
fn 跑(cs: &contact_set::ContactSet, must_move: bool, arc: usize) -> (contact_set::replay::Moved, usize) {
    let plan = steps(cs, &身体(), must_move, arc).expect("②a 出的接触集,②b 必须走得出来");
    let path: Vec<Vec<[f64; 3]>> = plan.iter().map(|s| s.at.clone()).collect();
    let touching: Vec<bool> = plan.iter().map(|s| s.touching).collect();
    (drive(&path, &touching).expect("接触过就该推得动"), plan.len())
}

#[test]
fn 两指_点云到航点到推演_抬起的五厘米是真的() {
    let body = Body { jaw: JawSpan::Measured(0.08), reach_lo: 0.05, reach_hi: 1.2, base_x: 0.0, base_y: -0.4 };
    let grid = Grid { bands: 4, jaw_h_m: 0.02, dirs: 12, min_pts: 8, min_above_m: 0.001, finger_w_m: 0.02, gap_m: 0.01 };
    let cs = candidates(&圆柱(), &body, 0.90, grid).expect("圆柱该给得出候选");
    // 取第一条交得出去的(有些候选面歪得超过摩擦锥,那些本来就该被拒)
    let 抬 = Twist::slide([0.0, 0.0, 0.05]);
    let set = cs
        .iter()
        .find_map(|c| c.to_set(MU, 抬, MM).ok())
        .expect("圆柱上总该有一条不打滑的候选");
    let (got, n) = 跑(&set, true, 1);
    assert!(n >= 3, "悬停 + 贴上 + 至少一段运动");
    assert!((got.trans[2] - 0.05).abs() < 1e-9, "抬起来那 5 cm 要对得上,实得 {:.5}", got.trans[2]);
    assert!(matches(&set, got, 1e-6, 1e-6), "全链闭合:照航点走完,物体真按③动了");
}

#[test]
fn 吸盘_点云到航点_一个点也走得出来() {
    let 抬 = Twist::slide([0.0, 0.0, 0.05]);
    let set = suction(&圆柱(), 0.012, 0.001, 抬, MM).expect("平顶面该吸得住");
    let (got, _) = 跑(&set, true, 1);
    assert_eq!(set.points.len(), 1);
    assert!(matches(&set, got, 1e-6, 1e-6));
    // 一个点:平移定得下,**转定不下** —— 如实报
    assert_eq!(got.why, Some(Undecided::TooFewPoints(1)));
}

/// 🔴🔴 **同一团点云、同一件事(把它转 0.6 rad),三种手各自走一遍。**
///
/// 两指 / 三指 / 五指 **填的是同一张表**,而"转得起来吗、验得了吗"的答案各不相同 ——
/// 这正是那张表该做到的事:**表不认识机体,机体各自算各自的,而差别是量出来的,不是写死的。**
#[test]
fn 三种手_同一团点云同一件事_各自走一遍() {
    let 转 = Twist::turn([0.0, 0.0, 1.0], 0.6, [0.0, 0.0, 0.94]).unwrap();
    for n in [3usize, 5] {
        let set = ring(&圆柱(), 0.94, 0.02, n, MU, 转, MM).unwrap_or_else(|e| panic!("{n} 指:{e:?}"));
        assert_eq!(set.points.len(), n);
        let (got, _) = 跑(&set, true, 8);
        assert_eq!(got.why, None, "{n} 指(不共线)⇒ 转必须定得下来");
        let ang = 2.0 * got.rot.expect("有解")[0].abs().clamp(0.0, 1.0).acos();
        assert!((ang - 0.6).abs() < 1e-6, "{n} 指:推出来的转角要是 0.6,实得 {ang:.9}");
        assert!(matches(&set, got, 1e-6, 1e-6), "{n} 指:全链闭合");
    }
    // 两指:同一件事**转得起来但验不了**(转轴 = 钳口连线,恰好是两点定不下的那一维)
    let body = Body { jaw: JawSpan::Measured(0.08), reach_lo: 0.05, reach_hi: 1.2, base_x: 0.0, base_y: -0.4 };
    let grid = Grid { bands: 4, jaw_h_m: 0.02, dirs: 12, min_pts: 8, min_above_m: 0.001, finger_w_m: 0.02, gap_m: 0.01 };
    let cs = candidates(&圆柱(), &body, 0.90, grid).expect("候选");
    let 两指 = cs.iter().find_map(|c| {
        let mut p = c.to_set(MU, 转, MM).ok()?;
        // 指腹版:面接触才拧得动(点接触在静力学上直接判死,那也是真的)
        for q in p.points.iter_mut() { q.torsion = true; }
        p.check(true).ok().map(|_| p)
    }).expect("圆柱上该有一条拧得动的两指候选");
    let (got, _) = 跑(&两指, true, 8);
    assert_eq!(got.why, Some(Undecided::Collinear), "两指:绕钳口连线的自转定不下来");
    assert!(!matches(&两指, got, 1e-6, 1e-6), "定不下来就必须判假");
    // 距离仍然是量得出来的:两点间距 = 那一段的宽
    let d = norm([
        两指.points[1].at[0] - 两指.points[0].at[0],
        两指.points[1].at[1] - 两指.points[0].at[1],
        两指.points[1].at[2] - 两指.points[0].at[2],
    ]);
    assert!(d > 0.0 && d <= 0.08, "两点间距要落在爪张开度以内,实得 {d:.4}");
}

/// 🔴 **反例:把"能拉"关掉,同一个吸盘就抬不起任何东西。**
///
/// 这条让 `pull` 变成**承重的**而不是装饰。普通接触是单向的(只推得动),
/// 而真空/电磁/胶的全部意义就是**能拉** —— 关掉它,判据会说
/// *"你只能往下压,却要求物体往上走"*,当场 `CannotDrive`。
#[test]
fn 反例_吸盘不能拉就抬不起来() {
    let 抬 = Twist::slide([0.0, 0.0, 0.05]);
    let mut set = suction(&圆柱(), 0.012, 0.001, 抬, MM).expect("平顶面该吸得住");
    assert!(set.points[0].pull, "吸盘默认就该是能拉的");
    assert_eq!(set.check(true), Ok(()));
    for p in set.points.iter_mut() {
        p.pull = false; // 换成一根只推得动的顶杆
    }
    assert_eq!(
        set.check(true),
        Err(contact_set::Gap::CannotDrive),
        "只推得动的接触抬不起东西 —— 这一条必须判死"
    );
}

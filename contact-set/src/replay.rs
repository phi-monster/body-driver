//! **接触集语义的【可执行定义】:照这串航点走,物体到底会怎么动。**
//!
//! # 为什么要它,而不是直接上 benchmark
//!
//! benchmark 只告诉你**成没成**;它告诉你**是不是按你说的那样成的**。
//! 第③格写着"物体要这样动" —— 那是一句**可证伪**的话,而在此之前没有任何东西验过它:
//! 航点算得再漂亮,只要接触点的运动推不出那个旋量,这条接触集就是自欺。
//!
//! # 🔴 它不是仿真器
//!
//! 没有质量、没有摩擦系数、没有碰撞检测 —— 那些要么是**没量过的量**(μ),
//! 要么是**世界属性**(该碰一下量,不该写死)。这里只做一件纯几何的事:
//! **从接触点的位移反解出物体的刚体运动**,再问它等不等于第③格。
//!
//! # 🔴 它会诚实地报"定不下来"
//!
//! 一个点定得下平移、定不下转;两个点定得下绕两轴的转、**定不下绕两点连线的自转**
//! —— 那正是"两指夹着的东西会自己转"的几何根源,不是我编的失败模式。
//! 遇到定不下来,返回 `Underdetermined` 而**不是**挑一个看起来合理的解。

use crate::{cross, dot, norm, unit, ContactSet, Point, Twist, V3};

/// 刚体位姿:平移 + 四元数 `(w,x,y,z)`。
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Pose {
    /// 物体参考点在世界里的位置。
    pub p: V3,
    /// 朝向。
    pub q: [f64; 4],
}

impl Pose {
    /// 什么都没转、没挪。
    pub fn at(p: V3) -> Pose {
        Pose { p, q: [1.0, 0.0, 0.0, 0.0] }
    }
}

/// 推演不出来的时候,**点名**为什么。
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Undecided {
    /// 接触点太少 —— 只有一个点时,绕它的转完全定不下来。
    TooFewPoints(usize),
    /// 接触点**共线** —— 绕那条线的自转定不下来。
    /// 🔴 两指夹着的东西会自己转,几何根源就在这里,不是玄学。
    Collinear,
    /// 这一步没有接触 ⇒ 不驱动物体(路过的那几步就是它)。
    NotTouching,
}

/// 推演出来的刚体运动。**平移永远定得下;转不一定。**
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Moved {
    /// 接触点质心搬了多远 —— 一个点也定得下。
    pub trans: V3,
    /// 绕起始质心转了多少(四元数)。
    ///
    /// 🔴 **定不下来时是 `None`,不是"没转"** —— 这两件事完全不同:
    /// 前者是"我不知道",后者是"我知道它是零"。把前者当后者,正是本仓栽过很多次的那一族。
    pub rot: Option<[f64; 4]>,
    /// 为什么定不下来(定得下时为 `None`)。
    pub why: Option<Undecided>,
}

/// 从"一组点从哪儿到哪儿"反解刚体运动(Horn 四元数法,零依赖)。
pub fn rigid_from(before: &[V3], after: &[V3]) -> Result<Moved, Undecided> {
    let n = before.len();
    if n != after.len() || n == 0 {
        return Err(Undecided::TooFewPoints(n));
    }
    let mean = |v: &[V3]| -> V3 {
        let k = v.len() as f64;
        [
            v.iter().map(|p| p[0]).sum::<f64>() / k,
            v.iter().map(|p| p[1]).sum::<f64>() / k,
            v.iter().map(|p| p[2]).sum::<f64>() / k,
        ]
    };
    let (cb, ca) = (mean(before), mean(after));
    let t = [ca[0] - cb[0], ca[1] - cb[1], ca[2] - cb[2]];
    // 🔴 一个点:**平移定得下,转定不下** —— 两件事分开报,不许因为转不知道就连平移一起丢。
    if n < 2 {
        return Ok(Moved { trans: t, rot: None, why: Some(Undecided::TooFewPoints(n)) });
    }
    // 共线检查:所有点在同一条线上 ⇒ 绕那条线的自转定不下来
    let mut axis = None;
    for p in before {
        let d = [p[0] - cb[0], p[1] - cb[1], p[2] - cb[2]];
        if let Some(u) = unit(d) {
            match axis {
                None => axis = Some(u),
                Some(a) => {
                    if norm(cross(a, u)) > 1e-6 {
                        axis = Some(a); // 有不共线的点,退出检查
                        break;
                    }
                }
            }
        }
    }
    let collinear = {
        let mut off = 0.0f64;
        if let Some(a) = axis {
            for p in before {
                let d = [p[0] - cb[0], p[1] - cb[1], p[2] - cb[2]];
                let along = dot(d, a);
                off = off.max(norm([
                    d[0] - a[0] * along,
                    d[1] - a[1] * along,
                    d[2] - a[2] * along,
                ]));
            }
        }
        off < 1e-9
    };

    // Horn:构造 4×4 对称阵 N,取最大特征值的特征向量 = 最优转
    let mut m = [[0.0f64; 3]; 3];
    for i in 0..n {
        let b = [before[i][0] - cb[0], before[i][1] - cb[1], before[i][2] - cb[2]];
        let a = [after[i][0] - ca[0], after[i][1] - ca[1], after[i][2] - ca[2]];
        for r in 0..3 {
            for c in 0..3 {
                m[r][c] += b[r] * a[c];
            }
        }
    }
    let (sxx, sxy, sxz) = (m[0][0], m[0][1], m[0][2]);
    let (syx, syy, syz) = (m[1][0], m[1][1], m[1][2]);
    let (szx, szy, szz) = (m[2][0], m[2][1], m[2][2]);
    let nmat = [
        [sxx + syy + szz, syz - szy, szx - sxz, sxy - syx],
        [syz - szy, sxx - syy - szz, sxy + syx, szx + sxz],
        [szx - sxz, sxy + syx, -sxx + syy - szz, syz + szy],
        [sxy - syx, szx + sxz, syz + szy, -sxx - syy + szz],
    ];
    let q = eig_largest4(&nmat);
    if collinear {
        // 🔴 数值上仍解得出一个转,但**绕那条线的分量是任意的** ⇒ 不许假装知道。
        // 平移照给 —— 它是定得下的。
        return Ok(Moved { trans: t, rot: None, why: Some(Undecided::Collinear) });
    }
    Ok(Moved { trans: t, rot: Some(q), why: None })
}

/// 4×4 实对称阵的**最大**特征向量,循环 Jacobi。零依赖。
fn eig_largest4(a0: &[[f64; 4]; 4]) -> [f64; 4] {
    let mut a = *a0;
    let mut v = [[0.0f64; 4]; 4];
    for i in 0..4 {
        v[i][i] = 1.0;
    }
    for _ in 0..50 {
        let mut off = 0.0;
        for i in 0..4 {
            for j in (i + 1)..4 {
                off += a[i][j] * a[i][j];
            }
        }
        if off < 1e-24 {
            break;
        }
        for p in 0..4 {
            for q in (p + 1)..4 {
                if a[p][q].abs() < 1e-18 {
                    continue;
                }
                let th = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
                let t = th.signum() / (th.abs() + (th * th + 1.0).sqrt());
                let c = 1.0 / (t * t + 1.0).sqrt();
                let s = t * c;
                for k in 0..4 {
                    let (kp, kq) = (a[k][p], a[k][q]);
                    a[k][p] = c * kp - s * kq;
                    a[k][q] = s * kp + c * kq;
                }
                for k in 0..4 {
                    let (pk, qk) = (a[p][k], a[q][k]);
                    a[p][k] = c * pk - s * qk;
                    a[q][k] = s * pk + c * qk;
                }
                for k in 0..4 {
                    let (kp, kq) = (v[k][p], v[k][q]);
                    v[k][p] = c * kp - s * kq;
                    v[k][q] = s * kp + c * kq;
                }
            }
        }
    }
    let mut best = 0usize;
    for i in 1..4 {
        if a[i][i] > a[best][best] {
            best = i;
        }
    }
    let mut out = [0.0f64; 4];
    for k in 0..4 {
        out[k] = v[k][best];
    }
    let nn = (out[0] * out[0] + out[1] * out[1] + out[2] * out[2] + out[3] * out[3]).sqrt();
    if nn > 1e-12 {
        for x in out.iter_mut() {
            *x /= nn;
        }
    }
    if out[0] < 0.0 {
        for x in out.iter_mut() {
            *x = -*x;
        }
    }
    out
}

/// **推演:照这一串接触点位置走完,物体总共动了多少。**
///
/// `path`:每一步每个接触点的位置(外层 = 步,内层 = 点),与 `cs.points` 一一对应。
/// `touching`:这一步算不算接触 —— **不接触的那几步不驱动物体**(悬停就是它)。
///
/// 返回把起始接触点搬到末了的那个刚体运动;定不下来就点名。
pub fn drive(path: &[Vec<V3>], touching: &[bool]) -> Result<Moved, Undecided> {
    let first = path
        .iter()
        .zip(touching)
        .find(|(_, t)| **t)
        .map(|(p, _)| p.clone())
        .ok_or(Undecided::NotTouching)?;
    let last = path
        .iter()
        .zip(touching)
        .filter(|(_, t)| **t)
        .map(|(p, _)| p.clone())
        .next_back()
        .ok_or(Undecided::NotTouching)?;
    rigid_from(&first, &last)
}

/// **判据:推演出来的运动,等不等于第③格说的那个旋量。**
///
/// 这就是"接触集没有自欺"的那条线 —— 而在此之前没有任何东西验过它。
pub fn matches(cs: &ContactSet, got: Moved, tol_m: f64, tol_rad: f64) -> bool {
    let want: Twist = cs.motion;
    // 转:比角度。
    // 🔴 **第③格要求转、而几何上转定不下来 ⇒ 这条接触集在自欺,当场判假。**
    // 一个接触点说"物体要绕轴转"就是这种情况 —— 而在此之前没有任何东西会不一致。
    match got.rot {
        Some(q) => {
            let ang_got = 2.0 * q[0].abs().clamp(0.0, 1.0).acos();
            if (ang_got - want.angle()).abs() > tol_rad {
                return false;
            }
        }
        None => {
            if want.angle() > tol_rad {
                return false;
            }
        }
    }
    // 平移:比"把接触点质心搬了多远"。**绕支点转也会搬动质心**,所以拿旋量自己算一遍再比。
    //
    // 🔴 只数【手】那些点。`drive` 看到的就是手的航点(世界那一侧的接触手够不到、不进航点),
    // 两边算质心时口径必须一致 —— 不一致时**推演明明是对的,判据却说对不上**。
    // 实测代价:三指/五指撬与翻,整整四格被冤枉成"③对不上"(2026-08-16 验收台)。
    let 手: Vec<&Point> = cs.points.iter().filter(|p| p.by == crate::Who::Hand).collect();
    if 手.is_empty() {
        return false;
    }
    let c = {
        let k = 手.len() as f64;
        [
            手.iter().map(|p| p.at[0]).sum::<f64>() / k,
            手.iter().map(|p| p.at[1]).sum::<f64>() / k,
            手.iter().map(|p| p.at[2]).sum::<f64>() / k,
        ]
    };
    let after = want.apply(c);
    let want_t = [after[0] - c[0], after[1] - c[1], after[2] - c[2]];
    norm([
        got.trans[0] - want_t[0],
        got.trans[1] - want_t[1],
        got.trans[2] - want_t[2],
    ]) <= tol_m
}

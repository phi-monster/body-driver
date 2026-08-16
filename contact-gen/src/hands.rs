//! **②a 的另外两种手:吸盘(1 点)· 环抓(n 点)。零学习,纯几何,全部从点云里算。**
//!
//! # 为什么不是"把两指的结果复制 n 份"
//!
//! 两指那条路是**切层 + 量跨度**:它问"沿这个方向,隔多宽有两个相对的面"。
//! 吸盘问的完全是另一件事(*"有没有一片够大、够平的面"*),
//! 环抓问的又是另一件(*"绕一圈,有没有 n 个方向都摸得到料"*)。
//! **同一张接触集表,三条不同的几何路径填** —— 这正是那张表该有的样子:
//! 表不认识机体,机体各自算各自的。
//!
//! # 🔴 这里一个"想当然"都不许有
//!
//! 吸盘不是"取最高点";环抓不是"在圆上均分 n 个点"。
//! 前者要**真的平**(残差量出来),后者要**真的摸得到料**(每个方向都得在点云里找到边界点),
//! 而且 n 个方向必须**正张成**平面 —— 否则那不是握住,是从一侧推。

use crate::{Handoff, P3};

/// 吸盘/环抓交不出去时,**点名**。
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum NoHand {
    /// 点太少,什么都算不出来。
    TooFewPoints,
    /// **找不到一片够大够平的面** —— 吸盘吸不住。带上实测的最大平坦半径与要求的半径。
    NoFlatPatch { found_r: f64, need_r: f64 },
    /// **某个方向上摸不到料** —— 那一圈不是闭合的,握不住。带上是第几个方向。
    NothingInDirection(usize),
    /// **n 个方向张不成平面** —— 全挤在一侧,那是推不是握。
    NotSurrounding,
    /// 交接到接触集时被拒 —— 转发原因。
    Handoff(Handoff),
}

/// **吸盘:从点云里找一片够大、够平的面,给出【一个】接触点。**
///
/// - `cup_r_m`:吸盘半径,**身体常数,量出来传进来**。
/// - `flat_tol_m`:多平才算平(点到拟合平面的距离上界),**它是采样噪声的函数,不是身体常数**。
///
/// 做法:对每个点,取它半径 `cup_r_m` 内的邻居,用协方差最小特征向量当法向,
/// 量所有邻居到该平面的最大偏差;偏差 ≤ `flat_tol_m` 且邻居铺满整个半径的,就是一片可吸的面。
/// **取铺得最满的那一片**(不是最高的那一片 —— 最高不等于最平)。
pub fn suction(
    cloud: &[P3],
    cup_r_m: f64,
    flat_tol_m: f64,
    mu: f64,
    motion: contact_set::Twist,
    tol_m: f64,
) -> Result<contact_set::ContactSet, NoHand> {
    if cloud.len() < 8 {
        return Err(NoHand::TooFewPoints);
    }
    if !(mu.is_finite() && mu > 0.0) {
        return Err(NoHand::Handoff(Handoff::MuUnknown));
    }
    let 采样间距 = 采样间距(cloud);
    let mut best: Option<(f64, [f64; 3], [f64; 3])> = None; // (铺满程度, 点, 法向)
    let mut 见过最好的半径 = 0.0f64;
    for c in cloud {
        let mid = [c.x, c.y, c.z];
        let near: Vec<[f64; 3]> = cloud
            .iter()
            .map(|p| [p.x, p.y, p.z])
            .filter(|p| {
                let d = [p[0] - mid[0], p[1] - mid[1], p[2] - mid[2]];
                contact_set::norm(d) <= cup_r_m
            })
            .collect();
        if near.len() < 6 {
            continue;
        }
        let n0 = match 最小特征向量(&near) {
            Some(v) => v,
            None => continue,
        };
        // 🔴 **把"背面"和"弯曲"分开 —— 这两件事长得一样,处理方式完全相反。**
        //
        // 一张 4 mm 厚的板子,两个面都落在吸盘半径以内 ⇒ 拟出来的"平面"横跨两面,
        // 法向是错的 ⇒ **一张又大又平的板子被判成吸不住**(实测:薄板整行 `found_r: 0.0`)。
        // ⇒ 背面那一片要**筛掉**。
        //
        // ⚠️ 但一个**球**的邻居也一样偏离平面,而那是**真的不平**,必须**判死**。
        // 只按"离平面远就筛掉"处理,等于**只挑最平的那一小块去拟**,
        // 于是球也吸得住了 —— 实测那一版球从 1/13 跳到 10/13,**是假的**。
        //
        // 判别式:偏离量**跟不跟半径走**。
        // - 背面:偏离量 ≈ 板厚,**与半径无关**(远近都是那个数)⇒ 是另一片面,筛掉。
        // - 弯曲:偏离量 ≈ ρ²/2R,**随半径长大**(近处贴合、远处翘起)⇒ 是真的不平,判死。
        let 偏 = |p: &[f64; 3]| {
            let d = [p[0] - mid[0], p[1] - mid[1], p[2] - mid[2]];
            let along = contact_set::dot(d, n0);
            let flat = [d[0] - n0[0] * along, d[1] - n0[1] * along, d[2] - n0[2] * along];
            (along, contact_set::norm(flat))
        };
        let 远: Vec<(f64, f64)> =
            near.iter().map(&偏).filter(|(a, _)| a.abs() > flat_tol_m).collect();
        if !远.is_empty() {
            let (mut lo, mut hi) = (f64::MAX, f64::MIN);
            let (mut ρ小, mut ρ大) = (f64::MAX, f64::MIN);
            for (a, r) in &远 {
                lo = lo.min(a.abs());
                hi = hi.max(a.abs());
                ρ小 = ρ小.min(*r);
                ρ大 = ρ大.max(*r);
            }
            // 偏离量本身散得开(不是一个常数)⇒ 它跟着半径走 ⇒ 是弯的,不是背面。
            let 像背面 = (hi - lo) <= flat_tol_m && ρ大 - ρ小 > cup_r_m / 2.0;
            if !像背面 {
                continue; // 真的不平 —— 这个种子点不算数
            }
        }
        let near: Vec<[f64; 3]> =
            near.into_iter().filter(|p| 偏(p).0.abs() <= flat_tol_m).collect();
        if near.len() < 6 {
            continue;
        }
        let n = match 最小特征向量(&near) {
            Some(v) => v,
            None => continue,
        };
        // 平面内的两条基,用来分扇区
        let (u, w) = 平面基(n);
        // 到平面的最大偏差 + **每个扇区各自铺开了多远**
        //
        // 🔴 为什么要分扇区、而不是只取"最远的邻居有多远"
        // 只取最远的,**吸盘落在面的边沿上也能过** —— 一侧铺满、另一侧悬空,那是吸不住的。
        // 铺满程度必须取**各扇区里最小的那一个**:每个方向都得有料。
        let mut off = 0.0f64;
        let mut 扇区 = [0.0f64; 8];
        for p in &near {
            let d = [p[0] - mid[0], p[1] - mid[1], p[2] - mid[2]];
            let along = contact_set::dot(d, n);
            off = off.max(along.abs());
            let flat = [d[0] - n[0] * along, d[1] - n[1] * along, d[2] - n[2] * along];
            let r = contact_set::norm(flat);
            if r < 1e-12 {
                continue;
            }
            let a = contact_set::dot(flat, w).atan2(contact_set::dot(flat, u));
            let k = (((a / core::f64::consts::TAU + 1.0) * 8.0) as usize) % 8;
            扇区[k] = 扇区[k].max(r);
        }
        if off > flat_tol_m {
            continue;
        }
        let span = 扇区.iter().fold(f64::MAX, |a, b| a.min(*b));
        见过最好的半径 = 见过最好的半径.max(span);
        // 🔴 采样密度是分辨率的下界:**点与点之间隔多远,就分辨不出比那更细的边界**。
        // 所以门槛是 `span + 采样间距 ≥ 吸盘半径`,不是 `span ≥ 吸盘半径` ——
        // 后者要求恰好有一个采样点落在吸盘边缘上,那是**采样伪影,不是几何**。
        if span + 采样间距 + 1e-12 >= cup_r_m && best.map_or(true, |(b, _, _)| span > b) {
            best = Some((span, mid, n));
        }
    }
    let (_, at, mut n) = best.ok_or(NoHand::NoFlatPatch { found_r: 见过最好的半径, need_r: cup_r_m })?;
    // 法向要**指向物体外侧**:取远离点云重心的那一支。
    let g = 重心(cloud);
    if contact_set::dot(n, [at[0] - g[0], at[1] - g[1], at[2] - g[2]]) < 0.0 {
        n = [-n[0], -n[1], -n[2]];
    }
    let set = contact_set::ContactSet {
        points: vec![contact_set::Point {
            by: contact_set::Who::Hand,
            at,
            normal: n,
            // 🔴 吸盘的锥 = **密封面与物体之间的摩擦锥**,半张角 `atan(μ)`。
            //
            // ⚠️ 我先写死成 **0**(只准沿法向吸),理由写的是"宁可窄不许宽"。
            // **那一版把吸盘废掉了一半** —— 实测:验收台上吸盘的 推/放/擦/倒/撬/翻/舀
            // **十四格全判死**,而真空吸盘搬箱子天天在做侧向移动。
            // 0 不是保守,是**错的模型**:抗剪力真实存在,而且量得出来。
            // ⇒ 与 `ring` 同一条规矩:**μ 由调用方给,没量过就别调这个函数**。
            cone: contact_set::Cone { axis: [-n[0], -n[1], -n[2]], half_angle: mu.atan() },
            // 🔴 吸盘的全部意义:它【拉】得动。少了这一项,吸住了也抬不起任何东西。
            pull: true,
            // 吸盘吸住了是**拧得动**的 —— 密封圈是一片面,不是一个点。
            torsion: true,
            // 🔴 而且**掰得动**:密封面有半径,扛得住剥离力矩。少了这一项,
            // 吸盘只转得动、翻不动 —— 撬/翻/倒/舀 九格全判死(验收台实测)。
            peel: true,
            tol_m,
        }],
        motion,
        approach: Some([-n[0], -n[1], -n[2]]),
    };
    Ok(set)
}

/// **环抓:在一个高度上绕物体一圈,给出 n 个接触点(三指 = 3,五指 = 5)。**
///
/// - `at_z` / `band_m`:在哪个高度、取多厚的一层。
/// - `n`:几根手指。
/// - `mu`:摩擦系数,**身体×世界的耦合,量出来传进来**(没量过就别调这个函数)。
///
/// 做法:取这一层的点 → 质心 → n 个均分方向 → **每个方向上取最外那个点**(真的摸得到料)
/// → 法向 = 由质心指向该点 → 锥 = 绕内法向的摩擦锥。
/// **n 个内法向必须正张成平面**,否则那不是握住、是从一侧推,当场拒绝。
pub fn ring(
    cloud: &[P3],
    at_z: f64,
    band_m: f64,
    n: usize,
    mu: f64,
    motion: contact_set::Twist,
    tol_m: f64,
) -> Result<contact_set::ContactSet, NoHand> {
    if n < 2 {
        return Err(NoHand::NotSurrounding);
    }
    if !(mu.is_finite() && mu > 0.0) {
        return Err(NoHand::Handoff(Handoff::MuUnknown));
    }
    let band: Vec<&P3> =
        cloud.iter().filter(|p| (p.z - at_z).abs() <= band_m / 2.0).collect();
    if band.len() < n * 2 {
        return Err(NoHand::TooFewPoints);
    }
    let (cx, cy) = (
        band.iter().map(|p| p.x).sum::<f64>() / band.len() as f64,
        band.iter().map(|p| p.y).sum::<f64>() / band.len() as f64,
    );
    let mut pts = Vec::with_capacity(n);
    let mut dirs = Vec::with_capacity(n);
    for k in 0..n {
        let a = core::f64::consts::TAU * (k as f64) / (n as f64);
        let (dx, dy) = (a.cos(), a.sin());
        // 这个方向上、**贴着这条射线**的那些点里最外的一个
        let mut far: Option<(f64, &P3)> = None;
        for p in &band {
            let (ux, uy) = (p.x - cx, p.y - cy);
            let along = ux * dx + uy * dy;
            if along <= 0.0 {
                continue;
            }
            let off = (ux * -dy + uy * dx).abs();
            // 只认落在这个扇区里的点:横向偏移不超过沿径向距离的 tan(π/n)
            if off > along * (core::f64::consts::PI / n as f64).tan() {
                continue;
            }
            if far.map_or(true, |(b, _)| along > b) {
                far = Some((along, p));
            }
        }
        let (_, p) = far.ok_or(NoHand::NothingInDirection(k))?;
        pts.push([p.x, p.y, p.z]);
        dirs.push([dx, dy, 0.0]);
    }
    // 🔴 正张成检查:内法向的合必须近乎为零(围住了),而不是全挤在一侧。
    let s = dirs.iter().fold([0.0f64; 3], |a, d| [a[0] + d[0], a[1] + d[1], a[2] + d[2]]);
    if contact_set::norm(s) > 0.5 {
        return Err(NoHand::NotSurrounding);
    }
    let half = mu.atan();
    let points = pts
        .iter()
        .zip(&dirs)
        .map(|(at, d)| contact_set::Point {
            by: contact_set::Who::Hand,
            at: *at,
            normal: *d,                                             // 由质心指向外 = 物体外侧
            cone: contact_set::Cone { axis: [-d[0], -d[1], -d[2]], half_angle: half }, // 朝里夹
            pull: false,
            torsion: false,
            peel: false, // 指尖当点接触;有指腹的把它改成 true(那是量出来的身体属性)
            tol_m,
        })
        .collect();
    Ok(contact_set::ContactSet { points, motion, approach: Some([0.0, 0.0, -1.0]) })
}

/// 点云自己的**采样间距**:每个点到最近邻的距离取中位。
/// 🔴 它不是身体常数,也不是参数 —— 它是这团点云自己的分辨率,**量得出来就不许拍**。
fn 采样间距(cloud: &[P3]) -> f64 {
    let mut d: Vec<f64> = cloud
        .iter()
        .map(|a| {
            cloud
                .iter()
                .filter(|b| !core::ptr::eq(*b, a))
                .map(|b| contact_set::norm([b.x - a.x, b.y - a.y, b.z - a.z]))
                .fold(f64::MAX, f64::min)
        })
        .filter(|v| v.is_finite())
        .collect();
    if d.is_empty() {
        return 0.0;
    }
    d.sort_by(|a, b| a.partial_cmp(b).unwrap());
    d[d.len() / 2]
}

/// 给一条法向配两条与它垂直的基。
fn 平面基(n: [f64; 3]) -> ([f64; 3], [f64; 3]) {
    let seed = if n[0].abs() < 0.9 { [1.0, 0.0, 0.0] } else { [0.0, 1.0, 0.0] };
    let d = contact_set::dot(seed, n);
    let u = contact_set::unit([seed[0] - n[0] * d, seed[1] - n[1] * d, seed[2] - n[2] * d])
        .unwrap_or([1.0, 0.0, 0.0]);
    (u, contact_set::cross(n, u))
}

fn 重心(cloud: &[P3]) -> [f64; 3] {
    let k = cloud.len() as f64;
    [
        cloud.iter().map(|p| p.x).sum::<f64>() / k,
        cloud.iter().map(|p| p.y).sum::<f64>() / k,
        cloud.iter().map(|p| p.z).sum::<f64>() / k,
    ]
}

/// 一组点的协方差最小特征向量 = 局部平面的法向。3×3 循环 Jacobi,零依赖。
fn 最小特征向量(pts: &[[f64; 3]]) -> Option<[f64; 3]> {
    let k = pts.len() as f64;
    let m = [
        pts.iter().map(|p| p[0]).sum::<f64>() / k,
        pts.iter().map(|p| p[1]).sum::<f64>() / k,
        pts.iter().map(|p| p[2]).sum::<f64>() / k,
    ];
    let mut c = [[0.0f64; 3]; 3];
    for p in pts {
        let d = [p[0] - m[0], p[1] - m[1], p[2] - m[2]];
        for i in 0..3 {
            for j in 0..3 {
                c[i][j] += d[i] * d[j];
            }
        }
    }
    let mut v = [[0.0f64; 3]; 3];
    for i in 0..3 {
        v[i][i] = 1.0;
    }
    for _ in 0..40 {
        let mut off = 0.0;
        for i in 0..3 {
            for j in (i + 1)..3 {
                off += c[i][j] * c[i][j];
            }
        }
        if off < 1e-28 {
            break;
        }
        for p in 0..3 {
            for q in (p + 1)..3 {
                if c[p][q].abs() < 1e-20 {
                    continue;
                }
                let th = (c[q][q] - c[p][p]) / (2.0 * c[p][q]);
                let t = th.signum() / (th.abs() + (th * th + 1.0).sqrt());
                let cc = 1.0 / (t * t + 1.0).sqrt();
                let ss = t * cc;
                for k in 0..3 {
                    let (kp, kq) = (c[k][p], c[k][q]);
                    c[k][p] = cc * kp - ss * kq;
                    c[k][q] = ss * kp + cc * kq;
                }
                for k in 0..3 {
                    let (pk, qk) = (c[p][k], c[q][k]);
                    c[p][k] = cc * pk - ss * qk;
                    c[q][k] = ss * pk + cc * qk;
                }
                for k in 0..3 {
                    let (kp, kq) = (v[k][p], v[k][q]);
                    v[k][p] = cc * kp - ss * kq;
                    v[k][q] = ss * kp + cc * kq;
                }
            }
        }
    }
    let mut lo = 0usize;
    for i in 1..3 {
        if c[i][i] < c[lo][lo] {
            lo = i;
        }
    }
    contact_set::unit([v[0][lo], v[1][lo], v[2][lo]])
}

#[cfg(test)]
mod 另外两种手 {
    use super::*;

    /// 一根竖着的圆柱(半径 3 cm、高 8 cm),顶面是平的。
    fn 圆柱() -> Vec<P3> {
        let mut v = Vec::new();
        for i in 0..36 {
            let a = core::f64::consts::TAU * i as f64 / 36.0;
            for k in 0..17 {
                v.push(P3 { x: 0.03 * a.cos(), y: 0.03 * a.sin(), z: 0.90 + 0.08 * k as f64 / 16.0 });
            }
        }
        // 顶面
        for i in 0..9 {
            for j in 0..9 {
                let (x, y) = (-0.03 + 0.06 * i as f64 / 8.0, -0.03 + 0.06 * j as f64 / 8.0);
                if x * x + y * y <= 0.03 * 0.03 + 1e-12 {
                    v.push(P3 { x, y, z: 0.98 });
                }
            }
        }
        v
    }

    #[test]
    fn 吸盘_从点云里真的找到那片平顶面() {
        let 静 = contact_set::Twist::still([0.0, 0.0, 0.94]);
        let set = suction(&圆柱(), 0.012, 0.001, 0.5, 静, 0.002).expect("顶面该吸得住");
        assert_eq!(set.points.len(), 1, "吸盘就是一个点");
        let p = set.points[0];
        assert!((p.at[2] - 0.98).abs() < 1e-9, "该落在顶面上,实得 z={}", p.at[2]);
        assert!(p.normal[2] > 0.99, "顶面法向朝上,实得 {:?}", p.normal);
        // 锥 = 密封面的摩擦锥。**不是 0** —— 写死 0 会把吸盘的侧向能力整个抹掉,
        // 而真空吸盘搬箱子天天在做侧移(实测:那一版验收台上吸盘十四格全判死)。
        assert!((p.cone.half_angle - 0.5f64.atan()).abs() < 1e-12, "锥 = atan(μ)");
        assert!(p.torsion, "吸住了是拧得动的");
        assert_eq!(set.check(false), Ok(()), "吸盘那一套四格必须自检就过");
    }

    #[test]
    fn 吸盘_吸盘比那片平面还大就必须拒绝_并报差多少() {
        // 顶面半径 3 cm;要一个 6 cm 半径的吸盘 ⇒ 铺不满
        match suction(&圆柱(), 0.06, 0.001, 0.5, contact_set::Twist::still([0.0; 3]), 0.002) {
            Err(NoHand::NoFlatPatch { found_r, need_r }) => {
                assert!((need_r - 0.06).abs() < 1e-12);
                assert!(found_r < 0.06, "报出来的最大平坦半径要小于要求的,实得 {found_r:.4}");
            }
            other => panic!("铺不满就必须拒绝,实得 {other:?}"),
        }
    }

    #[test]
    fn 三指与五指_绕一圈真的摸得到料() {
        for n in [3usize, 5] {
            let 静 = contact_set::Twist::still([0.0, 0.0, 0.94]);
            let set = ring(&圆柱(), 0.94, 0.02, n, 0.5, 静, 0.002)
                .unwrap_or_else(|e| panic!("{n} 指该围得住,实得 {e:?}"));
            assert_eq!(set.points.len(), n);
            for p in &set.points {
                // 每个点都得真在表面上(半径 3 cm),不是圆上臆造的
                let r = (p.at[0] * p.at[0] + p.at[1] * p.at[1]).sqrt();
                assert!((r - 0.03).abs() < 0.004, "{n} 指:接触点要落在真表面上,实得 r={r:.4}");
                assert!((p.cone.half_angle - 0.5f64.atan()).abs() < 1e-12, "锥 = 摩擦锥");
            }
            assert_eq!(set.check(false), Ok(()), "{n} 指必须填得满同一张表");
        }
    }

    /// 🔴 反例:一个**开口的 C 形壳**(半边根本没有料)。
    ///
    /// ⚠️ 我第一版拿"半个圆柱"当反例,**结果它通过了 —— 而且通过得对**:
    /// 切一半之后质心跟着挪,三个方向上仍然各自摸得到真表面(切面本身也是面)。
    /// **反例写错了,不是代码错了。** 真正握不住的是"某个方向上一个点都没有"。
    #[test]
    fn 环抓_某个方向上一个点都没有就必须点名() {
        // 只留角度在 [90°, 270°] 的那半圈壳,不带顶面 ⇒ 开口朝 +x
        let c: Vec<P3> = (0..36)
            .map(|i| core::f64::consts::TAU * i as f64 / 36.0)
            .filter(|a| *a >= core::f64::consts::FRAC_PI_2 && *a <= 3.0 * core::f64::consts::FRAC_PI_2)
            .flat_map(|a| {
                (0..17).map(move |k| P3 {
                    x: 0.03 * a.cos(),
                    y: 0.03 * a.sin(),
                    z: 0.90 + 0.08 * k as f64 / 16.0,
                })
            })
            .collect();
        let r = ring(&c, 0.94, 0.02, 5, 0.5, contact_set::Twist::still([0.0; 3]), 0.002);
        match r {
            Err(NoHand::NothingInDirection(k)) => assert_eq!(k, 0, "开口正对 +x = 第 0 个方向"),
            other => panic!("开口那边摸不到料,必须点名是哪个方向;实得 {other:?}"),
        }
    }

    #[test]
    fn 环抓_μ没量过就不许调() {
        let r = ring(&圆柱(), 0.94, 0.02, 3, 0.0, contact_set::Twist::still([0.0; 3]), 0.002);
        assert_eq!(r, Err(NoHand::Handoff(Handoff::MuUnknown)));
    }
}

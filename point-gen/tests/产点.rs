//! **产点器验收:眼睛 → 一团三维点 → 喂给 ②a → 抓的地方跟拿真点算的一样。**
//!
//! 这里的"真值"是**合成**的:先造一个已知形状,再把它投进虚拟相机,
//! 然后让产点器把它**还原**出来,跟原件逐点比。
//! 🔴 合成不等于放水 —— 它验的是**这套几何自己自洽**,而这一条不自洽的话,
//! 换成真相机只会更差。真相机那一关另有它自己的判据(镜头畸变、噪声),不在这里冒充。

use point_gen::*;

/// 一块长方体表面点(米)—— **没有转动对称**,拿它当"候选该不该一样"的判据。
fn 方块() -> Vec<P3> {
    let mut v = Vec::new();
    let (a, b, h) = (0.05f64, 0.035f64, 0.06f64);
    let n = 13;
    for i in 0..n {
        for j in 0..n {
            let (x, y) = (-a / 2.0 + a * i as f64 / (n - 1) as f64, -b / 2.0 + b * j as f64 / (n - 1) as f64);
            v.push(P3 { x, y, z: 0.90 });
            v.push(P3 { x, y, z: 0.90 + h });
        }
    }
    for i in 0..n {
        for k in 0..n {
            let (t, z) = (i as f64 / (n - 1) as f64, 0.90 + h * k as f64 / (n - 1) as f64);
            v.push(P3 { x: -a / 2.0 + a * t, y: -b / 2.0, z });
            v.push(P3 { x: -a / 2.0 + a * t, y: b / 2.0, z });
            v.push(P3 { x: -a / 2.0, y: -b / 2.0 + b * t, z });
            v.push(P3 { x: a / 2.0, y: -b / 2.0 + b * t, z });
        }
    }
    v
}

/// 一根竖着的圆柱表面点(米)。
fn 圆柱() -> Vec<P3> {
    let mut v = Vec::new();
    for i in 0..40 {
        let a = core::f64::consts::TAU * i as f64 / 40.0;
        for k in 0..15 {
            v.push(P3 { x: 0.03 * a.cos(), y: 0.03 * a.sin(), z: 0.90 + 0.08 * k as f64 / 14.0 });
        }
    }
    v
}

/// 一只朝下俯视的眼睛:架在 (0,0,1.40),往 −z 看。
/// 相机系 +z 朝前 ⇒ 把 +z 转到世界的 −z(绕 x 轴转 180°)。
fn 俯视眼(at: [f64; 3]) -> Eye {
    Eye { fx: 600.0, fy: 600.0, cx: 320.0, cy: 240.0, at, q: [0.0, 1.0, 0.0, 0.0] }
}

#[test]
fn 深度那条路_投出去再反投回来必须是原来那个点() {
    let eye = 俯视眼([0.0, 0.0, 1.40]);
    let mut n = 0;
    for p in 圆柱() {
        let px = match eye.project(p) {
            Some(v) => v,
            None => continue,
        };
        let depth = eye.into_cam(p)[2];
        let back = eye.back_project(px, depth).expect("深度是正的");
        let d = ((back.x - p.x).powi(2) + (back.y - p.y).powi(2) + (back.z - p.z).powi(2)).sqrt();
        assert!(d < 1e-12, "反投影要回到原点,差 {d:.2e}");
        n += 1;
    }
    assert!(n > 300, "该有几百个点参与,实得 {n}");
}

/// 🔴🔴 **架构底线那一条:两只普通相机,不用深度,也要还原出同一团点。**
#[test]
fn 两只相机那条路_没有深度也还原得出来() {
    let 左 = 俯视眼([-0.06, 0.0, 1.40]);
    let 右 = 俯视眼([0.06, 0.0, 1.40]);
    let 真 = 圆柱();
    let mut pairs = Vec::new();
    let mut 真序 = Vec::new();
    for p in &真 {
        if let (Some(a), Some(b)) = (左.project(*p), 右.project(*p)) {
            pairs.push((a, b));
            真序.push(*p);
        }
    }
    let (点, 丢) = from_pair(&左, &右, &pairs, 1e-6);
    assert_eq!(丢, 0, "配对是对的就不该丢");
    assert_eq!(点.len(), 真序.len());
    let mut worst = 0.0f64;
    for (g, p) in 点.iter().zip(&真序) {
        worst = worst.max(((g.x - p.x).powi(2) + (g.y - p.y).powi(2) + (g.z - p.z).powi(2)).sqrt());
    }
    assert!(worst < 1e-9, "两只相机三角化的最大误差 {worst:.2e} m");
}

/// 反例:左右眼**配错点**,必须被挡下来并**报出丢了几个**,不许硬凑进点云。
#[test]
fn 反例_左右眼配错点必须丢掉而且报数() {
    let 左 = 俯视眼([-0.06, 0.0, 1.40]);
    let 右 = 俯视眼([0.06, 0.0, 1.40]);
    let 真 = 圆柱();
    let mut pairs = Vec::new();
    for (i, p) in 真.iter().enumerate() {
        if let (Some(a), Some(b)) = (左.project(*p), 右.project(*p)) {
            // 每隔 5 个,把右眼那一半故意换成别的点 —— 这就是配错
            let b = if i % 5 == 0 {
                match 右.project(真[(i + 37) % 真.len()]) {
                    Some(x) => x,
                    None => b,
                }
            } else {
                b
            };
            pairs.push((a, b));
        }
    }
    let (点, 丢) = from_pair(&左, &右, &pairs, 0.002);
    assert!(丢 > 50, "配错的那些要被挡下来,实得只丢了 {丢} 个");
    assert!(点.len() + 丢 == pairs.len(), "丢了几个必须报得出来");
}

/// 🔴 **身体自己量出焦距** —— 看着自己的手挪几个地方就够,没有标定板、没有配置文件。
#[test]
fn 自己量出针孔_而且量得准() {
    let 真眼 = 俯视眼([0.0, 0.0, 1.40]);
    // 手挪到十几个位置,**深度要拉开**(这是关键,见下一条反例)
    let mut seen = Vec::new();
    for i in 0..4 {
        for j in 0..4 {
            let p = P3 {
                x: -0.15 + 0.10 * i as f64,
                y: -0.15 + 0.10 * j as f64,
                z: 0.85 + 0.06 * (i + j) as f64,
            };
            seen.push((p, 真眼.project(p).unwrap()));
        }
    }
    let 量出来 = fit(真眼.at, 真眼.q, &seen).expect("这组样本该解得出来");
    assert!((量出来.fx - 600.0).abs() < 1e-6, "焦距要量准,实得 {}", 量出来.fx);
    assert!((量出来.fy - 600.0).abs() < 1e-6);
    assert!((量出来.cx - 320.0).abs() < 1e-6, "主点要量准,实得 {}", 量出来.cx);
    assert!((量出来.cy - 240.0).abs() < 1e-6);
}

/// 🔴🔴 **反例,而且是本仓真栽过的那一个**:样本全挤在一个深度上 ⇒ 必须拒绝。
///
/// 当年那张"全局换算表"就是这么翻号的:它在**一个高度**上量,换个高度就外推到了模型之外
/// (同一台相机三个锚点,`dv` 整行三个数各自翻号,同布局重跑只差 2.6% —— 不是噪声)。
/// 焦距和距离在针孔里乘在一起,**只在一个深度上看,这两个分不开**。
#[test]
fn 反例_只在一个深度上量_必须拒绝而不是硬解() {
    let 真眼 = 俯视眼([0.0, 0.0, 1.40]);
    let mut seen = Vec::new();
    for i in 0..4 {
        for j in 0..4 {
            let p = P3 { x: -0.15 + 0.10 * i as f64, y: -0.15 + 0.10 * j as f64, z: 0.95 };
            seen.push((p, 真眼.project(p).unwrap()));
        }
    }
    match fit(真眼.at, 真眼.q, &seen) {
        Err(WhyNot::AllAtOneDepth(span)) => assert!(span < 1e-9, "跨度实得 {span}"),
        other => panic!("一个深度上分不开焦距和距离,必须拒绝;实得 {other:?}"),
    }
}

/// 🔴🔴 **全链:眼睛产的点 → ②a 找下手点 → 跟拿真点算出来的一样。**
#[test]
fn 眼睛产的点_喂给抓取生成器_跟真点算的一样() {
    use contact_gen::{candidates, Body, Grid, JawSpan};
    let 左 = 俯视眼([-0.06, 0.0, 1.40]);
    let 右 = 俯视眼([0.06, 0.0, 1.40]);
    // 🔴 用**方块**,不用圆柱。圆柱是转动对称的 ⇒ 十几个偏角上的候选几乎等价,
    // 一点点扰动就换一个胜出者 —— 那是**物体的简并**,不是算法在抖。
    // 拿对称物体做"结果该不该一模一样"的判据,读出来的差异是假的。
    // 🔴 加一点**真相机会有的**抖动(0.1 mm 量级)。
    // 理由:合成的规则物体,面正好贴着切层边界、宽正好压在阈值上 ⇒ 3 飞米的差就能翻一堆候选。
    // 那是**合成件的简并**,不是这条链的性质。加了噪声就没有东西"正好落在线上"了。
    let 真: Vec<P3> = 方块()
        .into_iter()
        .enumerate()
        .map(|(i, p)| {
            let w = |k: usize| ((i * 2654435761 + k * 40503) % 1000) as f64 / 1000.0 - 0.5;
            P3 { x: p.x + 1e-4 * w(1), y: p.y + 1e-4 * w(2), z: p.z + 1e-4 * w(3) }
        })
        .collect();
    let pairs: Vec<(Px, Px)> = 真
        .iter()
        .filter_map(|p| match (左.project(*p), 右.project(*p)) {
            (Some(a), Some(b)) => Some((a, b)),
            _ => None,
        })
        .collect();
    let (看出来的, 丢) = from_pair(&左, &右, &pairs, 1e-6);
    assert_eq!(丢, 0);
    println!("点数:真 {} vs 看出来 {}", 真.len(), 看出来的.len());
    let 最大点差 = 真.iter().zip(&看出来的).map(|(p, g)| ((p.x - g.x).powi(2) + (p.y - g.y).powi(2) + (p.z - g.z).powi(2)).sqrt()).fold(0.0f64, f64::max);
    println!("逐点最大差 = {最大点差:.3e} m");

    let body = Body { jaw: JawSpan::Measured(0.09), reach_lo: 0.02, reach_hi: 1.5, base_x: 0.0, base_y: -0.4 };
    let grid = Grid { bands: 4, jaw_h_m: 0.02, dirs: 12, min_pts: 8, min_above_m: 0.001, finger_w_m: 0.02, gap_m: 0.012 };
    let 换 = |v: &[P3]| -> Vec<contact_gen::P3> {
        v.iter().map(|p| contact_gen::P3 { x: p.x, y: p.y, z: p.z }).collect()
    };
    let a = candidates(&换(&真), &body, 0.90, grid).expect("真点该有候选");
    let b = candidates(&换(&看出来的), &body, 0.90, grid).expect("看出来的点也该有候选");
    assert_eq!(a.len(), b.len(), "候选条数要一样");
    // 🔴 按【集合】比,不按顺序比。
    // ⚠️ 我第一版是逐位比的,于是把"排序在并列附近换了个位"读成了"结果不一样" ——
    // 而真相是两团点云只差 2.8e-15 m。**比错了口径,读出来的是一个假的差异。**
    let 近 = |x: &contact_gen::Contact, y: &contact_gen::Contact| {
        ((x.point.x - y.point.x).powi(2)
            + (x.point.y - y.point.y).powi(2)
            + (x.point.z - y.point.z).powi(2))
        .sqrt()
            .max((x.width_m - y.width_m).abs())
    };
    let mut 配上 = 0usize;
    let mut 最差 = 0.0f64;
    for x in &a {
        let d = b.iter().map(|y| 近(x, y)).fold(f64::MAX, f64::min);
        if d < 1e-9 {
            配上 += 1;
            最差 = 最差.max(d);
        }
    }
    println!("集合比对:{} / {} 条在 1e-9 内配得上,最差 {:.2e} m", 配上, a.len(), 最差);
    assert_eq!(配上, a.len(), "眼睛产的点必须跟真点给出同一批下手点");
}

/// 🔴 **把点云抖动 1e-15 米,看候选表变不变** —— 查完的结论记在这儿。
///
/// **完美规则的合成件上,它确实会变**(方块:140 条 vs 156 条)。
/// 原因不是算法在抖:合成的方块**面正好压在切层边界上、宽正好等于阈值**,
/// 于是 3 飞米就足以把一堆点翻到边界另一侧。**那是合成件的简并。**
///
/// 🟢 **加上真相机会有的 0.1 mm 抖动之后,同一条链 156/156 完全一致** ——
/// 没有东西再"正好落在线上"。⇒ 这条刀口**不是这条链的性质**,
/// 而是"拿完美几何体当判据"这件事本身的坑。**记下来,别下次又当成 bug 去追。**
#[test]
fn 探针_抖动一皮米看结果动不动() {
    use contact_gen::{candidates, Body, Grid, JawSpan};
    let body = Body { jaw: JawSpan::Measured(0.09), reach_lo: 0.02, reach_hi: 1.5, base_x: 0.0, base_y: -0.4 };
    let grid = Grid { bands: 4, jaw_h_m: 0.02, dirs: 12, min_pts: 8, min_above_m: 0.001, finger_w_m: 0.02, gap_m: 0.012 };
    let 真: Vec<contact_gen::P3> =
        圆柱().iter().map(|p| contact_gen::P3 { x: p.x, y: p.y, z: p.z }).collect();
    let 抖: Vec<contact_gen::P3> = 真
        .iter()
        .enumerate()
        .map(|(i, p)| contact_gen::P3 {
            x: p.x + if i % 2 == 0 { 1e-15 } else { -1e-15 },
            y: p.y,
            z: p.z,
        })
        .collect();
    let a = candidates(&真, &body, 0.90, grid).unwrap();
    let b = candidates(&抖, &body, 0.90, grid).unwrap();
    // 🔴 按集合、按数值比 —— 上一版拿格式化字符串比,而 ±1e-15 会印成
    // "-0.000000000" 与 "0.000000000" 两个不同的串 ⇒ **比出来的差异是假的**。
    let 近 = |x: &contact_gen::Contact, y: &contact_gen::Contact| {
        ((x.point.x - y.point.x).powi(2) + (x.point.y - y.point.y).powi(2)).sqrt()
            .max((x.width_m - y.width_m).abs())
    };
    let 同 = a.iter().filter(|x| b.iter().map(|y| 近(x, y)).fold(f64::MAX, f64::min) < 1e-9).count();
    println!("抖动 1e-15 之后:{} / {} 条配得上(条数 {} vs {})", 同, a.len(), a.len(), b.len());
    let 宽差 = a
        .iter()
        .zip(&b)
        .map(|(x, y)| (x.width_m - y.width_m).abs())
        .fold(0.0f64, f64::max);
    println!("同序逐条比,宽的最大差 = {宽差:.3e} m");
}

// ───────────── 连"相机在哪"一起量出来(头相机那一格) ─────────────

/// 造一只**斜着架**的相机,位姿和焦距都不是好猜的数。
fn 斜相机() -> Eye {
    // 绕 x 轴转 200°(往下看一点)、再绕 z 轴转 30° —— 手写一个四元数,别用好猜的值
    let (a, b) = (200f64.to_radians() / 2.0, 30f64.to_radians() / 2.0);
    let qa = [a.cos(), a.sin(), 0.0, 0.0];
    let qb = [b.cos(), 0.0, 0.0, b.sin()];
    let q = [
        qb[0] * qa[0] - qb[3] * qa[3],
        qb[0] * qa[1] + qb[3] * qa[2],
        qb[0] * qa[2] - qb[3] * qa[1],
        qb[0] * qa[3] + qb[3] * qa[0],
    ];
    Eye { fx: 517.3, fy: 502.9, cx: 331.2, cy: 246.8, at: [0.21, -0.47, 1.33], q }
}

/// 手挪到一堆位置(**要占一个体积,不能都在一个平面上**)。
fn 手挪的位置() -> Vec<P3> {
    let mut v = Vec::new();
    for i in 0..3 {
        for j in 0..3 {
            for k in 0..3 {
                v.push(P3 {
                    x: -0.18 + 0.18 * i as f64,
                    y: -0.14 + 0.14 * j as f64,
                    z: 0.86 + 0.09 * k as f64,
                });
            }
        }
    }
    v
}

/// 🔴🔴 **只看着自己的手,把相机在哪、焦距多少,一起解出来。**
///
/// 没有标定板、没有配置文件、没有人手填的数 —— 手在哪由本体感受免费给。
/// 这一格是头相机的必需品:它固定在世界里,而那个位姿本身就是没量过的身体常数。
#[test]
fn 连相机在哪一起量出来() {
    let 真 = 斜相机();
    let seen: Vec<(P3, Px)> =
        手挪的位置().into_iter().filter_map(|p| 真.project(p).map(|px| (p, px))).collect();
    assert!(seen.len() >= 20, "样本要够,实得 {}", seen.len());
    let 量 = fit_full(&seen).expect("这组样本该解得出来");

    println!("焦距 真 {:.4}/{:.4} vs 量 {:.4}/{:.4}", 真.fx, 真.fy, 量.fx, 量.fy);
    println!("主点 真 {:.4}/{:.4} vs 量 {:.4}/{:.4}", 真.cx, 真.cy, 量.cx, 量.cy);
    println!("相机在 真 {:?} vs 量 {:?}", 真.at, 量.at.map(|v| (v * 1e6).round() / 1e6));
    assert!((量.fx - 真.fx).abs() < 1e-6, "焦距 fx 差 {}", (量.fx - 真.fx).abs());
    assert!((量.fy - 真.fy).abs() < 1e-6);
    assert!((量.cx - 真.cx).abs() < 1e-6);
    assert!((量.cy - 真.cy).abs() < 1e-6);
    let d = ((量.at[0] - 真.at[0]).powi(2) + (量.at[1] - 真.at[1]).powi(2) + (量.at[2] - 真.at[2]).powi(2)).sqrt();
    assert!(d < 1e-6, "相机位置差 {d:.2e} m");
    // 朝向:拿它去投影,像素要对得上(四元数可能差一个整体符号,比像素才是真判据)
    let mut worst = 0.0f64;
    for (p, px) in &seen {
        let g = 量.project(*p).unwrap();
        worst = worst.max((g[0] - px[0]).abs().max((g[1] - px[1]).abs()));
    }
    assert!(worst < 1e-6, "回代最大差 {worst:.2e} 像素");
}

/// 🔴 反例:手全在**一个平面**上挪(比如只在桌面高度上来回) ⇒ 必须拒绝。
///
/// 一张平面上的点定不下一个完整的投影矩阵 —— 这是经典退化,不是数值问题。
/// 硬解出来的相机位姿会是错的,而**它看起来完全正常**。
#[test]
fn 反例_手只在一个平面上挪必须拒绝() {
    let 真 = 斜相机();
    let mut seen = Vec::new();
    for i in 0..5 {
        for j in 0..5 {
            let p = P3 { x: -0.18 + 0.09 * i as f64, y: -0.14 + 0.07 * j as f64, z: 0.90 };
            if let Some(px) = 真.project(p) {
                seen.push((p, px));
            }
        }
    }
    assert!(seen.len() >= 20);
    assert_eq!(fit_full(&seen), Err(WhyNot::Coplanar), "共面必须拒绝,不许硬解");
}

/// 🔴 反例:样本太少 ⇒ 拒绝(11 个未知数,6 个点才够)。
#[test]
fn 反例_样本太少必须拒绝() {
    let 真 = 斜相机();
    let seen: Vec<(P3, Px)> = 手挪的位置()
        .into_iter()
        .take(4)
        .filter_map(|p| 真.project(p).map(|px| (p, px)))
        .collect();
    assert!(matches!(fit_full(&seen), Err(WhyNot::TooFewSamples(_))));
}

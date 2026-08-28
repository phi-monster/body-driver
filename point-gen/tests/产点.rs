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

// ───────────── 圈物体:锚在眼睛指的那一点上 ─────────────

/// 造一张深度图:一张大桌面(深 1.20 m)+ 桌上一个小物体(深 1.05 m)。
/// **这正是 LAB 那次失败的场景形状** —— 全局规则会把整张桌子一起圈走。
fn 一张桌子加一个物体(w: usize, h: usize) -> Vec<f64> {
    let mut d = vec![1.20f64; w * h];
    let (cx, cy, r) = (w as f64 * 0.5, h as f64 * 0.5, w as f64 * 0.05);
    for y in 0..h {
        for x in 0..w {
            let (dx, dy) = (x as f64 - cx, y as f64 - cy);
            if dx * dx + dy * dy <= r * r {
                d[y * w + x] = 1.05;
            }
        }
    }
    d
}

#[test]
fn 圈物体_只圈眼睛指的那一小块() {
    let (w, h) = (320usize, 240usize);
    let eye = 俯视眼([0.0, 0.0, 1.40]);
    let d = 一张桌子加一个物体(w, h);
    let mask = mask_around(&eye, &d, w, h, [160.0, 120.0], 0.10, 1.5).expect("该圈得出来");
    let n = mask.iter().filter(|b| **b).count();
    let 占 = n as f64 / (w * h) as f64;
    println!("圈出来 {n} 个像素,占全帧 {:.1}%", 占 * 100.0);
    assert!(n > 100, "物体那一片该有几百个像素,实得 {n}");
    // 🔴 圈出来的**必须**都是那个物体(深 1.05),一个桌面像素都不许混进来
    for (i, on) in mask.iter().enumerate() {
        if *on {
            assert!((d[i] - 1.05).abs() < 1e-9, "圈进来一个桌面像素,深度 {}", d[i]);
        }
    }
    assert!(占 < 0.05, "只该圈一小块,实得 {:.1}%", 占 * 100.0);
}

/// 🔴🔴 **把 LAB 那次失败重放一遍:全局规则圈走了 72% 全帧,而这个闸必须响。**
///
/// LAB 原文:那条纯几何规则(比 90 分位近 1 cm)⇒ 掩膜占**全帧 72%**,
/// 而手从 27 cm 逼近到 0.7 cm 时**物体像素宽 601→603 纹丝不动** —— 它圈的根本不是物体。
#[test]
fn 反例_圈得比眼睛说的大太多_必须当场拒绝() {
    let (w, h) = (320usize, 240usize);
    let eye = 俯视眼([0.0, 0.0, 1.40]);
    // 整幅都差不多深(一张大桌面)⇒ 任何"贴着中心深度"的规则都会圈一大片
    let d = vec![1.20f64; w * h];
    match mask_around(&eye, &d, w, h, [160.0, 120.0], 0.10, 8.0) {
        Err(NoMask::TooBig(实, 该)) => {
            println!("挡下来了:实占 {:.1}% vs 该占 {:.1}%", 实 * 100.0, 该 * 100.0);
            assert!(实 > 该);
        }
        other => panic!("圈了一大片桌面,必须当场拒绝;实得 {other:?}"),
    }
}

/// 眼睛指到没有深度的地方 ⇒ 说得出来,不硬圈。
#[test]
fn 反例_指到没东西的地方() {
    let (w, h) = (64usize, 48usize);
    let eye = 俯视眼([0.0, 0.0, 1.40]);
    let d = vec![f64::NAN; w * h];
    assert_eq!(mask_around(&eye, &d, w, h, [32.0, 24.0], 0.2, 1.5), Err(NoMask::NothingThere));
}

/// 🔴 **全链:深度图 + 眼睛指的那一点 → 点云 → 抓取候选。** 接真机时就是这个形状。
#[test]
fn 全链_深度图加一个点_一直走到抓取候选() {
    use contact_gen::{candidates, Body, Grid, JawSpan};
    let (w, h) = (320usize, 240usize);
    let eye = 俯视眼([0.0, 0.0, 1.40]);
    let mut d = vec![1.20f64; w * h];
    let (cx, cy) = (160.0f64, 120.0f64);
    let 半宽 = (0.03 / 1.10 * eye.fx).round();
    for y in 0..h {
        for x in 0..w {
            if (x as f64 - cx).abs() <= 半宽 && (y as f64 - cy).abs() <= 半宽 {
                d[y * w + x] = 1.10;
            }
        }
    }
    let span = 2.0 * 半宽 / w as f64;
    let mask = mask_around(&eye, &d, w, h, [cx, cy], span, 1.5).expect("该圈得出来");
    let 点 = from_depth(&eye, &d, &mask, w, h);
    println!("圈出 {} 个像素 ⇒ {} 个三维点", mask.iter().filter(|b| **b).count(), 点.len());
    assert!(点.len() > 200, "点太少,实得 {}", 点.len());
    let (zlo, zhi) = 点.iter().fold((f64::MAX, f64::MIN), |(a, b), p| (a.min(p.z), b.max(p.z)));
    println!("点的高度范围 {zlo:.4}..{zhi:.4} m");
    assert!((zlo - 0.30).abs() < 1e-9 && (zhi - 0.30).abs() < 1e-9, "顶面该是一个平面");

    // 🔴 只有一个顶面(相机看不见侧面)时,②a 会说什么 —— 照实记,不修饰
    let body = Body { jaw: JawSpan::Measured(0.09), reach_lo: 0.02, reach_hi: 1.5, base_x: 0.0, base_y: -0.4 };
    let grid = Grid { bands: 4, jaw_h_m: 0.02, dirs: 12, min_pts: 8, min_above_m: 0.001, finger_w_m: 0.02, gap_m: 0.012 };
    let 换: Vec<contact_gen::P3> = 点.iter().map(|p| contact_gen::P3 { x: p.x, y: p.y, z: p.z }).collect();
    match candidates(&换, &body, 0.0, grid) {
        Ok(cs) => println!("②a 给出候选 {} 条(只有一个顶面)", cs.len()),
        Err(e) => println!("②a 拒绝:{e:?} —— 只有一层点,切不出层"),
    }
}

// ───────── 一个俯视相机只看得见顶面:两条出路 ─────────

/// 造一个立方体的**表面**点(顶面 + 四个侧面),再由某只眼睛去"看"它:
/// 只有**朝着相机那一侧**的点会被看到(简化的可见性:法向朝着相机)。
fn 看得见的那些点(眼: &Eye, 中心: [f64; 3], 半: f64) -> Vec<P3> {
    let mut all: Vec<(P3, [f64; 3])> = Vec::new();
    let n = 21;
    for i in 0..n {
        for j in 0..n {
            let (a, b) = (
                -半 + 2.0 * 半 * i as f64 / (n - 1) as f64,
                -半 + 2.0 * 半 * j as f64 / (n - 1) as f64,
            );
            all.push((P3 { x: 中心[0] + a, y: 中心[1] + b, z: 中心[2] + 半 }, [0.0, 0.0, 1.0]));
            all.push((P3 { x: 中心[0] + a, y: 中心[1] - 半, z: 中心[2] + b }, [0.0, -1.0, 0.0]));
            all.push((P3 { x: 中心[0] + a, y: 中心[1] + 半, z: 中心[2] + b }, [0.0, 1.0, 0.0]));
            all.push((P3 { x: 中心[0] - 半, y: 中心[1] + a, z: 中心[2] + b }, [-1.0, 0.0, 0.0]));
            all.push((P3 { x: 中心[0] + 半, y: 中心[1] + a, z: 中心[2] + b }, [1.0, 0.0, 0.0]));
        }
    }
    all.into_iter()
        .filter(|(p, nrm)| {
            let d = [眼.at[0] - p.x, 眼.at[1] - p.y, 眼.at[2] - p.z];
            d[0] * nrm[0] + d[1] * nrm[1] + d[2] * nrm[2] > 0.0
        })
        .map(|(p, _)| p)
        .collect()
}

fn 试着抓(点: &[P3]) -> Result<usize, String> {
    use contact_gen::{candidates, Body, Grid, JawSpan};
    let body = contact_gen::Body { jaw: JawSpan::Measured(0.09), reach_lo: 0.02, reach_hi: 1.5, base_x: 0.0, base_y: -0.4 };
    let _ = std::mem::size_of::<Body>();
    let grid = Grid { bands: 5, jaw_h_m: 0.015, dirs: 12, min_pts: 6, min_above_m: 0.001, finger_w_m: 0.02, gap_m: 0.015 };
    let 换: Vec<contact_gen::P3> = 点.iter().map(|p| contact_gen::P3 { x: p.x, y: p.y, z: p.z }).collect();
    let zmin = 换.iter().map(|p| p.z).fold(f64::MAX, f64::min);
    candidates(&换, &body, zmin, grid).map(|c| c.len()).map_err(|e| format!("{e:?}"))
}

/// 🔴🔴 **一个俯视相机 → 抓不了;换个角度再看一眼 → 抓得了。**
///
/// 这一条是这一晚最值钱的读数之一:它是**合成单元测试给不出、而真机上立刻会撞到**的事。
#[test]
fn 一个俯视相机看不见侧面_换个角度就够了() {
    let 中心 = [0.0, 0.0, 0.93];
    let 顶上 = 俯视眼([0.0, 0.0, 1.40]);
    let 只看顶面 = 看得见的那些点(&顶上, 中心, 0.03);
    let (zlo, zhi) = 只看顶面.iter().fold((f64::MAX, f64::MIN), |(a, b), p| (a.min(p.z), b.max(p.z)));
    println!("俯视:{} 个点,高度跨度 {:.4} m", 只看顶面.len(), zhi - zlo);
    let 一个视角 = 试着抓(&只看顶面);
    println!("只用俯视 ⇒ {:?}", 一个视角);
    assert!(一个视角.is_err(), "一张平面切不出层,②a 该拒绝;实得 {:?}", 一个视角);

    // 换一只斜着看的眼睛(相机在侧上方)—— 它看得见一个侧面
    let 侧上 = Eye { fx: 600.0, fy: 600.0, cx: 320.0, cy: 240.0, at: [0.55, 0.0, 1.15], q: 俯视眼([0.0; 3]).q };
    let 侧面 = 看得见的那些点(&侧上, 中心, 0.03);
    let 合起来 = merge(&[只看顶面.clone(), 侧面.clone()]);
    let (zlo2, zhi2) = 合起来.iter().fold((f64::MAX, f64::MIN), |(a, b), p| (a.min(p.z), b.max(p.z)));
    println!("两个视角合起来:{} 个点,高度跨度 {:.4} m", 合起来.len(), zhi2 - zlo2);
    let 两个视角 = 试着抓(&合起来);
    println!("两个视角 ⇒ {:?}", 两个视角);
    assert!(两个视角.is_ok(), "合了侧面之后该抓得出来;实得 {:?}", 两个视角);
}

/// 另一条出路:**只用一个俯视相机,但把顶面朝桌面拉下去** —— 这是【假设】,不是测量。
#[test]
fn 拉到支撑面_能救回来_但那是假设() {
    let 中心 = [0.0, 0.0, 0.93];
    let 顶上 = 俯视眼([0.0, 0.0, 1.40]);
    let 只看顶面 = 看得见的那些点(&顶上, 中心, 0.03);
    let 拉下来 = extrude_to_support(&只看顶面, 0.90, 0.004);
    println!("拉到支撑面之后:{} 个点", 拉下来.len());
    let r = 试着抓(&拉下来);
    println!("拉下来之后 ⇒ {:?}", r);
    assert!(r.is_ok(), "拉出侧面之后该抓得出来;实得 {:?}", r);
    // 🔴 记准它的代价:拉出来的"侧面"是**假设**出来的。马克杯把手底下是空的,
    // 拉下来就成了一堵不存在的墙,而爪子会合到空气上。首选仍然是换个角度再看一眼。
}

// ───────── 外参错一点点,点云差多少 ─────────

/// 🔴🔴 **这一条决定"靠本体感受代替标定"成不成立,而公开资料里没有人量过。**
///
/// 业界主流双目模型都要求"左右图极线对齐 + 已知基线",而我们的主张是
/// *"两只相机的相对位姿由关节角免费给,不需要标定板"*。
/// 问题是:**关节角本身有误差**。差 0.1°,点云差多少毫米?
///
/// 调研 agent 给了一个推导值(f×θ,f=900 px 时 0.1° ≈ 1.57 像素),
/// 但它**自己标明那是几何推导、没有实验支撑**。⇒ 那就自己量。
#[test]
fn 外参差一点点_点云差多少() {
    let 基线 = 0.12f64;
    println!("\n相对朝向差 θ ⇒ 三维点误差(基线 {:.0} cm)", 基线 * 100.0);
    println!("{:>8} {:>12} {:>12} {:>12}", "θ(度)", "0.3 m", "0.6 m", "1.2 m");
    for 度 in [0.02f64, 0.05, 0.1, 0.3, 1.0] {
        let mut 行 = format!("{:>8.2}", 度);
        for 距 in [0.3f64, 0.6, 1.2] {
            let 左 = Eye { fx: 900.0, fy: 900.0, cx: 320.0, cy: 240.0, at: [-基线 / 2.0, 0.0, 1.40], q: [0.0, 1.0, 0.0, 0.0] };
            let 右真 = Eye { at: [基线 / 2.0, 0.0, 1.40], ..左 };
            // 把右眼的朝向拧 θ 度(绕 y 轴)—— 这就是"关节角差了一点"
            let a = 度.to_radians() / 2.0;
            let dq = [a.cos(), 0.0, a.sin(), 0.0];
            let q = 右真.q;
            let 右歪 = Eye {
                q: [
                    dq[0] * q[0] - dq[2] * q[2],
                    dq[0] * q[1] + dq[2] * q[3],
                    dq[0] * q[2] + dq[2] * q[0],
                    dq[0] * q[3] - dq[2] * q[1],
                ],
                ..右真
            };
            // 一个在正前方 `距` 米处的点
            let p = P3 { x: 0.0, y: 0.0, z: 1.40 - 距 };
            let (pa, pb) = (左.project(p).unwrap(), 右真.project(p).unwrap());
            // 用**歪掉的**外参去三角化那两个像素
            match triangulate(&左, pa, &右歪, pb, 1.0) {
                Ok(g) => {
                    let e = ((g.x - p.x).powi(2) + (g.y - p.y).powi(2) + (g.z - p.z).powi(2)).sqrt();
                    行 += &format!(" {:>11.2}mm", e * 1000.0);
                }
                Err(_) => 行 += &format!(" {:>13}", "交不上"),
            }
        }
        println!("{行}");
    }

    // 🔴 更要紧的一问:**外参歪了,这条链自己看得出来吗?**
    // 两条视线错开多远(`RaysMiss` 量的那个数)是不需要真值就观测得到的。
    println!("\n外参歪了,链条自己看不看得出来(两条视线错开多远):");
    for 度 in [0.02f64, 0.05, 0.1, 0.3, 1.0] {
        let 左 = Eye { fx: 900.0, fy: 900.0, cx: 320.0, cy: 240.0, at: [-基线 / 2.0, 0.0, 1.40], q: [0.0, 1.0, 0.0, 0.0] };
        let 右真 = Eye { at: [基线 / 2.0, 0.0, 1.40], ..左 };
        let a = 度.to_radians() / 2.0;
        let dq = [a.cos(), 0.0, a.sin(), 0.0];
        let q = 右真.q;
        let 右歪 = Eye {
            q: [
                dq[0] * q[0] - dq[2] * q[2],
                dq[0] * q[1] + dq[2] * q[3],
                dq[0] * q[2] + dq[2] * q[0],
                dq[0] * q[3] - dq[2] * q[1],
            ],
            ..右真
        };
        // 取一个**偏离光轴**的点,这样错开量才显出来
        let p = P3 { x: 0.05, y: 0.04, z: 0.80 };
        let (pa, pb) = (左.project(p).unwrap(), 右真.project(p).unwrap());
        let miss = match triangulate(&左, pa, &右歪, pb, 0.0) {
            Err(WhyNot::RaysMiss(d)) => d,
            _ => 0.0,
        };
        println!("  θ={度:>4.2}° ⇒ 两条视线错开 {:.3} mm", miss * 1000.0);
    }
}

/// 🔴🔴 **上一条说"外参歪了自己看不出来" —— 那就找一个看得出来的量。**
///
/// 手上有一把**量过的尺子**:钳口跨度 **0.0803 m**(驱动实测)。
/// 两只相机都看得见爪子的两个指尖 ⇒ 三角化出来的跨度,跟量过的那个数一比,
/// 外参歪没歪就露馅了。**这不需要任何外部真值 —— 尺子是身体自己的。**
#[test]
fn 拿量过的钳口跨度当尺子_能不能抓出外参歪了() {
    let 基线 = 0.12f64;
    let 跨度 = 0.0803f64; // 驱动实测的钳口跨度
    println!("\n拿钳口跨度({:.4} m)当尺子:", 跨度);
    println!("{:>8} {:>16} {:>16}", "θ(度)", "视线错开", "量出来的跨度差");
    for 度 in [0.02f64, 0.05, 0.1, 0.3, 1.0] {
        let 左 = Eye { fx: 900.0, fy: 900.0, cx: 320.0, cy: 240.0, at: [-基线 / 2.0, 0.0, 1.40], q: [0.0, 1.0, 0.0, 0.0] };
        let 右真 = Eye { at: [基线 / 2.0, 0.0, 1.40], ..左 };
        let a = 度.to_radians() / 2.0;
        let dq = [a.cos(), 0.0, a.sin(), 0.0];
        let q = 右真.q;
        let 右歪 = Eye {
            q: [
                dq[0] * q[0] - dq[2] * q[2],
                dq[0] * q[1] + dq[2] * q[3],
                dq[0] * q[2] + dq[2] * q[0],
                dq[0] * q[3] - dq[2] * q[1],
            ],
            ..右真
        };
        // 两个指尖:在 0.6 m 处、沿 x 张开一个已知跨度
        let a1 = P3 { x: -跨度 / 2.0, y: 0.0, z: 0.80 };
        let a2 = P3 { x: 跨度 / 2.0, y: 0.0, z: 0.80 };
        let mut 出 = Vec::new();
        let mut 错开 = 0.0f64;
        for p in [a1, a2] {
            let (pa, pb) = (左.project(p).unwrap(), 右真.project(p).unwrap());
            if let Err(WhyNot::RaysMiss(d)) = triangulate(&左, pa, &右歪, pb, 0.0) {
                错开 = 错开.max(d);
            }
            出.push(triangulate(&左, pa, &右歪, pb, 1.0).unwrap());
        }
        let 量出来 = ((出[1].x - 出[0].x).powi(2) + (出[1].y - 出[0].y).powi(2) + (出[1].z - 出[0].z).powi(2)).sqrt();
        println!(
            "{:>8.2} {:>13.4}mm {:>13.3}mm",
            度,
            错开 * 1000.0,
            (量出来 - 跨度).abs() * 1000.0
        );
    }
}

// ───────── 尺子:一道会自己响的闸 ─────────

/// 造一对眼睛,可选把右眼拧歪 θ 度。
fn 一对眼(θ度: f64) -> (Eye, Eye) {
    let 左 = Eye { fx: 900.0, fy: 900.0, cx: 320.0, cy: 240.0, at: [-0.06, 0.0, 1.40], q: [0.0, 1.0, 0.0, 0.0] };
    let 右真 = Eye { at: [0.06, 0.0, 1.40], ..左 };
    let a = θ度.to_radians() / 2.0;
    let dq = [a.cos(), 0.0, a.sin(), 0.0];
    let q = 右真.q;
    let 右 = Eye {
        q: [
            dq[0] * q[0] - dq[2] * q[2],
            dq[0] * q[1] + dq[2] * q[3],
            dq[0] * q[2] + dq[2] * q[0],
            dq[0] * q[3] - dq[2] * q[1],
        ],
        ..右真
    };
    (左, 右真, ).0;
    (左, 右)
}

/// 两个指尖在世界里的位置(张开 `跨度`,在 0.6 m 处)。
fn 两个指尖(跨度: f64) -> (P3, P3) {
    (P3 { x: -跨度 / 2.0, y: 0.0, z: 0.80 }, P3 { x: 跨度 / 2.0, y: 0.0, z: 0.80 })
}

#[test]
fn 尺子_没飘的时候放行() {
    let 跨度 = 0.0803f64;
    let (左, 右) = 一对眼(0.0);
    let (a1, a2) = 两个指尖(跨度);
    let 端1 = (左.project(a1).unwrap(), 右.project(a1).unwrap());
    let 端2 = (左.project(a2).unwrap(), 右.project(a2).unwrap());
    let got = check_ruler(&左, &右, 端1, 端2, 跨度, 0.001).expect("没飘就该放行");
    assert!((got - 跨度).abs() < 1e-9, "量出来该就是那个数,实得 {got:.6}");
}

/// 🔴🔴 **闸必须在"视线错开查不出来"的那种飘法上响** —— 那正是它存在的理由。
#[test]
fn 尺子_飘了就必须拦住_而视线错开查不出来() {
    let 跨度 = 0.0803f64;
    // 真值用没歪的右眼投影(模拟"世界没变,只是我以为的相机位姿歪了")
    let (左, 右真) = 一对眼(0.0);
    let (a1, a2) = 两个指尖(跨度);
    let 端1 = (左.project(a1).unwrap(), 右真.project(a1).unwrap());
    let 端2 = (左.project(a2).unwrap(), 右真.project(a2).unwrap());

    for (θ, 该不该拦) in [(0.1f64, false), (0.3, true), (1.0, true)] {
        let (_, 右歪) = 一对眼(θ);
        // ① 视线错开这个信号(容差设成 0 就把错开量报出来)
        let 错开 = match triangulate(&左, 端1.0, &右歪, 端1.1, 0.0) {
            Err(WhyNot::RaysMiss(d)) => d,
            _ => 0.0,
        };
        // ② 尺子
        let r = check_ruler(&左, &右歪, 端1, 端2, 跨度, 0.0015);
        match &r {
            Err(Drift::Off { got_m, .. }) => println!(
                "θ={θ:>4.1}° ⇒ 视线错开 {:.4} mm(查不出)· 尺子差 {:.3} mm ⇒ **拦住**",
                错开 * 1000.0,
                (got_m - 跨度).abs() * 1000.0
            ),
            Ok(g) => println!(
                "θ={θ:>4.1}° ⇒ 视线错开 {:.4} mm · 尺子差 {:.3} mm ⇒ 放行",
                错开 * 1000.0,
                (g - 跨度).abs() * 1000.0
            ),
            Err(e) => println!("θ={θ:>4.1}° ⇒ {e:?}"),
        }
        assert!(错开 * 1000.0 < 0.02, "🔴 视线错开在这种飘法下几乎为零 —— 拿它当闸没用");
        if 该不该拦 {
            assert!(matches!(r, Err(Drift::Off { .. })), "θ={θ}° 该被拦住");
        }
    }
}

/// 尺子填错了(给了一个非正数)⇒ 说得出来。
#[test]
fn 尺子_填错了也要说得出来() {
    let (左, 右) = 一对眼(0.0);
    let (a1, a2) = 两个指尖(0.08);
    let 端1 = (左.project(a1).unwrap(), 右.project(a1).unwrap());
    let 端2 = (左.project(a2).unwrap(), 右.project(a2).unwrap());
    assert_eq!(check_ruler(&左, &右, 端1, 端2, 0.0, 0.001), Err(Drift::BadRuler));
}

// ───────── 各种传感器怎么适配:出口只有一个 ─────────

/// 🔴🔴 **一只普通相机 + 机器人自己挪一下 = 双目,而且用的是同一个三角化。**
///
/// 双目真正的难点从来不是"两条视线怎么交",是"两个视角之间差多少"。
/// 别人靠标定板或 SLAM 去估它;**机器人自己挪了多远,本体感受直接给。**
#[test]
fn 单目加运动_就是双目_一行新几何都不用写() {
    let 相机在左 = 俯视眼([-0.06, 0.0, 1.40]);
    let 相机挪到右 = 俯视眼([0.06, 0.0, 1.40]); // 胳膊把它平移了 12 cm
    let 真 = 圆柱();
    let pairs: Vec<(Px, Px)> = 真
        .iter()
        .filter_map(|p| match (相机在左.project(*p), 相机挪到右.project(*p)) {
            (Some(a), Some(b)) => Some((a, b)),
            _ => None,
        })
        .collect();
    let (点, 丢) = from_motion(&相机在左, &相机挪到右, &pairs, 0.5, 0.1, 1e-6).expect("挪够了就该成");
    assert_eq!(丢, 0);
    let mut worst = 0.0f64;
    for (g, p) in 点.iter().zip(&真) {
        worst = worst.max(((g.x - p.x).powi(2) + (g.y - p.y).powi(2) + (g.z - p.z).powi(2)).sqrt());
    }
    println!("单目挪 12 cm ⇒ {} 个点,最大误差 {:.2e} m", 点.len(), worst);
    assert!(worst < 1e-9);
}

/// 🔴 反例:**挪得太少就必须拒绝** —— 三角形太扁,深度极度不敏感。
#[test]
fn 反例_挪得太少必须拒绝而不是给个烂点() {
    let 前 = 俯视眼([0.0, 0.0, 1.40]);
    let 后 = 俯视眼([0.004, 0.0, 1.40]); // 只挪了 4 mm
    let 真 = 圆柱();
    let pairs: Vec<(Px, Px)> = 真
        .iter()
        .filter_map(|p| match (前.project(*p), 后.project(*p)) {
            (Some(a), Some(b)) => Some((a, b)),
            _ => None,
        })
        .collect();
    match from_motion(&前, &后, &pairs, 0.5, 0.1, 1e-6) {
        Err(WhyNot::BaselineTooShort { got_m, need_m }) => {
            println!("挪了 {:.1} mm,至少要 {:.1} mm ⇒ 拒绝", got_m * 1000.0, need_m * 1000.0);
            assert!(got_m < need_m);
        }
        other => panic!("挪得太少必须拒绝;实得 {other:?}"),
    }
}

/// **激光雷达 / 摸 —— 类型完全一样,`merge` 直接拌在一起。**
#[test]
fn 雷达和摸_跟相机产的点是同一个类型_能直接混() {
    // 雷达在机器人身上某处,朝下扫;它出的点在自己的坐标系里
    let 雷达点: Vec<[f64; 3]> = (0..50)
        .map(|i| {
            let a = core::f64::consts::TAU * i as f64 / 50.0;
            [0.03 * a.cos(), 0.03 * a.sin(), 0.40]
        })
        .collect();
    let 雷达出的 = from_sensor_frame(&雷达点, [0.0, 0.0, 1.35], [0.0, 1.0, 0.0, 0.0]);
    // 摸到的两个点(碰一下拿到的,比看到的准,而且不怕反光/透明)
    let 摸到的 = from_touch(&[[0.031, 0.0, 0.93], [-0.031, 0.0, 0.93]]);
    // 相机产的点
    let 眼 = 俯视眼([0.0, 0.0, 1.40]);
    let 相机出的: Vec<P3> = 圆柱()
        .iter()
        .filter_map(|p| 眼.project(*p).map(|px| 眼.back_project(px, 眼.into_cam(*p)[2]).unwrap()))
        .collect();

    let 拌在一起 = merge(&[相机出的.clone(), 雷达出的.clone(), 摸到的.clone()]);
    println!(
        "相机 {} + 雷达 {} + 摸 {} = {} 个点,同一个类型,直接拌",
        相机出的.len(), 雷达出的.len(), 摸到的.len(), 拌在一起.len()
    );
    assert_eq!(拌在一起.len(), 相机出的.len() + 雷达出的.len() + 摸到的.len());
    // 雷达那圈点转到世界之后,该落在它该在的高度上(1.35 − 0.40 = 0.95)
    for p in &雷达出的 {
        assert!((p.z - 0.95).abs() < 1e-12, "雷达点该在 0.95 m,实得 {}", p.z);
    }
}

/// 每个来源**自报有多准** —— 混着用时,上层才判断得了信谁。
#[test]
fn 每个来源自报有多准() {
    println!("\n双目的误差(像素误差 0.5 px、焦距 900):");
    for 基线 in [0.06f64, 0.12, 0.25] {
        let mut 行 = format!("  基线 {:>4.0} cm:", 基线 * 100.0);
        for 距 in [0.3f64, 0.6, 1.2] {
            行 += &format!(" {:.0} m ⇒ {:>6.2}mm", 距, sigma_stereo(距, 基线, 900.0, 0.5) * 1000.0);
        }
        println!("{行}");
    }
    // 远处误差必须比近处大,基线大误差必须小 —— 这两条单调性是判据
    assert!(sigma_stereo(1.2, 0.12, 900.0, 0.5) > sigma_stereo(0.3, 0.12, 900.0, 0.5));
    assert!(sigma_stereo(0.6, 0.25, 900.0, 0.5) < sigma_stereo(0.6, 0.06, 900.0, 0.5));
}

// ───────── 相机模型不准时,坏的是"位置"还是"宽度" ─────────

/// 🔴🔴 **我一直在拿【瞄准】的尺子,量一件【形状】的事。这一炮把两者拆开。**
///
/// 那个 8 px(≈16 mm)的判据来自**合爪窗口** —— 那是"**手要伸到准确的位置上**"的要求。
/// 而接触集要的是*"这物体多宽、从哪儿下手"* —— **相对量**。
/// 相对量对**共模**误差免疫:整团点云平移/旋转,量出来的宽度**一点不变**。
///
/// 实测口径:拿真相机投影,再用**歪掉的**相机反投影,分别量
/// **① 整团点挪了多远(绝对)· ② 量出来的宽度差多少(相对)**。
#[test]
fn 相机歪了_坏的是位置还是宽度() {
    let 真 = 俯视眼([0.0, 0.0, 1.40]);
    let cloud = 圆柱(); // 半径 3 cm ⇒ 真宽 6 cm
    let 真宽 = 0.06f64;
    let 宽 = |v: &[P3]| -> f64 {
        let (mut lo, mut hi) = (f64::MAX, f64::MIN);
        for p in v {
            lo = lo.min(p.x);
            hi = hi.max(p.x);
        }
        hi - lo
    };
    println!("\n{:22} {:>14} {:>14}", "相机哪儿歪了", "整团挪了(mm)", "宽度差(mm)");
    let mut 案例: Vec<(&str, Eye)> = Vec::new();
    // ① 位姿平移歪 2 cm
    案例.push(("位姿平移歪 2 cm", Eye { at: [0.02, 0.0, 1.40], ..真 }));
    // ② 位姿转歪 1°(绕 y)
    {
        let a = 1f64.to_radians() / 2.0;
        let (c, s) = (a.cos(), a.sin());
        let q = 真.q;
        案例.push((
            "位姿转歪 1°",
            Eye {
                q: [c * q[0] - s * q[2], c * q[1] + s * q[3], c * q[2] + s * q[0], c * q[3] - s * q[1]],
                ..真
            },
        ));
    }
    // ③ 焦距歪 2%(= 尺度)
    案例.push(("焦距歪 2%", Eye { fx: 真.fx * 1.02, fy: 真.fy * 1.02, ..真 }));
    // ④ 主点歪 10 px
    案例.push(("主点歪 10 px", Eye { cx: 真.cx + 10.0, ..真 }));

    for (名, 歪) in &案例 {
        let mut 出 = Vec::new();
        for p in &cloud {
            if let Some(px) = 真.project(*p) {
                let d = 真.into_cam(*p)[2];
                if let Ok(q) = 歪.back_project(px, d) {
                    出.push(q);
                }
            }
        }
        let (mut mx, mut my, mut mz) = (0.0, 0.0, 0.0);
        for (a, b) in 出.iter().zip(&cloud) {
            mx += a.x - b.x;
            my += a.y - b.y;
            mz += a.z - b.z;
        }
        let n = 出.len() as f64;
        let 挪 = ((mx / n).powi(2) + (my / n).powi(2) + (mz / n).powi(2)).sqrt();
        println!("{:22} {:>13.1} {:>13.1}", 名, 挪 * 1000.0, (宽(&出) - 真宽).abs() * 1000.0);
    }
    println!("\n🔴 判据:合爪窗口 16 mm。看【宽度差】那一列够不够得着它 ——");
    println!("   共模的那几项(平移/转/主点)宽度差应当≈0;只有【尺度】那一项会污染宽度。");
}

/// 🔴 **减掉桌面:一张【斜着的】大平面 + 上面一个小物体 ⇒ 只该剩物体。**
///
/// 斜着是关键:g5 那次假成功正是因为相机斜看,桌面上一块 10 cm 区域自带 60 mm 高差,
/// 深度筛子把整片桌面当成了物体。**平面判据对倾斜免疫,深度数不免疫。**
#[test]
fn 减掉桌面_只剩物体() {
    let mut pts = Vec::new();
    // 一张斜 15° 的桌面(200 个点)
    let k = 15f64.to_radians().tan();
    for i in 0..20 {
        for j in 0..20 {
            let (x, y) = (-0.10 + 0.01 * i as f64, -0.10 + 0.01 * j as f64);
            pts.push(P3 { x, y, z: 0.90 + k * x });
        }
    }
    let 桌面数 = pts.len();
    // 桌上一个 3 cm 见方、高 4 cm 的物体(顶面 60 个点)
    for i in 0..8 {
        for j in 0..8 {
            let (x, y) = (-0.015 + 0.004 * i as f64, -0.015 + 0.004 * j as f64);
            pts.push(P3 { x, y, z: 0.90 + k * x + 0.04 });
        }
    }
    let 物体数 = pts.len() - 桌面数;
    let (剩, nv, 面上) = drop_support_plane(&pts, 0.004);
    println!("桌面 {桌面数} + 物体 {物体数} ⇒ 判为面上 {面上} · 剩 {} · 面法向 {:?}", 剩.len(),
        nv.map(|v| (v * 100.0).round() / 100.0));
    assert!(面上 >= 桌面数, "整张桌面都该被认出来,实得 {面上}");
    assert_eq!(剩.len(), 物体数, "剩下的该正好是物体那些点");
    for p in &剩 {
        assert!((p.z - (0.90 + k * p.x + 0.04)).abs() < 1e-9, "剩下的必须都在物体上");
    }
}

/// 🔴 **姿态不变时,指尖偏置和相机位置在数学上分不开 —— 必须拒绝。**
///
/// `观测点 = p + R·d`,R 恒定 ⇒ R·d 是常数偏移,**和把相机整体挪一段完全等价**。
/// 实测(这条测试):真值 d=0,而解出偏置 **-0.1300**、留出残差 **2.5e-16**(完美拟合),
/// 相机位置因此错 **13 cm** —— 拿去抓必然抓空,而所有数看起来都是绿的。
/// ⇒ 这一格只能靠**采样期间手腕真的转过**来解,松闸就是放进一个会毁掉抓取的解。
#[test]
fn 姿态不变时偏置解不出来_必须拒绝() {
    let 斜 = 斜相机();
    let 真 = Eye { fx: 1.083, fy: 1.047, cx: 0.517, cy: 0.482, at: 斜.at, q: 斜.q };
    let seen: Vec<([f64; 7], Px)> = 手挪的位置()
        .into_iter()
        .filter_map(|p| 真.project(p).map(|px| ([p.x, p.y, p.z, 1.0, 0.0, 0.0, 0.0], px)))
        .collect();
    assert!(seen.len() >= 12, "样本要够,实得 {}", seen.len());
    match fit_full_axis_offset(&seen) {
        Err(WhyNot::AxisAmbiguous(比)) => println!("按预期拒了:三根轴留出比 {比:.4}"),
        Err(e) => panic!("拒的理由不对,该是 AxisAmbiguous:{e:?}"),
        Ok((_, k, t, med)) => panic!("姿态不变时不该解得出来,却给了轴 {k} 偏置 {t:.4}(留出 {med:.2e})"),
    }
}

/// 🔴 **从导数把整台相机解回来 —— 造一台已知的,算它的导数,再解回去,必须一模一样。**
///
/// 这条路和 `fit_full` 是**两种东西**:那条要全局拟合,会被点共面 / 全在一个深度 / 姿态不变
/// 这些退化打死(实测连着六炮解不出来);这条**不拟合**,只用一点上的导数,闭式。
#[test]
fn 从导数把整台相机解回来() {
    let 真 = 斜相机(); // fx/fy/cx/cy 用像素单位,at/q 是它自己的位姿
    let 手 = P3 { x: 0.07, y: -0.11, z: 0.93 };
    // 数值求导:P ↦ (u, v, 沿光轴的深)
    let 深 = |p: P3| 真.into_cam(p)[2];
    let mut j = [[0.0f64; 3]; 3];
    let e = 1e-6;
    for k in 0..3 {
        let mut a = 手; let mut b = 手;
        match k { 0 => { a.x -= e; b.x += e } 1 => { a.y -= e; b.y += e } _ => { a.z -= e; b.z += e } }
        let (pa, pb) = (真.project(a).unwrap(), 真.project(b).unwrap());
        j[0][k] = (pb[0] - pa[0]) / (2.0 * e);
        j[1][k] = (pb[1] - pa[1]) / (2.0 * e);
        j[2][k] = (深(b) - 深(a)) / (2.0 * e);
    }
    let px = 真.project(手).unwrap();
    let 量 = eye_from_jacobian(j, px[0], px[1], 深(手), [手.x, 手.y, 手.z])
        .expect("导数是准的,这台相机该解得出来");
    println!("焦距 真 {:.3}/{:.3} vs 量 {:.3}/{:.3}", 真.fx, 真.fy, 量.fx, 量.fy);
    println!("主点 真 {:.3}/{:.3} vs 量 {:.3}/{:.3}", 真.cx, 真.cy, 量.cx, 量.cy);
    println!("相机在 真 {:?} vs 量 {:?}", 真.at, 量.at.map(|v| (v * 1e4).round() / 1e4));
    assert!((量.fx - 真.fx).abs() < 1e-3, "fx 差 {}", (量.fx - 真.fx).abs());
    assert!((量.fy - 真.fy).abs() < 1e-3, "fy 差 {}", (量.fy - 真.fy).abs());
    assert!((量.cx - 真.cx).abs() < 1e-3, "cx 差 {}", (量.cx - 真.cx).abs());
    assert!((量.cy - 真.cy).abs() < 1e-3, "cy 差 {}", (量.cy - 真.cy).abs());
    let d = ((量.at[0]-真.at[0]).powi(2) + (量.at[1]-真.at[1]).powi(2) + (量.at[2]-真.at[2]).powi(2)).sqrt();
    assert!(d < 1e-4, "相机位置差 {d:.2e} m");
    // 朝向:拿它去投影一批点,像素要对得上(四元数可能差整体符号,比像素才是真判据)
    let mut worst = 0.0f64;
    for p in 手挪的位置() {
        if let (Some(a), Some(b)) = (真.project(p), 量.project(p)) {
            worst = worst.max((a[0]-b[0]).abs().max((a[1]-b[1]).abs()));
        }
    }
    assert!(worst < 1e-3, "回代最大差 {worst:.2e} 像素");
}

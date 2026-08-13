//! **画面上的一点 ↔ 桌面上的一点。** 眼只会说"它在画面这儿",而手只会去世界坐标;这一格
//! 是两者之间唯一的换算,而且它**由机器人自己挥手量出来**,不是谁填进去的相机参数。
//!
//! # 为什么这一格存在,以及它替掉了什么
//!
//! 在此之前,物体的世界坐标来自两个已删掉的文件:一个**读判分文件**,一个**读完整三维模型**。
//! 那是作弊和特权。眼接上之后,能拿到的只有画面上的一个点 —— 于是"画面 → 桌面"成了整条链
//! 上唯一断掉的一环。
//!
//! 补法不需要任何外部标定:**手到几个已知位置,各晃一次钳口,读数器报出它在画面的哪一点**
//! (`crate::blob` + `crate::hand`),几组对子就把这张表定死了。真机上同样做得到 —— 机器人
//! 挥挥自己的手,就把自己的相机标了。
//!
//! # 为什么至少要四个点
//!
//! 仿射映射有六个自由度,**三个点恰好把它定死** —— 于是**任何**三个点都会报出零残差,包括
//! 三个完全错的点。`crate::floor` 为平面记过同一条:*"三点精确定平面,而一个恰定拟合报出的
//! 零残差不是证据"*。这里照抄那条规矩:**少于四个点直接拒绝**,四个点起才有一个自由度可以
//! 用来检查自己,而残差就是那份检查。
//!
//! # 它诚实地不知道什么
//!
//! 这张表是在**手当时那个高度**上量的。物体躺在桌面上,比手低一截,**视差没有算进去**,所以
//! 换算出来的位置有一个随高度而变的偏移。这一格不假装知道那个偏移;把它交给下游那一步**会
//! 失败的检查**:按换算的位置压下去,地面图必须答"这儿有东西";答"这是桌面"就是这张表在这
//! 一带不够用,而那是一条能报警的路径,不是一个悄悄的偏差。

/// 拟合不出来的原因。每一条都是"这批点本身说明不了问题",不是"算错了"。
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Bad {
    /// 少于四个点。三个点会给出零残差,而那不是证据。
    NotEnoughPoints,
    /// 点都挤在一条线上(或一个点上),两条轴分不开 —— 沿那条线之外的答案是编的。
    Degenerate,
    /// 有非有限的数。
    NotFinite,
}

/// 一张量出来的换算表,连同它自己的残差。
#[derive(Copy, Clone, Debug)]
pub struct Map {
    /// `u = a[0]*x + a[1]*y + a[2]`
    pub a: [f64; 3],
    /// `v = b[0]*x + b[1]*y + b[2]`
    pub b: [f64; 3],
    /// 拟合残差(归一化画面单位,均方根)。**这是这张表唯一的自检**,四点时才有意义。
    pub residual: f64,
    /// 用了几个点。
    pub n: usize,
}

/// 最少几个点。见文件头:三点零残差不是证据。
pub const MIN_POINTS: usize = 4;

/// 由若干组 `(世界x, 世界y, 画面u, 画面v)` 拟合。
pub fn fit(pts: &[(f64, f64, f64, f64)]) -> Result<Map, Bad> {
    if pts.len() < MIN_POINTS {
        return Err(Bad::NotEnoughPoints);
    }
    if pts.iter().any(|p| ![p.0, p.1, p.2, p.3].iter().all(|v| v.is_finite())) {
        return Err(Bad::NotFinite);
    }

    // 法方程:对 [x y 1] 做最小二乘,两条输出各解一次。
    let n = pts.len() as f64;
    let (mut sx, mut sy, mut sxx, mut sxy, mut syy) = (0.0, 0.0, 0.0, 0.0, 0.0);
    for p in pts {
        sx += p.0;
        sy += p.1;
        sxx += p.0 * p.0;
        sxy += p.0 * p.1;
        syy += p.1 * p.1;
    }
    // 去中心之后的散布矩阵;它的行列式为零就是"点在一条线上"。
    let (mx, my) = (sx / n, sy / n);
    let (cxx, cxy, cyy) = (sxx - n * mx * mx, sxy - n * mx * my, syy - n * my * my);
    let det = cxx * cyy - cxy * cxy;
    // 判据不是拍的:与两轴各自的散布相比,行列式小到这个地步就说明两轴线性相关。
    if !det.is_finite() || det.abs() <= 1e-12 * (cxx * cyy).abs().max(1e-12) {
        return Err(Bad::Degenerate);
    }

    let solve = |get: &dyn Fn(&(f64, f64, f64, f64)) -> f64| -> [f64; 3] {
        let mut sz = 0.0;
        let (mut sxz, mut syz) = (0.0, 0.0);
        for p in pts {
            let z = get(p);
            sz += z;
            sxz += p.0 * z;
            syz += p.1 * z;
        }
        let mz = sz / n;
        let (cxz, cyz) = (sxz - n * mx * mz, syz - n * my * mz);
        let k1 = (cxz * cyy - cyz * cxy) / det;
        let k2 = (cyz * cxx - cxz * cxy) / det;
        [k1, k2, mz - k1 * mx - k2 * my]
    };
    let a = solve(&|p| p.2);
    let b = solve(&|p| p.3);

    let mut ss = 0.0;
    for p in pts {
        let du = a[0] * p.0 + a[1] * p.1 + a[2] - p.2;
        let dv = b[0] * p.0 + b[1] * p.1 + b[2] - p.3;
        ss += du * du + dv * dv;
    }
    // 每个点两个方程、一共六个未知数,所以自由度是 2n-6。四点时是 2,不是 0。
    let dof = (2 * pts.len()).saturating_sub(6).max(1) as f64;
    Ok(Map { a, b, residual: (ss / dof).sqrt(), n: pts.len() })
}

/// 🔴 **留一验证:每个点都用【另外那些点】去预测它自己。**
///
/// 拟合残差有一个致命的诚实问题 —— 参数是**用同一批点算出来的**,所以残差衡量的是"这批点
/// 彼此有多自洽",不是"这张表在一个没见过的地方有多准"。四个点、六个自由度时,那份残差只
/// 剩两个自由度,几乎必然好看。
///
/// 留一给出的是**样本外**误差:抽掉一个点,用剩下的重新拟合,再去预测被抽掉那个。这就是这
/// 张表将来面对一个新位置时的处境,所以它才是能拿去用的那个数。
///
/// 返回每个点的样本外误差(归一化画面单位)。少于 `MIN_POINTS + 1` 个点时留一之后就不够拟
/// 合了,直接拒绝 —— 而不是返回一串好看的零。
pub fn leave_one_out(pts: &[(f64, f64, f64, f64)]) -> Result<Vec<f64>, Bad> {
    if pts.len() < MIN_POINTS + 1 {
        return Err(Bad::NotEnoughPoints);
    }
    let mut out = Vec::with_capacity(pts.len());
    for skip in 0..pts.len() {
        let rest: Vec<(f64, f64, f64, f64)> = pts
            .iter()
            .enumerate()
            .filter(|(i, _)| *i != skip)
            .map(|(_, p)| *p)
            .collect();
        let m = fit(&rest)?;
        let (pu, pv) = m.to_pixel(pts[skip].0, pts[skip].1);
        out.push(((pu - pts[skip].2).powi(2) + (pv - pts[skip].3).powi(2)).sqrt());
    }
    Ok(out)
}

impl Map {
    /// 画面上的一点 → 桌面上的一点。逆变换退化时返回 `None` 而不是一个编出来的坐标。
    pub fn to_world(&self, u: f64, v: f64) -> Option<(f64, f64)> {
        if !(u.is_finite() && v.is_finite()) {
            return None;
        }
        let det = self.a[0] * self.b[1] - self.a[1] * self.b[0];
        if !det.is_finite() || det.abs() < 1e-12 {
            return None;
        }
        let (du, dv) = (u - self.a[2], v - self.b[2]);
        Some((
            (du * self.b[1] - dv * self.a[1]) / det,
            (dv * self.a[0] - du * self.b[0]) / det,
        ))
    }

    /// 桌面上的一点 → 画面上的一点。用来自检:量过的点回代必须落回原处。
    pub fn to_pixel(&self, x: f64, y: f64) -> (f64, f64) {
        (
            self.a[0] * x + self.a[1] * y + self.a[2],
            self.b[0] * x + self.b[1] * y + self.b[2],
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 🔴 今晚真机量到的三个对子 —— **它们必须被拒绝**,因为三个点的零残差不是证据。
    /// 这条和 `crate::floor::MIN_SAMPLES` 是同一条教训,写成两处会失败的检查。
    #[test]
    fn three_real_pairs_are_refused_because_zero_residual_proves_nothing() {
        let three = [
            (-0.05, -0.18, 0.54001, 0.24535),
            (-0.05, -0.08, 0.52919, 0.13961),
            (-0.15, -0.08, 0.42660, 0.12647),
        ];
        assert_eq!(fit(&three).unwrap_err(), Bad::NotEnoughPoints);
    }

    /// 四个点(前三个是真机读数,第四个由前三个张成的仿射关系补出)能拟合,且回代还原。
    #[test]
    fn four_points_fit_and_round_trip() {
        // 由真机三点定出的关系再取一个点,模拟"第四炮量到了它"。
        let pts = [
            (-0.05, -0.18, 0.54001, 0.24535),
            (-0.05, -0.08, 0.52919, 0.13961),
            (-0.15, -0.08, 0.42660, 0.12647),
            (-0.15, -0.18, 0.43742, 0.23221),
        ];
        let m = fit(&pts).expect("四个点足够");
        assert_eq!(m.n, 4);
        for p in &pts {
            let (x, y) = m.to_world(p.2, p.3).expect("逆变换存在");
            assert!((x - p.0).abs() < 2e-3, "x {x} vs {}", p.0);
            assert!((y - p.1).abs() < 2e-3, "y {y} vs {}", p.1);
        }
        // 这四个点本来就相容,残差应当很小 —— 但**小不等于对**,它只说明这四点自洽。
        assert!(m.residual < 5e-3, "residual {}", m.residual);
    }

    /// 🔴 残差是这张表唯一的自检,所以它必须**能大**:塞一个错点进去,残差要跳起来。
    #[test]
    fn a_wrong_point_shows_up_in_the_residual() {
        let good = [
            (-0.05, -0.18, 0.54001, 0.24535),
            (-0.05, -0.08, 0.52919, 0.13961),
            (-0.15, -0.08, 0.42660, 0.12647),
            (-0.15, -0.18, 0.43742, 0.23221),
        ];
        let clean = fit(&good).unwrap().residual;
        let mut bad = good;
        bad[3].2 += 0.15; // 画面上错了 15% 的宽度
        let dirty = fit(&bad).unwrap().residual;
        assert!(dirty > 10.0 * clean.max(1e-6), "干净 {clean} 脏 {dirty}");
    }

    /// 点挤在一条线上 ⇒ 拒绝。沿线之外的答案没有任何数据支撑,而它看起来会完全正常。
    #[test]
    fn points_on_one_line_are_refused_rather_than_extrapolated() {
        let line = [
            (-0.05, -0.18, 0.54, 0.245),
            (-0.10, -0.18, 0.49, 0.240),
            (-0.15, -0.18, 0.44, 0.235),
            (-0.20, -0.18, 0.39, 0.230),
        ];
        assert_eq!(fit(&line).unwrap_err(), Bad::Degenerate);
    }

    #[test]
    fn non_finite_input_is_refused() {
        let pts = [
            (-0.05, -0.18, 0.54, 0.245),
            (-0.05, -0.08, 0.53, 0.140),
            (-0.15, -0.08, 0.43, 0.126),
            (f64::NAN, -0.18, 0.44, 0.232),
        ];
        assert_eq!(fit(&pts).unwrap_err(), Bad::NotFinite);
    }
}

#[cfg(test)]
mod loo_tests {
    use super::*;

    /// 🔴 今晚真机量到的**四个**对子(锁死朝向、同一套采样、15 cm 宽基线)。
    ///
    /// 四个点做留一之后只剩三个,而三点恰定 ⇒ 按本文件的规矩要拒绝。这条把"数据到手了但
    /// 还不够验"钉成一条会失败的检查,而不是让它悄悄退化成一个好看的零。
    #[test]
    fn four_real_pairs_cannot_be_leave_one_out_checked() {
        let four = [
            (-0.2500, -0.2000, 0.27432, 0.29791),
            (-0.1000, -0.2000, 0.43365, 0.31652),
            (-0.2500, -0.0600, 0.30814, 0.17977),
            (-0.1750, -0.1300, 0.36409, 0.23208),
        ];
        assert!(fit(&four).is_ok(), "四点足够拟合");
        assert_eq!(
            leave_one_out(&four).unwrap_err(),
            Bad::NotEnoughPoints,
            "但抽掉一个只剩三个,三点恰定 ⇒ 留一给不出诚实的样本外误差"
        );
    }

    /// 五个点起,留一才有意义,而它必须**能大** —— 塞一个错点进去,那个点的样本外误差要跳。
    #[test]
    fn leave_one_out_catches_the_odd_point_out() {
        let mut pts = vec![
            (-0.2500, -0.2000, 0.27432, 0.29791),
            (-0.1000, -0.2000, 0.43365, 0.31652),
            (-0.2500, -0.0600, 0.30814, 0.17977),
            (-0.1000, -0.0600, 0.46747, 0.19838),
            (-0.1750, -0.1300, 0.37088, 0.24815),
        ];
        let clean = leave_one_out(&pts).expect("五个点够留一");
        let worst_clean = clean.iter().cloned().fold(0.0, f64::max);

        pts[4].2 += 0.10; // 把中间那个点在画面上挪 10% 的宽度
        let dirty = leave_one_out(&pts).unwrap();
        assert!(dirty[4] > worst_clean + 0.05, "错点 {} 干净最差 {}", dirty[4], worst_clean);
    }
}

// ============================================================================================
// 🔴 透视变换:一个平面被相机拍下来,数学上就是这个,仿射只是它的一阶近似
// ============================================================================================
//
// # 为什么非换不可(不是调参,是把物理写对)
//
// 实测:仿射表在真机上**拟合残差 0.0059(约 4 px)看着没问题,而样本外误差平均 6.3 cm、最差
// 24.8 cm**。而且失败的形状本身指出了根因 —— **误差集中在边缘点上**,中间那几个点很准。
//
// 这正是"用仿射去近似透视"的签名:相机成像有一步**除以深度**(近大远小),仿射把那一步丢了。
// 在标定区中间丢得不明显,越往边上差得越多。加更多点救不了,因为**模型本身少了一项**。
//
// 透视有 8 个自由度(仿射 6 个),多出来的两个正是那一项。四个点恰定 —— 所以这里同样要求
// **≥5 个点**才谈得上自检,理由与 `fit` 那条一模一样。

/// 一张量出来的透视换算表。
#[derive(Copy, Clone, Debug)]
pub struct Homography {
    /// `u = (h[0]x + h[1]y + h[2]) / (h[6]x + h[7]y + 1)`,
    /// `v = (h[3]x + h[4]y + h[5]) / (h[6]x + h[7]y + 1)`
    pub h: [f64; 8],
    /// 拟合残差(归一化画面单位)。**同样不是精度** —— 精度看 [`loo_homography`]。
    pub residual: f64,
    /// 用了几个点。
    pub n: usize,
}

/// 由若干组 `(世界x, 世界y, 画面u, 画面v)` 拟合透视变换。
pub fn fit_homography(pts: &[(f64, f64, f64, f64)]) -> Result<Homography, Bad> {
    if pts.len() < MIN_POINTS {
        return Err(Bad::NotEnoughPoints);
    }
    if pts.iter().any(|p| ![p.0, p.1, p.2, p.3].iter().all(|v| v.is_finite())) {
        return Err(Bad::NotFinite);
    }
    // 法方程 (AᵀA)h = Aᵀb。每个点两行:
    //   [x y 1 0 0 0 −ux −uy] · h = u
    //   [0 0 0 x y 1 −vx −vy] · h = v
    let mut ata = [[0.0f64; 8]; 8];
    let mut atb = [0.0f64; 8];
    for p in pts {
        let (x, y, u, v) = *p;
        let rows = [
            ([x, y, 1.0, 0.0, 0.0, 0.0, -u * x, -u * y], u),
            ([0.0, 0.0, 0.0, x, y, 1.0, -v * x, -v * y], v),
        ];
        for (r, rhs) in rows {
            for i in 0..8 {
                atb[i] += r[i] * rhs;
                for j in 0..8 {
                    ata[i][j] += r[i] * r[j];
                }
            }
        }
    }
    let h = solve8(&mut ata, &mut atb).ok_or(Bad::Degenerate)?;

    let mut ss = 0.0;
    for p in pts {
        let (pu, pv) = apply(&h, p.0, p.1).ok_or(Bad::Degenerate)?;
        ss += (pu - p.2).powi(2) + (pv - p.3).powi(2);
    }
    let dof = (2 * pts.len()).saturating_sub(8).max(1) as f64;
    Ok(Homography { h, residual: (ss / dof).sqrt(), n: pts.len() })
}

/// 世界 → 画面。分母趋零(那条"地平线")时返回 `None`,不给一个爆掉的数。
fn apply(h: &[f64; 8], x: f64, y: f64) -> Option<(f64, f64)> {
    let w = h[6] * x + h[7] * y + 1.0;
    if !w.is_finite() || w.abs() < 1e-9 {
        return None;
    }
    Some(((h[0] * x + h[1] * y + h[2]) / w, (h[3] * x + h[4] * y + h[5]) / w))
}

impl Homography {
    /// 世界 → 画面。
    pub fn to_pixel(&self, x: f64, y: f64) -> Option<(f64, f64)> {
        apply(&self.h, x, y)
    }

    /// 画面 → 世界。把两条方程整理成 2×2 直接解,退化时返回 `None`。
    pub fn to_world(&self, u: f64, v: f64) -> Option<(f64, f64)> {
        if !(u.is_finite() && v.is_finite()) {
            return None;
        }
        let h = &self.h;
        let (a11, a12, b1) = (h[0] - u * h[6], h[1] - u * h[7], u - h[2]);
        let (a21, a22, b2) = (h[3] - v * h[6], h[4] - v * h[7], v - h[5]);
        let det = a11 * a22 - a12 * a21;
        if !det.is_finite() || det.abs() < 1e-12 {
            return None;
        }
        Some(((b1 * a22 - b2 * a12) / det, (a11 * b2 - a21 * b1) / det))
    }
}

/// 留一验证,透视版。理由与 [`leave_one_out`] 一字不差:残差衡量"这批点彼此多自洽",
/// 样本外才是"到了没见过的地方多准"。
pub fn loo_homography(pts: &[(f64, f64, f64, f64)]) -> Result<Vec<f64>, Bad> {
    if pts.len() < MIN_POINTS + 1 {
        return Err(Bad::NotEnoughPoints);
    }
    let mut out = Vec::with_capacity(pts.len());
    for skip in 0..pts.len() {
        let rest: Vec<(f64, f64, f64, f64)> =
            pts.iter().enumerate().filter(|(i, _)| *i != skip).map(|(_, p)| *p).collect();
        let m = fit_homography(&rest)?;
        let (pu, pv) = m.to_pixel(pts[skip].0, pts[skip].1).ok_or(Bad::Degenerate)?;
        out.push(((pu - pts[skip].2).powi(2) + (pv - pts[skip].3).powi(2)).sqrt());
    }
    Ok(out)
}

/// 8×8 高斯-约当,带部分主元。奇异就返回 `None`,不返回一个"看起来像解"的东西。
fn solve8(a: &mut [[f64; 8]; 8], b: &mut [f64; 8]) -> Option<[f64; 8]> {
    for c in 0..8 {
        let mut piv = c;
        for r in c + 1..8 {
            if a[r][c].abs() > a[piv][c].abs() {
                piv = r;
            }
        }
        if !a[piv][c].is_finite() || a[piv][c].abs() < 1e-14 {
            return None;
        }
        a.swap(c, piv);
        b.swap(c, piv);
        let d = a[c][c];
        for j in c..8 {
            a[c][j] /= d;
        }
        b[c] /= d;
        for r in 0..8 {
            if r == c {
                continue;
            }
            let f = a[r][c];
            if f == 0.0 {
                continue;
            }
            for j in c..8 {
                a[r][j] -= f * a[c][j];
            }
            b[r] -= f * b[c];
        }
    }
    Some(*b)
}

#[cfg(test)]
mod homography_tests {
    use super::*;

    /// 造一台**斜着看**的相机:世界平面上的点按真透视投影,分母随位置变化。
    fn truth(x: f64, y: f64) -> (f64, f64) {
        let w = 0.35 * x + 0.9 * y + 1.0;
        ((0.90 * x - 0.30 * y + 0.43) / w, (-0.03 * x - 0.95 * y + 0.18) / w)
    }

    fn grid() -> Vec<(f64, f64, f64, f64)> {
        let mut v = Vec::new();
        for &x in &[0.10, 0.22, 0.35] {
            for &y in &[-0.33, -0.22, -0.10] {
                let (u, vv) = truth(x, y);
                v.push((x, y, u, vv));
            }
        }
        v
    }

    /// 🔴 **对照:同一批点,仿射输、透视赢。** 没有这一条,"换了模型好了"就可能只是碰巧。
    #[test]
    fn on_a_truly_perspective_camera_affine_loses_and_homography_wins() {
        let pts = grid();
        let aff = leave_one_out(&pts).expect("九个点够留一");
        let hom = loo_homography(&pts).expect("九个点够留一");
        let (wa, wh) = (
            aff.iter().cloned().fold(0.0, f64::max),
            hom.iter().cloned().fold(0.0, f64::max),
        );
        assert!(wh < wa / 10.0, "透视应当远好于仿射:仿射最差 {wa:.5} 透视最差 {wh:.5}");
        assert!(wh < 1e-6, "点本来就来自一个透视相机,透视应当几乎精确:{wh}");
    }

    /// 回代还原:画面 → 世界 → 画面,要回到原处。
    #[test]
    fn round_trips_through_world_and_back() {
        let pts = grid();
        let m = fit_homography(&pts).expect("拟合得出");
        for p in &pts {
            let (x, y) = m.to_world(p.2, p.3).expect("逆变换存在");
            assert!((x - p.0).abs() < 1e-6 && (y - p.1).abs() < 1e-6, "{x} {y} vs {} {}", p.0, p.1);
        }
    }

    /// 四个点做不了留一(抽掉一个只剩三点,而透视要四点)—— 与仿射那条同一个道理。
    #[test]
    fn four_points_cannot_be_leave_one_out_checked() {
        // 🔴 必须取**任意三点不共线**的四个点(这里取四角)。随手取前四个会有三点共线,
        // 而共线的四点定不出透视 —— 上面那条 `Degenerate` 正是这么被自己的测试撞出来的。
        let g = grid();
        let pts = vec![g[0], g[2], g[6], g[8]];
        assert!(fit_homography(&pts).is_ok());
        assert_eq!(loo_homography(&pts).unwrap_err(), Bad::NotEnoughPoints);
    }

    /// 点共线 ⇒ 拒绝,而不是给一个沿线之外全靠编的解。
    #[test]
    fn collinear_points_are_refused() {
        let pts: Vec<(f64, f64, f64, f64)> = (0..5)
            .map(|i| {
                let x = 0.10 + 0.05 * i as f64;
                let (u, v) = truth(x, -0.22);
                (x, -0.22, u, v)
            })
            .collect();
        assert_eq!(fit_homography(&pts).unwrap_err(), Bad::Degenerate);
    }
}

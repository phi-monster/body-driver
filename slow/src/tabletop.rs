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
    /// 存盘的表里有一行读不成。**带行号**,因为"表短了"和"第七行坏了"要改的地方不一样。
    BadLine(usize),
}

/// 存盘表里的一行。
///
/// # 这一格为什么存在
///
/// 一次仿真只跑得完两个点,而拟合要 ≥4、留一要 ≥5 ⇒ **标定必须跨集攒**。真机上本来也是这样:
/// 中间可以停、可以换天、可以补点。
///
/// 🔴 而它更要紧的一半是**别把表只存在日志里**。实测过一次(2026-08-14):八个点的标定表只被
/// `println!` 到 `link.log`,而每次启动都覆盖那个文件 —— 于是**在那张表上建的四层东西
/// (拟合 / 留一 / 两把钥匙 / 跨集累积)全都没有原始记录可回溯**,想复查"当初那个点读的是多少"
/// 已经查不到了。表必须落在一个**只追加**的文件里。
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Row {
    /// 手当时所在的世界位置。
    pub xyz: [f64; 3],
    pub u: f64,
    pub v: f64,
    /// 有没有认出那**一对**钳口。⚠️ 决定这一行的**含义**:认出来 = 两指中点,没认出来 = 那一根
    /// 指头的中心,两者差半个钳口宽。**混着用过一次,算出来的比例整整偏了 60%。**
    pub paired: bool,
    /// 这一行自己报的噪声地板。地板高 = 读数器只抓到"越过极高门槛"的碎片,不是指尖。
    pub floor: u8,
}

/// 读存盘的标定表。格式一行七个数:`x y z u v paired floor`。
///
/// 🔴 **读不成的行一律报错,不许跳过。** 追加写有可能在中途断电/被杀,留下半行;悄悄跳过它
/// 会让表**静静地变短**,而"点不够"和"点坏了"在下游长得一模一样。
pub fn parse_table(s: &str) -> Result<Vec<Row>, Bad> {
    let mut out = Vec::new();
    for (i, line) in s.lines().enumerate() {
        let t = line.trim();
        if t.is_empty() || t.starts_with('#') {
            continue;
        }
        let f: Vec<&str> = t.split_whitespace().collect();
        if f.len() != 7 {
            return Err(Bad::BadLine(i + 1));
        }
        let mut n = [0f64; 5];
        for k in 0..5 {
            n[k] = f[k].parse().map_err(|_| Bad::BadLine(i + 1))?;
        }
        let paired: u8 = f[5].parse().map_err(|_| Bad::BadLine(i + 1))?;
        let floor: u8 = f[6].parse().map_err(|_| Bad::BadLine(i + 1))?;
        if !n.iter().all(|x| x.is_finite()) {
            return Err(Bad::BadLine(i + 1));
        }
        out.push(Row { xyz: [n[0], n[1], n[2]], u: n[3], v: n[4], paired: paired != 0, floor });
    }
    Ok(out)
}

/// 只留**同一个高度**上的那些行,返回 `(留下的, 那个高度)`。`tol` 是允许的高度差。
///
/// # 为什么这是一条会失败的检查,而不是一句叮嘱
///
/// 这张表是 `(x,y) → (u,v)` 的,**它只在一个高度上成立**(文件头那段"它诚实地不知道什么"讲
/// 的就是视差)。实测(2026-08-14):走位的三段没闩住,在"下降"和"回抬"之间来回摆,60 步预算
/// 耗尽后代码照常去量 ⇒ 四个点的实到高度是 **1.0976 / 1.0989 / 1.1031 / 1.0250**,而命令的
/// 是 **0.95**。x、y 准到 4 mm,**误差几乎整个在 z 上** —— 而只看 `(x,y,u,v)` 的拟合**永远看不见
/// 这件事**,它会照常给出一张残差不大的表,然后在每个高度不同的地方悄悄偏十几个像素。
///
/// 取**最大的那一簇**,而不是"离中位数近的" —— 一半点在过境高度、一半在桌面高度时,中位数
/// 落在两簇之间,会把两簇都判掉。
pub fn same_height(rows: &[Row], tol: f64) -> (Vec<Row>, f64) {
    let mut best: Vec<Row> = Vec::new();
    let mut best_z = f64::NAN;
    for anchor in rows {
        let c: Vec<Row> = rows.iter().filter(|r| (r.xyz[2] - anchor.xyz[2]).abs() <= tol).copied().collect();
        if c.len() > best.len() {
            best_z = c.iter().map(|r| r.xyz[2]).sum::<f64>() / c.len() as f64;
            best = c;
        }
    }
    (best, best_z)
}

/// 表里有几个**互不相同**的世界点(1 mm 以内算同一个)。
///
/// 🔴 为什么要单独数这个:`fit` 要"≥4 个点"指的是**四个不同的位置**。同一个点重复量八次也是
/// 八行,而它把两条轴一条都分不开 —— 那是 `Degenerate`,不是"够了"。重复行有它自己的用处
/// (量复现度),但**不能拿来充数**。
pub fn distinct_sites(rows: &[Row]) -> usize {
    let mut sites: Vec<[f64; 3]> = Vec::new();
    for r in rows {
        if !sites.iter().any(|s| {
            (s[0] - r.xyz[0]).abs() < 1e-3 && (s[1] - r.xyz[1]).abs() < 1e-3 && (s[2] - r.xyz[2]).abs() < 1e-3
        }) {
            sites.push(r.xyz);
        }
    }
    sites.len()
}

/// 同一个世界位置被量了多次时,**同类**读数之间差多远(归一化画面单位)。这是标定的复现度,
/// 也是这张表的误差下限 —— 表再准也准不过它自己的读数噪声。
///
/// 返回 `(最大差, 参与比较的对数)`。不同类(`paired` 不同)的**不比**,见 `Row::paired`。
pub fn repeatability(rows: &[Row]) -> (f64, usize) {
    let (mut worst, mut n) = (0f64, 0usize);
    for i in 0..rows.len() {
        for j in (i + 1)..rows.len() {
            let (a, b) = (&rows[i], &rows[j]);
            if a.paired != b.paired {
                continue;
            }
            if (a.xyz[0] - b.xyz[0]).abs() < 1e-3
                && (a.xyz[1] - b.xyz[1]).abs() < 1e-3
                && (a.xyz[2] - b.xyz[2]).abs() < 1e-3
            {
                let d = ((a.u - b.u).powi(2) + (a.v - b.v).powi(2)).sqrt();
                if d > worst {
                    worst = d;
                }
                n += 1;
            }
        }
    }
    (worst, n)
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

    /// 🔴 今晚真的存盘内容(`/root/calpairs.txt`,同一个世界点量了三次)。
    /// 它同时钉住三件事:读得出来 · **只有一个不同的位置** · 三次读数里只有两次可比。
    #[test]
    fn the_real_calfile_reads_back_and_says_it_is_only_one_site() {
        let s = "0.22000 -0.22000 0.95000 0.68211 0.33785 1 15\n\
                 0.22000 -0.22000 0.95000 0.68858 0.30155 0 14\n\
                 0.22000 -0.22000 0.95000 0.68992 0.31153 0 51\n";
        let rows = parse_table(s).expect("读得出来");
        assert_eq!(rows.len(), 3);
        assert!(rows[0].paired && !rows[1].paired);
        // 🔴 三行,但**一个位置** ⇒ 拿去拟合是 Degenerate,不是"够了"。
        assert_eq!(distinct_sites(&rows), 1, "三行都在同一个点上");
        // 只有后两行同类,能比的只有那一对。
        let (worst, n) = repeatability(&rows);
        assert_eq!(n, 1, "跨类不许比 ⇒ 三行里只有一对可比");
        assert!((worst - 0.010_08).abs() < 5e-4, "同类两次差 {worst:.5},约 4.9 px@640×480");
    }

    /// 🔴 今晚真的实到高度:命令全是 0.95,实到 1.0976/1.0989/1.1031/1.0250。
    /// 只看 (x,y,u,v) 的拟合**看不见这件事**,所以这一条必须在拟合之前就把它挡下来。
    #[test]
    fn points_measured_at_different_heights_are_split_not_averaged() {
        let s = "0.2998 -0.2843 1.0976 0.90091 0.27304 1 23\n\
                 0.1578 -0.2779 1.0989 0.70000 0.30000 1 20\n\
                 0.2999 -0.1419 1.1031 0.88703 0.09302 1 23\n\
                 0.2117 -0.1996 1.0250 0.68983 0.31090 1 48\n";
        let rows = parse_table(s).unwrap();
        let (keep, z) = same_height(&rows, 0.02);
        assert_eq!(keep.len(), 3, "1.0976/1.0989/1.1031 是一簇,1.0250 差 7.3 cm 不是");
        assert!((z - 1.0999).abs() < 1e-3, "留下那一簇的高度 {z:.4}");
        // 🔴 高度差 7.3 cm 的那个点如果混进来,拟合照样给得出一张表 —— 这就是它危险的地方。
        let all: Vec<_> = rows.iter().map(|r| (r.xyz[0], r.xyz[1], r.u, r.v)).collect();
        assert!(fit(&all).is_ok(), "混着高度也拟合得出来 ⇒ 残差挡不住它 ⇒ 必须先按高度切");
    }

    /// 🔴 半行必须**报错**,不许悄悄跳过 —— 追加写被杀会留下半行,而"表变短了"在下游长得
    /// 跟"点还不够"一模一样。
    #[test]
    fn a_truncated_line_is_refused_with_its_line_number() {
        let s = "0.22 -0.22 0.95 0.68 0.33 1 15\n0.30 -0.20 0.95 0.70\n";
        assert_eq!(parse_table(s), Err(Bad::BadLine(2)));
    }

    /// 同一个位置重复量,**不能拿来充四个点**。
    #[test]
    fn repeats_of_one_site_do_not_count_as_four_points() {
        let s = "0.22 -0.22 0.95 0.681 0.337 1 15\n0.22 -0.22 0.95 0.682 0.338 1 15\n\
                 0.22 -0.22 0.95 0.683 0.339 1 15\n0.22 -0.22 0.95 0.684 0.340 1 15\n";
        let rows = parse_table(s).unwrap();
        assert_eq!(rows.len(), 4);
        assert_eq!(distinct_sites(&rows), 1);
        let pts: Vec<_> = rows.iter().map(|r| (r.xyz[0], r.xyz[1], r.u, r.v)).collect();
        assert_eq!(fit(&pts).err(), Some(Bad::Degenerate), "四行同一点 ⇒ 两条轴一条都分不开");
    }

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

//! ②a 下手点生成器 —— 架构里一直缺的那一格。**零学习,纯几何。**
//!
//! # 它答的一件事
//!
//! 眼前这块表面上,**这具身体**真正能用的下手点有哪些。
//!
//! # 🔴 为什么它是一个【单独的 crate】,不进 body-layer
//!
//! `ARCH.md §二` 把这堵墙写死了:driver 的不变量是**只装这具身体、与眼前是什么无关**。
//! 把"这个物体哪儿能下手"塞进 driver,它就依赖场景,**"换机体不重训"当场作废**。
//! 所以身体常数是**传进来的参数**,不是这里读出来的 —— 这个方向是单向的,反过来就破墙。
//!
//! 它也不属于 ③(眼):它**算得出来**,不用学,也不需要语义。
//!
//! # 它替掉的是【我的手】
//!
//! `push_aug2026` 十几轮返工,每一轮病灶都落在这一格:力臂往哪挪 · 爪面朝哪 ·
//! 挪多少会挪出物体外 —— 全是人在拍脑袋。形状一换就废,那是"把过拟合从模型挪到老师身上"。
//!
//! # ⚠️ 已知限制:开放薄壳
//!
//! 宽度量的是**指头那一条**沿合爪方向的跨度。对**闭合实体**网格这是对的;对一张
//! **零厚度的开放壳**,一条切得与壳面相切的条只看得到一层点,跨度读出来接近零,
//! 于是一个夹不住的大块会被读成"能夹"。真资产是闭合实体,但**这条不是不存在,只是不常见** ——
//! 单元测试里因此不用壳做夹具。
//!
//! # 🔴 包围盒是不够的,这是实测
//!
//! 包围盒说"这一段 6 厘米实心",而剪刀的真身是**两片薄刃夹一条缝** —— 按盒子选出来的下手点,
//! 爪子从缝里合过去,指间什么都没有(实测:合爪停在 3.2 cm,而选中那段"宽 8 cm")。
//! **包围盒没有局部,而抓取只发生在局部。** 所以这里吃的是表面点,不是盒子。

#![forbid(unsafe_code)]

pub mod hands;
pub mod support;

/// 世界坐标里的一个点。
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct P3 {
    /// x, 米
    pub x: f64,
    /// y, 米
    pub y: f64,
    /// z, 米(向上)
    pub z: f64,
}

/// 爪能张多开 —— **它的出处必须跟着它走**。
///
/// 🔴 本仓最贵的一次手填就在这个量上:`GripperSpan` 在这具身体上是**拒绝**状态
/// (官方探针要用相机量爪尖像素间距,那个检测器从没跑过),而代码里一直用着手填的 8 cm。
/// 拿手填的数去判"夹不夹得下",判错了没有任何一个环节会不一致。
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum JawSpan {
    /// 驱动量出来的。只有这一档能被当成事实。
    Measured(f64),
    /// 机体描述文件里声明的。能用,但**每一条结果都要背着"这是声明值"这个标记**。
    Declared(f64),
    /// 既没量到也没声明 ⇒ **不许猜**。整层拒绝出候选。
    Unknown,
}

impl JawSpan {
    fn metres(self) -> Option<f64> {
        match self {
            JawSpan::Measured(v) | JawSpan::Declared(v) if v > 0.0 => Some(v),
            _ => None,
        }
    }
    fn declared(self) -> bool {
        matches!(self, JawSpan::Declared(_))
    }
}

/// 这具身体的尺寸。**全部由驱动量出来后传进来**,这里一个字面量都没有。
#[derive(Copy, Clone, Debug)]
pub struct Body {
    /// 爪张开度,带出处。
    pub jaw: JawSpan,
    /// 够得到的半径下界,米。
    pub reach_lo: f64,
    /// 够得到的半径上界,米。
    pub reach_hi: f64,
    /// 这条臂的根在世界里的 x,米。
    pub base_x: f64,
    /// 这条臂的根在世界里的 y,米。
    pub base_y: f64,
}

/// 一条下手点候选 —— `ARCH.md` 说的"接触集"的**几何那一半**。
///
/// 剩下那一半(要往哪使劲 · 物体要怎么动)由动词和眼睛填,不在这一层。
#[derive(Copy, Clone, Debug)]
pub struct Contact {
    /// 下手点(这一段截面的中心),世界坐标,米。
    pub point: P3,
    /// 合爪方向在水平面里的朝向,弧度。爪面垂直于它。
    pub close_yaw: f64,
    /// 这一段沿合爪方向有多宽,米。
    pub width_m: f64,
    /// 爪张开度减去这一段的宽度,米。**越大越不容易滑掉。**
    pub margin_m: f64,
    /// 这一段的底离支撑面多高,米。**贴着桌面的段抓不了 —— 爪子伸不到它下面。**
    pub above_support_m: f64,
    /// 下手点离这条臂根多远,米。
    pub reach_r: f64,
    /// 在够得到的范围里吗。
    pub reachable: bool,
    /// 🔴 这条候选是不是**用声明值(而非实测值)的爪张开度**排出来的。
    /// 一路传到落盘,别让它在中途消失。
    pub jaw_declared: bool,
    /// 这一段的跨度在爪张开度**以内**吗。
    ///
    /// 🔴 **它只用来排序,永远不用来删候选。** 仓里唯一那条可抓性规矩禁止
    /// *"拿钳口能张多少当阈值去筛物体"*,而 `PRIMITIVE_DATA` 另有一句:
    /// *"任何地方出现一个具体的钳口开度数值,都是手填的,不许拿它当阈值"*。
    /// ⇒ 放不下的段**仍然留在表里**,只是排在最后。谁想忽略这一位,忽略就是了。
    pub within_jaw: bool,
    /// 这一段上取到了几个表面点。太少 = 这个截面本身是噪声。
    pub n_pts: u32,
    /// 🔴 **这块料有多深** —— 沿指头方向,厚度还差不多的那一段有多长,米。
    ///
    /// # 它替掉了"余量最大排最前"
    ///
    /// 旧排序按 `margin_m`(爪张开度 − 宽度)从大到小 ⇒ **最窄的边角条排最前**,
    /// 而**捏一个物体最尖的角恰恰最容易滑脱**。实测(2026-08-12):
    /// 抓取率从整层量法的 **19/48** 掉到局部量法的 **26/96**,与这条一致。
    ///
    /// 深度是"捏得住"在几何上唯一算得出来的代理:指头压在一条**又深又匀**的料上才咬得住;
    /// 压在一个**厚度迅速收敛的尖端**上,接触面积趋零。
    /// ⚠️ 它**不是**"捏得住"本身 —— 摩擦、重心、指面材质都不在几何里。这一格仍然欠着。
    pub depth_m: f64,
    /// 🔴 **两个夹持面离"正对着"歪了多少**,弧度。**越小越不容易滑。**
    ///
    /// # 这就是摩擦锥那一条,写成不需要 μ 的形式
    ///
    /// 力封闭要求两个接触点的连线落在两点各自的摩擦锥内;锥的半角是 `atan(μ)`,
    /// 而连线就是合爪方向。所以条件等价于:**每个接触面的法向与合爪方向的夹角 < atan(μ)**。
    /// 这里量的正是那个夹角 —— 沿指头宽方向,近面/远面的深度随位置变化的斜率取反正切。
    ///
    /// 🔴 **不需要 μ**:μ 只决定"多大算过线",而**排序只需要"谁更小"**。
    /// 把一个没量过的 μ 写进来才是编数;报角度、按角度排,一个常数都不用引。
    /// ⚠️ 它仍然不是"捏得住"本身:指面材质、物体软硬、侧向扰动都不在几何里。
    pub face_tilt_rad: f64,
    /// 🔴 **下手点离这团点云的重心多远**(水平距离,米)。**越小越不容易转出去。**
    ///
    /// 抓在离重心远的地方,一提起来重力对抓点产生力矩,物体绕钳口转,然后滑脱 ——
    /// 渲图 2026-08-13 拍到过:目标是剪刀,抓在**手柄圆环**上,而重量全在刀刃那头。
    /// 摩擦锥管"横着滑走",这一格管"转出去",**是两件事,都得算**。
    pub com_offset_m: f64,
}

/// 交接给接触集时,为什么交不出去。**拒绝要说得出理由。**
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Handoff {
    /// **μ 没给。** 摩擦系数是身体×世界的耦合,量得出来(拿指头在参考面上蹭一下测打滑门槛),
    /// 但**没量过就不许瞎填一个** —— 填大了,一把会滑的抓取会被判成可行。
    MuUnknown,
    /// **这一把会横着滑走。** 两个夹持面歪了 `need_rad`,而摩擦锥只有 `have_rad`。
    WouldSlip { need_rad: f64, have_rad: f64 },
}

/// 为什么一条候选都给不出来。**拒绝要说得出理由**,不许静默返回空表。
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Refusal {
    /// 爪张开度既没量到也没声明 ⇒ "夹不夹得下"这一问无法回答。
    JawSpanUnknown,
    /// 表面点太少,算不出截面。
    TooFewPoints,
    /// 这块表面是平的(高度跨度为零),切不出层。
    Flat,
    /// 一条候选都算不出来 —— 每一层的点都不够,或者每一条的跨度都是零。
    ///
    /// 🔴 **它不再表示"这具身体夹不住这个东西"。** 那个含义在 2026-08-12 被删掉了:
    /// 仓里唯一那条规矩禁止**从数字判可抓性**,而"每一层都比爪子宽 ⇒ 夹不住"正是它。
    NoSection,
}

/// 切多少层、量多少个方向。**都是可观测性参数,不是身体常数** —— 调它们不改变物理,只改变分辨率。
#[derive(Copy, Clone, Debug)]
pub struct Grid {
    /// 水平切几层。
    ///
    /// ⚠️ **层的厚度必须由 `jaw_h_m` 定,不许由"物体高度 ÷ 层数"定** —— 见那个字段。
    /// 这里只决定**在多少个高度上试**,不决定每层多厚。
    pub bands: u32,
    /// 🔴 **爪面本身有多高**,米 —— 一层的厚度就是它。
    ///
    /// # 这是本仓典型的那类错:该由身体定的量,被场景定了
    ///
    /// 早先一层的厚度 = **物体高度 ÷ 层数**,与爪子无关。于是量出来的"这一段有多宽"
    /// 说的是**一片纸那么薄的横截面**,而爪面实际压上去时会碰到它上下几厘米内的所有料。
    ///
    /// 实测(2026-08-12,拿 59 次**真抓起来**的集当尺子):`段宽 ÷ 爪停读数` 的
    /// 中位是 0.0825,而**全距 0.0011 – 0.7516,近 700 倍** —— 一个散成这样的比值
    /// 不是单位换算,它说明**两个量根本不是同一件事**。最刺眼的一行:
    /// 一块**实心积木**被报成 **0.0048 m** 的段,而爪子停在读数 **0.30**。
    ///
    /// ⚠️ 我当天把这件事写成"只影响零厚度的开放薄壳,真资产是闭合实体不吃这个亏" ——
    /// **那句话是错的**,实心网格照样中招,因为切片薄得和爪面无关。
    ///
    /// 🔴 **爪面高度是一个【还没量到】的身体量**(驱动里没有这一格)。现在按参数传进来,
    /// 它一变结果就变 —— 这笔账记在这里,不藏。
    pub jaw_h_m: f64,
    /// 每层量多少个方向(0..π 均分,方向无正负之分)。
    pub dirs: u32,
    /// 一层里至少要几个点才算数。
    pub min_pts: u32,
    /// 🔴 **离支撑面至少要多高才算「爪子伸得进去」**,米。
    ///
    /// # 它是一条【下限】,不是「越高越好」—— 这一条我写错过,而且有实测代价
    ///
    /// 2026-08-12:排序里「离支撑面越高越前」压在「料越深」前面,于是**永远挑最高那一层**。
    /// 同一只鞋,整层量法选的是**鞋腰**(宽 5.4–8.8 cm、离桌 0–3.8 cm,前 8 集抬起 **5**);
    /// 局部量法选的是**鞋口那圈软边**(宽 0.7–3.1 cm、离桌 **6.0–7.6 cm**,前 8 集抬起 **2**)。
    /// 而同一条规则在**锤子**上恰好挑对(最高处是锤头,又厚又实):4/9 → 10/16 → 11/16。
    ///
    /// ⇒ 本意是"别去抓贴着桌面那一层(爪子伸不进去)",那是**下限**;
    ///   写成最大化就变成了"专挑最上面那张软皮"。
    pub min_above_m: f64,
    /// 🔴 指头本身有多宽,米 —— 量宽度只能量**指头覆盖到的那一条**。
    ///
    /// # 这个字段是一次实测逼出来的,不是设计出来的
    ///
    /// 早先这里量的是"这一层沿这个方向从这头到那头有多远"。**那个数不是爪子要跨过的东西。**
    /// 实测(2026-08-12,五种物体 19 个下手):**爪子停在哪 与 那个宽度的相关系数只有 0.256** ——
    /// 爪停值跨了 74% 的行程,而那个"宽度"几乎没动。唯一一次真抓起来的:按声明张开度反推
    /// 实际只夹住 **4.5 mm**,而那个"宽度"报的是 **79 mm**。
    ///
    /// 原因是几何的:一个碗沿、一个空心壳、一个锤头,**整层的跨度**很大,而**指头摸到的地方**
    /// 只有几毫米的料。这就是"包围盒没有局部,而抓取只发生在局部"在下一层重演 ——
    /// 我用切片替掉了包围盒,却仍然在量"整体"。
    ///
    /// ⚠️ **这是一个身体量,而驱动还没量到它**(`GripperSpan` 都还是拒绝态)。
    /// 现在按参数传进来,并且它一变结果就变 —— 这笔账记在这里,不藏。
    pub finger_w_m: f64,
    /// 🔴 两个表面点隔多远就不算同一块料了,米。
    ///
    /// **它是采样密度的函数,不是身体常数。** 存在的理由是一个实测出来的坑:
    /// 把一层里所有点直接取最小/最大投影,**会在两块分开的料之间算出一个悬空的下手点** ——
    /// 相距 1.2 m 的两根杆,沿某个方向看"这一层只有 2 cm 宽",而那 2 cm 的中心在半空中。
    /// 单元测试逮到的,不是推想的。摄像头给的点云里从来不会只有一个物体 ⇒ 必须先分块。
    pub gap_m: f64,
}

impl Default for Grid {
    fn default() -> Self {
        Grid { bands: 6, dirs: 16, min_pts: 6, min_above_m: 0.005, jaw_h_m: 0.03, finger_w_m: 0.02, gap_m: 0.01 }
    }
}

/// 把一层里的点按"挨着不挨着"分块(单链)。**下手点只能落在一块连着的料上。**
fn clusters(band: &[&P3], gap: f64) -> Vec<Vec<usize>> {
    let n = band.len();
    let mut owner: Vec<usize> = (0..n).collect();
    fn find(o: &mut Vec<usize>, mut i: usize) -> usize {
        while o[i] != i {
            o[i] = o[o[i]];
            i = o[i];
        }
        i
    }
    let g2 = gap * gap;
    for i in 0..n {
        for j in (i + 1)..n {
            let dx = band[i].x - band[j].x;
            let dy = band[i].y - band[j].y;
            if dx * dx + dy * dy <= g2 {
                let (a, b) = (find(&mut owner, i), find(&mut owner, j));
                if a != b {
                    owner[a] = b;
                }
            }
        }
    }
    let mut by_root: std::collections::BTreeMap<usize, Vec<usize>> = Default::default();
    for i in 0..n {
        let r = find(&mut owner, i);
        by_root.entry(r).or_default().push(i);
    }
    by_root.into_values().collect()
}

/// 主入口:一堆表面点 + 这具身体 → 能用的下手点,已排序。
///
/// 排序:**够得到的排前面 → 离支撑面越高越前 → 余量越大越前。**
/// 为什么是这个序:够不到的一条也用不上;贴着桌面的段爪子伸不进去;余量小的容易滑。
pub fn candidates(
    pts: &[P3],
    body: &Body,
    support_z: f64,
    grid: Grid,
) -> Result<Vec<Contact>, Refusal> {
    let span = body.jaw.metres().ok_or(Refusal::JawSpanUnknown)?;
    if pts.len() < 8 {
        return Err(Refusal::TooFewPoints);
    }
    let (z0, z1) = pts.iter().fold((f64::MAX, f64::MIN), |(a, b), p| (a.min(p.z), b.max(p.z)));
    // 这团点云的水平重心 —— 抓点离它越远,提起来越容易绕钳口转出去。
    // ⚠️ 这是**表面点的质心**,不是真重心(密度不均时两者不同)。几何能给的只有这个,
    //    而它已经足以把"抓在剪刀手柄圆环上"这类排到后面去。
    let (com_x, com_y) = {
        let n = pts.len() as f64;
        (pts.iter().map(|p| p.x).sum::<f64>() / n, pts.iter().map(|p| p.y).sum::<f64>() / n)
    };
    if z1 - z0 < 1e-4 {
        return Err(Refusal::Flat);
    }

    let mut out: Vec<Contact> = Vec::new();
    for bi in 0..grid.bands.max(1) {
        // 🔴 层的**中心**在物体高度上均匀铺开,而层的**厚度**是爪面高度 ——
        //    厚度归身体,位置归场景。早先厚度也用物体高度算,那是把身体量交给了场景。
        let c = z0 + (z1 - z0) * (bi as f64 + 0.5) / (grid.bands.max(1) as f64);
        let h = grid.jaw_h_m.max(1e-4) / 2.0;
        let (lo, hi) = (c - h, c + h);
        let band: Vec<&P3> = pts.iter().filter(|p| p.z >= lo && p.z <= hi).collect();
        if band.len() < grid.min_pts as usize {
            continue;
        }
        // 🔴 先分块再量宽度。不分块的话,两块分开的料之间会算出一个悬空的下手点。
        for cl in clusters(&band, grid.gap_m) {
            if cl.len() < grid.min_pts as usize {
                continue;
            }
            let n = cl.len() as f64;
            let cx = cl.iter().map(|&i| band[i].x).sum::<f64>() / n;
            let cy = cl.iter().map(|&i| band[i].y).sum::<f64>() / n;

            let _ = (cx, cy);
            for di in 0..grid.dirs.max(1) {
                let th = core::f64::consts::PI * (di as f64) / (grid.dirs.max(1) as f64);
                let (c, s) = (th.cos(), th.sin());
                // n = 合爪方向;t = 与它垂直的水平方向(指头沿 t 铺开)
                let (nx, ny) = (c, s);
                let (tx, ty) = (-s, c);
                // 🔴 沿 t 切成【指头宽】的条,**逐条**量沿 n 的跨度。
                //    这才是爪子真要跨过的东西:指头只覆盖它自己那一条。
                let (mut t_lo, mut t_hi) = (f64::MAX, f64::MIN);
                for &i in &cl {
                    let u = band[i].x * tx + band[i].y * ty;
                    t_lo = t_lo.min(u);
                    t_hi = t_hi.max(u);
                }
                let fw = grid.finger_w_m.max(1e-4);
                let n_strip = (((t_hi - t_lo) / fw).ceil() as i64).max(1);
                for si in 0..n_strip {
                    let (a, b) = (t_lo + fw * si as f64, t_lo + fw * (si + 1) as f64);
                    let (mut mn, mut mx) = (f64::MAX, f64::MIN);
                    let (mut sx2, mut sy2, mut k) = (0.0f64, 0.0f64, 0u32);
                    for &i in &cl {
                        let u = band[i].x * tx + band[i].y * ty;
                        if u < a || u > b {
                            continue;
                        }
                        let q = band[i].x * nx + band[i].y * ny;
                        mn = mn.min(q);
                        mx = mx.max(q);
                        sx2 += band[i].x;
                        sy2 += band[i].y;
                        k += 1;
                    }
                    if k < grid.min_pts {
                        continue;
                    }
                    let width = mx - mn;
                    if width <= 0.0 {
                        continue;
                    }
                    // 🔴🔴 **这里【不许】因为"太宽"而丢掉一条候选。**
                    //
                    // 仓里唯一那条关于可抓性的规矩(`PRIMITIVE_DATA` §物体库):
                    //   *"可抓性只能【看图】判,不许从任何数字判"*;
                    //   明确点名禁止 *"拿【钳口能张多少】当阈值去筛物体"*。
                    //   同一个错本仓犯过三次(按尺寸判 17.9% 夹不住 · 按张开度报"可用率 8%" ·
                    //   按外接盒判 21 集"物理上不可能"),三次都是同一句话:
                    //   **把工具的边界当成世界的边界。**
                    //
                    // 而**合法的用法写在 `README`**:*"它只读一件事:钳口下面那一小段沿闭合方向
                    // 有多宽,然后把腕转到窄的方向去夹"* —— **宽度用来【排序】,不是用来【否决】。**
                    //
                    // 我 2026-08-12 把 `width >= span => continue` 写了进来,那正是被作废的那条,
                    // 而同一天仓里刚有一个提交在删文档里所有这类诱因。已删。
                    // 🔴 深度:向两侧数,厚度还在同一档(±25%)的邻条能连多长。
                    //    尖端的厚度一条一个样,连不起来;一根杆 / 一条边则连很长。
                    let mut depth = fw;
                    for dir in [-1i64, 1i64] {
                        let mut j = si + dir;
                        while j >= 0 && j < n_strip {
                            let (a2, b2) = (t_lo + fw * j as f64, t_lo + fw * (j + 1) as f64);
                            let (mut n2, mut x2) = (f64::MAX, f64::MIN);
                            let mut c2 = 0u32;
                            for &i in &cl {
                                let u = band[i].x * tx + band[i].y * ty;
                                if u < a2 || u > b2 {
                                    continue;
                                }
                                let q = band[i].x * nx + band[i].y * ny;
                                n2 = n2.min(q);
                                x2 = x2.max(q);
                                c2 += 1;
                            }
                            if c2 < grid.min_pts {
                                break;
                            }
                            let w2 = x2 - n2;
                            if w2 <= 0.0 || (w2 - width).abs() > 0.25 * width {
                                break;
                            }
                            depth += fw;
                            j += dir;
                        }
                    }
                    // 🔴 **两个夹持面歪多少** —— 摩擦锥那一条,写成不需要 μ 的形式。
                    //    沿指头宽方向把这一条再切两半,看近面/远面的深度差多少:
                    //    面正对着爪 ⇒ 两半的深度一样 ⇒ 斜率 0;面是个楔子/尖端 ⇒ 斜率大。
                    //    夹角 = atan(斜率),而**排序只需要"谁更小",所以一个 μ 都不用引**。
                    let half = 0.5 * (a + b);
                    let (mut n_lo, mut x_lo, mut c_lo) = (f64::MAX, f64::MIN, 0u32);
                    let (mut n_hi, mut x_hi, mut c_hi) = (f64::MAX, f64::MIN, 0u32);
                    for &i in &cl {
                        let u = band[i].x * tx + band[i].y * ty;
                        if u < a || u > b {
                            continue;
                        }
                        let q = band[i].x * nx + band[i].y * ny;
                        if u < half {
                            n_lo = n_lo.min(q);
                            x_lo = x_lo.max(q);
                            c_lo += 1;
                        } else {
                            n_hi = n_hi.min(q);
                            x_hi = x_hi.max(q);
                            c_hi += 1;
                        }
                    }
                    let tilt = if c_lo == 0 || c_hi == 0 {
                        // 半条上没有点 ⇒ 量不出斜率。**不许当成 0(那是"完美正对")** ——
                        // 拿不到值必须倒向不利的那一边,给一个直角。
                        core::f64::consts::FRAC_PI_2
                    } else {
                        let run = 0.5 * fw;
                        let d_near = ((n_hi - n_lo) / run).atan().abs();
                        let d_far = ((x_hi - x_lo) / run).atan().abs();
                        d_near.max(d_far)
                    };
                    let (px, py) = (sx2 / k as f64, sy2 / k as f64);
                    let r = ((px - body.base_x).powi(2) + (py - body.base_y).powi(2)).sqrt();
                    out.push(Contact {
                        point: P3 { x: px, y: py, z: 0.5 * (lo + hi) },
                        // 爪面垂直于合爪方向
                        close_yaw: th + core::f64::consts::FRAC_PI_2,
                        width_m: width,
                        margin_m: span - width,
                        above_support_m: lo.max(z0) - support_z,
                        reach_r: r,
                        reachable: r >= body.reach_lo && r <= body.reach_hi,
                        jaw_declared: body.jaw.declared(),
                        within_jaw: width < span,
                        n_pts: k,
                        depth_m: depth,
                        face_tilt_rad: tilt,
                        com_offset_m: ((px - com_x).powi(2) + (py - com_y).powi(2)).sqrt(),
                    });
                }
            }
        }
    }
    if out.is_empty() {
        return Err(Refusal::NoSection);
    }
    // 排序:够得到 → 跨度在爪张开度以内 → 离桌面够高(下限)→ 料越深越前。
    //
    // 🔴🔴 第二项是**排序**,不是**否决** —— 放不下的段仍然在表里,只是垫底。
    //    仓里唯一那条可抓性规矩禁的是"拿钳口张开度**当阈值筛掉**",不是禁止把它当**偏好**。
    //    我 2026-08-12 一度写成 `width >= span => continue`(直接丢掉),那才是被作废的那条,已删。
    //
    // 🔴 "越窄越前"是 `README` 里那个合法用法:*"把腕转到窄的方向去夹"* ——
    //    它是**排序**,不是**否决**。最窄的那条**永远还在表里**,只是排在同深的后面。
    // 🔴 深度优先于窄:2026-08-12 实测,单按"越窄越前"会把**最尖的角**排最前,
    //    而那是最容易滑脱的抓法(抓取率 19/48 → 26/96)。
    // 门槛用中位数,先算出来再排序。
    let med = |mut v: Vec<f64>| -> f64 {
        if v.is_empty() {
            return f64::MAX;
        }
        v.sort_by(f64::total_cmp);
        v[v.len() / 2]
    };
    let tilt_med = med(out.iter().map(|k| k.face_tilt_rad).collect());
    let com_med = med(out.iter().map(|k| k.com_offset_m).collect());
    out.sort_by(|a, b| {
        let off = |k: &Contact| k.above_support_m >= grid.min_above_m;
        let tilt_ok = |k: &Contact| k.face_tilt_rad <= tilt_med;
        let com_ok = |k: &Contact| k.com_offset_m <= com_med;
        // 最后一项是**料有多深**。
        //
        // 🔴 **末位排序键还没有实测撑着,这里照实记着,不许当成定论。**
        // 我 2026-08-12 在两个人造夹具上反复改过三版,每版都是过一个测试、挂另一个:
        //   ① `深 → 窄` :深度打平时 **4 mm 的薄片赢了 2 cm 的杆**;
        //   ② `宽 x 深`   :12 cm 方块上一条 **8.3 cm 宽、只有一条深**的斜切赢了那根杆
        //                   —— 而 8.3 cm 已经贴着爪子极限 0.088,是最没有余量的抓法;
        //   ③ 现在这版 `只按深`:理由是**指面接触到的料只沿指头方向延伸,宽度是要跨过去的空档,
        //      本身不增加接触面积**。
        // ⇒ 三版都讲得通,而**能分出胜负的只有真机数据**,不是再造一个夹具。
        //   在拿到数据之前不再动这一项 —— 继续在夹具上拧就是没有判据地拧旋钮。
        //
        // 「把腕转到窄的方向去夹」(`README`)说的是**在同一处选朝向**,不是在不同地方之间选;
        // 我一度把这两件事混成同一条排序键,那是错的。
        b.reachable
            .cmp(&a.reachable)
            .then(b.within_jaw.cmp(&a.within_jaw))
            // 🔴 离桌面高是**下限**(伸得进去 / 伸不进去),不是"越高越前"。见 `Grid::min_above_m`。
            .then(off(b).cmp(&off(a)))
            // 🔴 **2026-08-13 新增两项,顺序就是它们各自管的物理分量:**
            //
            // ① `face_tilt_rad` —— **管"会不会横着滑走"**。两个夹持面越正对着爪,
            //    接触法向越贴合爪方向,越接近力封闭。这是摩擦锥那一条,写成不需要 μ 的形式:
            //    μ 只决定"多大算过线",而**排序只要"谁更小"**,所以一个常数都不用引。
            //
            // ② `com_offset_m` —— **管"会不会转出去"**。抓点离重心越远,提起来重力的力矩
            //    越大。渲图 2026-08-13 拍到过:抓剪刀抓在**手柄圆环**上,重量全在刀刃那头,
            //    爪子确实夹住了(停在 1.37 cm),一抬就脱手。
            //
            // 🔴 **两件事,不是一件** —— 只做前者抓不起长条形物体。
            // ⚠️ 仍然按本仓唯一那条可抓性规矩:**只排序,一条候选都不删。**
            // 🔴 **门槛,不是最大化** —— 写成最大化会一票压死后面所有项。
            //    这条学费本仓已经付过一次(「离桌面高」写成最大化 ⇒ 专挑最上面那层软皮),
            //    我在这里又犯了一遍:实测排第一的深度掉到一条指头宽 = 一片孤零零的薄片,
            //    因为规则网格上它的面恰好"完美正对",而离散化噪声就足以让它赢。
            //    ⇒ 面歪不歪、离重心远不远都只当**过线/不过线**,细分仍交给"料有多深"。
            // 🔴 门槛由**这一批候选自己的中位数**给,不是我拍一个角度 ——
            //    μ 没量过,拍一个 atan(μ) 就是编数;而"比这批里一半的候选更正"不用任何常数。
            .then(tilt_ok(b).cmp(&tilt_ok(a)))
            .then(com_ok(b).cmp(&com_ok(a)))
            .then(b.depth_m.total_cmp(&a.depth_m))
    });
    Ok(out)
}

/// **爪子停在这儿,这块料多厚?** —— 与 [`candidates`] 相反的方向:那个是"我该去哪",
/// 这个是"我到底停在了哪、那儿有什么"。
///
/// # 🔴 为什么必须有这一支
///
/// 标定爪张开度需要一对 (爪停值, 那儿的料厚)。第一次采集拿 [`candidates`] 报的宽度当料厚,
/// **对子结构上就不成立** —— 实测(2026-08-12):同一个爪停位置 0.30,积木料厚 **0.0048**、
/// 扳手 **0.0351**,**差 7 倍**。因为 [`candidates`] 说的是**打算夹的那一段**,
/// 而爪子实际停在别处(下降有误差、物体会动)。
///
/// # 🔴 它量的是"从外面合过来会碰到什么",不是"我以为夹住了什么"
///
/// 指头从两侧**外面**进来,各自停在第一块碰到的料上 ⇒ 这里报的是那一条上**材料的外缘跨度**。
/// 剪刀两片刃相距 7 cm 时,在一片刃上横着合爪跨的是 **7.9 cm(两片刃的外缘之间)**,
/// **不是单片刃的 9 mm** —— 除非指头能先钻进两刃中间的缝,而那取决于**进场时爪子张多开**,
/// 不取决于几何。这一条我在单元测试里写错过三次。
///
/// 参数是**爪子的最终位姿**:`px/py` 是接触点在水平面里的位置,`pz` 是高度,
/// `close_yaw` 是爪面朝向(与 [`Contact::close_yaw`] 同一个约定)。
/// `band_h_m` 是取多厚的一层(爪子有高度,不是一个数学平面)。
pub fn thickness_at(
    pts: &[P3],
    px: f64,
    py: f64,
    pz: f64,
    close_yaw: f64,
    band_h_m: f64,
    finger_w_m: f64,
) -> Option<f64> {
    if pts.len() < 4 {
        return None;
    }
    // close_yaw 是爪面朝向;合爪方向与它垂直。
    let th = close_yaw - core::f64::consts::FRAC_PI_2;
    let (nx, ny) = (th.cos(), th.sin());
    let (tx, ty) = (-th.sin(), th.cos());
    let (u0, hw) = (px * tx + py * ty, finger_w_m.max(1e-4) / 2.0);
    let hh = band_h_m.max(1e-4) / 2.0;
    let (mut mn, mut mx, mut k) = (f64::MAX, f64::MIN, 0u32);
    for q in pts {
        if (q.z - pz).abs() > hh {
            continue;
        }
        if (q.x * tx + q.y * ty - u0).abs() > hw {
            continue;
        }
        let v = q.x * nx + q.y * ny;
        mn = mn.min(v);
        mx = mx.max(v);
        k += 1;
    }
    // 🔴 那儿根本没有料 ⇒ `None`,**不是 0**。合爪合到空气里和夹住一片薄刃是两件事,
    //    读成 0 就把"指间什么都没有"当成了"夹住了零毫米"。
    if k < 3 {
        return None;
    }
    Some(mx - mn)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn body(jaw: JawSpan) -> Body {
        Body { jaw, reach_lo: 0.15, reach_hi: 0.75, base_x: 0.0, base_y: 0.0 }
    }

    /// 一根竖着的**实心**方杆,按 5 mm 采样。
    ///
    /// 🔴 这个夹具改过两次,两次都是**夹具假**而不是算法错:
    /// ① 最早只放四个角点 ⇒ 分块把它拆成四堆各一个点,一条候选都出不来。真实点云的相邻点
    ///    间距远小于"分块距离",是连通的。
    /// ② 然后只放四个**侧面** ⇒ 那是一张**零厚度的壳**,而按"指头那一条"量宽度时,
    ///    一条切得与壳面相切的条会读到一个很小的跨度,把 12 cm 的大块读成"能夹"。
    ///    真网格是闭合实体,不会这样;但**开放薄壳会**,这条限制写在 `candidates` 的文档里。
    fn rod(w: f64, h: f64, at_x: f64) -> Vec<P3> {
        let step = 0.005_f64;
        let n = ((w / step).round() as i32).max(2);
        let mut v = Vec::new();
        for i in 0..40 {
            let z = (i as f64 / 39.0) * h;
            for a in 0..=n {
                for b in 0..=n {
                    let dx = -w / 2.0 + w * (a as f64) / (n as f64);
                    let dy = -w / 2.0 + w * (b as f64) / (n as f64);
                    v.push(P3 { x: at_x + dx, y: dy, z });
                }
            }
        }
        v
    }

    #[test]
    fn thickness_at_reads_the_material_where_the_jaw_actually_stopped() {
        // 剪刀:两片 9 mm 厚的刃,相距 7 cm。
        let v = scissors();
        // 一片刃:沿 x 长 3 cm,沿 y 厚 9 mm。
        // 约定与 `Contact::close_yaw` 一致:合爪方向 = close_yaw 减 90 度。
        // 要夹住 9 mm 那一维 ⇒ 合爪方向沿 y ⇒ close_yaw = π。
        let across = core::f64::consts::PI;
        let along = core::f64::consts::FRAC_PI_2;
        // 🔴 这条断言我写错过三次,三次都是同一个毛病:**把"我以为爪子夹住什么"当成了
        //    "爪子从外面合过来会碰到什么"**。这里量的是后者,而后者才是物理。
        //
        //    指头从两侧【外面】进来,各自停在第一块碰到的料上。所以在一片刃上沿 y 合爪,
        //    跨的是**两片刃的外缘之间** = 7.9 cm,**不是单片刃的 9 mm** ——
        //    除非指头能先钻进两刃中间的缝里,而那取决于进场时爪子张多开,不取决于几何。
        let t = thickness_at(&v, 0.4, -0.035, 0.02, across, 0.01, 0.02).expect("刃上有料");
        assert!((t - 0.079).abs() < 0.004, "从外面合过来该跨两片刃 7.9 cm,读到 {t}");
        // 沿刃的长边合爪:同一条上只有那一片刃,跨 3 cm。
        let t2 = thickness_at(&v, 0.4, -0.035, 0.02, along, 0.01, 0.02).expect("同一处有料");
        assert!((t2 - 0.030).abs() < 0.004, "顺着夹该读到 3 cm,读到 {t2}");
        // 🔴 站在两片刃【中间的缝】上,读到的仍然是 7.9 cm —— **同一个道理的第四次**:
        //    "爪子的中心在缝里"不等于"指头在缝里"。指头还是从外面来的,照样跨两片刃。
        let t3 = thickness_at(&v, 0.4, 0.0, 0.02, across, 0.01, 0.02).expect("这一条上有料");
        assert!((t3 - 0.079).abs() < 0.004, "站在缝上照样跨两片刃,读到 {t3}");
        // 真正该是 None 的是这一档:**那一条上根本没有料**。
        //    这时必须返回"没有"而不是 0 —— 合爪合到空气里,和夹住零毫米,是两件事。
        assert!(thickness_at(&v, 1.0, 0.0, 0.02, across, 0.01, 0.02).is_none());
        // 高度不对(物体在 z=0.01..0.03,这里问 z=0.5)同样是"没有料"。
        assert!(thickness_at(&v, 0.4, -0.035, 0.5, across, 0.01, 0.02).is_none());
    }

    /// 两片 9 mm 厚的刃,相距 7 cm,实心采样。
    fn scissors() -> Vec<P3> {
        let mut v = Vec::new();
        for i in 0..60 {
            let z = 0.01 + (i as f64 / 59.0) * 0.02;
            for blade in [-0.035_f64, 0.035] {
                for a in 0..6 {
                    for b in 0..4 {
                        v.push(P3 {
                            x: 0.4 - 0.015 + 0.03 * (a as f64) / 5.0,
                            y: blade - 0.0045 + 0.009 * (b as f64) / 3.0,
                            z,
                        });
                    }
                }
            }
        }
        v
    }

    #[test]
    fn height_above_the_table_is_a_floor_not_a_maximiser() {
        // 🔴 这条钉住 2026-08-12 那个实测出来的排序错。
        //    「离桌面越高越前」压在「料越深」前面 ⇒ **永远挑最高那一层**。
        //    同一只鞋:整层量法选鞋腰(宽 5.4–8.8 cm,离桌 0–3.8 cm,前 8 集抬起 5);
        //    局部量法选鞋口那圈软边(宽 0.7–3.1 cm,离桌 6.0–7.6 cm,前 8 集抬起 2)。
        //    而同一条规则在锤子上恰好挑对(最高处是锤头)⇒ 它是 bug,不是"这条线不行"。
        //
        // 场景:一根**又深又匀**的杆(矮)+ 一片**又高又薄**的东西(高)。
        // 两者都离桌面够高 ⇒ 下限这一关都过 ⇒ 该由**深度**决定,而不是谁更高。
        // 🔴 高的那个做成又高又**浅**的一根细刺(沿指头方向只有一条),
        //    这样这条测的就是【下限 vs 最大化】本身,而不是末位排序键的平局。
        let mut pts = rod(0.02, 0.10, 0.35); // 杆:0–0.10 m,深
        for i in 0..40 {
            let z = 0.14 + (i as f64 / 39.0) * 0.06; // 细刺:0.14–0.20 m
            for a in 0..3 {
                for b in 0..3 {
                    pts.push(P3 {
                        x: 0.35 + 0.004 * (a as f64) / 2.0 - 0.002,
                        y: 0.004 * (b as f64) / 2.0 - 0.002,
                        z,
                    });
                }
            }
        }
        let c = candidates(&pts, &body(JawSpan::Measured(0.088)), 0.0, Grid::default())
            .expect("两处都有候选");
        let first = &c[0];
        // 排第一的必须是**杆**(深),不是那片又高又薄的
        assert!(
            first.point.z < 0.12,
            "又挑了最高那一层:z={} 宽={} 深={}",
            first.point.z,
            first.width_m,
            first.depth_m
        );
        assert!(first.depth_m > Grid::default().finger_w_m, "排第一的是孤零零一条");
        // 而"贴着桌面"那一档仍然要排在后面 —— 下限本身没被取消
        assert!(first.above_support_m >= Grid::default().min_above_m, "排第一的贴着桌面");

    }

    #[test]
    /// 🔴 **抓点离重心远的,必须排在后面** —— 这一格管的是"提起来会不会转出去",
    /// 与摩擦锥管的"横着滑走"是两件事。
    ///
    /// 夹具照着 2026-08-13 渲图拍到的那次失败造:一根长条,重量都在一头(点多),
    /// 另一头细(点少)—— 就是"剪刀:抓在手柄圆环上,刀刃那头才是重的"。
    #[test]
    fn a_grip_far_from_the_centre_of_mass_ranks_lower() {
        // 粗的一段(重心在这儿)+ 细长的一段(手柄)
        let mut pts = rod(0.03, 0.10, 0.30);
        pts.extend(rod(0.03, 0.10, 0.42));
        let c = candidates(&pts, &body(JawSpan::Measured(0.088)), 0.0, Grid::default())
            .expect("两段都能碰");
        let com_x: f64 = pts.iter().map(|p| p.x).sum::<f64>() / pts.len() as f64;
        // 排第一的离重心,必须不比排最后的远。**这就是这一格存在的全部理由。**
        let first = (c[0].point.x - com_x).abs();
        let last = (c[c.len() - 1].point.x - com_x).abs();
        assert!(
            first <= last,
            "排第一的反而离重心更远:first={first:.4} last={last:.4}"
        );
        // 而且这一格必须真的被算了 —— 全是 0 说明它根本没接上。
        assert!(
            c.iter().any(|k| k.com_offset_m > 1e-6),
            "com_offset 全是 0 ⇒ 这一格没接上"
        );
        assert!(
            c.iter().any(|k| k.face_tilt_rad > 0.0),
            "face_tilt 全是 0 ⇒ 这一格没接上"
        );
    }

    #[test]
    fn a_deep_even_grip_outranks_a_sharp_corner() {
        // 🔴 这条钉住 2026-08-12 换排序的理由。
        //    旧排序按"余量最大"(= 最窄)⇒ **把最尖的角排最前**,而那最容易滑脱。
        //    实测抓取率 19/48 → 26/96 与此一致。现在按"料有多深"排。
        //
        // ⚠️ 夹具改过一次:最早拿 12 cm 实心块当场景,而**那块上根本没有"又深又匀"的抓法**
        //    (每一条能夹的都是角上的薄片)⇒ 断言的前提不成立。现在场景里两样都有:
        //    一根 2 cm 见方的杆(深且匀)+ 一块 12 cm 的大块(只有角能捏),同高、都够得到。
        let mut pts = rod(0.02, 0.2, 0.35); // 杆
        pts.extend(rod(0.12, 0.2, 0.60)); // 大块
        let c = candidates(&pts, &body(JawSpan::Measured(0.088)), 0.0, Grid::default())
            .expect("两处都有能碰的地方");
        let first = &c[0];
        // 排第一的必须**在爪张开度以内**,且落在杆上 —— 大块的宽面虽然更深,但垫底
        assert!(first.within_jaw, "排第一的段爪子放不下:宽={}", first.width_m);
        assert!(
            (first.point.x - 0.35).abs() < 0.03,
            "排第一的没落在杆上:x={} 宽={} 深={}",
            first.point.x,
            first.width_m,
            first.depth_m
        );
        // 而且它必须**不是孤零零一条** —— 深度严格大于一条指头宽,说明两侧还有同样厚的料。
        // ⚠️ 这里不能要求更深:2 cm 的杆按 2 cm 一条切,最多就是两条 = 0.04。
        //    要求 >0.04 是拿一个夹具给不出的数当判据 —— 我写错过一次。
        assert!(
            first.depth_m > Grid::default().finger_w_m,
            "排第一的是孤零零一条(深度 {}),那就是个尖",
            first.depth_m
        );
        // 大块角上那些薄片必须排在后面
        let corner = c.iter().find(|k| (k.point.x - 0.60).abs() < 0.06);
        if let Some(k) = corner {
            assert!(k.depth_m <= first.depth_m, "角上的片比杆还深?{} vs {}", k.depth_m, first.depth_m);
        }
    }

    #[test]
    fn an_unmeasured_jaw_refuses_instead_of_guessing() {
        // 🔴 这一条是这一层最重要的测试。爪张开度在这具身体上是【拒绝】状态,
        //    而代码里一直用着手填的 8 cm。宁可一条候选都不给,也不许猜一个数出来。
        let e = candidates(&rod(0.02, 0.2, 0.4), &body(JawSpan::Unknown), 0.0, Grid::default());
        assert_eq!(e.unwrap_err(), Refusal::JawSpanUnknown);
    }

    #[test]
    fn declared_jaw_works_but_every_candidate_carries_the_stamp() {
        let c = candidates(&rod(0.02, 0.2, 0.4), &body(JawSpan::Declared(0.088)), 0.0, Grid::default())
            .expect("细杆该有候选");
        assert!(!c.is_empty());
        // 出处不许在中途消失 —— 落盘的每一条都得能看出它是拿声明值判的。
        assert!(c.iter().all(|x| x.jaw_declared));
    }

    #[test]
    fn a_wide_section_is_ranked_last_never_deleted() {
        // 🔴 这条断言**改过三次**,而这一次改的是【规矩】不是数字:
        //    仓里唯一那条可抓性规矩禁止*"拿钳口能张多少当阈值去筛物体"*,
        //    而我一度写成 `width >= span => continue` —— 直接把段丢掉,正是被作废的那条。
        //    现在:放不下的段**仍然在表里**,只是 `within_jaw=false` 并排在最后。
        let c = candidates(&rod(0.12, 0.2, 0.4), &body(JawSpan::Measured(0.088)), 0.0, Grid::default())
            .expect("12 cm 的方块也有能碰的地方");
        assert!(c.iter().any(|k| !k.within_jaw), "放不下的段被删掉了 —— 那是被作废的做法");
        assert!(c.iter().any(|k| k.within_jaw), "角上那些能夹的段该在");
        // 能夹的排在前,放不下的垫底
        let first_bad = c.iter().position(|k| !k.within_jaw).unwrap();
        let last_good = c.iter().rposition(|k| k.within_jaw).unwrap();
        assert!(last_good < first_bad, "排序没把放不下的段排到后面");
    }

    #[test]
    fn two_separate_objects_never_produce_a_contact_point_in_mid_air() {
        // 🔴 这一条是单元测试自己逮出来的真 bug,不是设想的。
        //    一层里所有点直接取最小/最大投影 ⇒ 相距 1.2 m 的两根杆,沿 y 看"只有 2 cm 宽",
        //    而那 2 cm 的中心在**半空中**(x≈1.0,两根杆之间什么都没有)。
        //    摄像头给的点云里从来不会只有一个物体 ⇒ 必须先按"挨着不挨着"分块。
        let mut pts = rod(0.02, 0.2, 0.4);
        pts.extend(rod(0.02, 0.2, 1.6));
        let c = candidates(&pts, &body(JawSpan::Measured(0.088)), 0.0, Grid::default()).unwrap();
        for x in &c {
            let on_a_rod = (x.point.x - 0.4).abs() < 0.05 || (x.point.x - 1.6).abs() < 0.05;
            assert!(on_a_rod, "下手点落在了半空:x={}", x.point.x);
        }
    }

    #[test]
    fn out_of_reach_sinks_below_reachable_ones() {
        let mut pts = rod(0.02, 0.2, 0.4); // 够得到
        pts.extend(rod(0.02, 0.2, 1.6)); // 够不到
        let c = candidates(&pts, &body(JawSpan::Measured(0.088)), 0.0, Grid::default()).unwrap();
        assert!(c[0].reachable, "排第一的必须是够得到的");
        assert!(c.iter().any(|x| !x.reachable), "够不到的也要留在表里,让上面看得见");
    }

    #[test]
    fn flat_on_the_table_ranks_below_higher_sections() {
        let c = candidates(&rod(0.02, 0.2, 0.4), &body(JawSpan::Measured(0.088)), 0.0, Grid::default()).unwrap();
        // 🔴 贴着支撑面的那一层排在后面 —— 爪子伸不到它下面。实测:平躺薄件合爪停在 0,指间是空的。
        let first = c[0].above_support_m;
        let last = c[c.len() - 1].above_support_m;
        assert!(first >= last, "高的段该排在前面:{first} vs {last}");
    }

    #[test]
    fn a_flat_sheet_is_refused_not_silently_empty() {
        let mut v = Vec::new();
        for i in 0..10 {
            for j in 0..10 {
                v.push(P3 { x: 0.4 + i as f64 * 0.005, y: j as f64 * 0.005, z: 0.0 });
            }
        }
        assert_eq!(
            candidates(&v, &body(JawSpan::Measured(0.088)), 0.0, Grid::default()).unwrap_err(),
            Refusal::Flat
        );
    }

    #[test]
    fn scissors_the_bbox_lied_about() {
        // 两片薄刃夹一条缝,整体"宽 8 cm",但真正能夹的是每片自己的 9 mm。
        // 包围盒会说"8 cm,能夹",于是爪子从缝里合过去,指间什么都没有(实测)。
        // 吃表面点就不会:量到的是每片刃自己的宽度。
        let mut v = Vec::new();
        for i in 0..60 {
            let z = 0.01 + (i as f64 / 59.0) * 0.02;
            for blade in [-0.035_f64, 0.035] {
                for a in 0..6 {
                    for b in 0..4 {
                        // 一片刃:沿 x 长 3 cm,沿 y 厚 9 mm(实心)
                        v.push(P3 {
                            x: 0.4 - 0.015 + 0.03 * (a as f64) / 5.0,
                            y: blade - 0.0045 + 0.009 * (b as f64) / 3.0,
                            z,
                        });
                    }
                }
            }
        }
        let c = candidates(&v, &body(JawSpan::Measured(0.088)), 0.0, Grid::default()).unwrap();
        // 存在一条明显比"整体 8 cm"窄得多的候选 —— 那才是真能夹的地方。
        assert!(c.iter().any(|x| x.width_m < 0.02), "该找得到刃上那条窄段");
    }
}

// ────────────────────── ②a 的产出:一个真正的接触集 ──────────────────────

impl Contact {
    /// 把这条候选变成一个**接触集**(四格 + 进场方向),交给 ②b。
    ///
    /// # 🔴 为什么必须有这一步
    ///
    /// 这个结构里的 `close_yaw` / `width_m` / `face_tilt_rad` 是**几何算出来的三个标量**,
    /// 而 ②b 要的是"碰哪几个点、每点法向与锥、物体怎么动、每点容差"。
    /// 不做这一步,这三个标量在交接时被压掉 —— 而**丢掉的恰好是最贵的那部分**:
    /// 法向没了 ⇒ 手腕只能压死朝下;进场方向没了 ⇒ ②b 只能 `NoFrame` 拒绝。
    ///
    /// # 两点从哪来
    ///
    /// `point` 是**下手点**(两指之间那个中点),`close_yaw` 是合爪方向,`width_m` 是那一段的宽。
    /// ⇒ 两个接触点 = 下手点 ± (宽/2) × 合爪方向。**这不是假设两根手指** ——
    /// 它是"沿这个方向、隔这么宽,有两个相对的面"这件几何事实;三指五指由别的生成器给更多点。
    ///
    /// # 🔴🔴 订正:这里原来把"要多大的锥"填进了"有多大的锥"那一格
    ///
    /// 上一版 `half_angle: self.face_tilt_rad`。**方向是反的,而且反得很危险**:
    /// `face_tilt_rad` 是**这一把需要多大的摩擦锥才不滑**(越大 = 这一把越差),
    /// 而 `Cone.half_angle` 被 `can_drive` 读成**实际允许往哪使劲**(越大 = 越使得上劲)。
    /// ⇒ **一把越差的抓取,在判据里看起来越可行。** 没有任何一个环节会不一致。
    ///
    /// 正解就写在 `face_tilt_rad` 自己的注释里:*"条件等价于每个接触面的法向与合爪方向的夹角
    /// < `atan(μ)`"*。⇒ **μ 必须由调用方给**(它是身体×世界的耦合,量得出来:
    /// 拿指头在参考面上蹭一下测打滑门槛),`half_angle = atan(μ)`,
    /// 而 `face_tilt_rad > atan(μ)` 时**当场拒绝并报出差多少**,不静默放行。
    ///
    /// `motion` 由调用方给 —— 第③格说的是**物体要怎么动**,而那是任务的事,不是几何的事。
    /// ②a 只知道"从哪儿下手"。
    pub fn to_set(
        &self,
        mu: f64,
        motion: contact_set::Twist,
        tol_m: f64,
    ) -> Result<contact_set::ContactSet, Handoff> {
        let have = mu.atan();
        if !(mu.is_finite() && mu > 0.0) {
            return Err(Handoff::MuUnknown);
        }
        if self.face_tilt_rad > have {
            return Err(Handoff::WouldSlip { need_rad: self.face_tilt_rad, have_rad: have });
        }
        Ok(self.build(have, motion, tol_m))
    }

    fn build(
        &self,
        half_angle: f64,
        motion: contact_set::Twist,
        tol_m: f64,
    ) -> contact_set::ContactSet {
        let (c, s) = (self.close_yaw.cos(), self.close_yaw.sin());
        let jaw = [c, s, 0.0]; // 合爪方向(水平面内)
        let half = self.width_m / 2.0;
        let mk = |sign: f64| contact_set::Point {
            by: contact_set::Who::Hand,
            pull: false,
            torsion: false,
            peel: false,
            at: [
                self.point.x + jaw[0] * half * sign,
                self.point.y + jaw[1] * half * sign,
                self.point.z,
            ],
            // 法向指向物体外侧 = 从下手点指向这一侧
            normal: [jaw[0] * sign, jaw[1] * sign, 0.0],
            cone: contact_set::Cone {
                // 允许的用力方向:朝里(把物体夹住)
                axis: [-jaw[0] * sign, -jaw[1] * sign, 0.0],
                half_angle,
            },
            tol_m,
        };
        contact_set::ContactSet {
            points: vec![mk(-1.0), mk(1.0)],
            motion,
            // 🔴 进场方向 = **支撑面法向的反向**。
            // 这不是"世界 z 是特权方向",而是 ②a 本来就建立在"有一张支撑面"之上:
            // 它按水平层切片、按 `min_above_m` 判"爪子伸不伸得进去"。支撑面朝上 ⇒ 从上面来。
            // 换一台把支撑面立起来的机器,这一项跟着支撑面走,而不是跟着 z 走。
            approach: Some([0.0, 0.0, -1.0]),
        }
    }
}

#[cfg(test)]
mod 产出接触集 {
    use super::*;

    fn 一块方料() -> Vec<P3> {
        let mut v = Vec::new();
        for i in 0..12 {
            for j in 0..12 {
                for k in 0..6 {
                    v.push(P3 {
                        x: -0.03 + 0.06 * i as f64 / 11.0,
                        y: -0.02 + 0.04 * j as f64 / 11.0,
                        z: 0.90 + 0.05 * k as f64 / 5.0,
                    });
                }
            }
        }
        v
    }

    #[test]
    fn 生成器吐出来的东西_执行层直接吃得下() {
        let body = Body {
            jaw: JawSpan::Measured(0.08),
            reach_lo: 0.05,
            reach_hi: 1.0,
            base_x: 0.0,
            base_y: -0.4,
        };
        let grid = Grid {
            bands: 4,
            jaw_h_m: 0.02,
            dirs: 12,
            min_pts: 8,
            min_above_m: 0.001,
            finger_w_m: 0.02,
            gap_m: 0.01,
        };
        let cs = candidates(&一块方料(), &body, 0.90, grid).expect("这块料该给得出候选");
        assert!(!cs.is_empty(), "候选不该是空的");
        let c = cs.iter().find(|c| c.reachable).unwrap_or(&cs[0]);

        // 抓:物体不动 —— 四格自检必须过。**μ 必须由调用方给**(这里当作量过的 0.5)。
        let 静 = contact_set::Twist::still([c.point.x, c.point.y, c.point.z]);
        let set = c.to_set(0.5, 静, 0.002).expect("这条候选该交得出去");
        assert_eq!(set.points.len(), 2, "下手点 + 宽度 + 合爪方向 ⇒ 两个相对的接触点");
        assert_eq!(set.check(false), Ok(()), "②a 吐出来的接触集必须自检就过");

        // 两点之间的距离要等于那一段的宽
        let d = contact_set::norm([
            set.points[1].at[0] - set.points[0].at[0],
            set.points[1].at[1] - set.points[0].at[1],
            set.points[1].at[2] - set.points[0].at[2],
        ]);
        assert!((d - c.width_m).abs() < 1e-12, "两点间距 = 段宽 {:.4},实得 {d:.4}", c.width_m);

        // 两个锥必须朝里、而且互相相反
        let a0 = set.points[0].cone.axis;
        let a1 = set.points[1].cone.axis;
        assert!(contact_set::dot(a0, a1) < -0.999, "对夹:两个锥必须相反");
        // 🔴 进场方向必须有 —— 没有它 ②b 只能拒绝(四格定不下这一个自由度)
        assert!(set.approach.is_some(), "②a 看得见空隙,进场方向该由它填");
        // 🔴 锥就是**摩擦锥**,半张角 = atan(μ);不是 face_tilt(那是"需要多大",不是"有多大")
        for p in &set.points {
            assert!((p.cone.half_angle - 0.5f64.atan()).abs() < 1e-12);
        }
    }

    /// 🔴🔴 **反例:歪得厉害的那一把,μ 小就必须交不出去。**
    ///
    /// 上一版把 `face_tilt_rad` 填进 `Cone.half_angle`,方向正好反了:
    /// **越歪(越差)的抓取,在判据里锥越大、看起来越可行**。这条测试锁死那个方向。
    #[test]
    fn 歪得厉害的那一把_摩擦不够就必须拒绝() {
        let 歪 = Contact {
            point: P3 { x: 0.0, y: 0.0, z: 0.95 },
            close_yaw: 0.0,
            width_m: 0.04,
            margin_m: 0.04,
            above_support_m: 0.05,
            reach_r: 0.4,
            reachable: true,
            jaw_declared: false,
            within_jaw: true,
            n_pts: 40,
            depth_m: 0.03,
            face_tilt_rad: 0.60, // 两个面歪了 34.4°
            com_offset_m: 0.0,
        };
        let 静 = contact_set::Twist::still([0.0, 0.0, 0.95]);
        // μ=0.5 ⇒ 摩擦锥只有 26.6° < 34.4° ⇒ 会滑,必须点名差多少
        match 歪.to_set(0.5, 静, 0.002) {
            Err(Handoff::WouldSlip { need_rad, have_rad }) => {
                assert!((need_rad - 0.60).abs() < 1e-12);
                assert!((have_rad - 0.5f64.atan()).abs() < 1e-12);
                assert!(need_rad > have_rad);
            }
            other => panic!("会滑的那一把必须拒绝,实得 {other:?}"),
        }
        // μ=1.0 ⇒ 摩擦锥 45° > 34.4° ⇒ 同一把就交得出去了。**差别只在 μ,不在几何。**
        assert!(歪.to_set(1.0, 静, 0.002).is_ok());
        // μ 没量过就不许瞎填
        assert_eq!(歪.to_set(0.0, 静, 0.002), Err(Handoff::MuUnknown));
    }
}

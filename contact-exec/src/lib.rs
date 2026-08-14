//! `contact-exec` —— **②b 闭式执行层**:接触集 + 物体要怎么动 → 一串该发的航点。
//!
//! # 为什么它必须是一个独立的东西,而不是策略里的几行
//!
//! 2026-08-13 之前这一层住在策略的 Python 里,而那一夜的每一次失败都落在它身上:
//! 悬停高度、搬运高度、合爪之后还发什么、什么时候算"偏了"。**每一条都是这一层的规矩,
//! 而它们当时全是我临时拍的。** 拍出来的数没有出处,换一个物体、换一具身体就废。
//!
//! # 它不许知道的
//!
//! 🔴 **任何 benchmark / 任务 / 物体的名字,任何动词分支。** 它只认:
//! 「碰这一点、爪面朝这个方向、这一段有这么宽」+「物体要往那儿走这么远」+ 这具身体的常数。
//!
//! # 身体常数从哪来
//!
//! **调用方先问驱动要,再递进来** —— 和 `contact-gen` 同一个理由:这一层一旦自己去读驱动,
//! "换机体不重训"就没有机制保证了。递进来的每一个数都在协议里看得见。

#![forbid(unsafe_code)]

/// 一个三维点,米,世界坐标。
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct P3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// 这具身体量出来的常数。**全部由调用方从驱动取来递进**,这一层不自己去读。
#[derive(Copy, Clone, Debug)]
pub struct Body {
    /// 钳口全开多少米。
    pub jaw_span_m: f64,
    /// 爪尖离所命令的那个法兰有多远(沿工具轴),米。
    pub tool_offset_m: f64,
    /// 够得着的水平带,离本臂臂根,米。**必须是带腕姿约束量出来的那一份** ——
    /// 不带腕姿的那个带在这套栈里用错了整整一夜(0.602 vs 0.419,差 18 cm)。
    pub reach_lo_m: f64,
    pub reach_hi_m: f64,
    /// 本臂臂根的水平位置。
    pub base_x: f64,
    pub base_y: f64,
    /// 这具身体在这一带能压到多低(爪尖 z)。低于它 = 这条胳膊没解了,**不是碰到东西**。
    pub floor_z: f64,
    /// 落点重复精度,米。**"偏了没有"的门槛只能由它给** ——
    /// 比它还紧的容差会让一具完全正常的身体永远判"没到位"。
    pub repeat_m: f64,
}

/// 接触集里那一条:碰哪儿、爪面朝哪、那一段多宽。
#[derive(Copy, Clone, Debug)]
pub struct Contact {
    pub point: P3,
    pub close_yaw: f64,
    pub width_m: f64,
}

/// 物体要怎么动 —— 接触集的第三格。**说的是物体,一个字不提机器人。**
#[derive(Copy, Clone, Debug)]
pub struct Motion {
    /// 单位方向。
    pub along: [f64; 3],
    /// 沿那个方向多少米。
    pub dist_m: f64,
}

/// 一个该发的航点。
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Waypoint {
    /// 爪尖要去哪。
    pub tcp: P3,
    /// 腕转到哪(爪面朝向)。
    pub yaw: f64,
    /// 爪子开到多少米。**注意是米,不是那个只有两指手才听得懂的归一化标量。**
    pub open_m: f64,
    /// 🔴 这一步要不要**碰到东西**。
    ///
    /// 要碰的:差几毫米就废,偏了必须重来。
    /// 只是路过的:差几厘米无所谓 —— 渲图 2026-08-13 拍到爪子**早已张开停在物体正上方**,
    /// 而代码因为悬停点差 5.11 cm 判"偏了",一遍遍重算,**一次都没往下走过**。
    pub touching: bool,
}

/// 为什么一条航点都给不出来。**拒绝要说得出理由。**
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Refusal {
    /// 下手点在够得着的带外面。
    OutOfReach,
    /// 下手点低于这具身体能压到的高度 —— 命令下去只会顶住,而那会被读成"碰到了"。
    BelowFloor,
    /// 那一段比钳口还宽。⚠️ **只在这里拒**(执行不了),排候选的时候**不许**拿宽度删。
    WiderThanJaw,
    /// 物体要动的方向不是一个方向。
    BadMotion,
}

/// 抬起多高才算"离开了支撑面"。**由物体自己的高度给,不是拍的**:
/// 要把物体的底抬过它自己的顶,才算真的离开。
fn lift_for(obj_top_z: f64, pick_z: f64) -> f64 {
    (obj_top_z - pick_z).max(0.0) + 0.02
}

/// 给定接触集 + 物体要怎么动 → 该发的一串航点。
///
/// `obj_top_z` 是这个物体最高点的高度(点云给的),用来定进场高度:
/// 🔴 **进场必须从物体顶上过去** —— 实测悬停只有 2.7 cm 时,爪子贴着桌面横扫,
/// 还没合爪就把目标推走了 2.5 cm。
pub fn waypoints(
    body: &Body,
    c: &Contact,
    m: &Motion,
    obj_top_z: f64,
) -> Result<[Waypoint; 7], Refusal> {
    let r = ((c.point.x - body.base_x).powi(2) + (c.point.y - body.base_y).powi(2)).sqrt();
    if r < body.reach_lo_m || r > body.reach_hi_m {
        return Err(Refusal::OutOfReach);
    }
    if c.point.z < body.floor_z {
        return Err(Refusal::BelowFloor);
    }
    if c.width_m >= body.jaw_span_m {
        return Err(Refusal::WiderThanJaw);
    }
    let n = (m.along[0].powi(2) + m.along[1].powi(2) + m.along[2].powi(2)).sqrt();
    if !(n > 0.0) || !m.dist_m.is_finite() {
        return Err(Refusal::BadMotion);
    }
    let u = [m.along[0] / n, m.along[1] / n, m.along[2] / n];
    let tgt = P3 {
        x: c.point.x + u[0] * m.dist_m,
        y: c.point.y + u[1] * m.dist_m,
        z: c.point.z + u[2] * m.dist_m,
    };

    let lift = lift_for(obj_top_z, c.point.z);
    // 🔴 **搬运高度取两者较高,不是相加。** 实测:相加之后算成 31.6 cm,手臂够不到,
    //    横移那一段每一集都差 12.2 cm 停在半路。
    let carry_z = (c.point.z + lift).max(tgt.z);
    let open = body.jaw_span_m;
    // 🔴 **合上之后保持在这一段的宽度,不再命令"合到底"。**
    //    继续往里挤 = 把小东西当西瓜籽挤出去;而"该多挤一点点"是个**没量过的身体量**,
    //    量到之前只能不挤。
    let hold = c.width_m;

    Ok([
        // 进场:从物体顶上过去
        Waypoint { tcp: P3 { z: c.point.z + lift, ..c.point }, yaw: c.close_yaw, open_m: open, touching: false },
        // 下到下手点
        Waypoint { tcp: c.point, yaw: c.close_yaw, open_m: open, touching: true },
        // 合爪到这一段的宽度
        Waypoint { tcp: c.point, yaw: c.close_yaw, open_m: hold, touching: true },
        // 抬起
        Waypoint { tcp: P3 { z: c.point.z + lift, ..c.point }, yaw: c.close_yaw, open_m: hold, touching: false },
        // 横移(在较高的那个高度上)
        Waypoint { tcp: P3 { z: carry_z, ..tgt }, yaw: c.close_yaw, open_m: hold, touching: false },
        // 下降到目标
        Waypoint { tcp: tgt, yaw: c.close_yaw, open_m: hold, touching: true },
        // 松手
        Waypoint { tcp: tgt, yaw: c.close_yaw, open_m: open, touching: true },
    ])
}

/// 走完一段之后:偏了没有,要不要重来。
///
/// 🔴 **门槛只能由这具身体自己的重复精度给。** 我 2026-08-13 拍过一个 5 mm 的门槛,
/// 而这条臂的落点残差中位就是 3.5 mm —— 于是**一半的段被判成"偏了"**,每一步都在半路重算,
/// 合爪那一段**一次都没执行到**。
pub fn off_course(body: &Body, w: &Waypoint, residual_m: f64) -> bool {
    // 🔴 **只是路过的点,永远不算"偏了"。** 它的职责只有"从物体上面绕过去",
    //    差几厘米不改变任何事 —— 渲图 2026-08-13 直接拍到:爪子已经张开停在物体正上方,
    //    而代码因为悬停点差 5.11 cm 判"偏了"、一遍遍重算,**一次都没往下走过**。
    //    ⇒ 这里**不给过境点任何门槛**。给一个就是又拍一个数,而我今晚已经拍错过两次。
    if !w.touching {
        return false;
    }
    // 要碰的点:门槛只能由这具身体自己的重复精度给。比它还紧的容差会让一具完全正常的身体
    // 永远判"没到位"(我拍过 5 mm,而这条臂的残差中位就是 3.5 mm ⇒ 一半的段被误判)。
    residual_m > (2.0 * body.repeat_m).max(0.002)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 这具身体今晚量出来的那一组数(不是编的):
    /// 钳口 0.0803 · 工具偏置 0.1451 · **带腕朝下的**够得着 [0.174, 0.419] · 地板 0.9208 −偏置
    /// · 落点重复精度 0.0035(实测残差中位)。
    fn arx() -> Body {
        Body {
            jaw_span_m: 0.0803,
            tool_offset_m: 0.1451,
            reach_lo_m: 0.17369,
            reach_hi_m: 0.41855,
            base_x: 0.30047,
            base_y: -0.3523,
            floor_z: 0.9208 - 0.1451,
            repeat_m: 0.0035,
        }
    }

    fn at(x: f64, y: f64, z: f64, w: f64) -> Contact {
        Contact { point: P3 { x, y, z }, close_yaw: 0.0, width_m: w }
    }

    /// 🔴 合上之后**保持在那一段的宽度**,不许再命令合到底。
    /// 今晚实测:合爪停在 1.39 cm(真夹上了),抬起那一段爪子**一路合到 0** ⇒ 东西没了。
    #[test]
    fn after_closing_it_holds_the_width_instead_of_squeezing_to_zero() {
        let b = arx();
        let ws = waypoints(&b, &at(0.40, -0.20, 0.80, 0.035),
                           &Motion { along: [0.0, 0.0, 1.0], dist_m: 0.12 }, 0.81).unwrap();
        // 合爪那一步开到这一段的宽度
        assert_eq!(ws[2].open_m, 0.035);
        // 之后每一步都保持,**没有一步是 0**
        for w in &ws[2..6] {
            assert_eq!(w.open_m, 0.035, "合上之后又去挤了");
        }
        // 最后松手才回到全开
        assert_eq!(ws[6].open_m, b.jaw_span_m);
    }

    /// 🔴 搬运高度取**两者较高**,不是相加。相加实测算成 31.6 cm ⇒ 够不到,
    /// 横移那一段每集都差 12.2 cm 停在半路。
    #[test]
    fn carry_height_is_the_max_not_the_sum() {
        let b = arx();
        let pick_z = 0.80;
        let ws = waypoints(&b, &at(0.40, -0.20, pick_z, 0.03),
                           &Motion { along: [0.0, 0.0, 1.0], dist_m: 0.12 }, 0.81).unwrap();
        let carry = ws[4].tcp.z;
        let lift_top = ws[3].tcp.z;
        let place_z = ws[5].tcp.z;
        assert_eq!(carry, lift_top.max(place_z));
        assert!(carry < lift_top + place_z - pick_z, "搬运高度被叠加了:{carry}");
    }

    /// 🔴 进场必须从**物体顶上**过去。实测悬停 2.7 cm 时贴着桌面横扫,还没合爪就把目标推走 2.5 cm。
    #[test]
    fn the_approach_clears_the_object_top() {
        let b = arx();
        let top = 0.85;
        let ws = waypoints(&b, &at(0.40, -0.20, 0.80, 0.03),
                           &Motion { along: [0.0, 0.0, 1.0], dist_m: 0.12 }, top).unwrap();
        assert!(ws[0].tcp.z > top, "进场高度 {} 没过物体顶 {top}", ws[0].tcp.z);
    }

    /// 🔴 够不着 / 低于地板 / 比钳口宽 —— 三种都必须**拒绝并说得出理由**,不许硬发。
    #[test]
    fn it_refuses_rather_than_commanding_something_impossible() {
        let b = arx();
        let m = Motion { along: [0.0, 0.0, 1.0], dist_m: 0.10 };
        // 离臂根 0.60 m —— 在不带腕姿的老带里够得着,在**带腕朝下**的新带里够不着
        assert_eq!(
            waypoints(&b, &at(0.30047 + 0.60, -0.3523, 0.80, 0.03), &m, 0.81),
            Err(Refusal::OutOfReach)
        );
        assert_eq!(
            waypoints(&b, &at(0.40, -0.20, b.floor_z - 0.01, 0.03), &m, 0.81),
            Err(Refusal::BelowFloor)
        );
        assert_eq!(
            waypoints(&b, &at(0.40, -0.20, 0.80, 0.09), &m, 0.81),
            Err(Refusal::WiderThanJaw)
        );
    }

    /// 🔴 "偏了没有"的门槛只能由**这具身体自己的重复精度**给,而且**要碰的点和路过的点不是一个数**。
    /// 我拍过一个 5 mm 的全局门槛,而这条臂的残差中位就是 3.5 mm ⇒ 一半的段被误判成偏,
    /// 每步都在半路重算,合爪那一段一次都没走到。
    #[test]
    fn the_tolerance_comes_from_this_body_and_differs_per_waypoint() {
        let b = arx();
        let touch = Waypoint { tcp: P3 { x: 0.0, y: 0.0, z: 0.0 }, yaw: 0.0, open_m: 0.0, touching: true };
        let pass = Waypoint { touching: false, ..touch };
        // 3.5 mm = 这具身体自己的水平 ⇒ 两种航点都不算偏
        assert!(!off_course(&b, &touch, 0.0035));
        assert!(!off_course(&b, &pass, 0.0035));
        // 5.11 cm:路过的点无所谓(渲图证过爪子已经就位),要碰的点必须判偏
        // 过境点无论差多少都不算偏 —— 它只是路过
        assert!(!off_course(&b, &pass, 0.0511), "悬停差 5 cm 被判成偏 —— 那正是白跑一夜的那个 bug");
        assert!(!off_course(&b, &pass, 0.30), "过境点不该有门槛");
        assert!(off_course(&b, &touch, 0.0511));
    }
}

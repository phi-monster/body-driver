//! `verb` — 动词分派。眼睛给【一个像素 + 一个动词】,这里决定这具身体走哪几步。
//!
//! # 为什么它必须住在驱动里
//!
//! "抓" 在不同身体上是完全不同的关节运动;但 **"让这个物体离开支撑面"** 在所有身体上是同一句话。
//! 动词描述的是 **物体要发生什么**,不含身体 —— 所以把它放进驱动**不破坏**换机体那条命根子,
//! 而把它留在策略里,就等于每换一个任务、每换一具身体都要重写一遍。
//!
//! # 这一层 2026-08-12 才存在,而它本该一开始就在
//!
//! 在此之前 `bl_world_ref.verb` 这个字段在整个驱动里 **一次都没有被读过**(grep 可证:只有结构体
//! 定义里的 `pub verb: u32`,没有任何一处分支)。所有招式逻辑住在策略的 Python 里,于是:
//! 一个任务一个毛病,一个毛病一个补丁 —— 42 个任务就是 42 份补丁。
//!
//! # 这里面装的每一条,都是实测换来的,不是设计出来的
//!
//! * `contact_seen`   —— 能被推动的物体**不会让手停下来**(实测:物体被推走 13.4 cm 而"命令 vs 实到"
//!                       检测器 0/3 全 False)。那把尺子是拿**往桌面上压**的阶梯验的,桌面推不动。
//! * `spannable`      —— 下手之前先问"这一段夹不夹得下";夹不下就别试(实测:铲子三个候选宽 7.3–8.5 cm
//!                       而爪子极限 8.8,三次全落空,纯浪费)。
//! * `Phase::Clear`   —— 失败后**先竖直抬到过境高度再张爪**(实测:原地张爪把贝壳弹出 39.9 cm,
//!                       比不改还差,当场判负回退)。

/// 一个动词被拆成的步骤。**顺序本身是承重的**,见 `Clear` 的注释。
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Phase {
    /// 抬到过境高度(与物体高矮无关:实测按"物体顶面+净空"定会贴着矮物体横扫)
    Transit,
    /// 在过境高度上横移到下手点正上方
    Over,
    /// 竖直下到下手点
    Descend,
    /// 合爪 / 施力
    Engage,
    /// 完成动词本身要求的物体运动(抬起 / 推动 / 转动 / 插入)
    Act,
    /// 🔴 失败后先**竖直**抬回过境高度,**然后**才松手。反过来会把物体弹飞。
    Clear,
    /// 回到这具身体的原位。30/42 个任务强制要,而且它是"动作可组合"的前提。
    Home,
}

/// 一次尝试的自查结果。**全部由已测量的量算出,不引入任何新传感器。**
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Check {
    /// 夹到的宽度与打算夹的那一段对得上
    AsPlanned,
    /// 合到底,指间空的
    ClosedOnAir,
    /// 夹住了,但夹的是别的一段。
    ///
    /// ⚠️ **保留是为了不改 ABI 的既有编号**;新代码应当读下面劈开的两档 ——
    /// 这一档把两件**修法相反**的事混在一起,见 [`Self::StoppedWide`] / [`Self::PinchedThinner`]。
    WrongSection,
    /// 还没夹到就把物体碰跑了
    KnockedAway,
    /// 🔴 **钳口【没走到】那一段就停住了** —— 实夹宽度 ÷ 计划段宽 **> 1**。
    ///
    /// 判别量与分组来自实测(n=185):锤子 **2.50** · 扳手 **1.72**(这一档)
    /// vs 钳子 **0.06** · 卷尺 **0.32**(另一档)—— **零重叠**,所以界就在 1,不是拍出来的。
    ///
    /// ⚠️ **这一档还有两种读法分不开,必须一起写**:那一处**确实更宽**,
    /// vs **爪尖压在支撑面上被摩擦卡住**。分开只要一个探针 ——
    /// **同一个 xy 抬高 3 cm 再合一次爪**。**该探针未做**,所以本档的建议只是**符号**。
    StoppedWide,
    /// 🔴 **钳口【走过头】,捏到了更细的一处** —— 实夹宽度 ÷ 计划段宽 **< 1**。
    ///
    /// 这一档才是"下手点选错了",换下一个候选有意义。
    PinchedThinner,
}

/// "碰到了没有"。
///
/// 🔴 **能被推动的物体不会让手停下来** —— 所以只看"命令走了多少 / 实到走了多少"是不够的,
/// 那把尺子是拿**往桌面上压**(压不动)的阶梯数据验的。推得动的东西,手照样走满,物体跟着跑。
/// ⇒ 两个信号取或:手停住了 **或者** 世界看得见地动了。
pub fn contact_seen(
    commanded_m: f64,
    achieved_m: f64,
    contact_threshold: f64,
    object_moved_m: f64,
    object_move_eps: f64,
) -> bool {
    let stalled = commanded_m > 0.0 && (achieved_m / commanded_m) < contact_threshold;
    let world_moved = object_moved_m > object_move_eps;
    stalled || world_moved
}

/// "这一段我夹不夹得下"。夹不下就不该试 —— 试了也是三次全落空。
pub fn spannable(section_width_m: f64, jaw_span_m: f64, margin_m: f64) -> bool {
    section_width_m > 0.0 && section_width_m + margin_m <= jaw_span_m
}

/// 合完爪之后,用**量出来的**爪值反推实际夹住多宽,再和打算夹的那一段比。
///
/// 这是驱动唯一需要的自查:不需要力传感器,不需要触觉,只需要爪子自己的读数。
pub fn classify(
    planned_w_m: f64,
    jaw_value: f64,
    jaw_span_m: f64,
    object_moved_m: f64,
    knock_eps_m: f64,
    tol_frac: f64,
) -> Check {
    if object_moved_m > knock_eps_m {
        return Check::KnockedAway;
    }
    let held = jaw_value * jaw_span_m;
    if held <= 1e-4 {
        return Check::ClosedOnAir;
    }
    let err = (held - planned_w_m).abs();
    if planned_w_m > 0.0 && err <= tol_frac * planned_w_m {
        return Check::AsPlanned;
    }
    if planned_w_m <= 0.0 {
        return Check::WrongSection; // 没有计划段宽 ⇒ 劈不开,如实报旧档
    }
    // 🔴 劈成两档:比值 = 实夹宽度 ÷ 计划段宽。**界在 1,不是拍的** ——
    //    实测 n=185 两组零重叠:锤子 2.50 / 扳手 1.72 vs 钳子 0.06 / 卷尺 0.32。
    //    两档的修法**相反**:前者该换动词(钳口没走到那一段),后者该换下手点(走过头捏细了)。
    if held > planned_w_m {
        Check::StoppedWide
    } else {
        Check::PinchedThinner
    }
}

/// 一次动作**停下来的理由**。每一条都必须对应驱动**已经在算**的一个量 ——
/// 不许引入新传感器,这和 [`Check`] 那一条是同一条规矩。
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u32)]
pub enum Until {
    /// 走完 `dist_m` / `turn_rad` 就停。**今天唯一的一种。**
    Amount = 0,
    /// 直到**碰上** —— 命令与实到开始分岔(`probe::contact_threshold` 量的就是这个分岔的下界)。
    Contact = 1,
    /// 直到**推不动** —— 分岔到了这具身体的上界,再发命令读数也不变。
    /// 合爪停在物体宽度上就是这一条(驱动 ⑥ 已经在用,只是以前没有名字)。
    Resist = 2,
    /// 直到**东西不再跟着我走** —— 被跟的那一块在画面上不再随命令位移(脱手 / 打滑)。
    Slip = 3,
    /// 直到**画面不再变** —— 倒完了、装满了、晃停了。
    Settle = 4,
}

impl Until {
    pub fn from_u32(v: u32) -> Option<Until> {
        use Until::*;
        Some(match v { 0 => Amount, 1 => Contact, 2 => Resist, 3 => Slip, 4 => Settle, _ => return None })
    }
    /// 这条停法要不要**一边走一边看**(而不是发完命令等它走完)。
    pub fn needs_watching(self) -> bool { self != Until::Amount }
}

/// 抬起之后,东西还在不在手里。
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Hold {
    /// 还在。抬起过程里爪子几乎没再动。
    Held,
    /// **滑掉了** —— 抬的过程中爪子继续合拢,说明指间的东西走了。
    Slipped,
    /// 合爪时指间就是空的,谈不上滑。
    WasEmpty,
}

/// **拿没拿住** —— 只用爪子自己的读数,不用物体位姿。
///
/// # 为什么这一格值钱
///
/// 在此之前,判"抓成没成"要看**物体的真实位姿**(仿真里的特权信息,真机上没有),
/// 而且要把整个抬升动作做完(18 cm)才知道。这里只要**提一点点**,读一次爪子就够了。
///
/// # 判据是实测的,不是设计的(2026-08-12,50 次下手)
///
/// | | n | 抬升中爪子又合拢了多少 |
/// |---|---|---|
/// | **拿住了** | 3 | 中位 **0.0049**,最大 **0.0165** |
/// | **没拿住** | 47 | 中位 **0.1755**,最大 **0.8000** |
///
/// 两组**量级差 36 倍且不重叠** —— 成功那组的最大值比失败组的中位还小一个量级。
/// 机制上也讲得通:**东西还在指间,爪子就合不下去。**
///
/// ⚠️ **成功样本只有 3 个**。这条结论靠的是**两组分布不重叠 + 机制**,不是靠 n。
/// ⚠️ `slip_eps`(合拢多少算滑)**是一个还没量到的身体量**,现在按参数传入。
///    实测把它放在 0.02–0.15 之间都能把上面那两组切开,但**那个区间本身还没被量过**。
pub fn holding(grip_at_close: f64, grip_after_lift: f64, air_eps: f64, slip_eps: f64) -> Hold {
    if grip_at_close <= air_eps {
        return Hold::WasEmpty;
    }
    if grip_at_close - grip_after_lift > slip_eps {
        Hold::Slipped
    } else {
        Hold::Held
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pushable_object_is_detected_by_world_motion_not_by_stall() {
        // 实测那一集:手一步没少走(交付率 1.0),而物体被推走 13.4 cm。
        assert!(!contact_seen(0.010, 0.010, 0.29383, 0.0, 0.002));
        assert!(contact_seen(0.010, 0.010, 0.29383, 0.134, 0.002));
    }

    #[test]
    fn immovable_contact_still_detected_by_stall() {
        // 往桌面上压:手停住,物体不动。旧尺子在这一档是对的,不能退化。
        assert!(contact_seen(0.010, 0.001, 0.29383, 0.0, 0.002));
    }

    #[test]
    fn too_wide_section_is_refused_before_trying() {
        // 爪子 8.8 cm。9 mm 的细边(剪刀,实测成功)必须放行;8.4 cm 留 5 mm 余量后必须拒。
        assert!(spannable(0.009, 0.088, 0.005));
        assert!(!spannable(0.084, 0.088, 0.005));
        // 🔴 而 7.3 cm 这一档是【未决】的:按 5 mm 余量它算"夹得下",但实测(铲子)夹不住。
        //    差的那个量是【爪子要比物体宽多少才真的合得上而不是把它推开】—— 一个还没测过的身体量。
        //    在它被测出来之前,这里只能记下事实,不许拿一个凑出来的余量把测试凑绿。
        assert!(spannable(0.073, 0.088, 0.005), "按当前(未测)的余量它是放行的,而实测失败");
    }

    #[test]
    fn slipping_is_told_apart_by_the_jaw_alone_no_object_pose() {
        // 实测那两组(2026-08-12):拿住了最大合拢 0.0165;没拿住中位 0.1755。
        let (air, slip) = (0.005, 0.05);
        assert_eq!(holding(0.30, 0.2951, air, slip), Hold::Held); // 合拢 0.0049
        assert_eq!(holding(0.30, 0.2835, air, slip), Hold::Held); // 合拢 0.0165(成功组最大)
        assert_eq!(holding(0.30, 0.1245, air, slip), Hold::Slipped); // 合拢 0.1755(失败组中位)
        assert_eq!(holding(0.80, 0.0000, air, slip), Hold::Slipped); // 合到底
        // 合爪时就是空的 ⇒ 谈不上滑
        assert_eq!(holding(0.000, 0.0, air, slip), Hold::WasEmpty);
    }
}

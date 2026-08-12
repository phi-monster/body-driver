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

/// 眼睛能说的动词。**加一个动词的代价是零** —— 它描述物体要发生什么,不提身体;
/// 而加一个**字段**的代价不是零(见 `bl_world_ref` 头注:那是唯一能悄悄毁掉换机体的改动)。
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u32)]
pub enum Verb {
    Reach = 0,
    Grasp = 1,
    Release = 2,
    Press = 3,
    Wipe = 4,
    /// 不抓,只把它推到别处 —— 接触点 + 推的方向
    Push = 5,
    /// 撬起一边(平躺薄件唯一的入口:爪子伸不到它下面)
    Pry = 6,
    /// 推倒 / 翻面,让它换一个支撑面
    Flip = 7,
    /// 握住之后绕一条轴转(倒水)
    Pour = 8,
    /// 握住之后绕它自己的轴转(拧盖)
    Twist = 9,
    /// 握住之后沿一条轴往里走(插)
    Insert = 10,
    /// 从散装里舀起一部分
    Scoop = 11,
    /// 放到指定位置并松手(与 Release 的区别:它有目标位姿)
    Place = 12,
}

impl Verb {
    pub fn from_u32(v: u32) -> Option<Verb> {
        use Verb::*;
        Some(match v {
            0 => Reach,
            1 => Grasp,
            2 => Release,
            3 => Press,
            4 => Wipe,
            5 => Push,
            6 => Pry,
            7 => Flip,
            8 => Pour,
            9 => Twist,
            10 => Insert,
            11 => Scoop,
            12 => Place,
            _ => return None,
        })
    }

    /// 这个动词需不需要先把物体握在手里。决定了失败时该退回哪一步。
    pub fn needs_hold(self) -> bool {
        use Verb::*;
        matches!(self, Grasp | Pour | Twist | Insert | Place | Scoop)
    }

    /// 这个动词要不要合爪。推/按/擦/撬/翻**不合爪**,所以"夹不夹得下"这一问对它们不成立。
    pub fn closes_jaw(self) -> bool {
        use Verb::*;
        matches!(self, Grasp | Pour | Twist | Insert | Place | Scoop)
    }
}

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
    /// 夹住了,但夹的是别的一段
    WrongSection,
    /// 还没夹到就把物体碰跑了
    KnockedAway,
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
        Check::AsPlanned
    } else {
        Check::WrongSection
    }
}

/// 一次自查之后该怎么办。**换下一个候选**只在"夹错了段"时有意义 ——
/// 合到底说明这一招不对(平躺薄件),碰跑了说明该重新看一眼世界。
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Next {
    /// 继续这个动词的后续步骤
    Proceed,
    /// 退回 Clear,换下一个下手点
    NextContact,
    /// 这一招对这个物体不成立,换动词(平躺薄件 ⇒ Pry)
    ChangeVerb(Verb),
    /// 世界变了,重新看一眼再决定
    Relook,
}

/// 自查之后走哪一步。合到底 ⇒ 换招式(不是换地方),这是实测换来的。
pub fn decide(v: Verb, c: Check) -> Next {
    match c {
        Check::AsPlanned => Next::Proceed,
        Check::WrongSection => Next::NextContact,
        Check::KnockedAway => Next::Relook,
        // 合到底 = 指间空的。对要合爪的动词,这通常意味着物体平贴支撑面、爪子伸不到它下面
        // ⇒ 换成"先撬起一边"。对不合爪的动词,这个判读不适用。
        Check::ClosedOnAir => {
            if v.closes_jaw() {
                Next::ChangeVerb(Verb::Pry)
            } else {
                Next::NextContact
            }
        }
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
    fn classify_matches_the_measured_episodes() {
        // 剪刀(成功):打算夹 0.009,爪值 0.1281,张开度 0.088 ⇒ 实夹 0.0113
        assert_eq!(
            classify(0.009, 0.1281, 0.088, 0.0, 0.03, 0.5),
            Check::AsPlanned
        );
        // 贝壳:打算夹 0.0287,实夹 0.0384 ⇒ 夹错了段
        assert_eq!(
            classify(0.0287, 0.0384 / 0.088, 0.088, 0.0, 0.03, 0.2),
            Check::WrongSection
        );
        // 铲子第二三次:爪值 0 ⇒ 合到底
        assert_eq!(
            classify(0.0835, 0.0, 0.088, 0.0, 0.03, 0.2),
            Check::ClosedOnAir
        );
        // 漆滚:物体被推走 18.9 cm
        assert_eq!(
            classify(0.0386, 0.3, 0.088, 0.189, 0.03, 0.2),
            Check::KnockedAway
        );
    }

    #[test]
    fn closed_on_air_while_grasping_means_change_the_verb() {
        assert_eq!(
            decide(Verb::Grasp, Check::ClosedOnAir),
            Next::ChangeVerb(Verb::Pry)
        );
        // 推不合爪,这个判读对它不适用
        assert_eq!(decide(Verb::Push, Check::ClosedOnAir), Next::NextContact);
    }

    #[test]
    fn verbs_round_trip() {
        for i in 0..13u32 {
            assert!(Verb::from_u32(i).is_some(), "verb {i} missing");
        }
        assert!(Verb::from_u32(13).is_none());
    }
}

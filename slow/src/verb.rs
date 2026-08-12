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

/// **物体**要怎么动 —— 接触集的第三格(`ARCH.md §零`)。
///
/// 名字是 `Motion` 不是 `Twist`,因为动词表里已经有一个 `Verb::Twist`(拧),
/// 两个 `Twist` 在同一个文件里读起来会骗人。
///
/// # 🔴 这一格就是旧接口结构上说不出的那句话
///
/// 旧接口一句话是「末端走到这个位姿,爪子合到 0.3」。要让物体转,只能靠腕关节自己转,
/// 而**"转到多少度"这个目标不在那个控制律追的量里** —— 它追位置,转角是副产品。
/// 实测:拧 **16 次成 0 次**,`LAB` 记的死因是"朝向不在控制律的不动点里"。
///
/// 这里把转角变成**被要求的量本身**。注意每一个分量说的都是**物体**,一个字不提机器人 ——
/// 那正是它能跨机体的原因。
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Motion {
    /// 物体要平移的方向(单位向量)。
    pub along: [f64; 3],
    /// 沿那个方向平移多少米。0 表示"不许平移"。
    pub dist_m: f64,
    /// 物体要绕的轴(单位向量,过接触集的中心)。
    pub about: [f64; 3],
    /// 绕那条轴转多少弧度。**符号承重**:正负是拧紧还是拧松,那是眼睛说的。
    pub turn_rad: f64,
}

impl Motion {
    /// 什么都不动。
    pub const STILL: Motion = Motion {
        along: [0.0; 3],
        dist_m: 0.0,
        about: [0.0, 0.0, 1.0],
        turn_rad: 0.0,
    };

    /// 这条旋量里有没有**非零的转动分量**。
    ///
    /// 🔴 `ARCH.md §六` 炮一的**装置闸**:接触集里必须出现非零转动分量,出现了才算这一炮
    /// 真的在测新接口 —— 否则跑的还是旧接口,只是换了个名字。
    pub fn rotates(&self) -> bool {
        self.turn_rad != 0.0 && self.about.iter().any(|c| *c != 0.0)
    }
}

/// 手上有两条轴,**它们买到的东西完全不同**。这是本仓花了一晚上才分清的一件事。
///
/// 🔴 `LAB` "TWIST 判词反转":*"绕【工具轴】转买不到 `is_axis_up`(全榜第一需求,95 次)"*
/// —— 俯抓时工具轴≈竖直,绕它转 = **把物体当转盘原地打转**;
/// *"绕【钳口轴】(近水平)才买得到"*。实测:工具轴 90° 让 `axis_up` 79.84→83.66(几乎没动);
/// 钳口轴 90° 让它 78.23→**13.36**(度,越小越正)。
///
/// ⇒ **「拧」和「扳倒」是两个不同的自由度,补上前者不等于补上后者。**
#[derive(Clone, Copy, Debug)]
pub struct Axes {
    /// **工具轴** —— 俯抓时近乎竖直。绕它转 = 物体原地自转(拧盖、拧螺丝)。
    pub tool: [f64; 3],
    /// **钳口轴** —— 两片爪面分开的方向,近乎水平。绕它转 = 把物体**扳倒 / 立起来 / 倾倒**。
    pub jaw: [f64; 3],
}

/// 一个动词 + 眼睛给的参数 → **物体**该怎么动。
///
/// 🔴 **两条轴一起传进来,由这里挑** —— 不是让调用方挑。
/// 让调用方挑就等于把上面那条教训交还给"人记不记得",而它已经错过一次了。
/// `amount` 由眼睛给(转多少 / 推多远)。这一层不认识"瓶子",只认识"绕这条轴转这么多"。
pub fn demand(v: Verb, ax: Axes, dir: [f64; 3], amount: f64) -> Motion {
    match v {
        // 物体在支撑面上平移
        Verb::Push | Verb::Wipe => Motion { along: dir, dist_m: amount, ..Motion::STILL },
        // 拧:物体绕**它自己的轴**原地自转(瓶盖、螺丝)⇒ 工具轴
        Verb::Twist => Motion { about: ax.tool, turn_rad: amount, ..Motion::STILL },
        // 倒 / 翻 / 撬:把物体**扳过去**,改变它的朝向 ⇒ 钳口轴。
        // 🔴 这三个写成工具轴是本仓犯过的错,`axis_up` 几乎没动。
        Verb::Pour | Verb::Flip | Verb::Pry => {
            Motion { about: ax.jaw, turn_rad: amount, ..Motion::STILL }
        }
        // 沿工具轴往里走
        Verb::Insert => Motion { along: ax.tool, dist_m: amount, ..Motion::STILL },
        // 抓 / 放 / 够 / 舀 / 松:物体跟着手走,方向由上层给
        Verb::Grasp | Verb::Place | Verb::Reach | Verb::Scoop | Verb::Release => {
            Motion { along: dir, dist_m: amount, ..Motion::STILL }
        }
        // 按 / 敲:物体不动,力才是重点
        Verb::Press => Motion::STILL,
    }
}

/// 转要在抬之前还是之后。
///
/// 🔴 `LAB` "TWIST 陷阱":*"θ=70 与 θ=90 的读数【逐位相同】⇒ 天花板不是手臂,是 benchmark 的
/// 抬起判据"* —— 扳倒会把物体原点抬高 2.5 cm,而抬升相已经用掉 0.077(判据阈 0.10),
/// 于是**台子在扭转中途就把这一集结束了**,而"被截断"和"手转不动"在记录里**完全同形**。
/// 修法记在那一条里:*"把转挪到抬起之前拿回全部 10 cm 余量"*。
pub fn turn_before_lift(v: Verb) -> bool {
    // 会改变朝向的动词都要往前挪:它们本身就会顺带抬高物体。
    matches!(v, Verb::Pour | Verb::Flip | Verb::Pry | Verb::Twist)
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
    fn twisting_a_cap_demands_a_real_rotation_of_the_object() {
        // 🔴 炮一的装置闸:接触集里必须出现【非零转动分量】。
        //    旧接口在同一套东西上 16 成 0,死因是"朝向不在控制律的不动点里" —— 它说不出这句话。
        let ax = Axes { tool: [0.0, 0.0, 1.0], jaw: [1.0, 0.0, 0.0] };
        let t = demand(Verb::Twist, ax, [0.0; 3], -1.5708);
        assert!(t.rotates(), "拧必须要求物体真的转");
        assert_eq!(t.dist_m, 0.0, "拧不该顺带要求平移");
        // 符号承重:拧紧和拧松是同一个动词的两个方向,那是眼睛说的,不是这里定的。
        assert!(demand(Verb::Twist, ax, [0.0; 3], 1.5708).turn_rad > 0.0);
        // 拧走【工具轴】:瓶盖绕它自己的轴自转
        assert_eq!(t.about, ax.tool);
    }

    #[test]
    fn pushing_demands_translation_and_no_rotation() {
        let ax = Axes { tool: [0.0, 0.0, 1.0], jaw: [1.0, 0.0, 0.0] };
        let t = demand(Verb::Push, ax, [1.0, 0.0, 0.0], 0.12);
        assert!(!t.rotates(), "推不该要求物体转");
        assert_eq!(t.dist_m, 0.12);
    }

    #[test]
    fn pressing_demands_the_object_stays_put() {
        let ax = Axes { tool: [0.0, 0.0, 1.0], jaw: [1.0, 0.0, 0.0] };
        assert_eq!(demand(Verb::Press, ax, [0.0, 0.0, -1.0], 0.02), Motion::STILL);
    }

    #[test]
    fn tipping_verbs_must_use_the_jaw_axis_not_the_tool_axis() {
        // 🔴 LAB "TWIST 判词反转":绕工具轴转买不到 `is_axis_up`(全榜第一需求,95 次)——
        //    俯抓时工具轴≈竖直,绕它转就是把物体当转盘原地打转。
        //    实测 工具轴 90°: 79.84→83.66(几乎没动);钳口轴 90°: 78.23→13.36。
        let ax = Axes { tool: [0.0, 0.0, 1.0], jaw: [1.0, 0.0, 0.0] };
        for v in [Verb::Pour, Verb::Flip, Verb::Pry] {
            let m = demand(v, ax, [0.0; 3], 1.5708);
            assert_eq!(m.about, ax.jaw, "{v:?} 必须绕钳口轴,绕工具轴买不到朝向");
        }
        // 拧是另一个自由度:它要的就是原地自转
        assert_eq!(demand(Verb::Twist, ax, [0.0; 3], 1.5708).about, ax.tool);
    }

    #[test]
    fn turning_comes_before_lifting() {
        // 🔴 LAB "TWIST 陷阱":扳倒会把物体抬高 2.5 cm,而抬升相已用掉 0.077(阈 0.10)⇒
        //    台子在扭转【中途】结束本集,而"被截断"和"手转不动"在记录里完全同形。
        for v in [Verb::Pour, Verb::Flip, Verb::Pry, Verb::Twist] {
            assert!(turn_before_lift(v), "{v:?} 的转必须排在抬之前");
        }
        assert!(!turn_before_lift(Verb::Grasp));
    }

    #[test]
    fn verbs_round_trip() {
        for i in 0..13u32 {
            assert!(Verb::from_u32(i).is_some(), "verb {i} missing");
        }
        assert!(Verb::from_u32(13).is_none());
    }
}

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
        // 走过头捏了更细的一处 ⇒ 下手点选错了,换下一个候选。
        Check::PinchedThinner => Next::NextContact,
        // 钳口没走到那一段 ⇒ 换地方没用,该换招式(平躺薄件先撬起一边)。
        Check::StoppedWide => {
            if v.closes_jaw() {
                Next::ChangeVerb(Verb::Pry)
            } else {
                Next::NextContact
            }
        }
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
    /// 🔴 **要顶多硬** —— 不是牛顿,是**这具身体自己那把尺上的位置**,∈[0,1]:
    /// `0` = 刚好能被认出"碰上了"的那一档(下界由 `probe::contact_threshold` 量出来),
    /// `1` = 这具身体顶到不能再顶的那一档(上界由"命令还在发、读数不再变"量出来)。
    ///
    /// 为什么不是牛顿:**这具机体没有力通道**(七个关节命令全零响应,只有末端认;
    /// `fast::admit` 那个 `force_newton` 从头到尾传的是 `0.0`)。写成牛顿就得先假设
    /// 一个刚度,那是身体假设;写成"自己尺上的位置",换一具身体照样成立,
    /// 而两个端点都是**量**出来的,不是拍的。
    pub press: f64,
    /// 🔴 **到位之后按住多久**(秒)。`0` = 到了就走。
    ///
    /// 擦桌子要按住走完一段、按压要按住、裸绞要持续力 + 时间 —— 这三样以前**没地方表达**。
    /// 秒是这一层唯一不含身体的时间单位;换算成拍由驱动用**量出来的帧时**做(它每 50 帧
    /// 自己报一次 `等帧 xxx ms/帧`),不写死频率。
    pub hold_s: f64,
    /// 🔴 **什么时候算完**。以前只有一种(走完就停),于是"推到推不动"、"合到夹住"、
    /// "压到碰上"全都只能写成"猜一个距离然后走完"。
    pub until: Until,
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

impl Motion {
    /// 什么都不动。
    pub const STILL: Motion = Motion {
        along: [0.0; 3],
        dist_m: 0.0,
        about: [0.0, 0.0, 1.0],
        turn_rad: 0.0,
        // 不顶(顶的下界都不到)· 不按住 · 走完就停 —— 三个新维度的"什么都不做"档。
        press: 0.0,
        hold_s: 0.0,
        until: Until::Amount,
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
    fn classify_matches_the_measured_episodes() {
        // 剪刀(成功):打算夹 0.009,爪值 0.1281,张开度 0.088 ⇒ 实夹 0.0113
        assert_eq!(
            classify(0.009, 0.1281, 0.088, 0.0, 0.03, 0.5),
            Check::AsPlanned
        );
        // 贝壳:打算夹 0.0287,实夹 0.0384 ⇒ 比值 1.34 > 1
        // 🔴 2026-08-13 劈开之后,这一例落在【钳口没走到那一段】,该换动词而不是换下手点。
        //    劈开依据是实测 n=185 两组零重叠:锤子 2.50 / 扳手 1.72 vs 钳子 0.06 / 卷尺 0.32。
        assert_eq!(
            classify(0.0287, 0.0384 / 0.088, 0.088, 0.0, 0.03, 0.2),
            Check::StoppedWide
        );
        // 反面也必须有:走过头捏到更细的一处(钳子 0.06 那一档)⇒ 换下手点才有意义。
        assert_eq!(
            classify(0.0500, 0.0030 / 0.088, 0.088, 0.0, 0.03, 0.2),
            Check::PinchedThinner
        );
        // 两档的下一步必须相反,否则劈开没有意义。
        assert_eq!(decide(Verb::Grasp, Check::StoppedWide), Next::ChangeVerb(Verb::Pry));
        assert_eq!(decide(Verb::Grasp, Check::PinchedThinner), Next::NextContact);
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

    #[test]
    fn verbs_round_trip() {
        for i in 0..13u32 {
            assert!(Verb::from_u32(i).is_some(), "verb {i} missing");
        }
        assert!(Verb::from_u32(13).is_none());
    }
}

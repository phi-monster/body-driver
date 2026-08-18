//! What this body still has to measure about itself, and in what order.
//!
//! # What was missing without this file
//!
//! There were probes and there was an executor, and nothing in between. Every rig had to know, by
//! hand, which quantities to probe, in which order, and when one had gone bad. That is the same
//! shape as the hand-welded gates [`crate::refuse`] replaced — it works exactly as well as somebody
//! remembering to do it, and this project recorded **seven** cases in one night of an apparatus
//! that was never actually built while every reading was green.
//!
//! Plugging in a new machine is now: `plan()` → run the probes it names → `submit()` each → repeat
//! until `is_ready()`. Nothing about the order is typed in per robot.
//!
//! # 🔴 The prerequisite table is not a per-robot constant
//!
//! [`prerequisites`] says *which quantity is defined against which* — the hand point is a pixel in
//! the camera's frame, so it cannot be measured before the image Jacobian exists; the arm's weight
//! is a deficit that grows with the **lever arm**, so it cannot be measured before the base the
//! lever arm is measured from. Those are facts about the **quantities**, not about any body. The
//! test is the one the rest of the layer uses: *move to another robot and not one line changes.*
//!
//! 🔴 The example above used to read *"a contact threshold on a joint signal carries the gravity
//! load, so it cannot be measured before the arm's weight"*. That is a **torque** judge's reason,
//! and ours is a displacement ratio in which gravity cancels. Kept here as a warning, because the
//! sentence was true-sounding, load-bearing, and wrong for two full days.
//!
//! # The cascade is the part that cannot be left to a person
//!
//! Re-measuring the image Jacobian invalidates everything measured against it — the hand point, the
//! gripper span, the occlusion map — **even though none of their own clocks moved**. So a plan that
//! re-measures the Jacobian schedules those too, before they have gone bad, rather than reporting
//! them as fine now and broken later. A rule enforced in one place everything passes through is a
//! rule; a rule everybody is supposed to remember is not.

use crate::measurement::Quantity;
use crate::Body;

/// Why a quantity is on the plan. Each is a distinct fact and they are never merged: "I have never
/// measured this" and "what I measured this against has moved" call for the same probe but mean
/// very different things in an audit trail.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[repr(u32)]
pub enum Need {
    /// No value on this body yet.
    NeverMeasured = 0,
    /// Older than its declared validity window.
    Stale = 1,
    /// Something it was measured against has been re-measured, or is about to be.
    DependencyMoved = 2,
    /// Its own self-test does not pass right now.
    SelfTestFailed = 3,
}

impl Need {
    /// Stable human-readable name. For logs and audit trails; never parsed.
    pub fn as_str(self) -> &'static str {
        match self {
            Need::NeverMeasured => "never_measured",
            Need::Stale => "stale",
            Need::DependencyMoved => "dependency_moved",
            Need::SelfTestFailed => "selftest_failed",
        }
    }
}

/// Which quantities must already hold before this one can be measured at all.
///
/// Structural, not physical: it records what a quantity is **expressed in terms of**. Every entry
/// is justified in one clause, and an entry that cannot be justified in one clause is a coupling
/// somebody assumed rather than a dependency that exists.
pub fn prerequisites(q: Quantity) -> &'static [Quantity] {
    use Quantity::*;
    match q {
        // 原位是**这具身体自己**的一个位形:上电归位后读一次就有了。
        // 🔴 它不需要相机、不需要力、不需要任何别的量 —— 空前置是**结论**,不是遗漏:
        //    给它挂一个前置(比如 ImageJacobian),就等于说"没有相机就不知道自己在哪",
        //    而那对一具有关节编码器的身体是假的,并且会让一台没相机的机器永远回不了家。
        HomePose => &[],
        // A hand point is a pixel in the camera's frame; without the Jacobian there is no frame to
        // express it in and no way to tell a hand from an elbow by how it responds.
        HandPixel => &[ImageJacobian],
        // The span is read off the image and converted with a ruler derived from the Jacobian.
        // 🔴 而且**先得让钳口在画面里**。这一格的手臂全程不动(只有钳口在动),所以它停在
        //    哪儿就一直在哪儿;而"哪儿"是上一相留下的位置,完全可能贴着画幅边缘。
        //    实测(2026-08-17):`hand_pixel = (0.413, 0.990)` —— v=0.99 是画面**底边**,
        //    两根手指半个在画外 ⇒ 双响 0、配对 0、整相零样本,而噪声地板只有 21(很干净)。
        //    ⇒ 开采之前要把手挪到画幅中间,而那需要知道**手现在在画面哪一点**。
        GripperSpan => &[ImageJacobian, HandPixel],
        // The occlusion map is a map of the camera's frame.
        SelfOcclusion => &[ImageJacobian],
        // 🔴 **这条依赖曾经写成 `&[ArmWeight]`,而那是给【力矩式】接触判据写的。**
        // 那句理由——"关节给得出的接触信号里含着重力负载"——对读关节力矩的判据成立;
        // 我们的判据不读力矩,它读的是**交付比例**:命令降 10 mm、实到降 1.8 mm。重力
        // 不进这个比值——分子分母是同一段位移的两端,它同增同减。而自由空间里因自重
        // 造成的下垂,**已经整个落在 `StepDelivery` 那 0.883 里**:接触判据比的就是
        // "现在这一步的交付,和自由空间的交付差多少"。
        // 代价照记(2026-08-17):挂着 ArmWeight 时,接触阈**已经量出 0.117、曲线干净**
        // (自由 4.5 / 碰上 1.8 / 抬起 3.9),却在 `submit` 被 `UnmeasuredDependency` 挡回;
        // 而 ArmWeight 在没有力矩通道的机器上**永远量不到** ⇒ 这一格连同 `Floor` 被
        // 一条写错的依赖永久锁死,日志上看起来却像身体自己说测不了。
        ContactThreshold => &[StepDelivery],
        // The arc the working point sweeps is read in the camera's frame and converted with the
        // same ruler the span uses.
        ToolAxisColumn => &[ImageJacobian],
        // 🔴 偏置要绕**垂直于工具轴**的那一列去转才扫得出半径。绕工具轴自己转,工作点原地
        //    不动、弧半径是 0 —— 拿那一列去拟,量到的是"这具身体没有工具",而它明明有,
        //    并且那个 0 长得和一个真正没有偏置的工具一模一样。
        ToolOffset => &[ImageJacobian, ToolAxisColumn],
        // 🔴 The floor is read as a stop in the delivered-motion signal, so it inherits that
        // ruler's dependency chain -- re-measure the contact threshold and the floor map built on
        // top of it is no longer trustworthy, automatically, without anyone remembering.
        Floor => &[ContactThreshold, StepDelivery],
        // 🔴 摩擦要先能【夹住】再谈滑不滑:判据是"东西还在不在钳口里",而那要拿钳口的
        //    实际开度去读 —— 没有跨度这把尺,"夹住了"和"合到底了"读起来一模一样。
        //    它**不**依赖接触阈:倾到滑是几何,不是接触事件。
        Friction => &[GripperSpan],
        // 🔴 手眼要靠"晃钳口,看什么跟着动"来认,而那要先分得开**动了**和**还没停**。
        //    协议里那两拍空转量的是相机自己的噪声地板,只有在身体真静下来之后才成立;
        //    "静下来要几拍"由延迟和交付率算(`derive::settle_periods`),所以这两条是
        //    结构上的前置,不是保险。
        //    代价照记(2026-08-17):静置写死 2 拍,而这具身体每拍交付 0.888 ⇒ 挪完一步
        //    第 2 拍还剩那一步的 1.2%(5 cm 步 ⇒ 0.6 mm ⇒ 约半个像素),而噪声地板取的是
        //    **逐像素最大差**,高对比边缘上半个像素就把它顶到 200/255 ⇒ 空转从来不空 ⇒
        //    认块器每次都答"没有候选" ⇒ **认手/跨度/工具偏置/工具轴/自遮挡五格一起欠着**。
        ImageJacobian => &[Latency, StepDelivery],
        // 🔴 臂重是"往上走和往下走的交付比例之差,随**力臂**线性增长",而力臂 =
        //    离基座多远;基座是 `reach` 那一格从"够不着的那几个点"反解出来的。
        //    ⇒ 没有基座就没有力臂,而没有力臂时**重量和一个力臂无关的偏置完全共线**。
        ArmWeight => &[Reach],
        Latency | Backlash | Reach | StepDelivery => &[],
    }
}

/// An ordered list of what to measure now.
#[derive(Copy, Clone, Debug)]
pub struct Plan {
    /// Quantities to probe, dependencies first.
    pub order: [(Quantity, Need); Quantity::COUNT],
    /// Used length of `order`.
    pub n: usize,
}

impl Plan {
    /// The quantities and reasons, in order.
    pub fn steps(&self) -> &[(Quantity, Need)] {
        &self.order[..self.n]
    }
}

/// Does this quantity need (re-)measuring right now, judged only from what is stored?
///
/// This is the same set of conditions [`crate::refuse::admit`] refuses on, asked ahead of time
/// instead of at the moment of use. Deliberately the same list: a scheduler that used a different
/// rule from the gate would leave a body that plans as ready and refuses in service.
fn direct_need(body: &Body, q: Quantity, now_ns: u64) -> Option<Need> {
    let m = body.get(q)?;
    if !m.selftest_passed {
        return Some(Need::SelfTestFailed);
    }
    if m.is_stale(now_ns) {
        return Some(Need::Stale);
    }
    for dep in m.deps.iter().flatten() {
        let (dq, epoch_at_measure) = *dep;
        match body.get(dq) {
            None => return Some(Need::DependencyMoved),
            Some(dm) if dm.epoch != epoch_at_measure => return Some(Need::DependencyMoved),
            Some(_) => {}
        }
    }
    None
}

/// Everything this body still owes itself, ordered so each probe's prerequisites come first.
///
/// An empty plan means every quantity this layer knows how to measure is currently valid. It does
/// **not** mean the body carries no hand-set constants — see [`crate::debt`], which counts the ones
/// that never came near this API and are therefore invisible to everything here.
pub fn plan(body: &Body, now_ns: u64) -> Plan {
    let mut need: [Option<Need>; Quantity::COUNT] = [None; Quantity::COUNT];
    for i in 0..Quantity::COUNT {
        let Some(q) = Quantity::from_u32(i as u32) else {
            continue;
        };
        need[i] = match body.get(q) {
            None => Some(Need::NeverMeasured),
            Some(_) => direct_need(body, q, now_ns),
        };
    }

    // 🔴 The cascade, as a fixpoint. If a prerequisite is going to be re-measured, everything
    // expressed in terms of it is going to be invalid the moment that happens — so it is scheduled
    // now, while the plan is being made, not discovered later by whoever happens to call `admit`.
    // At most COUNT passes: each pass either marks something new or the set has closed.
    for _ in 0..Quantity::COUNT {
        let mut changed = false;
        for i in 0..Quantity::COUNT {
            if need[i].is_some() {
                continue;
            }
            let Some(q) = Quantity::from_u32(i as u32) else {
                continue;
            };
            if prerequisites(q).iter().any(|p| need[*p as usize].is_some()) {
                need[i] = Some(Need::DependencyMoved);
                changed = true;
            }
        }
        if !changed {
            break;
        }
    }

    // Topological emit: a quantity may go out once every prerequisite that is also on the plan has
    // already gone out. Prerequisites that are fine and not on the plan impose no ordering.
    let mut placed = [false; Quantity::COUNT];
    let mut order = [(Quantity::HandPixel, Need::NeverMeasured); Quantity::COUNT];
    let mut n = 0usize;
    for _ in 0..Quantity::COUNT {
        let mut progressed = false;
        for i in 0..Quantity::COUNT {
            let Some(why) = need[i] else { continue };
            if placed[i] {
                continue;
            }
            let Some(q) = Quantity::from_u32(i as u32) else {
                continue;
            };
            let ready = prerequisites(q)
                .iter()
                .all(|p| need[*p as usize].is_none() || placed[*p as usize]);
            if ready {
                order[n] = (q, why);
                n += 1;
                placed[i] = true;
                progressed = true;
            }
        }
        if !progressed {
            // Unreachable while `prerequisites` is a DAG, and it is one by construction. Stopping
            // rather than looping means a table edited into a cycle produces a SHORT plan, which is
            // visible, instead of a hang in the layer that answers "may this body move".
            break;
        }
    }
    Plan { order, n }
}

/// 🔴🔴 **闲着的时候该重量哪一格** —— "越用越强"的那一半,直到 2026-08-18 都不存在。
///
/// # 为什么之前不可能变强
///
/// [`plan`] 只排"还欠着的"格。一格量到之后,除非它过期([`Measurement::is_stale`])或者
/// 前置换了版,**日程再也不会问它第二次** —— 而全仓除了 `image_jacobian` 之外
/// **每一格的 `valid_for_ns` 都是 0(永不过期)**。于是 [`Body::submit`] 里那道
/// `WorseThanStored` 闸**一次都不可能触发**:第二次测量根本不存在,没有东西可比。
/// 机器"量一次定终身",不是"越用越强"。
///
/// # 排序的规矩:两项都是**量出来的**,没有新常数
///
/// ① **相对不确定度大的先重量** —— 那是这具身体对自己最没把握的一处。
/// ② 并列时**最久没量的先** —— 保证每一格都轮得到,不会有一格永远排不上。
///
/// 安全性由 [`Body::submit`] 兜:重量出来的行**每一项都更差**就被挡回去,不会把好的覆盖掉。
/// 所以这个函数**只需要挑得合理,不需要挑得对** —— 挑错了的代价是一次白跑,不是一次损坏。
///
/// 返回 `None` 只在这具身体一格都还没量到时。
pub fn weakest(body: &Body) -> Option<Quantity> {
    let mut best: Option<(Quantity, f64, u64)> = None;
    for i in 0..Quantity::COUNT {
        let Some(q) = Quantity::from_u32(i as u32) else { continue };
        let Some(m) = body.get(q) else { continue };
        // 相对不确定度:值为零时退回绝对值(否则会除出无穷,把一格顶到永远第一)。
        let v = m.value[0].abs();
        let u = m.uncertainty[0].abs();
        let rel = if v > 1e-12 { u / v } else { u };
        let better = match best {
            None => true,
            Some((_, r0, e0)) => rel > r0 || (rel == r0 && m.epoch < e0),
        };
        if better {
            best = Some((q, rel, m.epoch));
        }
    }
    best.map(|(q, _, _)| q)
}

/// The next single thing to measure, or `None` if this body is currently complete.
pub fn next(body: &Body, now_ns: u64) -> Option<(Quantity, Need)> {
    let p = plan(body, now_ns);
    if p.n == 0 {
        None
    } else {
        Some(p.order[0])
    }
}

/// Is every quantity this layer knows how to measure currently valid on this body?
///
/// 🔴 Read the name narrowly. It answers "has the measuring half finished", not "is this body free
/// of hand-set constants" — those are different questions and [`crate::debt`] answers the second.
pub fn is_ready(body: &Body, now_ns: u64) -> bool {
    plan(body, now_ns).n == 0
}

//! **一个接触集说不完的那三件事:一串 · 并存 · 过渡。**
//!
//! # 🔴 先订正我自己写过的一句话
//!
//! 我把 擦/舀 的空格命名成 `NoRigidObject`(*"被作用的是一片区域/一堆散料,
//! 没有一个有位姿的物体"*)。**这个命名是错的,而且错在关键处。**
//!
//! 擦是**握着抹布**擦、舀是**握着勺子**舀 —— **工具本身就是刚体,而且它的运动就是一个旋量**。
//! 第③格填得满。真正说不出的是"那片区域变干净了没有 / 兜起了多少",
//! 而那是**任务的判据**,从来就不是接触集的活(接触集只说"物体怎么动",不说"世界变成什么样")。
//!
//! ⇒ 擦/舀 缺的**不是**"有没有刚体",是**【一串】这个结构** —— 一个接触集只说得出一段运动。
//!
//! # 三件事,各自的形状
//!
//! | 说不出的 | 形状 | 例 |
//! |---|---|---|
//! | 一段不够 | `Then` —— 按顺序做完,**执行层在两段之间自己插过渡** | 擦(来回若干道)· 舀(插进去 → 兜起来 → 抬出来) |
//! | 要同时成立 | `While` —— 一个在**维持**,另一个在**动** | 握住 + 扣扳机 · 按住纸 + 另一只手写字 |
//! | 物体不参与 | **不进接口** —— 由执行层在两个接触集之间自己产生 | 够(Reach) |
//!
//! 🔴 **`够` 没有变体,这是故意的。** 它不是"脑子对身体说的话"的一种,
//! 它是身体自己该会的事。给它一个变体,等于让脑子去操心手怎么绕过去。

use crate::{ContactSet, Gap, Who};

/// 这个接触集里【手】那几个点在哪(世界接触不算 —— 手够不到桌子底下那条边)。
fn hand_at(cs: &ContactSet) -> Vec<crate::V3> {
    cs.points.iter().filter(|p| p.by == Who::Hand).map(|p| p.at).collect()
}

/// **脑子对身体说的那句话 —— 完整形态。**
///
/// 绝大多数动词是 `One`。只有需要"一串"或"同时"的才用后两个。
#[derive(Clone, Debug, PartialEq)]
pub enum Move {
    /// 一个接触集。十三个动词里十个是它。
    One(ContactSet),
    /// **按顺序做完,段间【重新下手】。** 过渡由执行层自己产生 —— 那就是 `够`。
    ///
    /// 舀完这一勺、退开、换个地方再舀:两段之间手要松开退回悬停。
    Then(Vec<Move>),
    /// **按顺序做完,但【不松手】。** 段间不退回悬停。
    ///
    /// # 🔴 为什么必须跟 `Then` 分开(2026-08-16 由擦逼出来)
    ///
    /// 擦是握着抹布来回走,**中间一次都不松手**。若照 `Then` 走,执行层会在每一道之间
    /// 插一个"沿工具轴退开 standoff"的悬停 —— 那是**把抹布放下再拿起来**,
    /// 而且抹布多半会掉。一个字之差,动作完全不同。
    ///
    /// 代价是一条**必须成立的接续条件**:下一段的接触点,就是上一段末了那些点。
    /// 对不上就是 `KeepBreaksContact` —— 手上还握着东西,你却说下一段从别处开始。
    Keep(Vec<Move>),
    /// 🔴🔴 **"不要碰" —— 零接触点 + 一个净空。**
    ///
    /// # 为什么它必须是一个变体,而不是"一个接触集的特例"
    ///
    /// 接触集说的是*"碰这几个点"*,它说的是*"这几个地方【别】碰"*。
    /// 前者要求 `points` 非空(空了报 `NoPoints`),后者**本质上就是零接触**。
    /// 硬塞进接触集 = 让"碰"和"别碰"共用一个字段,读的人分不出来。
    ///
    /// 六条验收线的第 6 条(躲拳)原文:*"接口要能表达【不要碰】(零接触点 + 一个净空)"*。
    ///
    /// # ⚠️ 它只表达"躲",不表达"预判"
    ///
    /// 真正难的那一半不在这儿:*人反应 231 ms,而拳飞行只有 115–190 ms* ——
    /// **人躲的是起手征兆,不是拳本身**。那是③(眼)的活,这一层只负责
    /// *"给我这几个要躲开的地方 + 要留多宽,我把手挪开"*。**这条边界写在这里,不许含糊。**
    Clear {
        /// 要躲开的那些地方(世界系,米)。空的就没有意义 ⇒ 报 `NoKeepOut`。
        keep_out: Vec<crate::V3>,
        /// 至少要留多宽(米)。
        by_m: f64,
        /// 手上那些点**现在**在哪 —— 躲开是相对于"此刻在哪"说的。
        /// 🔴 由调用方给:身体层知道手在哪,而这一层不知道,**也不该知道**。
        from: Vec<crate::V3>,
    },
    /// **同时成立。** 第一个是**维持**的(它的第③格必须是"不动"),其余在动。
    ///
    /// 🔴 为什么维持的那个必须"不动":并存的意义就是*"这个别撒手,那个去动"*。
    /// 如果两个都在动,那是两只手各干各的,应当写成两条独立的 `Move`。
    While(Vec<Move>),
}

/// 一串/并存填不满时,**点名**。
#[derive(Clone, Debug, PartialEq)]
pub enum ManyGap {
    /// 里面某一段自己就填不满 —— 带上是第几段、哪一格。
    At(Vec<usize>, Gap),
    /// `Then` / `While` 里一段都没有。
    Empty(Vec<usize>),
    /// `While` 的第一段不是"维持" —— 它的第③格在动。
    HolderMoves(Vec<usize>),
    /// `While` 只有一段 ⇒ 没有"并存"可言,应当写成 `One`。
    NothingToPairWith(Vec<usize>),
    /// `Keep` 说"不松手",而下一段的接触点不在上一段末了那个位置上。
    /// 带上是第几段接不上、差多少米。
    KeepBreaksContact(Vec<usize>, usize, f64),
    /// `Keep` 前后两段的接触点**个数**都不一样 —— 不松手不可能换手指数。
    KeepChangesPointCount(Vec<usize>, usize),
    /// `Clear` 一个要躲的地方都没给 ⇒ 这句话没有内容。
    NoKeepOut(Vec<usize>),
    /// `Clear` 的净空不是一个正数,或者"手现在在哪"没给。
    BadClearance(Vec<usize>),
}

impl Move {
    /// 逐段自检。**过不了就点名是【第几段的哪一格】**,不许含糊到"这条计划不行"。
    ///
    /// `must_move`:整条计划要不要求物体动。传给每一段时的规矩:
    /// - `Then` 的每一段都按 `must_move` 判(一串里出现一段不动的,通常是写错了);
    /// - `While` 的**第一段永远按"不动"判**(它是维持的那一个),其余按 `must_move`。
    pub fn check(&self, must_move: bool) -> Result<(), ManyGap> {
        self.walk(must_move, &mut Vec::new())
    }

    fn walk(&self, must_move: bool, path: &mut Vec<usize>) -> Result<(), ManyGap> {
        match self {
            Move::One(cs) => cs.check(must_move).map_err(|g| ManyGap::At(path.clone(), g)),
            Move::Then(items) => {
                if items.is_empty() {
                    return Err(ManyGap::Empty(path.clone()));
                }
                for (i, m) in items.iter().enumerate() {
                    path.push(i);
                    let r = m.walk(must_move, path);
                    path.pop();
                    r?;
                }
                Ok(())
            }
            Move::Keep(items) => {
                if items.is_empty() {
                    return Err(ManyGap::Empty(path.clone()));
                }
                for (i, m) in items.iter().enumerate() {
                    path.push(i);
                    let r = m.walk(must_move, path);
                    path.pop();
                    r?;
                }
                // 🔴 接续条件:说了不松手,下一段就必须从上一段末了那些点接着走。
                for i in 1..items.len() {
                    let (prev, next) = (items[i - 1].end_points(), items[i].start_points());
                    if prev.len() != next.len() {
                        return Err(ManyGap::KeepChangesPointCount(path.clone(), i));
                    }
                    let mut worst = 0.0f64;
                    for (a, b) in prev.iter().zip(&next) {
                        worst = worst.max(crate::norm([a[0] - b[0], a[1] - b[1], a[2] - b[2]]));
                    }
                    // 门槛用**下一段自己声明的最严容差** —— 不另拍一个常数。
                    let tol = items[i]
                        .flatten()
                        .iter()
                        .flat_map(|c| c.points.iter())
                        .map(|p| p.tol_m)
                        .fold(f64::MAX, f64::min);
                    if worst > tol {
                        return Err(ManyGap::KeepBreaksContact(path.clone(), i, worst));
                    }
                }
                Ok(())
            }
            Move::Clear { keep_out, by_m, from } => {
                if keep_out.is_empty() {
                    return Err(ManyGap::NoKeepOut(path.clone()));
                }
                if !(by_m.is_finite() && *by_m > 0.0) || from.is_empty() {
                    return Err(ManyGap::BadClearance(path.clone()));
                }
                Ok(())
            }
            Move::While(items) => {
                if items.is_empty() {
                    return Err(ManyGap::Empty(path.clone()));
                }
                if items.len() == 1 {
                    return Err(ManyGap::NothingToPairWith(path.clone()));
                }
                for (i, m) in items.iter().enumerate() {
                    path.push(i);
                    // 第一段是维持的那一个:它必须**不动**
                    let r = if i == 0 {
                        match m.moves() {
                            true => Err(ManyGap::HolderMoves(path.clone())),
                            false => m.walk(false, path),
                        }
                    } else {
                        m.walk(must_move, path)
                    };
                    path.pop();
                    r?;
                }
                Ok(())
            }
        }
    }

    /// 这条计划里有没有任何一段要求物体动。
    pub fn moves(&self) -> bool {
        match self {
            Move::One(cs) => {
                crate::norm(cs.motion.lin) > 1e-9 || cs.motion.angle() > 1e-9
            }
            Move::Then(items) | Move::Keep(items) | Move::While(items) => {
                items.iter().any(|m| m.moves())
            }
            // 躲开是**手**在动,不是**物体**在动。第③格说的是物体 ⇒ 这里是 false。
            Move::Clear { .. } => false,
        }
    }

    /// 这一段**开头**那些手接触点在哪。
    pub fn start_points(&self) -> Vec<crate::V3> {
        match self {
            Move::One(cs) => hand_at(cs),
            Move::Then(items) | Move::Keep(items) | Move::While(items) => {
                items.first().map(|m| m.start_points()).unwrap_or_default()
            }
            Move::Clear { from, .. } => from.clone(),
        }
    }

    /// 这一段**末了**那些手接触点在哪 —— 起点让第③格搬过去。
    pub fn end_points(&self) -> Vec<crate::V3> {
        match self {
            Move::One(cs) => hand_at(cs).into_iter().map(|p| cs.motion.apply(p)).collect(),
            Move::Then(items) | Move::Keep(items) => {
                items.last().map(|m| m.end_points()).unwrap_or_default()
            }
            // 躲完手在哪由执行层算(它才知道往哪让);这一层只说"别碰那儿"。
            Move::Clear { from, .. } => from.clone(),
            // 并存:末了停在**维持**的那一段上(动的那些做完就撤,握着的没松)
            Move::While(items) => items.first().map(|m| m.end_points()).unwrap_or_default(),
        }
    }

    /// 摊平成"按时间先后的那些接触集"。**`While` 摊平后仍然是并存的**,
    /// 摊平只用来数数与遍历,不用来判先后 —— 想知道先后必须看结构本身。
    pub fn flatten(&self) -> Vec<&ContactSet> {
        let mut out = Vec::new();
        self.collect(&mut out);
        out
    }

    fn collect<'a>(&'a self, out: &mut Vec<&'a ContactSet>) {
        match self {
            Move::One(cs) => out.push(cs),
            Move::Then(items) | Move::Keep(items) | Move::While(items) => {
                for m in items {
                    m.collect(out);
                }
            }
            // 零接触点 ⇒ 摊平出来什么都没有,而那正是它的定义。
            Move::Clear { .. } => {}
        }
    }
}

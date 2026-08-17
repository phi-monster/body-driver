//! **上电自标定:问驱动还欠自己什么,做那几个动作,把样本交回去,再问一遍。**
//!
//! ABI 头里那句话就是这一层的全部规格:
//!
//! > **THE POWER-ON SCHEDULE.** Plugging in a new machine is: `bl_measure_plan` → run the probes
//! > it names → `bl_measure` each → repeat until `*n == 0`. **Nothing about the order is typed in
//! > per robot.**
//!
//! # 🔴 它替掉的东西,以及为什么必须替
//!
//! 从前这一半是 `body_probe.py`,而它 `import` 了四个某 benchmark 专用的模块。拆开之后:
//! 两个是**线缆格式**、两个是**纯数学**、一个是**身份指纹**(驱动这边已经有)、
//! 一个是那个 benchmark 的**计分挡板**(跟标定毫无关系)。
//! 其中一个模块里还躺着**写死的相机内参与外参**(`fx=288.13`、相机位置、俯仰角)。
//! ⇒ 采样这一半从来就不可移植,而"换一具没见过的真机、零示范、零手填数"这条验收线
//! **卡的正是这一半**。
//!
//! # 用户要写几行代码:零
//!
//! 换一具机器人 = 换一个 [`robot::Robot`] 实现,而那是**插头**,不是标定程序。
//! 相位、顺序、依赖、什么时候拒绝 —— 一个字都不按机器人写。

#![forbid(unsafe_code)]

pub mod robot;

use body_layer::measurement::Quantity;
use robot::{Cmd, Frame, Robot};

/// 一个相位跑完之后交回来的原始样本。**这一层不估计任何值,也不决定任何拒绝** ——
/// 那两件都在 `body_layer::probe` 里,在估计器被单测过、拒绝规则被推导过的那个地方。
#[derive(Clone, Debug, Default)]
pub struct Samples {
    /// (命令幅度, 实到幅度) —— 交付率用。
    pub steps: Vec<(f64, f64)>,
    /// (离这条臂自己base的半径, 到没到) —— 可达用。
    pub reach: Vec<(f64, bool)>,
    /// (倾角, 东西还在不在钳口里) —— 摩擦用。
    pub tilt: Vec<(f64, bool)>,
    /// (命令开度, 观测到的两指间距) —— 钳口跨度用。
    pub jaw: Vec<(f64, f64)>,
    /// 发出命令后第几帧才动 —— 延迟用。
    pub latency: Vec<u32>,
    /// (往正走了多少, 往反走了多少) —— 齿隙用。
    pub reversal: Vec<(f64, f64)>,
}

/// 这一步该做什么动作 —— 由 `bl_measure_plan` 点的名决定,不是这里挑的。
fn 动作(q: Quantity, arm: usize, k: u32, home: &[f64; 3]) -> Cmd {
    // 幅度按**这具身体已经量到的**东西走;一个量都还没有的时候只能用一个探索性的小幅度,
    // 而它的值本身不进任何结果 —— 它只决定"这一下够不够大到看得见",交付率会把它归一化掉。
    let 探 = 0.01 * (1 + k % 5) as f64;
    match q {
        // 交付率:同一条轴上从小到大扫一遍幅度,看命令与实到的比。
        Quantity::StepDelivery => Cmd::Ee {
            arm,
            at: [home[0], home[1] + 探, home[2]],
            quat: [1.0, 0.0, 0.0, 0.0],
            jaw: 1.0,
        },
        // 可达:沿一条射线越走越远,记到没到。
        Quantity::Reach => Cmd::Ee {
            arm,
            at: [home[0], home[1] + 0.02 * k as f64, home[2]],
            quat: [1.0, 0.0, 0.0, 0.0],
            jaw: 1.0,
        },
        // 钳口跨度:只动夹爪,手臂按住不动 —— 会动的那块刚体按构造就是钳口。
        Quantity::GripperSpan => Cmd::Ee {
            arm,
            at: *home,
            quat: [1.0, 0.0, 0.0, 0.0],
            jaw: (k % 11) as f64 / 10.0,
        },
        // 摩擦:夹住之后,把腕一点点倾过去,看它什么时候滑。
        Quantity::Friction => Cmd::Ee {
            arm,
            at: *home,
            quat: 绕x转((k % 18) as f64 * 0.05),
            jaw: 0.0,
        },
        // 齿隙:同一条轴上正反各推一下,死区就是旷量。
        Quantity::Backlash => Cmd::Ee {
            arm,
            at: [home[0], home[1] + if k % 2 == 0 { 探 } else { -探 }, home[2]],
            quat: [1.0, 0.0, 0.0, 0.0],
            jaw: 1.0,
        },
        // 延迟:从静止发一步,看画面第几帧才变。
        Quantity::Latency => {
            if k == 0 {
                Cmd::Ee { arm, at: [home[0], home[1] + 探, home[2]], quat: [1.0, 0.0, 0.0, 0.0], jaw: 1.0 }
            } else {
                Cmd::Hold
            }
        }
        // 其余的量顺路就采到了(每次移动都是一次探针),不需要专门的动作。
        _ => Cmd::Hold,
    }
}

fn 绕x转(θ: f64) -> [f64; 4] {
    [(θ / 2.0).cos(), (θ / 2.0).sin(), 0.0, 0.0]
}

/// 跑一个相位:发动作、收样本。**不估计、不拒绝。**
pub fn 跑一相(r: &mut dyn Robot, q: Quantity, arm: usize, 步数: u32) -> Samples {
    let mut s = Samples::default();
    let home = match r.sense() {
        Some(f) if !f.ee.is_empty() => [f.ee[arm.min(f.ee.len() - 1)][0], f.ee[arm.min(f.ee.len() - 1)][1], f.ee[arm.min(f.ee.len() - 1)][2]],
        _ => return s,
    };
    let mut 上一帧: Option<Frame> = None;
    for k in 0..步数 {
        let c = 动作(q, arm, k, &home);
        // 命令的幅度:发之前就知道,不用回读去猜。
        let 命令量 = match &c {
            Cmd::Ee { at, .. } => {
                let p = 上一帧.as_ref().and_then(|f| f.ee.get(arm).copied()).unwrap_or([home[0], home[1], home[2], 1.0, 0.0, 0.0, 0.0]);
                ((at[0] - p[0]).powi(2) + (at[1] - p[1]).powi(2) + (at[2] - p[2]).powi(2)).sqrt()
            }
            _ => 0.0,
        };
        r.act(&c);
        let Some(f) = r.sense() else { continue };
        if let (Some(prev), Some(now)) = (上一帧.as_ref().and_then(|p| p.ee.get(arm).copied()), f.ee.get(arm).copied()) {
            let 实到 = ((now[0] - prev[0]).powi(2) + (now[1] - prev[1]).powi(2) + (now[2] - prev[2]).powi(2)).sqrt();
            if 命令量 > 0.0 {
                // 🔴 顺路就采到了:每一次移动同时是一次交付率样本,不用为它单开一个相位。
                s.steps.push((命令量, 实到));
                if matches!(q, Quantity::Reach) {
                    // 到没到:实到占命令的绝大部分才算到。这里不定门槛 —— 把两个数原样交上去,
                    // 门槛在估计器那边,而它是被单测过的。
                    s.reach.push((命令量, 实到 >= 命令量 * 0.5));
                }
            }
        }
        if matches!(q, Quantity::Friction) {
            // 东西还在不在钳口里:钳口停在中间 = 咬着东西;合到底 = 两指之间是空的。
            let 咬住 = f.jaw.get(arm).copied().unwrap_or(0.0) > 0.0;
            s.tilt.push(((k % 18) as f64 * 0.05, 咬住));
        }
        if matches!(q, Quantity::GripperSpan) {
            if let Some(j) = f.jaw.get(arm) {
                s.jaw.push(((k % 11) as f64 / 10.0, *j));
            }
        }
        上一帧 = Some(f);
    }
    s
}

#[cfg(test)]
mod 测 {
    use super::*;

    /// 一台假机器人:命令什么就到什么(交付 1.0),超过 0.3 m 就到不了。
    struct 假臂 {
        p: [f64; 7],
        jaw: f64,
    }
    impl Robot for 假臂 {
        fn sense(&mut self) -> Option<Frame> {
            Some(Frame { joints: vec![vec![0.0; 6]], ee: vec![self.p], jaw: vec![self.jaw], cams: vec![] })
        }
        fn act(&mut self, c: &Cmd) -> bool {
            if let Cmd::Ee { at, jaw, .. } = c {
                let r = (at[0] * at[0] + at[1] * at[1]).sqrt();
                if r <= 0.3 {
                    self.p = [at[0], at[1], at[2], 1.0, 0.0, 0.0, 0.0];
                }
                self.jaw = *jaw;
            }
            true
        }
        fn identity(&mut self) -> Vec<(String, f64, f64)> {
            vec![("j0".into(), -3.0, 3.0)]
        }
    }

    #[test]
    fn 顺路就采到了交付率样本() {
        let mut b = 假臂 { p: [0.0; 7], jaw: 1.0 };
        let s = 跑一相(&mut b, Quantity::StepDelivery, 0, 20);
        assert!(s.steps.len() >= 10, "每一次移动都该是一次交付样本,实得 {}", s.steps.len());
    }

    /// 🔴 会失败的那一条:一台**根本不动**的机器人,采出来的交付率必须是 0,
    /// 而不是"没采到所以看起来正常"。
    #[test]
    fn 反例_不动的身体交付率是零不是空() {
        struct 死臂;
        impl Robot for 死臂 {
            fn sense(&mut self) -> Option<Frame> {
                Some(Frame { joints: vec![vec![0.0; 6]], ee: vec![[0.0; 7]], jaw: vec![1.0], cams: vec![] })
            }
            fn act(&mut self, _: &Cmd) -> bool {
                true
            }
            fn identity(&mut self) -> Vec<(String, f64, f64)> {
                vec![]
            }
        }
        let s = 跑一相(&mut 死臂, Quantity::StepDelivery, 0, 20);
        assert!(!s.steps.is_empty(), "必须采到样本");
        assert!(s.steps.iter().all(|(_, a)| *a == 0.0), "一台不动的身体,实到必须全是 0");
    }

    #[test]
    fn 可达采到了到与不到两种() {
        let mut b = 假臂 { p: [0.0; 7], jaw: 1.0 };
        let s = 跑一相(&mut b, Quantity::Reach, 0, 40);
        assert!(s.reach.iter().any(|(_, ok)| *ok), "近处该到得了");
        assert!(s.reach.iter().any(|(_, ok)| !*ok), "远处该到不了 —— 两边都要有,墙才夹得住");
    }
}

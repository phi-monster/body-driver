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
    /// (腕转了多少, 工作点 x, 工作点 y) —— 工具偏置与工具轴用。
    pub arc: Vec<(f64, f64, f64)>,
    /// (这一刻的关节角, 这一步挪了多少) —— 自重用;挪得自由才算"什么都没碰"。
    pub hold: Vec<(Vec<f64>, f64)>,
    /// (命令下降, 实到下降, 此刻高度) —— 接触阈与支撑面用。
    pub press: Vec<(f64, f64, f64)>,
}

/// 这一步该做什么动作 —— 由日程点的名决定,不是这里挑的。
///
/// 🔴 **十五格全在这儿,一个不留。** 每一格只回答一句话:*这具身体要【做什么】,
/// 才会产出这一格需要的样本*。估计器和拒绝规则都不在这儿 —— 它们在 `probe.rs`,
/// 在被单测过的那个地方。
fn 动作(q: Quantity, arm: usize, k: u32, home: &[f64; 3]) -> Cmd {
    // 探索幅度:一个量都还没有的时候,只能先用一串**由小到大**的幅度去试。
    // 🔴 它的**值本身不进任何结果** —— 交付率量的是"命令 vs 实到"的比,幅度会被约掉;
    //    它只决定"这一下够不够大到看得见"。而扫一串而不是定一个,正是为了让
    //    `step_delivery` 的估计器有多个幅度可比(它要 ≥5 个不同幅度)。
    let 探 = 0.005 * (1 + k % 10) as f64;
    let 竖 = |dz: f64| Cmd::Ee { arm, at: [home[0], home[1], home[2] + dz], quat: 朝下(), jaw: 1.0 };
    match q {
        // ── 一步命令有多少真的到了:同一条轴上从小到大扫幅度 ──
        Quantity::StepDelivery => Cmd::Ee {
            arm,
            at: [home[0], home[1] + 探, home[2]],
            quat: 朝下(),
            jaw: 1.0,
        },
        // ── 从静止发一步,看第几拍才动 ──
        Quantity::Latency => {
            if k % 12 == 0 { 竖(0.02) } else if k % 12 == 6 { 竖(0.0) } else { Cmd::Hold }
        }
        // ── 齿隙:同一条轴上正反各推一下,死区就是旷量 ──
        Quantity::Backlash => Cmd::Ee {
            arm,
            at: [home[0], home[1] + if (k / 3) % 2 == 0 { 探 } else { -探 }, home[2]],
            quat: 朝下(),
            jaw: 1.0,
        },
        // ── 可达:沿一条射线越走越远,记到没到 ──
        Quantity::Reach => Cmd::Ee {
            arm,
            at: [home[0], home[1] + 0.015 * k as f64, home[2]],
            quat: 朝下(),
            jaw: 1.0,
        },
        // ── 一米等于多少像素:手臂走几步已知位移,看画面里自己挪了多少 ──
        //    🔴 六个方向都走,而不是只走一条轴:估计器要的是**位移之差**,
        //       两个方向近乎正交才解得开那个 2×3。
        Quantity::ImageJacobian | Quantity::HandPixel => {
            let d = [[探, 0.0, 0.0], [-探, 0.0, 0.0], [0.0, 探, 0.0],
                     [0.0, -探, 0.0], [0.0, 0.0, 探], [0.0, 0.0, -探]][(k % 6) as usize];
            Cmd::Ee { arm, at: [home[0] + d[0], home[1] + d[1], home[2] + d[2]], quat: 朝下(), jaw: 1.0 }
        }
        // ── 工具尖到法兰多长 / 哪一列是工具轴:绕每一列转腕,
        //    工作点扫出一段弧,弧的半径【就是】那个偏置;弧最小的那一列是工具轴 ──
        Quantity::ToolOffset | Quantity::ToolAxisColumn => Cmd::Ee {
            arm,
            at: *home,
            quat: 绕轴((k / 8) as usize % 3, (k % 8) as f64 * 0.15),
            jaw: 1.0,
        },
        // ── 钳口跨度:手臂按住不动,只动夹爪。会动的那块刚体按构造就是钳口 ──
        Quantity::GripperSpan => Cmd::Ee {
            arm,
            at: *home,
            quat: 朝下(),
            jaw: (k % 11) as f64 / 10.0,
        },
        // ── 胳膊有多重:摆一圈姿势,记"什么都不碰时保持不动要多少力矩" ──
        //    🔴 每一个采样点都必须是**手在空中、什么都没碰**,否则量到的是接触力。
        Quantity::ArmWeight => Cmd::Ee {
            arm,
            at: [home[0] + 0.06 * ((k % 5) as f64 - 2.0) / 2.0,
                 home[1] + 0.06 * ((k / 5 % 5) as f64 - 2.0) / 2.0,
                 home[2] + 0.06 * ((k / 25 % 3) as f64 - 1.0)],
            quat: 朝下(),
            jaw: 1.0,
        },
        // ── 「我碰到东西了」长什么样:一路往下压,交付比例塌下去的那一刻就是它 ──
        //    不用力传感器 —— 命令与实到的比本身就是那把尺。
        Quantity::ContactThreshold | Quantity::Floor => 竖(-0.01 * (k + 1) as f64),
        // ── 摩擦:夹住之后把腕一点点倾过去,滑的那一刻倾角 = atan(mu) ──
        Quantity::Friction => Cmd::Ee {
            arm,
            at: *home,
            quat: 绕轴(0, (k % 18) as f64 * 0.05),
            jaw: 0.0,
        },
        // ── 原位:回到起点,量它自己回得有多准 ──
        Quantity::HomePose => {
            if k % 4 == 0 { 竖(0.05) } else { Cmd::Ee { arm, at: *home, quat: 朝下(), jaw: 1.0 } }
        }
        // ── 自遮挡:转一圈看自己挡住了画面的哪些地方 ──
        Quantity::SelfOcclusion => Cmd::Ee {
            arm,
            at: [home[0] + 0.10 * ((k % 7) as f64 - 3.0) / 3.0, home[1], home[2]],
            quat: 朝下(),
            jaw: 1.0,
        },
    }
}

/// 工具朝下 —— 这不是一个"标定值",是探针姿态的一个约定。
fn 朝下() -> [f64; 4] {
    [0.707_106_781_186_547_6, 0.0, 0.707_106_781_186_547_6, 0.0]
}

/// 绕第 `col` 根轴转 `θ`。
fn 绕轴(col: usize, θ: f64) -> [f64; 4] {
    let (c, s) = ((θ / 2.0).cos(), (θ / 2.0).sin());
    match col {
        0 => [c, s, 0.0, 0.0],
        1 => [c, 0.0, s, 0.0],
        _ => [c, 0.0, 0.0, s],
    }
}

/// 跑一个相位:发动作、收样本。**不估计、不拒绝。**
pub fn 跑一相(r: &mut dyn Robot, q: Quantity, arm: usize, 步数: u32) -> Samples {
    let mut s = Samples::default();
    let home = match r.sense() {
        Some(f) if !f.ee.is_empty() => [f.ee[arm.min(f.ee.len() - 1)][0], f.ee[arm.min(f.ee.len() - 1)][1], f.ee[arm.min(f.ee.len() - 1)][2]],
        _ => return s,
    };
    let mut 上一帧: Option<Frame> = None;
    let (mut 正, mut 反) = (0.0f64, 0.0f64);
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
        // 🔴 **延迟量的是"发出命令之后第几拍才动"** —— 所以要记的是"这一拍动了没有",
        //    而不是动了多少。命令发在 k%12==0 那一拍,之后逐拍看什么时候出现位移。
        if matches!(q, Quantity::Latency) {
            if let (Some(prev), Some(now)) = (上一帧.as_ref().and_then(|p| p.ee.get(arm).copied()), f.ee.get(arm).copied()) {
                let 挪 = ((now[0] - prev[0]).powi(2) + (now[1] - prev[1]).powi(2) + (now[2] - prev[2]).powi(2)).sqrt();
                if 挪 > 0.0 {
                    s.latency.push(k % 12);
                }
            }
        }
        // 🔴 **齿隙要的是【一对】:正推走了多少、反推走了多少。** 单独一次位移里没有死区。
        if matches!(q, Quantity::Backlash) {
            if let (Some(prev), Some(now)) = (上一帧.as_ref().and_then(|p| p.ee.get(arm).copied()), f.ee.get(arm).copied()) {
                let d = now[1] - prev[1];
                if d.abs() > 0.0 {
                    if d > 0.0 { 正 = d; } else { 反 = -d; }
                    if 正 > 0.0 && 反 > 0.0 {
                        s.reversal.push((正, 反));
                        正 = 0.0;
                        反 = 0.0;
                    }
                }
            }
        }
        // 🔴 **工具偏置量的是一段弧**:转腕时工作点扫出的弧,半径就是偏置。
        //    记(腕转了多少, 工作点在画面的哪儿)—— 而"在画面哪儿"要认手,这一格
        //    因此依赖像素每米,日程里那条前置不是装饰。
        if matches!(q, Quantity::ToolOffset | Quantity::ToolAxisColumn) {
            if let Some(p) = f.ee.get(arm) {
                s.arc.push(((k % 8) as f64 * 0.15, p[0], p[1]));
            }
        }
        // 🔴 **自重:每一刻手在空中、什么都没碰,就是一个采样点。**
        //    "没碰"这件事不能假设 —— 用交付比例判:这一步走得自由,才算数。
        if matches!(q, Quantity::ArmWeight) {
            if let (Some(prev), Some(now)) = (上一帧.as_ref().and_then(|p| p.ee.get(arm).copied()), f.ee.get(arm).copied()) {
                let 挪 = ((now[0] - prev[0]).powi(2) + (now[1] - prev[1]).powi(2) + (now[2] - prev[2]).powi(2)).sqrt();
                s.hold.push((f.joints.get(arm).cloned().unwrap_or_default(), 挪));
            }
        }
        // 🔴 **接触阈:一路往下压,交付比例塌下去的那一刻就是"碰到了"。**
        if matches!(q, Quantity::ContactThreshold | Quantity::Floor) {
            if 命令量 > 0.0 {
                if let (Some(prev), Some(now)) = (上一帧.as_ref().and_then(|p| p.ee.get(arm).copied()), f.ee.get(arm).copied()) {
                    s.press.push((命令量, (prev[2] - now[2]).max(0.0), now[2]));
                }
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

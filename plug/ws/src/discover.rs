//! **认出这台机器人的观测长什么样 —— 不靠键名。**
//!
//! # 为什么不许读键名
//!
//! 键名是**某一台机器 / 某一个仿真**的事。一旦驱动里写着 `left_arm_joint_state`,
//! 它就只对报这个名字的机器有效,而"装上就能用"这句话立刻不成立。
//! 上一版的采样程序正是这么焊死的,连**写死的相机内参**都跟着进来了。
//!
//! # 靠什么认
//!
//! **形状**。一台机器人报回来的东西,形状本身就说明了它是什么:
//!
//! | 看到的 | 认成 |
//! |---|---|
//! | 一串 6–7 个浮点,值域在关节限位量级(±2π) | 一条臂的**关节角** |
//! | 7 个浮点,后 4 个模长 ≈ 1 | 末端**位姿**(位置 + 单位四元数) |
//! | 单独 1 个浮点,在 [0,1] | **夹爪开度** |
//! | 一片 u8,长度 = 宽×高×3 | 一台**相机** |
//!
//! # 🔴 认不出来就拒绝,不许挑一个看起来像的
//!
//! 两个候选分不开时返回 [`Layout::ambiguous`],调用方**拒绝开跑**。
//! 一个认错了的布局会让整条自标定"照常跑完并产出一批看起来正常的数",
//! 而那种数比没有数更贵 —— 它会被当成量出来的东西用下去。

use rmpv::Value;

/// 认出来的布局:每一格记的是**到那个值的路径**,不是它的名字。
#[derive(Clone, Debug, Default)]
pub struct Layout {
    pub joints: Vec<Vec<String>>,
    pub ee: Vec<Vec<String>>,
    pub jaw: Vec<Vec<String>>,
    pub cams: Vec<Vec<String>>,
    /// 分不开的那些,原样报出来给人看。
    pub ambiguous: Vec<String>,
}

fn 走(v: &Value, path: &mut Vec<String>, out: &mut Vec<(Vec<String>, Value)>) {
    match v {
        Value::Map(m) => {
            for (k, sub) in m {
                let name = k.as_str().unwrap_or("?").to_string();
                path.push(name);
                走(sub, path, out);
                path.pop();
            }
        }
        other => out.push((path.clone(), other.clone())),
    }
}

fn 浮点串(v: &Value) -> Option<Vec<f64>> {
    let a = v.as_array()?;
    let mut o = Vec::with_capacity(a.len());
    for x in a {
        o.push(x.as_f64()?);
    }
    Some(o)
}

/// 从一帧观测里认出布局。**只看形状与值域,不看名字。**
pub fn 认(obs: &Value) -> Layout {
    let mut flat = Vec::new();
    走(obs, &mut Vec::new(), &mut flat);
    let mut l = Layout::default();
    for (path, v) in &flat {
        // 相机:一片 u8,而且长度分解得出 宽×高×3。
        if let Value::Binary(b) = v {
            if b.len() % 3 == 0 && b.len() >= 3 * 64 * 64 {
                l.cams.push(path.clone());
                continue;
            }
        }
        let Some(xs) = 浮点串(v) else { continue };
        match xs.len() {
            // 位姿:位置三个 + 单位四元数四个。四元数那一段的模长把它和"7 个关节角"分开。
            7 => {
                let n = (xs[3] * xs[3] + xs[4] * xs[4] + xs[5] * xs[5] + xs[6] * xs[6]).sqrt();
                if (n - 1.0).abs() < 1e-3 {
                    l.ee.push(path.clone());
                } else if xs.iter().all(|x| x.abs() <= 7.0) {
                    l.joints.push(path.clone());
                } else {
                    l.ambiguous.push(path.join("."));
                }
            }
            // 关节角:值域在限位量级。
            6 => {
                if xs.iter().all(|x| x.abs() <= 7.0) {
                    l.joints.push(path.clone());
                } else {
                    l.ambiguous.push(path.join("."));
                }
            }
            // 夹爪:单独一个,归一化。
            1 => {
                if (0.0..=1.0).contains(&xs[0]) {
                    l.jaw.push(path.clone());
                }
            }
            _ => {}
        }
    }
    l
}

impl Layout {
    /// 认出来的东西够不够跑自标定。**不够就说清缺哪一格。**
    pub fn 够吗(&self) -> Result<(), String> {
        if self.joints.is_empty() {
            return Err("没认出关节角:这台机器人没有报它自己的关节,自标定的每一相都要它".into());
        }
        if self.ee.is_empty() {
            return Err("没认出末端位姿".into());
        }
        if self.jaw.is_empty() {
            return Err("没认出夹爪开度:钳口跨度与摩擦两相都要它".into());
        }
        if !self.ambiguous.is_empty() {
            return Err(format!("有 {} 处形状分不开,拒绝硬认:{:?}", self.ambiguous.len(), self.ambiguous));
        }
        Ok(())
    }

    /// 把认出来的布局原样打出来 —— **认错了要一眼看得见**,不许它默默生效。
    pub fn 说一遍(&self) {
        let p = |v: &Vec<Vec<String>>| v.iter().map(|x| x.join(".")).collect::<Vec<_>>().join(" · ");
        println!("[认] 关节角:{}", p(&self.joints));
        println!("[认] 末端位姿:{}", p(&self.ee));
        println!("[认] 夹爪:{}", p(&self.jaw));
        println!("[认] 相机:{}", p(&self.cams));
        if !self.ambiguous.is_empty() {
            println!("[认] 🔴 分不开:{:?}", self.ambiguous);
        }
    }
}

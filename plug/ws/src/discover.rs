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
    /// 遍历实际看到的叶子(路径=形状),认不出来时全靠它。
    pub 叶子: Vec<String>,
}

/// 一台机器人报回来的数组,常常**不是**一个普通数组,而是一个自带 dtype 与 shape 的映射
/// (`{nd: true, type: "<f4", data: <二进制>, shape: [...]}`)。
///
/// 🔴 认不出这一点的后果是**静默的**:树遍历会一路钻进 `nd` / `type` / `data` / `shape`,
/// 于是那一串数**从来没有以数的形式出现过**,布局永远认不出来,而帧一直在到、
/// 两侧都不报错。实测:听满 250 帧、每 50 帧报一次"还没认出",顶层键完全正常。
/// 🔴🔴 **键不一定是文本键。**
///
/// 同一个观测里,外层(`state` / `vision` / `left_arm_joint_state`)是文本键,而**数组自己那一层
/// (`nd` / `type` / `data` / `shape`)是二进制键**。拿 `as_str()` 去比,那一层永远比不中 ⇒
/// 数组映射认不出来 ⇒ 遍历钻进去 ⇒ 那串数从来没有以数的形式出现过。
///
/// 实测(2026-08-17)的病相:叶子清单里整整一屏
/// `state.left_arm_joint_state.?=Boolean(true) | ?=String | ?=数组[1] | ?=字节[24]` ——
/// **键名全打成 `?`**,而 24 字节正好是 6 个 f32。夹爪那一格因为是普通数组,反而认得出来,
/// 于是"只有一格认出来"这个现象看起来像形状判据太严,其实是**键的读法错了**。
fn 名(k: &Value) -> Option<&str> {
    k.as_str().or_else(|| k.as_slice().and_then(|b| core::str::from_utf8(b).ok()))
}

fn 是数组映射(v: &Value) -> bool {
    v.as_map()
        .map(|m| m.iter().any(|(k, val)| 名(k) == Some("nd") && val.as_bool() == Some(true)))
        .unwrap_or(false)
}

fn 走(v: &Value, path: &mut Vec<String>, out: &mut Vec<(Vec<String>, Value)>) {
    if 是数组映射(v) {
        // 叶子:它自己就是一串数,不许再往里钻。
        out.push((path.clone(), v.clone()));
        return;
    }
    match v {
        Value::Map(m) => {
            for (k, sub) in m {
                let name = 名(k).unwrap_or("?").to_string();
                path.push(name);
                走(sub, path, out);
                path.pop();
            }
        }
        other => out.push((path.clone(), other.clone())),
    }
}

fn 键<'a>(v: &'a Value, k: &str) -> Option<&'a Value> {
    v.as_map()?.iter().find(|(kk, _)| 名(kk) == Some(k)).map(|(_, x)| x)
}

/// 这一格是不是一片图像:自报的 dtype 是字节,而 shape 是三维。
pub fn 是图(v: &Value) -> Option<(usize, usize)> {
    if !是数组映射(v) {
        return None;
    }
    let ty = 名(键(v, "type")?)?;
    if !(ty.ends_with("u1") || ty.ends_with("i1")) {
        return None;
    }
    let sh: Vec<usize> = 键(v, "shape")?.as_array()?.iter().filter_map(|x| x.as_u64().map(|n| n as usize)).collect();
    if sh.len() == 3 && sh[2] == 3 {
        Some((sh[1], sh[0]))
    } else {
        None
    }
}

/// 把一格取成一串数 —— 普通数组、或者自带 dtype 的那种映射,两种都认。
pub fn 浮点串(v: &Value) -> Option<Vec<f64>> {
    if let Value::Array(a) = v {
        let mut o = Vec::with_capacity(a.len());
        for x in a {
            o.push(x.as_f64()?);
        }
        return Some(o);
    }
    if !是数组映射(v) {
        return None;
    }
    let ty = 名(键(v, "type")?)?;
    let Value::Binary(d) = 键(v, "data")? else { return None };
    if ty.ends_with("f4") {
        Some(d.chunks_exact(4).map(|c| f32::from_le_bytes([c[0], c[1], c[2], c[3]]) as f64).collect())
    } else if ty.ends_with("f8") {
        Some(d.chunks_exact(8).map(|c| f64::from_le_bytes([c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7]])).collect())
    } else {
        None
    }
}

/// 从一帧观测里认出布局。**只看形状与值域,不看名字。**
pub fn 认(obs: &Value) -> Layout {
    let mut flat = Vec::new();
    走(obs, &mut Vec::new(), &mut flat);
    let mut l = Layout::default();
    // 🔴 遍历**实际看到的叶子**原样记下来。认不出来时,只有这一份能分开
    // "它没被当成叶子(钻进去了)" 和 "它是叶子但形状判错了" —— 而这两件修法相反。
    l.叶子 = flat
        .iter()
        .map(|(p, v)| {
            let 形 = match v {
                Value::Array(a) => format!("数组[{}]", a.len()),
                Value::Binary(b) => format!("字节[{}]", b.len()),
                Value::Map(m) => format!("映射{{{}}}", m.iter().filter_map(|(k, _)| k.as_str()).collect::<Vec<_>>().join(",")),
                o => format!("{o:?}").chars().take(18).collect(),
            };
            format!("{}={}", p.join("."), 形)
        })
        .collect();
    for (path, v) in &flat {
        // 相机:自报字节 dtype 且 shape 是三维。
        if 是图(v).is_some() {
            l.cams.push(path.clone());
            continue;
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
        for c in self.叶子.chunks(6) {
            println!("[认] 叶子:{}", c.join(" | "));
        }
    }
}

//! **把"支撑面在哪"从代码里拿出来,变成一个参数。**
//!
//! # 🔴 这里在修一句【文档说了而代码没做】的话
//!
//! `to_set` 的注释写着:*"换一台把支撑面立起来的机器,这一项跟着支撑面走,而不是跟着 z 走"*。
//! **而代码里写死的是 `[0,0,-1]`。** 一个只在"桌子是水平的"时候成立的常数,
//! 却挂着一句"它跟着支撑面走"的说明 —— 这是本仓最贵的那种文档病:
//! **读的人拿到的是承诺,跑的时候拿到的是那个常数,而两者不会不一致。**
//!
//! # 做法:换个坐标系,别改算法
//!
//! ②a 的两指那条路(切层 + 量跨度)从骨子里假设"高度沿 z、跨度在水平面里"。
//! 与其把那套几何全部改写,不如**把点云转到"支撑面法向就是 +z"的那个系里**去算,
//! 算完再把结果**转回来**。算法一个字不用动,而假设变成了显式的输入。
//!
//! ⚠️ 它买到的是**朝向无关**,不是**重力无关**:哪一面是"支撑面"仍然要由调用方说。
//! 这一层不知道重力往哪儿 —— **也不该知道**(那是身体层量出来的,加速度计就给得出)。

use crate::P3;

/// 一个旋转:把 `from` 转到 `to`。用罗德里格斯,零依赖。
#[derive(Copy, Clone, Debug)]
pub struct Turn {
    axis: [f64; 3],
    ang: f64,
}

impl Turn {
    /// 造一个把 `from` 转到 `to` 的最小旋转。两者反向时任取一条垂直轴。
    pub fn between(from: [f64; 3], to: [f64; 3]) -> Option<Turn> {
        let (a, b) = (contact_set::unit(from)?, contact_set::unit(to)?);
        let d = contact_set::dot(a, b).clamp(-1.0, 1.0);
        if d > 1.0 - 1e-12 {
            return Some(Turn { axis: [0.0, 0.0, 1.0], ang: 0.0 });
        }
        if d < -1.0 + 1e-12 {
            // 正好反向:任取一条与 a 垂直的轴转 π
            let seed = if a[0].abs() < 0.9 { [1.0, 0.0, 0.0] } else { [0.0, 1.0, 0.0] };
            let ax = contact_set::unit(contact_set::cross(a, seed))?;
            return Some(Turn { axis: ax, ang: core::f64::consts::PI });
        }
        Some(Turn { axis: contact_set::unit(contact_set::cross(a, b))?, ang: d.acos() })
    }
    /// 反过来转。
    pub fn inverse(&self) -> Turn {
        Turn { axis: self.axis, ang: -self.ang }
    }
    /// 转一个方向(不平移)。
    pub fn dir(&self, v: [f64; 3]) -> [f64; 3] {
        let (c, s) = (self.ang.cos(), self.ang.sin());
        let k = self.axis;
        let kxv = contact_set::cross(k, v);
        let kdv = contact_set::dot(k, v);
        [
            v[0] * c + kxv[0] * s + k[0] * kdv * (1.0 - c),
            v[1] * c + kxv[1] * s + k[1] * kdv * (1.0 - c),
            v[2] * c + kxv[2] * s + k[2] * kdv * (1.0 - c),
        ]
    }
    /// 转一个点(绕原点)。
    pub fn point(&self, p: P3) -> P3 {
        let v = self.dir([p.x, p.y, p.z]);
        P3 { x: v[0], y: v[1], z: v[2] }
    }
    /// 把一整个接触集转过去 —— 点、法向、锥轴、旋量、进场方向,**一个都不能漏**。
    ///
    /// 🔴 漏掉法向或锥轴,就会出现"点转过去了而面还朝着老方向"的接触集,
    /// 而那种接触集会被判据当场判死 —— 判得对,但病相看起来像"这一把物理上做不到"。
    pub fn set(&self, cs: &contact_set::ContactSet) -> contact_set::ContactSet {
        let mut out = cs.clone();
        for p in out.points.iter_mut() {
            p.at = self.dir(p.at);
            p.normal = self.dir(p.normal);
            p.cone.axis = self.dir(p.cone.axis);
        }
        out.motion = contact_set::Twist {
            lin: self.dir(cs.motion.lin),
            ang: self.dir(cs.motion.ang),
            pivot: self.dir(cs.motion.pivot),
        };
        out.approach = cs.approach.map(|a| self.dir(a));
        out
    }
}

/// **把点云转到"支撑面法向 = +z"的那个系里。** 返回转过去的点云 + 转回来的那个旋转。
///
/// `support_normal`:支撑面朝外的法向(桌子朝上、墙朝屋里)。**由调用方给** ——
/// 这一层不知道重力往哪儿,也不该知道。
pub fn to_upright(cloud: &[P3], support_normal: [f64; 3]) -> Option<(Vec<P3>, Turn)> {
    let t = Turn::between(support_normal, [0.0, 0.0, 1.0])?;
    Some((cloud.iter().map(|p| t.point(*p)).collect(), t.inverse()))
}

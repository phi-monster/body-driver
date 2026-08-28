//! **稠密光流 —— 金字塔式 Horn–Schunck。**
//!
//! # 为什么非要它不可
//!
//! 上一版用**块匹配**量"这一小块跑了多远":把画面切成格子,挨个去下一帧里找。
//! 它在有花纹的地方好用,而在**一大片光滑的白色**上完全失效 —— 一个纯白方块和它旁边那一片
//! 长得一模一样,你分不出它是没动还是挪了十个像素。这就是**孔径问题**。
//!
//! 实测代价(XW,2026-08-28,渲图看出来的):这条 Franka 甩到画面中央、占了大半个画面,
//! 而自图只认出 **2 格**(都在底座上),被跟的点落在空地板上 —— 而黑十字精准在棒球上。
//! 病不在"没数据",在**胳膊的肚子不携带位移信息**。
//!
//! # Horn–Schunck 为什么能治
//!
//! 它同时要两件事成立:
//!   ① **亮度不变**:`Ix·u + Iy·v + It = 0`(同一块东西挪过去,亮度不变)
//!   ② **平滑**:相邻像素的运动不该差太多
//!
//! ②就是那味药:在没花纹的地方①什么都约束不了(`Ix=Iy=0`),于是解**完全由②决定** ——
//! **运动从有花纹的边缘自动灌进无花纹的肚子里**。这正是我上一版手工"往邻居灌"那一步的严格版,
//! 区别是这里它是**解出来的**,不是我拍的三轮邻居平均。
//!
//! # 为什么要金字塔
//!
//! ①里的导数是**一阶展开**,只在位移小于约一个像素时成立。而我们的胳膊一步跑 ~6 像素。
//! ⇒ 先把图缩小若干倍(粗层上 6 像素变成不到 1 像素),在粗层解出大致的流,
//!   **拿它把下一帧扭回来**,在细层上只解剩下的那一点残差。逐层放大,直到原分辨率。
//!
//! # 这里没有一个"身体的数"
//!
//! 层数由画面尺寸定、迭代次数是**次数**、平滑权重取**这幅图自己的梯度尺度**(量出来的)。
//! 换一台相机、换一具身体,这段代码一个字都不用改。

/// 一帧的稠密光流:`(横向位移, 纵向位移)`,每像素一对,单位**像素**。
pub struct 流 {
    pub u: Vec<f32>,
    pub v: Vec<f32>,
    pub w: usize,
    pub h: usize,
}

/// 缩一半(2×2 盒式平均)。奇数尺寸时丢掉最后一行/列 —— 金字塔只求近似。
fn 缩(src: &[f32], w: usize, h: usize) -> (Vec<f32>, usize, usize) {
    let (nw, nh) = (w / 2, h / 2);
    let mut out = vec![0.0f32; nw * nh];
    // 0.25 = 四个像素取平均。**无量纲**的权重,不是长度。
    for y in 0..nh {
        for x in 0..nw {
            let (a, b) = (2 * x, 2 * y);
            out[y * nw + x] = 0.25
                * (src[b * w + a] + src[b * w + a + 1] + src[(b + 1) * w + a] + src[(b + 1) * w + a + 1]);
        }
    }
    (out, nw, nh)
}

/// 双线性取样(越界就夹到边上)。
fn 取(src: &[f32], w: usize, h: usize, x: f32, y: f32) -> f32 {
    let x = x.clamp(0.0, (w - 1) as f32);
    let y = y.clamp(0.0, (h - 1) as f32);
    let (x0, y0) = (x.floor() as usize, y.floor() as usize);
    let (x1, y1) = ((x0 + 1).min(w - 1), (y0 + 1).min(h - 1));
    let (fx, fy) = (x - x0 as f32, y - y0 as f32);
    let a = src[y0 * w + x0] * (1.0 - fx) + src[y0 * w + x1] * fx;
    let b = src[y1 * w + x0] * (1.0 - fx) + src[y1 * w + x1] * fx;
    a * (1.0 - fy) + b * fy
}

/// 把流放大一倍(位移也乘二 —— 它的单位是像素)。
fn 放(u: &[f32], w: usize, h: usize, nw: usize, nh: usize) -> Vec<f32> {
    let mut out = vec![0.0f32; nw * nh];
    for y in 0..nh {
        for x in 0..nw {
            out[y * nw + x] = 取(u, w, h, x as f32 * 0.5, y as f32 * 0.5) * 2.0;
        }
    }
    out
}

/// 单层 Horn–Schunck:在**已经扭正**的一对图上解剩下的位移。
///
/// 迭代式(教科书形):
/// ```text
/// u ← ū − Ix·(Ix·ū + Iy·v̄ + It) / (α² + Ix² + Iy²)
/// v ← v̄ − Iy·(Ix·ū + Iy·v̄ + It) / (α² + Ix² + Iy²)
/// ```
/// `ū, v̄` 是邻域平均 —— **平滑项就是从这里把运动灌进无花纹区域的**。
fn 一层(i1: &[f32], i2: &[f32], w: usize, h: usize, du: &mut [f32], dv: &mut [f32], 轮: usize, α2: f32) {
    // 空间导数取中心差分;时间导数是"扭正之后的第二帧 − 第一帧"。
    let mut ix = vec![0.0f32; w * h];
    let mut iy = vec![0.0f32; w * h];
    let mut it = vec![0.0f32; w * h];
    for y in 1..h.saturating_sub(1) {
        for x in 1..w.saturating_sub(1) {
            let i = y * w + x;
            ix[i] = 0.5 * (i1[i + 1] - i1[i - 1] + i2[i + 1] - i2[i - 1]) * 0.5;
            iy[i] = 0.5 * (i1[i + w] - i1[i - w] + i2[i + w] - i2[i - w]) * 0.5;
            it[i] = i2[i] - i1[i];
        }
    }
    let mut au = vec![0.0f32; w * h];
    let mut av = vec![0.0f32; w * h];
    for _ in 0..轮 {
        // 邻域平均(四邻 1/6 + 对角 1/12,Horn–Schunck 原文的那个拉普拉斯核)。
        // 这几个权重是**无量纲**的纯数学系数(加起来正好是 1),和图像的量纲、
        // 相机的分辨率、身体的尺寸都无关 —— 换任何东西都不用改。
        for y in 1..h.saturating_sub(1) {
            for x in 1..w.saturating_sub(1) {
                let i = y * w + x;
                au[i] = (du[i - 1] + du[i + 1] + du[i - w] + du[i + w]) / 6.0
                    + (du[i - w - 1] + du[i - w + 1] + du[i + w - 1] + du[i + w + 1]) / 12.0;
                av[i] = (dv[i - 1] + dv[i + 1] + dv[i - w] + dv[i + w]) / 6.0
                    + (dv[i - w - 1] + dv[i - w + 1] + dv[i + w - 1] + dv[i + w + 1]) / 12.0;
            }
        }
        for y in 1..h.saturating_sub(1) {
            for x in 1..w.saturating_sub(1) {
                let i = y * w + x;
                let 分子 = ix[i] * au[i] + iy[i] * av[i] + it[i];
                let 分母 = α2 + ix[i] * ix[i] + iy[i] * iy[i];
                let k = 分子 / 分母;
                du[i] = au[i] - ix[i] * k;
                dv[i] = av[i] - iy[i] * k;
            }
        }
    }
}

/// **算两帧之间的稠密光流。**
///
/// `层` = 金字塔层数(粗到细);`轮` = 每层迭代次数。两个都是**次数**,无量纲。
/// 平滑权重 `α²` **从这幅图自己的梯度尺度量出来**:取梯度平方的中位数。
/// 这样一来暗一点/亮一点的场景、8 位/16 位的相机,都不用改参数。
pub fn 算(前: &[u8], 后: &[u8], w: usize, h: usize, 层: usize, 轮: usize) -> Option<流> {
    if 前.len() < w * h || 后.len() < w * h || w < 8 || h < 8 {
        return None;
    }
    let f1: Vec<f32> = 前[..w * h].iter().map(|v| *v as f32).collect();
    let f2: Vec<f32> = 后[..w * h].iter().map(|v| *v as f32).collect();
    // 建金字塔(0 = 原图)
    let mut 塔1 = vec![(f1, w, h)];
    let mut 塔2 = vec![(f2, w, h)];
    for _ in 1..层.max(1) {
        let (a, aw, ah) = 塔1.last()?;
        if *aw < 16 || *ah < 16 { break }
        let (b, bw, bh) = 塔2.last()?;
        塔1.push(缩(a, *aw, *ah));
        塔2.push(缩(b, *bw, *bh));
    }
    // α²(平滑权重):这幅图自己的梯度平方**均值** —— 量出来的,不是填的。
    //
    // 🔴 **不能取中位数。** 实测(写这一层时的第一版):画面大部分是平的 ⇒ 梯度平方的中位数 ≈ 0
    // ⇒ α² 被夹到 1e-3 ⇒ **平滑项几乎不起作用** ⇒ 运动灌不进无花纹区域,
    // 而"灌进去"正是要它来治的那唯一一件事。一个纯白方块平移 4 px,中心只解出 **0.47**。
    // 均值在"平的地方多、边缘少"时由边缘主导,正好是这幅图真实的梯度尺度。
    let α2 = {
        let (a, aw, ah) = &塔1[0];
        let (mut 和, mut n) = (0.0f32, 0.0f32);
        for y in 1..ah - 1 {
            for x in 1..aw - 1 {
                let i = y * aw + x;
                let gx = 0.5 * (a[i + 1] - a[i - 1]);
                let gy = 0.5 * (a[i + aw] - a[i - aw]);
                和 += gx * gx + gy * gy;
                n += 1.0;
            }
        }
        if n < 1.0 { 1.0 } else { (和 / n).max(1e-3) }
    };
    // 从最粗一层往下解
    let 顶 = 塔1.len() - 1;
    let (_, mut cw, mut ch) = 塔1[顶];
    let mut u = vec![0.0f32; cw * ch];
    let mut v = vec![0.0f32; cw * ch];
    for lv in (0..=顶).rev() {
        let (ref a, aw, ah) = 塔1[lv];
        let (ref b, _, _) = 塔2[lv];
        if lv != 顶 {
            u = 放(&u, cw, ch, aw, ah);
            v = 放(&v, cw, ch, aw, ah);
        }
        cw = aw; ch = ah;
        // 🔴 **拿当前的流把第二帧扭回来** —— 这一步是金字塔的关键:
        // 扭正之后剩下的位移小于一个像素,一阶展开才成立。
        let mut 扭 = vec![0.0f32; aw * ah];
        for y in 0..ah {
            for x in 0..aw {
                let i = y * aw + x;
                扭[i] = 取(b, aw, ah, x as f32 + u[i], y as f32 + v[i]);
            }
        }
        let mut du = vec![0.0f32; aw * ah];
        let mut dv = vec![0.0f32; aw * ah];
        一层(a, &扭, aw, ah, &mut du, &mut dv, 轮, α2);
        for i in 0..aw * ah { u[i] += du[i]; v[i] += dv[i]; }
    }
    Some(流 { u, v, w: cw, h: ch })
}

impl 流 {
    /// 一个格子里的平均位移(像素)。
    pub fn 格里平均(&self, x0: usize, y0: usize, 边: usize) -> (f64, f64) {
        let (mut su, mut sv, mut n) = (0.0f64, 0.0f64, 0.0f64);
        for y in y0..(y0 + 边).min(self.h) {
            for x in x0..(x0 + 边).min(self.w) {
                su += self.u[y * self.w + x] as f64;
                sv += self.v[y * self.w + x] as f64;
                n += 1.0;
            }
        }
        if n < 1.0 { (0.0, 0.0) } else { (su / n, sv / n) }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 一整块**没有花纹**的白色方块平移 —— 块匹配在这儿必然失效,而光流靠平滑项该解得出来。
    /// **这条测试就是写这一层的全部理由。**
    #[test]
    fn 没花纹的一大片也能解出它在动() {
        let (w, h) = (128usize, 96usize);
        let 画 = |x: i64| -> Vec<u8> {
            let mut g = vec![40u8; w * h];
            for dy in 0..40i64 { for dx in 0..40i64 {
                let (px, py) = (x + dx, 28 + dy);
                if px >= 0 && py >= 0 && (px as usize) < w && (py as usize) < h {
                    g[py as usize * w + px as usize] = 235;   // 纯白,内部一点花纹都没有
                }
            }}
            g
        };
        let 前 = 画(30);
        let 后 = 画(34);   // 往右挪 4 像素
        let f = 算(&前, &后, w, h, 4, 60).expect("该解得出来");
        // 方块**正中间**那一格(纯白、零梯度)——块匹配在这儿只能返回 0
        let (u, v) = f.格里平均(44, 40, 12);
        // 符号约定:`u` 把**第一帧的坐标**映到第二帧 ⇒ 方块右移 4 px ⇒ u 该是 **+4** 附近。
        assert!(u > 1.0, "方块中间该解出明显的横向位移(该 ≈ +4),解出来的是 {u:.2}(纵 {v:.2})");
        // 远处的背景不该有大位移
        let (bu, bv) = f.格里平均(100, 8, 12);
        assert!(bu.abs() < 1.0 && bv.abs() < 1.0, "没动的背景不该有位移,解出来 ({bu:.2},{bv:.2})");
    }
}

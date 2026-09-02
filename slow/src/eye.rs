//! **问眼一句:那个名词在画面的哪一点。** 这是第③个生产者(学出来的那一格)接进来的地方。
//!
//! # 这一格只准给什么,以及为什么用一张"只有五个格子的表"锁住它
//!
//! 实测过一次(2026-08-13):裸着问「那东西离机器人多远、位姿多少、id 是几」,眼**不拒绝**,
//! 自己编出 `x=0.05,y=0.55,z=1.0` 再开方成 `≈1.14 m`。**编出来的数和量出来的在下游完全无法
//! 区分** —— 这是最坏的泄漏形态:不是拒答,是**伪装成测量**。
//!
//! 同一个问题套上五格 schema 之后,距离/位姿/id **一个字都出不来**,而且是合法结束不是被截断。
//! 所以这一格的约束是**结构化解码**,不是提示词纪律 —— 与本仓「规矩必须写成会失败的检查」同族。
//!
//! # 为什么自己写 BMP / base64 / HTTP
//!
//! 为了发一个请求给这条链再挂三个库,是把审计面积换成打字省事。这三样各几十行、格式都写死在
//! 标准里,而且**错了会当场发不出去**,不会悄悄给个错答案。BMP 选的是**不压缩**的那种:没有
//! 编码器也就没有"编码器把图轻微改了一点"这种查不出来的偏差。

use std::io::{Read, Write};
use std::net::TcpStream;

/// 眼能给的全部东西。**多一格都没有** —— 见文件头。
#[derive(Clone, Debug)]
pub struct Look {
    /// 归一化横坐标,0 = 最左,1 = 最右。
    pub u: f64,
    /// 归一化纵坐标,0 = 最上,1 = 最下。
    pub v: f64,
    /// 那一块占画面宽度的几分之几。
    pub span_frac: f64,
    /// 🔴 眼给的框,归一化 `[x0, y0, x1, y1]`(0=左/上,1=右/下)。
    /// `u`/`v`/`span_frac` 都是从它算出来的 —— 留原始框是为了下游能画出来核对。
    pub box01: [f64; 4],
}

/// 问一句。`what` 是要指的名词;`rgb` 是 `h*w*3` 的原始像素。
///
/// 失败一律返回 `Err(原因)`,**绝不返回一个凑出来的点位** —— 拿不到值却回个像样的数,是本仓
/// 最贵的一类事故。
pub fn ask(host: &str, port: u16, what: &str, rgb: &[u8], w: usize, h: usize) -> Result<Look, String> {
    if rgb.len() < w * h * 3 {
        return Err(format!("画面短了:要 {} 字节,只有 {}", w * h * 3, rgb.len()));
    }
    let bmp = bmp24(rgb, w, h);
    let b64 = base64(&bmp);
    // 🔴 **别在提示里给一条 schema 里不存在的退路。** 原文写着 `Set present=false if it is
    // not visible`,而这张表里**根本没有 `present` 这一格**(`additionalProperties:false` +
    // `strict:true`)⇒ 模型被要求走一条结构上走不通的门,只能照旧指一个点。
    // 实测代价:一张**空桌子 + 两片爪子**的腕相机图,它指到 (0.93,0.83) = 右下角那片爪子上,
    // 而 `span_frac` 写的是 **0.0000** —— 它其实"说"了看不见,只是说在另一格里。
    // 🔴 **也不许在提示里提爪子/夹爪。** 真机那条线实测过:提示里写"底部黑三角=夹爪",
    // 反而把那个词喂进去、让它照着指爪。要指的是世界,不是身体。
    // 🔴🔴🔴 **问【框】,不问【点】。**(2026-08-28 改;隔壁 universal-grounding 早就量过)
    //
    // `universal-grounding/README "眼给框 + 闭式弓字形路径"` 原文:
    //   **眼给框 ⇒ 10 次成 9(n=34 时 23/34);而眼给点 ⇒ 0/10**。
    // 同一份文档 §3.6 给了病因,而且是量出来的:同一批模型做四选一,
    //   **"选哪条轨迹" 0.916(超人类)· "选哪个抓取点" 0.403 ≈ 随机**
    //   ⇒ *"所有模型在需要空间精度的那问上都塌到随机 ⇒ 病因是【问法】,不是模型弱。"*
    // 我们这边一直在问点,正是被点名的那个问法。
    //
    // 本仓实测(W8,棒球,同一帧):球的真实中心 (440,183)、半径 ~15 px。
    //   · 问点  ⇒ (454,168):**偏右 14 px、偏上 15 px ⇒ 落在球外面**(球顶 y=169);
    //     且 `span_frac` 给 0.030 = 直径 19 px,而球真实 ~30 px ⇒ **小四成**。
    //   · 问框  ⇒ (426,168)-(455,190):**压在球上**,宽度对得上。
    // 位置偏一个半径 + 尺寸小四成,两件事叠加 ⇒ `mask_around` 的种子点落到桌面上
    //   ⇒ 那一点的深度 = 桌面深度 ⇒ **掩膜整片长成桌面**,下游在空桌面上规划抓取
    //   (`one1.pgm`:两个接触点一个在球的边缘、一个在空桌面上)。
    // ⇒ 框把**种子点**和**半径**同时修对,不需要任何新模型、不碰那三个被判死的检测器。
    let prompt = format!(
        "Task: {what}\\n\\nWhich single object in this image should the robot act on? \
         Output its tight bounding box as normalised image coordinates: \
         x0,y0 = top-left corner and x1,y1 = bottom-right corner, \
         where x is 0 at the left edge and 1 at the right edge, \
         and y is 0 at the top edge and 1 at the bottom edge."
    );
    let body = format!(
        r#"{{"model":"eye","max_tokens":600,"temperature":0,"chat_template_kwargs":{{"enable_thinking":false}},"response_format":{{"type":"json_schema","json_schema":{{"name":"contact_ask","strict":true,"schema":{{"type":"object","additionalProperties":false,"required":["x0","y0","x1","y1"],"properties":{{"x0":{{"type":"number","minimum":0,"maximum":1}},"y0":{{"type":"number","minimum":0,"maximum":1}},"x1":{{"type":"number","minimum":0,"maximum":1}},"y1":{{"type":"number","minimum":0,"maximum":1}}}}}}}}}},"messages":[{{"role":"user","content":[{{"type":"image_url","image_url":{{"url":"data:image/bmp;base64,{b64}"}}}},{{"type":"text","text":"{prompt}"}}]}}]}}"#
    );

    // 🔴 **上限 200 在腕相机的特写上会被顶到**(实测:偏移 635 处对象没收尾),
    // 而"回包被截断"与"眼看不见"在下游完全同形。放到 600,并把截断当**错误**报出来 ——
    // 一个半截 JSON 解析失败时,报的是"眼给的不是 JSON",读的人会去查眼而不是查长度。
    let raw = post(host, port, "/v1/chat/completions", &body)?;
    // 回包是 JSON 里嵌了一段 JSON 字符串;先取出那段,再解析它。
    let inner = extract_content(&raw).ok_or_else(|| {
        format!("回包里没有 content(前 200 字:{})", &raw[..raw.len().min(200)])
    })?;
    let j = crate::json::parse(&inner).map_err(|e| {
        // 🔴 解析失败时**把原文头几百字带出来**。没有它,"解析失败"这四个字同时兼容
        // "挖错了地方"和"回包是分块传输"两种病,而它们要改的地方完全不同。
        format!(
            "眼给的不是 JSON: {e} ‖ 挖出来的前 200 字:{} ‖ 回包前 200 字:{}",
            &inner[..inner.len().min(200)],
            &raw[..raw.len().min(200)]
        )
    })?;
    let num = |k: &str| -> Result<f64, String> {
        j.get(k)
            .and_then(|x| x.num())
            .ok_or_else(|| format!("眼没给 {k}"))
    };
    // 🔴 **不要往这张表里加格子,也不要把结论放在证据前面。** 两条都是实测:
    // 五格稳,加到九格时点位退化成 `1,1`;而把 `status` 这类结论放在第一个字段时,
    // 自回归解码让它在看到 u/v 之前就得先盖章 —— 三问全答弃权,而同一次里 u/v 写的是对的。
    // ⚠️ 「让眼自己说看不见」这条路本仓已判过:加弃权选项 / 调字段序 / 先列后选,三种都没治住。
    //    弃权要靠**几何自证**(拿它给的像素去看那处到底有没有东西),不靠它的自述。
    // 框 → 中心 + 尺寸。**中心和半径都来自框自己,不再由模型另外估一个"占画幅"。**
    // 顺序不许假设:有的回答会把两角写反,取 min/max。
    let (bx0, bx1) = { let (a, b) = (num("x0")?, num("x1")?); (a.min(b), a.max(b)) };
    let (by0, by1) = { let (a, b) = (num("y0")?, num("y1")?); (a.min(b), a.max(b)) };
    let 宽f = (bx1 - bx0).max(0.0);
    let 高f = (by1 - by0).max(0.0);
    if !(宽f > 0.0 && 高f > 0.0) {
        return Err(format!("眼给的框是空的:x {bx0:.3}..{bx1:.3} · y {by0:.3}..{by1:.3}"));
    }
    // `span_frac` 的下游语义是"**占画面宽度**的几分之几",`mask_around` 拿它同时定
    // 圈的半径和深度厚度。取框的**长边**换算到画宽上 —— 短边会让圈盖不住细长物体,
    // 而盖不住的病相是"掩膜只长出一小截",比圈大一点更难查。
    let 长边f = 宽f.max(高f * h as f64 / w as f64);
    Ok(Look {
        u: (bx0 + bx1) * 0.5,
        v: (by0 + by1) * 0.5,
        span_frac: 长边f,
        box01: [bx0, by0, bx1, by1],
    })
}

/// 从 OpenAI 风格回包里挖出 `choices[0].message.content` 那段字符串,并把转义还原。
///
/// 手写而不是全量解析:回包里有 base64 长度级别的字段,而我们只要那一小段。
fn extract_content(raw: &str) -> Option<String> {
    let key = "\"content\":\"";
    let s = raw.find(key)? + key.len();
    let rest = &raw[s..];
    let mut out = String::new();
    let mut it = rest.chars();
    while let Some(c) = it.next() {
        match c {
            '"' => return Some(out),
            '\\' => match it.next()? {
                'n' => out.push('\n'),
                't' => out.push('\t'),
                'r' => {}
                '"' => out.push('"'),
                '\\' => out.push('\\'),
                other => out.push(other),
            },
            other => out.push(other),
        }
    }
    None
}

/// 一次 HTTP POST。读到对端关闭为止 —— 所以显式要求 `Connection: close`。
fn post(host: &str, port: u16, path: &str, body: &str) -> Result<String, String> {
    let mut s = TcpStream::connect((host, port)).map_err(|e| format!("连不上眼: {e}"))?;
    s.set_read_timeout(Some(std::time::Duration::from_secs(180))).ok();
    let req = format!(
        "POST {path} HTTP/1.1\r\nHost: {host}\r\nContent-Type: application/json\r\n\
         Content-Length: {}\r\nConnection: close\r\n\r\n",
        body.len()
    );
    s.write_all(req.as_bytes()).map_err(|e| format!("发不出去: {e}"))?;
    s.write_all(body.as_bytes()).map_err(|e| format!("发不出去: {e}"))?;
    s.flush().ok();
    let mut buf = Vec::new();
    s.read_to_end(&mut buf).map_err(|e| format!("读不回来: {e}"))?;
    let text = String::from_utf8_lossy(&buf).into_owned();
    let head_end = text.find("\r\n\r\n").ok_or("回包没有头尾分隔")?;
    Ok(text[head_end + 4..].to_string())
}

/// 24 位不压缩 BMP。高度写成**负数** = 自上而下,省掉翻行这一步(翻错了是那种"图看着正常、
/// 上下颠倒"的错,而颠倒的图里模型照样会给你一个点)。
fn bmp24(rgb: &[u8], w: usize, h: usize) -> Vec<u8> {
    let row = w * 3;
    let pad = (4 - row % 4) % 4;
    let data = (row + pad) * h;
    let mut out = Vec::with_capacity(54 + data);
    out.extend_from_slice(b"BM");
    out.extend_from_slice(&((54 + data) as u32).to_le_bytes());
    out.extend_from_slice(&0u32.to_le_bytes());
    out.extend_from_slice(&54u32.to_le_bytes());
    out.extend_from_slice(&40u32.to_le_bytes());
    out.extend_from_slice(&(w as i32).to_le_bytes());
    out.extend_from_slice(&(-(h as i32)).to_le_bytes());
    out.extend_from_slice(&1u16.to_le_bytes());
    out.extend_from_slice(&24u16.to_le_bytes());
    out.extend_from_slice(&0u32.to_le_bytes());
    out.extend_from_slice(&(data as u32).to_le_bytes());
    for _ in 0..4 {
        out.extend_from_slice(&0u32.to_le_bytes());
    }
    for y in 0..h {
        for x in 0..w {
            let p = (y * w + x) * 3;
            // BMP 存的是 BGR,不是 RGB。写反了会得到一张颜色错、形状对的图 —— 而形状对
            // 就足以让模型给出一个看起来合理的点位。
            out.push(rgb[p + 2]);
            out.push(rgb[p + 1]);
            out.push(rgb[p]);
        }
        out.extend(std::iter::repeat(0u8).take(pad));
    }
    out
}

/// 标准 base64。
fn base64(b: &[u8]) -> String {
    const A: &[u8] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut out = String::with_capacity(b.len().div_ceil(3) * 4);
    for c in b.chunks(3) {
        let n = ((c[0] as u32) << 16)
            | ((*c.get(1).unwrap_or(&0) as u32) << 8)
            | (*c.get(2).unwrap_or(&0) as u32);
        out.push(A[(n >> 18) as usize & 63] as char);
        out.push(A[(n >> 12) as usize & 63] as char);
        out.push(if c.len() > 1 { A[(n >> 6) as usize & 63] as char } else { '=' });
        out.push(if c.len() > 2 { A[n as usize & 63] as char } else { '=' });
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn base64_matches_the_standard_vectors() {
        assert_eq!(base64(b""), "");
        assert_eq!(base64(b"f"), "Zg==");
        assert_eq!(base64(b"fo"), "Zm8=");
        assert_eq!(base64(b"foo"), "Zm9v");
        assert_eq!(base64(b"foobar"), "Zm9vYmFy");
    }

    /// 头部字节数、尺寸、以及**每行补齐到 4 字节**都要对 —— 补齐算错的 BMP 会被解出一张
    /// 逐行错位的图,而错位的图里模型照样会给你一个点位。
    #[test]
    fn bmp_header_and_row_padding_are_right() {
        let (w, h) = (3usize, 2usize); // 每行 9 字节 ⇒ 要补 3 字节
        let rgb: Vec<u8> = (0..w * h * 3).map(|i| i as u8).collect();
        let b = bmp24(&rgb, w, h);
        assert_eq!(&b[0..2], b"BM");
        assert_eq!(u32::from_le_bytes(b[10..14].try_into().unwrap()), 54);
        assert_eq!(i32::from_le_bytes(b[18..22].try_into().unwrap()), 3);
        assert_eq!(i32::from_le_bytes(b[22..26].try_into().unwrap()), -2, "负高度=自上而下");
        assert_eq!(u16::from_le_bytes(b[28..30].try_into().unwrap()), 24);
        assert_eq!(b.len(), 54 + 12 * 2, "每行 9 字节补到 12");
        // 第一个像素:BMP 里是 BGR
        assert_eq!((b[54], b[55], b[56]), (rgb[2], rgb[1], rgb[0]));
    }

    /// 回包里的那段 JSON 是**字符串里嵌的**,转义要还原对。
    #[test]
    fn content_is_unescaped_out_of_the_envelope() {
        let raw = r#"{"choices":[{"message":{"role":"assistant","content":"{\n  \"u\": 0.5,\n  \"v\": 0.25\n}"}}]}"#;
        let inner = extract_content(raw).expect("挖得出来");
        assert!(inner.contains("\"u\": 0.5"), "得到 {inner}");
        let j = crate::json::parse(&inner).expect("是合法 JSON");
        assert_eq!(j.get("u").and_then(|x| x.num()), Some(0.5));
    }

    /// 画面短了要报错,不许把不存在的像素当 0 编进去。
    #[test]
    fn a_short_frame_is_refused() {
        let e = ask("127.0.0.1", 1, "scissors", &[0u8; 10], 64, 48).unwrap_err();
        assert!(e.contains("画面短了"), "得到 {e}");
    }
}

// ────────────────────────────────────────────────────────────────────────────
// 减法③:**候选由几何出,眼只负责【挑】。**(owner 2026-08-28)
// ────────────────────────────────────────────────────────────────────────────

/// 让眼从**已经画在图上的编号框**里挑一个,返回编号(1 起)。
///
/// # 为什么这一步比问框还值
///
/// 同一批模型、同一份文档 §3.6 的实测:**"选哪条轨迹" 0.916(超人类)· "选哪个抓取点"
/// 0.403 ≈ 随机**。问框已经把"位置偏一个半径"治掉了(见 `ask` 头注 W8 那一组读数),
/// 但框的**边界仍然是模型脑补的**;而候选框是从深度图上量出来的真实结构 ⇒
/// ① 边界是对的(不用再猜半径,而"猜出来的半径"正是把桌面圈进点云的那一个);
/// ② **挑错了当场看得见** —— 编号 3 是不是那个棒球,渲一张图就能判;
///    而脑补出来的坐标,对的和错的长得一模一样。
///
/// # 挑不出来要说"没有",不许硬挑
///
/// 实测代价(WZ,2026-08-28):`general_pickup` **每一集换一张桌子**,而我把指令写死成
/// 一句 "Pick up the baseball",于是 8 集里有 7 集在**根本没有棒球**的桌上找棒球 ——
/// 眼只能挑一个最圆的东西交差。⇒ 这里给 `0` 这个出口:**一个都不是,就答 0**。
/// (这与"让眼自己说看不见"那条判死不同:那条问的是"你看不看得见一个点",
///  是自述;这里问的是**在这几个已经存在的框里选**,是选择题,而选择题它做得很好。)
pub struct Pick {
    /// 挑中的编号(1 起);`0` = 一块都不是。
    pub region: usize,
}

pub fn pick(
    host: &str,
    port: u16,
    what: &str,
    rgb: &[u8],
    w: usize,
    h: usize,
    n: usize,
) -> Result<Pick, String> {
    if n == 0 {
        return Err("没有候选框可挑".into());
    }
    if rgb.len() < w * h * 3 {
        return Err(format!("画面短了:要 {} 字节,只有 {}", w * h * 3, rgb.len()));
    }
    let bmp = bmp24(rgb, w, h);
    let b64 = base64(&bmp);
    let prompt = format!(
        "Task: {what}\\n\\nThe image has {n} numbered boxes drawn on it. \
         Each box outlines one physical object on the surface. \
         Which numbered box is the object the task refers to? \
         Answer with that number. If none of the boxes is that object, answer 0."
    );
    let body = format!(
        r#"{{"model":"eye","max_tokens":200,"temperature":0,"chat_template_kwargs":{{"enable_thinking":false}},"response_format":{{"type":"json_schema","json_schema":{{"name":"pick_region","strict":true,"schema":{{"type":"object","additionalProperties":false,"required":["region"],"properties":{{"region":{{"type":"integer","minimum":0,"maximum":{n}}}}}}}}}}},"messages":[{{"role":"user","content":[{{"type":"image_url","image_url":{{"url":"data:image/bmp;base64,{b64}"}}}},{{"type":"text","text":"{prompt}"}}]}}]}}"#
    );
    let raw = post(host, port, "/v1/chat/completions", &body)?;
    let inner = extract_content(&raw).ok_or_else(|| {
        format!("回包里没有 content(前 200 字:{})", &raw[..raw.len().min(200)])
    })?;
    let j = crate::json::parse(&inner).map_err(|e| {
        format!(
            "眼给的不是 JSON: {e} ‖ 挖出来的前 200 字:{} ‖ 回包前 200 字:{}",
            &inner[..inner.len().min(200)],
            &raw[..raw.len().min(200)]
        )
    })?;
    let r = j
        .get("region")
        .and_then(|x| x.num())
        .ok_or_else(|| "眼没给 region".to_string())?;
    if !(r.is_finite() && r >= 0.0) {
        return Err(format!("眼给的 region 不合法:{r}"));
    }
    let t = |k: &str| j.get(k).and_then(|x| x.text()).unwrap_or("").to_string();
    Ok(Pick { region: r.round() as usize })
}

// ══════════════════════════════════════════════════════════════════════════════
// 🔴🔴🔴 **把模型从"指哪个是球的工具"改成【这具身体的主体】。**(owner 2026-09-01 定)
//
// 依据是自由授权搜索扫出来的那道缝(结论在 `README` 的论文那一节):
//   · 「自己量身体」那条线里**一个大模型都没有**;
//   · 「大模型当脑子」那条线**从来不去量身体** —— HumanCLAW 原话:
//     *"No current VLM perceives its own body … it behaves like a ghost: it holds no
//      instinctive model of which pixels are its own limbs, and it never tries to infer
//      where they are."*(最好的模型在"找→走→坐"上只有 **16.8%**)
//   · **没有任何工作把这两半接起来。**
//
// 这个函数就是接口:给它的不再是"这几个框里哪个是球",而是
//   **我看见什么 · 我能下什么命令(它自己量出来的身体)· 我刚才干了什么、结果如何 · 我要什么**。
//
// 🔴 **三条硬约束,写在这里免得以后被绕过:**
//  ① 给它的身体描述**全部来自这台机器人当场量出来的**(部件图 / 命令 vs 实到),
//     **不许出现任何写死的身体常数**,也不许出现 URDF/CAD 里的东西。
//     换一具机体,这段描述整个变 ⇒ 它就是另一具身体上的它。**这不是"参数进策略",
//     因为什么都没训过、装机就是新生**(owner 2026-09-01:"换机体就是一个新生命")。
//  ② 它的输出**只允许是驱动真的能执行的那几件事**,而且**一个身体词、一个世界轴都不许有**
//     (不许"往上抬 5 厘米" —— 那既是米、又是世界的上)。
//     文献量过:VLM 在定性关系上强、**在定量几何上弱**(RoboVista 最好 56.5%,
//     30.2% 是把东西认错),所以只问它"怎么办",不问它"走多少"。
//  ③ 它答什么都**不许承重到控制**:走多少永远由量出来的方向盘算。它只改**做什么**。
//
/// 它对当前局面的判断。`做什么` 是一个封闭集合 —— 驱动对每一项都有对应动作。
/// 它对当前局面的判断。**里面没有一个动词。**
///
/// 🔴🔴🔴 **动词表在 2026-09-02 被 owner 下令整片删除**(原话:*"动词表必须现在删,不然算作弊"*)。
/// 删掉的是三张:这里的 8 个(approach/close/open/retreat/look_around/new_grasp_point/
/// switch_hand/done)· `pick` 的 6 个(grasp/push/place/pry/open/close)· `ask` 的同一份,
/// 外加 3 档力度(light/medium/firm)。
///
/// **替代品不是我发明的,是仓里早就建好并测过的那句话** —— 接触集第③格:
/// *"① 碰哪几个点 · ② 每点的法向和锥 · ③ **物体要怎么动** · ④ 容差"*。
/// 十三个动词本来就塌进这一张表(80 个单元测试;吸盘一个点、三指三个点、五指五个点填同一张)。
/// 这里把第③格说成**画面里的一格**:*"这个东西最后要落到第几格。"*
///
/// - **抓起来 10 厘米** = 球要落到它上方那一格
/// - **拿球砸小人** = 球要落到小人所在的那一格
/// - **换只手 / 换个下手点 / 退回去 / 张开** —— 全都不再是"一件事",
///   它们是驱动为了把东西送到那一格而自己解出来的中间步骤。
///
/// ⚠️ **为什么是"第几格"而不是坐标**:VLM 定量几何弱(RoboVista 最好 56.5%、30.2% 认错东西),
/// 但**选择题做得好**(同批模型选轨迹 0.916)。仓里挑物体已经是"第几块"这个形状,这里照抄。
pub struct 段 {
    /// 🔴🔴🔴 **要动的是第几号东西**(1 起;0 = 我说不上来)。
    ///
    /// 编号表由调用方拼:**先是"我身上的每一块"**(部件图量出来的,全是画面语言 ——
    /// 一动它跟着动的那一片在画面哪儿、占多大),**再是"世界里切出来的每一块"**。
    /// 里面**没有一个身体参数**(不出现关节、自由度、几根手指、多长多宽),
    /// 所以 §1.7 那条"策略的输入里不出现任何身体参数"仍然成立。
    ///
    /// **这一格拓宽之后,原来那三个【我手写的触发器】全部消失**(owner 2026-09-02:
    /// *"把你手写的规则全部删掉,让 vlm 彻底成为主角"*):
    /// - 靠近 = **我的接触面** → 目标所在那一格
    /// - **挪一步让相机多看见我** = **我的接触面** → 一个看得见的格子(原来是我写的"空转三拍就挪")
    /// - 合爪 = **我的一根手指** → **另一根手指**那一格
    /// - 抓起来 = **那个物体** → 它上方那一格
    /// - 砸过去 = **那个物体** → 目标那一格,**快**
    ///
    /// **一句话,五件事,零动词、零身体参数。**
    pub 动第几号: usize,
    /// **它最后要落到第几格**(1 起;0 = 我说不上来,你自己按上一段接着干)。
    pub 到哪一格: usize,
    /// 🔴🔴🔴 **做到什么条件为止再来找我 —— 这一格是它自己说的,不是任何人写死的触发器。**
    ///(owner 2026-09-02:*"为什么这个触发器需要你主动去设计?一个抓取任务就要你设计这么多,
    ///  那后面的机器人死斗、无人机任务你设计得完吗?你把 vlm 当成傻瓜了。"*)
    ///
    /// 这五个**是【事件】不是动词** —— 任何传感器都认得出来,换什么机器、什么任务都成立:
    /// `amount` 走完 · `contact` 碰上 · `resist` 推不动 · `slip` 东西不再跟着我 · `settle` 画面不再变。
    /// 而且**驱动本来就在量它们**(`verb::Until`)。
    pub 到什么为止: String,
    /// 它认为整件事已经做完了。
    pub 完了: bool,
    /// **多快**:快 = 步长用满、步间不等停;握着东西 + 快 + 直到脱手 ⇒ 边动边松(抛/砸)。
    /// 人抛球不知道球速,只知道"比上次狠一点";这里只有快/慢两档,倍数是量出来的探幅。
    pub 快: bool,
    /// **别碰**:编号表里不许碰的那几号(躲拳 / 别砸到旁边的东西)。驱动每步先算预测位置,要碰上就缩步并说出来。
    pub 别碰: Vec<usize>,
    /// 它自己的理由(只进日志、只给人看,**不进任何判据**)。
    pub 为什么: String,
}

/// 问它:**你现在这具身体、这个局面,那个东西该落到哪一格。**
///
/// 🔴🔴 **`why` 必须排在 `goal_cell` 【前面】。** 自检实测(2026-09-02):`goal_cell` 在前时,
/// 它写 *"I must move it to the cell directly above it, which is cell 6"* 而 `goal_cell` 填的是
/// **12**(球现在那一格)—— **理由和答案自相矛盾**,因为自回归解码逼它先盖章、后讲理。
/// 仓里早写过同一条:*"把结论放在证据前面 ⇒ 三问全答弃权,而同一次里 u/v 写的是对的。"*
///
/// `身体` 是调用方从**部件图**拼出来的一段话(第几号通道一动、画面里哪一片跟着动)。
/// `刚才` 是"我下了什么命令、实际发生了什么"。两段都必须是**量出来的**,
/// 调用方不许往里塞任何常数。`格数` 是画面上画了几格。
pub fn 问段(
    host: &str,
    port: u16,
    任务: &str,
    身体: &str,
    刚才: &str,
    列: usize,
    行: usize,
    条数: usize,
    rgb: &[u8],
    w: usize,
    h: usize,
) -> Result<段, String> {
    if rgb.len() < w * h * 3 {
        return Err(format!("画面短了:要 {} 字节,只有 {}", w * h * 3, rgb.len()));
    }
    let 格数 = 列 * 行;
    if 格数 == 0 { return Err("画面上一格都没画".into()) }
    let bmp = bmp24(rgb, w, h);
    let b64 = base64(&bmp);
    let esc = |t: &str| t.replace('\\', "\\\\").replace('"', "\\\"").replace('\n', "\\n");
    let prompt = format!(
        "You are not a model looking at a picture. You ARE this robot. This image is what you see right now, with a numbered grid drawn over it: {列} columns x {行} rows, numbered 1..{格数} left to right then top to bottom, so cell n-{列} is DIRECTLY ABOVE cell n and cell n+1 is directly to its right.\\n\\nYOUR BODY (you measured this yourself just now, by moving one channel at a time and watching which part of the picture followed):\\n{}\\n\\nWHAT YOU JUST DID AND WHAT HAPPENED:\\n{}\\n\\nWHAT YOU ARE TRYING TO DO: {}\\n\\nYou are in charge of the loop. There are NO action words - none exist. You say TWO numbers: WHICH NUMBERED ITEM must move, and WHICH NUMBERED CELL it must end up in. Also say whether to go FAST (fast = full steps without pausing; if you are holding something and say fast until slip, the hand lets go while moving - that is how you throw or strike) and list any numbered items that must NOT be touched (avoid_items, may be empty). The numbered items are listed under YOUR BODY above - first the pieces of yourself (measured just now by moving one channel at a time and watching which part of the picture followed), then the things out in the world. Moving a piece of yourself to a cell is how you reach, how you get a camera to see you better, and how you bring one finger to another. Moving a thing in the world to a cell is how you pick it up, put it down, or send it somewhere. There are no action words and none exist; the body works out which channels to push from what it measured.\\n\\nThe grid lies flat over the picture. A thing that is lifted toward the camera stays in the SAME cell (it only gets nearer); so to pick something up, name the cell it is already in - the body lifts it once it is held. Name a DIFFERENT cell only when the thing must end up somewhere else in the picture.\\n\\nAlso say WHEN to call you back. These are EVENTS the body measures, not actions: amount (the body finished the move it worked out) / contact (something is touched) / resist (it will not move any further) / slip (the thing stops following me) / settle (the picture stops changing). Pick the event that actually ends THIS piece of work - settle only means the world went quiet, which is not the same as the work being done.\\n\\nSet done=true only when the thing has ALREADY ended up where the task wants it.\\n\\nDo NOT give distances, angles, speeds or any numbers - you are bad at those and the body already measures them.",
        esc(身体), esc(刚才), esc(任务)
    );
    let body = format!(
        r#"{{"model":"eye","max_tokens":300,"temperature":0,"chat_template_kwargs":{{"enable_thinking":false}},"response_format":{{"type":"json_schema","json_schema":{{"name":"where_it_must_end_up","strict":true,"schema":{{"type":"object","additionalProperties":false,"required":["why","move_item","goal_cell","until","fast","avoid_items","done"],"properties":{{"why":{{"type":"string"}},"move_item":{{"type":"integer","minimum":0,"maximum":{条数}}},"goal_cell":{{"type":"integer","minimum":0,"maximum":{格数}}},"until":{{"type":"string","enum":["amount","contact","resist","slip","settle"]}},"fast":{{"type":"boolean"}},"avoid_items":{{"type":"array","maxItems":4,"items":{{"type":"integer","minimum":1,"maximum":{条数}}}}},"done":{{"type":"boolean"}}}}}}}}}},"messages":[{{"role":"user","content":[{{"type":"image_url","image_url":{{"url":"data:image/bmp;base64,{b64}"}}}},{{"type":"text","text":"{prompt}"}}]}}]}}"#
    );
    let raw = post(host, port, "/v1/chat/completions", &body)?;
    let inner = extract_content(&raw)
        .ok_or_else(|| format!("回包里没有 content(前 200 字:{})", &raw[..raw.len().min(200)]))?;
    let j = crate::json::parse(&inner)
        .map_err(|e| format!("眼给的不是 JSON: {e} ‖ 前 200 字:{}", &inner[..inner.len().min(200)]))?;
    let t = |k: &str| j.get(k).and_then(|x| x.text()).unwrap_or("").to_string();
    let u = t("until");
    if u.is_empty() { return Err("眼没给 until".into()) }
    let mv = j.get("move_item").and_then(|x| x.num()).ok_or_else(|| "眼没给 move_item".to_string())?;
    if !(mv.is_finite() && mv >= 0.0 && mv <= 条数 as f64) {
        return Err(format!("眼给的条号不合法:{mv}(只有 {条数} 条)"));
    }
    let g = j.get("goal_cell").and_then(|x| x.num()).ok_or_else(|| "眼没给 goal_cell".to_string())?;
    if !(g.is_finite() && g >= 0.0 && g <= 格数 as f64) {
        return Err(format!("眼给的格号不合法:{g}(只有 {格数} 格)"));
    }
    let d = j.get("done").and_then(|x| x.boolean()).unwrap_or(false);
    let 快 = j.get("fast").and_then(|x| x.boolean()).unwrap_or(false);
    let 别碰: Vec<usize> = j.get("avoid_items").map(|a| a.nums().into_iter().filter(|x| x.is_finite() && *x >= 1.0).map(|x| x.round() as usize).collect()).unwrap_or_default();
    Ok(段 { 动第几号: mv.round() as usize, 到哪一格: g.round() as usize, 快, 别碰, 到什么为止: u, 完了: d, 为什么: t("why") })
}


/// 🔴🔴🔴 **飞行员模式:每帧一张白纸,问一个方向。**(owner 2026-09-03 定)
///
/// 离线验证(CG 的 10 帧,方向画到帧上逐帧看过):
/// - 带上下文、每帧只吐 1 个字 ⇒ N/IN 交替,**跟画面无关**
/// - 带上下文、每帧一句短理由 ⇒ **编故事**:手臂缩回去了它还说"我已经在球正下方"
/// - **每帧独立问、不带历史、意图用文字带进去 ⇒ 10/10 方向粗略正确**,0.77 s/帧
/// ⇒ **这个模型一带上历史就相信自己讲过的话、不看画面;连续反馈必须每帧一张白纸。**
///
/// 输出是 12 个画面语言的码:8 个罗盘方向(N = 画面上方)· IN(朝桌面下)· OUT(离桌面)·
/// STOP · CLOSE。**里面没有身体参数**;把它变成推哪几个通道,是驱动拿量出来的通道表解的。
pub struct 方向 {
    /// 12 码之一。
    pub 码: String,
    /// 它认为自己的爪子在画面哪儿(只进日志)。
    pub 爪在: String,
    /// 它认为目标在画面哪儿(只进日志)。
    pub 目标在: String,
}

pub fn 问方向(
    host: &str,
    port: u16,
    任务: &str,
    刚才: &str,
    rgb: &[u8],
    w: usize,
    h: usize,
) -> Result<方向, String> {
    if rgb.len() < w * h * 3 {
        return Err(format!("画面短了:要 {} 字节,只有 {}", w * h * 3, rgb.len()));
    }
    let bmp = bmp24(rgb, w, h);
    let b64 = base64(&bmp);
    let esc = |t: &str| t.replace('\\', "\\\\").replace('"', "\\\"").replace('\n', "\\n");
    let prompt = format!(
        "You ARE this robot arm; this picture is what you see right now from a camera fixed above the table. Task: {}\\n\\nWhat happened just before: {}\\n\\nLook at THIS frame only. In a few words say where your gripper (the two-finger hand) is in the picture and where the thing you must act on is. Then ONE code for the direction the gripper must move next: N NE E SE S SW W NW are directions in the picture (N = toward the top of the picture); IN = down toward the table; OUT = up away from the table; STOP = hold still; CLOSE = close the fingers now (only when the thing is between them). No distances, no numbers.",
        esc(任务), esc(刚才)
    );
    let body = format!(
        r#"{{"model":"eye","max_tokens":90,"temperature":0,"chat_template_kwargs":{{"enable_thinking":false}},"response_format":{{"type":"json_schema","json_schema":{{"name":"dir","strict":true,"schema":{{"type":"object","additionalProperties":false,"required":["gripper","thing","d"],"properties":{{"gripper":{{"type":"string","maxLength":60}},"thing":{{"type":"string","maxLength":60}},"d":{{"type":"string","enum":["N","NE","E","SE","S","SW","W","NW","IN","OUT","STOP","CLOSE"]}}}}}}}}}},"messages":[{{"role":"user","content":[{{"type":"image_url","image_url":{{"url":"data:image/bmp;base64,{b64}"}}}},{{"type":"text","text":"{prompt}"}}]}}]}}"#
    );
    let raw = post(host, port, "/v1/chat/completions", &body)?;
    let inner = extract_content(&raw)
        .ok_or_else(|| format!("回包里没有 content(前 200 字:{})", &raw[..raw.len().min(200)]))?;
    let j = crate::json::parse(&inner)
        .map_err(|e| format!("眼给的不是 JSON: {e} ‖ 前 200 字:{}", &inner[..inner.len().min(200)]))?;
    let t = |k: &str| j.get(k).and_then(|x| x.text()).unwrap_or("").to_string();
    let d = t("d");
    if d.is_empty() { return Err("眼没给 d".into()) }
    Ok(方向 { 码: d, 爪在: t("gripper"), 目标在: t("thing") })
}


/// 🔴 **飞行员模式第二版:问两个格号,方向和步数由驱动算。**(owner 2026-09-03:"试试")
///
/// CH 实测:问"爪子在哪、球在哪、往哪走",它答 "center / center / IN",一压压在牛仔裤上 ——
/// 爪子在画面中部偏下、球在中部偏右上,**按"中间"这个粗度两者是一回事**。问得粗,答得就粗。
/// 改成数格子(图上画着编号网格,它选择题做得准):**指尖在第几格、目标在第几格**;
/// 往哪走、走几步是两个格号一减,**驱动算**,于是不会走过头。
pub struct 两格 {
    pub 指尖格: usize,
    pub 目标格: usize,
    /// 多快(同 `段::快`)。
    pub 快: bool,
    /// 只有指尖格 == 目标格 且身体顶住时才有意义。
    pub 合: bool,
    pub 看到: String,
}

pub fn 问格(
    host: &str,
    port: u16,
    任务: &str,
    刚才: &str,
    列: usize,
    行: usize,
    rgb: &[u8],
    w: usize,
    h: usize,
) -> Result<两格, String> {
    if rgb.len() < w * h * 3 {
        return Err(format!("画面短了:要 {} 字节,只有 {}", w * h * 3, rgb.len()));
    }
    let 格数 = 列 * 行;
    let bmp = bmp24(rgb, w, h);
    let b64 = base64(&bmp);
    let esc = |t: &str| t.replace('\\', "\\\\").replace('"', "\\\"").replace('\n', "\\n");
    let prompt = format!(
        "You ARE this robot arm; this picture is what you see right now from a camera fixed above the table. A numbered grid is drawn over it: {列} columns x {行} rows, cells 1..{格数}, left to right then top to bottom. Task: {}\\n\\nWhat happened just before: {}\\n\\nLook at THIS frame only. Your FINGERTIPS are the two small dark wedges at the very end of the arm (not the big pale wrist casing). Answer: which cell the fingertips are in, and which cell the thing you must act on is in. Set close=true ONLY if the fingertips are in the same cell as the thing AND the body reported it is touching something. Set fast=true when the hand is far from the thing and should take full steps; false when close. No distances, no numbers other than cell numbers.",
        esc(任务), esc(刚才)
    );
    let body = format!(
        r#"{{"model":"eye","max_tokens":120,"temperature":0,"chat_template_kwargs":{{"enable_thinking":false}},"response_format":{{"type":"json_schema","json_schema":{{"name":"cells","strict":true,"schema":{{"type":"object","additionalProperties":false,"required":["look","fingers_cell","thing_cell","close","fast"],"properties":{{"look":{{"type":"string","maxLength":80}},"fingers_cell":{{"type":"integer","minimum":1,"maximum":{格数}}},"thing_cell":{{"type":"integer","minimum":1,"maximum":{格数}}},"close":{{"type":"boolean"}},"fast":{{"type":"boolean"}}}}}}}}}},"messages":[{{"role":"user","content":[{{"type":"image_url","image_url":{{"url":"data:image/bmp;base64,{b64}"}}}},{{"type":"text","text":"{prompt}"}}]}}]}}"#
    );
    let raw = post(host, port, "/v1/chat/completions", &body)?;
    let inner = extract_content(&raw)
        .ok_or_else(|| format!("回包里没有 content(前 200 字:{})", &raw[..raw.len().min(200)]))?;
    let j = crate::json::parse(&inner)
        .map_err(|e| format!("眼给的不是 JSON: {e} ‖ 前 200 字:{}", &inner[..inner.len().min(200)]))?;
    let n = |k: &str| j.get(k).and_then(|x| x.num()).map(|v| v.round() as usize);
    let (Some(f), Some(t)) = (n("fingers_cell"), n("thing_cell")) else { return Err("眼没给格号".into()) };
    if f == 0 || f > 格数 || t == 0 || t > 格数 { return Err(format!("格号不合法:{f}/{t}")) }
    Ok(两格 { 指尖格: f, 目标格: t, 快: j.get("fast").and_then(|x| x.boolean()).unwrap_or(false), 合: j.get("close").and_then(|x| x.boolean()).unwrap_or(false),
             看到: j.get("look").and_then(|x| x.text()).unwrap_or("").to_string() })
}

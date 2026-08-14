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
    /// 动词。
    pub verb: String,
    /// 粗略力度。
    pub force: String,
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
    let prompt = format!(
        "Task: {what}\\n\\nWhich single object in this image should the robot act on? \
         Set present=false if it is not visible. If present, point to it: give its centre as \
         normalised image coordinates u (0=left,1=right) and v (0=top,1=bottom)."
    );
    let body = format!(
        r#"{{"model":"eye","max_tokens":600,"temperature":0,"response_format":{{"type":"json_schema","json_schema":{{"name":"contact_ask","strict":true,"schema":{{"type":"object","additionalProperties":false,"required":["u","v","span_frac","verb","force"],"properties":{{"u":{{"type":"number","minimum":0,"maximum":1}},"v":{{"type":"number","minimum":0,"maximum":1}},"span_frac":{{"type":"number","minimum":0,"maximum":1}},"verb":{{"type":"string","enum":["grasp","push","place","pry","open","close"]}},"force":{{"type":"string","enum":["light","medium","firm"]}}}}}}}}}},"messages":[{{"role":"user","content":[{{"type":"image_url","image_url":{{"url":"data:image/bmp;base64,{b64}"}}}},{{"type":"text","text":"{prompt}"}}]}}]}}"#
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
    let txt = |k: &str| -> String {
        j.get(k).and_then(|x| x.text()).unwrap_or("").to_string()
    };
    // 🔴 **不要往这张表里加格子,也不要把结论放在证据前面。** 两条都是实测:
    // 五格稳,加到九格时点位退化成 `1,1`;而把 `status` 这类结论放在第一个字段时,
    // 自回归解码让它在看到 u/v 之前就得先盖章 —— 三问全答弃权,而同一次里 u/v 写的是对的。
    // ⚠️ 「让眼自己说看不见」这条路本仓已判过:加弃权选项 / 调字段序 / 先列后选,三种都没治住。
    //    弃权要靠**几何自证**(拿它给的像素去看那处到底有没有东西),不靠它的自述。
    Ok(Look {
        u: num("u")?,
        v: num("v")?,
        span_frac: num("span_frac").unwrap_or(f64::NAN),
        verb: txt("verb"),
        force: txt("force"),
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

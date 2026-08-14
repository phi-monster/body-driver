//! 一个够用的 JSON 读取器。**没有任何依赖** —— 整个驱动的依赖表是空的,那是故意的
//! (见 `Cargo.toml`:每多一个依赖,就多一份审计者也必须读的代码)。
//!
//! # 它为什么必须存在
//!
//! 这具身体量出来的东西 —— 接触阈值 0.29383、工具偏置 0.1451、够得到的范围 —— 从 2026-08-09 起
//! 一直躺在 `/root/bodycal/<指纹>.json` 里,而**读它的代码住在驱动外面的 Python 里**。
//! 于是"身体常数只能向驱动要"这条规矩,实现上是靠一个驱动管不着的文件在维持。
//! 把读取搬进来之后,驱动才真的是那批数字的唯一出口。

use std::collections::BTreeMap;

/// 一个 JSON 值。
#[derive(Clone, Debug, PartialEq)]
pub enum Json {
    /// `null`
    Null,
    /// `true` / `false`
    Bool(bool),
    /// 数字。JSON 不分整浮点。
    Num(f64),
    /// 字符串,转义已还原。
    Str(String),
    /// 数组
    Arr(Vec<Json>),
    /// 对象。用有序表,遍历顺序才可复现。
    Obj(BTreeMap<String, Json>),
}

impl Json {
    /// 取对象里的一个键。不是对象就返回 `None`。
    pub fn get(&self, k: &str) -> Option<&Json> {
        match self {
            Json::Obj(m) => m.get(k),
            _ => None,
        }
    }

    /// 当成数看。数组取第 0 个 —— 本仓的量都写成 `[0.29383]` 这种一维数组。
    pub fn num(&self) -> Option<f64> {
        match self {
            Json::Num(v) => Some(*v),
            Json::Arr(a) => a.first().and_then(|x| x.num()),
            _ => None,
        }
    }

    /// 当成一串数看。
    pub fn nums(&self) -> Vec<f64> {
        match self {
            Json::Num(v) => vec![*v],
            Json::Arr(a) => a.iter().filter_map(|x| x.num()).collect(),
            _ => vec![],
        }
    }

    /// 当成真假看。**不认 `"true"` 这种字符串** —— 一个把字符串当真值读的取值器,会把
    /// `"false"` 读成真,而那正是"看不见"这一格最不能出错的方向。
    pub fn boolean(&self) -> Option<bool> {
        match self {
            Json::Bool(b) => Some(*b),
            _ => None,
        }
    }

    /// 当成字符串看。
    pub fn text(&self) -> Option<&str> {
        match self {
            Json::Str(s) => Some(s),
            _ => None,
        }
    }
}

/// 解析一整份 JSON。出错时给出**字节偏移**,因为标定文件是机器写的,人读不出哪里坏了。
pub fn parse(src: &str) -> Result<Json, String> {
    let b = src.as_bytes();
    let mut i = 0usize;
    let v = value(b, &mut i)?;
    skip_ws(b, &mut i);
    if i != b.len() {
        return Err(format!("尾部还有 {} 字节没吃掉,偏移 {i}", b.len() - i));
    }
    Ok(v)
}

fn skip_ws(b: &[u8], i: &mut usize) {
    while *i < b.len() && matches!(b[*i], b' ' | b'\t' | b'\n' | b'\r') {
        *i += 1;
    }
}

fn value(b: &[u8], i: &mut usize) -> Result<Json, String> {
    skip_ws(b, i);
    if *i >= b.len() {
        return Err(format!("提前结束,偏移 {i}"));
    }
    match b[*i] {
        b'{' => object(b, i),
        b'[' => array(b, i),
        b'"' => string(b, i).map(Json::Str),
        b't' => lit(b, i, "true", Json::Bool(true)),
        b'f' => lit(b, i, "false", Json::Bool(false)),
        b'n' => lit(b, i, "null", Json::Null),
        _ => number(b, i),
    }
}

fn lit(b: &[u8], i: &mut usize, want: &str, v: Json) -> Result<Json, String> {
    if b[*i..].starts_with(want.as_bytes()) {
        *i += want.len();
        Ok(v)
    } else {
        Err(format!("偏移 {i} 处不是 {want}"))
    }
}

fn object(b: &[u8], i: &mut usize) -> Result<Json, String> {
    *i += 1; // '{'
    let mut m = BTreeMap::new();
    skip_ws(b, i);
    if *i < b.len() && b[*i] == b'}' {
        *i += 1;
        return Ok(Json::Obj(m));
    }
    loop {
        skip_ws(b, i);
        let k = string(b, i)?;
        skip_ws(b, i);
        if *i >= b.len() || b[*i] != b':' {
            return Err(format!("偏移 {i} 处缺冒号"));
        }
        *i += 1;
        let v = value(b, i)?;
        m.insert(k, v);
        skip_ws(b, i);
        match b.get(*i) {
            Some(b',') => *i += 1,
            Some(b'}') => {
                *i += 1;
                return Ok(Json::Obj(m));
            }
            _ => return Err(format!("偏移 {i} 处对象没收尾")),
        }
    }
}

fn array(b: &[u8], i: &mut usize) -> Result<Json, String> {
    *i += 1; // '['
    let mut a = Vec::new();
    skip_ws(b, i);
    if *i < b.len() && b[*i] == b']' {
        *i += 1;
        return Ok(Json::Arr(a));
    }
    loop {
        a.push(value(b, i)?);
        skip_ws(b, i);
        match b.get(*i) {
            Some(b',') => *i += 1,
            Some(b']') => {
                *i += 1;
                return Ok(Json::Arr(a));
            }
            _ => return Err(format!("偏移 {i} 处数组没收尾")),
        }
    }
}

fn string(b: &[u8], i: &mut usize) -> Result<String, String> {
    if *i >= b.len() || b[*i] != b'"' {
        return Err(format!("偏移 {i} 处不是字符串"));
    }
    *i += 1;
    let mut s = String::new();
    while *i < b.len() {
        match b[*i] {
            b'"' => {
                *i += 1;
                return Ok(s);
            }
            b'\\' => {
                *i += 1;
                let c = *b.get(*i).ok_or_else(|| format!("转义在偏移 {i} 处断了"))?;
                *i += 1;
                match c {
                    b'"' => s.push('"'),
                    b'\\' => s.push('\\'),
                    b'/' => s.push('/'),
                    b'b' => s.push('\u{8}'),
                    b'f' => s.push('\u{c}'),
                    b'n' => s.push('\n'),
                    b'r' => s.push('\r'),
                    b't' => s.push('\t'),
                    b'u' => {
                        // 只还原基本平面;代理对按原样保留成替换字符,不猜。
                        let h = std::str::from_utf8(b.get(*i..*i + 4).unwrap_or(b""))
                            .map_err(|_| format!("\\u 在偏移 {i} 处坏了"))?;
                        let cp = u32::from_str_radix(h, 16)
                            .map_err(|_| format!("\\u 在偏移 {i} 处不是十六进制"))?;
                        *i += 4;
                        s.push(char::from_u32(cp).unwrap_or('\u{fffd}'));
                    }
                    _ => return Err(format!("偏移 {i} 处认不得的转义")),
                }
            }
            _ => {
                // 多字节 UTF-8 原样搬过去。
                let start = *i;
                let n = utf8_len(b[*i]);
                *i += n;
                match std::str::from_utf8(&b[start..(*i).min(b.len())]) {
                    Ok(t) => s.push_str(t),
                    Err(_) => return Err(format!("偏移 {start} 处不是合法 UTF-8")),
                }
            }
        }
    }
    Err("字符串没收尾".into())
}

fn utf8_len(c: u8) -> usize {
    if c < 0x80 {
        1
    } else if c >> 5 == 0b110 {
        2
    } else if c >> 4 == 0b1110 {
        3
    } else {
        4
    }
}

fn number(b: &[u8], i: &mut usize) -> Result<Json, String> {
    let start = *i;
    if *i < b.len() && (b[*i] == b'-' || b[*i] == b'+') {
        *i += 1;
    }
    while *i < b.len() && (b[*i].is_ascii_digit() || matches!(b[*i], b'.' | b'e' | b'E' | b'-' | b'+')) {
        *i += 1;
    }
    std::str::from_utf8(&b[start..*i])
        .ok()
        .and_then(|t| t.parse::<f64>().ok())
        .map(Json::Num)
        .ok_or_else(|| format!("偏移 {start} 处不是数"))
}

impl Json {
    /// 写回 JSON 正文。**缩进两格、键有序**(`Obj` 用的是有序表)⇒ 同样的内容永远写出同样的字节,
    /// 于是标定文件的 diff 只反映**真正变了的量**,而不是序列化顺序抖动。
    pub fn dump(&self, indent: usize) -> String {
        let pad = "  ".repeat(indent);
        let pad1 = "  ".repeat(indent + 1);
        match self {
            Json::Null => "null".into(),
            Json::Bool(b) => b.to_string(),
            Json::Num(v) => {
                if v.fract() == 0.0 && v.abs() < 1e15 {
                    format!("{}", *v as i64)
                } else {
                    format!("{v}")
                }
            }
            Json::Str(t) => quote(t),
            Json::Arr(a) => {
                if a.is_empty() {
                    return "[]".into();
                }
                let body: Vec<String> = a.iter().map(|x| format!("{pad1}{}", x.dump(indent + 1))).collect();
                format!("[\n{}\n{pad}]", body.join(",\n"))
            }
            Json::Obj(m) => {
                if m.is_empty() {
                    return "{}".into();
                }
                let body: Vec<String> = m
                    .iter()
                    .map(|(k, v)| format!("{pad1}{}: {}", quote(k), v.dump(indent + 1)))
                    .collect();
                format!("{{\n{}\n{pad}}}", body.join(",\n"))
            }
        }
    }
}

fn quote(t: &str) -> String {
    let mut s = String::with_capacity(t.len() + 2);
    s.push('"');
    for c in t.chars() {
        match c {
            '"' => s.push_str("\\\""),
            '\\' => s.push_str("\\\\"),
            '\n' => s.push_str("\\n"),
            '\r' => s.push_str("\\r"),
            '\t' => s.push_str("\\t"),
            c if (c as u32) < 0x20 => s.push_str(&format!("\\u{:04x}", c as u32)),
            c => s.push(c),
        }
    }
    s.push('"');
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_the_shape_a_calibration_file_actually_has() {
        let src = r#"{"fingerprint":"116bd6e559e10b06","quantities":{
            "contact_threshold":{"value":[0.29383],"uncertainty":[0.00039],
              "selftest_passed":true,"unit":"fraction of commanded motion delivered"},
            "gripper_span":{"refused":"NoResponse: the commanded motion did not move the signal",
              "selftest_passed":false}}}"#;
        let j = parse(src).expect("该解析得动");
        assert_eq!(j.get("fingerprint").and_then(|x| x.text()), Some("116bd6e559e10b06"));
        let q = j.get("quantities").expect("有 quantities");
        assert_eq!(q.get("contact_threshold").and_then(|x| x.get("value")).and_then(|x| x.num()),
                   Some(0.29383));
        // 🔴 被拒绝的量必须读出来是【拒绝】,不能读成 0 —— 读成 0 就等于悄悄发明了一个身体常数。
        assert!(q.get("gripper_span").and_then(|x| x.get("value")).is_none());
        assert!(q.get("gripper_span").and_then(|x| x.get("refused")).and_then(|x| x.text()).is_some());
    }

    #[test]
    fn escapes_and_unicode_survive() {
        let j = parse(r#"{"a":"x\ny \"q\" µm"}"#).unwrap();
        assert_eq!(j.get("a").and_then(|x| x.text()), Some("x\ny \"q\" µm"));
    }

    #[test]
    fn what_we_write_reads_back_identically() {
        // 🔴 写标定文件这件事一旦失真,失真的是**身体常数**,而且没有任何下游会不一致。
        //    所以往返必须逐字节稳定:写 → 读 → 再写,两次结果相同。
        let src = r#"{"fingerprint":"abc","quantities":{"gripper_span":{"value":[0.0803],
            "uncertainty":[0.0001],"selftest_passed":true,"unit":"metres between the jaws",
            "provenance":"line1\nline2 with \"quotes\""}}}"#;
        let a = parse(src).unwrap();
        let once = a.dump(0);
        let twice = parse(&once).unwrap().dump(0);
        assert_eq!(once, twice, "写出来的东西读回去不一样");
        // 内容也要在
        let b = parse(&once).unwrap();
        assert_eq!(
            b.get("quantities").and_then(|x| x.get("gripper_span")).and_then(|x| x.get("value")).and_then(|x| x.num()),
            Some(0.0803)
        );
        assert!(b.get("quantities").and_then(|x| x.get("gripper_span"))
            .and_then(|x| x.get("provenance")).and_then(|x| x.text()).unwrap().contains("quotes"));
    }

    #[test]
    fn trailing_garbage_is_an_error_not_a_shrug() {
        assert!(parse(r#"{"a":1} junk"#).is_err());
    }
}

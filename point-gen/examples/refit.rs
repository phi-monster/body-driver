//! 离线重放:把跨度相落盘的 `seen`(带四元数)喂给 `fit_full_offset`。
//! 用法:cargo run --release --example refit -- <span_raw.txt>
fn main() {
    let path = std::env::args().nth(1).expect("要 span_raw.txt");
    let txt = std::fs::read_to_string(&path).expect("读不到");
    let mut seen: Vec<([f64; 7], point_gen::Px)> = Vec::new();
    for l in txt.lines() {
        let t: Vec<&str> = l.split_whitespace().collect();
        if t.len() == 11 && t[0] == "seen" && t[1] == "0" {
            let f = |i: usize| t[i].parse::<f64>().unwrap();
            seen.push(([f(4), f(5), f(6), f(7), f(8), f(9), f(10)], [f(2), f(3)]));
        }
    }
    eprintln!("样本 {} 组(7 维)", seen.len());
    match point_gen::fit_full_offset(&seen) {
        Err(e) => eprintln!("拒绝:{e:?}"),
        Ok((eye, d, med)) => eprintln!(
            "🟢 fx={:.4} fy={:.4} 主点=({:.4},{:.4}) at=({:.3},{:.3},{:.3}) q=({:.3},{:.3},{:.3},{:.3}) · d=({:.4},{:.4},{:.4}) · 留出中位 {:.4}",
            eye.fx, eye.fy, eye.cx, eye.cy, eye.at[0], eye.at[1], eye.at[2],
            eye.q[0], eye.q[1], eye.q[2], eye.q[3], d[0], d[1], d[2], med),
    }
}

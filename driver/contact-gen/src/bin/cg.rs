//! `cg` —— ②a 也变成一个**进程**,和驱动同一个理由:一行一问,一行一答,谁都能问。
//!
//! # 🔴 为什么身体常数是【在协议里说出来的】,而不是它自己去读
//!
//! 这一层不许依赖 body-layer(见 `lib.rs` 头注:反向依赖一出现,"换机体不重训"就没有机制保证)。
//! 所以调用方得先问驱动要,再把要到的数**连同它的出处**递进来。
//! **这堵墙因此写在协议里,肉眼可见** —— `body measured 0.088 …` 和 `body declared 0.088 …`
//! 是两条不同的命令,后者产出的每一条候选都背着"这是声明值"的标记。
//!
//! # 协议
//!
//! ```text
//! body <measured|declared|unknown> <爪张开 m> <够到下界 m> <够到上界 m> <臂根 x> <臂根 y> -> ok | err
//! pts  <点文件路径>        每行 "x y z",米,世界坐标        -> ok <点数> | err
//! grid <层数> <方向数> <每层最少点数> <分块距离 m> [指头宽 m] [离桌下限 m] [爪面高 m] -> ok
//! gen  <支撑面 z>          -> 每条候选一行,末行 end          | refused <理由>
//! at   <x> <y> <z> <爪面朝向> [层厚] [指头宽]
//!                          -> val <米> | none      「爪子停在这儿,那一条上料多厚」
//! quit
//! ```
//!
//! 候选行的字段依次是:
//! `x y z 合爪朝向 这段多宽 余量 离支撑面多高 离臂根多远 够得到吗 爪值是声明的吗 这段几个点`

use contact_gen::{candidates, thickness_at, Body, Grid, JawSpan, P3, Refusal};
use std::io::{self, BufRead, Write};

fn main() {
    let stdin = io::stdin();
    let mut out = io::stdout();
    // 🔴 默认是 Unknown,不是某个数。忘了 `body` 那一行 ⇒ 拒绝,而不是拿一个猜的值往下跑。
    let mut body = Body {
        jaw: JawSpan::Unknown,
        reach_lo: 0.0,
        reach_hi: f64::MAX,
        base_x: 0.0,
        base_y: 0.0,
    };
    let mut grid = Grid::default();
    let mut pts: Vec<P3> = Vec::new();

    for line in stdin.lock().lines() {
        let Ok(line) = line else { break };
        let t: Vec<&str> = line.split_whitespace().collect();
        if t.is_empty() {
            continue;
        }
        if t[0] == "quit" {
            break;
        }
        let reply = match t[0] {
            "body" => set_body(&t, &mut body),
            "grid" => set_grid(&t, &mut grid),
            "pts" => load_pts(&t, &mut pts),
            "gen" => gen(&t, &body, &grid, &pts),
            // 爪子最后停在这儿 —— 那一条上料有多厚。标定爪张开度要的是这个,
            // 不是"我打算夹的那一段有多厚"(实测:同一爪停位置,积木 0.0048 / 扳手 0.0351,差 7 倍)。
            "at" => match nums(&t, 4) {
                None => "err at 要 <x> <y> <z> <爪面朝向> [层厚] [指头宽]".into(),
                Some(a) => {
                    let bh = t.get(5).and_then(|x| x.parse().ok()).unwrap_or(0.01);
                    let fw = t.get(6).and_then(|x| x.parse().ok()).unwrap_or(grid.finger_w_m);
                    match thickness_at(&pts, a[0], a[1], a[2], a[3], bh, fw) {
                        // 🔴 那一条上没有料 ⇒ 说"没有",不许报 0。
                        None => "none".into(),
                        Some(w) => format!("val {w:.5}"),
                    }
                }
            },
            _ => "err unknown".into(),
        };
        let _ = writeln!(out, "{reply}");
        let _ = out.flush();
    }
}

fn set_body(t: &[&str], body: &mut Body) -> String {
    if t.len() < 7 {
        return "err body 要 <出处> <爪张开> <够到下界> <够到上界> <臂根x> <臂根y>".into();
    }
    let n: Vec<f64> = match t[2..7].iter().map(|x| x.parse::<f64>().ok()).collect() {
        Some(v) => v,
        None => return "err body 的数读不出来".into(),
    };
    body.jaw = match t[1] {
        "measured" => JawSpan::Measured(n[0]),
        "declared" => JawSpan::Declared(n[0]),
        "unknown" => JawSpan::Unknown,
        _ => return "err 爪张开度的出处只能是 measured / declared / unknown".into(),
    };
    body.reach_lo = n[1];
    body.reach_hi = n[2];
    body.base_x = n[3];
    body.base_y = n[4];
    "ok".into()
}

fn set_grid(t: &[&str], grid: &mut Grid) -> String {
    if t.len() < 5 {
        return "err grid 要 <层数> <方向数> <每层最少点数> <分块距离> [指头宽] [离桌下限] [爪面高]".into();
    }
    let (Ok(b), Ok(d), Ok(m), Ok(g)) = (
        t[1].parse::<u32>(),
        t[2].parse::<u32>(),
        t[3].parse::<u32>(),
        t[4].parse::<f64>(),
    ) else {
        return "err grid 的数读不出来".into();
    };
    let fw = t.get(5).and_then(|x| x.parse().ok()).unwrap_or(grid.finger_w_m);
    let ab = t.get(6).and_then(|x| x.parse().ok()).unwrap_or(grid.min_above_m);
    let jh = t.get(7).and_then(|x| x.parse().ok()).unwrap_or(grid.jaw_h_m);
    *grid = Grid {
        bands: b,
        dirs: d,
        min_pts: m,
        min_above_m: ab,
        jaw_h_m: jh,
        finger_w_m: fw,
        gap_m: g,
    };
    "ok".into()
}

fn load_pts(t: &[&str], pts: &mut Vec<P3>) -> String {
    let Some(p) = t.get(1) else {
        return "err pts 少了路径".into();
    };
    let src = match std::fs::read_to_string(p) {
        Ok(s) => s,
        Err(e) => return format!("err 读不了 {p}: {e}"),
    };
    pts.clear();
    let mut bad = 0usize;
    for ln in src.lines() {
        let f: Vec<&str> = ln.split_whitespace().collect();
        if f.len() < 3 {
            if !ln.trim().is_empty() {
                bad += 1;
            }
            continue;
        }
        match (f[0].parse(), f[1].parse(), f[2].parse()) {
            (Ok(x), Ok(y), Ok(z)) => pts.push(P3 { x, y, z }),
            _ => bad += 1,
        }
    }
    // 读坏的行数报出来,不吞 —— 悄悄少读一半的点会让"这东西夹不住"变成一个假结论。
    format!("ok {} bad {}", pts.len(), bad)
}

fn gen(t: &[&str], body: &Body, grid: &Grid, pts: &[P3]) -> String {
    let sz: f64 = t.get(1).and_then(|x| x.parse().ok()).unwrap_or(0.0);
    match candidates(pts, body, sz, *grid) {
        Err(r) => format!("refused {}", why(r)),
        Ok(c) => {
            let mut s = String::new();
            for k in &c {
                s.push_str(&format!(
                    "{:.4} {:.4} {:.4} {:.4} {:.4} {:.4} {:.4} {:.4} {} {} {} {:.4} {}\n",
                    k.point.x,
                    k.point.y,
                    k.point.z,
                    k.close_yaw,
                    k.width_m,
                    k.margin_m,
                    k.above_support_m,
                    k.reach_r,
                    k.reachable as u8,
                    k.jaw_declared as u8,
                    k.n_pts,
                    // 🔴 排序用到的量不许只活在进程里 —— 记录里看不到它,诊断时就只能猜。
                    //    2026-08-12 实测:鞋那一腿排错了,而落盘的候选里恰好没有 `depth_m`。
                    k.depth_m,
                    k.within_jaw as u8
                ));
            }
            s.push_str("end");
            s
        }
    }
}

fn why(r: Refusal) -> &'static str {
    match r {
        Refusal::JawSpanUnknown => "爪张开度既没量到也没声明 —— 夹不夹得下这一问无法回答",
        Refusal::TooFewPoints => "表面点太少,算不出截面",
        Refusal::Flat => "这块表面是平的,切不出层",
        Refusal::NoSection => "一条候选都算不出来:点太少,或者每一条的跨度都是零",
    }
}

fn nums(t: &[&str], n: usize) -> Option<Vec<f64>> {
    if t.len() < n + 1 {
        return None;
    }
    t[1..=n].iter().map(|x| x.parse::<f64>().ok()).collect()
}

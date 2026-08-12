//! `bl` —— 把驱动变成一个**进程**,而不是一个必须为每种语言写绑定的库。
//!
//! # 它替掉了什么
//!
//! 在此之前,想用这一层就得先写一层 ctypes 壳(`bind/python/body_layer.py`,676 行),
//! 而那层壳住在驱动树里。于是"驱动"这个词同时指两样东西:一份 Rust,和一份跟着它漂的 Python。
//! 换一种宿主语言 = 再写一份壳;换一台真机 = 那份壳也得跟去。
//!
//! 现在:一行一问,一行一答,走标准输入输出。任何东西都能问它 —— 一个 shell、一个仿真进程、
//! 一台真机上的控制器、一个大模型。**驱动树里因此可以一行 Python 都没有。**
//!
//! # 协议(一行一条,空白分隔)
//!
//! ```text
//! load <标定文件路径>      -> ok <量到的> <总格数> <指纹>   | err <为什么>
//! ask  <量名>              -> val <v0> [v1..] # <单位>      | refused <理由> | never
//! why  <量名>              -> <出处原文>                     | refused <理由> | never
//! list                     -> <所有格名>
//! touch <命令 m> <实到 m> <阈值> <物体动了 m> <当动的门槛 m>  -> 1 | 0
//! span  <这段宽 m> <爪张开 m> <余量 m>                        -> 1 | 0
//! check <打算夹 m> <爪值> <爪张开 m> <物体动了 m> <碰跑门槛> <容差比> -> asplanned|closedonair|wrongsection|knockedaway
//! next  <动词> <自查结果>  -> proceed | nextcontact | changeverb <动词> | relook
//! move  <动词> <工具轴 xyz> <钳口轴 xyz> <方向 xyz> <多少>
//!        -> along <xyz> <米> about <xyz> <弧度> rotates <0|1> turnfirst <0|1>
//!        两条轴都要给:绕工具轴 = 原地自转(拧),绕钳口轴 = 扳倒/倾倒。挑哪条由驱动定。
//! jawcal <对子文件> [1σ]   每行 "爪停值 真实料厚(米)"
//!                          -> val <米> sigma <米> valid <..> n <个数> | refused <理由>
//! homecal <归位记录文件> [散布上限]  每行 "x y z qw qx qy qz"
//!                          -> val <xyz> q <四元数> spread <米> n <次数> | refused <理由>
//! athome <x> <y> <z> <几倍散布>  -> 1 | 0 | refused <理由>
//! verbs                    -> <所有动词名>
//! quit
//! ```
//!
//! 认不得的行答 `err unknown`。**任何一条都不会悄悄给默认值** —— 这一层的全部意义就是
//! "没量到就说没量到"。

use body_layer::store::{Answer, Store};
use body_layer::verb::{
    classify, contact_seen, decide, demand, spannable, turn_before_lift, Axes, Check, Next, Verb,
};
use body_layer::measurement::Quantity;
use body_layer::probe::{at_home, gripper_span_by_stall, home_pose};
use std::io::{self, BufRead, Write};

thread_local! {
    /// 本次会话 `homecal` 量到的原位。`athome` 拿它当尺子。
    static HOME: std::cell::RefCell<Option<body_layer::measurement::Measurement>> =
        const { std::cell::RefCell::new(None) };
}

fn main() {
    let stdin = io::stdin();
    let mut out = io::stdout();
    let mut store: Option<Store> = None;

    for line in stdin.lock().lines() {
        let line = match line {
            Ok(l) => l,
            Err(_) => break,
        };
        let t: Vec<&str> = line.split_whitespace().collect();
        if t.is_empty() {
            continue;
        }
        if t[0] == "quit" {
            break;
        }
        let reply = handle(&t, &mut store);
        let _ = writeln!(out, "{reply}");
        let _ = out.flush();
    }
}

fn handle(t: &[&str], store: &mut Option<Store>) -> String {
    match t[0] {
        "load" => match t.get(1) {
            None => "err load 少了路径".into(),
            Some(p) => match std::fs::read_to_string(p) {
                Err(e) => format!("err 读不了 {p}: {e}"),
                Ok(src) => match Store::from_str(&src) {
                    Err(e) => format!("err 解析不了 {p}: {e}"),
                    Ok(s) => {
                        let (ok, all) = s.tally();
                        let fp = s.fingerprint.clone();
                        *store = Some(s);
                        format!("ok {ok} {all} {fp}")
                    }
                },
            },
        },
        "ask" | "why" => {
            let Some(s) = store.as_ref() else {
                return "err 还没 load 标定".into();
            };
            let Some(name) = t.get(1) else {
                return format!("err {} 少了量名", t[0]);
            };
            match s.ask(name) {
                Answer::NeverMeasured => "never".into(),
                Answer::Refused { why } => format!("refused {why}"),
                Answer::Measured {
                    value,
                    unit,
                    provenance,
                    ..
                } => {
                    if t[0] == "why" {
                        if provenance.is_empty() {
                            // 没有出处的数不该被当成量出来的。说出来,别装作有。
                            "refused 有值但没有出处".into()
                        } else {
                            provenance.replace('\n', " ")
                        }
                    } else {
                        let v: Vec<String> = value.iter().map(|x| format!("{x}")).collect();
                        format!("val {} # {}", v.join(" "), unit.replace('\n', " "))
                    }
                }
            }
        }
        "list" => match store.as_ref() {
            None => "err 还没 load 标定".into(),
            Some(s) => s.names().join(" "),
        },
        "touch" => match nums(t, 5) {
            None => "err touch 要 5 个数".into(),
            Some(a) => bit(contact_seen(a[0], a[1], a[2], a[3], a[4])),
        },
        "span" => match nums(t, 3) {
            None => "err span 要 3 个数".into(),
            Some(a) => bit(spannable(a[0], a[1], a[2])),
        },
        "check" => match nums(t, 6) {
            None => "err check 要 6 个数".into(),
            Some(a) => check_name(classify(a[0], a[1], a[2], a[3], a[4], a[5])).into(),
        },
        "next" => {
            let (Some(v), Some(c)) = (t.get(1).and_then(|x| verb_of(x)), t.get(2).and_then(|x| check_of(x)))
            else {
                return "err next 要 <动词> <自查结果>".into();
            };
            match decide(v, c) {
                Next::Proceed => "proceed".into(),
                Next::NextContact => "nextcontact".into(),
                Next::Relook => "relook".into(),
                Next::ChangeVerb(nv) => format!("changeverb {}", verb_name(nv)),
            }
        }
        // 接触集的第三格:**物体**要怎么动。炮一的装置闸认的就是 rotates=1。
        "move" => {
            let Some(v) = t.get(1).and_then(|x| verb_of(x)) else {
                return "err move 要 <动词> <轴xyz> <方向xyz> <多少>".into();
            };
            let Some(a) = nums(&t[1..], 10) else {
                return "err move 要 <动词> <工具轴xyz> <钳口轴xyz> <方向xyz> <多少>".into();
            };
            // 🔴 两条轴都要给,由驱动挑 —— 让调用方挑就是把 LAB "TWIST 判词反转" 那条教训
            //    交还给"人记不记得",而它已经错过一次了。
            let ax = Axes { tool: [a[0], a[1], a[2]], jaw: [a[3], a[4], a[5]] };
            let m = demand(v, ax, [a[6], a[7], a[8]], a[9]);
            format!(
                "along {} {} {} {} about {} {} {} {} rotates {} turnfirst {}",
                m.along[0], m.along[1], m.along[2], m.dist_m,
                m.about[0], m.about[1], m.about[2], m.turn_rad,
                m.rotates() as u8,
                turn_before_lift(v) as u8
            )
        }
        // 爪张开度的第二条测法(不用相机):喂一份 "爪停值 真实料厚" 的表,驱动自己拟合。
        // 🔴 拟合必须在驱动里做,不能在策略里做 —— 身体常数只有一个出口,这一条也不例外。
        "jawcal" => {
            let Some(path) = t.get(1) else {
                return "err jawcal 要 <对子文件> [已知宽度的1σ]".into();
            };
            let sigma: f64 = t.get(2).and_then(|x| x.parse().ok()).unwrap_or(0.001);
            let src = match std::fs::read_to_string(path) {
                Ok(s) => s,
                Err(e) => return format!("err 读不了 {path}: {e}"),
            };
            let mut pairs: Vec<(f64, f64)> = Vec::new();
            for ln in src.lines() {
                let f: Vec<&str> = ln.split_whitespace().collect();
                if f.len() < 2 {
                    continue;
                }
                if let (Ok(a), Ok(b)) = (f[0].parse::<f64>(), f[1].parse::<f64>()) {
                    pairs.push((a, b));
                }
            }
            match gripper_span_by_stall(&pairs, sigma, 0, 0) {
                Err(d) => format!("refused {d:?} (n={})", pairs.len()),
                Ok(m) => format!(
                    "val {:.5} sigma {:.5} valid {:.5}..{:.5} n {} dep {:?}",
                    m.value[0], m.uncertainty[0], m.valid_lo[0], m.valid_hi[0], pairs.len(),
                    m.deps[0].map(|(q, _)| q as u32 == Quantity::ContactThreshold as u32)
                ),
            }
        }
        // 原位:归位若干次 → 位形 + **重复性**。容差是量出来的,不是拍的。
        "homecal" => {
            let Some(path) = t.get(1) else {
                return "err homecal 要 <归位记录文件> [散布上限 m]".into();
            };
            let cap: f64 = t.get(2).and_then(|x| x.parse().ok()).unwrap_or(0.02);
            let src = match std::fs::read_to_string(path) {
                Ok(s) => s,
                Err(e) => return format!("err 读不了 {path}: {e}"),
            };
            let mut rows: Vec<[f64; 7]> = Vec::new();
            for ln in src.lines() {
                let f: Vec<&str> = ln.split_whitespace().collect();
                if f.len() < 7 {
                    continue;
                }
                let mut a = [0.0f64; 7];
                let mut ok = true;
                for i in 0..7 {
                    match f[i].parse::<f64>() {
                        Ok(v) => a[i] = v,
                        Err(_) => ok = false,
                    }
                }
                if ok {
                    rows.push(a);
                }
            }
            match home_pose(&rows, cap, 0) {
                Err(d) => format!("refused {d:?} (n={})", rows.len()),
                Ok(m) => {
                    HOME.with(|h| *h.borrow_mut() = Some(m));
                    format!(
                        "val {:.5} {:.5} {:.5} q {:.5} {:.5} {:.5} {:.5} spread {:.5} n {}",
                        m.value[0], m.value[1], m.value[2],
                        m.value[3], m.value[4], m.value[5], m.value[6],
                        m.uncertainty[0], rows.len()
                    )
                }
            }
        }
        // 回到原位了没有。容差 = tol_k x 量出来的散布;tol_k < 1 一律拒绝回答。
        "athome" => {
            let Some(a) = nums(t, 4) else {
                return "err athome 要 <x> <y> <z> <几倍散布>".into();
            };
            HOME.with(|h| match h.borrow().as_ref() {
                None => "err 还没 homecal".to_string(),
                Some(m) => match at_home(&[a[0], a[1], a[2]], m, a[3]) {
                    // 🔴 拒绝回答 ≠ 判"没到位"。要求身体比自己的重复性还准,那永远判不过。
                    None => "refused 容差比这具身体自己的重复性还紧".to_string(),
                    Some(true) => "1".to_string(),
                    Some(false) => "0".to_string(),
                },
            })
        }
        "verbs" => (0..13u32)
            .filter_map(Verb::from_u32)
            .map(verb_name)
            .collect::<Vec<_>>()
            .join(" "),
        _ => "err unknown".into(),
    }
}

fn nums(t: &[&str], n: usize) -> Option<Vec<f64>> {
    if t.len() < n + 1 {
        return None;
    }
    t[1..=n].iter().map(|x| x.parse::<f64>().ok()).collect()
}

fn bit(b: bool) -> String {
    if b { "1".into() } else { "0".into() }
}

fn verb_name(v: Verb) -> &'static str {
    match v {
        Verb::Reach => "reach",
        Verb::Grasp => "grasp",
        Verb::Release => "release",
        Verb::Press => "press",
        Verb::Wipe => "wipe",
        Verb::Push => "push",
        Verb::Pry => "pry",
        Verb::Flip => "flip",
        Verb::Pour => "pour",
        Verb::Twist => "twist",
        Verb::Insert => "insert",
        Verb::Scoop => "scoop",
        Verb::Place => "place",
    }
}

fn verb_of(s: &str) -> Option<Verb> {
    // 数字也收 —— 眼睛那一侧递过来的就是 `bl_world_ref.verb` 里的那个整数。
    if let Ok(n) = s.parse::<u32>() {
        return Verb::from_u32(n);
    }
    (0..13u32)
        .filter_map(Verb::from_u32)
        .find(|v| verb_name(*v) == s)
}

fn check_name(c: Check) -> &'static str {
    match c {
        Check::AsPlanned => "asplanned",
        Check::ClosedOnAir => "closedonair",
        Check::WrongSection => "wrongsection",
        Check::KnockedAway => "knockedaway",
    }
}

fn check_of(s: &str) -> Option<Check> {
    Some(match s {
        "asplanned" | "0" => Check::AsPlanned,
        "closedonair" | "1" => Check::ClosedOnAir,
        "wrongsection" | "2" => Check::WrongSection,
        "knockedaway" | "3" => Check::KnockedAway,
        _ => return None,
    })
}

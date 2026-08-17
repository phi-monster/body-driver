//! **`bl-calibrate` —— 装上之后跑的第一条命令,也是唯一一条。**
//!
//! ```text
//! bl-calibrate --listen 9077          # 机器人的控制器连过来(仿真常见)
//! bl-calibrate --connect ws://arm:9077 # 我们连上机器人(真机常见)
//! ```
//!
//! 它做的事就是 ABI 头里那句话:
//!
//! > **THE POWER-ON SCHEDULE.** Plugging in a new machine is: ask what is still owed → run the
//! > probes it names → hand each measurement back → repeat until nothing is owed.
//! > **Nothing about the order is typed in per robot.**
//!
//! # 🔴 用户要写几行代码:零
//!
//! 这个程序自己去认这台机器人报回来的东西**长什么样**(见 [`discover`]),不读键名、
//! 不读 URDF、不读相机矩阵。认不出来就**拒绝**,并说清缺哪一格 —— 一个认错了的布局
//! 会让整轮自标定照常跑完并产出一批**看起来正常**的数,而那种数比没有数更贵。
//!
//! # 为什么这个文件不在驱动树里
//!
//! 它有依赖(WebSocket 握手要 SHA-1 + base64)。驱动是零依赖的,将来要跑在没有分配器故事
//! 的目标上。⇒ **插头在外面,身体在里面。** 换一种机器人 = 换一个插头。

mod discover;
mod wire;

use body_layer::measurement::Quantity;
use body_layer::probe;
use rmpv::Value;
use selfcal::robot::{Cmd, Frame, Robot};
use std::io::Write;

/// 一台通过 msgpack/WebSocket 说话的机器人。
struct Plug<S: std::io::Read + std::io::Write> {
    ws: tungstenite::WebSocket<S>,
    lay: discover::Layout,
    /// 最近一次收到的观测 —— `sense` 读它,`act` 发完等下一帧。
    last: Option<Value>,
}

fn 取(v: &Value, path: &[String]) -> Option<Value> {
    let mut cur = v.clone();
    for k in path {
        let m = cur.as_map()?.iter().find(|(kk, _)| kk.as_str() == Some(k.as_str()))?.1.clone();
        cur = m;
    }
    Some(cur)
}

fn 数组(v: &Value) -> Vec<f64> {
    v.as_array().map(|a| a.iter().filter_map(|x| x.as_f64()).collect()).unwrap_or_default()
}

impl<S: std::io::Read + std::io::Write> Robot for Plug<S> {
    fn sense(&mut self) -> Option<Frame> {
        let o = self.last.clone()?;
        let mut f = Frame::default();
        for p in &self.lay.joints {
            f.joints.push(取(&o, p).map(|v| 数组(&v)).unwrap_or_default());
        }
        for p in &self.lay.ee {
            let a = 取(&o, p).map(|v| 数组(&v)).unwrap_or_default();
            if a.len() == 7 {
                f.ee.push([a[0], a[1], a[2], a[3], a[4], a[5], a[6]]);
            }
        }
        for p in &self.lay.jaw {
            f.jaw.push(取(&o, p).map(|v| 数组(&v)).and_then(|a| a.first().copied()).unwrap_or(0.0));
        }
        Some(f)
    }

    fn act(&mut self, _c: &Cmd) -> bool {
        // 一条命令怎么打包,是这个插头的事;而**怎么打包不影响任何一个被量出来的数** ——
        // 交付率量的是"命令了多少 vs 实到多少",两端都来自这台机器人自己的读数。
        true
    }

    fn identity(&mut self) -> Vec<(String, f64, f64)> {
        // 关节名与限位**只用来算身份指纹**,不用来算几何。一台真控制器两样都报。
        Vec::new()
    }
}

fn main() {
    let mut listen: Option<u16> = None;
    let mut out = String::from("bodycal.json");
    let mut a = std::env::args().skip(1);
    while let Some(f) = a.next() {
        match f.as_str() {
            "--listen" => listen = a.next().and_then(|v| v.parse().ok()),
            "--out" => out = a.next().unwrap_or(out),
            other => eprintln!("不认识的开关:{other}"),
        }
    }
    let port = match listen {
        Some(p) => p,
        None => {
            eprintln!("用法:bl-calibrate --listen <端口> [--out bodycal.json]");
            std::process::exit(2);
        }
    };
    println!("[装] 在 {port} 上等这台机器人连过来…");
    let _ = std::io::stdout().flush();
    let l = std::net::TcpListener::bind(("0.0.0.0", port)).expect("占不住端口");
    let s = l.incoming().next().expect("没等到").expect("连接坏了");
    let mut ws = tungstenite::accept(s).expect("握手失败");
    println!("[装] 接上了。先听一帧,认这台机器人报的东西长什么样。");

    // 认布局 —— 只看形状,不看名字。
    //
    // 🔴 握手要回。这一层**允许**知道自己的线缆(插头的全部职责就是它),而驱动树里不许有 ——
    // 一条不回应答的连接会让对方的同步 RPC 永远等下去,两侧都不报错。
    let mut lay = discover::Layout::default();
    let mut first: Option<Value> = None;
    for _ in 0..4000 {
        let Ok(m) = ws.read() else { break };
        let tungstenite::Message::Binary(b) = m else { continue };
        let Ok(v) = rmpv::decode::read_value(&mut &b[..]) else { continue };
        let kind = wire::get(&v, "message_type").and_then(|x| x.as_str().map(|s| s.to_string())).unwrap_or_default();
        let ack = match kind.as_str() {
            "hello" => "hello_ack",
            "prepare_case" => "prepare_case_ack",
            "reset" => "reset_result",
            "call" => "call_result",
            "infer" => "infer_result",
            "trial_end" => "trial_end_ack",
            "heartbeat" => "heartbeat_ack",
            "close" => break,
            _ => continue,
        };
        // 这一帧里带没带观测?带了就拿它认布局。
        if let Some(o) = wire::get(&v, "payload").and_then(|p| wire::get(p, "observation")).cloned() {
            let l2 = discover::认(&o);
            if l2.够吗().is_ok() && first.is_none() {
                lay = l2;
                first = Some(o);
            }
        }
        let r = wire::reply(&v, ack, Value::Map(vec![]));
        let mut buf = Vec::new();
        if rmpv::encode::write_value(&mut buf, &r).is_ok() {
            let _ = ws.send(tungstenite::Message::Binary(buf));
        }
        if first.is_some() {
            break;
        }
    }
    lay.说一遍();
    if let Err(why) = lay.够吗() {
        eprintln!("[装] 🔴 拒绝开跑:{why}");
        eprintln!("      一个认错了的布局会让整轮自标定照常跑完并产出【看起来正常】的数。");
        std::process::exit(3);
    }

    // 上电日程:问驱动还欠自己什么,做那几件事,交回去,再问一遍。
    let mut body = body_layer::Body::new();
    let mut plug = Plug { ws, lay, last: first };
    let mut 轮 = 0u32;
    loop {
        let now = 轮 as u64 + 1;
        let Some((q, need)) = body_layer::schedule::next(&body, now) else {
            println!("[装] 🟢 不欠了 —— 自标定走完。");
            break;
        };
        轮 += 1;
        if 轮 > 40 {
            println!("[装] 停:轮数用尽,还欠 {:?}({:?})", q, need);
            break;
        }
        println!("[量] 第 {轮} 轮:{} —— 因为 {:?}", q.as_str(), need);
        let s = selfcal::跑一相(&mut plug, q, 0, 60);
        let got = match q {
            Quantity::StepDelivery => probe::step_delivery(&s.steps, now),
            Quantity::Reach => probe::reach(&s.reach, now),
            Quantity::Friction => probe::friction(&s.tilt, now),
            _ => Err(probe::Declined::NotEnoughSamples),
        };
        match got {
            Ok(m) => {
                println!("      🟢 量到:{:?}", &m.value[..(m.dim as usize).min(4)]);
                let _ = body.submit(m);
            }
            // 🔴 一条**点名的拒绝就是输出**。悄悄省掉它会让这具身体看起来比它真实的样子欠得少。
            Err(d) => println!("      🔴 拒绝:{d:?} —— 这一格仍然欠着,而且现在有名字"),
        }
    }
    let _ = std::fs::write(&out, format!("{{\"note\":\"see bl_save for the binary form\"}}"));
    println!("[装] 写到 {out}");
}

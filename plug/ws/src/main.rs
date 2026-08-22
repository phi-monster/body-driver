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
mod task;
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
    /// 最近一次收到的观测。
    last: Option<Value>,
    /// 手里攥着的那条命令,等对方问"给我一个动作"时交出去。
    待发: Option<Value>,
    /// 对方刚发过 `reset`(= 新的一集开始了)。干活模式据此清掉上一集的计划与"试过"名单。
    复位过: bool,
}

fn 取(v: &Value, path: &[String]) -> Option<Value> {
    let mut cur = v.clone();
    for k in path {
        let m = cur.as_map()?.iter().find(|(kk, _)| kk.as_str().or_else(|| kk.as_slice().and_then(|b| core::str::from_utf8(b).ok())) == Some(k.as_str()))?.1.clone();
        cur = m;
    }
    Some(cur)
}

fn 数组(v: &Value) -> Vec<f64> {
    discover::浮点串(v).unwrap_or_default()
}

impl<S: std::io::Read + std::io::Write> Plug<S> {
    /// 抽这条连接,直到拿到一帧新的观测。**握手照回,要动作就把手里那条命令交出去。**
    ///
    /// 🔴 这个方法就是插头存在的理由:`act` 只是**记下要做什么**,真正把命令送出去的时机
    /// 由对方决定(它什么时候问"给我一个动作")。把这两件事混在一起,就会出现
    /// "命令发了但对方那一步已经过去了" —— 而交付率量的正是命令与实到的比,错一拍就全错。
    fn 抽到下一帧(&mut self) -> bool {
        for _ in 0..400 {
            let Ok(m) = self.ws.read() else { return false };
            let tungstenite::Message::Binary(b) = m else { continue };
            let Ok(v) = rmpv::decode::read_value(&mut &b[..]) else { continue };
            let kind = wire::get(&v, "message_type").and_then(|x| x.as_str().map(String::from)).unwrap_or_default();
            if kind == "reset" {
                self.复位过 = true;
            }
            let ack = match kind.as_str() {
                "hello" => "hello_ack",
                "prepare_case" => "prepare_case_ack",
                "reset" => "reset_result",
                "call" => "call_result",
                "infer" => "infer_result",
                "trial_end" => "trial_end_ack",
                "heartbeat" => "heartbeat_ack",
                _ => continue,
            };
            let p = wire::get(&v, "payload").cloned().unwrap_or(rmpv::Value::Nil);
            let fname = wire::get(&p, "func_name").and_then(|x| x.as_str().map(String::from)).unwrap_or_default();
            // 带观测的那一帧:收下,并且这就是"下一帧到了"。
            let 有观测 = wire::get(&p, "obs").or_else(|| wire::get(&p, "observation")).cloned();
            let mut 新 = false;
            if let Some(o) = 有观测 {
                self.last = Some(o);
                新 = true;
            }
            // 要动作的那一帧:把手里那条交出去。没有就交空的 —— 空也是一个诚实的回答。
            // 🔴 **应答的形状是对方定的,不是我方便定的。** 少一个字段,对方的同步 RPC
            // 就永远等不到,而表现是"它卡住了" —— 连接正常、场景建好、两侧不报错。
            // 实测:握手回包少了 server / server_instance_id,客户端就停在握手之后
            // **一直心跳**,2600 帧一个观测都没发过。
            let payload = if fname == "get_action" {
                rmpv::Value::Map(vec![(
                    rmpv::Value::String("result".into()),
                    rmpv::Value::Array(self.待发.take().into_iter().collect()),
                )])
            } else if ack == "hello_ack" {
                rmpv::Value::Map(vec![
                    (rmpv::Value::String("ok".into()), rmpv::Value::Boolean(true)),
                    (rmpv::Value::String("server".into()), rmpv::Value::String("xpolicylab_policy_server".into())),
                    (rmpv::Value::String("server_instance_id".into()), rmpv::Value::String("bl-calibrate".into())),
                ])
            } else {
                rmpv::Value::Map(vec![(rmpv::Value::String("ok".into()), rmpv::Value::Boolean(true))])
            };
            let r = wire::reply(&v, ack, payload);
            let mut buf = Vec::new();
            if rmpv::encode::write_value(&mut buf, &r).is_ok() {
                let _ = self.ws.send(tungstenite::Message::Binary(buf));
            }
            if 新 {
                return true;
            }
        }
        false
    }
}

impl<S: std::io::Read + std::io::Write> Robot for Plug<S> {
    fn sense(&mut self) -> Option<Frame> {
        // 🔴 分段计时:等对方发帧 vs 我这边解图。两者要做的事完全不同 ——
        // 等得久 = 仿真慢/往返慢;解得久 = 我自己慢。仓规 §6.2:插打印,不推理。
        let t0 = std::time::Instant::now();
        if !self.抽到下一帧() {
            return None;
        }
        let t_等 = t0.elapsed();
        let t1 = std::time::Instant::now();
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
        // 🔴 相机图以前**根本没填进来**,于是"晃钳口看什么跟着动"那一格永远没有素材,
        // 而它是四格的前置 ⇒ 一个没填的字段卡住了五分之一的标定。
        // 转成灰度是因为下游认块只看"哪些像素变了",颜色不参与。
        for p in &self.lay.cams {
            if let Some(v) = 取(&o, p) {
                if let Some((w, h)) = discover::是图(&v) {
                    if let Some(rmpv::Value::Binary(b)) = v.as_map().and_then(|m| {
                        m.iter()
                            .find(|(k, _)| {
                                k.as_str().or_else(|| k.as_slice().and_then(|x| core::str::from_utf8(x).ok()))
                                    == Some("data")
                            })
                            .map(|(_, x)| x.clone())
                    }) {
                        let g: Vec<u8> = b
                            .chunks_exact(3)
                            .map(|c| ((c[0] as u32 * 299 + c[1] as u32 * 587 + c[2] as u32 * 114) / 1000) as u8)
                            .collect();
                        if g.len() == w * h {
                            // 🔴 **落图开关。** 认块器答"没有候选 · 双响 0 · 地板 11" 时,
                            // 数字说的是"画面很干净、什么都没动",而**"看不清"和"根本没东西可看"
                            // 在这三个数上完全同形**。仓规:空间/视觉的毛病,第一件事是渲图去看。
                            // 灰度直接写成 PGM(P5)—— 不引依赖,任何看图工具都认。
                            if let Ok(d) = std::env::var("BL_DUMP") {
                                let _ = std::fs::create_dir_all(&d);
                                let mut buf = format!("P5\n{w} {h}\n255\n").into_bytes();
                                buf.extend_from_slice(&g);
                                let _ = std::fs::write(format!("{d}/cam{}.pgm", f.cams.len()), buf);
                            }
                            f.cams.push((w, h, g));
                        }
                    }
                }
            }
        }
        // 每 50 帧报一次分段耗时。
        unsafe {
            static mut N: u64 = 0;
            static mut 等: u128 = 0;
            static mut 解: u128 = 0;
            N += 1;
            等 += t_等.as_micros();
            解 += t1.elapsed().as_micros();
            if N % 50 == 0 {
                println!("      [计时] 近 50 帧:等帧 {:.1} ms/帧 · 解图 {:.1} ms/帧 · 相机 {} 台",
                    等 as f64 / 50_000.0, 解 as f64 / 50_000.0, f.cams.len());
                等 = 0; 解 = 0;
            }
        }
        Some(f)
    }

    fn act(&mut self, c: &Cmd) -> bool {
        // 只记下要做什么;真正送出去在对方问"给我一个动作"的那一拍。
        let (arm, at, quat, 每通道) = match c {
            Cmd::Ee { arm, at, quat, jaw } => (*arm, *at, *quat, vec![*jaw]),
            Cmd::Grip { arm, at, quat, per } => (*arm, *at, *quat, per.clone()),
            _ => return true,
        };
        // 🔴🔴 **另一条臂必须发它【当前】的位姿,不能发空。**
        //
        // 这条线缆按键名判断动作类型:出现关节键算一种、出现位姿键算另一种,**两种都出现整帧被拒**;
        // 而只发一条臂、另一条留空,发出去的是 `left_ee_pose: []` —— 一条**形状不完整**的动作。
        // 实测(2026-08-17)的后果:身体**一步都不动**,而两侧零报错。
        // 它同时让自标定的 `step_delivery` 报 `NoResponse`(这具身体没响应)和任务那边
        // 每一步 `实降 0.0 mm` —— **一个根因,两条线一起瘫,而病相各自看起来都像别的问题**。
        // ⇒ "另一条臂待在原地"要**显式写出来**:发它此刻的位姿。
        // 🔴 **有几条臂、各自叫什么,全从【认出来的布局】拿** —— 一条也不许拼。
        // 布局记的是路径(`["state","ee_pose"]`),最后一节就是这台机器人自己用的键名。
        // 单臂 Franka 上老代码的 `lay.ee.get(1 - arm)` 取的是不存在的第 2 条臂 ⇒ 空位姿
        // ⇒ 形状不完整的动作 ⇒ 身体一步不动而两侧零报错(2026-08-17 已付过一次学费)。
        let 位姿键: Vec<String> = self.lay.ee.iter().filter_map(|p| p.last().cloned()).collect();
        let 钳口键: Vec<String> = self.lay.jaw.iter().filter_map(|p| p.last().cloned()).collect();
        let n = 位姿键.len().min(钳口键.len());
        if n == 0 {
            return true;
        }
        let 我 = arm.min(n - 1);
        let mut 位姿: Vec<Vec<f64>> = Vec::with_capacity(n);
        let mut 钳口: Vec<f64> = Vec::with_capacity(n);
        for i in 0..n {
            位姿.push(
                self.last
                    .as_ref()
                    .and_then(|o| 取(o, &self.lay.ee[i]))
                    .map(|v| 数组(&v))
                    .unwrap_or_default(),
            );
            钳口.push(
                self.last
                    .as_ref()
                    .and_then(|o| 取(o, &self.lay.jaw[i]))
                    .map(|v| 数组(&v))
                    .and_then(|a| a.first().copied())
                    .unwrap_or(1.0),
            );
        }
        // 每通道开合:线上协议本来就是每指一列(Frame.jaw 是 Vec)。
        // per 短于通道数 ⇒ 末值广播(标量 Ee = 全手同步开合,对 power grasp 通用)。
        for (ci, g) in 钳口.iter_mut().enumerate() {
            if ci == 我 || 钳口键.len() == 1 {
                *g = 每通道.get(ci.min(每通道.len().saturating_sub(1))).copied().unwrap_or(*g);
            }
        }
        self.待发 = Some(wire::pose_action(&位姿键, &钳口键, 我, &at, &quat, &位姿, &钳口));
        true
    }

    fn identity(&mut self) -> Vec<(String, f64, f64)> {
        // 关节名与限位**只用来算身份指纹**,不用来算几何。一台真控制器两样都报。
        Vec::new()
    }
}

fn main() {
    let mut listen: Option<u16> = None;
    let mut 读回: Option<String> = None;
    let mut out = String::from("bodycal.json");
    let mut 眼 = String::from("127.0.0.1:8077");
    let mut a = std::env::args().skip(1);
    while let Some(f) = a.next() {
        match f.as_str() {
            "--listen" => listen = a.next().and_then(|v| v.parse().ok()),
            "--out" => out = a.next().unwrap_or(out),
            // 🔴 **读回上一次量到的东西 —— 这就是"机器越用越强"的那一半。**
            // 没有它,每次上电都从零重标(实测这台 25 分钟),而**已经量到的格子会被
            // 重新量一遍,新的那次不一定更好**。有了它:已知的先装进身体,日程只安排
            // 还欠的那几格,而 `submit` 的 `WorseThanStored` 保证新证据不如旧的就不覆盖。
            "--in" => 读回 = a.next(),
            // 眼(VLM 指物)在哪个端点。端点是接线配置,不是身体量。
            "--eye" => 眼 = a.next().unwrap_or(眼),
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
    let mut 听到 = 0u32;
    let mut 计数: std::collections::BTreeMap<String, u32> = Default::default();
    let mut 诊断 = 0u32;
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
        // 🔴 观测的键**有两个可能**,而只认一个的后果是静默的:图每帧照收、每帧都当场
        // 说"里面没有观测",于是一步都没跑过,而两侧日志全绿。这一条是本仓付过学费的。
        let 观测 = wire::get(&v, "payload")
            .and_then(|p| wire::get(p, "obs").or_else(|| wire::get(p, "observation")))
            .cloned();
        if let Some(o) = &观测 {
            let l2 = discover::认(o);
            if l2.够吗().is_ok() && first.is_none() {
                lay = l2;
                first = Some(o.clone());
            } else if 诊断 < 3 {
                // 🔴🔴 **诊断打印必须挂在【要诊断的那一帧】上,不能挂在"每 N 帧"上。**
                // 实测(2026-08-17):这一条本来写成 `听到 % 50 == 0`,而带观测的帧全落在
                // 奇数位 ⇒ 采样点一半落在**不带观测**的那一拍上,于是这一行**一次都没打过**,
                // 而我据此得出了"观测帧从来没到过"这个**完全错误**的结论。
                // ⚠️ 教训比这个 bug 值钱:**"插打印不要推理"里,打印本身也可能在骗人** ——
                //    一个采样式的打印,会把交替出现的两种帧看成只有一种。
                诊断 += 1;
                println!("[装] 拿到观测了,但认不出来 ⇒ {}", l2.够吗().unwrap_err());
                l2.说一遍();
                if let Some(m) = o.as_map() {
                    println!("[装] 观测顶层键:{:?}", m.iter().filter_map(|(k, _)| k.as_str()).collect::<Vec<_>>());
                    for (k, v) in m {
                        if let Some(sub) = v.as_map() {
                            println!("[装]   {:?} 里:{:?}", k.as_str(),
                                sub.iter().filter_map(|(kk, _)| kk.as_str()).collect::<Vec<_>>());
                        }
                    }
                }
            }
        } else if 听到 % 50 == 0 {
            // 🔴 再往下一层:**payload 自己的键是什么、这一次调用叫什么名字**。
            // "没有观测"有两种,修法相反:这一帧本来就不带(是取动作那一拍),
            // 还是带了而我找错了键。只有把键原样印出来才分得开。
            let pk = wire::get(&v, "payload")
                .and_then(|p| p.as_map().map(|m| m.iter().filter_map(|(k, _)| k.as_str()).collect::<Vec<_>>()));
            let fname = wire::get(&v, "payload")
                .and_then(|p| wire::get(p, "func_name"))
                .and_then(|x| x.as_str())
                .unwrap_or("(没有 func_name)");
            println!("[装] 这一帧没有观测 · func_name={fname} · payload 的键={pk:?}");
        }
        // 🔴 线上每一条都记类型 —— "没收到"和"收到了但回错了"只有这一行能分开,而它零成本。
        *计数.entry(kind.clone()).or_insert(0u32) += 1;
        // 🔴 **头几十帧逐条打,不采样。** 每 50 帧打一次是在采样,而采样会把
        // "交替出现的两种帧"看成"只有一种" —— 真实序列必须原样摆出来一次。
        if 听到 < 30 {
            let fk = wire::get(&v, "payload")
                .and_then(|p| p.as_map().map(|m| m.iter().filter_map(|(k, _)| k.as_str()).collect::<Vec<_>>()));
            println!("[线] #{听到} 收 {kind} · payload 键={fk:?}");
        }
        if 听到 % 50 == 0 {
            let mut v: Vec<_> = 计数.iter().collect();
            v.sort();
            println!("[装] 收到过的消息类型:{v:?}");
        }
        let pl = if ack == "hello_ack" {
            Value::Map(vec![
                (Value::String("ok".into()), Value::Boolean(true)),
                (Value::String("server".into()), Value::String("xpolicylab_policy_server".into())),
                (Value::String("server_instance_id".into()), Value::String("bl-calibrate".into())),
            ])
        } else {
            Value::Map(vec![(Value::String("ok".into()), Value::Boolean(true))])
        };
        let r = wire::reply(&v, ack, pl);
        let mut buf = Vec::new();
        if rmpv::encode::write_value(&mut buf, &r).is_ok() {
            let _ = ws.send(tungstenite::Message::Binary(buf));
        }
        if first.is_some() {
            break;
        }
        听到 += 1;
        if 听到 % 50 == 0 {
            println!("[装] 已听 {听到} 帧还没认出布局 —— 这几帧顶层的键是:{:?}",
                v.as_map().map(|m| m.iter().filter_map(|(k, _)| k.as_str()).collect::<Vec<_>>()));
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
    // 🔴 **先把上一次量到的装回身体,再问还欠什么。**
    // 没有这一步,每次上电都从零重标(实测这台 25 分钟),而下面那个日程**看不出
    // 哪些格其实已经有了**。装回去之后:日程只安排还欠的,已有的由 `submit` 的
    // `WorseThanStored` 守着 —— 新证据不如旧的就不覆盖。**这就是"越用越强"的那一半。**
    let mut 相机们载: Vec<(usize, point_gen::Eye, f64)> = Vec::new();
    if let Some(path) = &读回 {
        match std::fs::read_to_string(path).ok().and_then(|t| body_layer::store::Store::from_str(&t).ok()) {
            None => println!("[装] 读不回 {path} —— 从零开始标"),
            Some(st) => {
                let mut 装 = 0usize;
                for q in body_layer::measurement::Quantity::ALL {
                    // 🔴 **带寿命的量不从磁盘装回来。**
                    // 手眼那一格自己声明 60 秒有效期,理由是"一磕就不对了" —— 而两次上电
                    // 之间相机完全可能被碰过。把它从文件里装回来,等于替这台相机担保
                    // 一件没人检查过的事。⇒ 它每次上电重量,而**它的下游(认手 / 跨度 /
                    // 工具偏置 / 工具轴 / 自遮挡)也因此跟着重量** —— 那是对的,依赖动了。
                    // 🔴 **凡是(传递地)依赖它的,也一律不装回来。**
                    // 装回去的测量丢掉了自己的依赖边(`blank_for` 不带 `deps`),于是
                    // `DependencyMoved` 对它们**永远不会触发** —— 一个对着旧手眼量出来的
                    // 手点,会安安静静地和新量的手眼一起被用下去,而两者根本不是一套。
                    // 少装几格换"不许出现一份自己都不知道自己过期了的测量"。
                    fn 依赖手眼(q: body_layer::measurement::Quantity) -> bool {
                        use body_layer::measurement::Quantity::ImageJacobian;
                        q == ImageJacobian
                            || body_layer::schedule::prerequisites(q).iter().any(|d| 依赖手眼(*d))
                    }
                    if 依赖手眼(q) {
                        continue;
                    }
                    if let body_layer::store::Answer::Measured { value, uncertainty, valid_lo, valid_hi, selftest_passed, .. } = st.ask(q.as_str()) {
                        if value.is_empty() { continue; }
                        let mut m = body_layer::measurement::Measurement::blank_for(q, value.len(), 1);
                        for i in 0..value.len().min(body_layer::measurement::MAX_DIM) {
                            m.value[i] = value[i];
                            m.uncertainty[i] = uncertainty.get(i).copied().unwrap_or(0.0);
                            m.valid_lo[i] = valid_lo.get(i).copied().unwrap_or(value[i]);
                            m.valid_hi[i] = valid_hi.get(i).copied().unwrap_or(value[i]);
                        }
                        m.selftest_passed = selftest_passed;
                        if body.submit(m).is_ok() { 装 += 1; }
                    }
                }
                println!("[装] 从 {path} 装回 {装} 格 —— 日程只会安排还欠的那几格");
            }
        }
        // 相机一节(若有)也装回来 —— 干活模式的深度反投影靠它。
        if let Ok(t) = std::fs::read_to_string(path) {
            if let Ok(j) = body_layer::json::parse(&t) {
                if let Some(body_layer::json::Json::Arr(arr)) = j.get("cameras") {
                    for c in arr {
                        let g = |k: &str| c.get(k).and_then(|x| x.num());
                        // 数组取分量:`nums()` 一次全取(json.rs 没有按下标取的接口)。
                        let ga = |k: &str, i: usize| c.get(k).map(|x| x.nums()).and_then(|v| v.get(i).copied());
                        if let (Some(i), Some(fx), Some(fy), Some(cx), Some(cy), Some(mpu)) =
                            (g("cam"), g("fx"), g("fy"), g("cx"), g("cy"), g("m_per_unit"))
                        {
                            let at = [ga("at", 0).unwrap_or(0.0), ga("at", 1).unwrap_or(0.0), ga("at", 2).unwrap_or(0.0)];
                            let q = [ga("q", 0).unwrap_or(1.0), ga("q", 1).unwrap_or(0.0), ga("q", 2).unwrap_or(0.0), ga("q", 3).unwrap_or(0.0)];
                            相机们载.push((i as usize, point_gen::Eye { fx, fy, cx, cy, at, q }, mpu));
                        }
                    }
                    println!("[装] 相机装回 {} 台", 相机们载.len());
                }
            }
        }
    }
    let mut plug = Plug { ws, lay, last: first, 待发: None, 复位过: false };

    // ── 🔴 看一眼钳口:BL_JAWLOOK=<目录> ────────────────────────────────
    // 为什么要有这个:跨度那一格的读数是「开度 0.34 与 0.45 量出同一个间距」,
    // 而钳口回读**已证实只是命令的回声**(命令 0.78 ⇒ 回读 0.7799999999999999)⇒
    // **没有任何非视觉的办法**分开"钳口没动"和"我认错了两个瓣"。仓规 §3.6:
    // 空间/视觉的毛病,第一动作是渲图去看,不是接着推。
    // 手臂全程不动,只扫开度;每一档停稳后把每台相机各落一张 PGM。
    if let Ok(dir) = std::env::var("BL_JAWLOOK") {
        let _ = std::fs::create_dir_all(&dir);
        // 🔴 **必须在【存档里那个原位】上看,不是"连上来时手在哪"。**
        // `hand_pixel` 是在原位上量的;换个位姿去核它,核的是另一件事(已踩过一次:
        // 在 (-0.300,-0.352,0.922) 上渲图,而存档原位是 (0.120,-0.261,1.030))。
        let 目标 = body.get(Quantity::HomePose)
            .filter(|m| m.value.len() >= 3)
            .map(|m| [m.value[0], m.value[1], m.value[2]]);
        if let Some(p) = 目标 {
            println!("[看钳口] 先回存档里的原位 ({:.3},{:.3},{:.3})", p[0], p[1], p[2]);
            let mut 上次 = f64::INFINITY; let mut 不再靠近 = 0u32;
            // 🔴 姿态回声身体自己此刻报的 —— 原来这里写死了 ARX 的"腕朝下"四元数,
            // 而那是一句关于法兰坐标系的断言,换一具身体(Franka)当场是错的。
            let mut q回 = plug.sense().and_then(|f| f.ee.first().map(|e| [e[3], e[4], e[5], e[6]])).unwrap_or([1.0, 0.0, 0.0, 0.0]);
            for _ in 0..200 {
                plug.act(&Cmd::Ee { arm: 0, at: p, quat: q回, jaw: 1.0 });
                let Some(f) = plug.sense() else { continue };
                let Some(e) = f.ee.get(0) else { continue };
                q回 = [e[3], e[4], e[5], e[6]];
                let d = ((e[0]-p[0]).powi(2)+(e[1]-p[1]).powi(2)+(e[2]-p[2]).powi(2)).sqrt();
                if d >= 上次 { 不再靠近 += 1; if 不再靠近 >= 3 { break } } else { 不再靠近 = 0 }
                上次 = d;
            }
            println!("[看钳口] 到位,离目标还差 {:.4} m", 上次);
        } else {
            println!("[看钳口] 存档里没有原位,就地看");
        }
        let home = match plug.sense() {
            Some(f) if !f.ee.is_empty() => [f.ee[0][0], f.ee[0][1], f.ee[0][2]],
            _ => { println!("[看钳口] 连上来就读不到末端位姿,放弃"); return; }
        };
        if let Some(m) = body.get(Quantity::HandPixel).filter(|m| m.value.len() >= 2) {
            println!("[看钳口] 存档里的 hand_pixel = ({:.4},{:.4}) —— 图落下来对着看", m.value[0], m.value[1]);
        }
        println!("[看钳口] 手不动,停在 ({:.3},{:.3},{:.3});开度从 0 扫到 1", home[0], home[1], home[2]);
        for lv in 0..=4u32 {
            let g = lv as f64 / 4.0;
            let mut 末: Option<Frame> = None;
            // 每档 30 拍:交付率 0.888 ⇒ 30 拍后残余 < 1e-30,足够停稳。
            for _ in 0..30 {
            // 🔴 姿态回声此刻身体自己报的(原来写死 ARX 腕朝下 —— 换机体就是错的)。
            let q回2 = plug.sense().and_then(|f| f.ee.first().map(|e| [e[3], e[4], e[5], e[6]])).unwrap_or([1.0, 0.0, 0.0, 0.0]);
            plug.act(&Cmd::Ee { arm: 0, at: home, quat: q回2, jaw: 1.0 });
                if let Some(f) = plug.sense() { 末 = Some(f); }
            }
            let Some(f) = 末 else { continue };
            println!("[看钳口] 开度命令 {:.2} ⇒ 回读 {:?} · 相机 {} 台", g, f.jaw, f.cams.len());
            for (i, (w, h, px)) in f.cams.iter().enumerate() {
                // 线缆里是灰度(解图那步已经转过);直接写 P5。
                let mut buf = format!("P5\n{w} {h}\n255\n").into_bytes();
                buf.extend_from_slice(px);
                let _ = std::fs::write(format!("{dir}/jaw{lv}_cam{i}.pgm"), buf);
            }
        }
        println!("[看钳口] 落图完毕 ⇒ {dir}");
        return;
    }

    // 🔴🔴 **一格拒绝之后必须往下走,不能原地重问。**
    // 实测(2026-08-17):第 1~5 轮全是同一格 `image_jacobian`,因为日程永远回答
    // "还欠这一格",而这一格这一轮量不出来 ⇒ **整轮自标定停在第一格上打转**,
    // 而它每一轮都在正常打印,看起来像在推进。
    // ⇒ 拒过的记下来,下一轮问日程时把它跳过;**拒绝本身是输出,不是重试的理由**。
    // 手眼那一格取出来的东西:水平面那 2×2、它的 1σ、以及它的纪元。
    // 🔴 纪元必须从**存下来的那一份**上取,不能拿"现在第几轮"顶 —— 否则下游每一轮
    //    都被判成"你依赖的那一格换版了,重来",于是永远量不完(实测发生过)。
    fn 平面尺(
        body: &body_layer::Body,
    ) -> Result<([f64; 4], [f64; 4], u64), probe::Declined> {
        let m = body
            .get(Quantity::ImageJacobian)
            .ok_or(probe::Declined::MissingDependency)?;
        if m.dim < 4 {
            return Err(probe::Declined::MissingDependency);
        }
        Ok((
            [m.value[0], m.value[1], m.value[2], m.value[3]],
            [m.uncertainty[0], m.uncertainty[1], m.uncertainty[2], m.uncertainty[3]],
            m.epoch,
        ))
    }
    /// 跨度那一相**自己在那一台相机上**量出来的尺:世界水平位移(米)⇒ 画面位移。
    ///
    /// 🔴 为什么不能用 `平面尺(&body)`:那把尺是 `image_jacobian` 那一相在**第 0 台**相机上量的。
    /// 间距若从第 1 台读、尺却是第 0 台的,相除**算得出一个完全正常的宽度**,而它是错的 ——
    /// 这条警告本来就写在选相机那段代码里。⇒ 同一台相机、同一批位形,自己量。
    ///
    /// 做法:四档里 1→2 是命令出去的 +x 一档,1→3 是 +y 一档(见 `selfcal::档偏`)。
    /// 每档取这台相机上所有样本的中位数(不是均值 —— 认错一次手就能把均值拽走)。
    /// 返回 `None` 的两种情形都当"这台相机换不出米":某一档没样本,或者画面响应小到
    /// 与不动分不开(**腕上的相机就长这样:手臂平移时手在它画面里根本不挪**)。
    /// 🔴🔴 **整台相机自己解出来 —— 不是一把只在某个深度成立的局部尺。**
    ///
    /// `本相尺` 解的是画面位移对世界位移的 2×2 线性近似。它有两条硬伤:
    /// ① **只在量它的那个深度成立**(近大远小),而钳口跟被拿来量尺的手臂位移**不在同一深度**;
    /// ② 这具身体的可达集把方向绑成近似一维时它**退化**,实测 |det|/σ 只有 0.5–1.0(判据要 ≥2)。
    ///
    /// `point_gen::fit_full` 解的是**整台相机**(焦距、主点、相机在世界哪、朝哪),
    /// 输入只有"手挪到哪儿(本体感受免费给)+ 它在画面的哪个像素"。仓里已经写好并测到 <1e-6。
    /// 有了它,**任何一个三维点在任何深度**都能换算,深度依赖从根上消失。
    ///
    /// 返回 `(眼, 手那个深度上一个归一化画面单位等于多少米)`。
    /// 🔴 单位:`u,v` 存的是**归一化**画面坐标,所以解出来的 `fx` 也是归一化单位 ⇒
    ///    米/归一化单位 = 深度 / fx。跨度那边读的间距也是归一化单位,两者同单位。
    fn 全相机(
        shift: &[(usize, f64, f64, [f64; 7])],
        cam: usize,
    ) -> Result<(point_gen::Eye, f64, [f64; 3]), point_gen::WhyNot> {
        // 🔴 联合解「观测点相对法兰的偏置」:认块认到的是指尖,不是法兰原点,
        // 且偏置随腕转 ⇒ 只喂 xyz 的 fit_full 在 Franka 上解出过一台假相机
        //(位置差 0.4 m · fx 差三个量级 · det(R)=−1),而所有闸都只能事后喊 Behind。
        let mut seen: Vec<([f64; 7], point_gen::Px)> = Vec::new();
        let mut c = [0.0f64; 3];
        // 🔴 按位置聚合、组内取像素中位附近的 3 条(2026-08-20,V3 两连判定案):
        // 剔后残差中位 0.0113 而 RMS 0.109 —— 肥尾是【位置级坏点】(个别格点整格
        // 认错,组内一致,单样本剔除拿它没辙)。中位滤掉组内认错的少数派;
        // 每组只留 3 条防某一格样本数碾压其他格。分组键 = 位置量化 0.1mm。
        {
            use std::collections::BTreeMap;
            let mut 组: BTreeMap<[i64; 3], Vec<([f64; 7], point_gen::Px)>> = BTreeMap::new();
            for &(i, u, v, p) in shift {
                if i != cam { continue }
                let k = [(p[0] * 1e4) as i64, (p[1] * 1e4) as i64, (p[2] * 1e4) as i64];
                组.entry(k).or_default().push((p, [u, v]));
            }
            for (_, mut v) in 组 {
                let n = v.len();
                let mut us: Vec<f64> = v.iter().map(|(_, q)| q[0]).collect();
                let mut vs: Vec<f64> = v.iter().map(|(_, q)| q[1]).collect();
                us.sort_by(|a, b| a.partial_cmp(b).unwrap());
                vs.sort_by(|a, b| a.partial_cmp(b).unwrap());
                let (mu, mv) = (us[n / 2], vs[n / 2]);
                v.sort_by(|(_, a), (_, b)| {
                    let da = (a[0] - mu).powi(2) + (a[1] - mv).powi(2);
                    let db = (b[0] - mu).powi(2) + (b[1] - mv).powi(2);
                    da.partial_cmp(&db).unwrap()
                });
                for (p, q) in v.into_iter().take(3) {
                    seen.push((p, q));
                    c[0] += p[0]; c[1] += p[1]; c[2] += p[2];
                }
            }
        }
        if seen.is_empty() { return Err(point_gen::WhyNot::TooFewSamples(0)) }
        // 🔴 **拒绝要可读**:`Coplanar` 有两种来路 —— 真的走成了一张平面,
        // 还是**根本只去过三个点**(三点必然共面)。少了这个数就分不开,而两者要改的完全不同。
        {
            let mut 异: Vec<[i64; 3]> = seen.iter()
                .map(|(p, _)| [(p[0] * 1e5) as i64, (p[1] * 1e5) as i64, (p[2] * 1e5) as i64]).collect();
            异.sort_unstable(); 异.dedup();
            println!("      [全相机] 第 {cam} 台:{} 个样本 · **{} 个不同的位置**(解一台完整相机至少要 6 个、且不共面)",
                seen.len(), 异.len());
        }
        let n = seen.len() as f64;
        let 手心 = [c[0] / n, c[1] / n, c[2] / n];
        // 🔴 轴约束版:自由 3 维的 d 与相机位置互相顶账(两发同残差、相机差 26 cm 的实锤),
        // "指尖长在工具轴上"这条物理约束把那一维退化砍死 —— 跨发复验:轴逐位同、偏置差 1%。
        let (eye, 轴, t, 留出) = point_gen::fit_full_axis_offset(&seen)?;
        let mut d偏 = [0.0f64; 3];
        d偏[轴] = t;
        println!("      [全相机] 第 {cam} 台:偏置沿第 {轴} 列 · t={t:.4} m · 留出中位 {留出:.4} 画幅");
        // 相机系约定 +z 朝前(见 `Eye::q` 的文档)⇒ 世界系里的前向 = q 把 [0,0,1] 转过去。
        let (w, x, y, z) = (eye.q[0], eye.q[1], eye.q[2], eye.q[3]);
        let 前 = [
            2.0 * (x * z + w * y),
            2.0 * (y * z - w * x),
            1.0 - 2.0 * (x * x + y * y),
        ];
        let d = (手心[0] - eye.at[0]) * 前[0] + (手心[1] - eye.at[1]) * 前[1] + (手心[2] - eye.at[2]) * 前[2];
        if !(d.is_finite() && d > 0.0) { return Err(point_gen::WhyNot::Behind) }
        if !(eye.fx.is_finite() && eye.fx.abs() > 1e-9) { return Err(point_gen::WhyNot::BadFit(eye.fx)) }
        // 🔴🔴 **解出来了 ≠ 解对了 —— 必须验它是不是物理上说得通的相机。**
        //
        // 实测代价(2026-08-18):这层包装原来只查"深度为正 + 焦距非零",于是一个退化解
        // 报出 **1 个归一化画面单位 = 6,541,472 米**,而且**覆盖了原本能用的 2×2 尺**
        // ⇒ 跨度那一格的成功率从 2/8 掉到 ~0/16。而跨度是**唯一挡住抓取的那一格**
        // (`link` 原话:`拒绝开跑:gripper_span 从没量过`)。
        //
        // 两道闸,都**不带任何身体知识**:
        // ① **主点必须落在画面里** —— 任何相机都成立(归一化坐标 ⇒ [0,1])。
        // ② **模型至少要比"永远猜平均像素"预测得好** —— 连这都做不到的解是垃圾。
        //    这一条**一个常数都不需要**:基准是数据自己的均值。
        if !(eye.cx > 0.0 && eye.cx < 1.0 && eye.cy > 0.0 && eye.cy < 1.0) {
            println!("      [全相机] 第 {cam} 台:主点 ({:.3},{:.3}) **在画面外** ⇒ 这不是一台相机,弃用",
                eye.cx, eye.cy);
            return Err(point_gen::WhyNot::BadFit(eye.cx));
        }
        let (mut 模, mut 均, mut n2) = (0.0f64, 0.0f64, 0.0f64);
        let (mu, mv) = (
            seen.iter().map(|(_, q)| q[0]).sum::<f64>() / seen.len() as f64,
            seen.iter().map(|(_, q)| q[1]).sum::<f64>() / seen.len() as f64,
        );
        for (p, q) in &seen {
            // 投影的是**观测点**(法兰 + R·d,和拟合时同一个物理点),不是法兰原点。
            let 转 = |qt: [f64; 4], v: [f64; 3]| -> [f64; 3] {
                let (w, x, y, z) = (qt[0], qt[1], qt[2], qt[3]);
                [
                    (1.0 - 2.0 * (y * y + z * z)) * v[0] + 2.0 * (x * y - z * w) * v[1] + 2.0 * (x * z + y * w) * v[2],
                    2.0 * (x * y + z * w) * v[0] + (1.0 - 2.0 * (x * x + z * z)) * v[1] + 2.0 * (y * z - x * w) * v[2],
                    2.0 * (x * z - y * w) * v[0] + 2.0 * (y * z + x * w) * v[1] + (1.0 - 2.0 * (x * x + y * y)) * v[2],
                ]
            };
            let w = 转([p[3], p[4], p[5], p[6]], d偏);
            let 点 = point_gen::P3 { x: p[0] + w[0], y: p[1] + w[1], z: p[2] + w[2] };
            if let Some(px) = eye.project(点) {
                模 += (px[0] - q[0]).powi(2) + (px[1] - q[1]).powi(2);
                均 += (mu - q[0]).powi(2) + (mv - q[1]).powi(2);
                n2 += 1.0;
            }
        }
        if n2 < 1.0 || !(模 < 均) {
            println!("      [全相机] 第 {cam} 台:重投影误差 {:.4} **还不如直接猜平均像素** {:.4} ⇒ 弃用",
                (模 / n2.max(1.0)).sqrt(), (均 / n2.max(1.0)).sqrt());
            return Err(point_gen::WhyNot::BadFit(模));
        }
        Ok((eye, d / eye.fx.abs(), d偏))
    }

    fn 本相尺(shift: &[(usize, f64, [f64; 3], [f64; 3], f64, f64)], cam: usize) -> Option<([f64; 4], [f64; 4])> {
        // 🔴🔴 **一次最小二乘吃掉全部样本,腕角当各自的截距;σ 从【残差】来。**
        //
        // 上一版走的是"每档取中位数 → 差分 → 跨腕角求散布"这种**两级归约**,而实测(spanQ)
        // 只凑出 **2 份**副本:行列式确实改好了(−0.015 → **+0.044**),σ 却炸到 **0.537** ——
        // 拿 2 个数算标准差,自由度只有 1,那个 σ 本身就是噪声。合成测试里有 3 份、每份 5 个样本
        // 才给出 0.0075,真机凑不出来。
        // ⇒ 直接回归:`u = a·gx + b·gy + α_腕角`、`v = c·gx + d·gy + β_腕角`,
        //   `(gx,gy)` 是**实到**的水平位移(命令 6 cm 而每拍只交付 0.888,还可能够不到)。
        //   腕角只改变那个中点的**偏移**、不改变尺本身,所以它进截距;
        //   σ 从残差和 (XᵀX)⁻¹ 来,自由度是"样本数 − 未知数",几十而不是 1。
        let mut 角: Vec<u64> = shift.iter().filter(|r| r.0 == cam).map(|r| (r.1 * 100.0).round() as u64).collect();
        角.sort_unstable(); 角.dedup();
        if 角.is_empty() { return None; }
        let k = 3 + 角.len(); // gx, gy, gz, 每个腕角一个截距
        if k > 8 { return None; }
        let mut xs: Vec<Vec<f64>> = Vec::new();
        let mut yu: Vec<f64> = Vec::new();
        let mut yv: Vec<f64> = Vec::new();
        for &(i, θ, _, got, u, v) in shift {
            if i != cam { continue; }
            let t = 角.iter().position(|&a| a == (θ * 100.0).round() as u64)?;
            let mut row = vec![0.0; k];
            // 🔴 自变量是**实到的三根轴** —— 哪两根独立由数据自己说,不由我挑。
            // 实测三次撞墙:+x 走不动 · 12 cm 抬不到 · y 伸出去之后 z 抬不过 5 cm
            // ⇒ 可达集把方向耦合在一起,手工挑必然反复撞墙。
            row[0] = got[0];
            row[1] = got[1];
            row[2] = got[2];
            row[3 + t] = 1.0;
            xs.push(row); yu.push(u); yv.push(v);
        }
        let n = xs.len();
        if n < k + 3 { return None; }
        // 正规方程 XᵀX·β = Xᵀy,高斯消元;同时求 (XᵀX)⁻¹ 的对角线给 σ 用。
        let mut a = vec![vec![0.0f64; k]; k];
        for r in &xs { for p in 0..k { for q in 0..k { a[p][q] += r[p] * r[q]; } } }
        let mut aug = vec![vec![0.0f64; 2 * k + 2]; k];
        for p in 0..k {
            for q in 0..k { aug[p][q] = a[p][q]; }
            aug[p][k + p] = 1.0;                                   // 右边接单位阵 ⇒ 顺带求逆
            for (idx, r) in xs.iter().enumerate() {
                aug[p][2 * k] += r[p] * yu[idx];
                aug[p][2 * k + 1] += r[p] * yv[idx];
            }
        }
        for c in 0..k {
            let piv = (c..k).max_by(|&x, &y| aug[x][c].abs().partial_cmp(&aug[y][c].abs()).unwrap())?;
            if aug[piv][c].abs() < 1e-12 { return None; }
            aug.swap(c, piv);
            let d = aug[c][c];
            for q in 0..2 * k + 2 { aug[c][q] /= d; }
            for r in 0..k {
                if r == c { continue; }
                let f = aug[r][c];
                if f == 0.0 { continue; }
                for q in 0..2 * k + 2 { aug[r][q] -= f * aug[c][q]; }
            }
        }
        let βu: Vec<f64> = (0..k).map(|p| aug[p][2 * k]).collect();
        let βv: Vec<f64> = (0..k).map(|p| aug[p][2 * k + 1]).collect();
        let 残 = |β: &Vec<f64>, y: &Vec<f64>| -> f64 {
            let ss: f64 = xs.iter().zip(y).map(|(r, &t)| {
                let f: f64 = r.iter().zip(β).map(|(x, b)| x * b).sum();
                (t - f) * (t - f)
            }).sum();
            ss / (n - k) as f64
        };
        let (s2u, s2v) = (残(&βu, &yu), 残(&βv, &yv));
        // 🔴🔴 **三根轴里挑条件最好的那一对 —— 由 |det|/σ 挑,不由我挑。**
        // 可达集把方向耦合在一起(x 走不动 / y 伸出去之后 z 抬不动),手工指定必然反复撞墙;
        // 而"哪两根轴在这个位形上真的独立"正是数据能回答的问题。
        let inv: Vec<f64> = (0..3).map(|c| aug[c][k + c]).collect();
        let 轴名 = ["x", "y", "z"];
        let mut 最好: Option<(f64, [f64; 4], [f64; 4], usize, usize)> = None;
        for (a, b) in [(0usize, 1usize), (0, 2), (1, 2)] {
            if !(inv[a] > 0.0 && inv[b] > 0.0) { continue; }
            let j = [βu[a], βu[b], βv[a], βv[b]];
            let js = [(s2u * inv[a]).sqrt(), (s2u * inv[b]).sqrt(), (s2v * inv[a]).sqrt(), (s2v * inv[b]).sqrt()];
            if j.iter().chain(js.iter()).any(|x| !x.is_finite()) { continue; }
            let det = j[0] * j[3] - j[1] * j[2];
            let det_σ = (j[3] * js[0]).hypot(j[0] * js[3]).hypot((j[2] * js[1]).hypot(j[1] * js[2]));
            if !(det.is_finite() && det_σ.is_finite() && det_σ > 0.0) { continue; }
            let 比 = det.abs() / det_σ;
            println!("      [跨度] 第 {cam} 台 ({},{}) 这一对:行列式 {det:.5} vs 1σ {det_σ:.5} ⇒ |det|/σ = {比:.2}",
                轴名[a], 轴名[b]);
            if 最好.as_ref().map(|(t, ..)| 比 > *t).unwrap_or(true) { 最好 = Some((比, j, js, a, b)); }
        }
        let Some((比, j, js, a, b)) = 最好 else {
            println!("      [跨度] 第 {cam} 台:三对轴没有一对算得出条件({n} 样本 · 自由度 {})—— 不用它", n - k);
            return None;
        };
        if 比 <= 2.0 {
            println!("      [跨度] 第 {cam} 台最好的一对是 ({},{}),|det|/σ 只有 {比:.2} —— **这个位形上给不出两个独立方向**,不用它",
                轴名[a], 轴名[b]);
            return None;
        }
        println!("      [跨度] 第 {cam} 台的尺:用 ({},{}) · J=[{:.4},{:.4},{:.4},{:.4}] · |det|/σ = {比:.2}({n} 样本 · 自由度 {})",
            轴名[a], 轴名[b], j[0], j[1], j[2], j[3], n - k);
        Some((j, js))
    }

    // 把弧上的每个点从画面单位换成水平面里的米。换不动的点原样丢掉 —— 补一个
    // 猜出来的坐标,比少一个点糟得多。
    fn 弧换米(
        arc: &[(u32, f64, f64, f64)],
        jac: &[f64; 4],
        sig: &[f64; 4],
    ) -> Vec<(u32, f64, f64, f64)> {
        arc.iter()
            .filter_map(|&(c, a, u, v)| {
                probe::image_to_plane(jac, sig, (u, v)).ok().map(|((x, y), _)| (c, a, x, y))
            })
            .collect()
    }
    // 这个方向是第几条射线。方向由探针发出去时就定死,这里只是把它认回来。
    // 🔴🔴 **这里【不许】再抄一份方向表 —— 抄了就是第二次犯同一个错。**
    // 原来这儿有一份写死的六条方向,而探针那边改成了八个角 ⇒ 一条都认不出来 ⇒
    // 每条射线的样本归不了组 ⇒ `0 条射线撞到了墙`,而每一行日志单看都正常。
    // ⇒ 直接调探针那份(`selfcal::射线号`),两处共用同一张表。
    use selfcal::射线号;
    let mut 拒过: std::collections::BTreeSet<&'static str> = Default::default();
    let mut 成: Vec<&'static str> = Vec::new();
    let mut 跟踪 = body_layer::hand::HandTracker::new(body_layer::probe::default_hand_config());
    let mut 轮 = 0u32;
    let mut 重轮 = 0u32;
    // 每一相开跑时离原位差多远 —— 跟结果一起落盘,验收时连着读。
    let mut 残差表: Vec<(&'static str, f64)> = Vec::new();
    // 重量阶段:强过 = 已经重量过几格;强试/强收 = 尝试与被收下的次数(WorseThanStored 挡回的差值)。
    let mut 强过 = 0u32;
    let mut 强试 = 0u32;
    let mut 强收 = 0u32;
    // 下压那一相实际用的命令幅度,存进接触阈那一格 —— 交付比例只在这个幅度上可比。
    let mut 探幅 = 0.0f64;
    // 基座是 `reach` 那一格顺手解出来的。臂重要拿它算力臂 —— 存下来,别丢。
    let mut 基座: Option<([f64; 3], u64)> = None;
    // 跨度是从哪台相机读的 —— 它存的是画面单位,而画面单位只在同一台相机里可比。
    let mut 跨度相机 = 0usize;
    // 跨度相自量的那把 2×2 尺(det/σ 轮轮 7-20,J 对角主导且跨轮一致 0.81-0.87),
    // 给同轮靠后的工具两格用 —— image_jacobian 格自己的探测(35 样本)det/σ 只有
    // 0.5-1.6,轮轮被 image_to_plane 正确地拒,弧点因此恒 0。同一个量,谁量得好用谁。
    // ⚠️ 债:这是会话内传递,没走账本正门(deps 记的仍是 jac 格)—— 正名待明日。
    let mut 会话尺: Option<([f64; 4], [f64; 4])> = None;
    // 相机原料的跨相位累计池(2026-08-20,GRAB5 尸检):Samples 每相独立,
    // jac/hand_pixel 相灌进 s.seen_cam 的认手样本随相位结束被丢,全相机拟合
    // 只读跨度相自己的 382 条 —— 灌注打印在、池子数没变的真相。相机是跨相位
    // 的量,原料池也必须跨相位。
    let mut 相机池: Vec<(usize, f64, f64, [f64; 7])> = Vec::new();
    // 🔴 这一轮标定里解出来的整台相机(台号, 眼, 手那个深度上 1 归一化单位 = 几米)。
    // 干活模式(深度反投影)要用它;写盘时一起存进标定文件,--in 时装回来。
    let mut 相机们: Vec<(usize, point_gen::Eye, f64)> = 相机们载;
    // 🔴 相机联合解顺手量出的「观测点相对法兰的偏置 d」—— 它同时是工具的测量:
    // 指尖长在工具轴上 ⇒ d 的主导分量是哪一列,工具轴就是哪一列;|d| 就是法兰到指尖。
    let mut 工具d: Option<[f64; 3]> = None;
    // 把机器人交给我们时它所在的那个位姿。每个相位开跑前先回到这儿 —— 按定义它走得到,
    // 而上一个相位可能把手臂停在墙上(可达那一格的工作就是把它顶到走不动)。
    let mut 起点: Option<[f64; 3]> = None;
    loop {
        let now = 轮 as u64 + 1;
        let 下一格 = (1..=40u64).find_map(|k| {
            body_layer::schedule::next(&body, now + k * 0).and_then(|(q, n)| {
                if 拒过.contains(q.as_str()) { None } else { Some((q, n)) }
            })
        });
        // 日程只回答"最先欠的那一格";它被拒过就手动往后找一格没拒过的。
        let 下一格 = 下一格.or_else(|| {
            // 🔴🔴 **后备的挑法不许写死顺序 —— 顺序由依赖表算。**
            // 上一版这里是一张手打的清单,而它把 `ToolOffset` 排在 `ToolAxisColumn`
            // **前面**;偏偏偏置依赖工具轴 ⇒ 它每一轮都在前置还没量到时被安排,
            // **每一轮都拒 `MissingDependency`,而日程本身从来没错**。
            // 一张与依赖表矛盾的手写顺序,读起来完全正常,却让一格永远轮不到。
            // ⇒ 挑"前置都已经量到、且自己还欠着、且这一轮没被拒过"的第一格。
            body_layer::measurement::Quantity::ALL
                .into_iter()
                .find(|q| {
                    !拒过.contains(q.as_str())
                        && body.get(*q).is_none()
                        && body_layer::schedule::prerequisites(*q)
                            .iter()
                            .all(|d| body.get(*d).is_some())
                })
                .map(|q| (q, body_layer::schedule::Need::NeverMeasured))
        });
        // 🔴🔴 **欠的都排完了 ⇒ 不停机,去重量【最弱的那一格】。** 这就是"越用越强"。
        //
        // 在这一行之前,这条链**从不可能变强**:日程只排欠着的格,而全仓除了
        // `image_jacobian` 之外每一格 `valid_for_ns` 都是 0(永不过期)⇒ 一格量到就
        // 再也不会被问第二次 ⇒ `submit` 里那道 `WorseThanStored` 闸**一次都触发不了**
        // (没有第二次测量,没有东西可比)。机器是"量一次定终身"。
        //
        // 安全性不靠这里挑得准:重量出来的行**每一项都更差**会被 `submit` 挡回去。
        // 所以这里挑错的代价是一次白跑,不是把好的覆盖掉。
        // 开关走 `BL_IMPROVE=<轮数>`,不开就照旧排完即停(别的消费者的行为不变)。
        let 还能强 = std::env::var("BL_IMPROVE").ok().and_then(|v| v.parse::<u32>().ok()).unwrap_or(0);
        let 下一格 = 下一格.or_else(|| {
            if 强过 >= 还能强 { return None }
            body_layer::schedule::weakest(&body).map(|q| (q, body_layer::schedule::Need::Stale))
        });
        // 🔴🔴 **原位必须第一个量 —— 它是所有相位的回位锚点。**
        // 实测(短炮 s0,2026-08-18):日程把 `home_pose` 排在**第 10 轮(最后)**,
        // 于是前面九相全在用"连上来时手在哪"那个**这具身体自己走不回去**的点当锚
        // (残差两炮之间一模一样 0.1129 m = 一堵墙),每一相都带偏开跑。
        // 它没有任何前置,排第一没有代价。
        let 下一格 = match 下一格 {
            Some((q, n)) if q != Quantity::HomePose && body.get(Quantity::HomePose).is_none()
                && !拒过.contains(Quantity::HomePose.as_str())
                && std::env::var("BL_ONLY").map(|o| o.split(',').any(|k| k.trim() == "home_pose")).unwrap_or(true) => {
                println!("[序] 先量原位 —— 它是每一相的回位锚点,排在 {} 前面", q.as_str());
                let _ = n;
                Some((Quantity::HomePose, body_layer::schedule::Need::NeverMeasured))
            }
            other => other,
        };
        let Some((q, need)) = 下一格 else {
            // 🔴 一轮完但还有格被拒 ⇒ 清拒格再跑一轮(≤3,协议数)。单轮过闸率是主敌
            // (方差不是趋势)—— GRAB 判词"重复打靶而不是赌单轮"(owner 拍板级)。
            // 拒多半是这一轮采样的性质(N22:跨度只配 3 对 ⇒ 尺薄 ⇒ 工具换米饿死),
            // 重跑 = 重摇采样,不是放宽闸;3 轮都拒的才算真欠。
            // 🔴 重跑只为【开工必需且装不回】的格烧预算。跨度/工具三格可由 --in 装回
            // (耐久硬件几何),arm_weight/floor 不在开抓必需清单里 —— 为它们补摇三轮
            // = 把抓取预算烧在不挡路的格上(N27 实测:重跑三轮 ≈ 9700 拍,服务段被
            // 推迟到只剩 1-2 次尝试)。
            let 该重跑 = 拒过.iter().any(|n| matches!(*n, "reach" | "contact_threshold" | "home_pose" | "step_delivery"));
            if !拒过.is_empty() && 该重跑 && 重轮 < 3 {
                重轮 += 1;
                println!("[装] 一轮完仍欠 {:?} ⇒ 重跑一轮补拒格({}/3)", 拒过, 重轮);
                拒过.clear();
                continue;
            }
            if !拒过.is_empty() && !该重跑 {
                println!("[装] 欠的 {:?} 全部可装回或不挡开工 ⇒ 不补摇,直接进干活模式", 拒过);
            }
            println!("[装] 🟢 走完一轮:量到 {} 格,点名拒绝 {} 格。", 成.len(), 拒过.len());
            if 强试 > 0 {
                println!("[装] 🟢 越用越强:重量 {强试} 次 ⇒ **换掉 {强收} 格**,{} 次被 WorseThanStored 挡回(旧值更好,保留)",
                    强试 - 强收);
            }
            println!("[装]    量到:{:?}", 成);
            println!("[装]    还欠:{:?}", 拒过);
            break;
        };
        轮 += 1;
        // 重量阶段单独计数:它不该被"轮数用尽"那条急停掐掉,也要能自己数够就停。
        if !成.is_empty() && body.get(q).is_some() {
            强过 += 1;
            强试 += 1;
        }
        if 轮 > 40 + 还能强 + 40 * 重轮 {
            println!("[装] 停:轮数用尽,还欠 {:?}({:?})", q, need);
            break;
        }
        // 🔴 `BL_ONLY=a,b,c` —— 只跑这几格。**诊断炮不该把 15 格全走一遍**:
        // 一炮 25 分钟里有 20 分钟是陪跑,而答案只在一条链上。仓规:GPU 时间才是成本。
        if let Ok(only) = std::env::var("BL_ONLY") {
            if !only.split(',').any(|k| k.trim() == q.as_str()) {
                println!("[跳] {} —— BL_ONLY 没点它", q.as_str());
                拒过.insert(q.as_str());
                continue;
            }
        }
        println!("[量] 第 {轮} 轮:{} —— 因为 {:?}", q.as_str(), need);
        // 🔴 相位长度按**这一格要多少证据**定,不是一刀切。接触阈要两簇各自够多才判得出界,
        // 可达要把两边的墙都夹住;而晃钳口那一格一个循环就要六步。
        let 步 = match q {
            Quantity::ContactThreshold | Quantity::Floor | Quantity::Reach => 240,
            Quantity::ToolOffset | Quantity::ToolAxisColumn => 400,
            Quantity::SelfOcclusion => 300,
            // 🔴 跨度要跑得久一点:真配对的产出率实测约 **13%**(30 个循环出 4 个),
            //    而估计器要 ≥5 个。砍掉单瓣兜底之后剩下的全是无假设的样本,
            //    **该补的是循环数,不是判据** —— 这一格今晚每一次改判据都改错了。
            // 🔴 分腕角拟合之后,**每个角度**都要够估计器的 5 个点。60 个循环 ÷ 3 个角度
            //    × 30% 配对产出率 ≈ 6 个 —— 擦边。翻倍到 1200 步(120 循环)才有余量。
            // 只剩一个腕角之后,1200 步就够 ≈60 个配对(原来 2400 步分给三个角度各 24 个)。
            // 🔴 `BL_SPAN_LEN=<步数>` —— 跨度这一相的长度可调。
            // 实测(L0–L7,2026-08-18):**8 炮里只有 1 炮凑够了同组 ≥5 个配对**(那一炮 21 个),
            // 其余 7 炮一个组都不够 ⇒ 堵点是**产出率**。而抬升幅度不是杠杆
            // (6 cm 两炮 8/0 · 9 cm 一炮 21 · 12–18 cm 全部 ≤2)。
            // ⇒ 下一个要分开的两件事:**"采样不够"还是"手在不在一个好位置"的二值问题**。
            //   前者跑长了配对数该线性涨,后者不会 —— 这一个旋钮就能分开它们。
            Quantity::GripperSpan => std::env::var("BL_SPAN_LEN").ok()
                .and_then(|v| v.parse::<u32>().ok()).unwrap_or(1200),
            // 49 个位形 × 6 拍
            Quantity::ArmWeight => 300,
            Quantity::ImageJacobian | Quantity::HandPixel => 300,
            _ => 90,
        };
        // 🔴🔴 **交付率低的身体要【多给步数】,不是发更大的命令。**
        //
        // 一步命令只走掉 10% 的身体,要走到同样远就得多走 ~9 倍的步。实测(Franka,交付 0.104):
        // 下压探针整相 139 步里 **134 步花在自由空间**,压住的只剩 5 个 ⇒ 接触阈只能拒
        // `NotEnoughSamples`。而"把命令放大 1/交付"这条路**当天就被自己推翻了**:
        // 交付率随命令变大而变小(同相实测 0.027 m ⇒ 0.902,0.070 m ⇒ 0.227),
        // 放大命令等于人为制造"交付塌了",桌面那一格当场把半空判成桌面。
        // ⇒ 步数 × (量出来的交付率的倒数),封顶 10 倍,免得一具坏身体把一相拖成无限长。
        let 步 = match body.get(Quantity::StepDelivery).filter(|m| m.dim >= 1).map(|m| m.value[0]) {
            Some(d) if d.is_finite() && d > 0.0 => {
                // 封顶 10 倍是协议数:再低的交付率说明上游那格坏了,不是步数能补的。
        let 倍 = (1.0 / d).min(10.0);
                let n = ((步 as f64) * 倍) as u32;
                // 1.5 只决定打不打这行日志,无量纲。
        if 倍 > 1.5 {
                    println!("      [步数] 这具身体每步只交付 {:.3} ⇒ 本相步数 {步} × {:.1} = {n}", d, 倍);
                }
                n
            }
            _ => 步,
        };
        // 🔴 静置多少拍**由这具身体自己决定**:延迟 + 每拍收掉 `交付率` 的残余,
        //    要等到残余降到千分之一。写死 2 拍的后果实测过 —— 认块的五格全欠着。
        //    量不出来就不许跑认块那几格:那不是"少采几下",是**这一格的前置还没到**。
        let 静置 = match body_layer::derive::settle_periods(&body, 1e-3) {
            // 🔴 静置比整个相位还长 ⇒ 它不是一个能执行的静置,是上游那一格坏了的**征状**。
            //    实测:交付率被墙毒成 0.00066 ⇒ 静置算出 10499 拍 ⇒ 五格连着塌。
            //    这里报缺前置而不是照跑,是为了让**病灶**具名,而不是让下游各自报"采不够"。
            Ok(n) if n >= 步 => {
                println!("      🔴 拒绝:MissingDependency —— 推出来的静置 {n} 拍比整相 {步} 拍还长,说明交付率/延迟那两格坏了");
                拒过.insert(q.as_str());
                continue;
            }
            Ok(n) => n,
            Err(_) if selfcal::认块相(q) => {
                println!("      🔴 拒绝:MissingDependency —— 静置拍数要由延迟+交付率推出来,而它们还没量到");
                拒过.insert(q.as_str());
                continue;
            }
            Err(_) => 4,
        };
        if selfcal::认块相(q) {
            println!("      [协议] 静置 {静置} 拍(由这具身体的延迟+交付率推出),周期 {} 拍", 静置 + 6);
        }
        // 🔴🔴 **回位点要用【量到的原位】,不是"连上来时手臂在哪"。**
        //
        // "连上来时在哪"什么都不是 —— 上一相把手臂停在哪儿它就是哪儿,而可达那一格的
        // 工作就是**把手臂顶到走不动为止**。连着跑几轮之后手臂一路漂,漂到爪子完全出画面。
        // 实测(2026-08-17,calrunE):三台相机的双响读数 `[(0,0), (3,12080), (0,0)]` ——
        // **头相机什么都看不见**,而估计器吃的正是它 ⇒ 全线拒绝,而每一条拒绝
        // 单看都像"这一格自己的问题"。
        // ⇒ `home_pose` 就是为这件事存在的:它是**量出来的身体常数**、存在文件里、
        //   按定义回得去。它没量到时才退回"连上来时在哪"。
        // 🔴🔴 **锚点一旦量到 `home_pose` 就换成它 —— 不许锁死在"连上来时手在哪"上。**
        //
        // 上一版是 `if 起点.is_none()`:第一相拿连上来那个点当锚,之后**永不更新**。
        // 而那个点是**重置时摆出来的**,不是命令得到的 —— 这具身体自己走不回去。
        // 实测(短炮,2026-08-18):回位残差两炮之间**一模一样是 0.1129 m** ——
        // 一模一样说明它不是噪声,是**走到那儿就是走不动了**的一堵墙。
        // 而 `home_pose` 那一格量的正是"这具身体命令回原位时**实际**落在哪",按定义走得到。
        let 新锚 = body.get(Quantity::HomePose).map(|m| [m.value[0], m.value[1], m.value[2]]);
        if let Some(p) = 新锚 {
            if 起点 != 新锚 {
                println!("      [归位] 锚点换成**量到的原位** ({:.3},{:.3},{:.3})", p[0], p[1], p[2]);
            }
            起点 = 新锚;
        }
        if 起点.is_none() {
            起点 = body.get(Quantity::HomePose).map(|m| [m.value[0], m.value[1], m.value[2]]);
            if let Some(p) = 起点 {
                println!("      [归位] 用**量到的原位** ({:.3},{:.3},{:.3}) —— 每相开跑前先回这儿", p[0], p[1], p[2]);
            } else if let Some(f) = plug.sense() {
                起点 = f.ee.get(0).map(|p| [p[0], p[1], p[2]]);
                if let Some(p) = 起点 {
                    println!("      [归位] 原位还没量到,暂用连上来时的位置 ({:.3},{:.3},{:.3})", p[0], p[1], p[2]);
                }
            }
        }
        // 🔴 钳口那一格的手臂全程不动 ⇒ 它停在哪儿就一直在哪儿。把它摆到**画幅中间**再开采:
        //    位移由已经量到的两格算出来 —— 手现在在画面哪一点(`hand_pixel`),
        //    以及画面里一段距离等于世界里多少米(`image_jacobian` 的水平 2×2 求逆)。
        //    这是②算的那一半:量到的东西拿来**算**,不是再填一个"抬高多少"。
        // 🔴🔴 **跨度那一相不再挪手 —— 挪不动。**
        // 上一版按 `hand_pixel + image_jacobian` 算出"把手挪到画幅中间"的世界位移,
        // 实测算出 **Δ=(0.707,-0.011) m** —— 而这条臂的可达带只有 0.383–0.448 m 半径,
        // 一步 0.7 m 的命令实到几乎为零(实测 `命令 0.805 ⇒ 实到 0.014`)。
        // 手于是停在一个**谁也不知道在哪**的位置,而"只认手附近"那道门槛(半径 0.25)
        // 就把不是钳口的东西放了进来 —— 配出 `两瓣相距 0.50114`(**半个画幅**,
        // 一副钳口物理上不可能)。⇒ 就在原位上量,原位已经是量出来的、回得去的。
        let 落点 = if false && matches!(q, Quantity::GripperSpan) {
            match (平面尺(&body), body.get(Quantity::HandPixel), 起点) {
                (Ok((jac, sig, _)), Some(hp), Some(p0)) => {
                    match probe::image_to_plane(&jac, &sig, (0.5 - hp.value[0], 0.5 - hp.value[1])) {
                        Ok(((dx, dy), _)) => {
                            println!("      [摆位] 手现在在画面 ({:.3},{:.3}),把它挪到画幅中间 ⇒ world Δ=({:.3},{:.3}) m",
                                hp.value[0], hp.value[1], dx, dy);
                            Some([p0[0] + dx, p0[1] + dy, p0[2]])
                        }
                        Err(_) => 起点,
                    }
                }
                _ => 起点,
            }
        } else {
            起点
        };
        // 跨度那一相每档挪多远:用**量到的可达带宽度**的三分之一。
        //
        // 🔴🔴 **上一版量不到 `reach` 就 `unwrap_or(0.0)` —— 一档变成 0,四档全叠在原位。**
        // 于是"把手抬到空背景上再量"和"两个水平位移当尺"两件事**一次都没发生过**,
        // 而日程照常安排跨度跑,拒绝理由报成 `NoResponse`(下一步:多采几下),
        // 真正该做的是**去补前置**。静默退化成 0 比拒绝更贵:拒绝会把人送对方向。
        // ⇒ 量不到 `reach` 就退到探针自己的幅度(接触那一相一步 0.01 m,这里走两步),
        //   **永远不为零**,并且把用的是哪一条打出来 —— 日志里不许出现"看起来正常的 0"。
        // 🔴 `BL_SPAN_STEP=<米>` 直接指定一档多大 —— 只为**并排比不同基线长度**:
        // 尺是"命令出去一档,画面里挪几个像素",一档太短则那几个像素淹在噪声里。
        // 不给就照常从量到的可达带推。
        // 🔴🔴 **一档的大小由「画面位移要压过画面噪声」定,不由可达带定。**
        //
        // 上一版取 `可达带/3` = **2.2 cm**,而离线复核(span_samples.txt,2026-08-18)显示:
        // 四档的画面位置差 ~0.005–0.010,而每一档自己的散布就有 ~0.005–0.008 ——
        // **尺的基线和噪声同量级**。三种取法 (x,y)/(x,z)/(y,z) 的 |det|/σ = 0.50/0.38/0.09,
        // **全部降秩**,后面每一步都是噪声放大(1.65 米那次就是这么来的)。
        //
        // 可达带说的是"离基座的**半径**能变多少"(这具身体 6.5 cm),
        // **不是"手能横着挪多远"** —— 拿它去限制一个横向激励幅度,是把一个约束用错了地方。
        // ⇒ 默认 6 cm:手在 640 宽的画面里挪十几个像素,压过 4 个像素的散布。
        //   够不够得到不用猜 —— 尺是用**实到**位移算的(见 `cam_shift`),够不到会自己显出来。
        let (抬一档, 尺来源) = match std::env::var("BL_SPAN_STEP").ok().and_then(|v| v.parse::<f64>().ok()) {
            Some(v) if v > 0.0 => (v, "BL_SPAN_STEP 指定"),
            // 🔴 原来写死 0.06 m("默认 6 cm")—— 一句身体断言;而这段注释自己记着
            // "一档的大小不由可达带定" ⇒ 唯一不描述身体的取法是几何阶梯:
            // 1 mm × 2^6 = 64 mm(与原 6 cm 同量级)。阶梯是协议(从很小开始、指数地找),
            // 起点与倍率无量纲地跨机体成立;够不够得到由实到位移自己显出来。
            // 🔴 第 6 档(64 mm)在增益修正后把手指整个举出画面顶部(认手就在 v≈0.15;
            // 老身体命令 64 只交付 6 才碰巧留框内)⇒ 双响 9700→500、配对饿死。
            // 尺度早已由整台相机接管(fit_full_axis_offset),举升只剩"衬空背景"一个活,
            // **留在画面里**是它的硬约束 ⇒ 降到第 4 档。阶梯是协议,不描述任何身体。
            _ => (1e-3 * 2f64.powi(4), "几何阶梯第 4 档(16 mm)"),
        };
        if matches!(q, Quantity::GripperSpan) {
            println!("      [跨度分档] 一档 {抬一档:.4} m —— {尺来源}");
        }
        // 手在画面里的位置 —— 跨度那一相拿它来判"这一对是不是我的手"。
        let 手在 = body.get(Quantity::HandPixel)
            .filter(|m| m.dim >= 2)
            .map(|m| (m.value[0], m.value[1]));
        // 这只手在画面里有多大 —— 取 `hand_pixel` 自己的 1σ,量出来的。
        let 手大 = body.get(Quantity::HandPixel).filter(|m| m.dim >= 2).map(|m| m.uncertainty[0].max(m.uncertainty[1]));
        // 一米在画面里是多少 —— image_jacobian 各分量绝对值的最大(画幅/米)。
        // 近手闸用它补偿"跨度相把手从 hand_pixel 相位的位置挪走了多少"。
        let 像素每米 = body.get(Quantity::ImageJacobian)
            .filter(|m| m.dim >= 1)
            .map(|m| m.value.iter().take(m.dim).fold(0.0f64, |a, v| a.max(v.abs())));
        // 🔴 探针幅度按**量到的可达带**给。量不到就传 None,探针走几何阶梯(不带尺度假设)。
        let 可达 = body.get(Quantity::Reach).filter(|m| m.dim >= 2).map(|m| (m.value[0], m.value[1]));
        // 这具身体每步交付多少 —— 量到了就递进去,探针据此放大命令(见 `跑一相` 的注释)。
        let 交付 = body.get(Quantity::StepDelivery).filter(|m| m.dim >= 1).map(|m| m.value[0]);
        // 🔴 跨度相静置 x4(2026-08-19,SPANX2 尸检):档偏水平 x4 到 6.4cm 后,
        // 晃钳口窗内双响 8.5k-17.6k px(正常 1-2k)—— 手臂没停稳就开晃,横移档配对全灭。
        // settle_periods 是按小步交付率(0.65)推的等比模型,而交付率随幅度暴跌
        // (0.027m=>0.90 · 0.28m=>0.02),大步的真实收敛慢得多。档偏放大几倍,
        // 静置就放大几倍 —— 系数与 selfcal::档偏 的 4 同源,不另立数。
        let 静置相 = if matches!(q, Quantity::GripperSpan) { 静置 * 4 } else { 静置 };
        let mut s = selfcal::跑一相(&mut plug, q, 0, 步, 静置相, 落点, 抬一档, 手在, 手大, 像素每米, 可达, 交付);
        相机池.extend_from_slice(&s.seen_cam);
        // 🔴 七卡当一卡(2026-08-21,owner 死命令"只跑一炮"):同种子同布局的仿真是
        // 同一台机器人的克隆,N 份短采样合并 = 一炮的完整统计。BL_SEED_SAMPLES=
        // 逗号分隔的 dump 文件表,把外部样本预灌进本相样本池,估计器照常吃、照常判、
        // 照常入账 —— 全走正门,不造离线旁路。只在对应相位灌对应段。
        if let Ok(fs) = std::env::var("BL_SEED_SAMPLES") {
            let mut 灌 = (0usize, 0usize, 0usize, 0usize);
            for f in fs.split(',') {
                let Ok(t) = std::fs::read_to_string(f.trim()) else { continue };
                for line in t.lines() {
                    let w: Vec<&str> = line.split_whitespace().collect();
                    let g = |i: usize| w.get(i).and_then(|x| x.parse::<f64>().ok());
                    match (w.first().copied(), q) {
                        (Some("jaw"), Quantity::GripperSpan) => {
                            if let (Some(c), Some(a), Some(m), Some(du), Some(dv)) = (g(1), g(2), g(3), g(4), g(5)) {
                                s.jaw.push((c as usize, a, m, du, dv)); 灌.0 += 1;
                            }
                        }
                        (Some("shift"), Quantity::GripperSpan) => {
                            if w.len() >= 11 {
                                if let (Some(c), Some(a)) = (g(1), g(2)) {
                                    let cmd = [g(3).unwrap_or(0.0), g(4).unwrap_or(0.0), g(5).unwrap_or(0.0)];
                                    let got = [g(6).unwrap_or(0.0), g(7).unwrap_or(0.0), g(8).unwrap_or(0.0)];
                                    s.cam_shift.push((c as usize, a, cmd, got, g(9).unwrap_or(0.0), g(10).unwrap_or(0.0)));
                                    灌.1 += 1;
                                }
                            }
                        }
                        (Some("seen"), Quantity::GripperSpan) => {
                            if w.len() >= 11 {
                                if let (Some(c), Some(u), Some(v)) = (g(1), g(2), g(3)) {
                                    let mut p7 = [0.0f64; 7];
                                    for i in 0..7 { p7[i] = g(4 + i).unwrap_or(0.0); }
                                    相机池.push((c as usize, u, v, p7)); 灌.2 += 1;
                                }
                            }
                        }
                        (Some("arc"), Quantity::ToolAxisColumn) | (Some("arc"), Quantity::ToolOffset) => {
                            if let (Some(c), Some(a), Some(u), Some(v)) = (g(1), g(2), g(3), g(4)) {
                                s.arc.push((c as u32, a, u, v)); 灌.3 += 1;
                            }
                        }
                        _ => {}
                    }
                }
            }
            if 灌.0 + 灌.1 + 灌.2 + 灌.3 > 0 {
                println!("      [合炮] 预灌外部样本:jaw {} · shift {} · seen {} · arc {}", 灌.0, 灌.1, 灌.2, 灌.3);
            }
        }
        // 🔴🔴 **从错的位姿开跑,量出来的每个数都长得正常而全是别处的值。**
        // 实测(f0,2026-08-18):可达那一相把手臂顶到走不动,回位没真回来,于是画面雅可比
        // 那一相在**画幅左下角 (0.049,0.978)** 开测,探针幅度一路涨到 **命令 0.805 m ⇒ 实到
        // 0.014 m**(顶在限位上),而拒绝的名字是 `NotEnoughSamples` —— 送错方向。
        // ⇒ 残差摆出来;超过这一相自己要用的探针幅度就**点名拒绝**,不带着错位姿测。
        if 落点.is_some() {
            // 下限走几何阶梯第 4 档(16 mm)—— 阶梯是协议,不描述任何身体。
        let 容 = 抬一档.max(1e-3 * 2f64.powi(4));
            println!("      [归位] 开跑时离目标 {:.4} m(容差 {:.4} m = 这一相自己的探针幅度)", s.回位残差, 容);
            // 🔴 **这里【不拒绝】,只记账。**
            // 拒绝那一版实测(短炮 s0–s15)16 路全灭、0 格量到 —— 一个数都拿不到就
            // 什么也学不到。而残差本身是**这一相所有读数的可信度**:把它印出来 + 落进
            // 结果文件,之后就能直接查"跨炮散布是不是跟着残差走",而不是再猜一轮。
            // ⚠️ 这是一条**明知有偏还照量**的路,所以它必须留下痕迹 —— 印一行醒目的,
            //    并且这一相的数在验收时要连着残差一起读。
            if s.回位残差 > 容 {
                println!("      ⚠️ **带偏开跑**:差 {:.4} m > 容差 {:.4} m —— 这一相的读数要连着这个残差一起读", s.回位残差, 容);
            }
            残差表.push((q.as_str(), s.回位残差));
        }
        // 🔴 **每一格都接到它自己的估计器上。** 接不上的那几格,拒绝的理由要是
        // `MissingDependency`(缺前置)而不是 `NotEnoughSamples`(没采够)——
        // 两者要做的下一步完全不同:前者去补前置,后者去多采几下。
        let got = match q {
            Quantity::StepDelivery => probe::step_delivery(&s.steps, now),
            // 🔴 可达要的是**离基座的半径**,而基座不在观测里。先从"每条射线上走不动的
            //    那个点"反解出基座(它们落在同一个球面上),再把每个采样点换成半径。
            //    上一版直接把**步长**塞进去当半径 —— 一个和半径毫无关系的数,于是每轮
            //    都拒 `Inconsistent`,而那条拒绝读起来像是身体自相矛盾。
            Quantity::Reach => {
                // 每条射线上最后一个还走得动的点 = 那条射线的卡住点。
                let mut 卡住: Vec<([f64; 3], [f64; 3])> = Vec::new();
                for j in 0..12usize {
                    let 本条: Vec<_> = s.reach.iter().filter(|(_, d, _)| 射线号(*d) == j).collect();
                    // 🔴🔴 **"从第一步就走不动"的射线,是在【起点】撞的墙 —— 不许整条丢掉。**
                    // 上一版只在"找得到最后一个走得动的点"时才记 ⇒ 一条一步都没走动的射线
                    // `rposition` 返回 None ⇒ **整条被静默丢弃**,而它恰恰是信息最强的一条。
                    // 实测(2026-08-18):8 条射线只数出 **3 条撞墙**,而定一个球要 4 个点 —— 差一个。
                    if 本条.iter().all(|(_, _, ok)| !*ok) {
                        if let Some(第一) = 本条.first() {
                            卡住.push((第一.0, 第一.1));
                        }
                    } else if let Some(idx) = 本条.iter().rposition(|(_, _, ok)| *ok) {
                        // 后面还有走不动的,才算真的撞到墙;一路都走得动 = 这条射线没探到边。
                        if idx + 1 < 本条.len() {
                            卡住.push((本条[idx].0, 本条[idx].1));
                        }
                    }
                }
                // 🔴🔴 **卡住且两侧都不报错 ⇒ 插打印,不是推理。**
                // 我在这一格上连着猜了四轮(射线数 / 射线方向 / 阶梯 / 走不动算不算撞墙),
                // 每一轮都在改探针,而**没有一次去看它为什么解不出来**。
                // 解基座做的是"把撞墙点拟合到**一个球面**上",而拒绝来自消元奇异 = **那些点共面**。
                // ⇒ 把点本身和它们的**共面程度**打出来,让下一步由数据定。
                println!("      [可达] {} 条射线撞到了墙", 卡住.len());
                if 卡住.len() >= 4 {
                    // 三个主轴的伸展:最小的那个 ÷ 最大的那个,越接近 0 越共面。
                    let n = 卡住.len() as f64;
                    let c = [
                        卡住.iter().map(|(p, _)| p[0]).sum::<f64>() / n,
                        卡住.iter().map(|(p, _)| p[1]).sum::<f64>() / n,
                        卡住.iter().map(|(p, _)| p[2]).sum::<f64>() / n,
                    ];
                    let mut m = [[0.0f64; 3]; 3];
                    for (p, _) in &卡住 {
                        let d = [p[0] - c[0], p[1] - c[1], p[2] - c[2]];
                        for i in 0..3 { for j in 0..3 { m[i][j] += d[i] * d[j]; } }
                    }
                    let tr = m[0][0] + m[1][1] + m[2][2];
                    let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
                    println!("      [可达] 撞墙点的散布:迹 {:.5} · 行列式 {:.3e} ⇒ **{}**",
                        tr, det,
                        if det.abs() < 1e-12 { "共面(球心定不下来)" } else { "不共面" });
                    for (i, (p, d)) in 卡住.iter().enumerate().take(12) {
                        println!("      [可达]   第 {i} 个:撞在 ({:.3},{:.3},{:.3}) · 方向 ({:.2},{:.2},{:.2})",
                            p[0], p[1], p[2], d[0], d[1], d[2]);
                    }
                }
                // 🔴🔴 **半径从【这一相的起点】算,不从"基座"算 —— 球那个模型是错的。**
                //
                // 原来的路是:把撞墙点拟合到**一个球面**上求出基座,再拿离基座的距离当半径。
                // 那等于断言"这条胳膊的可达域是一个球壳"。实测(2026-08-19,Franka FR3):
                // 10 条射线撞墙、散布明确**不共面**(行列式 1.4e-3),而撞墙点离基座
                // **0.44–0.75 m,差 70%** —— 它们根本不在一个球面上,于是拟合报 `Inconsistent`,
                // 而**那条拒绝是对的:错的是模型**。7 轴臂在桌子上的可达边界里混着关节限位、
                // 自碰撞和桌面,没有理由是个球。
                //
                // 而下游真正要的从来不是球心:抓取那侧问的是「**这个点离我的手多远、够不够得着**」
                // (`grasp.rs` 里就是拿 `离手` 去比)。那个问题**不需要基座** ——
                // 从起点出发沿各方向走到走不动为止,走出来的距离就是答案,直接量得到。
                //
                // 🔴 基座仍然解一次,但只**打出来当诊断**,不再当闸:解不出来不影响这一格。
                if let Ok(b) = probe::base_from_stalls(&卡住, now) {
                    println!("      [可达] (只作诊断)球拟合的基座 ({:.3},{:.3},{:.3})、半径 {:.3} m(残差 {:.4})",
                        b.value[0], b.value[1], b.value[2], b.value[3], b.uncertainty[0]);
                    基座 = Some(([b.value[0], b.value[1], b.value[2]], now));
                }
                let h = s.home;
                let 半径: Vec<(f64, bool)> = s
                    .reach
                    .iter()
                    .map(|(p, _, ok)| {
                        let r = ((p[0] - h[0]).powi(2) + (p[1] - h[1]).powi(2) + (p[2] - h[2]).powi(2)).sqrt();
                        (r, *ok)
                    })
                    .collect();
                // 🔴 原始量必须能看见(可达连拒四轮之后,不再对着比值猜)。
                if let Ok(d) = std::env::var("BL_DUMP_REACH") {
                    let mut t = String::new();
                    for (r, ok) in &半径 {
                        t.push_str(&format!("{r:.5} {}\n", u8::from(*ok)));
                    }
                    let _ = std::fs::write(&d, t);
                    println!("      [可达] 原始 (半径,到没到) 落盘 ⇒ {d}({} 条)", 半径.len());
                }
                println!("      [可达] 半径从起点 ({:.3},{:.3},{:.3}) 算,{} 个样本", h[0], h[1], h[2], 半径.len());
                // 🔴🔴 逐拍标签喂带子估计器在这具身体上是【结构性】死路(四轮原始序列定案):
                // 去/回程把每个半径都成功路过无数遍,82% 到达率平铺 1–57 cm,墙形被拍汤稀释
                // —— 平坦曲线闸拒得对。撞墙数也不可赌(标签修干净后仅 3 条硬墙)。
                // ⇒ **每条射线取自己的极限半径**:撞墙的 = 准确停点;没撞的 = 这方向至少到过的
                //   最远点(截尾,只低估不虚报)。中位 ± MAD 出格,方向差异如实进误差棒。
                //   射线 ≥8 条才给(覆盖协议,无量纲);不足走老路点名拒。
                let 每条 = {
                    let mut m: std::collections::BTreeMap<[i64; 3], f64> = std::collections::BTreeMap::new();
                    for (p, d, _) in &s.reach {
                        let k3 = [(d[0] * 100.0).round() as i64, (d[1] * 100.0).round() as i64, (d[2] * 100.0).round() as i64];
                        let r = ((p[0] - h[0]).powi(2) + (p[1] - h[1]).powi(2) + (p[2] - h[2]).powi(2)).sqrt();
                        let e = m.entry(k3).or_insert(0.0);
                        if r > *e {
                            *e = r;
                        }
                    }
                    m
                };
                if 每条.len() >= 8 {
                    let mut 极: Vec<f64> = 每条.values().copied().collect();
                    极.sort_by(|a, b| a.partial_cmp(b).unwrap());
                    let 中 = 极[极.len() / 2];
                    let mut 偏: Vec<f64> = 极.iter().map(|w| (w - 中).abs()).collect();
                    偏.sort_by(|a, b| a.partial_cmp(b).unwrap());
                    let mad = 偏[偏.len() / 2];
                    let mut m = body_layer::measurement::Measurement::blank_for(Quantity::Reach, 2, now);
                    m.value[0] = 0.0;
                    m.value[1] = 中;
                    m.uncertainty[0] = 半径.iter().map(|(r, _)| *r).fold(f64::MAX, f64::min).max(0.0);
                    m.uncertainty[1] = mad.max(1e-4);
                    m.valid_lo[0] = 0.0;
                    m.valid_hi[0] = *极.last().unwrap();
                    m.valid_lo[1] = 0.0;
                    m.valid_hi[1] = *极.last().unwrap();
                    m.valid_for_ns = 0;
                    m.selftest_passed = true;
                    println!("      [可达] {} 条射线各自的极限半径:中位 {:.3} m ± MAD {:.3}(全距 {:.3}–{:.3};其中撞到硬墙 {} 条)",
                        极.len(), 中, mad, 极[0], 极.last().unwrap(), 卡住.len());
                    Ok(m)
                } else {
                    probe::reach(&半径, now)
                }
            }
            // 🔴 摩擦:采样端已经停了(见 selfcal 里那段)。这里拒绝的**名字**要送对方向 ——
            // `NotEnoughSamples` 会把人送去"多采几下",而真正欠的是一次被验证过的抓取。
            Quantity::Friction => {
                println!("      🔴 拒绝:MissingDependency —— 摩擦要的是【一次被验证过的抓取】+【能分开「咬住」与「合在空气上」的信号】");
                println!("         钳口回读已实测是命令的回声(命令 0.78 ⇒ 回读 0.7799999999999999)⇒ 拿它判「咬住」是在读自己");
                println!("         另:§1.6 把\"会不会滑\"归在**世界**不在身体 —— 它随物体变,不是这具身体的常数");
                Err(probe::Declined::MissingDependency)
            }
            // 🔴 延迟从**原始逐拍位移**上判,不由这一层先挑好"第一拍动的是哪一拍"。
            //    上一版喂的是 `k % 12` 的第一个非零下标,没有静止参照 ⇒ 量到的是
            //    这一相最早拿到两帧连续观测的那个下标(实测报 6,而同一相一拍交付 89%)。
            Quantity::Latency => {
                println!("      [延迟] 静止段 {} 拍 · 命令后 {} 拍", s.rest.len(), s.latency.len());
                // 🔴 原始量必须能看见(仓规:先看原始量,再看导出量)——
                // `Inconsistent` 只有一个来源(静止段后半比前半吵),数不摆出来就只能猜。
                let 摆 = |v: &[f64]| v.iter().map(|x| format!("{:.5}", x)).collect::<Vec<_>>().join(" ");
                println!("      [延迟] 静止逐拍: {}", 摆(&s.rest));
                println!("      [延迟] 命令后逐拍: {}", s.latency.iter().map(|(o, d)| format!("{o}:{d:.5}")).collect::<Vec<_>>().join(" "));
                probe::latency_from_beats(&s.rest, &s.latency, now)
            }
            Quantity::Backlash => probe::backlash(&s.reversal, now),
            // 关节角与"保持不动要多少力矩"配成对。🔴 这具身体的观测里**没有力矩通道**,
            // 所以这一格只能拿"这一步挪了多少"当代理量 —— 而那不是力矩。
            // ⇒ 它会被估计器按自己的判据拒掉,而那正是对的:**代理量不是测量**。
            // 🔴 **这具身体的观测里没有力矩通道。**
            // 自重量的是"什么都不碰时保持不动要多少力矩",而我手上只有"这一步挪了多少" ——
            // 那是一个**代理量,不是测量**。拿它去算,会得到一个看起来正常的数
            // (实测 0.029),然后被当成量出来的东西一路用下去。
            // ⇒ 老实报缺通道。**一条点名的拒绝,比一个漂亮的假数值钱。**
            // 🔴🔴 **臂重不要力矩通道。** 撤回记录见 `probe::arm_weight_by_asymmetry` 的文件头:
            // 我连着三次判"这台没有力矩通道 ⇒ 量不了、天花板 14/15",而那是把
            // `arm_weight` 的入参当成了那个量的规格。这一层里凡是跟力有关的量全从
            // 交付比例里读 —— 臂重同理:往上顶着重力走、往下顺着重力走,两次交付之差
            // 就是那一处的重力负载,随力臂线性增长。
            Quantity::ArmWeight => match 基座 {
                None => Err(probe::Declined::MissingDependency),
                Some((b, _)) => {
                    // 🔴🔴 **依赖里要存的是【可达那一格的版本号】,不是"解基座那一刻的时间戳"。**
                    //
                    // 原来传的是 `基座` 里带的 `now`(解基座的时刻)。而调度器判"依赖动没动"
                    // 是拿它和 `body.get(Reach).epoch` **逐位比** —— 两个数来源不同,永远不相等
                    // ⇒ 臂重每一轮都被判成 `DependencyMoved`,量完立刻又要重量。
                    // 实测(2026-08-19,Franka):第 5 轮量到臂重,第 6..17 轮**全在重量同一格**,
                    // 六路 GPU 一起空转,而每一行日志都正常。
                    // 这个死循环之前一直没露面,是因为**可达从没成功过**,臂重根本轮不到。
                    let ep = body.get(Quantity::Reach).map(|m| m.epoch).unwrap_or(0);
                    // 把同一处的"往上"和"往下"配成对:相邻两条记录属于同一个位形。
                    let mut 对: Vec<(f64, f64, f64)> = Vec::new();
                    let mut 上: Option<(f64, f64)> = None; // (力臂, 交付比例)
                    for &(p, 向上, cmd, got) in &s.hold {
                        if cmd <= 0.0 { continue; }
                        let 力臂 = ((p[0] - b[0]).powi(2) + (p[1] - b[1]).powi(2)).sqrt();
                        let r = got / cmd;
                        if 向上 {
                            上 = Some((力臂, r));
                        } else if let Some((l, ru)) = 上.take() {
                            // 同一个位形的一对:力臂取两者平均(那两步只差 2 cm 的竖直位移)。
                            对.push((0.5 * (l + 力臂), ru, r));
                        }
                    }
                    println!("      [臂重] {} 对上下步,力臂 {:.3}–{:.3} m", 对.len(),
                        对.iter().map(|x| x.0).fold(f64::INFINITY, f64::min),
                        对.iter().map(|x| x.0).fold(0.0f64, f64::max));
                    probe::arm_weight_by_asymmetry(&对, now, ep)
                }
            },
            // 🔴 接触阈要**两簇**:自由空间里的交付比例、压住时的交付比例。
            // 一路往下压,自然会先给出前者、碰到之后给出后者 —— 界由估计器去找,不由我切。
            Quantity::ContactThreshold => match body.get(Quantity::StepDelivery) {
                // 🔴 分两簇要有个参照,而**那个参照就是自由空间的交付率**,不是一个填的 0.5。
                //    这里原来写死 `> 0.5` —— 换一具交付率只有 0.4 的身体(慢、软、增益低),
                //    它**自由移动时的每一步都会被归进"压住"那一簇**,于是两簇变成一簇,
                //    估计器答"你这两组读起来一样",而身体一切正常。
                //    以交付率的一半为界:自由那簇围着 0.883,压住那簇趴在 0.01–0.26,
                //    中间空得很宽,界落在哪儿都不敏感 —— 敏感的是它**跟着身体走**。
                None => Err(probe::Declined::MissingDependency),
                Some(sd) => {
                    探幅 = s.press.iter().map(|(c, _, _)| *c).fold(0.0f64, f64::max);
                    let 界 = sd.value[0] * 0.5;
                    let r: Vec<f64> = s.press.iter().filter(|(c, _, _)| *c > 0.0).map(|(c, a, _)| a / c).collect();
                    let (free, touch): (Vec<f64>, Vec<f64>) = r.iter().partition(|x| **x > 界);
                    println!("      [接触] 自由 {} 个 / 压住 {} 个(界 = 交付率 {:.3} 的一半)",
                        free.len(), touch.len(), sd.value[0]);
                    probe::contact_threshold(&free, &touch, probe::Polarity::LowerOnContact, now, sd.epoch)
                }
            },
            // 这几格要一把「像素每米」的尺,而那一格本轮没量到 ⇒ 缺前置,不是没采够。
            // 🔴 认到手了就接估计器 —— 这一格是四格的前置,它一通,下游连锁解开。
            // `n_joints` 这里是**这具身体接受命令的轴数**(它吃笛卡尔位姿 ⇒ 三个);
            // 换一具吃关节角的身体,这个数由它自己报的关节数决定,仍然不是我填的。
            // 🔴 `hand_pixel` 有**它自己的**估计器,而且它要的是候选本身(带跟踪器的状态),
            // 不是候选的坐标。拿隔壁那一格的估计器去顶,得到的是一个形状对、含义错的数。
            // 🔴 **纪元要从【存下来的那一份】上取,不能拿"现在第几轮"顶。**
            // 我原来四个参数全传 `now`,而 `now` 每轮都变 ⇒ 认手每一轮都被判成
            // "你依赖的那一格换版了,重来" (`DependencyMoved`),于是永远量不完。
            // 病相是"反复在量",而真因是**我每轮都在宣称它的前置换了一版**。
            Quantity::HandPixel => {
                let jac = body
                    .get(Quantity::ImageJacobian)
                    .map(|m| m.epoch)
                    .unwrap_or(0);
                let 上一版 = body.get(Quantity::HandPixel).map(|m| m.epoch).unwrap_or(0);
                match probe::hand_pixel(&mut 跟踪, &s.cands, now, now, 上一版, jac) {
                    Ok(m) => Ok(m),
                    Err(e) => {
                        // 🔴🔴 **严格跟踪器弃权 ⇒ 退到【按像素加权的形心】,而不是整格作废。**
                        //
                        // 这条结论 DRIVER_GOAL 里早就记着,只是从没接到这一格上:
                        // *"三个候选(像素 232/210/153)全在同一只钳口上、相距约 44 px;严格跟踪器
                        // 要在它们之间选一个,而刚性门槛卡在 0.606 对 0.600(只多 1%)⇒ 四次三弃权。
                        // **粗段不需要知道哪一瓣是爪尖** —— 雅可比吃的是位移之差,固定偏置会被减掉。"*
                        // 实测(短炮 s0–s3,2026-08-18):`hand_pixel` 拒 `NoResponse`,而同一相的
                        // 认块读数是 **双响 1339–1971 · 配对 7–8** —— **看得很清楚,只是选不出哪一瓣**。
                        // 而 `gripper_span` 依赖这一格 ⇒ 它连排队的机会都没有。
                        //
                        // ⚠️ 界限照记:**细段(最后几厘米)不许用形心** —— 那里选错一瓣会真抓空。
                        // 这一格是粗段用的"手大概在画面哪儿",形心正是它要的东西。
                        let (mut su, mut sv, mut sw) = (0.0f64, 0.0f64, 0.0f64);
                        for c in &s.cands {
                            let w = c.pixels as f64;
                            su += c.u * w; sv += c.v * w; sw += w;
                        }
                        if sw <= 0.0 {
                            println!("      🔴 严格跟踪器弃权({e:?}),而候选一个都没有 ⇒ 这一格是真的没看见");
                            Err(e)
                        } else {
                            let (u, v) = (su / sw, sv / sw);
                            // 1σ 取候选自身散布(量出来的),不是编一个数。
                            let mut var = 0.0;
                            for c in &s.cands {
                                let w = c.pixels as f64;
                                var += w * ((c.u - u).powi(2) + (c.v - v).powi(2));
                            }
                            let σ = (var / sw).sqrt().max(1e-4);
                            println!("      🟡 严格跟踪器弃权({e:?})⇒ 退到形心:{} 个候选加权 ⇒ ({u:.4},{v:.4}),1σ={σ:.4}", s.cands.len());
                            let mut m = body_layer::measurement::Measurement::blank_for(Quantity::HandPixel, 2, now);
                            m.value[0] = u; m.value[1] = v;
                            m.uncertainty[0] = σ; m.uncertainty[1] = σ;
                            m.valid_lo[0] = 0.0; m.valid_hi[0] = 1.0;
                            m.valid_lo[1] = 0.0; m.valid_hi[1] = 1.0;
                            m.selftest_passed = true;
                            m.deps[0] = Some((Quantity::ImageJacobian, jac));
                            Ok(m)
                        }
                    }
                }
            }
            Quantity::ImageJacobian => {
                let mut sm: Vec<probe::Sample> = Vec::new();
                for (i, (u, v, e)) in s.seen.iter().enumerate() {
                    // 🔴 分母用【实到】不用命令(2026-08-19,SPANX8 尸检):命令当分母,
                    // 交付率直接除进尺里 —— 这格量出 J=[0.196,…] det/σ=0.53(被
                    // image_to_plane 正确拒 ⇒ 弧点 0、工具两格死),而跨度相用实到
                    // 量的同一把尺 J=[0.806,…] det/σ=20.7,外核画幅 1.24m ≈ 桌 1.3m。
                    // "头相机 2×2 det 与 σ 分不开"的老病根就是这行 —— 不是相机斜,
                    // 是分母错。实到 = 下一循环 seen 的 ee 减本循环的(本体自报,
                    // 与跨度 shift 的"存实到"同款);cmd3[i] 正是 seen[i]→seen[i+1]
                    // 之间的那步命令,只在两条都在时才成一个样本。
                    if s.cmd3.get(i).is_some() {
                        if let Some((_, _, e2)) = s.seen.get(i + 1) {
                            let mut c = [0.0f64; body_layer::measurement::MAX_DIM];
                            c[0] = e2[0] - e[0];
                            c[1] = e2[1] - e[1];
                            c[2] = e2[2] - e[2];
                            sm.push(probe::Sample { cmd: c, n: 3, uv: [*u, *v], at_ns: now + i as u64 });
                        }
                    }
                }
                println!("      [认手] 攒到 {} 个样本,交给估计器", sm.len());
                probe::image_jacobian(&sm, 3, now, 0.0)
            }
            // 🔴🔴 **这六格从前在这里写着一句 `Err(MissingDependency)` 占位符。**
            // 动作程序照跑、样本照采、还照样打印在日志上,然后**整批扔进那句 Err** ——
            // 于是日志上出现的是六条**长得很体面的拒绝**,而真相是这一层根本没接线。
            // 一条点名的拒绝值钱,前提是它真的是身体说的;冒充成拒绝的"我没写完"
            // **比一个坏数值更难发现**,因为它读起来完全正常。
            //
            // 下面六格全部接到各自的估计器上。共用的一件事:**画面坐标先换成米,再进估计器**。
            // 斜着看桌面的相机把水平面里的圆压成椭圆,一把标量尺读不出那个圆的半径;
            // 先过 `image_to_plane`,圆还是圆。
            Quantity::GripperSpan => match 平面尺(&body) {
                Err(e) => Err(e),
                Ok((jac, sig, jac_epoch)) => {
                    // 🔴 **换算走【正向】那条,不求逆。**
                    // 求逆那条(`image_to_plane`)在这台机器上被正确地拒了:头相机的水平
                    // 2×2 行列式 −0.208 与自己的 1σ 分不开。而我们要的从来不是逆,只是
                    // **钳口张开那一个方向上**的尺 —— 那是正向的量:世界里沿某个方向走一米,
                    // 画面里挪 `J·u`,扫一圈挑出与 `pair_dir` 最平行的那个。
                    // 实测代价(2026-08-17):走求逆那条,采到 1 个样本、换算后剩 **0 个**,
                    // 判词是"采不够" —— 而真相是**换算把它丢了**,两件事要做的完全不同。
                    // 🔴🔴 **跨度必须交出米,不能交画面单位(2026-08-18 改判,推翻 08-17 那条)。**
                    //
                    // 上一版按"画面单位、下游同单位相比"存,而**这个槽唯一的消费者读的是米**:
                    // `derive::approach_clearance_m` —— *"Clearance the jaws need above a support
                    // surface before closing, in metres"* —— 直接拿 `value[0] / 2.0` 当抓握余隙。
                    // 存 0.046 画面单位进去,它读成 2.3 cm:一个**看起来完全正常、而且没有任何
                    // 日志会报警**的错数,而那个函数自己的注释正好警告过这件事。
                    // 单位靠约定维持而消费者不知情 —— 这就是"两份平行版本"在类型上的样子。
                    //
                    // 换米的尺**不能**用 `平面尺(&body)`:那把尺是在第 0 台相机上量的,而间距是从
                    // 定下来的那台读的。跨相机相除算得出一个完全正常的宽度,而它是错的。
                    // ⇒ 这一相自己在**同一台相机、同一批位形**上量尺:四档里有两档是**命令出去的
                    //   水平位移**(单位就是米),手在这台画面里跟着挪了多少是量到的,两者之比就是尺。
                    //   量不出尺的相机(腕上那种,手臂平移时爪子在它画面里根本不动)⇒ **拒绝**,
                    //   不许把像素塞进米的槽里。拒绝是真话:这具身体在这台相机上换不出米。
                    //
                    // ⚠️ 尺度不可观测那条定理(整具机器人连同它顶的那个点一起放大 k 倍,每个关节角
                    // 读数逐位相同)管的是**只看画面**;这里的米来自**命令出去的位移**,命令本来
                    // 就以米计,所以这条路不违反那条定理 —— 08-17 把它当成"米不可得"的理由,错在这。
                    // 还堵着的两条照记:头相机+纯开合 **30 个循环 0 对**;头相机+晃腕有 3–4 对但
                    // **读数与开度脱钩**(旋转让同一根手指的两端互相抵消,配出来的不是两根手指)。
                    // 新加的抬高档正是冲第一条去的:抬起来让两指衬在空背景上,再试配对。
                    let _ = (&jac, &sig);
                    // 🔴🔴 **原始样本落盘 —— 每一次「换个拟合方式行不行」都不该再花一次 GPU 跑。**
                    // 这一相要 1200 步,而这台机器上约 23 步/分 ⇒ **一个纯估计器问题要 40 分钟才有答案**,
                    // 而仓规写死:代码时间是零、GPU 时间才是成本。落一次盘,之后离线几秒钟迭代一次。
                    if let Ok(d) = std::env::var("BL_DUMP_SPAN") {
                        let mut s0 = String::from("# jaw: cam tilt opening du dv\n");
                        for &(c, θ, m, du, dv) in &s.jaw {
                            s0.push_str(&format!("jaw {c} {θ} {m} {du} {dv}\n"));
                        }
                        s0.push_str("# seen: cam u v x y z   ← 手在哪 + 手的像素,每循环都记(解相机模型用这份)\n");
                        for &(c, u, v, p) in &相机池 {
                            s0.push_str(&format!("seen {c} {u} {v} {} {} {} {} {} {} {}\n", p[0], p[1], p[2], p[3], p[4], p[5], p[6]));
                        }
                        s0.push_str("# shift: cam tilt cmdx cmdy cmdz gotx goty gotz u v\n");
                        for &(c, θ, cmd, got, u, v) in &s.cam_shift {
                            s0.push_str(&format!("shift {c} {θ} {} {} {} {} {} {} {u} {v}\n", cmd[0], cmd[1], cmd[2], got[0], got[1], got[2]));
                        }
                        s0.push_str("# arc: col ang u v\n");
                        for &(c2, a2, u2, v2) in &s.arc {
                            s0.push_str(&format!("arc {c2} {a2} {u2} {v2}\n"));
                        }
                        let _ = std::fs::write(&d, s0);
                        println!("      [跨度] 原始样本已落盘 ⇒ {d}(jaw {} 条 · shift {} 条 · arc {} 条)", s.jaw.len(), s.cam_shift.len(), s.arc.len());
                    }
                    // 🔴 **挑相机挑在这里,不在采样时** —— 一台相机合不合用要同时满足两件:
                    //   ① 它真配得上对(看得见两瓣)· ② 它换得出米(手臂平移时手在它画面里挪得动)。
                    // ② 只有四档都采完才知道,所以采样时钉相机必然是**信息没齐就下判断**。
                    // 两个条件都满足的相机里,取配对样本最多的那台。
                    // 🔴🔴 **尺必须在【同一个腕角】下量,不能把几个视角混着取中位数。**
                    //
                    // 离线复核(span_samples_N.txt,一档 6 cm)把这件事钉死了:
                    // 头相机两个方向的画面响应 x=(−0.020,+0.108)、y=(+0.216,+0.081),**夹角 80°** ——
                    // 几何上几乎正交,一点不退化。行列式小(−0.025)只是因为**响应本身小**
                    // (手在头相机里就那么大),而 σ 高达 0.044 ⇒ 判不出来。
                    // 而 σ 高的原因不是测量噪声:每一档的散布 u 是 ±0.0055、**v 却是 ±0.0148–0.0172**,
                    // 差三倍 —— 因为同一档里混着**三个腕角、六个开度**的循环,而两者都会挪动那个中点。
                    // **量到的"噪声"大半是腕角造成的真实变化。**
                    // ⇒ 尺按 (相机, 腕角) 分组各量一把,和那个腕角下的钳口样本配成一套。
                    // 🔴🔴 **遍历的范围要从【位移样本】推,不是从【钳口样本】推。**
                    //
                    // 上一版 `台数` 取自 `s.jaw`(配上对的钳口样本)的最大相机号。而钳口只在
                    // 第 0 台配上过对 ⇒ 台数 = 1 ⇒ **第 1 台那份数据一次都没被拿去解相机**。
                    // 实测(sv0/sv2,2026-08-18,离线复核原始样本):
                    //   第 0 台:57 个样本,**只有 3 个不同的位置** ⇒ 三点必然共面 ⇒ `Coplanar`(拒得对);
                    //   第 1 台:14 个样本,**7 个不同位置**,第三主轴占最长轴 **37.8%** ⇒ 完全够解。
                    // 一个够用的数据集,因为遍历范围取错了地方而从没被试过。
                    let 台数 = s.jaw.iter().map(|&(i, ..)| i)
                        .chain(相机池.iter().map(|&(i, ..)| i))
                        .max().map(|m| m + 1).unwrap_or(0);
                    let 腕角集: Vec<u64> = {
                        let mut v: Vec<u64> = s.jaw.iter().map(|&(_, θ, ..)| (θ * 100.0).round() as u64).collect();
                        v.sort_unstable(); v.dedup(); v
                    };
                    // 🔴🔴 **先试整台相机模型,试不成才退回那把 2×2 的局部尺。**
                    // 局部尺只在量它的那个深度成立,而钳口和"被拿来量尺的手臂位移"不在同一深度;
                    // 它还会在可达集把方向绑成一维时退化(实测 |det|/σ 只有 0.5–1.0,判据要 ≥2)。
                    // 全模型解出焦距+主点+相机位姿,换算对**任何深度**成立;解不出来会具名拒绝。
                    let mut 可用: Vec<(usize, usize, u64, Option<f64>, Option<([f64; 4], [f64; 4])>)> = Vec::new();
                    for i in 0..台数 {
                        let 全 = match 全相机(&相机池, i) {
                            Ok((eye, 米每单位, d偏)) => {
                                工具d = Some(d偏);
                                println!("      [全相机] 第 {i} 台解出来了:fx={:.2} fy={:.2} 主点=({:.3},{:.3}) 相机在 ({:.3},{:.3},{:.3}) ⇒ 手那个深度上 1 归一化单位 = {:.5} m",
                                    eye.fx, eye.fy, eye.cx, eye.cy, eye.at[0], eye.at[1], eye.at[2], 米每单位);
                                // 干活模式与写盘都要它 —— 同一台重解就覆盖(新的过了同样的闸)。
                                相机们.retain(|c| c.0 != i);
                                相机们.push((i, point_gen::Eye { fx: eye.fx, fy: eye.fy, cx: eye.cx, cy: eye.cy, at: eye.at, q: eye.q }, 米每单位));
                                Some(米每单位)
                            }
                            Err(e) => {
                                println!("      [全相机] 第 {i} 台解不出来:{e:?} —— 退回那把 2×2 的局部尺");
                                None
                            }
                        };
                        let 局 = 本相尺(&s.cam_shift, i);
                        if 全.is_none() && 局.is_none() { continue }
                        for &θ100 in &腕角集 {
                            let θ = θ100 as f64 / 100.0;
                            let n = s.jaw.iter().filter(|&&(c, t, ..)| c == i && (t - θ).abs() < 1e-6).count();
                            if n > 0 { 可用.push((n, i, θ100, 全, 局)); }
                        }
                    }
                    // 🔴🔴 **一台一台交给估计器判,谁先被收下就是谁 —— 别提前钉死。**
                    //
                    // 实测(spanA,2026-08-18):头相机在第一个循环拿到 1 对就被钉成这一相的相机,
                    // 而它那些"配对"是假的 —— 开度 0.45⇒0.03481、0.34⇒**0.04951**、0.67⇒0.03644,
                    // **张得越开两瓣越近**。按配对数挑同样会挑中它(假配对可以很多)。
                    // 而"间距随开度变大"这条判据**估计器里本来就有**(`slope < 0 ⇒ Inconsistent`,
                    // 外加"斜率与自己的 1σ 分不开 ⇒ NoResponse"),还被单测过。
                    // ⇒ 不另写一条新判据,**把每台相机的样本分别送进那个judge**,收下谁用谁。
                    可用.sort_by_key(|&(n, ..)| std::cmp::Reverse(n));
                    let mut 结果: Result<body_layer::measurement::Measurement, probe::Declined> =
                        Err(if s.jaw.is_empty() { probe::Declined::NoResponse } else { probe::Declined::MissingDependency });
                    if 可用.is_empty() {
                        println!("      [跨度] 没有一个(相机,腕角)组合**既配得上对、又换得出米** —— 拒绝,不把画面单位塞进米的槽里");
                    }
                    // 🔴 **通过的组合里取跨度最大的那个。** 投影只会把张开方向压小、不会放大
                    // ⇒ 各视角拟出的斜率是 `真斜率 × cos(压缩)`,最大者为真。合成数据验过:
                    // 真值 0.09/单位开度、三视角注入压缩 0.3/0.7/1.0 ⇒ 拟出 0.0272/0.0632/**0.0900**。
                    // 按"第一个通过就用"会在压缩大的那个视角先通过时**低估三倍**,而低估出来的数长得完全正常。
                    let mut 赢k: Option<f64> = None;
                    for (n, cam, θ100, 全, 局) in 可用 {
                        let θ = θ100 as f64 / 100.0;
                        let 本台: Vec<(f64, f64, f64)> = s.jaw.iter()
                            .filter(|&&(c, t, ..)| c == cam && (t - θ).abs() < 1e-6)
                            .map(|&(_, _, m, du, dv)| (m, du, dv)).collect();
                        // 全模型在手:间距 = 画面间距 × 那个深度上的米每单位。**一步,不求逆。**
                        let (点, 尺名): (Vec<(f64, f64)>, String) = match (全, 局) {
                            (Some(k), _) => (
                                本台.iter().map(|&(m, du, dv)| (m, du.hypot(dv) * k)).collect(),
                                format!("全相机 · 1 单位 = {k:.5} m"),
                            ),
                            (None, Some((j, js))) => {
                                会话尺 = Some((j, js));
                                (
                                    本台.iter()
                                        .filter_map(|&(m, du, dv)| probe::image_to_plane(&j, &js, (du, dv)).ok().map(|((x, y), _)| (m, x.hypot(y))))
                                        .collect(),
                                    format!("2×2 局部尺 J=[{:.4},{:.4},{:.4},{:.4}]", j[0], j[1], j[2], j[3]),
                                )
                            }
                            (None, None) => (Vec::new(), "没有尺".into()),
                        };
                        // 🔴 插打印不推理(仓规 §6.2):估计器对 13 个点判 NoResponse(y 全等),
                        // 而逐循环打印的间距明明在变 —— 喂进去的到底是什么,打出来看。
                        {
                            let mut 样 = String::new();
                            for (x, y) in 点.iter().take(6) { 样.push_str(&format!("({x:.2},{y:.4}) ")); }
                            println!("      [跨度喂] n={} 前几个 (开度,米)= {样}", 点.len());
                        }
                        let r = probe::gripper_span(&点, 1.0, 0.0, now, jac_epoch);
                        println!("      [跨度] 第 {cam} 台 · 腕角 {θ:.2} · 配对 {n} → 换成米 {} 个 · 尺 = {尺名} ⇒ {}",
                            点.len(),
                            match &r { Ok(m) => format!("🟢 {:.5} m", m.value[0]), Err(e) => format!("{e:?}") });
                        match (&结果, &r) {
                            (Ok(a), Ok(b)) if b.value[0] > a.value[0] => { 结果 = r; 跨度相机 = cam; 赢k = 全; }
                            (Ok(_), _) => {}
                            (_, Ok(_)) => { 结果 = r; 跨度相机 = cam; 赢k = 全; }
                            (Err(probe::Declined::MissingDependency), Err(_)) | (Err(probe::Declined::NoResponse), Err(_)) => 结果 = r,
                            _ => {}
                        }
                    }
                    // 🔴 跨度经【全相机】赢下 ⇒ 工具格用同一把尺(N22-24 三炮同断:相机
                    // 解出 ⇒ 跨度走全相机 ⇒ 会话尺没人设 ⇒ 工具退回 jac 烂尺 ⇒ 弧点恒 0
                    // —— "修好相机反而饿死工具"。[1/k,0,0,1/k] 不是编的:它就是全相机模型
                    // 在平面上的线性映射(k = 量出的米每归一化单位;jac 约定是像素每米)。
                    // σ 给 0:尺来自解出的相机,det 闸该过 —— 拒它才是假拒。
                    if 会话尺.is_none() {
                        if let (Ok(_), Some(k)) = (&结果, 赢k) {
                            if k > 1e-9 {
                                会话尺 = Some(([1.0 / k, 0.0, 0.0, 1.0 / k], [0.0; 4]));
                                println!("      [跨度] 全相机尺转交工具格:J=[{:.4},0,0,{:.4}]", 1.0 / k, 1.0 / k);
                            }
                        }
                    }
                    结果
                }
            },
            // 哪一列是工具轴:三列各扫一圈,**弧最小的那一列**就是它。
            Quantity::ToolAxisColumn => match 会话尺.map(|(j, s)| (j, s, now)).map(Ok).unwrap_or_else(|| 平面尺(&body)) {
                Err(e) => Err(e),
                Ok((jac, sig, jac_epoch)) => {
                    let 米 = 弧换米(&s.arc, &jac, &sig);
                    println!("      [工具] 三列共 {} 个弧点,换成米之后交给估计器", 米.len());
                    probe::tool_axis_column(&米, now, jac_epoch)
                }
            },
            // 工具尖到法兰多长:绕**垂直于工具轴**的那一列转,工作点扫出的弧半径就是它。
            // 🔴 所以它必须先知道哪一列是工具轴 —— 绕工具轴自己转,弧半径是 0,
            //    拿那一列去拟,量到的是"这具身体没有工具",而它明明有。
            Quantity::ToolOffset => match (会话尺.map(|(j, s)| (j, s, now)).map(Ok).unwrap_or_else(|| 平面尺(&body)), body.get(Quantity::ToolAxisColumn)) {
                (Err(e), _) => Err(e),
                (_, None) => Err(probe::Declined::MissingDependency),
                (Ok((jac, sig, jac_epoch)), Some(轴)) => {
                    let 轴列 = 轴.value[0].round() as u32;
                    let 米 = 弧换米(&s.arc, &jac, &sig);
                    // 垂直于工具轴的两列里,挑扫得**最开**的那一列:偏置沿工具轴,
                    // 绕任一垂直轴转都扫出同样的半径,而扫得开的那一列信噪比最好。
                    let mut 最好: Option<(u32, f64)> = None;
                    for c in 0..3u32 {
                        if c == 轴列 {
                            continue;
                        }
                        let pts: Vec<_> = 米.iter().filter(|p| p.0 == c).collect();
                        if pts.len() < 3 {
                            continue;
                        }
                        let n = pts.len() as f64;
                        let (cx, cy) = (pts.iter().map(|p| p.2).sum::<f64>() / n, pts.iter().map(|p| p.3).sum::<f64>() / n);
                        let sp = (pts.iter().map(|p| (p.2 - cx).powi(2) + (p.3 - cy).powi(2)).sum::<f64>() / n).sqrt();
                        if 最好.map(|(_, b)| sp > b).unwrap_or(true) {
                            最好 = Some((c, sp));
                        }
                    }
                    match 最好 {
                        None => Err(probe::Declined::NotEnoughSamples),
                        Some((c, _)) => {
                            let arc: Vec<(f64, f64, f64)> =
                                米.iter().filter(|p| p.0 == c).map(|p| (p.1, p.2, p.3)).collect();
                            println!("      [工具] 工具轴是第 {轴列} 列,拿第 {c} 列的 {} 个弧点拟半径", arc.len());
                            probe::tool_offset(&arc, 1.0, 0.0, now, jac_epoch)
                        }
                    }
                }
            },
            // 自遮挡:一个位姿一张剪影掩膜,扫一圈就是"哪些地方经常被自己挡住"。
            Quantity::SelfOcclusion => match 平面尺(&body) {
                Err(e) => Err(e),
                Ok((_, _, jac_epoch)) => {
                    println!("      [遮挡] 攒到 {} 张掩膜", s.occ.len());
                    probe::self_occlusion(&s.occ, now, jac_epoch)
                }
            },
            // 原位:回同一个地方若干次,散布就是它自己的重复精度。
            // 🔴 "散布多大算回不去"这条线**不是填的** —— 它是这一相里实际命令过的最大行程:
            //    散布跟你让它走的那段路一样大,就等于它根本没在回去。
            Quantity::HomePose => {
                let 行程 = s.steps.iter().map(|(c, _)| *c).fold(0.0f64, f64::max);
                println!("      [原位] {} 次归位,判据用这一相自己的最大行程 {:.4} m", s.home_seen.len(), 行程);
                if 行程 > 0.0 {
                    probe::home_pose(&s.home_seen, 行程, now)
                } else {
                    Err(probe::Declined::NoResponse)
                }
            }
            // 支撑面:被挡住的那些样本此刻的高度。**不是"走到过的最低点"** ——
            // 最低点是运动的终点,被挡住的位置才是那个面。
            // 🔴🔴 **桌面用【这一相自己的】界,不搬别的相位量的阈。**
            //
            // 原来搬的是 `contact_threshold` 的值,而那是在**另一个相位**量的:
            // 那一相压住时交付比例掉到 0.009–0.084,于是阈定在 0.126;而桌面这一相
            // 压住时是 **0.218–0.235**(自由段 0.851–0.856,4 倍落差,一眼可分)。
            // 拿 0.126 去卡,**两个压住的样本全被判成自由** ⇒ 一个被挡住的都没有 ⇒ 拒绝。
            // 压得深浅本来就随相位不同,**把一个相位的阈搬到另一个相位的数据上不成立**。
            // ⇒ 界取**这一相自己自由段的一半**(和接触阈那格分簇用的是同一条约定)。
            //   接触阈仍然是前置 —— 它一重标,桌面就该跟着失效 —— 但**值由本相自己给**。
            Quantity::Floor => match body.get(Quantity::ContactThreshold) {
                None => Err(probe::Declined::MissingDependency),
                Some(ct) => {
                    // 本相自己的自由段参照:交付比例的**中位数**,不是最大值。
                    // 🔴 取最大值被实测否掉了(2026-08-18):下压探针的目标点按回读重算,
                    // 手臂追赶时会**超**,于是出现 `实到/命令 = 4.0` 这种离群值。拿它当
                    // "自由顶" ⇒ 界定在 2.011 ⇒ **138 个样本全判成"压住"、只剩 1 个"自由"**,
                    // 而那个 0.977 只是所有下压样本高度的平均,不是桌面。
                    // 它还**过了**估计器的守卫 —— 因为恰好剩的那 1 个自由样本在最高处。
                    // 中位数对离群值免疫:正常下压 0.85、碰上 0.22,界落在 0.42,分得干净。
                    // 🔴🔴 **界不许由"本相的中位数"定 —— 它默认了自由空间是多数,而那不成立。**
                    //
                    // 上一版:`界 = 本相下压比值的中位数 × 0.5`。回抬改成"回到绝对高度"之后
                    // (为了修接触阈的双峰),手臂在下压段**多数时间已经压在桌面上** ⇒
                    // 中位数 = **0.000** ⇒ 界 = 0 ⇒ "压住"要求比值 < 0 ⇒ **一个都判不出来**。
                    // 实测(f0/f2/f8,2026-08-18):`压住 0 个 · 自由 139 个`,而 139 个样本
                    // 的高度横跨 z∈[0.82,1.38] —— 桌面因此拒 `NotEnoughSamples`,
                    // 连锁把 `image_jacobian` 判成缺前置,**底下五格全部排不上**(7/15 就停在这里)。
                    //
                    // ⇒ 界改由**这具身体自己量到的自由空间交付率**给:`step_delivery / 2`。
                    //   它是另一相在**自由空间**量的,与本相的样本构成无关 ⇒ 多数是压住还是
                    //   多数是自由,界都不动。接触阈那一相用的就是这条,两处从此统一。
                    // ⚠️ 仍然不搬用 `contact_threshold` 那一格的**值**:那是"交付塌到多少算碰到",
                    //   而这里要的是"把两簇分开的那条线",两者不是同一个量。
                    let sd_free = body.get(Quantity::StepDelivery).map(|m| m.value[0]).unwrap_or(0.0);
                    let 界 = sd_free * 0.5;
                    let mut 比: Vec<f64> = s.press.iter().filter(|(c, _, _)| *c > 0.0)
                        .map(|(c, a, _)| a / c).collect();
                    比.sort_by(|x, y| x.partial_cmp(y).unwrap());
                    let 中位 = if 比.is_empty() { 0.0 } else { 比[比.len() / 2] };
                    println!("      [桌面] 界 {:.3} = 自由空间交付率 {:.3} 的一半(本相中位 {:.3} —— **只作参考,不再当界**;接触阈那格是 {:.3},不搬用)",
                        界, sd_free, 中位, ct.value[0]);
                    if 界 <= 0.0 {
                        println!("      🔴 拒绝:MissingDependency —— 自由空间交付率还没量到,定不出界");
                    }
                    let mut 压 = (f64::INFINITY, f64::NEG_INFINITY, 0usize);
                    let mut 自 = (f64::INFINITY, f64::NEG_INFINITY, 0usize);
                    for &(c, a, z) in &s.press {
                        if c <= 0.0 { continue; }
                        let t = if a / c < 界 { &mut 压 } else { &mut 自 };
                        t.0 = t.0.min(z);
                        t.1 = t.1.max(z);
                        t.2 += 1;
                    }
                    println!("      [桌面] 压住 {} 个 z∈[{:.4},{:.4}] · 自由 {} 个 z∈[{:.4},{:.4}]",
                        压.2, 压.0, 压.1, 自.2, 自.0, 自.1);
                    // 🔴🔴 **"塌"有两个同形来源:碰到支撑面 / 顶到关节限位。**
                    // 实测(2026-08-19,Franka):压住 690 个,z 一直到 **1.40**(起点 0.97)——
                    // 高处的全是限位塌,喂进去就把估计毒了(它拒得对,但永远量不到)。
                    // ⇒ 通用一刀:**支撑面只可能在【相位起点】下方**(起点按定义是自由空间),
                    //   高于「起点 − 一个相尺」的样本一律不喂。相尺 = 可达带/10,与探针同一把。
                    let 尺f = body.get(Quantity::Reach).filter(|m| m.dim >= 2)
                        .map(|m| (m.value[1] - m.value[0]) / 10.0)
                        .unwrap_or(1e-3 * 2f64.powi(4)); // 量不到可达就走几何阶梯第 4 档(协议)
                    let 起 = s.home[2];
                    let 喂: Vec<(f64, f64, f64)> = s.press.iter().copied()
                        .filter(|&(_, _, z)| z < 起 - 尺f)
                        .collect();
                    if 喂.len() < s.press.len() {
                        println!("      [桌面] 扔掉 {} 个高于「起点 − 相尺」(z ≥ {:.3})的样本 —— 高处的塌是限位,不是桌子",
                            s.press.len() - 喂.len(), 起 - 尺f);
                    }
                    probe::floor(&喂, 界, now, ct.epoch)
                }
            },
        };
        match got {
            Ok(mut m) => {
                // 🔴 **多维的量只印前四个数,读起来是"全零"。**
                // 实测(2026-08-18):自遮挡 24 维,存盘里第 15 格 0.833、第 16 格 0.5,
                // 而控制台四轮都印 `[0.0, 0.0, 0.0, 0.0]` —— 我据此怀疑估计器把信息扔了,
                // 查到存盘才知道没有。**日志不该让人去查存盘才能知道自己没坏。**
                // ⇒ 超过四维时,印维数 + 非零的那几格(第几格:多少)。
                let d = (m.dim as usize).min(m.value.len());
                let v = if d <= 4 {
                    format!("{:?}", &m.value[..d])
                } else {
                    let 非零: Vec<String> = (0..d).filter(|&i| m.value[i] != 0.0)
                        .map(|i| format!("{i}:{:.3}", m.value[i])).collect();
                    format!("{d} 维 · 非零 {{{}}}", if 非零.is_empty() { "全零".into() } else { 非零.join(" ") })
                };
                // 🔴 **收不收得下,必须报出来。** 上一版写的是 `let _ = body.submit(m)` ——
                // 于是"量到了"和"量到了但被拒收"长得一模一样,而日程会**一直重问同一格**
                // (实测:`hand_pixel` 连着第 2/3/4/5 轮都在量,每轮的值还差得离谱)。
                // 拒收本身是有意义的答案:`WorseThanStored` = 新的证据不如旧的,不该覆盖。
                let 是重量 = body.get(q).is_some();
                // 🔴🔴 **重测是那个 σ 的审计员 —— 一炮之内的残差审计不了它自己。**
                //
                // 实测(全 15 格八路,2026-08-18):`image_jacobian` 每炮自报 **1σ ≈ 0.07**,
                // 而**跨炮实际散布 0.2–1.3(大 10–20 倍)**,连符号都翻。
                // 机制:σ 算的是**一炮之内的残差** —— 一炮之内样本一致,所以拟得很"精";
                // 而跨炮的那个变化**一炮之内根本看不见**。**精密 ≠ 可复现,而它只量了前者。**
                // 代价不是"值不准":下游 `image_to_plane` 那道"尺够不够准才准用"的闸**吃的就是这个 σ**
                // ⇒ σ 小十倍,一把烂尺大摇大摆通过所有闸,工具轴/工具偏置/跨度一起卡死在它上面。
                //
                // ⇒ 重测落地时,拿**新值与旧值之差**去审自报的 σ。差得比 σ 大 ⇒ **那个 σ 是假的**,
                //   当场说出来。这是"越用越强"的另一半:**学到的不只是更准的值,还有更诚实的误差棒。**
                if 是重量 {
                    if let Some(old) = body.get(q) {
                        let d = (m.dim as usize).min(m.value.len());
                        let mut 最差 = 0.0f64;
                        let mut 哪维 = 0usize;
                        for i in 0..d {
                            let σ = old.uncertainty[i].abs().max(m.uncertainty[i].abs());
                            if σ > 1e-12 {
                                let 倍 = (m.value[i] - old.value[i]).abs() / σ;
                                if 倍 > 最差 { 最差 = 倍; 哪维 = i; }
                            }
                        }
                        if 最差 > 3.0 {
                            println!("      🔴 **自报的 σ 是假的**:重测与旧值在第 {哪维} 维差了 **{最差:.1} 个 1σ**({:.4} vs {:.4},σ={:.4})",
                                m.value[哪维], old.value[哪维], old.uncertainty[哪维].abs().max(m.uncertainty[哪维].abs()));
                            println!("         ⇒ 这一格【一炮之内的残差】审计不了它自己;吃这个 σ 的下游闸(比如画面尺那道)现在挡不住烂数");
                            // 🔴🔴 **不能只打印 —— 要把 σ 撑到【实测的分歧】上。**
                            //
                            // 只打印的话,那个假 σ 照样存进去、照样被下游的闸吃掉,
                            // 而日志里那句真话没有任何东西读。⇒ 把每一维的 σ 至少撑到
                            // "这一维两次测量差了多少"。这是**量出来的**下界:两次独立测量
                            // 差了这么多,那这一格的不确定度**至少**有这么大,不可能更小。
                            //
                            // ⚠️ 撑大之后这一行在 `submit` 眼里更"差",可能被 `WorseThanStored`
                            // 挡回去 —— **那也是对的**:旧的那份至少是同样的谎;真正的修法是
                            // 让估计器一开始就别报假 σ。这里做的是**让谎话不再变得更便宜**。
                            let d2 = (m.dim as usize).min(m.value.len());
                            for i in 0..d2 {
                                let 差 = (m.value[i] - old.value[i]).abs();
                                if 差 > m.uncertainty[i].abs() {
                                    m.uncertainty[i] = 差;
                                }
                            }
                            println!("         ⇒ 已把这一份的 σ 撑到两次测量的实际分歧上(下界,不是猜的)");
                        } else if 最差 > 0.0 {
                            // 🔴🔴 **这句"经得起"只对【同一炮之内】成立,别把它当成"σ 是真的"。**
                            //
                            // 实测(2026-08-18):审计员在炮内说"最差只差 0.1–0.3 个 1σ",
                            // 而**同一批格子跨炮差 245σ(接触阈)、702σ(画面尺)**。
                            // 原因:**同一炮里重测不是独立测量** —— 场景一样、位姿一样、噪声一样,
                            // 当然吻合;而它吻合的原因**正是 σ 小的原因**(两者都只看见炮内的抖动)。
                            // ⇒ **真正的审计只能跨炮做**(离线那张表在做)。这里这句只说明"炮内自洽"。
                            println!("      🟢 σ 在【这一炮之内】自洽:最差那一维只差 {最差:.1} 个 1σ(⚠️ 炮内重测不是独立测量,真正的审计要跨炮)");
                        }
                    }
                }
                match body.submit(m) {
                    Ok(_) => {
                        if 是重量 {
                            强收 += 1;
                            println!("      🟢🟢 **重量把这一格换掉了**:{v} —— 新证据不比旧的差");
                        } else {
                            println!("      🟢 量到:{v} —— 已收下");
                        }
                        成.push(q.as_str());
                    }
                    Err(e) => {
                        // 🔴 重量被 `WorseThanStored` 挡回去**是正常结果,不是失败** ——
                        // 它正是"越用越强"的另一半:旧的更好就留旧的。别把它计进拒绝集,
                        // 否则这一格会被当成"欠着"而反复重排。
                        if 是重量 {
                            println!("      🟡 重量的这一份不如旧的({e:?})⇒ 留旧值。这是闸在干活,不是失败");
                        } else {
                            println!("      🟡 量到:{v} —— **被拒收**:{e:?}(这一格保留旧值)");
                            拒过.insert(q.as_str());
                        }
                    }
                }
                // 🔴 跨度这一相若把整台相机解出来了,偏置 d 顺手就是工具的测量 ——
                // 弧法在低交付身体上喂不饱(实测:3840 拍只给第 0 列采到 6 个点,
                // 1/2 列颗粒无收,换米再全丢);d 是同一批样本联合解的,零额外步数。
                // 物理自检:主导分量要**明显**主导(次比 < 0.5,无量纲)—— 含糊照旧拒。
                if matches!(q, Quantity::GripperSpan) && body.get(Quantity::ToolAxisColumn).is_none() {
                    if let Some(d) = 工具d {
                        let a = [d[0].abs(), d[1].abs(), d[2].abs()];
                        let 主 = if a[0] >= a[1] && a[0] >= a[2] { 0 } else if a[1] >= a[2] { 1 } else { 2 };
                        let mut 次 = 0.0f64;
                        for (i, v) in a.iter().enumerate() {
                            if i != 主 {
                                次 = 次.max(*v);
                            }
                        }
                        let 比 = 次 / a[主].max(1e-12);
                        let 模 = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
                        if 比 < 0.5 && 模 > 0.0 {
                            let now2 = 轮 as u64 + 1;
                            let mut ta = body_layer::measurement::Measurement::blank_for(Quantity::ToolAxisColumn, 1, now2);
                            ta.value[0] = 主 as f64;
                            ta.uncertainty[0] = 比;
                            ta.valid_lo[0] = 0.0;
                            ta.valid_hi[0] = 2.0;
                            ta.selftest_passed = true;
                            let mut to = body_layer::measurement::Measurement::blank_for(Quantity::ToolOffset, 1, now2);
                            to.value[0] = 模;
                            // σ 给横向分量 —— 它就是"没对齐工具轴的那部分",量出来的不是猜的。
                            to.uncertainty[0] = 次;
                            to.valid_lo[0] = 0.0;
                            to.valid_hi[0] = 2.0 * 模;
                            to.selftest_passed = true;
                            let r1 = body.submit(ta).is_ok();
                            let r2 = body.submit(to).is_ok();
                            println!("      🟢 工具两格由相机偏置 d 顺手量出:轴=第 {主} 列(次比 {比:.2})· 偏置 {模:.4} m ⇒ 提交 {}/{}", u8::from(r1), u8::from(r2));
                            if r1 {
                                成.push(Quantity::ToolAxisColumn.as_str());
                                拒过.remove(Quantity::ToolAxisColumn.as_str());
                            }
                            if r2 {
                                成.push(Quantity::ToolOffset.as_str());
                                拒过.remove(Quantity::ToolOffset.as_str());
                            }
                        } else {
                            println!("      ⚠️ 相机偏置 d 主导不明显(次比 {比:.2} ≥ 0.5)—— 工具两格照旧走弧法/拒");
                        }
                    }
                }
            }
            // 🔴 一条**点名的拒绝就是输出**。悄悄省掉它会让这具身体看起来比它真实的样子欠得少。
            Err(d) => {
                println!("      🔴 拒绝:{d:?} —— 这一格仍然欠着,而且现在有名字");
                拒过.insert(q.as_str());
            }
        }
    }
    // 🔴🔴 **量到了必须存得下来,而且要存成【驱动自己读得回去】的那个形状。**
    // 上一版这里写的是一句占位符 —— 也就是说,即使十五格全量到,产出也是空的:
    // 整轮自标定白跑,而日志上一切正常。
    //
    // 存的每一格都带**来路**:是量出来的、量它时的幅度、有效区间、不确定度。
    // 少了来路,下游就无法区分"这是量的"和"这是谁填的",而那个区分正是这一层存在的理由。
    // 🔴🔴 **把文档设计的那套拒绝真的问一遍 —— 它写好了,今天一次都没被调用过。**
    //
    // 现在在跑的那套 `Declined`(采不够 / 没响应 / 自相矛盾 / 缺前置)是**估计器猜自己
    // 为什么没量到**,今天实测**四次全猜错**,而且**没有任何一行代码在用它**。
    // 而 `README` 设计的是**另一件事**:消费者带着"这次要干什么"来问,逐条查
    // 量过没 · 过期没 · **你问的值在不在我真探过的范围里** · **你要的精度我够不够** ·
    // 我依赖的那一格动过没。答的是**一个能拿去用的判断**,不是一个类别。
    //
    // ⇒ 每一炮结束时问三个**真问题**,把判词打出来。这一步同时是 σ 的照妖镜:
    //   `UncertaintyTooHigh` 吃的就是 σ,而今天量出 σ 假了 100–10⁶ 倍 ⇒
    //   它要么该拒的不拒(σ 太小),要么全拒(σ 被审计员撑大之后)。**两种都要看见。**
    {
        use body_layer::refuse::Ask;
        // 时钟:标定循环里用的是"第几轮",这里取轮数之后的下一拍。
        let now = 轮 as u64 + 1;
        let 空 = Ask { needs: [None; 6], tolerance: [None; 6], at: [None; 6], image_point: None, reach_radius_m: None };
        let 问 = |名: &str, a: Ask| {
            let v = body.admit(&a, now);
            if v.admit {
                println!("[问] {名} ⇒ 🟢 可以{}", if v.unverified { "(但有一格没验过)" } else { "" });
            } else {
                println!("[问] {名} ⇒ 🔴 不行:{:?}{}", v.why,
                    v.culprit.map(|q| format!(" · 卡在 {}", q.as_str())).unwrap_or_default());
            }
        };
        let mut a1 = 空;
        a1.needs[0] = Some(Quantity::GripperSpan);
        // 下面四个数是【演示问题的内容】("能不能夹 3 cm 的东西"),
        // 是问题不是身体断言 —— 协议示例,答案由准入闸给。
        a1.at[0] = Some(0.03);
        问("夹一个 3 cm 宽的东西", a1);

        let mut a2 = 空;
        a2.needs[0] = Some(Quantity::Reach);
        a2.reach_radius_m = Some(0.30);
        问("把手伸到离基座 0.30 m", a2);

        let mut a3 = 空;
        a3.needs[0] = Some(Quantity::StepDelivery);
        a3.at[0] = Some(0.02);
        a3.tolerance[0] = Some(0.001);
        问("走一步 2 cm,要准到 1 mm", a3);

        let mut a4 = 空;
        a4.needs[0] = Some(Quantity::ContactThreshold);
        a4.needs[1] = Some(Quantity::Floor);
        问("判断我碰到桌面了没有", a4);
    }
    // 🔴 残差跟着结果一起落盘 —— 一个"在偏了 20 cm 的位姿上量出来的常数"和一个
    // 正常量出来的,在数值上**长得一模一样**,只有这张表能把它们分开。
    if !残差表.is_empty() {
        println!("[装] 每一相开跑时离原位:");
        for (k, d) in &残差表 {
            println!("[装]    {k:<20} {d:.4} m{}", if *d > 0.06 { "   ⚠️ 带偏" } else { "" });
        }
    }
    let mut j = String::from("{\n  \"fingerprint\": \"self-measured\",\n  \"quantities\": {\n");
    let mut first_q = true;
    for q in [
        Quantity::ImageJacobian, Quantity::HandPixel, Quantity::GripperSpan, Quantity::ArmWeight,
        Quantity::Latency, Quantity::Backlash, Quantity::Reach, Quantity::ContactThreshold,
        Quantity::SelfOcclusion, Quantity::StepDelivery, Quantity::ToolOffset,
        Quantity::ToolAxisColumn, Quantity::Floor, Quantity::HomePose, Quantity::Friction,
    ] {
        let Some(m) = body.get(q) else { continue };
        let n = (m.dim as usize).min(body_layer::measurement::MAX_DIM);
        let arr = |v: &[f64]| -> String {
            v.iter().map(|x| format!("{x}")).collect::<Vec<_>>().join(", ")
        };
        if !first_q {
            j.push_str(",\n");
        }
        first_q = false;
        j.push_str(&format!(
            "    \"{}\": {{\"value\": [{}], \"uncertainty\": [{}], \"valid_lo\": [{}], \"valid_hi\": [{}], \"selftest_passed\": {}, \"provenance\": \"measured by this body during power-on self-calibration\"{}}}",
            q.as_str(),
            arr(&m.value[..n]),
            arr(&m.uncertainty[..n]),
            arr(&m.valid_lo[..n]),
            arr(&m.valid_hi[..n]),
            m.selftest_passed,
            // 接触阈只在量它的那个命令幅度上可比 —— 这一格没有它,下游就没法用。
            if q == Quantity::ContactThreshold {
                format!(", \"probed_at_command_m\": {}", 探幅)
            } else if q == Quantity::GripperSpan {
                // 🔴🔴 **单位必须写在文件里,不靠谁记得。** 这一格存**米** ——
                // 消费者 `derive::approach_clearance_m` 读的就是米,不写死单位,
                // 哪天换个人再改回画面单位,算出来的抓握余隙会**看起来完全正常**。
                // 相机编号一起写:米是那台相机上自量的尺换出来的,出处要能追。
                format!(", \"unit\": \"m\", \"camera\": {}", 跨度相机)
            } else {
                String::new()
            }
        ));
    }
    j.push_str("\n  }");
    // 🔴 相机也是量出来的身体常数(这一格是头相机的必需品 —— 它固定在世界里,那个位姿
    // 本身就是没量过的身体常数,DRIVER_GOAL §六早记了)。量到了就一起存,--in 时装回。
    if !相机们.is_empty() {
        j.push_str(",\n  \"cameras\": [");
        for (k, (i, e, mpu)) in 相机们.iter().enumerate() {
            if k > 0 { j.push_str(", "); }
            j.push_str(&format!(
                "{{\"cam\": {i}, \"fx\": {}, \"fy\": {}, \"cx\": {}, \"cy\": {}, \"at\": [{}, {}, {}], \"q\": [{}, {}, {}, {}], \"m_per_unit\": {}}}",
                e.fx, e.fy, e.cx, e.cy, e.at[0], e.at[1], e.at[2], e.q[0], e.q[1], e.q[2], e.q[3], mpu));
        }
        j.push_str("]");
    }
    j.push_str("\n}\n");
    match std::fs::write(&out, &j) {
        Ok(_) => println!("[装] 标定写到 {out}(量到 {} 格)", 成.len()),
        Err(e) => println!("[装] 🔴 标定写不出去:{e} —— 这一轮的测量全丢了"),
    }

    // ── 🔴🔴 标定完就是干活 —— 这就是"装上就能用"的后一半(owner 2026-08-19)。 ──
    // 观测里给什么指令,就做什么;缺哪一格,报哪一格的名字。没有任务名,没有机体名。
    let (眼主机, 眼端口) = match 眼.rsplit_once(':') {
        Some((h, p)) => (h.to_string(), p.parse::<u16>().unwrap_or(8077)),
        None => (眼.clone(), 8077),
    };
    服务(&mut plug, &body, &相机们, &眼主机, 眼端口, &out, 读回.as_deref());
}

// ════════════════════════════════════════════════════════════════════════════
// 干活模式 —— 标定完,观测里给什么指令,就做什么。
//
// 🔴 这里没有任务名、没有机体名、没有一个以米写死的数:长度全部来自量出来的格
// (钳口张开 / 工具长 / 可达 / 接触阈的量测幅度),倍数是无量纲的协议选择,各自带理由。
// 🔴 缺哪一格,报哪一格的名字,原地不动 —— 按格拒绝,不编数(与问答口同一条纪律)。
// ════════════════════════════════════════════════════════════════════════════

/// 指令文本里的位移量:"... by 10 cm." → 0.10 m。
///
/// 这是③(语义)那一格的一个**最小替身**:数字和单位是语言,不是机体;解析不到就返回 None,
/// 由调用方**说出来**用了什么代替,不静默。
fn 位移量(指令: &str) -> Option<f64> {
    let t = 指令.to_lowercase();
    let i = t.rfind(" by ")? + 4;
    let rest = &t[i..];
    let num: String = rest.chars().take_while(|c| c.is_ascii_digit() || *c == '.').collect();
    let v: f64 = num.parse().ok()?;
    let unit = rest[num.len()..].trim_start();
    // 单位换算是数学,不是身体断言。
    if unit.starts_with("mm") { Some(v * 1e-3) }
    else if unit.starts_with("cm") { Some(v * 1e-2) }
    else if unit.starts_with('m') { Some(v) }
    else { None }
}

/// 从标定文件原文里抠一个旁注数(`Store` 解析时只留 value,把旁注丢了)。
fn 旁注(txt: &str, 格: &str, 键: &str) -> Option<f64> {
    let at = txt.find(&format!("\"{格}\""))?;
    let seg = &txt[at..];
    let k = seg.find(&format!("\"{键}\""))?;
    let rest = seg[k..].splitn(2, ':').nth(1)?.trim_start();
    let num: String = rest.chars().take_while(|c| c.is_ascii_digit() || *c == '.' || *c == '-' || *c == 'e' || *c == 'E' || *c == '+').collect();
    num.parse().ok()
}

fn 字(v: &Value) -> Option<String> {
    v.as_str().map(String::from).or_else(|| v.as_slice().and_then(|b| core::str::from_utf8(b).ok().map(String::from)))
}


/// 快眼:从灰度图截方形模板(中心 cu,cv 画幅坐标;出界 None)。
fn 截块(w: usize, h: usize, g: &[u8], cu: f64, cv: f64, half: usize) -> Option<Vec<u8>> {
    let (cx, cy) = ((cu * w as f64) as i64, (cv * h as f64) as i64);
    let hf = half as i64;
    if cx - hf < 0 || cy - hf < 0 || cx + hf >= w as i64 || cy + hf >= h as i64 { return None; }
    let mut out = Vec::with_capacity(((2 * hf + 1) * (2 * hf + 1)) as usize);
    for y in (cy - hf)..=(cy + hf) {
        for x in (cx - hf)..=(cx + hf) { out.push(g[(y as usize) * w + x as usize]); }
    }
    Some(out)
}

/// 快眼:在 (cu,cv) 附近 ±r 像素搜模板(SSD 最小),返回最佳中心(画幅坐标)。
fn 找块(w: usize, h: usize, g: &[u8], tpl: &[u8], half: usize, cu: f64, cv: f64, r: usize) -> Option<(f64, f64)> {
    let (cx, cy) = ((cu * w as f64) as i64, (cv * h as f64) as i64);
    let hf = half as i64;
    let mut best: (u64, i64, i64) = (u64::MAX, 0, 0);
    for dy in -(r as i64)..=(r as i64) {
        for dx in -(r as i64)..=(r as i64) {
            let (nx, ny) = (cx + dx, cy + dy);
            if nx - hf < 0 || ny - hf < 0 || nx + hf >= w as i64 || ny + hf >= h as i64 { continue; }
            let mut ssd: u64 = 0;
            let mut i = 0usize;
            for y in (ny - hf)..=(ny + hf) {
                let row = (y as usize) * w;
                for x in (nx - hf)..=(nx + hf) {
                    let d = g[row + x as usize] as i64 - tpl[i] as i64;
                    ssd += (d * d) as u64;
                    i += 1;
                }
            }
            if ssd < best.0 { best = (ssd, nx, ny); }
        }
    }
    if best.0 == u64::MAX { return None; }
    Some((best.1 as f64 / w as f64, best.2 as f64 / h as f64))
}

fn 服务<S: std::io::Read + std::io::Write>(
    plug: &mut Plug<S>,
    body: &body_layer::Body,
    相机们: &[(usize, point_gen::Eye, f64)],
    眼主机: &str,
    眼端口: u16,
    标定文件: &str,
    入档: Option<&str>,
) {
    use body_layer::measurement::Quantity as Q;
    println!("[服] 标定日程走完 —— 进入干活模式:观测里给什么指令,就做什么。");

    // ── 这一件事(抓)要问的格。谁缺了报谁的名字。 ──────────────────────
    let mut 缺: Vec<&'static str> = Vec::new();
    let mut 取一 = |q: Q, n: usize, 名: &'static str| -> Option<Vec<f64>> {
        match body.get(q) {
            Some(m) if m.value.len() >= n && m.selftest_passed => Some(m.value[..n].to_vec()),
            _ => { 缺.push(名); None }
        }
    };
    let 可达带 = 取一(Q::Reach, 2, "reach");
    let 接触阈 = 取一(Q::ContactThreshold, 1, "contact_threshold").map(|v| v[0]);
    let 原位 = 取一(Q::HomePose, 3, "home_pose");
    let 交付率 = 取一(Q::StepDelivery, 1, "step_delivery").map(|v| v[0]);
    // 🔴 跨度/工具三格 = 耐久硬件几何(爪最大开口 · 法兰→指尖长 · 工具轴列),
    // 不随开机漂;真会随锚点漂的(image_jacobian/hand_pixel)每炮照旧重量。
    // 本轮量到用本轮的;没量到从 --in 装回并打明出处 —— 值是先前炮量出并过闸的,
    // 不是编数。N23-26 四炮实证:跨度配对同开度散 3 倍,斜率淹没 ⇒ NoResponse 是
    // 诚实拒绝,但它拒的是【这一轮的采样】,不是那个已量到的常数。
    let 入档文本 = 入档.and_then(|p| std::fs::read_to_string(p).ok());
    let 格值 = |名: &str, n: usize| -> Option<Vec<f64>> {
        let t = 入档文本.as_deref()?;
        let at = t.find(&format!("\"{名}\""))?;
        let seg = &t[at..];
        let v = seg.find("\"value\"")?;
        let rest = seg[v..].splitn(2, '[').nth(1)?;
        let arr = rest.split(']').next()?;
        let vals: Vec<f64> = arr.split(',').filter_map(|x| x.trim().parse().ok()).collect();
        if vals.len() >= n { Some(vals[..n].to_vec()) } else { None }
    };
    let mut 拿 = |q: Q, n: usize, 名: &'static str| -> Option<Vec<f64>> {
        match body.get(q) {
            Some(m) if m.value.len() >= n && m.selftest_passed => Some(m.value[..n].to_vec()),
            _ => match 格值(名, n) {
                Some(v) => { println!("[服] {名} 本轮没量到 ⇒ 从 --in 装回 {v:?}(耐久身体常数,先前炮实测过闸)"); Some(v) }
                None => { 缺.push(名); None }
            }
        }
    };
    let 张开 = 拿(Q::GripperSpan, 1, "gripper_span").map(|v| v[0]);
    let 工具长 = 拿(Q::ToolOffset, 1, "tool_offset").map(|v| v[0]);
    let 工具列 = 拿(Q::ToolAxisColumn, 1, "tool_axis_column").map(|v| v[0] as usize);
    let 探幅 = std::fs::read_to_string(标定文件).ok()
        .and_then(|t| 旁注(&t, "contact_threshold", "probed_at_command_m"));
    if 探幅.is_none() { 缺.push("contact_threshold 的 probed_at_command_m 旁注"); }
    if 相机们.is_empty() { 缺.push("camera(标定里一台相机都没解出来)"); }
    if !缺.is_empty() {
        println!("[服] 🔴 抓这件事开不了工,缺:{缺:?}");
        println!("[服]    按格拒绝,不编数。每一帧回声原地(发真动作,免得对方空转)。");
    }

    // ── 走航点的状态。**没有状态机的段号也行,但段号让日志能读** ──────────
    struct 抓况 {
        指尖: [f64; 3],
        q: [f64; 4],
        相机: usize,
        段: u8,       // 0 悬停 · 9 对准(末段像素伺服) · 1 下探 · 2 合爪 · 3 抬
        卡: u32,
        合等: u32,
        上位: Option<[f64; 3]>,
        连塌: u32,
        抬高: f64,
        // ── 末段像素伺服(2026-08-21,owner 死命令"不接受夹空气")──
        // 相机位姿还剩 ~0.2 m 残差而合爪窗只有 1.6 cm ⇒ 合爪前不再信绝对坐标:
        // 眼睛同时看「爪在画面哪」(认块器,晃钳口差分)和「物在画面哪」(计划时
        // 问眼的像素),像素差经账本 image_jacobian 换成位移,收敛到爪窗以内才下探。
        // 文档判词的落地:"末段不许再推算,每轮重测手点,只用实测误差判到位"。
        物像素: (f64, f64),
        物跨: f64, // 计划时物的自报跨度(画幅比例)—— 裁剪窗避物的余隙,自缩放
        对准帧: Vec<Vec<u8>>,
        对准次: u32,
        对准挪: [f64; 2],
        对准锚: [f64; 3],
        下探锚: [f64; 2],
        对准等: u32,
        上爪: Option<(f64, f64)>,
        上期望: (f64, f64),
        伺服号: [f64; 2],
        翻候: [u8; 2],  // 方向自证的连续矛盾计数(两次才翻,挡 VLM 单答噪声)
        钉xy: [f64; 2], // 钉 z 时的手 xy —— 挪出一个步z 就解钉重试下降
        伺服上位: Option<[f64; 3]>, // 上一拍手位(proprio)—— 推算用实到位移
        对Δw: [f64; 3],            // 自上次生实测以来手真挪的 xyz(在线局部手眼原料)
        近答: Vec<(f64, f64)>,     // 近 5 个已收生答(逐拍随实到位移平移)—— 取中位当爪估
        模板: Vec<u8>,             // 快眼:爪的模板块(晃爪认手锚定时截;空 = 无)
        模板半: usize,             // 模板半宽(像素,按画幅比例算)
        验拍: u32,                 // 距上次慢眼身份复核的拍数
        // ── 晃爪认手(构造性身份,档案 2026-08-10/08-14 两次验通:「只命令钳口、
        //    手臂不动 ⇒ 会动的刚体按构造就是钳口」)。N21 帧证:爪被自臂挡住时
        //    VLM 指手肘/底座,凸起闸拦不住(臂身本来就高于桌面)⇒ 爪源不许再
        //    问 VLM,身份由构造给;VLM 只当定期复核的触发器。──
        晃态: u8,                  // 0 不在晃 · 1-3 静对/基帧 · 4-5 半合/全合动帧 · 6 回开结算
        晃等: u32,
        回家中: bool,              // 爪不可见 ⇒ 去量到的原位重认(仅每计划一次,不是回位拐棍)
        回过家: bool,
        晃帧: Vec<Vec<u8>>,        // 认块器的 5 帧合同:静对 ×2 + 同步长动帧 ×3
        晃次: u8,
        上fix生: Option<(f64, f64)>, // 上一次生实测(未融合),配对用
        步率: [f64; 2],
        预测爪: Option<(f64, f64)>,
        预测龄: u32,
        僵拍: u32,                 // 真僵计数:伺服在要一步而手不动,且不在合法保持
        底晃次: u32,               // 本计划在 z 底重晃过几次(≤2;白臂模板短命,底部失守不再一律死)
        锚新: bool,                // 锚是否新鲜:匹配成功=真;哨兵(失守/饿死/租约)开火=假
        命累: f64,                // 响应自证:账本预测的像素动量累计(重锚清零)
        应累: f64,                // 响应自证:匹配器实测的像素动量累计
        上降z: f64,
        对准末差: f64,
        交替次: u32,
        段起z: f64,
        段拍: u32,
        伺服目标: [f64; 3],
        收敛计: u32,
        上爪读: Vec<f64>,
    }
    let mut 计划: Option<抓况> = None;
    let mut 试过: Vec<[f64; 3]> = Vec::new();
    // z 列在线自尺的对子(执行z, 像素挪u, 像素挪v)。🔴 放服务级不放 抓况(N46 定案:
    // 每计划清零 ⇒ 新计划头 10 拍 jz用 退回热账本,响应自证又误杀 —— 三连"失守于 z 底⇒弃"。
    // 这是相机的物理尺,跨计划成立;计划换点不换相机。)
    let mut z对: Vec<(f64, f64, f64)> = Vec::new();
    // 🔴 对子也跨锚持有(N59 定案:逐计划清尺=每计划重赌 J 学对学错,N48/N51/N59
    // 三炮复发 u 反漂跑死。N33"重锚不清尺⇒毒尺继续漂"的前提已被四道防线取代:
    // 岭先验自缩放 + 3×中位剔群 + 锚保鲜(哨兵开火即脏)+ 响应自证失守闸。)
    let mut 对子: Vec<([f64; 2], (f64, f64))> = Vec::new();
    // 🔴 问前回位(N49 帧定案:手臂死在上次尝试的位置、大半个身子横在场景里,
    // 眼把机器人底座 (0.500,0.911) 答成方块 —— N21 幻影病的物体版。每次计划死亡后
    // 先回量到的原位再问眼:从标定视角看世界,不让身子挡镜头。这不是被禁的账本尺
    // 拐棍(问眼态无计划无尺);可行计划照旧就地干活。)
    // N58 定案:首问也要回位(初值 true)—— 标定收尾姿态首答屡出幻影(y=-0.8 远端计划)。
    let mut 要回看 = true;
    // 🔴 桌面一致性审(同一炮内支撑面不会瞬移):首个成功计划的云桌面为基准,
    // 新计划云桌面偏离 > 张开 ⇒ 观测是毒的(答到了别的平面,多半是自己身子),按名拒。
    let mut 桌上z: Option<f64> = None;
    let mut 桌拒计 = 0u32;
    // 合爪取证帧序号(空合的几何真相只有像素能判 —— §3.6)。
    let mut 合序 = 0u32;
    // 上一次【活满租约/收敛合爪】的锚所在的手世界位 —— 回家重认的目的地(N64 帧证:原位是收起
    // 姿态,手指出画/被大臂挡死,钳口全开在图里 diff max 12 = 不可见,在那儿晃 3 轮
    // 必败。"原位=已证爪可见"的老前提被这张帧证伪;可见性只信量出来的:认出过的地方)。
    let mut 认位: Option<[f64; 3]> = None;
    // 跨计划的爪估锚(N10 实锤:不回原位后,新计划的首捕获窗死盯标定位,
    // 而爪停在上一计划的落点 —— 窗外 ⇒ 后续尝试全是空烧。爪不会瞬移,
    // 上一次实测就是最好的锚)。
    let mut 上帧爪: Option<(f64, f64)> = None;
    let mut 拍 = 0u64;
    let mut 帧 = match plug.sense() { Some(f) => f, None => return };

    loop {
        拍 += 1;
        if plug.复位过 {
            plug.复位过 = false;
            if 计划.is_some() { println!("[服] 新的一集 —— 计划清空"); }
            计划 = None;
            试过.clear();
        }
        let Some(e) = 帧.ee.first().copied() else {
            match plug.sense() { Some(f) => { 帧 = f; continue } None => return }
        };
        let here = [e[0], e[1], e[2]];
        let q此 = [e[3], e[4], e[5], e[6]];
        let 回声 = Cmd::Ee { arm: 0, at: here, quat: q此, jaw: 帧.jaw.first().copied().unwrap_or(1.0) };

        // 缺格 ⇒ 永远回声。诚实失败,不瞎动。
        if !缺.is_empty() {
            plug.act(&回声);
            match plug.sense() { Some(f) => { 帧 = f; continue } None => return }
        }
        let (张开, 可达带, 工具长, 工具列, 接触阈, 原位, 交付率, 探幅) = (
            张开.unwrap(), 可达带.clone().unwrap(), 工具长.unwrap(), 工具列.unwrap(),
            接触阈.unwrap(), 原位.clone().unwrap(), 交付率.unwrap(), 探幅.unwrap(),
        );

        let cmd = if let Some(p) = &mut 计划 {
            // ── 走航点 ────────────────────────────────────────────────
            let ax = task::列(&p.q, 工具列 % 3);
            // 法兰 = 指尖 − 工具轴 × 工具长(全是量出来的)。
            let 法兰 = |t: [f64; 3], 抬: f64| {
                [t[0] - ax[0] * 工具长, t[1] - ax[1] * 工具长, t[2] - ax[2] * 工具长 + 抬]
            };
            // 🔴 钳口哪一头是【开】,是量出来的:跨度那一格的估计器只在
            // 「间距随命令变大」时才收下(slope<0 ⇒ Inconsistent)⇒ 跨度量到了,
            // 就等于量到了"命令 1.0 = 张开"。合 = 0.0。零个新数。
            let (开, 合) = (1.0f64, 0.0f64);
            let (目标, mut jaw, 名) = match p.段 {
                // ── V5 伺服接近(最终版,owner 令 2026-08-21):xy 伺服 + z 下降同拍进行,
                //    目标由推进段每拍算好存在 伺服目标;jaw 常开。旧 悬停/对准/下探 三段合一。
                0 | 1 | 9 => {
                    let t = if p.伺服目标[0].is_finite() { p.伺服目标 } else { here };
                    (t, 开, "接近")
                }
                2 => (法兰(p.指尖, 0.0), 合, "合爪"),
                11 => {
                    // 计划被准入拒 ⇒ 回量到的原位重问眼(可见性恢复:拒绝多半因毒观测 ——
                    // 自遮挡/畸变云。这不是被禁的"换点后回原位迁就账本尺"拐棍:此刻【没有】
                    // 计划、没有尺在跑;有可行计划时照旧就地干活,一步家不回。)
                    let d回 = ((原位[0] - here[0]).powi(2) + (原位[1] - here[1]).powi(2) + (原位[2] - here[2]).powi(2)).sqrt();
                    if d回 < 0.33 * 张开 {
                        println!("[服] 已回量到的原位(差 {:.3} m)⇒ 重新问眼", d回);
                        p.段 = 12;
                    }
                    // 🔴 N57 实测:回位路上也会僵(差 0.138→0.137 跨百拍,主差是 z 升不动)。
                    // 僵了不能弃回 段8(要回看 会立刻再生成回位计划 = 死循环),
                    // 而是【放弃回家、就地问眼】:走 段12 出口(清 要回看)。
                    // (此臂在 挪/卡阈 绑定之前 ⇒ 用 上位 自算;0.5mm/拍 = 既有协议地板
                    // "以下就是没在走,对任何机器人成立"。)
                    let 动11 = p.上位.map(|u| ((here[0] - u[0]).powi(2) + (here[1] - u[1]).powi(2) + (here[2] - u[2]).powi(2)).sqrt());
                    if 动11.map_or(false, |m| m < 0.0005) && d回 > 0.0025 {
                        p.僵拍 += 1;
                        if p.僵拍 >= 30 {
                            println!("[服] 回位路上僵住(连 {} 拍,还差 {:.3} m)⇒ 放弃回家,就地问眼", p.僵拍, d回);
                            p.僵拍 = 0; p.段 = 12;
                        }
                    } else { p.僵拍 = 0; }
                    ([原位[0], 原位[1], 原位[2]], 开, "回位重看")
                }
                8 => {
                    // 弃计划:【不回原位】(owner 死命令 2026-08-21:真机边移动边工作,
                    // 回原位迁就账本尺 = 拐棍)。手臂就地重来;尺跟人走靠在线局部手眼。
                    p.段 = 10;
                    (here, 开, "换点")
                }
                _ => {
                    // 抬:目标高度 = 指令要的位移 × 1.5(无量纲余量:交付有损耗,评分只看
                    // 到没到,过冲无害);同时往身体收半个可达半径(无量纲 0.5)——
                    // 下手点常在可达边界上,直上是满伸展姿态,收回来才抬得动(档案实测 54mm 卡死)。
                    let b = 法兰(p.指尖, p.抬高 * 1.5);
                    let mut d = [原位[0] - p.指尖[0], 原位[1] - p.指尖[1]];
                    let l = (d[0] * d[0] + d[1] * d[1]).sqrt().max(1e-9);
                    d = [d[0] / l, d[1] / l];
                    let 收 = 0.5 * 可达带[1];
                    ([b[0] + d[0] * 收, b[1] + d[1] * 收, b[2]], 合, "抬")
                }
            };
            let 差 = ((目标[0] - here[0]).powi(2) + (目标[1] - here[1]).powi(2) + (目标[2] - here[2]).powi(2)).sqrt();
            let 挪 = p.上位.map(|u| ((here[0] - u[0]).powi(2) + (here[1] - u[1]).powi(2) + (here[2] - u[2]).powi(2)).sqrt());
            let 实降 = p.上位.map(|u| (u[2] - here[2]).max(0.0)).unwrap_or(0.0);
            p.上位 = Some(here);
            // 到位判据:三分之一个钳口(无量纲 0.33;比它细的差距合爪自己吸收)。
            let 到 = 差 < 0.33 * 张开;
            // 手不跟:连 15 拍(协议数)挪不动 0.05 个探幅(无量纲比例)。
            // 🔴 卡判据阈要有物理地板(2026-08-21,M0 悬停死锁 500 拍):探幅是接触相的
            // 微小量(0.1mm 级),0.05×探幅只有几微米 —— 仿真抖动就把卡计数清零,
            // "卡 15 推进"永远不响。0.5mm/拍以下 = 没在走,对任何机器人成立(协议数)。
            let 卡阈 = (0.05 * 探幅).max(0.0005);
            if let Some(m) = 挪 { if m < 卡阈 && 差 > 2.0 * 探幅 { p.卡 += 1 } else if m >= 卡阈 { p.卡 = 0 } }
            if 拍 % 25 == 1 {
                println!("[服] {}:差 {:.3} m · 手 ({:.3},{:.3},{:.3}) · 钳口指令 {:.1}", 名, 差, here[0], here[1], here[2], jaw);
            }
            match p.段 {
                0 | 1 | 9 => {
                    // ── V5 伺服接近:每拍一步。爪像素 = 低频问眼校准 + 拍间航位推算。
                    p.段拍 += 1;
                    if p.对准等 > 0 { p.对准等 -= 1; }
                    // 每拍:实到位移(proprio 回声)入账 —— 推算 + 在线局部手眼共用。
                    let 本拍挪 = p.伺服上位.map(|u| (here[0] - u[0], here[1] - u[1], here[2] - u[2]));
                    p.伺服上位 = Some(here);
                    if let Some((ex, ey, ez)) = 本拍挪 { p.对Δw[0] += ex; p.对Δw[1] += ey; p.对Δw[2] += ez; }
                    // 🔴 z 列必须进补偿(N15-17 三炮同形定案 + 档案 8-15 §五第一行原文:
                    // "dv/dz=14.55 vs dv/dy=0.21,差 68 倍,而走位只命令 x/y、z 是自己沉
                    // 的" —— V5 重写把这条又丢了)。每拍下降给爪估/近答/对子注入没补偿
                    // 的竖直漂移,在线尺的 v 行就是被它喂坏的。z 列用账本(6 维里的后两
                    // 维,开机量的),在线只拟 xy 的 2×2。
                    let jz = body.get(Quantity::ImageJacobian).filter(|m| m.dim >= 6)
                        .map(|m| [m.value[4], m.value[5]]).unwrap_or([0.0, 0.0]);
                    // 🔴 z 列在线自尺(N40 实锤:账本 jz 把"命动"虚估 ~3×,响应自证被逼成
                    // 每 ~12 拍误杀一次好跟踪。对子认识论延到 z:z 主导拍配 (执行z, 像素挪),
                    // 过原点最小二乘 ≥6 对接管账本列;不随重锚清 —— 这是相机的物理尺,不是这门课的。)
                    let jz用 = if z对.len() >= 6 {
                        // 🔴 与 xy 岭同配方(N47 定案:裸最小二乘被钉死追踪器的零对拉到 0,
                        // 预测不下行 ⇒ 更钉死,自增强塌缩;跨计划持有把毒也持有)。
                        // 先验重 = 对子自身平均功率(自缩放,零新常数);剔群 3×中位残差。
                        let 拟z = |跳: &[bool]| -> [f64; 2] {
                            let lam = {
                                let m: f64 = z对.iter().map(|(z, _, _)| z * z).sum::<f64>() / (z对.len() as f64);
                                m.max(1e-9)
                            };
                            let (mut g, mut bu, mut bv) = (lam, lam * jz[0], lam * jz[1]);
                            for (i, (z, u, v)) in z对.iter().enumerate() {
                                if 跳.get(i).copied().unwrap_or(false) { continue; }
                                g += z * z; bu += z * u; bv += z * v;
                            }
                            [bu / g, bv / g]
                        };
                        let m0 = 拟z(&[]);
                        let mut r: Vec<f64> = z对.iter().map(|(z, u, v)| {
                            let (eu, ev) = (u - m0[0] * z, v - m0[1] * z);
                            (eu * eu + ev * ev).sqrt()
                        }).collect();
                        let mut rs = r.clone();
                        rs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
                        let med = rs[rs.len() / 2];
                        if med > 0.0 {
                            let 跳: Vec<bool> = r.drain(..).map(|x| x > 3.0 * med).collect();
                            if 跳.iter().any(|&b| b) { 拟z(&跳) } else { m0 }
                        } else { m0 }
                    } else { jz };
                    // ── 在线局部手眼(owner 死命令:不许回原位迁就账本 —— 尺跟人走)──
                    // 对子 = (两次生实测之间手真挪的 xy, 爪像素真挪),全是白拿的实测。
                    // 岭回归拉向账本 J:先验重 = 一步²(量出的步长),数据一多就接管;
                    // 病态时自动退向账本。档案移动线同型(在线量方向,18/18 含接线反接)。
                    let j_use: Option<([f64; 4], [f64; 4])> = body
                        .get(Quantity::ImageJacobian)
                        .filter(|m| m.dim >= 4)
                        .map(|m| {
                            let jl = [m.value[0], m.value[1], m.value[2], m.value[3]];
                            let js = [m.uncertainty[0], m.uncertainty[1], m.uncertainty[2], m.uncertainty[3]];
                            if 对子.len() >= 4 {
                                let s = (0.03 * 可达带[1]).max(探幅);
                                // 岭权重自缩放(N13 仪表定案:对子几乎全在一根轴上时,弱轴
                                // 回归把噪声放大成假增益 —— du/dy 被吹到 8.01(账本 0.96)且
                                // 钉死,伺服照它解步必歪。先验重 = 对子自己的平均功率 ⇒
                                // 弱轴自动留在账本,强轴 n 个一致对子 n:1 压过先验。零拍数)。
                                let lam = {
                                    let m: f64 = 对子.iter().map(|(w, _)| w[0] * w[0] + w[1] * w[1]).sum::<f64>()
                                        / (对子.len() as f64);
                                    m.max(s * s)
                                };
                                // 拟一把:岭回归拉向账本(对子集合可指定跳过哪些)。
                                let 拟 = |跳: &[bool]| -> [f64; 4] {
                                    let (mut g00, mut g01, mut g11) = (lam, 0.0, lam);
                                    let (mut b00, mut b01, mut b10, mut b11) =
                                        (lam * jl[0], lam * jl[2], lam * jl[1], lam * jl[3]);
                                    for (i, (w, px)) in 对子.iter().enumerate() {
                                        if 跳.get(i).copied().unwrap_or(false) { continue; }
                                        g00 += w[0] * w[0]; g01 += w[0] * w[1]; g11 += w[1] * w[1];
                                        b00 += px.0 * w[0]; b01 += px.0 * w[1];
                                        b10 += px.1 * w[0]; b11 += px.1 * w[1];
                                    }
                                    let det = g00 * g11 - g01 * g01; // lam>0 ⇒ 恒正
                                    let (i00, i01, i11) = (g11 / det, -g01 / det, g00 / det);
                                    [b00 * i00 + b01 * i01, b10 * i00 + b11 * i01,
                                     b00 * i01 + b01 * i11, b10 * i01 + b11 * i11]
                                };
                                // 剔离群(N9 实锤:作废重捕获后 VLM 偶指臂身,一个幻影对子
                                // 把当地尺转歪 ⇒ 逼近→丢失循环)。SPANX14 同配方:按残差
                                // 3×中位剔一轮再拟 —— 倍数是仓里已验的协议数,零新常数。
                                let m0 = 拟(&[]);
                                let mut r: Vec<f64> = 对子.iter().map(|(w, px)| {
                                    let (eu, ev) = (px.0 - m0[0] * w[0] - m0[2] * w[1],
                                                    px.1 - m0[1] * w[0] - m0[3] * w[1]);
                                    (eu * eu + ev * ev).sqrt()
                                }).collect();
                                let mut rs = r.clone();
                                rs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
                                let med = rs[rs.len() / 2];
                                let m1 = if med > 0.0 {
                                    let 跳: Vec<bool> = r.drain(..).map(|x| x > 3.0 * med).collect();
                                    if 跳.iter().any(|&b| b) { 拟(&跳) } else { m0 }
                                } else { m0 };
                                // 🔴 两行都交给数据(N44 定案,推翻 N18 的"v 行冻结用账本"):
                                // N18 冻 v 是因为 v 激励差时拟合行列式翻号;而 N13 的岭权重
                                // 自缩放本身就把弱激励轴钉在先验(账本)上 —— N18 怕的病 N13
                                // 已经机制化防住。N44 实测:桌面滑行 y 扫 ±0.25m,v 激励很足,
                                // 【账本 v 行才是错的】—— v 单调跑飞 0.517→0.219,双拍翻号因
                                // 每拍 v 动量 <0.005 永远武装不起来,u 修好也被 v 拖歪到僵。
                                (m1, js)
                            } else {
                                (jl, js)
                            }
                        });
                    // 尺可视化(N48 缺口:u 反向跑 125 拍无从判"拟合错"还是"解步错"——矩阵从没打印过)。
                    if 拍 % 25 == 1 && !p.模板.is_empty() {
                        if let Some((jj, _)) = j_use {
                            println!("[服]   [尺] J=[{:+.2},{:+.2};{:+.2},{:+.2}] jz用=[{:+.2},{:+.2}] 对子={} z对={} 号=[{:+.0},{:+.0}]",
                                jj[0], jj[2], jj[1], jj[3], jz用[0], jz用[1], 对子.len(), z对.len(), p.伺服号[0], p.伺服号[1]);
                        }
                    }
                    // 深度工具:候选凸起 + 中心深度(判影 + 物同一性,全量出)。
                    let 凸起 = |uu: f64, vv: f64| -> Option<(f64, f64)> {
                        let 路 = plug.lay.cams.get(p.相机)?;
                        let mut 深路 = 路.clone();
                        if let Some(last) = 深路.last_mut() { *last = "depth".to_string(); }
                        let dv = plug.last.as_ref().and_then(|o| 取(o, &深路))?;
                        let (dw, dh, dep) = wire::as_f32_grid(&dv)?;
                        let (x, y) = (((uu * dw as f64) as usize).min(dw - 1), ((vv * dh as f64) as usize).min(dh - 1));
                        let c = f64::from(*dep.get(y * dw + x)?);
                        let r = (dw / 40).max(2) as i64;
                        let mut ring = Vec::new();
                        for (dx, dy) in [(r,0),(-r,0),(0,r),(0,-r),(r,r),(r,-r),(-r,r),(-r,-r)] {
                            let (nx, ny) = (x as i64 + dx, y as i64 + dy);
                            if nx >= 0 && ny >= 0 && (nx as usize) < dw && (ny as usize) < dh {
                                ring.push(f64::from(dep[ny as usize * dw + nx as usize]));
                            }
                        }
                        if ring.is_empty() { return None }
                        ring.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
                        // 桌面在环的【远】侧 ⇒ 取远侧分位当"那里的桌面",凸起 = 比它近多少。
                        // 环中位是错的(N4 实锤:手降低后在画面里变大,环落在手自己身上 ⇒
                        // 中位=手深 ⇒ 凸起≈0 ⇒ 真爪连片被判"像影",而同一片像素高处刚被收过)。
                        // 影 = 与远侧同深(平贴支撑面);爪/物 = 比远侧近。零新常数。
                        Some((((ring[(ring.len() * 3) / 4] - c).max(0.0)), c))
                    };
                    let 物凸 = 凸起(p.物像素.0, p.物像素.1);
                    // 低频问眼校准(节流:量出的静置拍;VLM ~1s 一答,不堵环)。
                    // 🔴 末段(爪估离物 < 2×物跨)停问 VLM(N34 终线定案:追踪器正贴在物上,
                    // VLM 答到腕上,离 0.14 触发争议、z 底争议=换点 ⇒ 7 个计划全死在终点线;
                    // 档案原话"爪贴物后 VLM 只能证到 ±0.05-0.09,多等的每一答都可能框到手腕
                    // 被劫持"。末段的裁判本来就是合爪自检+空爪重试,不是 VLM)。
                    let 离物近 = p.预测爪.map(|(u, v)| {
                        ((u - p.物像素.0).powi(2) + (v - p.物像素.1).powi(2)).sqrt() < 2.0 * p.物跨.max(0.05)
                    }).unwrap_or(false);
                    if p.对准等 == 0 && p.晃态 == 0 && !p.模板.is_empty() && p.验拍 >= 40 && !离物近 {
                        // 🔴 N51 帧+尺双证:白臂上模板会滑(爪已物理贴方块,模板滑到前臂
                        // (0.787,0.425),伺服被假位置拖着绕圈,垃圾对子把拟合搅到符号乱翻)。
                        // 身份到期不再问 VLM(它近场答手腕,z 底复核裂是 N34-N51 惯性死法)——
                        // 改晃爪重锚:构造性身份,便宜(~15 拍)、不会认错。模板只准活 40 拍。
                        println!("[服] 40 拍身份到期 ⇒ 晃爪重锚(白臂模板会滑,构造性身份代替 VLM 复核)");
                        // 🔴 认位只记【活满租约】的锚(N65 帧证:出生即记会被臂/光斑假锚投毒 ——
                        // (0.735,0.369) 假锚活不过几拍,却把"指不可见"的位姿写进 认位,回去晃全败
                        // diff 10-21)。活满 40 拍 = 响应自证+没饿死,这里才是"爪确实在这可见"的证词。
                        认位 = Some([here[0], here[1], here[2]]);
                        p.锚新 = false;
                        p.晃态 = 1; p.晃等 = 0; p.验拍 = 0;
                    }
                    #[allow(clippy::never_loop)]
                    while false {
                        p.对准等 = body_layer::derive::settle_periods(&body, 1e-3).map(|n| n as u32).unwrap_or(8).max(4);
                        let 眼爪 = match plug.lay.cams.get(p.相机)
                            .and_then(|路| plug.last.as_ref().map(|o| (路.clone(), o.clone())))
                            .and_then(|(路, o)| 取(&o, &路))
                            .and_then(|v| wire::as_rgb(&v).map(|(w2, h2, rgb)| (w2, h2, rgb))) {
                            None => { if 拍 % 25 == 1 { println!("[服]   问眼取图失败(相机 {} 路径/格式)", p.相机); } None }
                            Some((w2, h2, rgb)) => {
                                // 裁剪问眼(MT 探针实锤:全幅问,VLM 每次把画面中央最显眼的
                                // 方块当"爪"指,被同一性闸全拦 ⇒ 永远没有爪源)。以账本
                                // hand_pixel(爪的标定像素,量出的)为中心裁半幅 —— 物在
                                // 中央被裁掉,VLM 想指错都看不见它。答案坐标映射回全幅。
                                // 裁剪中心 = 账本推算的爪当前像素:hand_pixel(标定位)+
                                // J·(here − 原位)(MX 实锤:死用标定位,悬停姿态爪不在窗里,
                                // 8 答全被闸拦 ⇒ 无爪源。全账本推算,零新常数)。
                                // 窗心跟爪走(N0 实锤:J 外推在伺服姿态把窗推到画面右缘,
                                // VLM 只能在窗内指臂身,差 0.47 钉死)。有爪估 ⇒ 窗心=爪估
                                // (自跟踪);首捕获 ⇒ 标定位 + 窗放大(2/3 幅容住姿态差)。
                                let hp = p.预测爪.or(上帧爪).unwrap_or_else(|| {
                                    body.get(Quantity::HandPixel).filter(|m| m.dim >= 2)
                                        .map(|m| (m.value[0], m.value[1])).unwrap_or((0.5, 0.5))
                                });
                                // 首捕获隔次交替全幅(N14 实锤:旧爪锚 + 避物切割可把真爪
                                // 正好切出窗外 ⇒ 350+ 拍零爪源。奇偶交替 = 两个假设轮流
                                // 覆盖:锚窗(爪没走远)/ 全幅−物侧(爪在意外处),零计数器)。
                                let 全幅 = p.预测爪.is_none() && (p.段拍 / 8) % 2 == 1;
                                let (mut cw, mut ch) = if 全幅 { (w2, h2) }
                                    else if p.预测爪.is_some() { (w2 / 2, h2 / 2) } else { (w2 * 2 / 3, h2 * 2 / 3) };
                                let mut x0 = (((hp.0 * w2 as f64) as usize).saturating_sub(cw / 2)).min(w2 - cw);
                                let mut y0 = (((hp.1 * h2 as f64) as usize).saturating_sub(ch / 2)).min(h2 - ch);
                                // 窗里必须没有【问过的物】(N2 落图定案 2026-08-21:首捕获窗
                                // 2/3 幅夹紧后方块仍在窗内 —— VLM 11/11 指物、被同一性闸拒
                                // ⇒ 永远没爪源;裁剪问眼的全部理由就是"物被裁掉")。沿爪估↔物
                                // 分得最开的那根轴,把窗在物那一侧的边切到「物 ± 物自报跨度」;
                                // 两轴都分不开(末段爪已贴物)⇒ 不切,交给深度双闸。余隙自缩放,
                                // 用的只有爪估/物像素/物跨 —— 零机体假设,5/20 指手同样成立。
                                {
                                    let (ou, ov) = p.物像素;
                                    let m = p.物跨.max(0.05);
                                    let (du, dv) = (hp.0 - ou, hp.1 - ov);
                                    if du.abs() >= dv.abs() && du.abs() > m {
                                        if du > 0.0 {
                                            let nx = (((ou + m) * w2 as f64) as usize).min(w2 - 1);
                                            if nx > x0 && nx < x0 + cw { cw -= nx - x0; x0 = nx; }
                                        } else {
                                            let nx = (((ou - m) * w2 as f64) as usize).max(1);
                                            if nx > x0 && nx < x0 + cw { cw = nx - x0; }
                                        }
                                    } else if dv.abs() > m {
                                        if dv > 0.0 {
                                            let ny = (((ov + m) * h2 as f64) as usize).min(h2 - 1);
                                            if ny > y0 && ny < y0 + ch { ch -= ny - y0; y0 = ny; }
                                        } else {
                                            let ny = (((ov - m) * h2 as f64) as usize).max(1);
                                            if ny > y0 && ny < y0 + ch { ch = ny - y0; }
                                        }
                                    }
                                    if 拍 % 25 == 1 {
                                        println!("[服]   避物窗:物({:.2},{:.2}) 爪估({:.2},{:.2}) ⇒ 窗 [{}..{})×[{}..{})", ou, ov, hp.0, hp.1, x0, x0 + cw, y0, y0 + ch);
                                    }
                                }
                                let mut sub = Vec::with_capacity(cw * ch * 3);
                                for yy in y0..y0 + ch {
                                    let row = (yy * w2 + x0) * 3;
                                    sub.extend_from_slice(&rgb[row..row + cw * 3]);
                                }
                                match body_layer::eye::ask(眼主机, 眼端口, "the robot gripper", &sub, cw, ch) {
                                    Err(e) => { if 拍 % 25 == 1 { println!("[服]   问眼失败:{e}"); } None }
                                    Ok(mut l) => {
                                        l.u = (x0 as f64 + l.u * cw as f64) / w2 as f64;
                                        l.v = (y0 as f64 + l.v * ch as f64) / h2 as f64;
                                        Some(l)
                                    }
                                }
                            }
                        };
                        if let Some(look) = 眼爪 {
                            // 双闸重定义(N3 落图定案 2026-08-21:末段爪降到与物同高,
                            // 「深度差 ≥ 半物凸」那一半把【真爪】也全拒了 ⇒ 首校准后 0 次
                            // 实测,推算冻在"差 0.013"而真爪离物还有 0.14 画幅)。
                            // 「是不是物」判像素距离(物自报跨度当半径)—— 同深不等于同物;
                            // 「是不是影」仍判凸起(影平贴支撑面,凸起≈0,MO 实锤)。
                            let 离物 = ((look.u - p.物像素.0).powi(2) + (look.v - p.物像素.1).powi(2)).sqrt();
                            let 非物 = 离物 >= p.物跨.max(0.05);
                            let 过 = 非物 && match (凸起(look.u, look.v), 物凸) {
                                (Some((k, _)), Some((wv, _))) if wv > 0.0 => k >= 0.5 * wv,
                                _ => true,
                            };
                            if 过 {
                                if !p.模板.is_empty() {
                                    // 跟段:慢眼只做【身份复核】,不掺位置(它抖 ±0.03-0.05;
                                    // 位置归快眼)。复核失败 = 快眼可能跟丢/跟错 ⇒ 重锚。
                                    let (tu, tv) = p.预测爪.unwrap_or((look.u, look.v));
                                    let dd = ((look.u - tu).powi(2) + (look.v - tv).powi(2)).sqrt();
                                    if dd > 2.0 * p.物跨.max(0.05) {
                                        // 裁判是构造不是 VLM(N21 帧证:爪被挡时 VLM 指手肘/底座,
                                        // 信它重锚 = 把跟踪劫持到臂身上)。争议 ⇒ 晃爪重认;已在
                                        // z 底(爪贴物,晃爪有碰物风险)⇒ 直接换下手点。
                                        p.收敛计 = 0;
                                        if p.连塌 >= 10 {
                                            println!("[服]   身份复核失败(慢眼 ({:.3},{:.3}) vs 快眼 ({:.3},{:.3}) 离 {:.3})且已在 z 底 ⇒ 换下手点", look.u, look.v, tu, tv, dd);
                                            p.段 = 8;
                                        } else {
                                            println!("[服]   身份复核失败(慢眼 ({:.3},{:.3}) vs 快眼 ({:.3},{:.3}) 离 {:.3})⇒ 晃爪重认", look.u, look.v, tu, tv, dd);
                                            p.晃态 = 1; p.晃等 = 0;
                                        }
                                    } else {
                                        p.验拍 = 0;
                                        println!("[服]   身份复核过(离 {:.3})", dd);
                                    }
                                }
                            } else if 拍 % 25 == 1 {
                                println!("[服]   问眼答 ({:.3},{:.3}) 拒:{}(离物 {:.3})⇒ 弃", look.u, look.v,
                                    if !非物 { "指的是物" } else { "凸起像影" }, 离物);
                            }
                        }
                    }
                    // 每拍:按当前爪估计走一步(实测或推算)。
                    // 推算 = 账本 J ×【实到位移】(proprio 回声),不是命令的比例
                    // (N5 实锤:J 尺度偏小 ⇒ 限幅比 k=1 ⇒ 预测一拍瞬移到物上,
                    //  爪估差 0.0023 假收敛、最新实测差 0.1777、手臂高空怠速 ——
                    //  08-15 小脑那条"累加命令会漂,要用实到"的像素版退化)。
                    if let (Some((pu0, pv0)), Some((ex, ey, ez)), Some((j, _))) = (p.预测爪, 本拍挪, j_use) {
                        let (su, sv) = (j[0] * ex + j[2] * ey + jz用[0] * ez,
                                        j[1] * ex + j[3] * ey + jz用[1] * ez);
                        p.预测爪 = Some((pu0 + su, pv0 + sv));
                        p.命累 += (su * su + sv * sv).sqrt();
                        // 近答同步平移(位移补偿含 z:旧答案挪到当前时刻,中位才不带滞后)。
                        for a in p.近答.iter_mut() { a.0 += su; a.1 += sv; }
                    }
                    // ── 晃爪认手(构造性身份,档案两次验通:「只命令钳口、手臂不动 ⇒
                    //    会动的刚体按构造就是钳口」)。臂冻住,只动指通道(标量 jaw = 全
                    //    通道广播,2/5/20 指同一段码)。静对 A/B(全合、各自等静后拍)定
                    //    逐像素噪声地板;动帧 C(张开、等静)− B = 只有爪在变 ⇒ 加权形心
                    //    = 爪像素。等静用量出的 settle(帧间等静,钳口交付延迟 ~2 拍)。──
                    if p.回家中 {
                        // 爪不在画面里(N28 帧证:臂整个出画,晃爪 m1 只有 ~20 像素)⇒
                        // 去【量到的原位】重认。原位 = 标定认手锚定处 = 已证爪可见的位姿;
                        // 每计划最多一次 —— 这是"首捕获的可见性恢复",不是被禁的
                        // "换点后回原位重来"那根拐棍(那条禁的是拿回位迁就账本尺)。
                        let 回点 = 认位.unwrap_or([原位[0], 原位[1], 原位[2]]);
                        p.伺服目标 = [回点[0], 回点[1], 回点[2]];
                        let d回 = ((回点[0] - here[0]).powi(2) + (回点[1] - here[1]).powi(2) + (回点[2] - here[2]).powi(2)).sqrt();
                        if d回 < 0.33 * 张开 {
                            println!("[服]   已到重认点(差 {:.3} m,{})⇒ 重新晃爪认手", d回, if 认位.is_some() {"上次认出处"} else {"原位兜底"});
                            p.回家中 = false; p.晃态 = 1; p.晃等 = 0; p.晃次 = 0;
                        } else if 拍 % 25 == 1 {
                            println!("[服]   去重认点:还差 {:.3} m", d回);
                        }
                        // 回家路上也会僵(N54 实测:差 0.281→0.282 跨百拍不动,等 900 拍太贵)。
                        if 挪.map_or(false, |m| m < 卡阈) && d回 > 5.0 * 卡阈 {
                            p.僵拍 += 1;
                            if p.僵拍 >= 30 {
                                println!("[服] 回原位路上僵住(连 {} 拍)⇒ 弃这个计划换下手点", p.僵拍);
                                p.僵拍 = 0; p.段 = 8;
                            }
                        } else { p.僵拍 = 0; }
                    } else if p.晃态 > 0 {
                        // 🔴 晃中不许追漂(N63 定案:每拍 目标=here 让目标追着漂移走,手永远
                        // 静不下来 —— 900 拍烧在等静死循环零打印。晃入时的目标冻住不再改)。
                        let 静置 = body_layer::derive::settle_periods(&body, 1e-3).map(|n| n as u32).unwrap_or(8).max(4);
                        // 认块器的 5 帧合同(标定认手同款,不重写):静对 null_a/null_b(无命令)
                        // + f0/f1/f2(相邻两帧之间各一步同长的钳口命令)。帧间等静(钳口交付
                        // 延迟 ~2 拍,连拍会打乱"静/动"合同 —— M9 实锤)。
                        let mut 拍帧 = |p: &mut 抓况, 下态: u8, 下等: u32| {
                            if p.晃等 > 0 { p.晃等 -= 1; }
                            if p.晃等 == 0 {
                                if let Some(img) = 帧.cams.get(p.相机) { p.晃帧.push(img.2.clone()); }
                                p.晃态 = 下态; p.晃等 = 下等;
                            }
                        };
                        // 🔴 先合到底、往【开】晃(08-15 铁律:往合晃会撞机械底 + 两指并成
                        // 一团折不出对 —— N30 实测配对恒 0)。步长 = 标定认手的同一 pub 常数
                        // 钳口窗宽(0.45),整套合同与标定逐字同款。
                        match p.晃态 {
                            1 => {
                                jaw = 合;
                                // 🔴 动静门控静置(N62 定案:大位移后 8 拍死等不够 —— 两轮晃认
                                // 地板 244/173 爆表 = "静止对"被臂的余振污染,双响 0 全轮作废还
                                // 吃掉 晃次 预算。量出来的静才算静:开拍前须连续 2 拍 |挪|<卡阈。)
                                // 🔴 等静要有预算(N63 定案:死等无预算 ⇒ 整段 900 拍烧光零打印。
                                // 预算 = 10×静置拍;烧完记一次晃败推进递增梯子,三败弃计划换下手点。)
                                let 静了 = 挪.map_or(false, |m| m < 卡阈);
                                if !静了 {
                                    p.僵拍 += 1;
                                    if p.僵拍 >= 10 * 静置 {
                                        println!("[服]   晃认:等静 {} 拍未静 ⇒ 记一次晃败,递增重试", p.僵拍);
                                        p.僵拍 = 0; p.晃帧.clear(); p.晃等 = 0; p.晃次 += 1;
                                        if p.晃次 >= 3 {
                                            println!("[服]   等静三败 ⇒ 弃这个计划换下手点");
                                            p.晃次 = 0; p.晃态 = 0; p.段 = 8;
                                        }
                                    }
                                }
                                else if p.晃等 == 0 && p.晃帧.is_empty() { p.僵拍 = 0; p.晃等 = 静置; } else { 拍帧(p, 2, 2); }
                            }
                            2 => { jaw = 合; 拍帧(p, 3, 1); }
                            3 => { jaw = 合; 拍帧(p, 4, 静置); }
                            // 🔴 晃次递增激励(N60 定案:失败晃认全是"双响 0-1,m1/m2 十几像素"——
                            // 该姿态下定幅 0.45 窗宽的指动几乎不产生像素,原样再晃两轮自然还是瞎。
                            // 第 k 轮摆幅 ×(1+k),上限=量到的全开(合到开的一半×2 步)。零新常数。)
                            4 => {
                                let 幅 = (selfcal::钳口窗宽 * (1.0 + p.晃次 as f64)).min(0.5);
                                jaw = 合 + (开 - 合) * 幅; 拍帧(p, 5, 静置);
                            }
                            5 => {
                                let 幅 = (selfcal::钳口窗宽 * (1.0 + p.晃次 as f64)).min(0.5);
                                jaw = 合 + (开 - 合) * 2.0 * 幅; 拍帧(p, 6, 静置);
                            }
                            _ => {
                                jaw = 开; // 回开(等静后结算,模板在张开态截 —— 跟踪时爪就是张开的)
                                if p.晃等 > 0 { p.晃等 -= 1; }
                                if p.晃等 == 0 {
                                    let mut 锚好 = false;
                                    if p.晃帧.len() == 5 {
                                        if let Some(img) = 帧.cams.get(p.相机) {
                                            match body_layer::blob::candidates(
                                                &p.晃帧[0], &p.晃帧[1], &p.晃帧[2], &p.晃帧[3], &p.晃帧[4],
                                                img.0, img.1, (开 - 合) * (selfcal::钳口窗宽 * (1.0 + p.晃次 as f64)).min(0.5), selfcal::最少像素(img.0, img.1),
                                            ) {
                                                Ok(r) => {
                                                    println!("[服]   晃认:双响 {} · 配对 {} · 地板 {} · m1 {} m2 {}",
                                                        r.moved_px, r.pairs, r.floor, r.m1_px, r.m2_px);
                                                    // 取证帧(N63:配对连 0 到最大幅 —— 两指是前后叠死还是被光斑吃
                                                    // 掉,只有图能分)。最坏轮才落盘,不刷盘。
                                                    if r.pairs == 0 && p.晃次 >= 2 {
                                                        if let Ok(d) = std::env::var("BL_DUMP") {
                                                            for (k, tag) in [(0usize, "零"), (4usize, "开")] {
                                                                if let Some(fr) = p.晃帧.get(k) {
                                                                    let mut buf = format!("P5\n{} {}\n255\n", img.0, img.1).into_bytes();
                                                                    buf.extend_from_slice(fr);
                                                                    let _ = std::fs::write(format!("{d}/晃{合序}_{tag}.pgm"), buf);
                                                                }
                                                            }
                                                            合序 += 1;
                                                        }
                                                    }
                                                    // 🔴 配对是奢侈品,不是锚定的前提(N32 实测:竖腕朝下时两指
                                                    // 端对相机前后叠死,配对恒 0 —— 档案原话"从这个角度两根本来
                                                    // 就分不开";而【臂冻住只动钳口】的构造保证:任何双响块都是
                                                    // 爪。有对用对的中点(更准),没对用最大双响块的形心。
                                                    if let Some(c) = r.cands.get(0) {
                                                        // 🔴 指尖端锚(N45 定案:配对=0 时旧锚=双响形心,在指身中段 ——
                                                        // 收敛把形心怼上物心,指尖越过物体,合爪咬边挤出;空合同签名
                                                        // N35×3 + N40二集 + N45)。改:锚取包围盒沿 jz 图像方向(下降=
                                                        // 朝指尖;方向 image_jacobian 探针直接量)那条边,主轴取端、
                                                        // 侧轴留形心平均噪声。配对≥1 时对中点已是指尖级,不动。
                                                        let (mut au, mut av) = (c.u, c.v);
                                                        let jn = (jz[0] * jz[0] + jz[1] * jz[1]).sqrt();
                                                        if r.pairs == 0 && jn > 1e-9 {
                                                            let (du_, dv_) = (jz[0] / jn, jz[1] / jn);
                                                            if dv_.abs() >= du_.abs() {
                                                                av = if dv_ > 0.0 { c.ext[3] } else { c.ext[1] };
                                                            } else {
                                                                au = if du_ > 0.0 { c.ext[2] } else { c.ext[0] };
                                                            }
                                                            println!("[服]   指尖端锚:形心 ({:.3},{:.3}) → 端 ({:.3},{:.3})", c.u, c.v, au, av);
                                                        }
                                                        let half = ((img.0 as f64) * 0.03) as usize;
                                                        if let Some(tp) = 截块(img.0, img.1, &img.2, au, av, half) {
                                                            p.模板 = tp; p.模板半 = half; p.验拍 = 0;
                                                            p.预测爪 = Some((au, av)); p.预测龄 = 0;
                                                            上帧爪 = Some((au, av));
                                                            p.上fix生 = Some((au, av)); p.对Δw = [0.0, 0.0, 0.0];
                                                            // 🔴 重锚 = 重开一门课:上一段跟错/漂移攒的对子与翻号是毒
                                                            // (N33 实测:重锚不清尺 ⇒ 新锚照旧毒尺继续反向漂)。
                                                            p.伺服号 = [1.0, 1.0]; p.翻候 = [0, 0]; p.命累 = 0.0; p.应累 = 0.0;
                                                            锚好 = true;
                                                            println!("[服]   晃爪认手:爪 ({:.3},{:.3}) ⇒ 快眼开跟(半 {} px)", au, av, half);
                                                        }
                                                    }
                                                }
                                                Err(e) => { println!("[服]   晃认:认块器拒绝 {e:?}"); }
                                            }
                                        }
                                    }
                                    p.晃帧.clear();
                                    if 锚好 { p.晃态 = 0; p.晃次 = 0; }
                                    else {
                                        p.晃次 += 1;
                                        if p.晃次 >= 3 {
                                            // 🔴 N55 定案:开局差 0.0566 的黄金位被"3 败⇒回家"整套重启扔掉。
                                            // 回家留给【真失明】(本计划从没锚上过);已有锚的 3 败多半是贴桌
                                            // 贴物时合同过不了 —— 就地按推算续跑,消耗底晃预算,租约到期再晃。
                                            if p.预测爪.is_some() && p.锚新 && p.底晃次 < 2 {
                                                p.底晃次 += 1;
                                                println!("[服]   晃 3 败但本计划有锚 ⇒ 按推算续跑(底晃 {}/2),租约到期再晃", p.底晃次);
                                                p.晃态 = 0; p.晃次 = 0; p.验拍 = 0; p.预测龄 = 0;
                                            } else if !p.回过家 {
                                                println!("[服]   晃爪 3 轮没认出 —— 按帧证判爪不在画面里 ⇒ 去量到的原位重认(本计划仅此一次)");
                                                p.回过家 = true; p.回家中 = true; p.晃态 = 0; p.晃次 = 0;
                                            } else {
                                                println!("[服]   回原位后仍 3 轮没认出 ⇒ 弃这个计划换下手点");
                                                p.段 = 8; p.晃态 = 0; p.晃次 = 0;
                                            }
                                        } else {
                                            println!("[服]   晃爪认手这轮没认出 ⇒ 再晃({}/3)", p.晃次);
                                            p.晃态 = 1; p.晃等 = 0;
                                        }
                                    }
                                }
                            }
                        }
                    } else if let Some((pu0t, pv0t)) = p.预测爪 {
                        // ── 快眼(档案原话:"the eye can only ever be the slow loop …
                        //    while a tracker(146.4 Hz)and the body layer close the fast
                        //    loop. That split is already the architecture")。模板只在慢眼
                        //    锚定时刷新(档案:自动刷新会把跟踪点走离手);相关度自评不可信
                        //    ⇒ 身份由慢眼定期复核。──
                        let (pu, pv) = if !p.模板.is_empty() {
                            if let Some(img) = 帧.cams.get(p.相机) {
                                let r = ((img.0 as f64) * 0.06) as usize;
                                if let Some((tu, tv)) = 找块(img.0, img.1, &img.2, &p.模板, p.模板半, pu0t, pv0t, r) {
                                    // 方向自证搬进快眼流(FV 铁律"单步信噪比<1 只能测一次
                                    // 定一次"判死的是 VLM 差分;模板匹配单步噪声 ~1px,合规)。
                                    if let Some((lu, lv)) = p.上fix生 {
                                        let 实 = (tu - lu, tv - lv);
                                        for a in 0..2 {
                                            // 拟合接管(≥4 对)后两轴自证都停:v 行已由数据
                                            // 定号,再翻 伺服号 会跟拟合打架(N44 起 v 行不再冻结)。
                                            if 对子.len() >= 4 { continue; }
                                            let (sg, e) = if a == 0 { (实.0, p.上期望.0) } else { (实.1, p.上期望.1) };
                                            if sg.abs() > 0.005 && e != 0.0 {
                                                if sg * e < 0.0 {
                                                    p.翻候[a] += 1;
                                                    if p.翻候[a] >= 2 {
                                                        p.伺服号[a] = -p.伺服号[a]; p.翻候[a] = 0;
                                                        println!("[服]   方向自证 {}:连续两次矛盾 ⇒ 翻 ⇒ {:+.0}", if a == 0 { "u" } else { "v" }, p.伺服号[a]);
                                                    }
                                                } else { p.翻候[a] = 0; }
                                            }
                                        }
                                        // 对子:逐拍(真挪 xy, 爪像素真挪),z 贡献用账本列扣掉。
                                        // 快眼流喂在线尺 —— 比 VLM 生答密两个量级、噪声小一个量级。
                                        let w_ = p.对Δw;
                                        if (w_[0] * w_[0] + w_[1] * w_[1]).sqrt() > 1e-4 {
                                            对子.push(([w_[0], w_[1]], (tu - lu - jz用[0] * w_[2], tv - lv - jz用[1] * w_[2])));
                                            if 对子.len() > 32 { 对子.remove(0); }
                                        }
                                        // z 对采集(N49 [尺]定案:原"xy<0.2×z"条件在 V5 同拍 xy+z 设计下
                                        // 结构性不可满足 ⇒ z对 恒 0,自尺器官从没收过一对。改:z 有执行就收,
                                        // xy 的像素贡献用当前拟合扣掉(对子=32 时拟合已可信,坐标下降式一致估计)。
                                        if w_[2].abs() > 1e-4 {
                                            if let Some((jf, _)) = j_use {
                                                z对.push((w_[2], 实.0 - jf[0] * w_[0] - jf[2] * w_[1],
                                                                 实.1 - jf[1] * w_[0] - jf[3] * w_[1]));
                                                if z对.len() > 24 { z对.remove(0); }
                                            }
                                        }
                                        // 🔴 响应自证(N36 实锤:匹配器锁死静止图块,每拍"成功"⇒预测龄
                                        // 永远 0,饿死闸闻不到;z 照降到底,首次复核才裁,两计划同签名死。
                                        // 晃爪认手同一认识论连续化:爪=被命令时共动的那块;命动攒过
                                        // 0.05 幅而应动不足其三分之一 ⇒ 这块已不是爪 ⇒ 当拍作废重锚。
                                        p.应累 += (实.0 * 实.0 + 实.1 * 实.1).sqrt();
                                        if p.命累 > 0.05 && p.应累 < p.命累 * 0.33 {
                                            println!("[服] 快眼失守:命动 {:.3} 应动 {:.3} ⇒ 模板作废,晃爪重认", p.命累, p.应累);
                                            p.锚新 = false;
                                            p.命累 = 0.0; p.应累 = 0.0; p.收敛计 = 0;
                                            // 🔴 N52 定案:白臂模板只活 10-30 拍,底部失守若一律死,任何计划都
                                            // 到不了合爪。底部重晃解禁(只动钳口;碰歪由空合自检+问前回位兜底),
                                            // 预算 2 次/计划,超了才换点。
                                            if p.连塌 >= 10 && p.底晃次 >= 2 { println!("[服]   z 底重晃预算用尽 ⇒ 弃这个计划换下手点"); p.段 = 8; }
                                            else { if p.连塌 >= 10 { p.底晃次 += 1; println!("[服]   z 底第 {} 次重晃(≤2)", p.底晃次); } p.晃态 = 1; p.晃等 = 0; }
                                        }
                                    }
                                    p.上fix生 = Some((tu, tv)); p.对Δw = [0.0, 0.0, 0.0];
                                    p.预测爪 = Some((tu, tv));
                                    p.预测龄 = 0;
                                    p.锚新 = true;
                                    (tu, tv)
                                } else { (pu0t, pv0t) }
                            } else { (pu0t, pv0t) }
                        } else { (pu0t, pv0t) };
                        let d = (p.物像素.0 - pu, p.物像素.1 - pv);
                        let 差px = (d.0 * d.0 + d.1 * d.1).sqrt();
                        p.上期望 = d;
                        if !p.模板.is_empty() {
                            // 跟段:快眼连续伺服(边走边看回来了 —— 眼是快的那只)。
                            let mut 挪xy = [0.0f64, 0.0];
                            if 差px > 0.015 {
                                if let Some((j, js)) = j_use {
                                    if let Ok(((dx0, dy0), _)) = probe::image_to_plane_damped(&j, &js, d) {
                                        let (dx, dy) = (dx0 * p.伺服号[0], dy0 * p.伺服号[1]);
                                        let l = (dx * dx + dy * dy).sqrt();
                                        let cap = 0.1 * 张开;
                                        let k = if l > cap { cap / l } else { 1.0 };
                                        挪xy = [dx * k, dy * k];
                                    }
                                }
                            }
                            let 步z = (0.03 * 可达带[1]).max(探幅);
                            // 🔴 N56 取证帧终判:差px 还有 0.3+ 时 z 已落底,爪贴桌在【方块的高度】
                            // 横扫 20-30cm 进场 —— 推土机(2026-06-10 MPPI 老病换装):方块被指侧
                            // 顶走,撑住=顶着推,合上必空(空合 7 连的真凶)。修:端带外(差px >
                            // 2×物跨,与 VLM 压制共用同一条带,零新常数)z 悬停不降 —— 先对准再下压。
                            let 降 = if p.连塌 >= 10 { 0.0 }
                                else if 差px > 2.0 * p.物跨.max(0.05) { 0.0 }
                                else if 差px > 0.05 { 0.5 * 步z } else { 步z };
                            if let Some((_, _, ez)) = 本拍挪 {
                                if 降 > 0.0 && ez > -0.1 * 步z { p.连塌 += 1; }
                                else if ez < -0.25 * 步z { p.连塌 = 0; }
                            }
                            p.伺服目标 = [here[0] + 挪xy[0], here[1] + 挪xy[1], here[2] - 降];
                            p.验拍 += 1;
                        } else {
                            // 看段:站住不动,攒慢眼答案。
                            p.伺服目标 = [here[0], here[1], here[2]];
                        }
                        if p.段起z == f64::MAX { p.段起z = here[2]; }
                        // (z 的到底计数移进跳段 —— 看段不动 z,老的 15 拍净降窗在这里只会假钉。)
                        if 拍 % 25 == 1 {
                            println!("[服] 接近:爪估 ({:.3},{:.3}) 物 ({:.3},{:.3}) 差 {:.4} · 手 ({:.3},{:.3},{:.3}) · z到底 {}",
                                pu, pv, p.物像素.0, p.物像素.1, 差px, here[0], here[1], here[2], p.连塌);
                        }
                        // 合爪判据:z 到底 且 画面收敛【连续 2 拍】(MV 实锤:爪估噪声
                        // 瞬间穿透单拍判据 —— 0.089 下一拍蹦 0.014 就合,空)。
                        // 只认【校准拍】的达标(预测龄==0 = 本拍刚被 VLM 实测校准;MW 实锤:
                        // 推算拍把预测爪朝物前推,差自动缩到 0.0004"收敛"——推算自我实现,
                        // 三连假收敛空爪)。连续两次独立实测 <0.015 才是真对齐。
                        // 收敛线 = max(合爪窗, 仪器自身散布)(驱动的容差铁律"不许要求比
                        // 自身重复性紧"用在这:N15/N16 实锤 —— 爪贴物后与物黏成一团,
                        // VLM 只能证到 ±0.05-0.09,0.015 在末段是永远够不着的门,而多等的
                        // 每一答都可能框到手腕被劫持。爪窗 80mm ≫ 方块 35mm,±3cm 合爪
                        // 兜得住;兜不住由空爪自检+换点接盘 —— 档案老链"末段交给几何与
                        // 接触"的同一形状。散布 = 近答 MAD(量出来的,零拍数)。
                        let 线 = if !p.模板.is_empty() { 0.015 } else if p.近答.len() >= 3 {
                            let med = |v: &mut Vec<f64>| {
                                v.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
                                v[v.len() / 2]
                            };
                            let mut us: Vec<f64> = p.近答.iter().map(|a| a.0).collect();
                            let mut vs: Vec<f64> = p.近答.iter().map(|a| a.1).collect();
                            let (mu, mv) = (med(&mut us), med(&mut vs));
                            let mut du: Vec<f64> = p.近答.iter().map(|a| (a.0 - mu).abs()).collect();
                            let mut dv: Vec<f64> = p.近答.iter().map(|a| (a.1 - mv).abs()).collect();
                            let (su, sv) = (med(&mut du), med(&mut dv));
                            (su * su + sv * sv).sqrt().max(0.015)
                        } else { 0.015 };
                        if p.连塌 >= 10 && 差px < 线 && p.预测龄 == 0 { p.收敛计 += 1; }
                        else if 差px >= 2.0 * 线 { p.收敛计 = 0; } // 滞回随线走(原 0.015/0.03 的同一比例)
                        // 🔴 僵拍(N43 定案:上一刀直接读 卡≥30,而 卡 的武装线 差>2×探幅
                        // (探幅 0.1mm 级)被【晃爪保持位】的微米噪声武装 ⇒ 30 拍杀死健康晃认,
                        // 计划九连同点循环 —— 手臂从没楔死,凶手是那把闸自己。真僵 = 伺服在要
                        // 一步(差>5×卡阈)· 手不动(挪<卡阈)· 且不在"z 到底等收敛"的合法保持
                        // (连塌<10 或 差px≥2×线)。全量出来的。晃/回家拍不进这条路,天然不计。
                        if 差 > 5.0 * 卡阈 && (p.连塌 < 10 || 差px >= 2.0 * 线)
                            && 挪.map_or(false, |m| m < 卡阈) {
                            p.僵拍 += 1;
                            if p.僵拍 >= 30 {
                                println!("[服] 🔴 手不跟:连 {} 拍挪不动(差 {:.3} m)⇒ 弃这个计划换下手点", p.僵拍, 差);
                                p.僵拍 = 0; p.段 = 8;
                            }
                        } else { p.僵拍 = 0; }
                        if p.收敛计 >= 2 {
                            let axc = task::列(&p.q, 工具列 % 3);
                            p.指尖 = [here[0] + axc[0] * 工具长, here[1] + axc[1] * 工具长, here[2] + axc[2] * 工具长];
                            println!("[服] 🟢 收敛(差 {:.4})且 z 到底 ⇒ 合爪", 差px);
                            // 取证帧:合爪起点的指-物几何(空合第 6 次同签名,咬边/斜弹/错位只有图能分)。
                            认位 = Some([here[0], here[1], here[2]]); // 收敛到合爪 = 锚的终极自证,这位姿爪必可见
                            合序 += 1;
                            if let (Ok(d), Some(img)) = (std::env::var("BL_DUMP"), 帧.cams.get(p.相机)) {
                                let mut buf = format!("P5\n{} {}\n255\n", img.0, img.1).into_bytes();
                                buf.extend_from_slice(&img.2);
                                let _ = std::fs::write(format!("{d}/close{合序}.pgm"), buf);
                            }
                            p.段 = 2; p.卡 = 0;
                            let n = (0.05f64.ln() / (1.0 - 交付率).max(1e-6).ln()).ceil();
                            p.合等 = (n as u32).clamp(3, 120);
                        }
                        // 限龄每拍必走(N3 实锤:原来只在「差>0.015」的分支里加龄 ⇒ 推算
                        // 一旦漂到"已对准"就冻住、永不作废 —— 正是"差 0.013 钉死、真爪在
                        // 0.14 画幅外"的机制)。校准拍在上面清零;40 拍无实测 ⇒ 作废重捕。
                        p.预测龄 += 1;
                        if p.预测龄 > 40 {
                            println!("[服] 快眼 {} 拍没咬上 ⇒ 晃爪重认", p.预测龄 - 1);
                            p.锚新 = false;
                            p.收敛计 = 0;
                            if p.连塌 >= 10 && p.底晃次 >= 2 { println!("[服]   z 底重晃预算用尽 ⇒ 弃这个计划换下手点"); p.段 = 8; }
                            else { if p.连塌 >= 10 { p.底晃次 += 1; println!("[服]   z 底第 {} 次重晃(≤2)", p.底晃次); } p.晃态 = 1; p.晃等 = 0; p.预测龄 = 0; }
                        }
                    } else {
                        // 没有爪锚 ⇒ 晃爪认手(首捕获;VLM 不再是爪源)。
                        p.伺服目标 = [here[0], here[1], here[2]];
                        p.晃态 = 1; p.晃等 = 0;
                        if 拍 % 25 == 1 { println!("[服] 接近:无爪锚 ⇒ 晃爪认手(拍 {})", p.段拍); }
                    }
                    // 段超时:换下手点重来(预算内多试)。
                    if p.段拍 > 900 {
                        println!("[服] 接近段超时(900 拍)⇒ 弃这个计划换下手点");
                        p.段 = 8;
                    }
                }
                2 => {
                    // 通道无关的推进-饱和(终态配置,owner 验收 2026-08-21):合 = 全通道
                    // 朝收拢推进;停 = 任一通道【被撑住】(读数与上拍差 < 撑阈 且 读数还在
                    // 半途)或全通道读数到底(空合)。2/5/20 指同一段代码,零结构假设。
                    let 撑阈 = 0.01; // 拍间读数不再走的判定(归一开合/拍;协议数,机体无关)
                    // 🔴 【全部】通道都停在半途才算夹住(N35 三连定案:单通道停走就抬 ⇒
                    // 一根手指从侧面顶住方块也读成"夹到",立刻抬,方块从没进指间 ——
                    // 收敛 0.006-0.013 合上撑住抬起全空 ×3。物体真在指间 = 它约束整只手,
                    // 每个通道都被它停住;只停一部分 = 侧顶/边缘,继续合让其余通道追上,
                    // 追不上会全到底走空爪自检。2/5/20 指同一句,零机体假设。
                    let mut 撑数 = 0usize;
                    let mut 全到底 = !帧.jaw.is_empty();
                    for (ci, g) in 帧.jaw.iter().enumerate() {
                        let 上 = p.上爪读.get(ci).copied().unwrap_or(1.0);
                        if (上 - g).abs() < 撑阈 && *g > 0.05 { 撑数 += 1; }
                        if *g > 0.05 { 全到底 = false; }
                    }
                    let 有撑 = !帧.jaw.is_empty() && 撑数 == 帧.jaw.len();
                    p.上爪读 = 帧.jaw.iter().copied().collect();
                    if p.合等 > 0 { p.合等 -= 1 }
                    if 有撑 && p.合等 < 100 {
                        println!("[服] 有指通道被撑住(读数停在半途)⇒ 合完,抬");
                            if let (Ok(d), Some(img)) = (std::env::var("BL_DUMP"), 帧.cams.get(p.相机)) {
                                let mut buf = format!("P5\n{} {}\n255\n", img.0, img.1).into_bytes();
                                buf.extend_from_slice(&img.2);
                                let _ = std::fs::write(format!("{d}/grip{合序}.pgm"), buf);
                            }
                        p.段 = 3; p.卡 = 0;
                    } else if p.合等 == 0 {
                        if 全到底 { println!("[服] 全通道收到底(没撑住任何东西)⇒ 仍按流程抬,由抬后自检兜底"); }
                        println!("[服] 合爪到位 ⇒ 抬");
                        p.段 = 3; p.卡 = 0;
                    }
                }
                3 if 到 || p.卡 >= 15 => {
                    // 夹住 = 【任一通道】被撑住(收拢命令下读数停在半途)。逐通道判,
                    // 指数无关 —— 两爪 1 通道、五指 5 通道、廿指 20 通道同一行。
                    let 撑住数 = 帧.jaw.iter().filter(|g| **g > 0.05).count();
                    if 撑住数 > 0 {
                        if 拍 % 50 == 1 { println!("[服] 🟢 抬到位且 {}/{} 指通道被撑住 —— 像是夹住了,保持", 撑住数, 帧.jaw.len()); }
                    } else {
                        println!("[服] 抬到位但全部 {} 个指通道空合到底 ⇒ 空爪,弃这个计划换下手点重来", 帧.jaw.len());
                        p.段 = 8; // 弃标:出了借用区再清计划
                    }
                }
                _ => {}
            }
            // 单步封顶:0.7 个可达半径(无量纲;跨出线性范围的命令是白发的)。
            let 上限 = 0.7 * 可达带[1];
            let d = [目标[0] - here[0], 目标[1] - here[1], 目标[2] - here[2]];
            let l = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
            let at = if l > 上限 { [here[0] + d[0] / l * 上限, here[1] + d[1] / l * 上限, here[2] + d[2] / l * 上限] } else { 目标 };
            Cmd::Ee { arm: 0, at, quat: p.q, jaw }
        } else {
            // ── 没有计划:从观测里拿指令,算一把 ─────────────────────────
            let o = plug.last.clone();
            let 指令 = o.as_ref()
                .and_then(|o| 取(o, &["instruction".to_string()]))
                .and_then(|v| 字(&v))
                .filter(|s| !s.is_empty());
            let mut 出 = 回声.clone();
            if let Some(指令) = 指令 {
                // 找一台【标定解出来的 + 这一帧真有图和深度的】相机。
                let mut 找到 = None;
                for (i, eyeN, _) in 相机们 {
                    let Some(路) = plug.lay.cams.get(*i) else { continue };
                    let Some(rgbv) = o.as_ref().and_then(|o| 取(o, 路)) else { continue };
                    let Some((w, h, rgb)) = wire::as_rgb(&rgbv) else { continue };
                    let mut 深路 = 路.clone();
                    if let Some(last) = 深路.last_mut() { *last = "depth".to_string(); }
                    let Some(dv) = o.as_ref().and_then(|o| 取(o, &深路)) else { continue };
                    let Some((dw, dh, dep)) = wire::as_f32_grid(&dv) else { continue };
                    if dw != w || dh != h { continue }
                    找到 = Some((*i, eyeN, w, h, rgb, dep));
                    break;
                }
                if let Some((cam, eyeN, w, h, rgb, dep)) = 找到 {
                    let d回0 = ((原位[0] - here[0]).powi(2) + (原位[1] - here[1]).powi(2) + (原位[2] - here[2]).powi(2)).sqrt();
                    if 要回看 && d回0 > 0.33 * 张开 {
                        println!("[服] 上一计划死了且身子不在原位(差 {:.3} m)⇒ 先回位再问眼", d回0);
                        计划 = Some(抓况 {
                            指尖: [0.0; 3], q: [0.0, 0.0, 0.0, 1.0], 相机: cam, 段: 11, 卡: 0, 合等: 0,
                            物像素: (0.5, 0.5), 物跨: 0.05, 对准帧: Vec::new(), 对准次: 0, 对准挪: [0.0, 0.0],
                            上位: None, 连塌: 0, 抬高: 0.0, 对准锚: [0.0; 3], 下探锚: [0.0; 2], 对准等: 0, 上爪: None, 上期望: (0.0, 0.0), 伺服号: [1.0, 1.0], 翻候: [0, 0], 钉xy: [0.0, 0.0], 伺服上位: None, 对Δw: [0.0, 0.0, 0.0], 近答: Vec::new(), 模板: Vec::new(), 模板半: 0, 验拍: 0, 晃态: 0, 晃等: 0, 晃帧: Vec::new(), 晃次: 0, 回家中: false, 回过家: false, 上fix生: None, 步率: [1.0, 1.0], 预测爪: None, 命累: 0.0, 应累: 0.0, 预测龄: 0, 僵拍: 0, 底晃次: 0, 锚新: false, 上降z: f64::MAX, 对准末差: f64::MAX, 交替次: 0, 段起z: f64::MAX, 段拍: 0, 伺服目标: [f64::NAN; 3], 收敛计: 0, 上爪读: Vec::new(),
                        });
                        出 = 回声.clone();
                        plug.act(&出);
                        match plug.sense() { Some(f) => { 帧 = f; continue } None => return }
                    }
                    println!("[服] 指令:{指令:?} —— 整句直接问眼(它的提示词本来就是 Task: ...)");
                    match body_layer::eye::ask(眼主机, 眼端口, &指令, &rgb, w, h) {
                        Err(e) => { if 拍 % 25 == 1 { println!("[服] 眼答不上:{e}"); } }
                        Ok(look) => {
                            // 裁一块重问:物体在小图里占比大,指点更准;两次的分歧就是
                            // 眼自己的定位不确定度,拿它当圈物体的宽容(全是量出来的)。
                            let (mut u, mut v) = (look.u, look.v);
                            let mut 宽容 = 1.5f64; // 下限:眼给两位小数,分歧至少一个量化台阶(无量纲)。
                            // 裁边 = 指出的占宽 × 3(无量纲:目标 + 两侧各一个身位),
                            // 夹在画幅的 1/8..1/3(比例)之间 —— 太小眼看不清,太大重问失去意义。
                            let 半 = ((look.span_frac.max(0.05) * 3.0 * w as f64) as usize).clamp(w / 8, w / 3);
                            let cx = (u * w as f64) as usize;
                            let cy = (v * h as f64) as usize;
                            let (x0, y0) = (cx.saturating_sub(半), cy.saturating_sub(半));
                            let (x1, y1) = ((cx + 半).min(w), (cy + 半).min(h));
                            let (cw, ch) = (x1 - x0, y1 - y0);
                            if cw > w / 16 && ch > h / 16 {
                                let mut sub = Vec::with_capacity(cw * ch * 3);
                                for yy in y0..y1 {
                                    let a = (yy * w + x0) * 3;
                                    sub.extend_from_slice(&rgb[a..a + cw * 3]);
                                }
                                if let Ok(l2) = body_layer::eye::ask(眼主机, 眼端口, &指令, &sub, cw, ch) {
                                    let nu = (x0 as f64 + l2.u * cw as f64) / w as f64;
                                    let nv = (y0 as f64 + l2.v * ch as f64) / h as f64;
                                    let 分歧 = ((nu - u).powi(2) + (nv - v).powi(2)).sqrt();
                                    宽容 = (1.0 + 分歧 / look.span_frac.max(1e-6)).max(宽容);
                                    println!("[服] 眼重问:({:.3},{:.3})→({:.3},{:.3}) 分歧 {:.4} ⇒ 宽容 {:.2}", u, v, nu, nv, 分歧, 宽容);
                                    u = nu; v = nv;
                                }
                            }
                            // 归一化的眼 → 这幅图的像素单位。
                            let eye = point_gen::Eye {
                                fx: eyeN.fx * w as f64, fy: eyeN.fy * h as f64,
                                cx: eyeN.cx * w as f64, cy: eyeN.cy * h as f64,
                                at: eyeN.at, q: eyeN.q,
                            };
                            let r = task::尺 {
                                张开, 工具长, 可达内: 可达带[0], 可达: 可达带[1], 探幅, 工具列,
                            };
                            match task::算一把(&eye, &dep, w, h, [u * w as f64, v * h as f64],
                                look.span_frac, Some(&rgb), 0.5, 宽容, here, &试过, &r) {
                                Err(e) => {
                                    println!("[服] 🔴 算不出这一把:{e:?} ⇒ 回原位重看再问");
                                    计划 = Some(抓况 {
                                        指尖: [0.0; 3], q: [0.0, 0.0, 0.0, 1.0], 相机: cam, 段: 11, 卡: 0, 合等: 0,
                                        物像素: (u, v), 物跨: look.span_frac, 对准帧: Vec::new(), 对准次: 0, 对准挪: [0.0, 0.0],
                                        上位: None, 连塌: 0, 抬高: 0.0, 对准锚: [0.0; 3], 下探锚: [0.0; 2], 对准等: 0, 上爪: None, 上期望: (0.0, 0.0), 伺服号: [1.0, 1.0], 翻候: [0, 0], 钉xy: [0.0, 0.0], 伺服上位: None, 对Δw: [0.0, 0.0, 0.0], 近答: Vec::new(), 模板: Vec::new(), 模板半: 0, 验拍: 0, 晃态: 0, 晃等: 0, 晃帧: Vec::new(), 晃次: 0, 回家中: false, 回过家: false, 上fix生: None, 步率: [1.0, 1.0], 预测爪: None, 命累: 0.0, 应累: 0.0, 预测龄: 0, 僵拍: 0, 底晃次: 0, 锚新: false, 上降z: f64::MAX, 对准末差: f64::MAX, 交替次: 0, 段起z: f64::MAX, 段拍: 0, 伺服目标: [f64::NAN; 3], 收敛计: 0, 上爪读: Vec::new(),
                                    });
                                }
                                Ok((n候, n步, 宽, 指尖0, q, 桌面c)) => {
                                    // 🔴 z 重基(N42 终判:每炮相机模型把云的 z 放得乱漂 —— N41 云桌面
                                    // -0.199、N42 ≈-0.36、N37 +0.42,而身体自测 floor=0.871 ⇒ 悬停航点
                                    // 插到真桌面下 ⇒ IK 冻,同一计划兜底无限循环)。两个桌面都是量出
                                    // 来的:云系 zfloor(不接触测高)与本体系 floor(标定格,探到的)。
                                    // z 偏移 = floor − 云桌面,加回指尖;xy 残差交给像素伺服(N40 已证)。
                                    // ⚠️ 试过 表存【云系】指尖 —— 它要和云系候选比距离,存重基值会把排斥圈全失效。
                                    if let Some(基) = 桌上z {
                                        if (桌面c - 基).abs() > 张开 {
                                            println!("[服] 🔴 桌面一致性拒:这计划的云桌面 {:.3} 离本炮基准 {:.3} 超一个张开 —— 答到了别的平面(多半是自己身子)⇒ 回位重看", 桌面c, 基);
                                            // 🔴 自愈(反转陷阱:若基准本身是幻影,真计划会被连拒到死)。
                                            // 连拒 2 次 ⇒ 承认基准可疑,清零重学。
                                            桌拒计 += 1;
                                            if 桌拒计 >= 2 {
                                                println!("[服]   连拒 {} 次 ⇒ 基准可疑,清零重学", 桌拒计);
                                                桌上z = None; 桌拒计 = 0;
                                            }
                                            要回看 = true;
                                            出 = 回声.clone();
                                            plug.act(&出);
                                            match plug.sense() { Some(f) => { 帧 = f; continue } None => return }
                                        }
                                    } else { 桌上z = Some(桌面c); 桌拒计 = 0; }
                                    let mut 指尖 = 指尖0;
                                    if let Some(fl) = body.get(Quantity::Floor).filter(|m| m.dim >= 1).map(|m| m.value[0]) {
                                        let dz = fl - 桌面c;
                                        println!("[服] z 重基:云桌面 {:.3} → 本体地板 {:.3}(Δ {:+.3})⇒ 指尖 z {:.3}→{:.3}", 桌面c, fl, dz, 指尖0[2], 指尖0[2] + dz);
                                        指尖[2] += dz;
                                    }
                                    let 抬高 = match 位移量(&指令) {
                                        Some(v) => v,
                                        None => {
                                            // 指令没说抬多高 —— 语义缺口,具名。用 2.5 个钳口跨度顶上
                                            //(无量纲;"拿起来"至少要离开支撑面一手的量级),并说出来。
                                            println!("[服] ⚠️ 指令里解析不到位移量 —— 抬高用 2.5 × 钳口跨度顶上");
                                            2.5 * 张开
                                        }
                                    };
                                    println!("[服] 🟢 计划:候选 {n候} · 航点 {n步} · 下手宽 {:.1} mm · 指尖 ({:.3},{:.3},{:.3}) · 抬 {:.2} m",
                                        宽 * 1000.0, 指尖[0], 指尖[1], 指尖[2], 抬高);
                                    试过.push(指尖0);
                                    计划 = Some(抓况 {
                                        指尖, q, 相机: cam, 段: 0, 卡: 0, 合等: 0,
                                        物像素: (u, v), 物跨: look.span_frac, 对准帧: Vec::new(), 对准次: 0, 对准挪: [0.0, 0.0],
                                        上位: None, 连塌: 0, 抬高, 对准锚: [0.0; 3], 下探锚: [0.0; 2], 对准等: 0, 上爪: None, 上期望: (0.0, 0.0), 伺服号: [1.0, 1.0], 翻候: [0, 0], 钉xy: [0.0, 0.0], 伺服上位: None, 对Δw: [0.0, 0.0, 0.0], 近答: Vec::new(), 模板: Vec::new(), 模板半: 0, 验拍: 0, 晃态: 0, 晃等: 0, 晃帧: Vec::new(), 晃次: 0, 回家中: false, 回过家: false, 上fix生: None, 步率: [1.0, 1.0], 预测爪: None, 命累: 0.0, 应累: 0.0, 预测龄: 0, 僵拍: 0, 底晃次: 0, 锚新: false, 上降z: f64::MAX, 对准末差: f64::MAX, 交替次: 0, 段起z: f64::MAX, 段拍: 0, 伺服目标: [f64::NAN; 3], 收敛计: 0, 上爪读: Vec::new(),
                                    });
                                    出 = 回声.clone();
                                }
                            }
                        }
                    }
                } else if 拍 % 50 == 1 {
                    println!("[服] 有指令但没有一台【标定解出的相机 + 本帧有图有深度】—— 回声等待");
                }
            }
            出
        };
        // 弃标(段 8):抬到位但空爪 —— 在借用区外把计划清掉,下一拍重新计划换下手点。
        if 计划.as_ref().map_or(false, |p| p.段 == 12) { 计划 = None; 要回看 = false; }
        else if 计划.as_ref().map_or(false, |p| p.段 == 10) { 计划 = None; 要回看 = true; }
        plug.act(&cmd);
        match plug.sense() { Some(f) => { 帧 = f; continue } None => return }
    }
}

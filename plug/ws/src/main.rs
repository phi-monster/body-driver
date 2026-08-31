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
mod flow;
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
    /// 上一条**真正发出去**的动作。没有新命令时重发它 —— 见 `抽到下一帧` 里那段。
    上次发出: Option<Value>,
    /// 对方刚发过 `reset`(= 新的一集开始了)。干活模式据此清掉上一集的计划与"试过"名单。
    复位过: bool,
    /// 连拍计数(`BL_FILM`)。每炮都要有录像可看 —— 见 `sense` 里那段。
    胶片: u64,
    /// 🔴 **量出来的帧时(秒/帧)** —— 减法②的"持续时间"这一维靠它换成拍数。
    /// 以前它只被打印(`[计时] 等帧 xxx ms/帧`),**没有任何地方读得到** ⇒
    /// "按住 N 秒"在驱动里无法表达,于是擦/按/拧只能写成"走够多少距离"。
    /// 走的是滑动平均,开机第一拍还没有值时按 0 处理(调用方退回按拍数)。
    帧时秒: f64,
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
                // 🔴🔴 **没有新命令时不许回空 —— 回"照现在这样保持"。**
                // 空动作在线上是一句"我没意见",而评测循环把它读成"这一集到此为止":
                // 实测(2026-08-23,N128)每一集都在第一拍结束,19 集全废,
                // 而两侧零报错 —— 驱动在等下一帧观测,sim 在开下一集,谁都没错。
                // 自标定的第一拍必然还没有命令(它要先看一帧才知道往哪走),
                // 所以这一拍决定了**整个标定能不能开始**。
                // 保持动作的每一个数都照抄机器人自己此刻报的:位姿、姿态、每个钳口通道。
                // 读不到就还是回空 —— 那时候空是诚实的(我连它现在在哪都不知道)。
                // 🔴🔴 **没有新命令时要【重发上一条命令】,不是"保持在当前测到的位置"。**
                // 实测(2026-08-24,N128,连炸四轮):发"保持在当前开度"会把**正在执行中的
                // 张开命令当场撤销** —— 钳口那一相每拍发一次新开度,而中间那些没有新命令的拍
                // 被这条保持顶掉,爪子于是原地不动:腕相机贴着指尖 10 cm、两指清晰可见,
                // 9 秒内画面差 <52 像素,双响恒 0,跨度四次量不出来。
                // 手臂没被坑到,因为它的保持发的就是自己当前位姿(无害);
                // **只有"必须持续朝一个新值走"的量会被抵消**,而钳口正是这种量。
                // ⇒ 语义应当是"继续执行我上一条命令",空动作只在**从未下过命令**时才发。
                let 出 = match self.待发.take() {
                    Some(a) => { self.上次发出 = Some(a.clone()); Some(a) }
                    None => self.上次发出.clone().or_else(|| self.保持()),
                };
                rmpv::Value::Map(vec![(
                    rmpv::Value::String("result".into()),
                    rmpv::Value::Array(出.into_iter().collect()),
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

    /// 「照现在这样保持」:把机器人此刻报的位姿/姿态/每通道开合原样编成一条动作。
    /// 它不是一个默认值,是一次**回声** —— 零机体假设,零常数。
    fn 保持(&mut self) -> Option<rmpv::Value> {
        let 位姿键: Vec<String> = self.lay.ee.iter().filter_map(|p| p.last().cloned()).collect();
        let 钳口键: Vec<String> = self.lay.jaw.iter().filter_map(|p| p.last().cloned()).collect();
        let n = 位姿键.len().min(钳口键.len());
        if n == 0 { return None; }
        let o = self.last.as_ref()?;
        let mut 位姿: Vec<Vec<f64>> = Vec::with_capacity(n);
        let mut 钳口: Vec<f64> = Vec::with_capacity(n);
        for i in 0..n {
            位姿.push(取(o, &self.lay.ee[i]).map(|v| 数组(&v)).unwrap_or_default());
            钳口.push(取(o, &self.lay.jaw[i]).map(|v| 数组(&v)).and_then(|a| a.first().copied()).unwrap_or(1.0));
        }
        let 我 = 0usize;
        let e = 位姿.get(我)?;
        if e.len() < 7 { return None; }
        let at = [e[0], e[1], e[2]];
        let quat = [e[3], e[4], e[5], e[6]];
        Some(wire::pose_action(&位姿键, &钳口键, 我, &at, &quat, &位姿, &钳口))
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
                            // 🔴🔴 **每一帧都编号存一份 —— 事后拼成视频【自己看】。**
                            // 仓规 §3.6:空间性的毛病,第一件事是渲图去看,不是拿坐标推。
                            // 事件式落图(`BL_DUMP`)只在"出事那一帧"存,而**为什么会出事在没出事的那些帧里**;
                            // 只有连续的帧序列能回答"手到底往哪走了、卡在哪儿、爪子合在什么上面"。
                            // 🔴 **两台相机都要落图。** 只落头部那台的时候,腕相机里发生了什么
                            // 完全看不见 —— 而"手到底有没有靠近物体"这件事,只有腕相机答得干脆。
                            if let Ok(d) = std::env::var("BL_VID") {
                                let ci = f.cams.len();
                                let _ = std::fs::create_dir_all(&d);
                                let n = unsafe { static mut N: u32 = 0; if ci == 0 { N += 1; } N };
                                // 🔴🔴 **落图要稀释 —— 盘写满会把整炮弄死。**(YF 实测 2026-08-29)
                                // YF 一集写了 178040 张 = **52G**,把 250G 的盘顶到 100%,
                                // IsaacSim 当场崩(`'IsaacRLEnv' object has no attribute 'scene'`),
                                // 官方成绩 `Success/Fail/Unstable` 三个全 0 —— **一集都没跑完**。
                                // 开头密、后面疏:开头那段是"它到底怎么起步的",最需要逐帧;
                                // 之后一动作要几十帧才变一次样,隔着存不丢信息。
                                // 2000 / 20 是**帧计数**(无量纲),只管落图密度,不进任何判据。
                                if n < 2000 || n % 20 == 0 {
                                let mut buf = format!("P5\n{w} {h}\n255\n").into_bytes();
                                buf.extend_from_slice(&g);
                                let _ = std::fs::write(format!("{d}/f{:06}_c{ci}.pgm", n), buf);
                                }
                            }
                            if let Ok(d) = std::env::var("BL_DUMP") {
                                let _ = std::fs::create_dir_all(&d);
                                let mut buf = format!("P5\n{w} {h}\n255\n").into_bytes();
                                buf.extend_from_slice(&g);
                                let _ = std::fs::write(format!("{d}/cam{}.pgm", f.cams.len()), buf);
                                // 🔴 **深度也落一张。** 彩色图黑掉有两种完全不同的原因 ——
                                // 场景没光,和相机没对着东西 —— 而**这两种在彩色图上同形**。
                                // 深度不吃光照:图里有桌面就是"对着了、只是黑",全是远/无穷就是"没对着"。
                                // 实测(2026-08-23,N128):彩色图最亮 61/255、均值 10.8,
                                // 光靠它我只能猜;这一张图是用来终结猜测的。
                                let mut 深路 = p.clone();
                                // 🔴 **深度图的路径由 `discover` 认出来,不许把彩色路径的最后一段换成字面量 `depth`。**
                                // 那是这一台机器的名字;换一台叫 `depth_image` / `range` 的就断了(owner 2026-08-28「减法①」)。
                                if let Some(dp) = self.lay.depth.get(f.cams.len()).or_else(|| self.lay.depth.first()) { 深路 = dp.clone(); }
                                if let Some((dw, dh, dep)) = 取(&o, &深路).and_then(|dv| wire::as_f32_grid(&dv).map(|(a, b, c)| (a, b, c.to_vec()))) {
                                    let 有限: Vec<f64> = dep.iter().copied().filter(|x| x.is_finite() && *x > 0.0).collect();
                                    let (lo, hi) = 有限.iter().fold((f64::MAX, f64::MIN), |(a, b), x| (a.min(*x), b.max(*x)));
                                    println!("[看] 深度 {dw}x{dh} · 有限点 {}/{} · 范围 {:.3}–{:.3} m", 有限.len(), dep.len(), lo, hi);
                                    let mut db = format!("P5\n{dw} {dh}\n255\n").into_bytes();
                                    let 跨 = (hi - lo).max(1e-6);
                                    // 255 是 8 位灰度的**格式**上限(PGM),无量纲,不描述身体。
            db.extend(dep.iter().map(|x| if x.is_finite() && *x > 0.0 { (((x - lo) / 跨) * 255.0) as u8 } else { 0u8 }));
                                    let _ = std::fs::write(format!("{d}/depth{}.pgm", f.cams.len()), db);
                                }
                                // 🔴 相机自己报的内外参打一次 —— "相机在哪、朝哪、视角多宽"
                                // 是"看不见东西"这件事唯一能一次问清的三件。
                                unsafe {
                                    static mut 打过: bool = false;
                                    if !打过 {
                                        打过 = true;
                                        for k in ["intrinsic_matrix", "extrinsic_matrix", "shape"] {
                                            let mut kp = p.clone();
                                            if let Some(last) = kp.last_mut() { *last = k.to_string(); }
                                            if let Some(v) = 取(&o, &kp) {
                                                let a = 数组(&v);
                                                println!("[看] {k} = {:?}", a.iter().map(|x| (x * 1000.0).round() / 1000.0).collect::<Vec<_>>());
                                            } else {
                                                println!("[看] {k} = (没有)");
                                            }
                                        }
                                        if let Some(e) = f.ee.get(0) {
                                            println!("[看] 此刻末端 = ({:.3},{:.3},{:.3})", e[0], e[1], e[2]);
                                        }
                                    }
                                }
                            }
                            f.cams.push((w, h, g));
                        }
                    }
                }
            }
        }
        // 🔴 **连拍(`BL_FILM=<目录>`)。** 死规矩:起下一炮之前要先看完这一炮的录像。
        // ⚠️ **`BL_VID` 已经在干这件事,但只在【干活循环】里**(见 `拍 % 4` 那段)——
        // 于是**自标定那一段全程没有像**,而这条线上八成的时间和全部的疑难都在自标定里。
        // 这里补的是那一段:落在 `sense()` 上 ⇒ 从握手第一帧起就有;**每台相机都落**
        // (腕相机配对恒 0 那种问题,只有腕相机自己的图能分)。
        // 事件式落图(`BL_DUMP`)只在"出事那一帧"存,而**为什么会出事**在没出事的那些帧里。
        // 半分辨率 + 抽帧是为了让整炮装得下:原分辨率几千帧是几个 GB。
        if let Ok(d) = std::env::var("BL_FILM") {
            let 隔 = std::env::var("BL_FILM_STRIDE").ok().and_then(|v| v.parse::<u64>().ok()).unwrap_or(6);
            let 上限 = std::env::var("BL_FILM_MAX").ok().and_then(|v| v.parse::<u64>().ok()).unwrap_or(6000);
            let n = self.胶片;
            self.胶片 += 1;
            if 隔 > 0 && n % 隔 == 0 && n / 隔 < 上限 {
                let _ = std::fs::create_dir_all(&d);
                for (ci, (w, h, g)) in f.cams.iter().enumerate() {
                    let (hw, hh) = (w / 2, h / 2);
                    let mut buf = format!("P5\n{hw} {hh}\n255\n").into_bytes();
                    for y in 0..hh {
                        for x in 0..hw { buf.push(g[(y * 2) * w + x * 2]); }
                    }
                    let _ = std::fs::write(format!("{d}/c{ci}_{:06}.pgm", n / 隔), buf);
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
                // 🔴 一并**存下来**:等帧 + 解图 = 一拍真正花掉的时间。减法②的"按住 N 秒"用它。
                self.帧时秒 = (等 + 解) as f64 / 50e6;
                println!("      [计时] 近 50 帧:等帧 {:.1} ms/帧 · 解图 {:.1} ms/帧 · 一拍 {:.3} s · 相机 {} 台",
                    等 as f64 / 50_000.0, 解 as f64 / 50_000.0, self.帧时秒, f.cams.len());
                等 = 0; 解 = 0;
            }
        }
        Some(f)
    }

    fn act(&mut self, c: &Cmd) -> bool {
        // 只记下要做什么;真正送出去在对方问"给我一个动作"的那一拍。
        // 🔴🔴 **关节命令以前在这里被【静默扔掉】,而且返回"发出去了"。**
        // 实测代价(2026-08-25,自模那一炮):七根关节各命令 +0.2000 rad,
        // 实到 +0.0053 / −0.0032 / +0.0061 / +0.0017 —— 那是漂移噪声,**一条命令都没发**。
        // 于是画面里什么都没变,而日志上"命令发了"。⇒ 接上,并且**发不出去就说发不出去**。
        if let Cmd::Joints { arm, q, jaw } = c {
            let 关节键: Vec<String> = self.lay.joints.iter().filter_map(|p| p.last().cloned()).collect();
            let 钳口键: Vec<String> = self.lay.jaw.iter().filter_map(|p| p.last().cloned()).collect();
            let n = 关节键.len().min(钳口键.len());
            if n == 0 { return false; }
            let mut 关节: Vec<Vec<f64>> = Vec::with_capacity(n);
            let mut 钳口: Vec<f64> = Vec::with_capacity(n);
            for i in 0..n {
                关节.push(self.last.as_ref().and_then(|o| 取(o, &self.lay.joints[i])).map(|v| 数组(&v)).unwrap_or_default());
                钳口.push(self.last.as_ref().and_then(|o| 取(o, &self.lay.jaw[i])).map(|v| 数组(&v))
                    .and_then(|a| a.first().copied()).unwrap_or(*jaw));
            }
            let 我 = (*arm).min(n - 1);
            self.待发 = Some(wire::joint_action(&关节键, &钳口键, 我, q, &关节, &钳口));
            return true;
        }
        let (arm, at, quat, 每通道) = match c {
            Cmd::Ee { arm, at, quat, jaw } => (*arm, *at, *quat, vec![*jaw]),
            Cmd::Grip { arm, at, quat, per } => (*arm, *at, *quat, per.clone()),
            // 🔴 认不出来的命令**不许报告成功** —— "发不出去"和"发了没动"是两件事。
            Cmd::Hold => return true,
            _ => return false,
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


/// 机体自报的相机标定:内参(每台 RGBD 相机出厂就带)+ 外参(装在哪)+ 手那个深度。
///
/// 🔴🔴 **读它不是机体假设** —— 和读关节角、读末端位姿是同一类事:机体发布什么就读什么。
/// 自解整台相机是为"机体什么都不报"准备的后备;**报了还去自解,是拿一个估计量换掉一个已知量**。
/// 实测代价(2026-08-23,N128):自解出 fx=0.09,而相机自报 0.450(**差 5 倍**),
/// 于是尺算成「1 归一化单位 = 3.66 m」(真值约 1.18 m),
/// 把一副 8 cm 的钳口量成 0.25 m —— 而这个数看起来完全正常,只有跟机体宽度对一下才露馅。
/// 尺本身也只用自报的量:**手那一个像素上的深度 ÷ 焦距**,一个常数都不填。
fn 自报相机(
    o: &rmpv::Value,
    路: &[String],
    手像素: Option<(f64, f64)>,
    ee路: Option<&[String]>,
    // 🔴 深度那一路由 `discover` 靠形状认出来传进来(减法①);认不出才退回按名字取。
    深路: Option<&[String]>,
) -> Option<(point_gen::Eye, f64)> {
    let 取键 = |k: &str| -> Option<rmpv::Value> {
        let mut p = 路.to_vec();
        if let Some(l) = p.last_mut() { *l = k.to_string(); }
        取(o, &p)
    };
    let k = 取键("intrinsic_matrix").map(|v| 数组(&v))?;
    let shape = 取键("shape").map(|v| 数组(&v))?;
    if k.len() < 9 || shape.len() < 2 { return None; }
    let (h, w) = (shape[0], shape[1]);
    if !(w > 1.0 && h > 1.0) { return None; }
    // 驱动内部一律用**画幅比例**,所以把像素单位的内参除以画幅。
    let (fx, fy, cx, cy) = (k[0] / w, k[4] / h, k[2] / w, k[5] / h);
    if !(fx.is_finite() && fy.is_finite() && fx > 0.0 && fy > 0.0) { return None; }
    let e = 取键("extrinsic_matrix").map(|v| 数组(&v))?;
    if e.len() < 16 { return None; }
    let at = [e[3], e[7], e[11]];
    // 相机→世界的旋转阵(行主序前三行前三列)转四元数 (w,x,y,z)。
    let m = [[e[0], e[1], e[2]], [e[4], e[5], e[6]], [e[8], e[9], e[10]]];
    let tr = m[0][0] + m[1][1] + m[2][2];
    let q = if tr > 0.0 {
        let sq = (tr + 1.0).sqrt() * 2.0;
        // 🔴 下面四支是「由旋转阵反解四元数」的**标准公式**,0.25 是代数常数
        // (来自 1+tr(R) 的展开),**无量纲**,和任何身体、任何单位无关。
        [0.25 * sq, (m[2][1] - m[1][2]) / sq, (m[0][2] - m[2][0]) / sq, (m[1][0] - m[0][1]) / sq]
    } else if m[0][0] > m[1][1] && m[0][0] > m[2][2] {
        let sq = (1.0 + m[0][0] - m[1][1] - m[2][2]).sqrt() * 2.0;
        [(m[2][1] - m[1][2]) / sq, 0.25 * sq, (m[0][1] + m[1][0]) / sq, (m[0][2] + m[2][0]) / sq]
    } else if m[1][1] > m[2][2] {
        let sq = (1.0 + m[1][1] - m[0][0] - m[2][2]).sqrt() * 2.0;
        [(m[0][2] - m[2][0]) / sq, (m[0][1] + m[1][0]) / sq, 0.25 * sq, (m[1][2] + m[2][1]) / sq]
    } else {
        let sq = (1.0 + m[2][2] - m[0][0] - m[1][1]).sqrt() * 2.0;
        // (续上)同一条标准公式的第四支,0.25 仍是代数常数,**无量纲**。
        [(m[1][0] - m[0][1]) / sq, (m[0][2] + m[2][0]) / sq, (m[1][2] + m[2][1]) / sq, 0.25 * sq]
    };
    // 🔴🔴 **相机坐标系的约定要换过来:USD/Isaac 的相机看的是【−z】,而这一层的 `Eye` 看的是【+z】。**
    //
    // `Eye` 的约定(见 `point_gen::Eye::q` 的文档):**+z 朝前 · +x 朝右 · +y 朝下**(计算机视觉那一套)。
    // 而 Isaac/USD 的相机 prim 是 **−z 朝前 · +y 朝上**(OpenGL 那一套)。两者差一个**绕 x 轴 180°**。
    // 机体自报的外参是那个 prim 的位姿,直接当 `Eye::q` 用 ⇒ 光轴朝后、上下颠倒。
    //
    // 🔴 实测指纹(2026-08-25,炮1 第一次真的去抓):
    // ① `[链] 减支撑面 … 法向 [0.44, 0.21, 0.87]` —— 桌面法向偏离竖直 **29.5°**,
    //    而头顶相机自己的俯仰正好是 **30°**(`camera_rgbd_franka.yml` 的 `ori [30,0,0]`)。
    // ② 反投影出来的物点 z = **1.6442..1.7131**,而相机自己在 z = **1.308** ——
    //    物体被放到了**相机上方 0.34–0.40 m**;而真桌面在相机**下方**约 0.35–0.40 m
    //    (手的原位 z=1.023)。**差值一分不差地翻了个号。**
    // ③ 于是"物体离手 0.822 m"而量到的可达上界只有 0.345 m ⇒ 每一把都算不出航点(`NoFrame`)。
    //
    // ⚠️ 这一条**只**改机体自报那条路。`全相机` 是从数据自己解出来的,解出来时就已经在
    //    `Eye` 的约定里,再转一次就是转反了。
    let q = {
        // q_cv←world = q_usd←world ⊗ (绕 x 轴 180°) = q ⊗ (w,x,y,z)=(0,1,0,0)
        let (w0, x0, y0, z0) = (q[0], q[1], q[2], q[3]);
        [-x0, w0, z0, -y0]
    };

    // 尺:手那个像素上的深度 ÷ 焦距。
    //
    // 🔴🔴 **手的像素必须是【此刻】的,不许用 `hand_pixel` 那个存下来的常数。**
    // 实测(2026-08-24,N128):`hand_pixel=(0.231,0.108)` 是认手那一相在 home 位形上量的,
    // 而跨度相把手抬走、还绕腕转了四档 —— 那个像素此刻落在**手左边的远处背景**上。
    // 深度图实测:手臂在 0.20–0.30 m,而那一点深 **1.84 m**(全图第 95 百分位)。
    // 于是尺被放大约 6 倍(1 单位 = 5.06 m),**所有换算成米的量跟着放大 6 倍**,
    // 而每一条读数单看都完全正常 —— 这正是"太干净的错"那一类。
    // ⇒ 用**机体自己报的外参**把**此刻的末端位姿**投影到画面上,取那一点的深度。
    //   投影用的全是机体发布的量(内参/外参)+ 本体感受,零手填数。
    let eye0 = point_gen::Eye { fx, fy, cx, cy, at, q };
    let 末端 = ee路
        .and_then(|p| 取(o, p))
        .map(|v| 数组(&v))
        .filter(|a| a.len() >= 3)
        .map(|a| [a[0], a[1], a[2]]);
    let (hu, hv) = match 末端.and_then(|p| eye0.project(point_gen::P3 { x: p[0], y: p[1], z: p[2] })) {
        Some(px) if (0.0..1.0).contains(&px[0]) && (0.0..1.0).contains(&px[1]) => (px[0], px[1]),
        _ => 手像素?,
    };
    // 🔴 **深度那一路不许按名字取。**(减法①,owner 2026-08-28)
    // `取键("depth")` 是把彩色路径的最后一段换成字面量 `depth` —— 那是**这一台机器的名字**。
    // 现在 `discover` 靠形状认得出深度(自报浮点 dtype + 二维 shape),用它给的路径。
    let dv = match 深路 { Some(p) => 取(o, p), None => 取键("depth") }?;
    let (dw, dh, dep) = wire::as_f32_grid(&dv)?;
    let (cu, cv) = ((hu * dw as f64) as isize, (hv * dh as f64) as isize);
    let r = (dw.min(dh) as isize / 40).max(2);
    let mut 片: Vec<f64> = Vec::new();
    for y in (cv - r).max(0)..(cv + r + 1).min(dh as isize) {
        for x in (cu - r).max(0)..(cu + r + 1).min(dw as isize) {
            let d = dep[y as usize * dw + x as usize];
            if d.is_finite() && d > 0.0 { 片.push(d); }
        }
    }
    if 片.is_empty() { return None; }
    片.sort_by(|a, b| a.partial_cmp(b).unwrap());
    // 🔴 取**近侧**分位(20%),不取中位。窗口跨在手的轮廓边上时,一半像素是手、
    // 一半是它后面的远景,而中位数会滑到远景那一半去 —— 前景物体的深度是近侧那一支。
    let 深 = 片[片.len() / 5];
    println!("      [尺] 手投影到 ({hu:.3},{hv:.3}) · 该处近侧深度 {深:.3} m · 窗内 {} 点(最近 {:.3} / 最远 {:.3})",
        片.len(), 片[0], 片[片.len() - 1]);
    Some((eye0, 深 / fx))
}

/// 把此刻身体里量到的每一格写成驱动自己读得回去的那份 JSON,返回存了几格。
///
/// 🔴🔴 **每量完一格就调一次,不要只在最后调。**
/// 只在末尾存,等于赌"这一炮能跑到底"。实测(2026-08-23,N128):评测一共只给
/// 55 集 × 200 步,而自校准要上千步 —— 集数先用完,进程被对面关掉,
/// **整轮量到的东西一格都没落地**,下一炮又从零开始。
/// 增量落盘把"越用越强"从一句设计变成一个事实:跑到哪存到哪,下一炮 `--in` 接着量。

/// **抖一下指通道,把「会动的那一块」收成自模的一条证据。**
///
/// 冻住手臂、只动指通道 ⇒ 画面里会动的那一块**按构造**就是钳口
/// (构造性身份,2026-08-10 / 08-14 两次验通)。那一块的远端到末端的距离就是**法兰到指尖**;
/// 那一块沿哪一维长出去就是**开合方向**。**一个约定都不需要。**
///
/// 🔴 判据全在 `body_layer::selfmodel::一抖::算数` 里,这里只负责把证据收上来 ——
/// 没走到位 / 走反了 / 没越过噪声地板的,照收不误但不进推导。
/// 冻住手臂、只抖第 `手` 个末端的指通道。
/// 🔴 **哪个末端不写死** —— 由调用方逐个试(owner 2026-08-31:driver 不许有任何写死的东西)。
fn 抖指通道<S: std::io::Read + std::io::Write>(
    plug: &mut Plug<S>,
    相机们: &[(usize, point_gen::Eye, f64)],
    手: usize,
) -> body_layer::selfmodel::自模 {
    use body_layer::selfmodel::{一抖, 自模, 通道};
    let mut m = 自模::new();
    let Some(f0) = plug.sense() else { return m };
    let 冻 = match f0.ee.get(手) { Some(e) => [e[0], e[1], e[2]], None => return m };
    let 冻q = match f0.ee.get(手) { Some(e) => [e[3], e[4], e[5], e[6]], None => return m };
    let jaw0 = f0.jaw.get(手).or_else(|| f0.jaw.first()).copied().unwrap_or(1.0);
    // 走到位就停:剩下不足起点距离的 2%。不是"看起来不动了"——
    // 交付率低的身体上,"还没开始动"和"停了"在读数上同形(实测 +0.2 rad ⇒ 实到 +0.005)。
    let mut 落 = |plug: &mut Plug<S>, j: f64, 上限: u32| -> Option<Frame> {
        let mut 末 = None;
        let mut 起差: Option<f64> = None;
        for _ in 0..上限 {
            plug.act(&Cmd::Ee { arm: 手, at: 冻, quat: 冻q, jaw: j });
            let Some(f) = plug.sense() else { break };
            // 🔴 只看**这一只手**的指通道:双手身体上把两只都加起来,另一只永远不动 ⇒ 差永远不收敛。
            let d: f64 = f.jaw.get(手).or_else(|| f.jaw.first()).map(|g| (g - j).abs()).unwrap_or(0.0);
            let 起 = *起差.get_or_insert(d.max(1e-9));
            末 = Some(f);
            // 0.02 是**比例**(剩下不足起始距离的 2% 就算到位),无量纲。
        if d <= 0.02 * 起 { break }
        }
        末
    };
    let 灰 = |f: &Frame, ci: usize| f.cams.get(ci).map(|(w, h, d)| (*w, *h, d.clone()));
    let 基 = match 落(plug, jaw0, 200) { Some(f) => f, None => return m };
    let 静 = match 落(plug, jaw0, 6) { Some(f) => f, None => return m };
    let 反 = if jaw0 > 0.5 { 0.0 } else { 1.0 };
    let 动 = match 落(plug, 反, 300) { Some(f) => f, None => return m };
    let 实到 = 动.jaw.get(手).or_else(|| 动.jaw.first()).copied().unwrap_or(jaw0) - jaw0;
    for (ci, eye, _) in 相机们 {
        let (Some((w, h, gb)), Some((_, _, gs)), Some((_, _, gd))) =
            (灰(&基, *ci), 灰(&静, *ci), 灰(&动, *ci)) else { continue };
        let n = (w * h).min(gb.len()).min(gs.len()).min(gd.len());
        // 地板:两帧不动之间的最大差。地板是判据的一部分,不是背景信息。
        let mut 地板 = 0u8;
        for i in 0..n { let v = (gb[i] as i16 - gs[i] as i16).unsigned_abs() as u8; if v > 地板 { 地板 = v } }
        let (mut px, mut su, mut sv) = (0u32, 0f64, 0f64);
        let (mut u0, mut v0, mut u1, mut v1) = (1.0f64, 1.0f64, 0.0f64, 0.0f64);
        let mut 点: Vec<(f64, [f64; 3])> = Vec::new();
        let 深 = plug.lay.cams.get(*ci).and_then(|路| {
            let mut 深路 = 路.clone();
            // 🔴 **深度图的路径由 `discover` 认出来,不许把彩色路径的最后一段换成字面量 `depth`。**
            // 那是这一台机器的名字;换一台叫 `depth_image` / `range` 的就断了(owner 2026-08-28「减法①」)。
            if let Some(dp) = plug.lay.depth.get(*ci).or_else(|| plug.lay.depth.first()) { 深路 = dp.clone(); }
            plug.last.as_ref().and_then(|o| 取(o, &深路)).and_then(|dv| wire::as_f32_grid(&dv))
        });
        for i in 0..n {
            if (gb[i] as i16 - gd[i] as i16).unsigned_abs() as u8 <= 地板.saturating_add(2) { continue }
            px += 1;
            let (x, y) = (i % w, i / w);
            let (u, v) = (x as f64 / w as f64, y as f64 / h as f64);
            su += u; sv += v;
            if u < u0 { u0 = u } if u > u1 { u1 = u }
            if v < v0 { v0 = v } if v > v1 { v1 = v }
            if let Some((dw, dh, dep)) = &深 {
                let (dx, dy) = ((u * *dw as f64) as usize, (v * *dh as f64) as usize);
                if dx < *dw && dy < *dh {
                    let d = dep[dy * *dw + dx] as f64;
                    if d.is_finite() && d > 0.0 {
                        // 🔴 `back_project` 收的是**像素**(它内部拿 px−cx 去减主点,主点是像素值),
                        // 这里的 u,v 是归一化的 ⇒ 必须换算。传归一化进去 = 反投到几十厘米开外。
                        if let Ok(p) = eye.back_project([u * *dw as f64, v * *dh as f64], d) { 点.push((d, [p.x, p.y, p.z])); }
                    }
                }
            }
        }
        // 🔴 只留**近侧四分之一**再找最远点:包围盒里除了爪子还有两指之间和周围的桌面,
        // 均匀采会把桌面点选成"最远"(实测量出 0.3945,而桌面高度正是那个 z)。
        // 用的是一条现成的物理事实:**爪子离相机比桌子近**;分位数是数据自己的。
        let mut 远: Option<(f64, [f64; 3])> = None;
        if !点.is_empty() {
            let mut ds: Vec<f64> = 点.iter().map(|(d, _)| *d).collect();
            ds.sort_by(|a, b| a.partial_cmp(b).unwrap());
            let 近闸 = ds[ds.len() / 4];
            for (d, p) in &点 {
                if *d > 近闸 { continue }
                let r = ((p[0] - 冻[0]).powi(2) + (p[1] - 冻[1]).powi(2) + (p[2] - 冻[2]).powi(2)).sqrt();
                if 远.map(|(b, _)| r > b).unwrap_or(true) { 远 = Some((r, *p)); }
            }
        }
        m.收(一抖 {
            通道: 通道::指(0),
            命令: 反 - jaw0,
            实到,
            相机: *ci,
            动了多少像素: px,
            盒: [u0, v0, u1, v1],
            形心: if px > 0 { (su / px as f64, sv / px as f64) } else { (0.0, 0.0) },
            最远点: 远.map(|(_, p)| p),
            最远多远: 远.map(|(r, _)| r).unwrap_or(f64::NAN),
            地板,
        });
    }
    let _ = 落(plug, jaw0, 200);
    m
}

fn 存标定(
    out: &str,
    body: &body_layer::Body,
    相机们: &[(usize, point_gen::Eye, f64)],
    探幅: f64,
    跨度相机: usize,
    // 干活时量到的手的几何(腕系):(指尖偏移, 开合方向, 那一块占画幅多宽)。
    手: Option<([f64; 3], [f64; 3], f64)>,
    // 🔴🔴🔴 **干活时量到的这三样也要存 —— 它们是身体的属性,与任务无关。**(owner 2026-08-28)
    //
    // 查出来的断点:往身体里存东西的唯一入口是 `body.submit(...)`,而**干活模式一次都没调过它**
    //(全仓三个调用点:一个在读回时、两个在标定模式里)。于是干活时量到的东西全落在局部变量里,
    // 而本函数遍历 15 个格子时每一格都是"身体里没有就跳过" ⇒ 只写得出很早以前标定模式存的两格。
    // `image_jacobian` 之所以活下来,是因为它被当成**单独的顶层键**、绕开了 `body`。
    // ⇒ 这三样同样不是定长的"量",也走顶层键。
    通道表: Option<&[[f64; 6]]>,
    通道是关节: Option<bool>,
    手上相机: Option<usize>,
    // 🔴 拟出来的钳口张开(斜率×满开,截距吃掉面宽),不是那个"含面宽"的上界。
    张开: Option<f64>,
    // 🔴 **空手合到底时机体报的那个读数**。判"指间有没有东西"要拿它当零点 ——
    // 以前写死的是"停在 0 就是空的",而 **0 是这具 Franka 的约定,不是所有身体的**
    // (下一步就要零改动换 ARX 双臂)。量一次存住,换身体自动重量。
    空合读数: Option<f64>,
    // 🔴🔴 **"这台相机看不见我的钳口张合"** —— 一条量出来的身体事实。
    // XN 实测:一集 200 拍里约 180 拍花在"量自己",而认接触面在这台相机上**物理上不可能成功**
    //(张合只让 0–4 个像素变),却每一拍都试一次、每次 40 拍。量到了就存住,下一炮直接跳过。
    认不出接触面: Option<bool>,
    // 干活时量到的**画面雅可比**:Δ(画面横, 画面纵, 那一点的深) = 雅 · Δ(世界 x,y,z)。
    雅: Option<[[f64; 3]; 3]>,
) -> usize {
    // 🔴🔴 **落盘必须【合并】,不许用"此刻身体里有什么"改写整份文件。**
    // 实测(2026-08-24):上一炮量到的爪宽 0.085 m 存在文件里,这一炮开机只装回 7 格,
    // 第一次落盘就把文件从 11 格覆盖成 7 格 —— **一次成功的测量被一次不完整的开机抹掉**。
    // 这让"跨炮累积"变成了"跨炮侵蚀":每次上电只要有一格没装回来,它就从磁盘上永久消失。
    // ⇒ 先把磁盘上已有的读进来,身体里有的覆盖同名格,身体里没有的**原样留着**。
    let 旧 = std::fs::read_to_string(out).unwrap_or_default();
    let 旧格: Vec<(String, String)> = {
        let mut v = Vec::new();
        if let Some(i) = 旧.find("\"quantities\"") {
            let 段 = &旧[i..];
            // 每一格是 `    "名字": {...}` 一行(存标定自己就是这么写的)。
            for 行 in 段.lines() {
                let t = 行.trim();
                if !t.starts_with('"') { continue }
                let Some(j) = t[1..].find('"') else { continue };
                let 名 = t[1..1 + j].to_string();
                let Some(k) = t.find('{') else { continue };
                let 体 = t[k..].trim_end_matches(',').to_string();
                if 体.starts_with('{') && 体.ends_with('}') { v.push((名, 体)); }
            }
        }
        v
    };
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
    // 身体里没有、而磁盘上有的格,原样留着 —— 见函数头那段:不许把一次成功的测量抹掉。
    for (名, 体) in &旧格 {
        if body_layer::measurement::Quantity::ALL.iter().any(|q| q.as_str() == 名 && body.get(*q).is_some()) {
            continue;
        }
        if !first_q { j.push_str(",\n"); }
        first_q = false;
        j.push_str(&format!("    \"{名}\": {体}"));
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
    // 🔴🔴 **干活时量到的手的几何,也要存。**
    // 它们是**在腕系里表达的**,所以手腕怎么转都不用重量;换只手 / 抓着东西 / 撞坏了
    // 才会变,而那时**预测会对不上**,当场重量(见 `服务` 的"先预测再核对")。
    // 不存的话每抓一次都要完整晃一遍爪子 —— 那正是"看完压成一个数、把模型丢了"的老病。
    if let Some(h) = 手 {
        j.push_str(&format!(
            ",\n  \"hand\": {{\"tip_in_wrist\": [{}, {}, {}], \"open_in_wrist\": [{}, {}, {}], \"blob_frac\": {}}}",
            h.0[0], h.0[1], h.0[2], h.1[0], h.1[1], h.1[2], h.2));
    }
    // 🔴🔴 **画面雅可比也要存 —— 现量一次要挪三下、每下晃一遍爪子,吃掉整整一集。**
    // 存下来:下一集开场就有,直接伺服。**过时了不用怕** —— 每一步都拿它预测、拿观测核对,
    // 对不上就当场重量(相机动了 / 换了深度 / 换了只手,全归这一条)。
    if let Some(m) = 雅 {
        j.push_str(&format!(
            // 存成**九个数一行**(行优先)。嵌套数组用 `nums()` 读只会拿到第一列 —— 踩过。
            ",\n  \"image_jacobian\": [{}, {}, {}, {}, {}, {}, {}, {}, {}]",
            m[0][0], m[0][1], m[0][2], m[1][0], m[1][1], m[1][2], m[2][0], m[2][1], m[2][2]));
    }
    if let Some(t) = 通道表 {
        // 一行一个通道,每行六个数(两个接触面 × 横/纵/深)。行优先,和读回时一致。
        // 🔴 **存成一维、行优先** —— 仓里那条教训:*"嵌套数组用 `nums()` 读只会拿到第一列 —— 踩过"*。
        // 通道数 = 长度 ÷ 6,读回时自己算得出来,不用另存一个数。
        let 平: Vec<String> = t.iter().flat_map(|c| c.iter().map(|x| format!("{x}"))).collect();
        j.push_str(&format!(",\n  \"channel_table\": [{}]", 平.join(", ")));
    }
    if let Some(b) = 通道是关节 {
        j.push_str(&format!(",\n  \"channels_are_joints\": {b}"));
    }
    if let Some(c) = 手上相机 {
        j.push_str(&format!(",\n  \"camera_on_hand\": {c}"));
    }
    if let Some(v) = 张开 {
        j.push_str(&format!(",\n  \"jaw_span_m\": {v}"));
    }
    if let Some(v) = 空合读数 {
        j.push_str(&format!(",\n  \"jaw_closed_on_nothing\": {v}"));
    }
    if let Some(b) = 认不出接触面 {
        j.push_str(&format!(",\n  \"jaw_motion_invisible\": {b}"));
    }
    // 🔴🔴🔴 **顶层键也要【合并】,不许用"这次没量到"把上一炮量到的抹掉。**
    //
    // 上面那段已经把 `quantities` 合并了(2026-08-24 的教训:"跨炮累积"变成"跨炮侵蚀"),
    // **而顶层键漏了同一条**。实测代价(XA,2026-08-28):开机那次落盘走的是
    // `存标定(…, None, None, None, None, None, None, None)`,于是上一炮辛苦量到并存下的
    // **通道表 / 命令类型 / 手上相机 三样,在开机第一次落盘时被整片抹掉** ——
    // 而这三样正是"越用越强"的全部家当。病相还特别温和:日志里先印
    // `[装] 通道表装回:6 个通道`(装回来了,是真的),几行之后
    // `[装] 落盘(已量到 2 格)`(把文件写回去了,也是真的),**没有任何一行看起来不对**。
    // ⇒ 凡是这次没给值的顶层键,**原样从旧文件里抄回来**。
    for 键 in ["image_jacobian", "channel_table", "channels_are_joints",
               "camera_on_hand", "jaw_span_m", "jaw_closed_on_nothing", "jaw_motion_invisible", "hand"] {
        if j.contains(&format!("\"{键}\"")) { continue }
        let 找 = format!("\"{键}\"");
        let Some(k0) = 旧.find(&找) else { continue };
        // 从键名后面的冒号开始,抄到**同一层**的下一个逗号或收尾大括号为止。
        let 之后 = &旧[k0 + 找.len()..];
        let Some(c0) = 之后.find(':') else { continue };
        let 值段 = &之后[c0 + 1..];
        let (mut 深, mut 串, mut 转) = (0i32, false, false);
        let mut 终 = 值段.len();
        for (i, ch) in 值段.char_indices() {
            if 转 { 转 = false; continue }
            match ch {
                '\\' if 串 => 转 = true,
                '"' => 串 = !串,
                '[' | '{' if !串 => 深 += 1,
                ']' | '}' if !串 => {
                    if 深 == 0 { 终 = i; break }
                    深 -= 1;
                }
                ',' if !串 && 深 == 0 => { 终 = i; break }
                _ => {}
            }
        }
        let 值 = 值段[..终].trim();
        if 值.is_empty() { continue }
        println!("[装] 顶层键 \"{键}\" 这次没量到 ⇒ **从旧文件原样抄回**(不许把上一炮的抹掉)");
        j.push_str(&format!(",\n  \"{键}\": {值}"));
    }
    j.push_str("\n}\n");
    let n格 = j.matches("\"provenance\"").count();
    match std::fs::write(out, &j) {
        Ok(_) => n格,
        Err(e) => { println!("[装] 🔴 标定写不出去:{e} —— 这一轮的测量全丢了"); 0 }
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
    let mut 手载: Option<([f64; 3], [f64; 3], f64)> = None;
    let mut 雅载: Option<[[f64; 3]; 3]> = None;
    // 🔴 **通道表:6 行(两个接触面 × 三个数)× 通道数列。** 通道数由这具身体报,不设上限形状。
    let mut 通道表: Option<Vec<[f64; 6]>> = None;
    // 装回来的"哪台相机长在手上" —— 探一次要动一下手臂,存住就不用重探。
    let mut 手上相机装回: Option<usize> = None;
    let mut 张开装回: Option<f64> = None;
    let mut 空合装回: Option<f64> = None;
    let mut 认面装回 = false;
    // 这具身体的通道是哪一种 —— 关节,还是末端那六个自由度。**试出来的,不是假设的。**
    let mut 通道是关节 = true;
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
                    // 🔴🔴 **区分"用相机量出来的"和"值本身是相机坐标的"**(2026-08-24 owner 指出)。
                    //
                    // 上一版把**所有(传递地)依赖手眼的格**都不装回来,理由是"装回去的测量丢了依赖边,
                    // `DependencyMoved` 永远不会对它们触发"。那条理由**只对【值本身写在相机坐标里】的格成立** ——
                    // `hand_pixel` 是一个像素,相机一动它就不是同一件事了。
                    //
                    // 而 `gripper_span`(米)、`tool_offset`(米)、`tool_axis_column`(哪一列)、
                    // `self_occlusion` 是**这只手自己的物理事实**:相机动了,手不会变长。
                    // 它们被作废,只因为当初**是用相机量出来的** —— 那是**测量路径**,不是**物理依赖**。
                    //
                    // 🔴 对移动机体这条错误是致命的:人形的相机**每一秒都在动**,
                    // 于是"每次上电重量手眼"在真机上等于"每一帧都要重量",下游五格永远立不住,
                    // 驱动永远开不了工。实测(2026-08-23 N128):手眼重量一次 ⇒ 已经量到的
                    // 爪宽 0.085 m、工具偏置、工具轴当场作废 ⇒ 抓取被自己挡在门外。
                    //
                    // ⇒ 只有**值写在相机坐标里**的两格不装回来;以米/以轴计的物理事实照常装回。
                    fn 相机坐标里的(q: body_layer::measurement::Quantity) -> bool {
                        use body_layer::measurement::Quantity::{HandPixel, ImageJacobian};
                        q == ImageJacobian || q == HandPixel
                    }
                    if 相机坐标里的(q) {
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
                        // 🔴 装不回来必须点名。实测(2026-08-24):11 格只装回 7 格,
                        // 而**默默丢掉的那几格正好是最贵的**(爪宽 0.085 m 就这么没的),
                        // 然后增量落盘按"此刻身体里有什么"改写文件,把它从磁盘上也抹了。
                        // 一个 `.is_ok()` 吞掉的错误,让"越用越强"变成了"越用越少"。
                        match body.submit(m) {
                            Ok(_) => 装 += 1,
                            Err(e) => println!("[装] 🔴 {} 装不回来:{e:?}", q.as_str()),
                        }
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
                // 🔴 手的几何(腕系)也装回来 —— 干活那侧先拿它预测,核对得上就不用重晃。
                // 🔴 画面雅可比装回来 —— 干活那侧先拿它伺服,预测对不上就当场重量。
                if let Some(a) = j.get("image_jacobian") {
                    let r = a.nums();
                    if r.len() >= 9 && r.iter().all(|x| x.is_finite()) {
                        雅载 = Some([[r[0], r[1], r[2]], [r[3], r[4], r[5]], [r[6], r[7], r[8]]]);
                        println!("[装] 画面雅可比装回:世界往 +x 走 1 m ⇒ 画面 ({:+.3},{:+.3}) · 深 {:+.3}", r[0], r[3], r[6]);
                    }
                }
                // 🔴🔴🔴 **通道表 / 认哪种命令 / 哪台相机在手上,也装回来。**(owner 2026-08-28)
                // 这三样是**身体的属性、与任务无关**,而在此之前它们每炮都要从零重做:
                // 试七条关节命令才发现"一个都不响应"、重探"哪台长在手上"、重量整张通道表。
                // 存不进去的根因是干活模式从没调过 `body.submit`,而它们又不是定长的"量" ——
                // 所以和 `image_jacobian` 一样走顶层键。
                if let Some(a) = j.get("channel_table") {
                    // 存的是一行一个通道、每行六个数(两个接触面 × 横/纵/深),行优先。
                    let v = a.nums();
                    let mut t: Vec<[f64; 6]> = Vec::new();
                    let ok = v.len() >= 6 && v.len() % 6 == 0 && v.iter().all(|x| x.is_finite());
                    if ok {
                        for c in v.chunks_exact(6) { t.push([c[0], c[1], c[2], c[3], c[4], c[5]]); }
                    }
                    if ok && !t.is_empty() {
                        println!("[装] 通道表装回:{} 个通道 × 六行(两个接触面 × 横/纵/深)—— 不用重量", t.len());
                        通道表 = Some(t);
                    }
                }
                if let Some(b) = j.get("channels_are_joints") {
                    if let Some(v) = b.boolean() {
                        通道是关节 = v;
                        println!("[装] 这具机体认哪种命令:{} —— 不用再拿命令去撞一遍",
                            if v { "关节" } else { "末端那六个自由度" });
                    }
                }
                if let Some(v) = j.get("jaw_span_m") {
                    if let Some(x) = v.num() {
                        if x.is_finite() && x > 0.0 {
                            张开装回 = Some(x);
                            println!("[装] 钳口张开装回:{x:.4} m(拟出来的,不是含面宽的上界)—— 不用重量");
                        }
                    }
                }
                if let Some(v) = j.get("jaw_closed_on_nothing") {
                    if let Some(x) = v.num() {
                        if x.is_finite() {
                            空合装回 = Some(x);
                            println!("[装] 空手合到底的读数装回:{x:.4} —— 判「指间有没有东西」拿它当零点");
                        }
                    }
                }
                if let Some(v) = j.get("jaw_motion_invisible") {
                    if v.text() == Some("true") || v.num() == Some(1.0) {
                        认面装回 = true;
                        println!("[装] 上一炮量到:**这台相机看不见我的钳口张合** —— 直接走搬过去,那 40 拍留给走路");
                    }
                }
                if let Some(c) = j.get("camera_on_hand") {
                    if let Some(v) = c.num() {
                        手上相机装回 = Some(v as usize);
                        println!("[装] 长在手上的是第 {} 台相机 —— 不用重探", v as usize);
                    }
                }
                if let Some(h) = j.get("hand") {
                    let g = |k: &str| h.get(k).map(|x| x.nums()).unwrap_or_default();
                    let (t, o) = (g("tip_in_wrist"), g("open_in_wrist"));
                    let b = h.get("blob_frac").and_then(|x| x.num()).unwrap_or(0.0);
                    if t.len() >= 3 && o.len() >= 3 && b > 0.0 {
                        手载 = Some(([t[0], t[1], t[2]], [o[0], o[1], o[2]], b));
                        println!("[装] 手装回:指尖偏移(腕系) ({:.4},{:.4},{:.4}) · 开合 ({:.2},{:.2},{:.2}) · 那一块占画幅 {:.4}",
                            t[0], t[1], t[2], o[0], o[1], o[2], b);
                    }
                }
            }
        }
    }
    let mut plug = Plug { ws, lay, last: first, 待发: None, 上次发出: None, 复位过: false, 胶片: 0, 帧时秒: 0.0 };

    // ── 🔴🔴 **功能性自建模:`BL_SELFMODEL=<目录>`** ─────────────────────────────
    //
    // **机器人眼里的自己,不是长相,是一张因果表**:*我动这个通道 ⇒ 画面里这一块跟着动*。
    // 做法就是这一层一直在用的那条构造性身份 —— **别的全冻住,只动一个通道,
    // 会动的那一块【按构造】就是这个通道管的部件**。一根一根地动,攒起来就是
    // 一具**自己动出来的、带关节的身体**:没有 CAD、没有 URDF、没有一个人手填的数。
    //
    // 幅度不是身体常数:取**这个关节自己报的行程**的十分之一(机体自报限位只用来
    // 定"动多少",不进任何几何 —— 与 `Robot::identity` 的用法一致)。
    // 输出:每通道一张掩膜 + 一张彩色合成(每个通道一个颜色)+ 一张部件表。
    if let Ok(dir) = std::env::var("BL_SELFMODEL") {
        let _ = std::fs::create_dir_all(&dir);
        // ── 第一步:**先解眼睛。** ────────────────────────────────────────────
        //
        // 🔴 脱离相机的三维身体,前提是知道"我的眼在哪"。而这件事**不许问机体要**
        // (那不是自标定)—— 只能自己解:`fit_full_axis_offset` 只看着自己的手挪十几个
        // 地方,就把**焦距 / 主点 / 相机位姿 / 法兰到工作点的偏置**一次全解出来,
        // 而且样本共面 / 全在一个深度 / 不足数都会**拒绝**,不硬解。
        //
        // 「手在哪」本体感受白给;「手在画面哪一点」用同一条构造性身份拿:
        // **手臂冻住、只动爪子 ⇒ 会动的那一块就是手。**
        //
        // 幅度走**几何阶梯**(1 mm × 2ⁿ),挑第一档能让画面真的动起来的 ——
        // 换一具十倍大的身体自动给十倍的幅度,**不是照着这条臂挑的数**。
        let Some(f0) = plug.sense() else { println!("[自模] 拿不到第一帧"); std::process::exit(0) };
        // 🔴🔴🔴 **用哪个末端不许写死 —— 推一下,认这条命令的那个就是它。**
        //(owner 2026-08-31 死命令:"driver 不能有任何写死的东西,适用于所有奇形怪状的硬件")
        // 判据是**命令 vs 实到**:给每个末端发一个同样小的位移,谁真的走了谁就是能用的那只。
        // 单末端身体上这个循环只跑一次、必然选中它,行为不变。
        let 手 = {
            let n = f0.ee.len().max(1);
            let 步 = 0.01f64;   // 只用来问"你认不认这条命令",不做任何测量
            let mut 最 = (0usize, -1.0f64);
            for i in 0..n {
                let Some(e0) = f0.ee.get(i).cloned() else { continue };
                if e0.len() < 7 { continue }
                let j = plug.sense().and_then(|f| f.jaw.get(i).copied()).unwrap_or(1.0);
                plug.act(&Cmd::Ee { arm: i, at: [e0[0] + 步, e0[1], e0[2]],
                                    quat: [e0[3], e0[4], e0[5], e0[6]], jaw: j });
                for _ in 0..40 { if plug.sense().is_none() { break } }
                let d = plug.sense().and_then(|f| f.ee.get(i).cloned())
                    .map(|e| ((e[0]-e0[0]).powi(2)+(e[1]-e0[1]).powi(2)+(e[2]-e0[2]).powi(2)).sqrt())
                    .unwrap_or(0.0);
                println!("[自模] 第 {i} 个末端:命令走 {步:.4} m,实到 {d:.4} m");
                plug.act(&Cmd::Ee { arm: i, at: [e0[0], e0[1], e0[2]],
                                    quat: [e0[3], e0[4], e0[5], e0[6]], jaw: j });
                for _ in 0..40 { if plug.sense().is_none() { break } }
                if d > 最.1 { 最 = (i, d) }
            }
            println!("[自模] ⇒ **用第 {} 个末端**(它最认这条命令,实到 {:.4} m)", 最.0, 最.1);
            最.0
        };
        let Some(f0) = plug.sense() else { println!("[自模] 拿不到第一帧"); std::process::exit(0) };
        let 起 = match f0.ee.get(手) { Some(e) => [e[0], e[1], e[2]], None => { println!("[自模] 这台机器人不报末端位姿"); std::process::exit(0) } };
        let 起q = match f0.ee.get(手) { Some(e) => [e[3], e[4], e[5], e[6]], None => std::process::exit(0) };
        let jaw0 = f0.jaw.get(手).or_else(|| f0.jaw.first()).copied().unwrap_or(1.0);
        let 台 = f0.cams.len();
        println!("[自模] {} 个关节(读得到)· {} 个指通道 · {台} 台相机", f0.joints.first().map(|v| v.len()).unwrap_or(0), f0.jaw.len());
        println!("[自模] ⚠️ 关节角**读得到但动不了**:七根各命令 0.2 rad,回读四次逐位相同");
        println!("[自模]    ⇒ 通道 = 这具身体**真的接受**的命令(末端位姿 + 爪子),不是我以为它该有的关节");

        let mut 落E = |plug: &mut _, at: [f64; 3], j: f64, 上限: u32| -> Option<Frame> {
            let plug: &mut Plug<_> = plug;
            let mut 末 = None;
            let mut 起差: Option<f64> = None;
            for _ in 0..上限 {
                plug.act(&Cmd::Ee { arm: 手, at, quat: 起q, jaw: j });
                let Some(f) = plug.sense() else { break };
                let d = f.ee.get(手).map(|e| ((e[0]-at[0]).powi(2)+(e[1]-at[1]).powi(2)+(e[2]-at[2]).powi(2)).sqrt()).unwrap_or(f64::NAN);
                let s0 = *起差.get_or_insert(d.max(1e-9));
                末 = Some(f);
                // 0.02 是**比例**(剩下不足起始距离的 2% 就算到位),无量纲。
        if d <= 0.02 * s0 { break }
            }
            末
        };
        let 灰 = |f: &Frame, ci: usize| f.cams.get(ci).map(|(w, h, d)| (*w, *h, d.clone()));
        // 🔴🔴 **认那一块用 `blob::candidates`,不许自己写阈值差分。**
        //
        // 实测代价(2026-08-25):我手写"差值越过地板就算"⇒ 渲染噪声全放进来,
        // 「会动的那一块」的包围盒是 **1.4116 画幅**(整幅画的对角线才 1.414)——
        // 铺满了整张图,形心毫无意义。而仓里那个认块器本来就带**噪声地板 + 连通 + 刚性**,
        // 而且**认不出来会拒绝**。⚠️ 档案原话:「重造轮子第 4 次未遂」—— 这是第 5 次。
        //
        // 它要的是**五帧合同**:两帧不动(量这台相机自己的地板)+ 三帧等步长(同一个命令走三次)。
        let mut 晃爪 = |plug: &mut _, at: [f64; 3]| -> Vec<Option<body_layer::hand::Candidate>> {
            let plug: &mut Plug<_> = plug;
            let mut out: Vec<Option<body_layer::hand::Candidate>> = vec![None; 台];
            // 0.30 是**钳口命令行程的分数**(命令域本身就是 0..1),无量纲。
    let 步 = (if jaw0 > 0.5 { -1.0 } else { 1.0 }) * 0.30;
            let mut 帧们: Vec<Frame> = Vec::new();
            let Some(a) = 落E(plug, at, jaw0, 200) else { return out };
            帧们.push(a);
            let Some(b) = 落E(plug, at, jaw0, 6) else { return out };
            帧们.push(b);
            for k in 1..=3 {
                let Some(f) = 落E(plug, at, (jaw0 + 步 * k as f64).clamp(0.0, 1.0), 200) else { return out };
                帧们.push(f);
            }
            let _ = 落E(plug, at, jaw0, 200);
            if 帧们.len() < 5 { return out }
            for ci in 0..台 {
                let Some((w, h, _)) = 灰(&帧们[0], ci) else { continue };
                let g: Vec<Vec<u8>> = 帧们.iter().filter_map(|f| 灰(f, ci).map(|(_, _, d)| d)).collect();
                if g.len() < 5 { continue }
                match body_layer::blob::candidates(&g[0], &g[1], &g[2], &g[3], &g[4], w, h,
                        步.abs(), selfcal::最少像素(w, h)) {
                    Ok(r) => {
                        // 有配对用配对(两瓣反向抵消 = 这副钳口是对开的证据),没配对用最大那块。
                        out[ci] = r.cands.get(0).copied();
                        println!("[自模]     第 {ci} 台:双响 {} · 配对 {} · 地板 {} · 块 {}",
                            r.moved_px, r.pairs, r.floor, r.cands.len());
                    }
                    Err(e) => println!("[自模]     第 {ci} 台:认块器拒绝 {e:?}"),
                }
            }
            out
        };

        // 几何阶梯挑幅度:第一档能让手在画面里真挪起来的
        // 🔴 这是**阶梯的起跳档**,不是结论:下面 `'ladder` 每轮 `幅 *= 2`,十轮能从
        // 1 mm 长到 1 m,**哪一档算数由「手在画面里真的动了」决定,不由这个数决定**。
        // 起跳点小只会多试几轮,不会把任何身体挡在外面 ⇒ 它是**协议**起点,不是身体断言。
        let mut 幅 = 0.001f64;
        let mut 样本: Vec<Vec<([f64; 7], point_gen::Px)>> = vec![Vec::new(); 台];
        'ladder: for _ in 0..10 {
            样本.iter_mut().for_each(|v| v.clear());
            // 🔴 **每一档先只探两个位置。** 手在画面里看不看得见靠的是**晃爪子**,
            // 跟末端挪多远无关;末端幅度只影响**拟合的几何**(要不共面、深度要有跨度)。
            // ⇒ 一档的代价从 12 个位置降到 2 个:两点的手像素挪不开这一档就没用,翻倍重来。
            // 实测代价(2026-08-25):不这么做,从 1 mm 起步一档要跑满 12 个位置 ≈ 8 分钟,
            // 十档就是一小时二十分,而前几档注定没用。
            {
                let a = 晃爪(&mut plug, [起[0] - 幅, 起[1], 起[2]]);
                let b = 晃爪(&mut plug, [起[0] + 幅, 起[1], 起[2]]);
                let mut 挪开 = false;
                for ci in 0..台 {
                    let (Some(ca), Some(cb)) = (a[ci], b[ci]) else { continue };
                    let d = ((ca.u - cb.u).powi(2) + (ca.v - cb.v).powi(2)).sqrt();
                    // 判据不带常数:挪开的距离要比**认块器自己报的定位散布**大 ——
                    // 比它还小,和"认块认歪了一点"分不开。
                    let σ = ca.spread.max(cb.spread).max(1e-6);
                    println!("[自模]   幅度 {:.4} m:第 {ci} 台手像素挪了 {:.4} 画幅(认块器自报散布 {:.4})", 幅, d, σ);
                    if d > σ { 挪开 = true }
                }
                if !挪开 {
                    // 🔴 **到不了就别再翻。**(同 AS 那条毁桌梯子的病:命令越顶不动、越使劲推)
                    // 命令走 `幅` 而实际连一半都没走到 ⇒ 这个方向已经顶住了(够不着 / 撞上了),
                    // 再翻倍只是把整条胳膊往那个方向甩得更远。命令 vs 实到,两个都是量出来的。
                    let 到了 = plug.sense().and_then(|f| f.ee.get(手).copied())
                        .map(|e| ((e[0]-起[0]).powi(2)+(e[1]-起[1]).powi(2)+(e[2]-起[2]).powi(2)).sqrt())
                        .unwrap_or(幅);
                    if 到了 < 幅 * 0.5 {
                        println!("[自模]   命令走 {幅:.4} m,实际只到了 {到了:.4} m ⇒ **这个方向已经够不着了**,不再翻倍");
                        break 'ladder
                    }
                    幅 *= 2.0; continue 'ladder
                }
            }
            // 立方体八角 + 两个面心 —— 是**协议选择**(要不共面),不是身体常数;
            // 尺寸由阶梯给。fit_full_axis_offset 要 ≥12 个样本。
            let 角: Vec<[f64; 3]> = {
                let mut v = Vec::new();
                for sx in [-1.0, 1.0f64] { for sy in [-1.0, 1.0f64] { for sz in [-1.0, 1.0f64] {
                    v.push([起[0] + sx * 幅, 起[1] + sy * 幅, 起[2] + sz * 幅]);
                }}}
                for s in [-1.5, 1.5f64] { v.push([起[0] + s * 幅, 起[1], 起[2]]); v.push([起[0], 起[1] + s * 幅, 起[2]]); }
                v
            };
            let mut 见 = 0u32;
            for (k, at) in 角.iter().enumerate() {
                let r = 晃爪(&mut plug, *at);
                println!("[自模]   第 {}/{} 个位置", k + 1, 角.len());
                let 实 = plug.sense().and_then(|f| f.ee.get(手).copied());
                for ci in 0..台 {
                    let Some(c) = r[ci] else { continue };
                    if let Some(e) = 实 { 样本[ci].push((e, [c.u, c.v])); 见 += 1; }
                }
            }
            println!("[自模] 幅度 {:.4} m ⇒ 收到 {见} 条(手在哪 ↔ 手在画面哪)", 幅);
            if 样本.iter().any(|v| v.len() >= 12) { break 'ladder }
            幅 *= 2.0;
        }
        let mut 眼们: Vec<(usize, point_gen::Eye, f64, usize, f64)> = Vec::new();
        for ci in 0..台 {
            match point_gen::fit_full_axis_offset(&样本[ci]) {
                Ok((eye, 轴, t, 留出)) => {
                    println!("[自模] 🟢 第 {ci} 台眼睛**自己解出来了**:fx={:.4} 主点=({:.3},{:.3}) 眼在 ({:.3},{:.3},{:.3}) · 法兰→工作点沿第 {轴} 列 {t:.4} m · 留出中位 {留出:.4}",
                        eye.fx, eye.cx, eye.cy, eye.at[0], eye.at[1], eye.at[2]);
                    眼们.push((ci, eye, t, 轴, 留出));
                }
                Err(e) => println!("[自模] 🔴 第 {ci} 台眼睛解不出来:{e:?}(样本 {} 条)", 样本[ci].len()),
            }
        }
        if 眼们.is_empty() {
            println!("[自模] 一只眼都没解出来 ⇒ 出不了三维身体。**不编数,就说解不出来。**");
            std::process::exit(0);
        }
        // ── 第二步:**建三维身体。** 抖每一个真的动得了的通道,把那一块反投影成世界点。
        let 通道们: Vec<(&str, [f64; 3], f64)> = vec![
            ("末端+x", [幅, 0.0, 0.0], jaw0),
            ("末端+y", [0.0, 幅, 0.0], jaw0),
            ("末端+z", [0.0, 0.0, 幅], jaw0),
            ("爪子",   [0.0, 0.0, 0.0], if jaw0 > 0.5 { 0.0 } else { 1.0 }),
        ];
        let Some(基) = 落E(&mut plug, 起, jaw0, 200) else { std::process::exit(0) };
        let Some(静) = 落E(&mut plug, 起, jaw0, 6) else { std::process::exit(0) };
        let mut 文 = String::new();
        for (ci, eye, _, _, _) in &眼们 {
            文.push_str(&format!("眼cam{ci} {:.5} {:.5} {:.5}\n", eye.at[0], eye.at[1], eye.at[2]));
        }
        for (名, d, j) in &通道们 {
            let at = [起[0] + d[0], 起[1] + d[1], 起[2] + d[2]];
            let Some(动) = 落E(&mut plug, at, *j, 300) else { continue };
            for (ci, eye, _, _, _) in &眼们 {
                let (Some((w, h, gb)), Some((_, _, gs)), Some((_, _, gd))) = (灰(&基, *ci), 灰(&静, *ci), 灰(&动, *ci)) else { continue };
                let 深 = plug.lay.cams.get(*ci).and_then(|路| {
                    let mut 深路 = 路.clone();
                    // 🔴 **深度图的路径由 `discover` 认出来,不许把彩色路径的最后一段换成字面量 `depth`。**
        // 那是这一台机器的名字;换一台叫 `depth_image` / `range` 的就断了(owner 2026-08-28「减法①」)。
        if let Some(dp) = plug.lay.depth.get(*ci).or_else(|| plug.lay.depth.first()) { 深路 = dp.clone(); }
                    plug.last.as_ref().and_then(|o| 取(o, &深路)).and_then(|dv| wire::as_f32_grid(&dv))
                });
                let n = (w*h).min(gb.len()).min(gs.len()).min(gd.len());
                let mut 地板 = 0u8;
                for i in 0..n { let v = (gb[i] as i16 - gs[i] as i16).unsigned_abs() as u8; if v > 地板 { 地板 = v } }
                let mut 数 = 0u32;
                if let Some((dw, dh, dep)) = &深 {
                    for i in 0..n {
                        if (gb[i] as i16 - gd[i] as i16).unsigned_abs() as u8 <= 地板.saturating_add(2) { continue }
                        let (u, v) = ((i % w) as f64 / w as f64, (i / w) as f64 / h as f64);
                        let (dx, dy) = ((u * *dw as f64) as usize, (v * *dh as f64) as usize);
                        if dx >= *dw || dy >= *dh { continue }
                        let dd = dep[dy * *dw + dx] as f64;
                        if !(dd.is_finite() && dd > 0.0) { continue }
                        if let Ok(p) = eye.back_project([u * *dw as f64, v * *dh as f64], dd) {
                            文.push_str(&format!("{名} {:.5} {:.5} {:.5}\n", p.x, p.y, p.z));
                            数 += 1;
                        }
                    }
                }
                println!("[自模]   {名} · 第 {ci} 台 ⇒ {数} 个三维点(地板 {地板})");
            }
            let _ = 落E(&mut plug, 起, jaw0, 300);
        }
        let _ = std::fs::write(format!("{dir}/身体.xyz"), 文);
        println!("[自模] 三维身体 ⇒ {dir}/身体.xyz");
        println!("[自模] 完事。");
        std::process::exit(0);
    }

    // ── 🔴 看一眼钳口:BL_JAWLOOK=<目录> ────────────────────────────────
    // 为什么要有这个:跨度那一格的读数是「开度 0.34 与 0.45 量出同一个间距」,
    // 而钳口回读**已证实只是命令的回声**(命令 0.78 ⇒ 回读 0.7799999999999999)⇒
    // **没有任何非视觉的办法**分开"钳口没动"和"我认错了两个瓣"。仓规 §3.6:
    // 空间/视觉的毛病,第一动作是渲图去看,不是接着推。
    // 手臂全程不动,只扫开度;每一档停稳后把每台相机各落一张 PGM。
    if let Ok(dir) = std::env::var("BL_JAWLOOK") {
        let _ = std::fs::create_dir_all(&dir);
        // 🔴 **用哪个末端不写死** —— 同上:发一个同样小的位移,谁真的走了就是谁。
        //(owner 2026-08-31 死命令。单末端身体上必然选中那一个,行为不变。)
        let 手 = {
            let n = plug.sense().map(|f| f.ee.len()).unwrap_or(1).max(1);
            let mut 最 = (0usize, -1.0f64);
            for i in 0..n {
                let Some(e0) = plug.sense().and_then(|f| f.ee.get(i).cloned()) else { continue };
                if e0.len() < 7 { continue }
                let j = plug.sense().and_then(|f| f.jaw.get(i).copied()).unwrap_or(1.0);
                plug.act(&Cmd::Ee { arm: i, at: [e0[0] + 0.01, e0[1], e0[2]],
                                    quat: [e0[3], e0[4], e0[5], e0[6]], jaw: j });
                for _ in 0..40 { if plug.sense().is_none() { break } }
                let d = plug.sense().and_then(|f| f.ee.get(i).cloned())
                    .map(|e| ((e[0]-e0[0]).powi(2)+(e[1]-e0[1]).powi(2)+(e[2]-e0[2]).powi(2)).sqrt())
                    .unwrap_or(0.0);
                plug.act(&Cmd::Ee { arm: i, at: [e0[0], e0[1], e0[2]],
                                    quat: [e0[3], e0[4], e0[5], e0[6]], jaw: j });
                for _ in 0..40 { if plug.sense().is_none() { break } }
                println!("[看钳口] 第 {i} 个末端:命令走 0.0100 m,实到 {d:.4} m");
                if d > 最.1 { 最 = (i, d) }
            }
            println!("[看钳口] ⇒ **用第 {} 个末端**(它最认这条命令)", 最.0);
            最.0
        };
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
            let mut q回 = plug.sense().and_then(|f| f.ee.get(手).map(|e| [e[3], e[4], e[5], e[6]])).unwrap_or([1.0, 0.0, 0.0, 0.0]);
            for _ in 0..200 {
                plug.act(&Cmd::Ee { arm: 手, at: p, quat: q回, jaw: 1.0 });
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
            Some(f) if !f.ee.is_empty() => [f.ee[手][0], f.ee[手][1], f.ee[手][2]],
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
            let q回2 = plug.sense().and_then(|f| f.ee.get(手).map(|e| [e[3], e[4], e[5], e[6]])).unwrap_or([1.0, 0.0, 0.0, 0.0]);
            plug.act(&Cmd::Ee { arm: 手, at: home, quat: q回2, jaw: 1.0 });
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
    // 自模:身体知识的因果那一半。缺几何格时现抖现收,不再各量各的。
    let mut 自模 = body_layer::selfmodel::自模::new();

    // 🔴🔴🔴 **没有「日程」了(owner 2026-08-25 定)。**
    //
    // 旧形状:开机先按一张日程把 15 格挨个量一遍,量完才进干活。
    // 代价照记:N128–N143 共 **14 炮,进入干活模式 0 次** —— 用户让它拿东西,
    // 它先坐下来量自己;而评测里每 200 步打断一次,于是**永远量不完,也永远不干活**。
    // 而且「装机时量一次、永久有效」本身是个手填的假设:给宇树换只灵巧手、
    // 给机器人挂个武器,爪宽和指尖长**当场全变**,时间型的过期管不住它。
    //
    // 新形状:**一个入口 —— 下命令就去干。** 干到需要某个身体量而手上没有,
    // 就在那一刻**点名要它**,由这里量完再回去接着干。量出来的东西就等于原来
    // 「装机那一次」量的,只是它长在任务里,不单独占一段时间。
    let mut 点名: Option<Quantity> = None;
    // 🔴 **同一格要过几次而始终量不到 ⇒ 这具身体给不出它。** 不再要,让干活那侧降级。
    // 没有这一条就是死循环:点名 → 量 → 量不到 → 再点名同一个(新架构第一炮实测 6 轮)。
    // 3 是**协议数**(重复打靶而不是赌单轮),与"重跑一轮补拒格 ≤3"同源。
    let mut 要过: std::collections::BTreeMap<&'static str, u32> = Default::default();
    let mut 给不出: std::collections::BTreeSet<&'static str> = Default::default();
    // 干活那侧点名要哪一格 —— 名字 ⇒ 格。不在这张表里的(相机 / 旁注)不是"格",
    // 它们缺了只能具名拒绝,点名要不来。
    fn 点名成格(名: &str) -> Option<Quantity> {
        use Quantity::*;
        Some(match 名 {
            "home_pose" => HomePose, "latency" => Latency, "backlash" => Backlash,
            "reach" => Reach, "arm_weight" => ArmWeight, "step_delivery" => StepDelivery,
            "image_jacobian" => ImageJacobian, "hand_pixel" => HandPixel,
            "gripper_span" => GripperSpan, "contact_threshold" => ContactThreshold,
            "self_occlusion" => SelfOcclusion, "tool_offset" => ToolOffset,
            "tool_axis_column" => ToolAxisColumn, "floor" => Floor, "friction" => Friction,
            _ => return None,
        })
    }
    let _ = 点名成格;

    '外: loop {
    loop {
        let now = 轮 as u64 + 1;
        let _ = now;
        // 🔴 只量【干活那侧点名要的】那一格。没人点名 ⇒ 一格都不量,直接去干活。
        let 下一格 = 点名.take().map(|q| (q, body_layer::schedule::Need::NeverMeasured));
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
        if 轮 > 40 + 40 * 重轮 {
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
        // 🔴🔴 **等到"低于自己量得出的噪声"就够了,不是等到千分之一。**
        //
        // 上一版传的是写死的 `1e-3`:一步 13.6 mm 的千分之一 = **0.014 mm**,
        // 比这具身体任何一个读数的噪声都小两个量级 —— 后面几十拍全在等一个公式,
        // 而公式假设的等比收敛在真机上早就到底了。实测(2026-08-24,交付 0.063):
        // 静置 **107 拍**,一个采样点 113 拍,自标定因此以小时计。
        //
        // 容差改成**这具身体自己量到的位置噪声 ÷ 这一相的命令幅度**:再等下去也分辨不出来。
        // 噪声取已量到的里最小的那个位置 σ(回差是纯位置尺度的量);一格都没有就退回旧的 1e-3。
        // 上下夹在 [1e-3, 0.2]:下界就是旧行为,上界防止一具噪声极大的身体把静置压成 0。
        let 容差 = {
            let σ = body.get(Quantity::Backlash)
                .filter(|m| m.dim >= 1 && m.uncertainty[0].is_finite() && m.uncertainty[0] > 0.0)
                .map(|m| m.uncertainty[0]);
            let 幅 = body.get(Quantity::Reach)
                .filter(|m| m.dim >= 2 && m.value[1] > m.value[0])
                // 10 是**比例**(量出来那条带宽的十分之一),不是长度。
        // 10 是**比例**(量出来那条带宽的十分之一),不是长度。
        .map(|m| (m.value[1] - m.value[0]) / 10.0);
            match (σ, 幅) {
                (Some(σ), Some(a)) if a > 0.0 => {
                    // σ/a 本身是**比值**,上下限自然也是无量纲的。
        let t = (σ / a).clamp(1e-3, 0.2);
                    println!("      [静置] 容差按量到的噪声给:σ={σ:.5} m / 幅 {a:.5} m ⇒ {t:.4}(旧的写死值是 0.001)");
                    t
                }
                _ => 1e-3,
            }
        };
        let 静置 = match body_layer::derive::settle_periods(&body, 容差) {
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
            println!("      [协议] 静置 {静置} 拍(由这具身体的延迟+交付率推出),这一格周期 {} 拍", selfcal::周期(q, 静置));
        }
        // 🔴🔴 **步数预算按【要采几个循环】给,不是按固定拍数给。**(2026-08-24,owner 死线:自检十分钟)
        //
        // 上面那套固定拍数(300/1000/90 × 交付率倒数)是在**静置 107 拍**的年代定的。
        // 静置降到个位数之后,那套预算比需要的大 4–5 倍,而**没有任何一处跟着改** ——
        // 这正是本仓最贵的那种病:参数变了,依赖它的数没跟着变。
        // 一个循环 = 一次采样 = `静置 + 6` 拍。估计器要几个点,就给几个循环,乘出来就是预算。
        // 循环数按各自估计器的**最低门槛**给(跨度要每个腕角 ≥5 点 × 4 个角、产出率约 8 成 ⇒ 30)。
        // 少采的代价是常数更噪 —— 而噪到不能用时**闸会点名拒绝,不会编数**,这是能接受的失败方式。
        let 循环数: u32 = match q {
            Quantity::ImageJacobian | Quantity::HandPixel => 30,
            // 🔴 钳口:一个循环 = 一个开度档,一个腕角走满 `钳口窗数`(10)档,四个腕角 ⇒ 40。
            // ⚠️ **撤回"再多给几十个循环就够了"**(2026-08-24 上午写的 60):产出率低不是
            // 轮数问题 —— 是钳口每一档只给一拍、爪子只走了 4% 行程(见 `selfcal::周期`)。
            // 波形改成每档保持 `静置` 拍之后,一档就是一个真样本,40 个循环刚好走满一遍。
            Quantity::GripperSpan => std::env::var("BL_SPAN_CYCLES").ok()
                .and_then(|v| v.parse::<u32>().ok()).unwrap_or(40),
            Quantity::SelfOcclusion | Quantity::ArmWeight => 15,
            // 🔴 工具轴/工具长 8 → 30(2026-08-25 炮1 实测):8 个循环只攒出
            // **三列共 4 个弧点**,而估计器要每一列各拟一段弧 ⇒ 当场拒。
            // 产出是「每个循环最多一个弧点、还要分摊到三列」,8 个循环从设计上就够不着 ——
            // 与钳口那一格「6 档探针配 5 条门槛」是同一种病。
            Quantity::ToolAxisColumn | Quantity::ToolOffset => 30,
            _ => 8,
        };
        // 🔴 周期由 selfcal 说了算(跨度那一相和别的不一样,见 `selfcal::周期`)——
        // 在这里再写一次 `静置 + 6` 就是"同一个量出现在两处",而两处一定会分岔。
        // 🔴🔴 **预算要取【周期】和【静置+6】里大的那个,不能直接用周期。**
        // 实测(2026-08-24,N142,我自己当天下午改出来的回退):`周期()` 对**不走认块器**的格
        // 返回 **1**(它们没有"空转两拍+晃钳口三拍"那套协议),于是预算 `循环数 × 周期`
        // 从原来的 8×68=544 拍**塌成 8 拍** —— 采两个样本就被判 `NotEnoughSamples`。
        // 后果:原位/齿隙/可达/臂重/接触阈/地板/摩擦**七格全线退化**,当天早上还能量到 8 格,
        // 改完只剩 5 格,而**掉的每一格都是不走相机的那一类**,病相看起来像"这具身体变差了"。
        // ⇒ 跨度那一相保留它加长后的周期(112 拍),其余格拿回 `静置 + 6` 的预算。
        let 周期 = selfcal::周期(q, 静置).max(静置 + 6);
        let 步 = 循环数 * 周期;
        println!("      [预算] {} 个循环 × 周期 {周期} = {步} 拍", 循环数);
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
                        // 🔴 机体自报的标定优先(见 `自报相机` 上面那段);报了才不用去解。
                        let 自报 = plug.lay.cams.get(i).and_then(|路| {
                            plug.last.as_ref().and_then(|o| {
                                自报相机(
                                    o,
                                    路,
                                    body.get(Quantity::HandPixel).filter(|m| m.dim >= 2).map(|m| (m.value[0], m.value[1])),
                                    plug.lay.ee.first().map(|v| v.as_slice()),
                                    plug.lay.depth.get(i).or_else(|| plug.lay.depth.first()).map(|v| v.as_slice()),
                                )
                            })
                        });
                        if let Some((eye, 米每单位)) = 自报 {
                            println!("      [自报相机] 第 {i} 台用**机体自己发布的**标定:fx={:.3} fy={:.3} 主点=({:.3},{:.3}) 相机在 ({:.3},{:.3},{:.3}) ⇒ 手那个深度上 1 归一化单位 = {:.5} m",
                                eye.fx, eye.fy, eye.cx, eye.cy, eye.at[0], eye.at[1], eye.at[2], 米每单位);
                            相机们.retain(|c| c.0 != i);
                            相机们.push((i, eye, 米每单位));
                            let 局 = 本相尺(&s.cam_shift, i);
                            for &θ100 in &腕角集 {
                                可用.push((腕角集.len(), i, θ100, Some(米每单位), 局));
                            }
                            continue;
                        }
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
            // 🔴🔴 **工具那两格改从【自模】读 —— 不再绕手腕扫弧。**
            //
            // 扫弧那条路要先知道"哪一列是工具轴",而工具长又要先知道工具轴 ⇒ 两个未知数互相咬;
            // 实测(2026-08-25,Franka):给到 30 个循环、弧点稳定产出,估计器仍判 **0.0001 m**
            // ——读成"这具身体没有工具",而它明明有。
            //
            // 自模那条路一个约定都不需要:**冻住别的、只动指通道 ⇒ 会动的那一块按构造就是钳口**,
            // 那一块的**远端**就是指尖(到末端的距离 = 工具长),那一块**沿哪一维长出去**就是开合方向。
            // 「第几列」这个问题在这个形状下**根本不存在**,于是文档 §六 那条
            // 「两边列的约定不一样 ⇒ 爪子歪 90°,而两侧日志全绿」也没有地方长。
            Quantity::ToolOffset => {
                if !自模.答得上(Quantity::ToolOffset) {
                    // 🔴 **抖哪一只手不写死:身体报几个末端就逐个抖,哪个抖出通道就用哪个。**
                    //(owner 2026-08-31 死命令。单手身体上这个循环只跑一次,行为不变。)
                    let 手数 = plug.sense().map(|f| f.ee.len()).unwrap_or(1).max(1);
                    println!("      [自模] 先抖一下指通道(冻住手臂,只动爪子)—— 这具身体报了 {手数} 个末端,逐个试");
                    for 手 in 0..手数 {
                        let 新 = 抖指通道(&mut plug, &相机们, 手);
                        let 条 = 新.证据.len();
                        for e in 新.证据 { 自模.收(e); }
                        println!("      [自模]   第 {手} 个末端抖出 {条} 条证据");
                        if 自模.答得上(Quantity::ToolOffset) { break }
                    }
                }
                if !自模.答得上(Quantity::ToolOffset) {
                    println!("      [自模] 抖完仍答不上 —— 见过的通道 {:?},算数 {} 条",
                        自模.见过的通道(), 自模.算数几条(body_layer::selfmodel::通道::指(0)));
                    Err(probe::Declined::NotEnoughSamples)
                } else {
                let d = 自模.工具长().unwrap();
                println!("      [自模] 法兰到指尖 = {d:.4}(指通道那一块的远端到末端;{} 条算数证据)",
                    自模.算数几条(body_layer::selfmodel::通道::指(0)));
                let mut m = body_layer::measurement::Measurement::blank_for(Quantity::ToolOffset, 1, now);
                m.value[0] = d;
                // 0.1 是**比例**(把不确定度取成读数的一成),无量纲。
        m.uncertainty[0] = d * 0.1;
                m.valid_lo[0] = 0.0;
                m.valid_hi[0] = d * 4.0;
                m.selftest_passed = true;
                Ok(m)
                }
            }
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
                        // 10 是**比例**(量出来那条带宽的十分之一),不是长度。
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
        // 🔴 每一轮结束就落一次盘 —— 见 `存标定` 上面那段:只在末尾存等于赌这一炮能跑到底。
        let n格 = 存标定(&out, &body, &相机们, 探幅, 跨度相机, None, None, None, None, None, None, None, None);
        println!("      [存] 已落盘 {n格} 格 ⇒ {out}");
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
        // (续上)这两个也是**演示问句的内容**(「走一步 2 cm,要准到 1 mm」),
        // 是**问题**不是身体断言 —— 协议示例,答案由准入闸给。
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
    let n格 = 存标定(&out, &body, &相机们, 探幅, 跨度相机, None, None, None, None, None, None, None, None);
    println!("[装] 落盘(已量到 {n格} 格 · 本次点名量到 {} 格)", 成.len());

    // ── 🔴🔴 **下命令就去干。** 干到缺某个身体量,它点名要,回上面量完再回来。 ──
    // 观测里给什么指令,就做什么;没有任务名,没有机体名。
    let (眼主机, 眼端口) = match 眼.rsplit_once(':') {
        Some((h, p)) => (h.to_string(), p.parse::<u16>().unwrap_or(8077)),
        None => (眼.clone(), 8077),
    };
    match 服务(&mut plug, &body, &相机们, &眼主机, 眼端口, &out, 读回.as_deref(), &给不出, &mut 手载, &mut 雅载, &mut 通道表, &mut 通道是关节, &mut 手上相机装回, &mut 张开装回, &mut 空合装回, &mut 认面装回) {
        None => break '外,
        Some(名) => match 点名成格(&名) {
            Some(q) => {
                // 🔴🔴 **点名要 X 之前,先把 X 的前置补上。**
                //
                // 依赖表说的是"哪个量是拿哪个量表达的" —— 手点是相机系里的一个像素,没有手眼
                // 账本就无从表达;接触阈读的是**交付比例**,没量过每拍走多远就没有分母。
                // 这些是**量之间**的事实,不是某具身体的常数(换一具机体,一行都不改)。
                //
                // 实测代价(2026-08-25,新架构第一炮):拆日程时把用依赖表的那一行也删了,
                // 于是干活那侧点名要 `contact_threshold`,而前置 `step_delivery` 还没量
                // ⇒ 静置和周期算出来只有 **10 拍**、总预算 **80 拍** ⇒ 采不到 ⇒ 量不出来
                // ⇒ 再点名同一个 ⇒ **六轮死循环**,而每一行日志单看都正常。
                fn 补前置(q: Quantity, body: &body_layer::Body, 链: &mut Vec<Quantity>) {
                    for d in body_layer::schedule::prerequisites(q) {
                        if body.get(*d).is_none() && !链.contains(d) {
                            补前置(*d, body, 链);
                            链.push(*d);
                        }
                    }
                }
                let mut 链: Vec<Quantity> = Vec::new();
                补前置(q, &body, &mut 链);
                let 先 = 链.first().copied();
                match 先 {
                    Some(p) => println!("[装] ⤴ 要 **{}**,而它要先有 **{}** ⇒ 先量前置(整条链 {:?})",
                        q.as_str(), p.as_str(), 链.iter().map(|x| x.as_str()).collect::<Vec<_>>()),
                    None => println!("[装] ⤴ 干活那侧点名要 **{}** ⇒ 就地量它,量完回去接着干", q.as_str()),
                }
                let 这次 = 先.unwrap_or(q);
                let n = 要过.entry(这次.as_str()).or_insert(0);
                *n += 1;
                if *n > 3 {
                    println!("[装] 🔴 **{}** 要过 {} 次仍然量不到 ⇒ 这具身体给不出它。", 这次.as_str(), *n);
                    println!("[装]    不再要它,让干活那侧带着这个缺口降级干活 —— 缺东西的代价是**慢**,不是**不能**。");
                    给不出.insert(这次.as_str());
                    点名 = None;
                    continue '外;
                }
                点名 = Some(这次);
                成.clear();
                拒过.clear();
            }
            None => {
                println!("[装] 🔴 干活那侧缺的是 **{名}** —— 它不是一个可以点名去量的格,具名拒绝。");
                break '外;
            }
        },
    }
    }
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
/// 3×5 的数字点阵 —— 只为在候选框上写编号。**十个字模是字形,不是身体假设。**
const 字模: [[u8; 5]; 10] = [
    [0b111, 0b101, 0b101, 0b101, 0b111], // 0
    [0b010, 0b110, 0b010, 0b010, 0b111], // 1
    [0b111, 0b001, 0b111, 0b100, 0b111], // 2
    [0b111, 0b001, 0b111, 0b001, 0b111], // 3
    [0b101, 0b101, 0b111, 0b001, 0b001], // 4
    [0b111, 0b100, 0b111, 0b001, 0b111], // 5
    [0b111, 0b100, 0b111, 0b101, 0b111], // 6
    [0b111, 0b001, 0b001, 0b001, 0b001], // 7
    [0b111, 0b101, 0b111, 0b101, 0b111], // 8
    [0b111, 0b101, 0b111, 0b001, 0b111], // 9
];

/// 在 RGB 图上画一个框 + 它的编号。**这张图就是眼看到的那张** —— 挑错了当场看得见。
fn 画编号框(rgb: &mut [u8], w: usize, h: usize, 框: [usize; 4], 号: usize, 色: [u8; 3], 粗: usize) {
    let put = |v: &mut [u8], x: usize, y: usize| {
        if x < w && y < h {
            let i = (y * w + x) * 3;
            if i + 2 < v.len() { v[i] = 色[0]; v[i + 1] = 色[1]; v[i + 2] = 色[2]; }
        }
    };
    let (x0, y0, x1, y1) = (框[0].min(w - 1), 框[1].min(h - 1), 框[2].min(w - 1), 框[3].min(h - 1));
    for t in 0..粗.max(1) {
        for x in x0..=x1 {
            put(rgb, x, y0.saturating_add(t).min(h - 1));
            put(rgb, x, y1.saturating_sub(t));
        }
        for y in y0..=y1 {
            put(rgb, x0.saturating_add(t).min(w - 1), y);
            put(rgb, x1.saturating_sub(t), y);
        }
    }
    // 🔴🔴 **编号画在框【里面】,字号跟【画面】走而不是跟框走,并且带底色。**
    //
    // 实测代价(XD,2026-08-28,渲图看出来的):上一版把数字画在框**外**、字号取
    // `框长边/12`(小框上只有 2 像素一格)⇒ 棒球那个框(约 25 px)的"7"只有 6×10 像素、
    // 而且悬在框上方、离邻框比离自己还近。结果:几何把 9 件东西**一物一框切得干干净净、
    // 棒球就是第 7 号**,而眼挑了第 1 号(那条牛仔裤)。**切对了、读错了号。**
    // ⇒ 字号 = 画宽的 1/160(640 宽 ⇒ 一格 4 px ⇒ 数字 12×20),画在框内左上角,
    //   先铺一块深底再写亮字 —— 底色让数字在任何背景上都读得出来。
    let 放 = (w / 160).max(3);
    let 数: Vec<usize> = { let mut v = Vec::new(); let mut n = 号; if n == 0 { v.push(0) } else { while n > 0 { v.push(n % 10); n /= 10 } } v.reverse(); v };
    let (字w, 字h) = (数.len() * 4 * 放, 5 * 放);
    let (底w, 底h) = (字w + 2 * 放, 字h + 2 * 放);
    // 🔴 **底块贴在框【外面】** —— 画在框里会把小东西整个糊掉。
    // 实测(XD 重渲):放进框里之后,棒球那个 25 px 的框被自己的编号盖掉大半,
    // 眼看到的是一块黑方块而不是球。⇒ 优先贴框上方;上方放不下就贴下方;都放不下才退回框内。
    let 底y = if y0 >= 底h { y0 - 底h } else if y1 + 底h < h { y1 + 1 } else { y0 };
    let 底x = x0.min(w.saturating_sub(底w));
    for dy in 0..底h {
        for dx in 0..底w {
            let (x, y) = (底x + dx, 底y + dy);
            if x < w && y < h {
                let i = (y * w + x) * 3;
                if i + 2 < rgb.len() { rgb[i] = 0; rgb[i + 1] = 0; rgb[i + 2] = 0; }
            }
        }
    }
    let mut 笔 = 底x + 放;
    for d in 数 {
        for (r, 行) in 字模[d.min(9)].iter().enumerate() {
            for c in 0..3usize {
                if 行 >> (2 - c) & 1 == 1 {
                    for dy in 0..放 { for dx in 0..放 {
                        let (x, y) = (笔 + c * 放 + dx, 底y + 放 + r * 放 + dy);
                        if x < w && y < h {
                            let i = (y * w + x) * 3;
                            if i + 2 < rgb.len() { rgb[i] = 255; rgb[i + 1] = 255; rgb[i + 2] = 255; }
                        }
                    }}
                }
            }
        }
        笔 += 4 * 放;
    }
}

/// **能把这几个点【都】切下来的最大半径**(不超过 `想要`);一个都切不下来就返回 0。
///
/// 🔴🔴 为什么是"缩半径"而不是"把框往画面里挪":
/// 挪框会**悄悄换掉被跟踪的那个点** —— 模板中心不再是接触面,而下游每一个量
/// (画面雅可比、追的误差、退回的目标)都以为它还是。这类"读起来完全正常的错"是本仓最贵的一类
/// (`找块` 头注那条窗口 bug 就是同一族:从不报错,调用方无从分辨真锁和假锁)。
/// 缩半径不动中心:模板变小 ⇒ 区分力变弱、会被日志说出来,但**它跟的还是那个点**。
///
/// 实测代价(WZ,2026-08-28):这具身体开局爪子压在头部相机左下角、一半在画面外
/// (认出来那次落在 (0.161,0.937),由它算出的接触面一个在 **v=1.134**)⇒ `截块` 一律返回 None
/// ⇒ 日志刷「截不出模板 ⇒ 换个位形」,而"换个位形"并不会把手挪进画面 ⇒ **死锁**。
/// 一集 80 分钟里合爪那一步只走到过 2 次。
fn 合用半(w: usize, h: usize, 点们: &[(f64, f64)], 想要: usize) -> usize {
    let mut 半 = 想要;
    while 半 >= 1 {
        let 全切得下 = 点们.iter().all(|(cu, cv)| {
            let (cx, cy) = ((cu * w as f64) as i64, (cv * h as f64) as i64);
            let hf = 半 as i64;
            cx - hf >= 0 && cy - hf >= 0 && cx + hf < w as i64 && cy + hf < h as i64
        });
        if 全切得下 { return 半 }
        半 -= 1;
    }
    0
}

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

/// 快眼:**整张画面**里搜模板(SSD 最小),返回最佳中心(画幅坐标)。
///
/// 🔴🔴 这里本来有一个"只搜 (cu,cv) 附近 ±r 像素"的窗口,已经删掉。
/// **窗口比目标的真实位移小的时候,SSD 会在窗口里挑一个【完全错误、而读起来
/// 毫无异常】的位置回来** —— 它从不报错,调用方也无从分辨真锁和假锁。
/// 实测代价(V2,2026-08-27):探针迈 0.0306 m,接触面在图上跑约 34 px,
/// 而窗口只有 ±33 px ⇒ 通道表里混进了"末端只挪 0.0296 m、我的接触面却在深度上
/// 跑了 0.2552 m"的假列(差 8.6 倍,物理上不可能)。整个转向都靠这张表 ⇒ 方向盘接错线。
/// 全画面搜没有这个参数,也就没有这个坑;快慢由**早停**兜住(比现任差了就不必算完,
/// 结果与不早停完全一致)。
fn 找块(w: usize, h: usize, g: &[u8], tpl: &[u8], half: usize) -> Option<(f64, f64)> {
    let hf = half as i64;
    let n = (2 * hf + 1) as usize;
    if tpl.len() != n * n || w < n || h < n { return None; }
    let mut best: (u64, i64, i64) = (u64::MAX, -1, -1);
    for ny in hf..(h as i64 - hf) {
        for nx in hf..(w as i64 - hf) {
            let mut ssd: u64 = 0;
            let mut i = 0usize;
            let mut 弃 = false;
            for y in (ny - hf)..=(ny + hf) {
                let row = (y as usize) * w;
                for x in (nx - hf)..=(nx + hf) {
                    let d = g[row + x as usize] as i64 - tpl[i] as i64;
                    ssd += (d * d) as u64;
                    i += 1;
                }
                if ssd >= best.0 { 弃 = true; break }   // 早停:已经比现任差,不必算完
            }
            if !弃 && ssd < best.0 { best = (ssd, nx, ny); }
        }
    }
    if best.1 < 0 { return None; }
    Some((best.1 as f64 / w as f64, best.2 as f64 / h as f64))
}

/// 快眼:在**给定的一小片**里搜模板(SSD 最小)。窗口中心必须是**推出来的位置**,
/// 不是"上次在哪"—— 后者正是当初那个会静默锁错的小窗(见 `找块` 的注释)。
fn 找块窗(w: usize, h: usize, g: &[u8], tpl: &[u8], half: usize, cu: f64, cv: f64, r: usize)
    -> Option<(f64, f64)> {
    let hf = half as i64;
    let n = (2 * hf + 1) as usize;
    if tpl.len() != n * n || w < n || h < n { return None }
    let (cx, cy) = ((cu * w as f64) as i64, (cv * h as f64) as i64);
    let mut best: (u64, i64, i64) = (u64::MAX, -1, -1);
    for ny in (cy - r as i64).max(hf)..=(cy + r as i64).min(h as i64 - hf - 1) {
        for nx in (cx - r as i64).max(hf)..=(cx + r as i64).min(w as i64 - hf - 1) {
            let mut ssd: u64 = 0;
            let mut i = 0usize;
            let mut 弃 = false;
            for y in (ny - hf)..=(ny + hf) {
                let row = (y as usize) * w;
                for x in (nx - hf)..=(nx + hf) {
                    let d = g[row + x as usize] as i64 - tpl[i] as i64;
                    ssd += (d * d) as u64;
                    i += 1;
                }
                if ssd >= best.0 { 弃 = true; break }
            }
            if !弃 && ssd < best.0 { best = (ssd, nx, ny); }
        }
    }
    if best.1 < 0 { None } else { Some((best.1 as f64 / w as f64, best.2 as f64 / h as f64)) }
}

/// 快眼:**一次找两块,而且不许落在同一处。**
///
/// 🔴🔴 两根接触面在画面里长得几乎一样(同一只手的两根手指)⇒ 全画面搜的时候
/// **两块模板会同时锁到其中一根上**(SSD 几乎相等,谁先扫到谁赢)。
/// 原来那个"只搜 ±r 像素"的小窗恰好挡住过这件事,删窗口时我只挡了一半 ——
/// 判得出来(两块隔得比模板还近就拒),却没有救。实测代价(NV8,2026-08-27):
/// 六列全空,日志印"两块只隔 11 px,比模板半径 16 还近"。
///
/// 做法:先各自全画面找一次;撞在一起就以**匹配得更好**的那一块为准,
/// 另一块在"离它至少一个模板宽"之外重找。判据是**比例**(模板自己的宽),不是填的数。
fn 找两块(w: usize, h: usize, g: &[u8], a: &[u8], b: &[u8], half: usize)
    -> Option<((f64, f64), (f64, f64))> {
    let 扫 = |tpl: &[u8], 避: Option<(i64, i64)>| -> Option<(u64, i64, i64)> {
        let hf = half as i64;
        let n = (2 * hf + 1) as usize;
        if tpl.len() != n * n || w < n || h < n { return None }
        let mut best: (u64, i64, i64) = (u64::MAX, -1, -1);
        for ny in hf..(h as i64 - hf) {
            for nx in hf..(w as i64 - hf) {
                if let Some((ax, ay)) = 避 {
                    // 🔴 排除半径必须和"算不算撞在一起"用**同一把尺**(一个模板半径)。
                    // 实测代价(NV9):我按两个模板宽去排除,而两块模板是**紧挨着切的**
                    //(半径 = 两面间距的一半)⇒ 另一块的真实位置恰好就在两个模板宽处,
                    // 被我连正确答案一起排掉,于是它跑到画面别处,刚体闸报 差 25.046。
                    if (nx - ax).abs() < hf && (ny - ay).abs() < hf { continue }
                }
                let mut ssd: u64 = 0;
                let mut i = 0usize;
                let mut 弃 = false;
                for y in (ny - hf)..=(ny + hf) {
                    let row = (y as usize) * w;
                    for x in (nx - hf)..=(nx + hf) {
                        let d = g[row + x as usize] as i64 - tpl[i] as i64;
                        ssd += (d * d) as u64;
                        i += 1;
                    }
                    if ssd >= best.0 { 弃 = true; break }
                }
                if !弃 && ssd < best.0 { best = (ssd, nx, ny); }
            }
        }
        if best.1 < 0 { None } else { Some(best) }
    };
    let pa = 扫(a, None)?;
    let pb = 扫(b, None)?;
    let 撞 = (pa.1 - pb.1).abs() < half as i64 && (pa.2 - pb.2).abs() < half as i64;
    let (qa, qb) = if !撞 { (pa, pb) }
        else if pa.0 <= pb.0 { (pa, 扫(b, Some((pa.1, pa.2)))?) }
        else { (扫(a, Some((pb.1, pb.2)))?, pb) };
    Some(((qa.1 as f64 / w as f64, qa.2 as f64 / h as f64),
          (qb.1 as f64 / w as f64, qb.2 as f64 / h as f64)))
}

/// 干活。缺某个身体量就**点名要它**(返回那一格的名字),由调用方量完再进来一次。
/// 返回 `None` = 干完了 / 连接断了。
// ════════════════════════════════════════════════════════════════════════════
// 干活 —— **一条路,没有"抓"这件事。**
//
// 🔴🔴 **这里删掉了 2019 行手写的抓取状态机**(悬停 / 下探 / 合爪 / 抬起 / 补抓 /
// 像素伺服 / 模板跟踪 / 让开视线 …,26 个换段点)。它是老架构长出来的:
// 通用执行层出的航点说的是**指尖该到哪**,而驱动只会发**法兰**的命令,
// 中间那段换算靠三个各量各的标量(爪宽 / 工具长 / 工具轴)——
// 于是补丁越打越多,长成了两千行,而**今晚所有的 bug 都长在那三个标量上**。
//
// 新架构把中间那段整个消掉,靠一条几何事实:
//
// ```text
// 末端平移时,指尖跟着【刚性平移】
//   ⇒ 末端目标 = 指尖目标 − (指尖现在 − 末端现在)
//   ⇒ 「法兰到指尖多长」在这个减法里自己抵消了
// ```
//
// 所以驱动只要知道**指尖此刻在三维空间的哪里** —— 而那是**看出来的**:
// 手臂冻住、只动爪子 ⇒ 会动的那一块按构造就是钳口(构造性身份,08-10 / 08-14 两验)。
//
// 🔴 这条路上**没有"抓"**:接触集说碰哪几个点、往哪使劲、物体要怎么动,
// 执行层就去让它发生。推 / 撬 / 拧 / 吸盘 / 三指 / 五指走的是同一句话。

fn 服务<S: std::io::Read + std::io::Write>(
    plug: &mut Plug<S>,
    body: &body_layer::Body,
    相机们: &[(usize, point_gen::Eye, f64)],
    眼主机: &str,
    眼端口: u16,
    标定文件: &str,
    _入档: Option<&str>,
    // 🔴 这具身体**给不出**的那几格 —— 要过三次仍量不到的。不再点名要,带着缺口降级干。
    给不出: &std::collections::BTreeSet<&'static str>,
    // 🔴 手的几何(腕系):(指尖偏移, 开合方向, 那一块占画幅多宽)。
    // **先拿它预测,核对得上就不重晃;对不上就当场重量并写回。**
    手载: &mut Option<([f64; 3], [f64; 3], f64)>,
    // 🔴 画面雅可比(世界位移 → 画面三数)。**现量一次吃掉一集,所以存着用;用之前先核。**
    雅载: &mut Option<[[f64; 3]; 3]>,
    // 🔴 通道表:6 行(两个接触面 × 三个数)× 通道数列。**控制走它,不走末端。**
    通道表: &mut Option<Vec<[f64; 6]>>,
    // 通道是关节,还是末端那六个自由度。**试出来的**(这具机体七个关节命令全零响应)。
    通道是关节: &mut bool,
    // 🔴 哪台相机长在手上 —— 探一次要动一下手臂,装回来就不重探(owner 2026-08-28)。
    手上相机载: &mut Option<usize>,
    // 🔴 钳口张开(拟出来的那个,不是"含面宽"的上界)—— 量一次存住,下一炮装回。
    张开载: &mut Option<f64>,
    // 🔴 空手合到底时机体报的读数 —— "指间有没有东西"的零点。量一次存住。
    空合载: &mut Option<f64>,
    // 🔴 "这台相机看不见我的钳口张合" —— 装回来就直接跳过那一步,把拍数留给走路。
    认面载: &mut bool,
) -> Option<String> {
    use body_layer::measurement::Quantity as Q;
    println!("[服] 干活模式:观测里给什么指令,就做什么。没有标定阶段,缺什么当场点名要。");

    // 这条路真正要的身体量 —— **只有三个**,而且都不是几何标量:
    //   可达(能不能够到)· 交付率(一条命令走多远)
    // 🔴🔴 **「接触阈」这一格已经从这条路上删掉。**
    // 它存在的唯一理由是给"这一拍走得比平时少 ⇒ 碰到东西了"当基准。而那个判据本身是错的:
    // **走得少既可能是碰到了,也可能只是这一步命令给大了、它还没走完** —— 两件事同形。
    // (文档实测:同一条臂,命令 0.027 m 交付 90%,命令 0.070 m 只有 22% —— 分母变大了,不是它变懒。)
    // 正确的判据**一个基准都不需要**:**把目标按住不放,看残差还缩不缩** ——
    // 一直缩 = 空的;**不再缩而且还差着一截 = 有东西挡着**。浮点数相等就是相等,无阈值。
    // ⇒ 少一个要量的量,而它正是新架构第一炮把整炮卡死六轮的那一个。
    // 🔴 爪宽 / 工具长 / 工具轴**不在这里** —— 它们在减法里抵消了,或者由自模现读。
    let 量 = |q: Q, n: usize| -> Option<Vec<f64>> {
        body.get(q).filter(|m| m.value.len() >= n && m.selftest_passed).map(|m| m.value[..n].to_vec())
    };
    let Some(可达带) = 量(Q::Reach, 2) else { return Some("reach".into()) };
    let Some(交付率) = 量(Q::StepDelivery, 1).map(|v| v[0]) else { return Some("step_delivery".into()) };
    // 🔴 相机在下面**自己解**出来之后才有 —— 这里不许先取 `相机们[0]`
    // (实测:留了这一行 ⇒ 空表取下标 ⇒ `index out of bounds: the len is 0 but the index is 0`,
    //  整个驱动 panic 掉,而它发生在"解相机"这件事**之前**)。
    let 灰 = |f: &Frame, ci: usize| f.cams.get(ci).map(|(w, h, d)| (*w, *h, d.clone()));
    let 彩 = |f: &Frame, ci: usize| f.cams.get(ci).map(|(w, h, d)| {
        let mut rgb = Vec::with_capacity(w * h * 3);
        for g in d { rgb.push(*g); rgb.push(*g); rgb.push(*g); }
        (*w, *h, rgb)
    });

    // 走到位就停:剩下不足起点距离的 2%。**不是"看起来不动了"** ——
    // 交付率低的身体上,"还没开始动"和"停了"在读数上同形(实测 +0.2 rad ⇒ 实到 +0.005)。
    // **把目标按住不放,直到走到 —— 或者直到它不再缩短。**
    //
    // 返回 `(那一帧, 挡住了没有, 还差多少)`。
    // 🔴 「挡住了」的判据不带任何常数:**残差连着三拍一模一样,而且还差着一截**。
    //    浮点数相等就是相等。这与"走得比平时少"完全不同 —— 后者把"碰到了"和"还没走完"
    //    压成了同一个读数,而这具身体在空中本来就走不满(命令越大交付比例越低)。
    // 🔴🔴🔴 **用哪只手 / 哪个末端,全程读这两个格子,代码里一个数字都不许写死。**
    //(owner 2026-08-31 死命令:"driver 不能有任何写死的东西,所有都要 driver 自己决定,
    //  适用于所有奇形怪状的硬件")
    //
    // 它们的值在**眼指出目标之后**由测量填进去(见下面「挑手」那一段):
    // 上面那次逐臂推一下已经量到了「哪个末端属于哪条臂」和「这条臂在主眼里占哪一片」,
    // 目标一到就取**离目标最近的那条臂**。在那之前是 0,只是个占位 ——
    // 而**填之前不会有任何朝目标的动作**,所以占位不会把手选错。
    // 用 `Cell` 是因为下面这一大片闭包都要读它,而它要在闭包定义之后才被填。
    let 手号 = std::cell::Cell::new(0usize);   // 第几个末端(`f.ee` 的下标 = `Cmd::Ee{arm}`)
    let 臂号 = std::cell::Cell::new(0usize);   // 第几个关节组(`Cmd::Joints{arm}`)
    let 落 = |plug: &mut Plug<S>, at: [f64; 3], q: [f64; 4], j: f64, 上限: u32| -> Option<(Frame, bool, f64)> {
        // 🔴🔴 **"到没到"的分母是【这次命令要走多远】,不是【第一拍量到的残差】。**
        //
        // 实测代价(2026-08-25,G4):晃爪子的时候**末端目标没变** ⇒ 起始残差本来就接近 0,
        // 而判据写成"缩到起始残差的 2%"就等于要它缩到一个比噪声还小的数 ⇒ **每次晃爪都跑满
        // 200 拍**。12 个位置 × 5 次晃 × 200 = **12000 拍**,一档就是六十集,整炮卡死在解相机上。
        // ⇒ 末端没让它走(该走≈0)时**立刻算到位**;让它走了才按比例判。
        let 起位 = plug.sense().and_then(|f| f.ee.get(手号.get()).map(|e| [e[0], e[1], e[2]]))?;
        let 该走 = ((at[0]-起位[0]).powi(2) + (at[1]-起位[1]).powi(2) + (at[2]-起位[2]).powi(2)).sqrt();
        let mut 末: Option<Frame> = None;
        let mut 上次: Option<f64> = None;
        let mut 停 = 0u32;
        let mut 差 = f64::NAN;
        for _ in 0..上限 {
            plug.act(&Cmd::Ee { arm: 手号.get(), at, quat: q, jaw: j });
            let Some(f) = plug.sense() else { break };
            let d = f.ee.get(手号.get()).map(|e| ((e[0]-at[0]).powi(2)+(e[1]-at[1]).powi(2)+(e[2]-at[2]).powi(2)).sqrt())?;
            差 = d;
            末 = Some(f);
            // 0.02 是**比例**(剩下不足起始距离的 2% 就算到位),无量纲。
        if d <= 0.02 * 该走.max(1e-9) { return 末.map(|f| (f, false, d)) }
            // **挡住 = 残差不再缩,而且还差着一截。** 浮点相等就是相等,无阈值。
            if 上次 == Some(d) { 停 += 1; if 停 >= 3 { return 末.map(|f| (f, true, d)) } } else { 停 = 0 }
            上次 = Some(d);
        }
        末.map(|f| (f, false, 差))
    };

    // 🔴🔴 **只动爪子、手臂按住不动的那种等法 —— 等的是【爪子读数不再变】。**
    //
    // 实测代价(S1,2026-08-26):看爪要 6 次 `落`,而 `落` 等的是**手臂走到位**;
    // 晃爪子时手臂目标压根没变,"走到位"那条判据(残差 ≤ 该走的 2%,而该走≈0)**永远触发不了**
    // ⇒ 每次白等满 28 拍 ⇒ 一次看爪 **168 拍**,而一集只有 **200 拍** ⇒ 一集只够看一次爪。
    // 伺服要"看爪→挪→再看爪"闭环,这个预算下根本转不起来。
    // ⇒ 手臂不动的那几拍,判据换成**爪子读数连着两拍一模一样**(浮点相等就是相等,无阈值),
    //   一次看爪掉到 ~40 拍。这和合爪、落位用的是同一条规矩。
    // 🔴🔴 **"还没开始动"和"已经停了"在读数上是同一个样子 —— 必须先看见它离开起点。**
    //
    // 实测代价(S7,2026-08-26):第一版写成"读数连着两拍一样就算停了",于是爪子还没启动
    // 就被判成停了,三拍退出 ⇒ 五帧合同里那三帧**爪子一动没动** ⇒ 认块器 `双响 0`
    // (画面上一个像素都没超过噪声地板)⇒ 连着整集"这一眼没看清爪子"。
    // 这条坑仓里对**手臂**早就写过(见 `落` 上面那段:交付率低时两者同形),我在**爪子**上又踩一遍。
    // ⇒ `要动=true` 时:先等它**离开起点**,再等它**停下**;两条都满足才算数。
    //   `要动=false`(空帧,本来就不该动)走另一条:等两拍就够,不然白等满上限。
    // 🔴 **基线由调用方给,这里【不许】自己再 `sense` 一次** —— `sense` 是**阻塞读下一帧**,
    // 而这时通常没有待读的帧(上一次 `落`/`晃` 刚把它读完)⇒ 它会卡住、返回空。
    // 实测代价(S8):开机挑相机那一步整个空掉,打出"没有一台相机认得出这只手"直接退出,
    // 而**手和相机都好好的**。
    // 🔴🔴 **拍空帧之前必须先等【手臂】真的停住 —— 否则噪声地板被余晃抬高,手指那点动静全被埋掉。**
    //
    // 认块器的噪声地板是拿**两张本该完全一样的空帧**算出来的。我为了省拍数把空帧改成"等两拍就拍",
    // 而那时手臂还在余晃 ⇒ 两张空帧不一样 ⇒ 地板被抬得老高。
    // 实测(D2):地板在 **4 到 158** 之间乱跳,高的时候(93/146/158)`双响 0` —— 一个像素都没超过它,
    // 于是"看不清爪子" 18 次、看清只有 5 次,整集出不来。
    // 判据无阈值:**位置连着两拍一模一样**(浮点相等就是相等),和落位、合爪同一条。
    let 稳住 = |plug: &mut Plug<S>, at: [f64; 3], q: [f64; 4], j: f64, 上限: u32| -> Option<Frame> {
        let mut 上次: Option<Vec<f64>> = None;
        let mut 稳 = 0u32;
        let mut 末: Option<Frame> = None;
        for _ in 0..上限 {
            plug.act(&Cmd::Ee { arm: 手号.get(), at, quat: q, jaw: j });
            let Some(f) = plug.sense() else { break };
            let 此: Vec<f64> = f.ee.get(手号.get()).map(|e| e[..3].to_vec()).unwrap_or_default();
            末 = Some(f);
            if 上次.as_ref() == Some(&此) { 稳 += 1; if 稳 >= 2 { break } } else { 稳 = 0 }
            上次 = Some(此);
        }
        末
    };

    let 晃 = |plug: &mut Plug<S>, at: [f64; 3], q: [f64; 4], j: f64, 起: &[f64], 要动: bool, 上限: u32| -> Option<Frame> {
        let mut 上次: Option<Vec<f64>> = None;
        let mut 稳 = 0u32;
        let mut 动过 = false;
        let mut 末: Option<Frame> = None;
        for 拍 in 0..上限 {
            plug.act(&Cmd::Ee { arm: 手号.get(), at, quat: q, jaw: j });
            let Some(f) = plug.sense() else { break };
            let 此: Vec<f64> = f.jaw.iter().copied().collect();
            末 = Some(f);
            if !要动 { if 拍 >= 1 { break } continue }
            if 此.as_slice() != 起 { 动过 = true }
            if 动过 && 上次.as_ref() == Some(&此) { 稳 += 1; if 稳 >= 2 { break } } else { 稳 = 0 }
            上次 = Some(此);
        }
        if 要动 && !动过 { println!("[服]   ⚠️ 让爪子走到 {j:.2},{上限} 拍里读数**一次都没变过** —— 爪子没动"); }
        末
    };

    // 🔴🔴🔴 **长距离要【滑行】,不是"下一条命令→等它停稳→再下一条"。**
    //
    // 实测代价(SI/SJ,整炮 0/55):`想迈 0.31 m 结果只挪了 0.0001 m` ——
    // 手臂对**大跨度**的命令**一动不动**(位姿被拒),而小跨度走得动。
    // 于是只能一小步一小步走,而每一步都要等它停稳(28 拍)⇒ **每拍只推进约 3 mm**,
    // 末尾还在减速;一集 200 拍走不完 0.37 m,集一换手臂回原处,**永远到不了**。
    //
    // ⇒ 滑行:**每一拍都把目标往前放一小截**(那一截取"已知走得动"的跨度),手臂一路不加不减;
    //   中途**不低头看手**,滑完再看一眼。0.37 m 约 60–80 拍走完,而不是 200 拍走不完。
    //   停的判据照旧无阈值:**位置连着几拍一模一样 = 到边了**。
    // 返回 (最后位置, 到边了没有, 实际走了多远)。
    let 滑 = |plug: &mut Plug<S>, 向: [f64; 3], 总距: f64, q: [f64; 4], j: f64, 跳: f64, 上限: u32|
        -> Option<([f64; 3], bool, f64)> {
        let mut 位 = plug.sense().and_then(|f| f.ee.get(手号.get()).map(|e| [e[0], e[1], e[2]]))?;
        let 起 = 位;
        let mut 停 = 0u32;
        for _ in 0..上限 {
            let 走过 = ((位[0]-起[0]).powi(2) + (位[1]-起[1]).powi(2) + (位[2]-起[2]).powi(2)).sqrt();
            if 走过 >= 总距 { return Some((位, false, 走过)) }
            let 剩 = (总距 - 走过).min(跳);
            let 目标 = [位[0] + 向[0]*剩, 位[1] + 向[1]*剩, 位[2] + 向[2]*剩];
            plug.act(&Cmd::Ee { arm: 手号.get(), at: 目标, quat: q, jaw: j });
            let Some(f) = plug.sense() else { break };
            let Some(e) = f.ee.get(手号.get()).copied() else { break };
            let 新 = [e[0], e[1], e[2]];
            let 这拍 = ((新[0]-位[0]).powi(2) + (新[1]-位[1]).powi(2) + (新[2]-位[2]).powi(2)).sqrt();
            位 = 新;
            if 这拍 <= 0.0 { 停 += 1; if 停 >= 5 {
                let 走过 = ((位[0]-起[0]).powi(2) + (位[1]-起[1]).powi(2) + (位[2]-起[2]).powi(2)).sqrt();
                return Some((位, true, 走过))
            } } else { 停 = 0 }
        }
        let 走过 = ((位[0]-起[0]).powi(2) + (位[1]-起[1]).powi(2) + (位[2]-起[2]).powi(2)).sqrt();
        Some((位, false, 走过))
    };

    // 🔴 **等几拍由【量到的交付率】算出来**:一拍走掉 `交付率`,要走掉 95% 需要
    // `ln(0.05)/ln(1-交付率)` 拍。这是**推出来的**,不是拍的数。
    //
    // 实测代价(R1):解眼那一步用的是"等它精确走到命令的点(残差 ≤ 起始的 2%)",
    // 而第一档幅度只有 **1 毫米** ⇒ 容差 **20 微米**,这条手臂停不到那么准
    // ⇒ **每次都跑满 200 拍**,一个位置两集,12 个位置 24 集。
    // 而解眼**根本不需要它走到那个点** —— 要的是「**它实际在哪** ↔ 它在画面哪」这一对,
    // 实际在哪本体感受直接给。⇒ 等够拍数就读,不问它到没到。
    // 🔴🔴 **等【一个时间常数】就走,不等它走完。**
    // 这一层的全部前提就是"命令和实到是两件事,实到是**量出来的**" —— 既然每一步实际走了多少
    // 都当场读回来,就没有任何理由干等它把命令执行干净。
    // 实测代价(NV6,2026-08-27):等到"走掉 95%"= **28 拍/步**(这台机体一拍只交付 10.4%),
    // 于是一集 **2000 步**跑完了都没做完一把抓取("追的中途换集了")—— 而官方一集只有 200 步。
    // 换成一个时间常数(e⁻¹ ≈ 剩 37%)⇒ **9 拍/步**,同样的位移用不到三分之一的时间。
    // 没有手填数:0.37 就是 e⁻¹ 本身,**无量纲**。
    let 等拍 = ((-1.0f64).exp().ln() / (1.0 - 交付率).max(1e-6).ln()).ceil().clamp(3.0, 120.0) as u32;
    println!("[服] 一拍交付 {:.3} ⇒ 走掉一个时间常数(63%)要 {等拍} 拍(推出来的,不是填的;不等它走完,实到照量)", 交付率);

    // 只动爪子的时候,等的是**爪子读数不再变** —— 和手臂那条等法无关,也不吃 `等拍`。
    // 🔴🔴 **等手臂真停下来。**(WJ 实测,2026-08-28)
    // `看爪` 认爪子靠"晃一下钳口、前后帧做差",它要求那几帧里**手臂是静止的**。
    // 而伺服每一拍都在动手臂 ⇒ 日志连着出现
    //   `认块器没给出候选(双响 0 · 配对 0 · 地板 137)—— 地板高:空帧那会儿手臂还在晃`
    // ⇒ **它一动就看不见自己的手,看不见就算不出误差。** 伺服与观测互相打架。
    // `定爪` 只等钳口的读数稳,不等手臂 ⇒ 另立一个:发 Hold、等**末端位姿读数**连着两拍不变。
    // "连着两拍"是**计数**,无量纲;上限由量出来的 `等拍` 给。
    let 等停 = |plug: &mut Plug<S>, 上限: u32| {
        let mut 上: Option<Vec<u64>> = None; let mut 稳 = 0u32;
        for _ in 0..上限 {
            plug.act(&Cmd::Hold);
            let Some(f) = plug.sense() else { break };
            let 此: Vec<u64> = f.ee.get(手号.get()).map(|e| e.iter().map(|x| x.to_bits()).collect()).unwrap_or_default();
            if 此.is_empty() { break }
            if 上.as_ref() == Some(&此) { 稳 += 1; if 稳 >= 2 { break } } else { 稳 = 0 }
            上 = Some(此);
        }
    };
    let 定爪 = |plug: &mut Plug<S>, 上限: u32| {
        let mut 上 = None; let mut 稳 = 0u32;
        for _ in 0..上限 {
            plug.act(&Cmd::Hold);
            let Some(f) = plug.sense() else { break };
            let 此: Vec<u64> = f.jaw.iter().map(|x| x.to_bits()).collect();
            if 上.as_ref() == Some(&此) { 稳 += 1; if 稳 >= 2 { break } } else { 稳 = 0 }
            上 = Some(此);
        }
    };

    // 🔴🔴🔴 **把"该动多少"变成这具身体的命令 —— 只允许有【一处】派发。**
    // 关节还是末端那六个自由度,由**量出来的结论**(通道是关节)决定,不由写代码的人决定。
    // 实测代价:同一个 bug 已经咬过三处 ——
    //   ① 开合爪走 `Cmd::Joints`(F6)⇒ 爪子永远合不上 + "接触面最远分开多少"量成 0;
    //   ② 追那一段写死关节 ⇒ 换了通道种类就动不了;
    //   ③ **退回那一段仍然写死 `Cmd::Joints`**(2026-08-27 查出)⇒ 这具机体一个关节命令都不响应,
    //      于是**就算夹住了也退不回去**,东西永远离不开原位,官方判定必然不算成功。
    // 返回:(实际走了多远, 各通道实际动了多少, 路上有没有被挡住)。
    // 「各通道实际动了多少」的量纲与通道表**同一套**(平移取轴上分量、转动取相对四元数矢部 ×2),
    // 因为干活时那条边干边长的修正要拿它做除数;两处不一致会把好表越修越坏。
    // 🔴🔴🔴 **搬运不许等停 —— 一集只有 200 个仿真步。**(XK 实测,2026-08-28)
    //
    // `迈通道` 每走一步要等手臂读数稳住(`等拍 * 2` 拍)。量表和追要这个,因为它们**要读**
    // 这一步的结果;而**把手从桌子这头搬到那头不需要读中间过程**。
    // 实测代价(XK):要走 0.5058 m、算出要 264 步,而每步吃掉约 20 个仿真步 ⇒
    // 一集只够走 **10 步**,第 0 步刚打印完就被换集打断,然后合了一把空气。
    // 日志里既没有"走到了"也没有"过不去" —— 它是被 `复位过` 静默切走的。
    // ⇒ 搬运走 `稳拍 = 1`:发一次命令、读一帧就进下一步,一步只花约 2 个仿真步。
    // 🔴🔴🔴 **同一条胳膊会被报两遍 —— 状态一遍、命令回声一遍。量出来去重,不看键名。**
    //(AL 实测 2026-08-30)
    //
    // 这具身体报的关节是四组:`state.left_arm · state.right_arm · action.left_arm · action.right_arm`。
    // 后两组是**我发出去的命令的回声**,不是多出来的两条胳膊。我把它们当成了额外自由度 ⇒
    // "推通道 12~23"实际是往命令回声里写数,**身体一动不动** ⇒ 实测 `通道表:24 列里量到 0 列`。
    // ⚠️ 由此撤回我说过的"通道从 6 涨到 24 是零改动换体的证据" —— 真实自由度是 **12**。
    //
    // 去重判据不看 `state`/`action` 这些字眼(那是**这台机器的名字**,减法①禁止):
    // **两组值在同一帧里逐位相同 ⇒ 它们是同一条胳膊被报了两遍**,只留第一组。
    // 命令回声在稳态下必然等于状态;真的两条胳膊逐位相同的概率是零。
    let 真臂 = |f: &Frame| -> Vec<usize> {
        let mut 留: Vec<usize> = Vec::new();
        for (i, v) in f.joints.iter().enumerate() {
            if v.is_empty() { continue }
            let 重 = 留.iter().any(|&j| {
                let w = &f.joints[j];
                w.len() == v.len() && w.iter().zip(v.iter()).all(|(a, b)| (a - b).abs() < 1e-9)
            });
            if !重 { 留.push(i) }
        }
        if 留.is_empty() && !f.joints.is_empty() { 留.push(0) }
        留
    };

    let 迈通道稳 = |plug: &mut Plug<S>, 动: &[f64], 比: f64, jaw: f64, 是关节: bool, 稳拍: u32| -> Option<(f64, Vec<f64>, bool)> {
        let f0 = plug.sense()?;
        if 是关节 {
            // 🔴🔴🔴 **通道横跨【这具身体报出来的所有关节组】,不是第一组。**(2026-08-30,ARX 双臂逼出来)
            //
            // 仓里自己的设计原话:*"通道 = 观测里报出来的【每一个】能下命令的自由度
            //(关节/手指/桨/轮/舵),数量由布局给"*。而代码一直写 `joints.first()` ——
            // 单臂上碰巧等价,换成 ARX 双臂就**把 12 个臂关节当成 6 个**,右手的关节从没上过桌。
            // 实测(AD):`通道表装回:6 个通道`,而这具身体报的是 left_arm + right_arm 两组。
            //
            // 🔴 **"用哪只手"不该由我写死,也不需要一条规则** —— 把所有自由度摆上桌,
            // 最小二乘自己解:被跟的那两个接触面长在哪只手上,另一只手的关节对它的系数就是零,
            // 解出来自然只动该动的那只。一个"左""右"都不出现,换三条胳膊/五指手同样成立。
            //
            // ⚠️ 这条线缆一次只换一条臂的值(其余臂发它此刻的值,见 `Cmd::Joints` 那段注释),
            //    所以这一步发**位移最大的那条臂**;要两条臂同时动,下一步再发另一条。
            //    这是线缆的形状,不是判据 —— 照实写出来,不假装同时动了。
            let 号 = 真臂(&f0);
            let 各臂: Vec<Vec<f64>> = 号.iter().map(|&i| f0.joints[i].clone()).collect();
            if 各臂.is_empty() { return None }
            let 长: Vec<usize> = 各臂.iter().map(|v| v.len()).collect();
            let q0: Vec<f64> = 各臂.concat();
            let (mut 起, mut 臂, mut 最大) = (0usize, 0usize, -1.0f64);
            for (a, l) in 长.iter().enumerate() {
                let s: f64 = (0..*l).map(|i| 动.get(起 + i).copied().unwrap_or(0.0).abs()).sum();
                if s > 最大 { 最大 = s; 臂 = a; }
                起 += l;
            }
            let 偏: usize = 长[..臂].iter().sum();
            let mut q = 各臂[臂].clone();
            for i in 0..q.len() { q[i] += 动.get(偏 + i).copied().unwrap_or(0.0) * 比 }
            if !plug.act(&Cmd::Joints { arm: 号[臂], q, jaw }) { return None }
            let mut 上: Option<Vec<f64>> = None; let mut 稳 = 0u32; let mut 末 = None;
            for _ in 0..稳拍.max(1) {
                let f = plug.sense()?;
                let 此: Vec<f64> = 真臂(&f).iter().map(|&i| f.joints[i].clone()).collect::<Vec<_>>().concat();
                末 = Some(此.clone());
                if 上.as_ref() == Some(&此) { 稳 += 1; if 稳 >= 2 { break } } else { 稳 = 0 }
                上 = Some(此);
            }
            let q1 = 末.unwrap_or_else(|| q0.clone());
            let 动实: Vec<f64> = q1.iter().zip(q0.iter()).map(|(a, b)| a - b).collect();
            Some((动实.iter().map(|x| x * x).sum::<f64>().sqrt(), 动实, false))
        } else {
            let e0 = f0.ee.get(手号.get()).copied()?;
            let (p0, q0) = ([e0[0], e0[1], e0[2]], [e0[3], e0[4], e0[5], e0[6]]);
            let mut p = p0;
            for k in 0..3 { if k < 动.len() { p[k] += 动[k] * 比 } }
            let mut qq = q0;
            for k in 3..6 {
                if k >= 动.len() { break }
                let a = 动[k] * 比 * 0.5;
                let mut ax = [0.0; 3]; ax[k - 3] = 1.0;
                let r = [a.cos(), a.sin()*ax[0], a.sin()*ax[1], a.sin()*ax[2]];
                let (w1, x1, y1, z1) = (qq[0], qq[1], qq[2], qq[3]);
                let (w2, x2, y2, z2) = (r[0], r[1], r[2], r[3]);
                qq = [w1*w2 - x1*x2 - y1*y2 - z1*z2,
                      w1*x2 + x1*w2 + y1*z2 - z1*y2,
                      w1*y2 - x1*z2 + y1*w2 + z1*x2,
                      w1*z2 + x1*y2 - y1*x2 + z1*w2];
            }
            let (f2, 挡, _) = 落(plug, p, qq, jaw, 等拍)?;
            let e2 = f2.ee.get(手号.get()).copied().unwrap_or(e0);
            let 走 = ((e2[0]-p0[0]).powi(2)+(e2[1]-p0[1]).powi(2)+(e2[2]-p0[2]).powi(2)).sqrt();
            let (w1, x1, y1, z1) = (q0[0], -q0[1], -q0[2], -q0[3]);
            let (w2, x2, y2, z2) = (e2[3], e2[4], e2[5], e2[6]);
            let 相对 = [w1*x2 + x1*w2 + y1*z2 - z1*y2,
                        w1*y2 - x1*z2 + y1*w2 + z1*x2,
                        w1*z2 + x1*y2 - y1*x2 + z1*w2];
            Some((走, vec![e2[0]-p0[0], e2[1]-p0[1], e2[2]-p0[2],
                           2.0*相对[0], 2.0*相对[1], 2.0*相对[2]], 挡))
        }
    };

    // **只认"会动的那一块在画面哪一点" —— 不需要相机模型。**
    // 解相机要的原料就是它:手在哪(本体感受)↔ 手在画面哪(这里)。
    let 看爪像素 = |plug: &mut Plug<S>, at: [f64; 3], q: [f64; 4], j0: f64| -> Vec<Option<body_layer::hand::Candidate>> {
        let 台 = plug.lay.cams.len();
        let mut out: Vec<Option<body_layer::hand::Candidate>> = vec![None; 台];
        // 0.30 是**钳口命令行程的分数**(命令域本身就是 0..1),无量纲。
    let 步 = (if j0 > 0.5 { -1.0 } else { 1.0 }) * 0.30;
        let mut 帧们: Vec<Frame> = Vec::new();
        let Some(a) = 稳住(plug, at, q, j0, 等拍 * 2) else { return out };
        let 基: Vec<f64> = a.jaw.iter().copied().collect();
        帧们.push(a);
        let Some(b) = 晃(plug, at, q, j0, &基, false, 等拍) else { return out };
        帧们.push(b);
        for k in 1..=3 {
            let Some(f) = 晃(plug, at, q, (j0 + 步 * k as f64).clamp(0.0, 1.0), &基, true, 等拍) else { return out };
            帧们.push(f);
        }
        let _ = 晃(plug, at, q, j0, &基, true, 等拍);
        if 帧们.len() < 5 { return out }
        for ci in 0..台 {
            let Some((w, h, _)) = 灰(&帧们[0], ci) else { continue };
            let g: Vec<Vec<u8>> = 帧们.iter().filter_map(|f| 灰(f, ci).map(|(_, _, d)| d)).collect();
            if g.len() < 5 { continue }
            match body_layer::blob::candidates(&g[0], &g[1], &g[2], &g[3], &g[4], w, h,
                    步.abs(), selfcal::最少像素(w, h)) {
                Ok(r) => out[ci] = r.cands.get(0).copied(),
                Err(_) => {}
            }
        }
        out
    };

    // 🔴🔴🔴 **「先把整台相机解出来」这一段已经删掉。**(改架构 2026-08-26,owner 拍板)
    //
    // 它连着六炮解不出来,而**每炮的拒绝理由都不一样**:点共面(z 只有 1.6 cm)/ 手走出画面还硬走
    // 13 步一对没收 / 掉头后在一根轴上来回蹭(y 只有 5 mm)/ 点太扁 / 姿态不变导致指尖偏置与
    // 相机位置在数学上分不开(合成实测:残差 2.5e-16 完美,而相机错 13 cm)。
    // 六种病同一类:**全局解不唯一**。而且它和硬约束结构性冲突 —— 相机会动(移动底盘/头/直播),
    // 解一次要吃掉 20 集以上,解完就作废。
    // ⇒ 换成下面那张**局部、现量、天天更新**的画面雅可比。`point_gen::fit_full*` 留在仓里
    //   (28 条测试都在),它以后可以从**干活时顺手攒下的**对子里白掉出来,不必专门为它花一炮。

    // 哪台相机认得出这只手 —— **试出来的,不是指定的**。
    // 🔴 一次没认出来**不许让整个驱动死掉** —— 换个位形真挪一步再试。
    // 实测代价(S8):开机这一步空了一次就 `return None`,整个驱动退出,而手和相机都好好的。
    // 🔴🔴🔴 **主眼 = 【长在世界里】的那台,不是"第一台认得出我的手的"。**(2026-08-30,渲图定案)
    //
    // 旧规则是"挑第一台认得出这只手的"。它在两具身体上各错一次,而且**病相完全不同**:
    //   · Franka(单臂):头相机先认出手 ⇒ 主眼 = 头相机,而它**看不见爪子** ⇒ 一集 344 次"看不清爪子"
    //   · ARX(双臂):头相机这次没认出手、左腕认出了 ⇒ 主眼 = **左腕相机**,而它侧对着墙
    //     ⇒ 交给眼的那张图里是风扇和键盘,眼当然说"这张桌上没有它要的东西" ——
    //     **球一直在头相机里,只是没人去那张图上找**(`AH_眼看到的.png` 一看就完)。
    //
    // 根子:**"找东西"和"认爪子"是两件活儿,却共用一个 `相机号`。**
    // 主眼要的是**看得见工作区**,判据是**它不跟着我动** —— 长在世界里的那台,
    // 手一动它画面里只有手那一小块变;长在手上的那台整幅都在变。这一条本来就在下面量,
    // 只是量完只用来挑"收尾那台",没用来挑主眼。
    // ⇒ 主眼 = 变化最小的那台(长在世界里);收尾那台 = 变化最大的那台(长在手上)。
    // 认爪子在主眼里失败时,本来就有"立刻交给腕相机"那条路(`看爪` 失败分支)。
    // 🔴🔴🔴 **"用哪只手"必须是量出来的,不许写死。**(owner 2026-08-31,死命令)
    //
    // 代价照记:末端探针和整条伺服路上写的是 `Cmd::Ee { arm: 0 }` / `f.ee.get(手号.get())` ——
    // 一个写死的手号就是一句身体断言。AS 实测后果:**右臂全程一次没动**,
    // 而左臂被那个没上界的梯子甩出去,横扫时**多次撞上右臂**(owner 看视频指出)。
    // "自己选手"此前只落在挑主眼和关节表挑哪条臂动两处,**没落到末端和伺服**,而那才是全程在跑的。
    //
    // 这一段同时量出三件事,一次动作全拿到,换几条胳膊、几只末端都成立:
    //   ① 每台相机跟着哪条臂动 ⇒ 主眼 = 对谁都不怎么变的那台(长在世界里)
    //   ② **哪个末端属于哪条臂** —— 推这条臂时位移最大的那个末端就是它的,
    //      不假设"第 a 条臂配第 a 个末端"(那又是一句身体断言)
    //   ③ **这条臂在主眼画面里占哪一片** —— 变化像素的重心;
    //      等眼指出目标之后,离目标最近的那条臂就是该用的手
    // 🔴🔴🔴 **"用哪只手"必须是量出来的,不许写死。**(owner 2026-08-31,死命令)
    //
    // 代价照记:写死 `Cmd::Ee { arm: 0 }` / `f.ee.first()` 的后果是 ARX 双臂上
    // **右臂全程一次没动**,而左臂横扫时多次撞上右臂(owner 看视频指出)。
    //
    // 🔴 **探测必须用【这具机体真正认的那种命令】,否则它一下都不会动。**(AW 实测)
    // 上一版发的是 `Cmd::Joints`,而日志第 22 行就写着
    // `[装] 这具机体认哪种命令:末端那六个自由度` —— 它不吃关节命令。
    // 后果:两条臂一下都没动 ⇒ 两个末端位移都是 0 ⇒ argmax 默认落回 0 ⇒
    // 两条臂都判成"末端号 0",而主控路发的正是末端命令 ⇒ 右臂照样一次不动。
    //
    // ⇒ **逐个末端**发它认的那种命令推一小步,一次动作同时量到三样:
    //   ① 每台相机跟着它变多少 ⇒ 主眼 = 对谁都不怎么变的那台(长在世界里)
    //   ② 它认不认这条命令(实到 / 命令)
    //   ③ 它在主眼画面里占哪一片(变化像素重心)+ 哪一组关节跟着变最多(= 长着它的那条臂)
    // 眼指出目标后,**离目标最近的那个末端**就是该用的手。不认识"左/右",几个末端都成立。
    let (相机号, 各台变, 臂心, 臂末, 臂关) = {
        let 台 = plug.lay.cams.len();
        if 台 == 0 { println!("[服] 🔴 一台相机都没认出来 ⇒ 干不了。**不编数。**"); return None }
        let Some(f0) = plug.sense() else { return None };
        let 末数 = f0.ee.len().max(1);
        let 组号 = 真臂(&f0);
        let mut 变: Vec<f64> = vec![0.0; 台];
        let mut 臂心: Vec<Vec<Option<(f64, f64)>>> = vec![vec![None; 台]; 末数];
        let mut 臂末: Vec<Option<usize>> = vec![None; 末数];
        let mut 臂关: Vec<usize> = vec![0; 末数];
        // 10 是**比例**(量出来的可达带的十分之一),不是米。
        let 步 = 可达带[1] / 10.0;
        for i in 0..末数 {
            let Some(fa) = plug.sense() else { break };
            let 前: Vec<Option<(usize, usize, Vec<u8>)>> = (0..台).map(|c| 灰(&fa, c)).collect();
            let Some(e0) = fa.ee.get(i).cloned() else { continue };
            if e0.len() < 7 { continue }
            let 前组: Vec<Vec<f64>> = fa.joints.clone();
            let j0 = fa.jaw.get(i).or_else(|| fa.jaw.first()).copied().unwrap_or(1.0);
            let q0 = [e0[3], e0[4], e0[5], e0[6]];
            plug.act(&Cmd::Ee { arm: i, at: [e0[0] + 步, e0[1], e0[2]], quat: q0, jaw: j0 });
            for _ in 0..等拍 { if plug.sense().is_none() { break } }
            let Some(f1) = plug.sense() else { break };
            let 实到 = f1.ee.get(i).map(|e|
                ((e[0]-e0[0]).powi(2)+(e[1]-e0[1]).powi(2)+(e[2]-e0[2]).powi(2)).sqrt()).unwrap_or(0.0);
            let mut 最组 = (0usize, -1.0f64);
            for &g in 组号.iter() {
                let (Some(a0), Some(a1)) = (前组.get(g), f1.joints.get(g)) else { continue };
                if a0.len() != a1.len() || a0.is_empty() { continue }
                let d: f64 = a0.iter().zip(a1.iter()).map(|(x, y)| (x - y).abs()).sum();
                if d > 最组.1 { 最组 = (g, d) }
            }
            if 实到 > 0.0 { 臂末[i] = Some(i); 臂关[i] = 最组.0; }
            for c in 0..台 {
                let (x0, y0) = (前[c].clone(), 灰(&f1, c));
                match (x0, y0) {
                    (Some((w, h, x)), Some((_, _, y))) if x.len() == y.len() && !x.is_empty() => {
                        let mut n = 0usize; let (mut su, mut sv) = (0.0f64, 0.0f64);
                        for k in 0..x.len() {
                            // 8 是灰度差的噪声底(和认接触面那一处同一个来源),无量纲。
                            if x[k].abs_diff(y[k]) > 8 {
                                n += 1; su += (k % w) as f64 / w as f64; sv += (k / w) as f64 / h as f64;
                            }
                        }
                        let v = n as f64 / x.len() as f64;
                        if v > 变[c] { 变[c] = v }
                        // 🔴 **动的像素太少 = 它其实没动** —— 那一片是噪声,不许当成"这只手在这儿"。
                        if v > 0.001 { 臂心[i][c] = Some((su / n as f64, sv / n as f64)); }
                    }
                    _ => { 变[c] = 变[c].max(1.0); }
                }
            }
            plug.act(&Cmd::Ee { arm: i, at: [e0[0], e0[1], e0[2]], quat: q0, jaw: j0 });
            for _ in 0..等拍 { if plug.sense().is_none() { break } }
            println!("[服] 第 {i} 个末端:命令走 {步:.4} m,**实到 {实到:.4} m** · 跟着变最多的关节组 {}", 最组.0);
        }
        let i = if 台 == 1 { 0usize } else {
            变.iter().enumerate().fold((0usize, f64::INFINITY), |a, (i, v)| if *v < a.1 { (i, *v) } else { a }).0
        };
        println!("[服] 手动一下,各台画面变了多大一片:{:?}", 变.iter().map(|v| (v * 1000.0).round() / 1000.0).collect::<Vec<_>>());
        println!("[服] ⇒ **主眼用第 {i} 台**(它变得最少 = 长在世界里,看得见工作区)");
        for a in 0..末数 {
            println!("[服]   第 {a} 个末端 ⇒ 关节组 {} · 在主眼里占的那一片中心 {:?}",
                臂关[a], 臂心[a].get(i).and_then(|o| *o).map(|(u, v)| ((u*1000.0).round()/1000.0, (v*1000.0).round()/1000.0)));
        }
        (i, 变, 臂心, 臂末, 臂关)
    };
    // 🔴🔴🔴 **最后一两厘米交给【长在手上】的那台相机。**(owner 2026-08-27 定)
    //
    // 所有难题都挤在"快碰上的那一两厘米":**头部相机在那个尺度上先天不准** ——
    // 爪子只有二三十个像素、被自己挡、会走出画面、两指连成一块。我为此发明了一串图像小技巧
    // (取最大块 / 沿长边取两端 / 两帧相减 / 取最强像素),**每换一个技巧就带来一批新 bug**
    // —— 今晚十个 bug 里有一半是同一个问题:"我的手在画面里的哪儿"。
    //
    // 而**长在手上的那台相机里,手是不动的、而且占满画面** ⇒ **"找我的手"这个问题整个消失**。
    // ⚠️ 文档 2026-08-15 判过"腕相机当**主眼**"不行(*"腕相机首帧里只有桌面 + 左右下角两片指头,
    //   没有物体"*)—— 判的是**开局**,不是**收尾**。头部把手带过去、腕相机收最后一寸,
    //   正是"全局 + 操作部相机"那个结构,两台各用在自己准的那一段。
    // 哪一台长在手上,是**试出来的**:命令手动一下,**画面整体跟着动的那台**就是它
    //(长在世界里的那台,画面里只有手那一小块在动)。
    let 手上相机 = if let Some(c) = *手上相机载 {
        println!("[服] 长在手上的是第 {c} 台(装回来的,不重探)");
        Some(c)
    } else {
        let 台 = plug.lay.cams.len();
        let mut 挑: Option<usize> = None;
        if 台 >= 2 {
            let Some(f0) = plug.sense() else { return None };
            let Some(e0) = f0.ee.get(手号.get()).copied() else { return None };
            let 前: Vec<Option<Vec<u8>>> = (0..台).map(|c| 灰(&f0, c).map(|(_, _, g)| g)).collect();
            // 10 是**比例**(量出来的可达带的十分之一),不是米。
            let 步 = 可达带[1] / 10.0;
            let Some((f1, _, _)) = 落(plug, [e0[0] + 步, e0[1], e0[2]], [e0[3], e0[4], e0[5], e0[6]], 1.0, 等拍) else { return None };
            let mut 变: Vec<f64> = Vec::new();
            for c in 0..台 {
                let (a, b) = (前[c].clone(), 灰(&f1, c).map(|(_, _, g)| g));
                变.push(match (a, b) {
                    (Some(x), Some(y)) if x.len() == y.len() && !x.is_empty() =>
                        x.iter().zip(y.iter()).filter(|(p, q)| p.abs_diff(**q) > 8).count() as f64 / x.len() as f64,
                    _ => 0.0,
                });
            }
            println!("[服] 手动一下,各台画面变了多大一片:{:?}", 变.iter().map(|v| (v * 100.0).round() / 100.0).collect::<Vec<_>>());
            // 长在手上的那台:整幅画面都在变(相机自己在动);长在世界里的那台只有一小块在变。
            // 🔴 判据是**相对**的,不是一个绝对百分比:长在手上的那台,变化远大于长在世界里的那台。
            // 实测代价(W1):两台变了 **[0.06, 0.21]** —— 差 **3.5 倍**,分得清清楚楚,
            // 却被我写的"必须超过 30%"这个**手填绝对阈值**拦住,收尾整段没跑。
            // 改成倍数(次大的两倍以上),顺手又少一个手填的数。
            let (i, v) = 变.iter().enumerate().fold((0usize, 0.0f64), |a, (i, v)| if *v > a.1 { (i, *v) } else { a });
            let 次 = 变.iter().enumerate().filter(|(j, _)| *j != i).fold(0.0f64, |a, (_, v)| a.max(*v));
            if v > 次 * 2.0 && v > 0.0 && i != 相机号 {
                println!("[服] ⇒ 第 {i} 台**长在手上**(它变了 {:.0}%,别的最多 {:.0}% —— 差 {:.1} 倍)⇒ 最后一两厘米交给它",
                    v * 100.0, 次 * 100.0, v / 次.max(1e-9));
                挑 = Some(i);
            } else {
                println!("[服] ⇒ 没认出哪台长在手上(最大 {:.0}%,次大 {:.0}%,差不到两倍)—— 只用第 {相机号} 台,收尾精度照实降级",
                    v * 100.0, 次 * 100.0);
            }
        }
        挑
    };
    *手上相机载 = 手上相机;
    let _ = &相机们;

    /// 老名字:等手臂停稳再返回。量表、追、退回都要它 —— 它们要读这一步的结果。
    let 迈通道 = |plug: &mut Plug<S>, 动: &[f64], 比: f64, jaw: f64, 是关节: bool| -> Option<(f64, Vec<f64>, bool)> {
        迈通道稳(plug, 动, 比, jaw, 是关节, 等拍 * 2)
    };

    /// 一个像素周围那一小窗里的**近侧**深度。窗里一半是物体、一半是它后面的桌面,
    /// 中位会滑到桌面那一半去 ⇒ 取近侧四分之一。分位数是数据自己的,不是填的。
    let 近侧深 = |plug: &mut Plug<S>, u: f64, v: f64, 窗: f64| -> Option<f64> {
        let 路 = plug.lay.cams.get(相机号)?;
        let mut 深路 = 路.clone();
        // 🔴 深度路径由 `discover` 靠形状认出来,不许把彩色路径末段换成字面量 `depth`(减法①)。
        if let Some(dp) = plug.lay.depth.get(相机号).or_else(|| plug.lay.depth.first()) { 深路 = dp.clone(); }
        let dv = plug.last.as_ref().and_then(|o| 取(o, &深路))?;
        let (dw, dh, dep) = wire::as_f32_grid(&dv)?;
        let x0 = (((u - 窗) * dw as f64).floor().max(0.0)) as usize;
        let x1 = (((u + 窗) * dw as f64).ceil().min(dw as f64 - 1.0)).max(0.0) as usize;
        let y0 = (((v - 窗) * dh as f64).floor().max(0.0)) as usize;
        let y1 = (((v + 窗) * dh as f64).ceil().min(dh as f64 - 1.0)).max(0.0) as usize;
        let mut 有: Vec<f64> = Vec::new();
        for y in y0..=y1.min(dh - 1) { for x in x0..=x1.min(dw - 1) {
            let d = dep[y * dw + x] as f64;
            if d.is_finite() && d > 0.0 { 有.push(d) }
        }}
        if 有.is_empty() { return None }
        有.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        Some(有[有.len() / 4])
    };
    // `近侧深` 只认那一台;收尾要在**另一台**上读深度 ⇒ 同一段逻辑,相机号当参数。
    let 近侧深2 = |plug: &mut Plug<S>, ci: usize, u: f64, v: f64, 窗: f64| -> Option<f64> {
        let 路 = plug.lay.cams.get(ci)?;
        let mut 深路 = 路.clone();
        // 🔴 深度路径由 `discover` 靠形状认出来,不许把彩色路径末段换成字面量 `depth`(减法①)。
        if let Some(dp) = plug.lay.depth.get(ci).or_else(|| plug.lay.depth.first()) { 深路 = dp.clone(); }
        let dv = plug.last.as_ref().and_then(|o| 取(o, &深路))?;
        let (dw, dh, dep) = wire::as_f32_grid(&dv)?;
        let x0 = (((u - 窗) * dw as f64).floor().max(0.0)) as usize;
        let x1 = (((u + 窗) * dw as f64).ceil().min(dw as f64 - 1.0)).max(0.0) as usize;
        let y0 = (((v - 窗) * dh as f64).floor().max(0.0)) as usize;
        let y1 = (((v + 窗) * dh as f64).ceil().min(dh as f64 - 1.0)).max(0.0) as usize;
        let mut 有: Vec<f64> = Vec::new();
        for y in y0..=y1.min(dh - 1) { for x in x0..=x1.min(dw - 1) {
            let d = dep[y * dw + x] as f64;
            if d.is_finite() && d > 0.0 { 有.push(d) }
        }}
        if 有.is_empty() { return None }
        有.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        Some(有[有.len() / 4])
    };
    // 0.05 是**残留比例**(走到只剩 5% 就算到位),400 是**拍数**上限——两者都无量纲。
    let 步数 = (0.05f64.ln() / (1.0 - 交付率).max(1e-6).ln()).ceil().clamp(3.0, 400.0) as u32;

    // 🔴🔴🔴 **不解相机 —— 只量「手往世界挪一下,它在画面上跑多少」。**(改架构 2026-08-26)
    //
    // 旧路是先把整台相机解出来(焦距 + 主点 + 相机在世界哪儿),再拿它把像素换算成三维点。
    // **连着六炮解不出来,而且每炮的拒绝理由都不一样** —— 点共面 / 走出画面 / 只在一根轴上蹭 /
    // 点太扁 / 姿态不变导致偏置与相机位置分不开。它们全是**同一类病:全局解不唯一**。
    // 而且这条路和这个项目的硬约束**结构性冲突**:相机会动(移动底盘 / 头 / 直播),
    // 一开机解一次的位姿当场作废,而解一次要吃掉 20 集以上。
    //
    // ⇒ 换成**局部、可现量、天天在更新**的那一个:一张 3×3 的表 `雅`,定义是
    //   `Δ(画面横, 画面纵, 那一点的深) = 雅 · Δ(世界 x, y, z)`。
    // 量法:手往世界三个方向各挪一小步,记下爪子在画面上的三个数各变了多少。**三步就有。**
    // 用法:要把指尖挪到目标那个像素,`Δ世界 = 雅⁻¹ · (目标三数 − 我的三数)`,走一步,重量,再来。
    //
    // 它为什么比解相机好:
    //   ① **没有全局退化**。全局解不唯一的那六种病一个都不存在 —— 它只描述"此时此地"。
    //   ② **相机自己在动也不怕**:每次用之前先核对,对不上就当场重量(20 集 → 3 步)。
    //   ③ **错了当场看得见**:手没往预期方向跑,下一步就纠回来;解相机是量 20 集才知道对不对。
    //   ④ **顺手把尺也带出来了**:`雅⁻¹` 作用在一个画幅单位上,长度就是"此深度下 1 画幅 = 几米"
    //      ⇒ 爪子有多宽、该抬多高,全部从它现读,不需要焦距。
    // 代价照记:它只在量它的那一小片邻域成立 ⇒ **必须闭环迭代**,不能一步到位。

    /// 爪子此刻在画面的哪儿。返回 (横, 纵, 那一点的深, 那一块占画幅多宽, 开合方向的画面向量)。
    /// 手臂冻住、只动爪子 ⇒ 会动的那一块按构造就是钳口(五帧合同交给仓里的认块器,认不出会拒)。
    /// **反投影没有了,相机模型也没有了** —— 留在画面里。
    // `上眼` = 上一次看到爪子在画面哪儿。给了就**挑离它最近的那个候选**。
    let 看爪幅 = |plug: &mut Plug<S>, at: [f64; 3], q: [f64; 4], j0: f64, 上眼: Option<(f64, f64)>, 幅: f64|
        -> Option<(f64, f64, f64, f64, f64, (f64, f64), Vec<(f64, f64)>, [f64; 4])> {
        // 0.30 是**钳口命令行程的分数**(命令域本身就是 0..1),无量纲。
    let 步 = (if j0 > 0.5 { -1.0 } else { 1.0 }) * 0.30 * 幅;
        let mut 帧们 = Vec::new();
        // 第一张空帧:**先等手臂停住**(见 `稳住` 上面那段 —— 余晃会把噪声地板抬高)。
        帧们.push(稳住(plug, at, q, j0, 等拍 * 2)?);
        let 基: Vec<f64> = 帧们[0].jaw.iter().copied().collect();
        帧们.push(晃(plug, at, q, j0, &基, false, 等拍)?);
        for k in 1..=3 { 帧们.push(晃(plug, at, q, (j0 + 步 * k as f64).clamp(0.0, 1.0), &基, true, 等拍)?); }
        // 🔴 末尾那一次"把爪子放回原位"删掉 —— 调用方下一条命令本来就会重设爪子,
        // 而它要等爪子走完,白烧十来拍。一集只有 200 拍,这是省得起的十分之一。
        // 🔴🔴🔴 **五帧之间胳膊必须【真的没动】—— 动过就作废重来,不许拿脏样本认爪。**
        //(ZK 渲图定案 2026-08-30)
        //
        // 这个办法的全部前提是:*锁死胳膊、只张合手指 ⇒ 画面里动的按定义就是手指*,
        // 而**大臂的位移精确为零、根本没机会参赛**(`LAB "认手不许靠「哪块动得最像我」"`)。
        // 前提一破,又大又亮的大臂就赢下这一票。
        // ZK 实证:`配对 4 · 占画幅 0.046` 读起来完全正常,而渲图显示那个点**画在大臂上**,
        // 真正的爪子当时已经开到棒球正旁边(`ZK_方框落在哪.png`)。
        // 机制:`晃` 每一帧都重发同一个位姿,而这具身体**一拍只交付约 10%** ⇒
        // 五帧之间胳膊一直在往那个位姿爬 —— 日志早就警告过 `空帧那会儿手臂还在晃`。
        //
        // 对策仓里有现成的,只是装错了地方:`LAB` 2026-08-16 的重复性实验里
        // *"朝向变过 0.5° 就把已攒的作废重来"*,当时**连响 8 次**,证明污染是真的。
        // 这里照搬:门槛由量出来的东西给 —— 五帧之间的漂移必须**小于一步真正走得动的距离**
        // 的十分之一(`探步 × 交付率 × 0.1`),否则这一眼不算数。
        {
            let 位们: Vec<[f64; 3]> = 帧们.iter()
                .filter_map(|f| f.ee.get(手号.get()).map(|e| [e[0], e[1], e[2]])).collect();
            if 位们.len() >= 2 {
                let mut 漂 = 0.0f64;
                for a in &位们 { for b in &位们 {
                    漂 = 漂.max(((a[0]-b[0]).powi(2) + (a[1]-b[1]).powi(2) + (a[2]-b[2]).powi(2)).sqrt());
                }}
                // 10 是"可达带的十分之一 = 一步"(和 `探步` 同一个来源,它在这一段之后才定义);
                // 0.1 是余量比例。两个都是无量纲的,底下的量全是量出来的。
                let 界 = 可达带[1] / 10.0 * 交付率 * 0.1;
                if 漂 > 界 {
                    // 🔴 **改成标脏,不再作废。**(owner 2026-08-31)
                    // 前提("只动手指")确实破了 —— 但**作废一次就是一拍不干活**,而这具身体
                    // 交付只有约 10%,漂移几乎每次都超界 ⇒ 等于永远不干。
                    // ⇒ 照实说这一眼可能混进了大臂,**照样用**;下游有"占画幅太大"和碰撞去纠它。
                    println!("[服]   看爪:⚠️ 这五帧之间胳膊动了 {漂:.4} m(界 {界:.4} m)⇒ 可能混进大臂;**照样用**,结果照实报");
                }
            }
        }
        let (w, h, _) = 灰(&帧们[0], 相机号)?;
        let g: Vec<Vec<u8>> = 帧们.iter().filter_map(|f| 灰(f, 相机号).map(|(_, _, d)| d)).collect();
        if g.len() < 5 { println!("[服]   看爪:只拿到 {} 帧(要 5 帧)", g.len()); return None }
        let r = match body_layer::blob::candidates(&g[0], &g[1], &g[2], &g[3], &g[4], w, h,
                步.abs(), selfcal::最少像素(w, h)) {
            Ok(r) => r,
            Err(e) => { println!("[服]   看爪:认块器拒绝 {e:?} —— 这一眼不算数"); return None }
        };
        // 🔴🔴 **认块器给的是一堆候选,不许闭眼取第一个。**
        //
        // 实测代价(S9):第三下探针取到的那一块**占了 35% 的画面**(平时 5%)、双响 6110 ——
        // 那不是爪子,是**整条手臂**:手臂刚走完还在晃,晃爪子那五帧里整条臂都在动。
        // 于是那一列的画面响应量成 -3.9(x 那一列只有 0.68),雅可比的第三列整根废掉。
        // ⇒ 挑**离上一次看到的位置最近的那个** —— 手一步只挪几厘米,爪子在画面上不可能瞬移。
        //   这是跟踪(用已经量到的连续性),不是猜。第一次没有上一眼时才取第一个。
        if r.cands.is_empty() {
            // 🔴 诊断句不许一句话套两种病。地板**高** = 空帧那会儿手臂还在晃(噪声把信号淹了);
            // 地板**低**而双响还是 0 = 手指真的没在画面上动(多半被腕部挡住了,只有转手腕有用)。
            // 印错的诊断比不印更贵 —— 会把人往错的方向修。
            println!("[服]   看爪:认块器没给出候选(双响 {} · 配对 {} · 地板 {})—— {}",
                r.moved_px, r.pairs, r.floor,
                if r.floor > 40 { "**地板高**:空帧那会儿手臂还在晃,信号被噪声淹了" }
                else { "**地板低但一样没动**:手指真的没在画面上张合,多半被腕部挡住" });
            return None
        }
        let 头 = *r.cands.get(0)?;
        let c = match 上眼 {
            None => 头,
            Some((pu, pv)) => {
                let (mut best, mut bd) = (头, f64::MAX);
                for i in 0..r.cands.len() {
                    let Some(k) = r.cands.get(i) else { continue };
                    let d = ((k.u - pu).powi(2) + (k.v - pv).powi(2)).sqrt();
                    if d < bd { bd = d; best = *k }
                }
                if r.cands.len() > 1 {
                    println!("[服]   看爪:{} 个候选,挑离上一眼 ({pu:.3},{pv:.3}) 最近的那个(差 {bd:.3} 画幅)", r.cands.len());
                }
                best
            }
        };
        let c = &c;
        let (u0, v0, u1, v1) = (c.ext[0], c.ext[1], c.ext[2], c.ext[3]);
        let (du, dv) = if r.pairs > 0 && (r.pair_dir.0.abs() + r.pair_dir.1.abs()) > 1e-9 {
            r.pair_dir
        } else if (u1 - u0) >= (v1 - v0) { (1.0, 0.0) } else { (0.0, 1.0) };
        // 🔴🔴 **"那一块有多宽"要沿【开合方向】量,不许用外接框的对角线。**
        //
        // 会动的那一块 = 晃爪子时所有动了的像素,里面还含着腕部;**对角线把"手指有多长"
        // 和"腕子有多大"一起算了进去**,而钳口只沿开合方向张。
        // 实测代价(G1):爪子宽算成 **0.187 m**(Franka 实际约 0.08),连带
        // "凸出桌面多少才算物体"的门槛被抬到 1.87 cm,把一把几毫米厚的剪刀整个减掉。
        // 沿开合方向取投影,垂直那一维(手指长度 / 腕子)自然不进来。
        // 🔴🔴 **"钳口能张多开"取外接框的【长边】,不取对角线、也不取沿配对方向的投影。**
        //
        // · 对角线 = √(长²+短²) ⇒ 把"手指有多长 / 腕子有多大"一起算进来
        //   ⇒ 实测(G1)爪子宽算成 **0.187 m**(Franka 实际约 0.08),连带"凸出桌面多少算物体"
        //   的门槛被抬到 1.87 cm,把几毫米厚的剪刀整个减掉。
        // · 沿配对方向投影 ⇒ 方向取错时**塌到 0.002 个画幅**(G5 实测),爪子宽算成 0,候选全被拒。
        // · 长边:平行爪张开时,会动的那一片**本来就沿张开方向拉长** ⇒ 长边就是它,而且**永远不会塌**。
        // ⚠️ 仍然欠一条更正的:**全开时的两指间距 − 全合时的两指间距**(差值天生免疫"多认了半个腕子")。
        let 张 = (u1 - u0).abs().max((v1 - v0).abs()).max(1e-6);
        // 🔴🔴 **这一块【整体】有多大,和【沿开合方向】有多宽,是两件事,不许混用。**
        // 实测代价(G3):我把开合方向的投影同时当成了"读深度的取样窗",而投影会塌到
        // **0.001 个画幅** ⇒ 窗小到一个像素 ⇒ 读到的是**背景 2.442 m**,整条链全污染。
        // 窗要用**整块**的大小(外接框对角线),爪子宽才用开合方向的投影。
        let 块 = ((u1 - u0).powi(2) + (v1 - v0).powi(2)).sqrt().max(1e-3);
        // 🔴🔴 **这一块【整体】有多大,和【沿开合方向】有多宽,是两件事,不许混用。**
        // 实测代价(G3):我把开合方向的投影同时当成了"读深度的取样窗",而投影会塌到
        // **0.001 个画幅** ⇒ 窗小到一个像素 ⇒ 读到的是**背景 2.442 m**,整条链全污染。
        // 窗要用**整块**的大小(外接框对角线),爪子宽才用开合方向的投影。
        let 块 = ((u1 - u0).powi(2) + (v1 - v0).powi(2)).sqrt().max(1e-3);
        let Some(深) = 近侧深(plug, c.u, c.v, 块 * 0.5) else {
            println!("[服]   看爪:认出那一块在 ({:.3},{:.3}),但那儿读不到深度", c.u, c.v);
            return None
        };
        println!("[服]   看爪:双响 {} · 配对 {} · 在画面 ({:.3},{:.3}) · 深 {:.3} m · 占画幅 {:.3}",
            r.moved_px, r.pairs, c.u, c.v, 深, 张);
        // 🔴🔴🔴 **两个接触面 = 认块器给出的【最大的两块】,不是我拿偏移凑的两个patch。**
        //
        // 实测代价(F7):我拿"那一块的中心 ± 四分之一长度"切两个模板,跟踪器跟的是**图像纹理**
        // 而不是手指 ⇒ "接触面最远能分开多少"量出 **2.9 mm**(Franka 实际约 80 mm),
        // 而且"合到底 0.0306 比张到底 0.0277 还大" —— **方向都反了**。
        // 认块器本来就在做这件事:晃抓握通道时**往相反方向动的那两块**就是两根手指
        // (它连 `pairs` 都数出来了),我却只取了 `cands[0]`,第二个自己编。
        // ⇒ 直接取**最大的两块**。一块 = 吸盘(单点接触集)、两块 = 两指、五块 = 五指,同一段代码。
        let mut 块们: Vec<(f64, f64, u32)> = Vec::new();
        for i in 0..r.cands.len() {
            if let Some(k) = r.cands.get(i) { 块们.push((k.u, k.v, k.pixels)); }
        }
        块们.sort_by(|a, b| b.2.cmp(&a.2));
        let mut 面们: Vec<(f64, f64)> = 块们.iter().take(2).map(|k| (k.0, k.1)).collect();
        // 🔴🔴🔴 **认块器说没认出一对钳口,就是没认出。不许在这里编第二个点。**
        //
        // `fold_opposed` 的注释写得很清楚:一对钳口不是"解"出来的,是**认**出来的 ——
        // 两瓣朝**相反方向**走、位移互相抵消,而"抵消"的容差**不是填的**,是这两块自己量到的散布。
        // `pairs == 1` ⇒ 输出的那个候选就是**两指中点**,`pair_dir` 就是**钳口轴**,
        // 于是两个接触面 = 中点 ∓ pair_dir/2,**全是量出来的**。
        //
        // 实测代价(2026-08-27 通宵):`pairs == 0` 时我沿"外接框的长边"取两端编了两个点 ——
        // 而那一片是 **93×127**(竖着更长),于是编出来的两个点是这一片的**上端和下端**,
        // 可放大看,两根手指是**左右**分开的。**编出来的答案连轴都是错的**,
        // 后面所有的跟丢、锁错、闸门误判全长在它上面,我为此手写了掩码/连通块/预测窗一整套,
        // 全是在给一个编造的输入打补丁。**仓里那个读数器整晚都在日志里说 `配对 0`,我没听。**
        if r.pairs > 0 && (r.pair_dir.0.abs() + r.pair_dir.1.abs()) > 1e-9 {
            let (hu, hv) = (r.pair_dir.0 * 0.5, r.pair_dir.1 * 0.5);
            面们 = vec![(c.u - hu, c.v - hv), (c.u + hu, c.v + hv)];
            println!("[服]   认出一对钳口 ⇒ 两个接触面 = 中点 ∓ 钳口轴/2 = ({:.3},{:.3}) / ({:.3},{:.3})",
                面们[0].0, 面们[0].1, 面们[1].0, 面们[1].1);
        } else if 面们.len() < 2 {
            println!("[服]   🔴 认块器**没认出一对钳口**(配对 {} · 候选 {})⇒ 这台相机里我分不出自己的两瓣。\
                     **不编造** —— 这一眼只交出一个接触面,由调用方决定降级怎么走", r.pairs, r.cands.len());
        }
        if 面们.len() >= 2 {
            println!("[服]   认出 {} 块 ⇒ 两个接触面在 ({:.3},{:.3}) 和 ({:.3},{:.3})",
                r.cands.len(), 面们[0].0, 面们[0].1, 面们[1].0, 面们[1].1);
        } else {
            println!("[服]   只认出 {} 块 ⇒ 这具手只有一个接触面(吸盘那一类)", r.cands.len());
        }
        Some((c.u, c.v, 深, 张, 块, (du, dv), 面们, [u0, v0, u1, v1]))
    };

    /// 3×3 求逆。奇异就拒(不硬解)。
    let 逆3 = |m: [[f64; 3]; 3]| -> Option<[[f64; 3]; 3]> {
        let det = m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1])
                - m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0])
                + m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0]);
        if !det.is_finite() || det.abs() < 1e-12 { return None }
        Some([
            [(m[1][1]*m[2][2]-m[1][2]*m[2][1])/det, (m[0][2]*m[2][1]-m[0][1]*m[2][2])/det, (m[0][1]*m[1][2]-m[0][2]*m[1][1])/det],
            [(m[1][2]*m[2][0]-m[1][0]*m[2][2])/det, (m[0][0]*m[2][2]-m[0][2]*m[2][0])/det, (m[0][2]*m[1][0]-m[0][0]*m[1][2])/det],
            [(m[1][0]*m[2][1]-m[1][1]*m[2][0])/det, (m[0][1]*m[2][0]-m[0][0]*m[2][1])/det, (m[0][0]*m[1][1]-m[0][1]*m[1][0])/det],
        ])
    };
    let 看爪 = |plug: &mut Plug<S>, at: [f64; 3], q: [f64; 4], j0: f64, 上眼: Option<(f64, f64)>|
        -> Option<(f64, f64, f64, f64, f64, (f64, f64), Vec<(f64, f64)>, [f64; 4])> { 看爪幅(plug, at, q, j0, 上眼, 1.0) };

    /// **超定最小二乘:`A x ≈ b`,A 是 m×n(m 个方程、n 个通道)。**
    /// 正规方程 `(AᵀA + λI) x = Aᵀb`,高斯消元。λ 只用来在通道多于约束时挑**最小动作**那一解
    /// (无量纲:取 AᵀA 对角线的均值的百万分之一),**不改变有解时的答案**。
    /// 解不出来就拒(不硬解)。通道数由这具身体报的自由度决定,这里不设上限形状。
    let 最小二乘 = |a: &[Vec<f64>], b: &[f64]| -> Option<Vec<f64>> {
        let m = a.len();
        if m == 0 || b.len() != m { return None }
        let n = a[0].len();
        if n == 0 || a.iter().any(|r| r.len() != n) { return None }
        let mut ata = vec![vec![0.0f64; n + 1]; n];
        for i in 0..n {
            for j in 0..n { ata[i][j] = (0..m).map(|k| a[k][i] * a[k][j]).sum(); }
            ata[i][n] = (0..m).map(|k| a[k][i] * b[k]).sum();
        }
        let 迹: f64 = (0..n).map(|i| ata[i][i]).sum::<f64>() / n as f64;
        if !(迹.is_finite() && 迹 > 0.0) { return None }
        for i in 0..n { ata[i][i] += 迹 * 1e-6; }
        for c in 0..n {
            let mut p = c;
            for r in (c + 1)..n { if ata[r][c].abs() > ata[p][c].abs() { p = r } }
            if ata[p][c].abs() < 1e-15 { return None }
            ata.swap(c, p);
            let d = ata[c][c];
            for j in c..=n { ata[c][j] /= d; }
            for r in 0..n {
                if r == c { continue }
                let f = ata[r][c];
                if f == 0.0 { continue }
                for j in c..=n { ata[r][j] -= f * ata[c][j]; }
            }
        }
        let x: Vec<f64> = (0..n).map(|i| ata[i][n]).collect();
        if x.iter().all(|v| v.is_finite()) { Some(x) } else { None }
    };

    /// 3×3 解:`雅 · x = e`。奇异就拒(不硬解)。
    let 解3 = |m: [[f64; 3]; 3], e: [f64; 3]| -> Option<[f64; 3]> {
        let det = m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1])
                - m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0])
                + m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0]);
        // 🔴🔴🔴 **奇异不许拒绝求解 —— 换阻尼最小二乘,给一个能动的方向。**(owner 2026-08-31)
        // owner 原话:*"一个一直拒绝工作的 driver 没有任何 ux,就算把桌子上的东西全砸碎
        // 也是我们 driver 牛逼"*。矩阵奇异说明"有些方向这几列管不着",而**管得着的那些方向
        // 照样该动**。阻尼最小二乘 (MᵀM + λI)⁻¹Mᵀe 在满秩时等于精确解,缺秩时给出
        // **能动的那部分**,永远有输出。λ 由矩阵自己的尺度给(迹的千分之一),不是填的。
        if !det.is_finite() || det.abs() < 1e-12 {
            let mut ata = [[0.0f64; 3]; 3];
            let mut atb = [0.0f64; 3];
            for i in 0..3 { for j in 0..3 { for k in 0..3 { ata[i][j] += m[k][i] * m[k][j]; } } }
            for i in 0..3 { for k in 0..3 { atb[i] += m[k][i] * e[k]; } }
            let 迹 = ata[0][0] + ata[1][1] + ata[2][2];
            if !(迹 > 0.0) { println!("[服]   ⚠️ 这几列一个方向都管不着(全零)⇒ 这一步不动,照实说"); return None }
            let lam = 迹 * 1e-3;
            for i in 0..3 { ata[i][i] += lam; }
            let d2 = ata[0][0]*(ata[1][1]*ata[2][2]-ata[1][2]*ata[2][1])
                   - ata[0][1]*(ata[1][0]*ata[2][2]-ata[1][2]*ata[2][0])
                   + ata[0][2]*(ata[1][0]*ata[2][1]-ata[1][1]*ata[2][0]);
            if !d2.is_finite() || d2.abs() < 1e-18 { return None }
            let iv = [
                [(ata[1][1]*ata[2][2]-ata[1][2]*ata[2][1])/d2, (ata[0][2]*ata[2][1]-ata[0][1]*ata[2][2])/d2, (ata[0][1]*ata[1][2]-ata[0][2]*ata[1][1])/d2],
                [(ata[1][2]*ata[2][0]-ata[1][0]*ata[2][2])/d2, (ata[0][0]*ata[2][2]-ata[0][2]*ata[2][0])/d2, (ata[0][2]*ata[1][0]-ata[0][0]*ata[1][2])/d2],
                [(ata[1][0]*ata[2][1]-ata[1][1]*ata[2][0])/d2, (ata[0][1]*ata[2][0]-ata[0][0]*ata[2][1])/d2, (ata[0][0]*ata[1][1]-ata[0][1]*ata[1][0])/d2],
            ];
            println!("[服]   ⚠️ 方向盘缺秩(det≈0)⇒ 用阻尼最小二乘走**管得着的那部分**,照实说不精确");
            return Some([
                iv[0][0]*atb[0]+iv[0][1]*atb[1]+iv[0][2]*atb[2],
                iv[1][0]*atb[0]+iv[1][1]*atb[1]+iv[1][2]*atb[2],
                iv[2][0]*atb[0]+iv[2][1]*atb[1]+iv[2][2]*atb[2],
            ]);
        }

        let inv = [
            [(m[1][1]*m[2][2]-m[1][2]*m[2][1])/det, (m[0][2]*m[2][1]-m[0][1]*m[2][2])/det, (m[0][1]*m[1][2]-m[0][2]*m[1][1])/det],
            [(m[1][2]*m[2][0]-m[1][0]*m[2][2])/det, (m[0][0]*m[2][2]-m[0][2]*m[2][0])/det, (m[0][2]*m[1][0]-m[0][0]*m[1][2])/det],
            [(m[1][0]*m[2][1]-m[1][1]*m[2][0])/det, (m[0][1]*m[2][0]-m[0][0]*m[2][1])/det, (m[0][0]*m[1][1]-m[0][1]*m[1][0])/det],
        ];
        Some([
            inv[0][0]*e[0]+inv[0][1]*e[1]+inv[0][2]*e[2],
            inv[1][0]*e[0]+inv[1][1]*e[1]+inv[1][2]*e[2],
            inv[2][0]*e[0]+inv[2][1]*e[1]+inv[2][2]*e[2],
        ])
    };

    // 🔴🔴🔴 **抓握也要走【这具身体真正响应的那种命令】。**
    //
    // 实测代价(F6):我用 `Cmd::Joints` 去命令爪子开合,而这具机体**根本不响应关节命令**
    //(F2 已经验过:七个关节命令全零响应)⇒ 爪子从头到尾没动 ⇒
    // "接触面最远能分开多少"量出 **张到底 0.0243 / 合到底 0.0243**(一模一样,差 0),
    // 而且**合爪那一步也永远合不上** —— 一个 bug,两处致命,而两处的病相看起来毫不相干。
    // ⇒ 通道种类是试出来的,那就**用到底**:关节机体走关节,末端机体走末端,同一个函数。
    /// 把抓握通道开到某个开度。**两种命令形式都发一遍 —— 机体认哪种就走哪种,不靠外面先告诉我。**
    ///
    /// 🔴🔴 实测代价(NVH,2026-08-27,**渲图看出来的,日志上看不出来**):
    /// 我把"认接触面"挪到了建表之前,而它发抓握命令时用的"这具机体认哪种命令"还是默认值(关节),
    /// 而这台机体**静默丢弃**关节命令 ⇒ 爪子一下都没动 ⇒ 张合两帧一模一样 ⇒ 认不出接触面 ⇒
    /// 建表永远到不了 —— 而"认哪种命令"恰恰是在建表里试出来的。**死锁。**
    /// 而日志里它长得像"抓握通道没有改变这两块之间的距离",我据此改了三版算法,全在治一个
    /// **根本没发生的动作**。两幅腕相机帧并排一看:掩码全黑,爪子纹丝不动。
    ///
    /// ⇒ 这条命令**不许依赖任何外部状态**。两种形式各发一次:
    ///   末端那一形的目标就是它此刻的位姿(对认末端的机体是纯粹的开合;对不认的机体是空操作),
    ///   关节那一形的目标就是它此刻的关节角(同理)。**谁认哪种,由机体自己决定。**
    let 抓握 = |plug: &mut Plug<S>, 开度: f64, _用关节: bool| {
        let Some(f) = plug.sense() else { return };
        let q = f.joints.get(臂号.get()).cloned().unwrap_or_default();
        if !q.is_empty() { plug.act(&Cmd::Joints { arm: 臂号.get(), q, jaw: 开度 }); }
        let Some(f2) = plug.sense() else { return };
        if let Some(e) = f2.ee.get(手号.get()).copied() {
            plug.act(&Cmd::Ee { arm: 手号.get(), at: [e[0], e[1], e[2]], quat: [e[3], e[4], e[5], e[6]], jaw: 开度 });
        }
    };
    // 🔴 **不许有「最小一步不得小于 X 米」这种下限**(owner 2026-08-28:影响自主性的写死的东西全删)。
    // 它说的是「不管你是什么身体,一步不许小于 5 mm」—— 换一具小机器人就是错的,
    // 而对这具身体它**根本不会触发**(可达带 0.367 m ⇒ 十分之一 = 3.7 cm ≫ 5 mm),
    // 纯粹是「留着以防万一」的身体假设。删掉不会变成零位移:`可达带` 在进这个函数时
    // 就已经是量出来的(量不到会当场具名要 `reach` 并返回),身体小 ⇒ 步子小,那是对的。
    // 10 是**比例**(可达带的十分之一),不是米。
    let 探步 = 可达带[1] / 10.0;
    // 🔴🔴 **量表的那三下要迈大步 —— 小步的信号被认块器的形心抖动淹掉。**
    //
    // 实测代价(D6):三下里 x 那一下量出"挪 1 m 画面只跑 0.034",而 y 那一下跑 0.931 ——
    // **同样挪一米,一个有反应一个几乎没反应,这不可能**。对账:x 那一列把"挪了一米"
    // 只解释掉 **42%**,剩下 58% 不知去向。原因是探针只挪 3.7 cm,而形心本身就抖
    // (同一只爪子占画幅在 0.081 和 0.045 之间跳)⇒ **信号和噪声一样大**。
    // 拿这样一列去解算,解出来的方向就是错的(D6 实测:它要求手往 −x 走 0.90 m,越走越远)。
    // ⇒ 量表用**可达带的三分之一**(约 12 cm),信噪比翻三倍;走路仍用十分之一那一档。
    let 探幅 = 可达带[1] / 3.0;

    // 🔴🔴🔴 **腕相机收尾:提成具名闭包,因为它现在有【两个】入口。**(2026-08-29)
    // 原来它内联在"追完之后",而追要先过 `看爪` 那道闸 —— 头相机上那道闸赢不了
    // (YF 实测 344 连败,这一段 0 次被执行)。提出来之后,`看爪` 失败的分支也能直接进。
    let 收尾腕 = |plug: &mut Plug<S>, 腕机: usize, jaw0: f64, 通道是关节: bool,
                  look: &body_layer::eye::Look, 位: &mut [f64; 3]| -> Option<()> {
                // 两个接触面在这台相机里的位置 —— 手不动,所以量一次就够。
                let f0 = plug.sense()?;
                let e0 = f0.ee.get(手号.get()).copied()?;
                let (位0, 姿0) = ([e0[0], e0[1], e0[2]], [e0[3], e0[4], e0[5], e0[6]]);
                // 🔴🔴 **这一步不需要认块器 —— 手上那台相机里手是不动的、又大,张合两帧相减就够。**
                // 实测代价(W2):我用"晃爪子认块"去找腕相机里的手,而认块器在那台里认不出来
                //(日志早就有 `各台 [true, false]`),我又用 `?` 静默返回 ⇒
                // **整段收尾一行日志都没有**,看起来像没写过。**静默失败是本仓最贵的一类。**
                let _ = (位0, 姿0);
                // 🔴🔴🔴 **张合相减必须分得清「手指在动」和「胳膊在漂」。**(AP 实测 2026-08-31)
                //
                // AP:`张合 105595 个像素变了` —— **34% 的画面**。只有钳口在动不可能这么多,
                // 那是这两帧之间**整条胳膊还在漂**(上一段刚走完,身体没停稳)。
                // 于是"变化区的中位点"落在一大坨漂移上,`两指中间` 整个是假的,
                // 而后面**每一条**判据都以它为基准(爪子多深 / 砍掉比爪子近的 / 追到哪儿)⇒ 一起错。
                //
                // 修法不加闸,是换一种拿读数的办法:**开 → 合 → 再开**,三帧。
                //   手指是**来回**的:亮→暗→亮 ⇒ 两次差**同号**,且"回得来"(|开−再开| < 每次的摆幅)
                //   胳膊漂是**单向**的:A→B→C ⇒ 两次差**异号**,自己就被筛掉
                // 不引入任何身体词,也不需要知道手指长什么样。
                抓握(plug, 1.0, 通道是关节);
                定爪(plug, 等拍 * 8);
                let (w2, h2, 开2) = plug.sense().and_then(|f| 灰(&f, 腕机))?;
                let ee开 = plug.sense().and_then(|f| f.ee.get(手号.get()).copied());
                抓握(plug, 0.0, 通道是关节);
                定爪(plug, 等拍 * 8);
                let (_, _, 合2) = plug.sense().and_then(|f| 灰(&f, 腕机))?;
                抓握(plug, 1.0, 通道是关节);
                定爪(plug, 等拍 * 8);
                let (_, _, 开b) = plug.sense().and_then(|f| 灰(&f, 腕机))?;
                let ee开b = plug.sense().and_then(|f| f.ee.get(手号.get()).copied());
                if let (Some(a), Some(b)) = (ee开, ee开b) {
                    let 漂 = ((a[0]-b[0]).powi(2)+(a[1]-b[1]).powi(2)+(a[2]-b[2]).powi(2)).sqrt();
                    println!("[服]   [收尾] 这三帧之间手掌自己漂了 {漂:.4} m —— 越大,张合相减越脏(照实报,不拦)");
                }
                let mut 强2: Vec<(u8, usize, usize)> = Vec::new();
                let mut 单向 = 0usize;
                for y in 0..h2 { for x in 0..w2 {
                    let i = y * w2 + x;
                    let d1 = 开2[i] as i32 - 合2[i] as i32;
                    let d2 = 开b[i] as i32 - 合2[i] as i32;
                    if d1.abs() <= 3 || d2.abs() <= 3 { continue }
                    if d1.signum() != d2.signum() { 单向 += 1; continue }
                    let 回 = (开2[i] as i32 - 开b[i] as i32).abs();
                    if 回 >= d1.abs().min(d2.abs()) { 单向 += 1; continue }
                    强2.push((d1.abs().min(d2.abs()).min(255) as u8, x, y));
                }}
                println!("[服]   [收尾] 张合三帧:{} 个像素**来回动**(占画幅 {:.4}),另有 {单向} 个只单向变(判为胳膊漂,已剔)",
                    强2.len(), 强2.len() as f64 / (w2 * h2) as f64);
                if 强2.len() < 32 {
                    println!("[服]   [收尾] 手上那台里张合只有 {} 个像素来回动 ⇒ 认不出接触面,收尾降级(退回主眼那条路,不停手)", 强2.len());
                    return None;
                }
                强2.sort_unstable_by(|a, b| b.0.cmp(&a.0));
                let 取2 = (强2.len() / 4).max(32).min(强2.len());
                let mut xs2: Vec<usize> = 强2[..取2].iter().map(|k| k.1).collect();
                let mut ys2: Vec<usize> = 强2[..取2].iter().map(|k| k.2).collect();
                xs2.sort_unstable(); ys2.sort_unstable();
                // **两指中间** = 变化区的中位点 —— 张合时两指对称地离开/靠拢,中位点就是它们中间。
                let c = (xs2[取2 / 2] as f64 / w2 as f64, ys2[取2 / 2] as f64 / h2 as f64);
                let c = point_gen::Px::from([c.0, c.1]);
                let (_, _, g2) = plug.sense().and_then(|f| 灰(&f, 腕机))?;
                println!("[服]   [收尾] 手上那台里:张合 {} 个像素变了 ⇒ **两指中间**在 ({:.3},{:.3})", 强2.len(), c[0], c[1]);
                // 🔴🔴🔴 **物体用【几何切块】从这台相机自己的深度图里长出来,不再用"最近的会动的像素"。**
                //(owner 2026-08-30 令"彻底修好";ZG/ZL 三炮的共同死因)
                //
                // 旧办法:"手一动、画面里跟着动的就是世界,取离两指中间最近的一撮"。
                // 它的前提是"两指附近除了目标没别的东西",而那只在**最后一两厘米**成立。
                // 手还在半空时,离两指最近的会动的像素永远是**手底下的地板 / 面前的天花板**:
                //   YG:占画幅 0.353(三分之一个画面)· ZG:天花板一根黑梁,横向位移恒为 0
                //   · ZL:占画幅 0.316 ⇒ 三列的两遍分歧 81% / 93%,深度跳 0.4 m
                // 我为此加过"比爪子大就是地板"那道闸,而它挡不住**小块的**地板/板缝。
                //
                // 换成 `分块`(减法③,已在头相机上验过"立方体/球/钞票/小车四件全中"):
                // 空间里一张平面满足 1/z 关于像素线性 ⇒ 拟出支撑面,**比它鼓出来的连通块**才是物体。
                // **地板和天花板是平面,鼓不出来,压根不会成为候选** —— 这一条把上面三炮的死因
                // 整个消掉,而且不认识任何物体名、不用检测器、不用相机内参。
                // 贴画面边的块丢掉:同一帧上的伪影(整条背景 / 细缝 / **机器人自己的本体**)
                // 唯一的共同点就是贴边,而一件我能拿起来的东西是**完整地在画面里**的。
                let (dw2, dh2, dep2) = {
                    let 深路 = plug.lay.depth.get(腕机).or_else(|| plug.lay.depth.first())?.clone();
                    let dv = plug.last.as_ref().and_then(|o| 取(o, &深路))?;
                    wire::as_f32_grid(&dv)?
                };
                let dep2f: Vec<f64> = dep2.iter().map(|x| *x as f64).collect();
                // 3 是**无量纲倍数**,和头相机那一处同一个来源(真实深度图上验出来的)。
                let mut 区2 = point_gen::分块(&dep2f, dw2, dh2, selfcal::最少像素(dw2, dh2) as usize, 3.0);
                区2.retain(|r| r.框[0] > 0 && r.框[1] > 0 && r.框[2] + 1 < dw2 && r.框[3] + 1 < dh2);
                if 区2.is_empty() {
                    println!("[服]   [收尾] 这台相机的深度图里**一块鼓出来的东西都没有**(只有平面)⇒ 目标不在视野里,收尾降级");
                    return None;
                }
                // 🔴🔴🔴 **要抓的东西必须【在爪子前方、且够得着】—— 两头各砍一刀。**
                //(ZM 实测 2026-08-30)
                //
                // 几何切块挡住了地板和天花板(平面鼓不出来),但没挡住另外两类,ZM 全撞上了:
                //   `深 0.063 m · 2255 px · 占画幅 0.327` ⇒ 6 厘米,那是**机器人自己的躯干**
                //   `深 2.121 m · 129 px` / `深 2.017 m` ⇒ 两米开外的**墙上凸起**
                // 而球在头相机里稳定读到 `深 0.645 m · 鼓出 0.061 m` —— 它当时**根本不在腕相机视野里**,
                // 于是它只能在"自己身上"和"两米外"之间挑一个。
                //
                // 两条判据都用**已经量到的**数,不新增常数:
                //   · 比【爪子自己那一点】还近的 ⇒ 那是我,不是世界(这台相机长在手上,爪子的深度现读)
                //   · 比【量出来的可达上限】还远的 ⇒ 够不着,这一拍轮不到它
                let 全区 = 区2.clone();
                let 爪深 = 近侧深2(plug, 腕机, c[0], c[1], 0.02);
                if let Some(zc) = 爪深 {
                    let 前 = 区2.len();
                    区2.retain(|r| r.深 > zc && r.深 <= zc + 可达带[1]);
                    if 区2.is_empty() {
                        // 🔴🔴🔴 **降级分两级,而且【永远不把"我自己"捡回来】。**(AP 实测 2026-08-31)
                        //
                        // 上一版在这里直接 `区2 = 全区`,把**比爪子还近的块也放了回来**。
                        // 代价照记:AP 切出的两块是 `深 0.077 m` / `深 0.063 m` —— 6~8 厘米,
                        // 那是**我自己的两根手指**。驱动于是对着自己的手指伺服了 24 轮,
                        // 读数在 (0.790,·) 和 (0.209,·) 两个**镜像**位置来回跳(左指/右指,x 加起来正好 ≈1),
                        // 横纵差钉死在 0.3242 / 0.2571。**朝自己的手指推,推一万步也碰不到球。**
                        // ⇒ "碰到就是测量"这条**只对世界成立,对自己不成立** —— 碰自己学不到任何东西。
                        //
                        // 一级:只松掉「够得着」那一刀 —— 够不着的东西照样朝它推,碰到算一次测量。
                        // 二级:连一块在爪子前方的都没有 ⇒ 目标**不在这台相机视野里**
                        //       (AP 渲图为证:腕相机整个后半程对着空桌沿,球根本不在画面上)。
                        //       这时**不是停手**,是 `return None` ⇒ 调用方立刻退回【主眼画面里闭环】,
                        //       而主眼这一炮找球 **4/4 全对**。换只眼睛接着干,比对着自己的手指空转强。
                        let 前方: Vec<_> = 全区.iter().filter(|r| r.深 > zc).cloned().collect();
                        println!("[服]   [收尾] ⚠️ {前} 块里没有一块又在爪子前方又够得着(爪子在 {zc:.3} m,够到 {:.3} m)", zc + 可达带[1]);
                        if !前方.is_empty() {
                            println!("[服]      ⇒ 松掉「够得着」那一刀,还剩 {} 块在爪子前方:**照样朝最近那块推**,碰到就当一次测量", 前方.len());
                            区2 = 前方;
                        } else {
                            println!("[服]      ⇒ 连一块在爪子前方的都没有 ⇒ 切出来的**全是我自己**(比爪子还近)");
                            println!("[服]      ⇒ 目标不在这台相机视野里 ⇒ **换主眼接着干**(退回画面里闭环,不是停手)");
                            return None;
                        }
                    }
                    if 前 != 区2.len() {
                        println!("[服]   [收尾] {前} 块里砍掉 {} 块(比爪子 {zc:.3} m 还近的是我自己,比 {:.3} m 还远的够不着)", 前 - 区2.len(), zc + 可达带[1]);
                    }
                } else {
                    println!("[服]   [收尾] ⚠️ 读不到爪子自己那一点的深度 ⇒ 这一刀砍不了,候选照单全收(结果照实报)");
                }
                // 要抓的那一块 = 离两指中间最近的那一块。语义在头相机那一段已经定完了。
                区2.sort_by(|a, b| {
                    let da = (a.心[0] - c[0]).hypot(a.心[1] - c[1]);
                    let db = (b.心[0] - c[0]).hypot(b.心[1] - c[1]);
                    da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
                });
                let 块2 = 区2[0].clone();
                let 看2 = body_layer::eye::Look {
                    u: 块2.心[0],
                    v: 块2.心[1],
                    // 0.01 是占画幅的分数下限(无量纲),不是米。
                    span_frac: (((块2.框[2] - 块2.框[0]) as f64 / dw2 as f64)
                        .max((块2.框[3] - 块2.框[1]) as f64 / dh2 as f64)).max(0.01),
                    verb: look.verb.clone(),
                    force: look.force.clone(),
                    box01: [
                        块2.框[0] as f64 / dw2 as f64, 块2.框[1] as f64 / dh2 as f64,
                        块2.框[2] as f64 / dw2 as f64, 块2.框[3] as f64 / dh2 as f64,
                    ],
                };
                println!("[服]   [收尾] 几何切出 {} 块鼓起来的东西 ⇒ 离两指中间最近的那块在 ({:.3},{:.3}) · 占画幅 {:.3} · 深 {:.3} m · {} px",
                    区2.len(), 看2.u, 看2.v, 看2.span_frac, 块2.深, 块2.像素数);
                // `span_frac` 是占画幅的分数 ⇒ 这两个夹值也是**画幅的分数**,无量纲。
                let 窗2 = (看2.span_frac * 0.5).clamp(1.0 / w2 as f64, 0.25);
                let (_, _, g2) = plug.sense().and_then(|f| 灰(&f, 腕机))?;
                // 这台相机的表:命令一个通道,**物体**在它画面里跑多少(手不动,所以跑的就是相对量)
                let 模2 = 截块(w2, h2, &g2, 看2.u, 看2.v, ((看2.span_frac * w2 as f64 * 0.5) as usize).clamp(3, 40))?;
                // 🔴🔴🔴 **跟不住的块要当场拒绝 —— 否则它会交出一份【干净而错误】的方向盘。**
                //(ZG 实测 2026-08-29)
                //
                // ZG:收尾那一刻腕相机对着**天花板**(渲图为证),它取到的"物体"是一根黑梁的直边。
                // 量出来的两列**又干净又可复现**(去 +0.0424 / +0.0423,两遍分歧 1%),而**横向位移
                // 精确等于 0**(小数点后四位全零)。那不是巧合,是**孔径问题的指纹**:沿一条直边横着滑,
                // 匹配代价处处相同 ⇒ 横向永远选到同一个位置。
                // ⚠️ 这正是仓规那条"**太干净的结果先当烟雾弹**" —— 数越漂亮越该查它量的是什么。
                //
                // 判据是这块自己的**结构张量**:两个特征值就是"沿两个主方向各有多少纹理"。
                // 一条直边 ⇒ 小的那个≈0 ⇒ 横竖只定得住一个方向 ⇒ 跟不住。
                // 门槛不引入新常数:用仓里各处已在用的**灰度噪声地板 8**,
                // 弱方向的结构必须压过"纯噪声也会有的那么多"(每像素 2σ²)。
                {
                    let n2 = ((看2.span_frac * w2 as f64 * 0.5) as usize).clamp(3, 40) * 2 + 1;
                    let (mut a, mut b, mut c) = (0.0f64, 0.0f64, 0.0f64);
                    for y in 1..n2.saturating_sub(1) { for x in 1..n2.saturating_sub(1) {
                        let gx = 模2[y * n2 + x + 1] as f64 - 模2[y * n2 + x - 1] as f64;
                        let gy = 模2[(y + 1) * n2 + x] as f64 - 模2[(y - 1) * n2 + x] as f64;
                        a += gx * gx; b += gx * gy; c += gy * gy;
                    }}
                    let 迹 = a + c;
                    let 判 = ((a - c) * (a - c) + 4.0 * b * b).max(0.0).sqrt();
                    let 弱 = (迹 - 判) * 0.5;
                    let 强 = (迹 + 判) * 0.5;
                    // 8 是灰度噪声地板(和认接触面那一处同一个来源);2σ² 是纯噪声在一个方向上的期望结构。
                    let 噪 = 2.0 * 8.0 * 8.0 * ((n2.saturating_sub(2)) * (n2.saturating_sub(2))) as f64;
                    if !(弱 > 噪) {
                        // 🔴 **改成降级,不再拒绝。**(owner 2026-08-31:闸只许降级不许停手)
                        // 弱方向没纹理(直边)⇒ 那一维读数不可信(ZG 实测横向恒为 0);
                        // **但强方向那一维是可信的,照样该走。** 拒绝一整段的代价是什么都不做。
                        println!("[服]   [收尾] ⚠️ 这一块**只跟得住一个方向**:强 {强:.0} / 弱 {弱:.0}(纯噪声 {噪:.0})");
                        println!("[服]      ⇒ 弱方向多半是一条直边,那一维不可信;**照样走**,靠强方向推进,结果照实报");
                    }
                    println!("[服]   [收尾] 这一块跟得住:主方向结构 强 {强:.0} / 弱 {弱:.0}(噪声地板 {噪:.0})");
                }
                let 半2 = ((看2.span_frac * w2 as f64 * 0.5) as usize).clamp(3, 40);
                let mut 表2: Vec<[f64; 3]> = Vec::new();
                // 🔴 这一段原来也是"每列之前先回到记住的那个位姿" —— 同一个复位键,一并删掉。
                // 改成**来回**:+δ 再 −δ(相对命令),两遍各除以自己的实到 ⇒ 应当相等,
                // `分歧 < 共识` 才收;净位移为零,不需要记住任何位姿。
                // 🔴🔴🔴 **腕相机里追物体也不许全画面搜。**(WX 实证,2026-08-28)
                //
                // 上一版这里是 `找块`(全画面)。头相机上同一个写法让它**锁到了肘部**;
                // 腕相机视野更近、纹理更少,更容易锁到别的一块 —— 于是深度一跳,
                // 那道"物体深度跑得比我自己还远"的闸就响,整列空掉。
                // WX 实测:通道 1、2 **都**因此空掉 ⇒ `量不齐三列 ⇒ 收尾降级,直接合` ⇒ 合了个空。
                // ⇒ 改成**只在上次位置附近搜**:物体是静的、相机一步只挪半个探幅,不可能瞬移。
                //   搜索半径由模板自己的大小给(量出来的),不是我挑的长度。
                let mut 上物 = (看2.u, 看2.v);
                for k in 0..3usize {
                    // 🔴🔴 **这三步原来是静默 `?` —— 三列全空而【一个字都不打印】。**(ZC 实测 2026-08-29)
                    // 日志只剩一句"量不齐三列",查不出是跟丢了、还是读不到深度、还是压根没帧。
                    // **静默失败是本仓最贵的一类**(这一句注释在收尾段开头就写着,而我又犯了一次)。
                    let mut 读 = |plug: &mut Plug<S>, 上: (f64, f64)| -> Option<(f64, f64, f64)> {
                        let Some((_, _, gg)) = plug.sense().and_then(|f| 灰(&f, 腕机)) else {
                            println!("[服]   [收尾]   读:这一帧没有第 {腕机} 台的图"); return None };
                        let Some((a, b)) = 找块窗(w2, h2, &gg, &模2, 半2, 上.0, 上.1, 半2 * 3) else {
                            println!("[服]   [收尾]   读:在 ({:.3},{:.3}) 附近 {} px 窗里跟丢了模板", 上.0, 上.1, 半2 * 3); return None };
                        let Some(dd) = 近侧深2(plug, 腕机, a, b, 窗2) else {
                            println!("[服]   [收尾]   读:跟到 ({a:.3},{b:.3}) 但那一点读不出深度"); return None };
                        Some((a, b, dd))
                    };
                    let 迈 = |plug: &mut Plug<S>, d: f64| -> Option<f64> {
                        let e0 = plug.sense()?.ee.get(手号.get()).copied()?;
                        let mut p = [e0[0], e0[1], e0[2]]; p[k] += d;
                        let (f1, _, _) = 落(plug, p, [e0[3], e0[4], e0[5], e0[6]], jaw0, 等拍)?;
                        let e1 = f1.ee.get(手号.get()).copied()?;
                        Some(e1[k] - e0[k])
                    };
                    let Some(甲) = 读(plug, 上物) else { println!("[服]   [收尾] 通道 {k}:第一次读就没读到 ⇒ 这一列空着"); 表2.push([0.0; 3]); continue };
                    上物 = (甲.0, 甲.1);
                    let Some(去实) = 迈(plug, 探幅 / 2.0) else { println!("[服]   [收尾] 通道 {k}:去那一步的命令发不出去 ⇒ 这一列空着"); 表2.push([0.0; 3]); continue };
                    let Some(乙) = 读(plug, 上物) else { println!("[服]   [收尾] 通道 {k}:去完之后读不到 ⇒ 这一列空着"); 表2.push([0.0; 3]); continue };
                    上物 = (乙.0, 乙.1);
                    let Some(回实) = 迈(plug, -探幅 / 2.0) else { println!("[服]   [收尾] 通道 {k}:回那一步的命令发不出去 ⇒ 这一列空着"); 表2.push([0.0; 3]); continue };
                    let Some(丙) = 读(plug, 上物) else { println!("[服]   [收尾] 通道 {k}:回完之后读不到 ⇒ 这一列空着"); 表2.push([0.0; 3]); continue };
                    上物 = (丙.0, 丙.1);
                    if 去实.abs() < 1e-4 || 回实.abs() < 1e-4 { println!("[服]   [收尾] 通道 {k}:命令发了但身体没动(去 {去实:+.5} 回 {回实:+.5})⇒ 这一列空着"); 表2.push([0.0; 3]); continue }
                    let 去 = [(乙.0-甲.0)/去实, (乙.1-甲.1)/去实, (乙.2-甲.2)/去实];
                    let 回 = [(丙.0-乙.0)/回实, (丙.1-乙.1)/回实, (丙.2-乙.2)/回实];
                    let 分 = (0..3).map(|i| (去[i]-回[i]).powi(2)).sum::<f64>().sqrt();
                    let 共 = (0..3).map(|i| (去[i]+回[i]).powi(2)).sum::<f64>().sqrt();
                    if !(分 < 共) {
                        println!("[服]   [收尾] 通道 {k}:去和回对不上(分歧 {分:.3} ≥ 共识 {共:.3})⇒ 跟丢了,这一列空着");
                        表2.push([0.0; 3]); continue
                    }
                    // 🔴 物体的深度变化不许超过我自己实际动的距离(容差 = 来回之后没回到原深那点残差)。
                    if (乙.2 - 甲.2).abs() > 去实.abs() + (丙.2 - 甲.2).abs() {
                        println!("[服]   [收尾] 通道 {k}:物体深度跑得比我自己还远 ⇒ 锁错图案了,这一列空着");
                        表2.push([0.0; 3]); continue
                    }
                    表2.push([(去[0]+回[0])*0.5, (去[1]+回[1])*0.5, (去[2]+回[2])*0.5]);
                    println!("[服]   [收尾] 通道 {k}:去 {去实:+.4} 回 {回实:+.4} ⇒ 物体在手上那台里跑 ({:+.4},{:+.4}) 深 {:+.4} · 两遍分歧 {:.0}%",
                        乙.0-甲.0, 乙.1-甲.1, 乙.2-甲.2, 100.0 * 分 / 共.max(1e-12));
                }
                // 🔴 **有几列用几列,不因为"不齐"就停手。**(owner 2026-08-31:闸只许降级不许拒绝)
                // 缺的那一列在解里权重是零,`解3` 缺秩时走阻尼最小二乘,**照样给一个能动的方向**。
                let 有几列 = 表2.iter().filter(|c| c.iter().any(|x| x.abs() > 0.0)).count();
                if 有几列 == 0 {
                    println!("[服]   [收尾] 三列一列都没量到(这台相机里没有一个通道让物体动过)⇒ 收尾降级,直接合"); return None;
                }
                if 有几列 < 3 {
                    println!("[服]   [收尾] ⚠️ 只量到 {有几列}/3 列 ⇒ **照样走**,缺的那些方向管不着,照实说不精确");
                }
                let mut m2 = [[0.0f64; 3]; 3];
                for c in 0..3 { for r in 0..3 { m2[r][c] = 表2[c][r]; } }
                // 追:把物体挪到两指中间,**并且压到手指自己所在的那个深度**。
                //
                // 🔴🔴 这里原来写的是 `误 = [c0-nu, c1-nv, **0.0**]` —— 深度那一项恒为零 ⇒
                // 它只会把物体在画面里左右上下对准两指中间,**永远不往下压**。
                // 手指对得再准,离物体还差一截,合下去仍然是空的。这是"合到底读数停在 0"的一条直接原因。
                // 目标深度不需要任何身体词:**这台相机长在手上,手指在它画面里不动,
                // 所以"手指那一点有多深"直接读一次就是目标** —— 把物体压到那个深度,它就在指间。
                let 指深 = 近侧深2(plug, 腕机, c[0], c[1], 窗2);
                // 深度读数自己抖多少:同一点再读一次,差多少就是多少。门槛由它给,不是我填。
                let 深抖 = match (指深, 近侧深2(plug, 腕机, c[0], c[1], 窗2)) {
                    (Some(a), Some(b)) => (a - b).abs(), _ => 0.0 };
                match 指深 {
                    Some(z) => println!("[服]   [收尾] 手指那一点自己有多深:{z:.3} m(抖 {深抖:.4})⇒ 把物体压到这个深度"),
                    None => println!("[服]   [收尾] 读不到手指那一点的深度 ⇒ 深度那一项只能留空,靠碰到东西停"),
                }
                // 🔴 同上:只在上次位置附近搜,不全画面搜(全画面 = 锁到别的一块)。
                // 🔴🔴 **轮数也不许写死 12。** 它和"追"那边的 20 是同一个病:
                //    把一个还在收敛的过程按计数掐掉。上限由量出来的东西给,只防死循环;
                //    真正停手看**碰上了**或**误差不再缩**。
                let mut 上物2 = (看2.u, 看2.v);
                let mut 上深2 = 块2.深;
                let mut 重复 = 0u32;
                let mut 上读 = (f64::NAN, f64::NAN, f64::NAN);
                // 12 / 4000 是**轮数**上下限(计数,无量纲);2 是余量倍数,3 是搜索窗的模板倍数,都无量纲。
                let 收尾轮 = ((可达带[1] / (探步 * 交付率).max(1e-9)) * 2.0).ceil().clamp(12.0, 4000.0) as u32;
                // 🔴🔴🔴 **每一帧【重新切】,不再靠模板一路跟。**(2026-08-31)
                //
                // 今天所有"跟丢"都出在模板追踪上:跟到肘、跟到肩、跟到天花板梁(横向恒为 0)、
                // 锁到背景墙(AN 实测:物体深度从 0.688 m 跳到 2.13 m,之后三拍一字不差地钉在墙上)。
                // 而**几何重认从没错过**(头相机上找球 13/13)。
                // 追踪会漂、会锁错、有孔径问题;**重认没有状态,错一帧下一帧就自己回来**。
                // 代价只是"这一块和上一帧是不是同一块" —— 而两指附近只有一块凸起时,那是白送的。
                // ⚠️ 这不是加一道闸,是**换一种拿读数的办法**:它永远给得出一个位置。
                let 重切 = |plug: &mut Plug<S>, 上: (f64, f64), 上深: f64| -> Option<(f64, f64, f64)> {
                    let 深路 = plug.lay.depth.get(腕机).or_else(|| plug.lay.depth.first())?.clone();
                    let dv = plug.last.as_ref().and_then(|o| 取(o, &深路))?;
                    let (dw, dh, dep) = wire::as_f32_grid(&dv)?;
                    let depf: Vec<f64> = dep.iter().map(|x| *x as f64).collect();
                    let mut v = point_gen::分块(&depf, dw, dh, selfcal::最少像素(dw, dh) as usize, 3.0);
                    v.retain(|r| r.框[0] > 0 && r.框[1] > 0 && r.框[2] + 1 < dw && r.框[3] + 1 < dh);
                    // 🔴🔴 **进门时砍过的那两刀,追的每一帧都要照砍。**(AP 实测 2026-08-31)
                    // 上一版的重切**一刀不砍**,只挑"离上一帧最近的一块" ⇒ 它一路走到
                    // `深 1.049 m` 的背景凸起上,然后**七拍一字不差**地钉死在那儿。
                    // 进门砍、追的时候不砍 = 没砍。
                    if let Some(zc) = 爪深 { v.retain(|r| r.深 > zc && r.深 <= zc + 可达带[1]); }
                    // 物体不会瞬移:上一帧到这一帧**手最多走一个探步**,
                    // 所以到一个静止物体的深度也最多变那么多(再加上深度读数自己的抖)。
                    // 这个界是量出来的(探步是我自己下的命令,深抖是同点两读之差),不是填的。
                    if 上深.is_finite() { v.retain(|r| (r.深 - 上深).abs() <= 探步 + 深抖); }
                    if v.is_empty() { return None }
                    // 同一块 = 离上一帧那个位置最近的一块。物体是静的、一步只挪半个探幅,不可能瞬移。
                    v.sort_by(|a, b| {
                        let da = (a.心[0] - 上.0).hypot(a.心[1] - 上.1);
                        let db = (b.心[0] - 上.0).hypot(b.心[1] - 上.1);
                        da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
                    });
                    Some((v[0].心[0], v[0].心[1], v[0].深))
                };
                for 轮 in 0..收尾轮 {
                    if plug.复位过 { break }
                    let _ = plug.sense()?;
                    let (nu, nv, nd) = match 重切(plug, 上物2, 上深2) {
                        Some(t) => t,
                        None => {
                            // 重切不出来(这一帧只有平面)⇒ 退回模板跟一帧,**仍然不停手**。
                            let (_, _, gg) = plug.sense().and_then(|f| 灰(&f, 腕机))?;
                            let (a, b) = 找块窗(w2, h2, &gg, &模2, 半2, 上物2.0, 上物2.1, 半2 * 3)?;
                            let d = 近侧深2(plug, 腕机, a, b, 窗2)?;
                            (a, b, d)
                        }
                    };
                    上物2 = (nu, nv);
                    上深2 = nd;
                    // 🔴 **读数一字不差地重复 ⇒ 这台相机不再给新信息。**(AP 实测:七拍全等,钉在 1.049 m)
                    // 这不是停手:退回主眼那条路接着干,而主眼这一炮找球 4/4 全对。
                    if (nu, nv, nd) == 上读 { 重复 += 1 } else { 重复 = 0 }
                    上读 = (nu, nv, nd);
                    if 重复 >= 5 {
                        println!("[服]   [收尾] 🔴 连着 {重复} 拍读数一字不差(({nu:.3},{nv:.3}) 深 {nd:.3})⇒ 这台相机不再给新信息");
                        println!("[服]      ⇒ **换主眼接着干**(退回画面里闭环,不是停手)");
                        return None;
                    }
                    let 深误 = 指深.map(|z| z - nd).unwrap_or(0.0);
                    let 误 = [c[0] - nu, c[1] - nv, 深误];
                    let 差 = (误[0].powi(2) + 误[1].powi(2)).sqrt();
                    if 轮 % 3 == 0 { println!("[服]   [收尾] 追 {轮}:物体在 ({nu:.3},{nv:.3}) 深 {nd:.3} · 两指中间在 ({:.3},{:.3}) ⇒ 横纵差 {差:.4} 深度差 {深误:+.4}", c[0], c[1]); }
                    // 🔴 深度那一项的容差:`深抖`(读数自己的抖)在确定性的仿真深度图上可以**恰好是 0**,
                    // 于是 `深误.abs() <= 0` 永远不成立 ⇒ **这条成功判据一次都不会响**,只能靠碰上。
                    // 容差取 `max(深抖, 探步)`:**探步是我一步能走的距离,深度也不可能伺服得比它更细**。
                    // 两个都是量出来的,不是填的。
                    if 差 <= 看2.span_frac * 0.5 && 深误.abs() <= 深抖.max(探步) { println!("[服]   [收尾] 🟢 物体已经在两指中间,深度也压到手指那一层了(容差 {:.4} m)", 深抖.max(探步)); break }
                    let dp = 解3(m2, 误)?;
                    let 长 = (dp[0].powi(2)+dp[1].powi(2)+dp[2].powi(2)).sqrt();
                    if !(长 > 1e-9) { break }
                    let 比 = (探步 / 长).min(1.0);
                    let e = plug.sense()?.ee.get(手号.get()).copied()?;
                    let (f2, 挡, _) = 落(plug, [e[0]+dp[0]*比, e[1]+dp[1]*比, e[2]+dp[2]*比],
                        [e[3], e[4], e[5], e[6]], jaw0, 等拍)?;
                    if let Some(ee) = f2.ee.get(手号.get()).copied() { *位 = [ee[0], ee[1], ee[2]]; }
                    if 挡 { println!("[服]   [收尾] 碰到东西了 ⇒ 就地合"); break }
                }
                Some(())
    };

    /// 合到读数不再变为止,返回停在几。**唯一一处**回答"合爪合到哪儿了"。
    let 合到停住 = |plug: &mut Plug<S>, 通道是关节: bool| -> Option<f64> {
        // 🔴🔴🔴 **盯的必须是【这一炮在用的那只手】的钳口读数,不是第 0 只。**(AY 实测 2026-08-31)
        // AY 选中了 1 号末端,而这里写的是 `jaw.first()` ⇒ 盯着一只**根本没在合**的爪子,
        // "动过"永远为假 ⇒ 600 拍空转、一行日志不打(整整 2700 行只有 `[看]`/`[计时]`)。
        // 同一类:写死的下标在换手之后静默失效。
        let 我爪 = |f: &Frame| f.jaw.get(手号.get()).or_else(|| f.jaw.get(手号.get()).or_else(|| f.jaw.first())).copied();
        let 开着读数 = plug.sense().and_then(|f| 我爪(&f));
        let mut 上次: Option<Vec<f64>> = None;
        let mut 稳 = 0u32;
        let mut 停在 = f64::NAN;
        for _ in 0..600 {
            抓握(plug, 0.0, 通道是关节);
            let Some(f) = plug.sense() else { return None };
            let 此: Vec<f64> = f.jaw.iter().copied().collect();
            let 此我 = 我爪(&f);
            // "动过" = 和**合之前**那个读数不一样了(不假设开着时是几)。
            let 动过 = match (开着读数, 此我) { (Some(a), Some(b)) => (a - b).abs() > 1e-9, _ => false };
            if 动过 && 上次.as_ref() == Some(&此) { 稳 += 1; if 稳 >= 3 { 停在 = 此我.unwrap_or(f64::NAN); break } }
            else { 稳 = 0 }
            上次 = Some(此);
        }
        Some(停在)
    };

    /// 这台相机此刻的深度图(米,行优先)。深度路径由 `discover` 靠**形状**认出来(减法①),
    /// 不写死任何字段名。拿不到就返回 None —— 那时自图只学横纵两行,而这件事会被日志说出来。
    let 深图 = |plug: &mut Plug<S>, ci: usize| -> Option<Vec<f64>> {
        let 路 = plug.lay.cams.get(ci)?.clone();
        let mut 深路 = 路;
        if let Some(dp) = plug.lay.depth.get(ci).or_else(|| plug.lay.depth.first()) { 深路 = dp.clone(); }
        plug.last.as_ref().and_then(|o| 取(o, &深路)).and_then(|dv| wire::as_f32_grid(&dv)).map(|(_, _, d)| d)
    };
    // 🔴🔴🔴 **自图(稠密光流那一套)整个摘掉。**(owner 2026-08-31 定,实测为据)
    //
    // AS 那一炮它被算了 **270 次、被采用 0 次**:
    //   `方向盘用的是**接触面那张(自图还没学到)**` × **134**
    //   `「我」用**看爪认出的两瓣** —— 不用自图挑点(它会挑到肩/墙上)` × 每一次
    // 而 AP / AQ / AR 三炮的日志里它连出现都没出现过。LAB 对它的最后状态是 `未落地`。
    // ⇒ 它既没帮上忙,也**不是**画面闭环跑不起来的原因(那是通道表建不出来,已另修)。
    // 留着它只有两个后果:每帧多算一遍,以及**多一处会静默给出错答案的地方**。
    // 被跟的那一点改回**模板跟踪**(摘掉它之前本来就是这么跟的)。
    // 🔴🔴 **看不见自己 ⇒ 把自己挪进画面,不许站着。**(WZ 渲图查出来的死锁,2026-08-28)
    //
    // 实测代价(WZ):这具身体开局时爪子压在头部相机的**左下角边缘、一半在画面外**
    //(唯一认出来那次落在 (0.161,0.937),由它算出的接触面一个在 **v=1.134 —— 画面外**)。
    // 于是:认不出爪 ⇒ 建不了表 ⇒ 追不起来 ⇒ 手一直不动 ⇒ 下一拍还是认不出。**死锁**。
    // 80 分钟一集里合爪那一步只走到过 **2** 次,其余全卡在这一句上。
    //
    // ⚠️ 这**不是**我以前删掉的那两条逃生动作(回原位 / 把手腕转八分之一圈)——
    // 那两条的方向和角度是我拍的。这一条里我一个方向都没有指定:
    //   **目标** = "让相机多看见我一点",而"多看见"是量出来的(命令一下,画面上跟着变的像素数);
    //   **怎么挪** = 逐个通道试一步,变多了就留着,变少了就退到另一边。
    //             没有哪个通道是"抬"、哪个是"转",它们对我一律只是编号。
    // 顺带的好处:跟着变的像素天然偏向**动得最快的那一头**,也就是手那一头。
    //
    // 🔴 **两处失败都要走它**:`看爪`(建表之前)和 `认接触面`(下手之前)。
    // XA 实测:只修了前一处,后一处每集照样报"认不出我的接触面"然后站着 —— 同一个病两个入口。
    let 挪进画面 = |plug: &mut Plug<S>, 轮: u32, jaw0: f64, 是关节: bool| -> Option<bool> {
        // 所有关节组之和 —— 双臂就是两条臂的关节都算通道(见 `迈通道稳` 那段注释)。
        let 关节数 = { let f = plug.sense()?; 真臂(&f).iter().map(|&i| f.joints[i].len()).sum::<usize>() };
        let 通道数 = if 是关节 { 关节数 } else { 6 };
        if 通道数 == 0 { return None }
        let 见 = |plug: &mut Plug<S>| -> Option<Vec<u8>> { plug.sense().and_then(|f| 灰(&f, 相机号)).map(|(_, _, g)| g) };
        let 变数 = |a: &[u8], b: &[u8]| -> f64 {
            if a.len() != b.len() || a.is_empty() { return 0.0 }
            a.iter().zip(b.iter()).filter(|(p, q)| p.abs_diff(**q) > 8).count() as f64 / a.len() as f64
        };
        // 轮流试通道 —— 每失败一次轮号加一,于是一拍试一个,试完一圈从头再来。
        let j = (轮 as usize).saturating_sub(1) % 通道数;
        let 前 = 见(plug)?;
        let mut 动 = vec![0.0; 通道数];
        动[j] = 探幅;
        let (_, _实1, _) = 迈通道(plug, &动, 1.0, jaw0, 是关节)?;
        等停(plug, 等拍 * 2);
        let 后1 = 见(plug)?;
        let 正 = 变数(&前, &后1);
        // 往回走两步 = 落在起点的另一边,和正向那一步等距、可比。
        动[j] = -探幅 * 2.0;
        let (_, _实2, _) = 迈通道(plug, &动, 1.0, jaw0, 是关节)?;
        等停(plug, 等拍 * 2);
        let 后2 = 见(plug)?;
        let 反 = 变数(&前, &后2);
        if 正 >= 反 {
            动[j] = 探幅 * 2.0;
            let (_, _实3, _) = 迈通道(plug, &动, 1.0, jaw0, 是关节)?;
            等停(plug, 等拍 * 2);
            let _ = 见(plug);
        }
        println!("[服]   挪进画面:试第 {j} 号通道 ⇒ 正着走画面变了 {:.3}、反着走变了 {:.3} ⇒ 停在**{}**那一边",
            正, 反, if 正 >= 反 { "正" } else { "反" });
        Some(正.max(反) > 0.0)
    };
    println!("[服] 干活:不解相机,只量画面雅可比(量表迈 {:.4} m = 可达带 {:.3} 的三分之一;走路一截 {:.4} m)",
        探幅, 可达带[1], 探步);

    let mut 集 = 0u32;
    // 🔴 这一集**已经试过而且失败了**的下手点 —— 接触集那一层要它才能换一个独立的候选。
    // 🔴🔴🔴 **驱动不许对客户说"不干"。**(owner 2026-08-27 死命令)
    //
    // 这一层里到处是"量不到 ⇒ 这一拍不下手 ⇒ `continue`"。每一条单看都合理(不编数),
    // 合起来的效果却是:**跑了一整晚,一次尝试都没做出来,一个视频都没有**。
    // owner 的原话:*"一个 driver 还敢拒绝客户?你完不成任务都得去干活。有失败视频也比没有视频好。"*
    //
    // ⇒ 兜底:**连着两拍没做出任何尝试,就用手上最粗的办法干一次** ——
    //   朝手腕正前方压过去、合、抬、看东西离没离开原位。它多半会失败,
    //   但它**会动、会留下一整段可以看的视频**,而失败的视频是能拿去查病的,空白不是。
    let mut 白转 = 0u32;
    // 上一拍的深度图 —— 用来挖掉"我自己"(见下面算支撑面那一段)。
    let mut 上深: Option<(usize, usize, Vec<f64>)> = None;
    let mut 试过: Vec<[f64; 3]> = Vec::new();
    let mut 静 = 0u32;   // 连着看不清时换方向用的计数
    // 🔴🔴 **相机解出来一台就用它,不许每一拍重解。**(XQ 实测,2026-08-28)
    //
    // 相机是**身体的属性**,不随这一拍看见什么而变;而三维目标全靠它反投影。
    // 实测代价:同一个棒球、同一个框(0.661–0.709 × 0.354–0.412),
    //   好的一拍解出 `焦距 342/325 · 相机在 (0.132,-0.503,1.400)` ⇒ 指尖去 (0.229,0.051,0.857)、下手宽 59.5 mm(对)
    //   坏的一拍 ⇒ 指尖去 (0.784,-0.052,0.607)、下手宽 **137.7 mm**、距离 **0.97 m**(被可达带挡下,整拍作废)
    // 同一个球被反投到一米开外,差别只在"这一拍的表解得好不好"。
    // ⇒ 解出一台就留着用;只有当它算出**不合理**的结果(超出可达带)才作废重解。
    // 这和画面雅可比那条规矩是同一条:*"先拿它预测,核对得上就不重量;对不上就当场重量"*。
    // ⚠️ 只在**这一炮之内**留着,不落盘:落盘的相机来自机体自报,和"从表解出来的那台"
    //    配的指尖偏置不同,混用会双重修正(那条陷阱就写在解相机那一段的头注里)。
    let mut 眼稳: Option<point_gen::Eye> = None;
    // 🔴 连着几次认不出接触面 —— 到三次就当成"这台相机看不见我的钳口张合"这条身体事实。
    let mut 认面失败 = if *认面载 { 3 } else { 0 };
    let mut 上眼: Option<(f64, f64)> = None;   // 上一次看到爪子在画面哪儿(跟踪用)
    // 🔴🔴🔴 **接触面只认一次,之后靠【跟踪】。**(WL 实测,2026-08-28)
    // `看爪` 认爪子靠"晃钳口 + 前后帧做差",认不出来时还会加大幅度反复重试 ——
    // 实测**每一拍烧掉约 430 步**,而官方一集只有 200 步 ⇒ **一集连一拍都走不完**,
    // 这样的闭环永远收敛不了(WL:伺服刚到第 2 拍,这一集就被步数耗尽了)。
    // ⇒ 建表那一段本来就截好了模板(整只手一块 + 两个接触面各一块);伺服每一拍
    //   **只抓一帧**,先全画面匹配"整只手",再拿它的位移把两个接触面的搜索窗推过去 ——
    //   **一帧一拍**,比原来便宜五十倍,而且不要求手臂静止(模板匹配不是帧间差分)。
    let mut 跟相机 = 0usize;
    let (mut 跟fw, mut 跟fh) = (0usize, 0usize);
    let mut 跟模手: Option<Vec<u8>> = None;
    let mut 跟模2: Option<[Vec<u8>; 2]> = None;
    let (mut 跟半, mut 跟半手) = (0usize, 0usize);
    let mut 跟块 = 0.0f64;
    let mut 跟上手 = (0.0f64, 0.0f64);
    let mut 跟上面 = [(0.0f64, 0.0f64); 2];
    // 🔴 **指尖偏置那一套已整个删除**(owner 2026-08-28:直接最终形态,不留历史遗留垃圾)。
    // 它只在"算末端该去哪 → 发位姿"那条老路上才需要。最终形态在**画面里**比误差、
    // 按通道表解动作,接触点是从物体自己的像素反投影来的、再投回去就是原像素 ⇒
    // 相机与偏置的误差**精确抵消**,这个量根本不出现。连带删掉:偏置样本、联合解调用、
    // "没量到偏置就不许转手腕"那道闸,以及 `最小转()`(指定手腕的最后一处)。
    let _ = &手载;
    // 🔴🔴🔴 **六方程那条路走不通时,自动改走单面 —— 而不是原地换位形再掷一次同样的骰子。**
    //(AQ 实测 2026-08-31)
    //
    // AQ 修好"两瓣隔 0 px"之后,表从 `0 列` 变成 `1 列` —— 还是过不了"至少三列"。
    // 剩下的死因换成了刚体闸:`通道 末端4:两个接触面没一起动(差 1.346 > 刚体上限 0.233)`。
    // 那道闸本身是对的(两根手指长在同一只手上),它说的是**跟踪器锁错了图案** ——
    // 头相机里两瓣只隔五十来个像素、长得一模一样,转手腕的时候必然跟丢。
    // 而 `好列 < 3 ⇒ continue` 只是换个位形**再掷一次同样的骰子**:AP 和 AQ 都死在这个循环里。
    //
    // 出路一直都在代码里:**单面** —— 只跟"整只手"那一块(四十来像素、独一无二、全画面搜也不会错),
    // 每个通道给三个数而不是六个 ⇒ **位置受控,朝向不受控**。
    // 而这一炮的任务是**把球抓起来**:球是圆的,腕子朝哪根本不影响能不能夹住它。
    // ⇒ 六方程试一次;拿不到三列就**从此改走单面**,别再拿朝向去换"一步都走不了"。
    let mut 表败 = 0u32;
    let mut 挑过手 = false;
    loop {
        let Some(帧) = plug.sense() else { return None };
        if plug.复位过 { plug.复位过 = false; 集 += 1; 试过.clear(); 白转 = 0; println!("[服] ── 第 {集} 集 ──"); }
        白转 += 1;
        if 白转 >= 3 {
            // 连着两拍什么都没干成 ⇒ **粗着干一次**,别站着。
            白转 = 0;
            let 帧0 = plug.sense();
            if let Some(e0) = 帧0.and_then(|f| f.ee.get(手号.get()).copied()) {
                let (p0, q0) = ([e0[0], e0[1], e0[2]], [e0[3], e0[4], e0[5], e0[6]]);
                let 前 = {
                    let (w, x, y, z) = (q0[0], q0[1], q0[2], q0[3]);
                    [2.0*(x*z + y*w), 2.0*(y*z - x*w), 1.0 - 2.0*(x*x + y*y)]
                };
                // 0.25 是**比例**(量出来的可达带的四分之一),不是米。
                let 去 = [p0[0] + 前[0] * 可达带[1] * 0.25,
                          p0[1] + 前[1] * 可达带[1] * 0.25,
                          p0[2] + 前[2] * 可达带[1] * 0.25];
                println!("[服] ⚠️ 连着两拍没做成任何尝试 ⇒ **粗着干一次**(朝手腕正前方压过去、合、抬),失败也留视频");
                落(plug, 去, q0, 1.0, 等拍);
                for _ in 0..60 { 抓握(plug, 0.0, *通道是关节); if plug.sense().is_none() { break } }
                // 同上,**比例**。
                let 抬 = [去[0] - 前[0] * 可达带[1] * 0.25, 去[1] - 前[1] * 可达带[1] * 0.25, 去[2] - 前[2] * 可达带[1] * 0.25];
                落(plug, 抬, q0, 0.0, 等拍);
                抓握(plug, 1.0, *通道是关节);
            }
            continue;
        }
        let Some(e) = 帧.ee.get(手号.get()).copied() else { continue };
        let (此位, 此姿) = ([e[0], e[1], e[2]], [e[3], e[4], e[5], e[6]]);
        let jaw0 = 帧.jaw.get(手号.get()).or_else(|| 帧.jaw.first()).copied().unwrap_or(1.0);

        // ① 指令 —— 整句直接问眼,驱动不解析任务名。
        // 🔴 **客户可以直接下单。** 环境自带的 `instruction` 是"这一集的任务",
        //    而 driver 本来就该听得懂**当场给的一句话**(现场改需求是常态,不是特例)。
        //    `BL_ORDER` 有内容就用它,没有就照旧用观测里那句。
        //    通用:它不认识任何任务名/物体名/机体名,整句原样交给眼。
        let 指令 = std::env::var("BL_ORDER").ok().filter(|t| !t.trim().is_empty())
            .or_else(|| plug.last.clone()
                .and_then(|o| 取(&o, &["instruction".to_string()]))
                .and_then(|v| 字(&v)))
            .filter(|t| !t.is_empty());
        let Some(指令) = 指令 else {
            plug.act(&Cmd::Hold);
            continue;
        };
        let Some((w, h, rgb)) = 彩(&帧, 相机号) else { continue };

        // 🔴🔴🔴 **减法③:候选区域由【几何】出,眼只负责【挑一个】。**(owner 2026-08-28)
        //
        // 以前是"问眼要一个框",框的边界是模型脑补的。量过的代价:
        //   · 让模型给坐标 **0.403 ≈ 瞎猜** vs 让它从选项里挑 **0.916 ≈ 超人类**(同一批模型)
        //   · 脑补出来的坐标,**对的和错的长得一模一样**;而挑错一个画在图上的框,渲一张图就看得见
        //   · 框自带边界 ⇒ **不用再猜半径** —— 而"猜出来的半径"正是把桌面圈进点云的那一个
        // 候选从**深度图**里长出来:空间里一张平面满足 `1/z` 关于像素线性,拟出支撑面,
        // 比它鼓出来的连通块就是一个个物体。**不认识任何物体名,也没有任何检测器。**
        let 区们: Vec<point_gen::区> = plug.lay.cams.get(相机号).and_then(|路| {
            let mut 深路 = 路.clone();
            if let Some(dp) = plug.lay.depth.get(相机号).or_else(|| plug.lay.depth.first()) { 深路 = dp.clone(); }
            plug.last.as_ref().and_then(|o| 取(o, &深路)).and_then(|dv| wire::as_f32_grid(&dv))
        }).map(|(dw0, dh0, dep0)| {
            // 🔴🔴🔴 **算支撑面之前,先把"我自己"从深度图里挖掉。**(ARX 双臂逼出来,2026-08-30)
            //
            // 找东西的办法是:拟合出支撑面,**比它鼓出来的连通块**才算物体。
            // 单臂时我只占画面一角,不挖也无所谓;**ARX 是双臂,两条又大又近的胳膊占了下半幅**,
            // 支撑面拟合被它们拧歪 ⇒ 要么什么都鼓不出来,要么鼓出来的就是我自己。
            // 实测(AE):`几何一块都没切出来` **10 次**;唯一切出来的一块 2816 px · 深 0.356 m
            // 就是胳膊,眼睛看了都说"这张桌上没有它要的东西"。而球一直在画面里。
            //
            // 判据用仓里本来那条,不引入新东西:**我一动,画面里跟着动的就是我。**
            // 拿上一拍的深度图和这一拍相减,变了的像素挖掉(设成 NaN,`分块` 会跳过)。
            // 门槛是**自己量的**:全图变化量的中位数 × 5(大部分像素是静的,中位数就是噪声地板),
            // 5 是无量纲倍数。第一拍没有上一帧就不挖 —— 少挖一拍,不编数。
            // ⚠️ 我推过去的东西也会被挖掉 —— 那是对的:它这一拍在动,下一拍停了自然回来。
            let mut dep0 = dep0;
            if let Some((pw, ph, pd)) = 上深.as_ref() {
                if *pw == dw0 && *ph == dh0 && pd.len() == dep0.len() {
                    let mut 变: Vec<f64> = dep0.iter().zip(pd.iter())
                        .map(|(a, b)| if a.is_finite() && b.is_finite() { (a - b).abs() } else { 0.0 })
                        .collect();
                    // 🔴🔴🔴 **门槛要按"这一拍最大的那个变化"定,不能按中位数。**(AG 实测 2026-08-30)
                    //
                    // 我先写"全图中位 × 5" ⇒ 大部分像素没变时中位是 0,门槛塌到零,**有一丁点
                    // 变化的像素全被挖掉**(那一拍挖了 81480 个,随后眼挑到 11 px 的伪影)。
                    // 改成"只取变过的那些的中位" —— **还是塌**:深度是浮点,**几乎每个像素都有
                    // 亚毫米级的非零差**,它们的中位数照样约等于 0(实测 `界 0.0000 = 中位 0.0000 × 5`)。
                    // 我两次都把"变过"当成了"动过"。
                    //
                    // 真正要分的是两个量级:**胳膊动几厘米 vs 噪声亚毫米**。
                    // ⇒ 门槛 = 这一拍**最大变化的 5%**(最大取 99.9 分位,防单点野值)。
                    // 胳膊动 3 cm ⇒ 门槛 1.5 mm,天然压在噪声之上;什么都没动时最大值本身就是噪声,
                    // 5% 更小,挖不掉东西。**一个无量纲比例,没有绝对常数。**
                    let mut 排: Vec<f64> = 变.clone();
                    if 排.is_empty() {
                        println!("[服] 🎯 这一拍深度图一个像素都没变 ⇒ 不挖(不编数)");
                    } else {
                        排.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
                        let 中 = 排[(((排.len() as f64) * 0.999) as usize).min(排.len() - 1)];
                        let 界 = 中 * 0.05;
                        let mut 挖 = 0usize;
                        for i in 0..dep0.len() {
                            if 变[i] > 界 { dep0[i] = f64::NAN; 挖 += 1; }
                        }
                        if 挖 > 0 {
                            println!("[服] 🎯 算支撑面前挖掉「我自己」:{挖} 个像素这一拍动过(界 {界:.4} m = 这一拍最大变化 {中:.4} 的 5%)");
                        }
                    }
                    变.clear();
                }
            }
            上深 = Some((dw0, dh0, dep0.iter().map(|x| *x).collect::<Vec<f64>>()));
            // 3 是**无量纲倍数**:鼓出来超过背景残差自己的稳健 σ 的 3 倍才算一块。
            // 这个数是**在真实深度图上验出来的**(2026-08-28):3 ⇒ 立方体/球/钞票/小车四件全中;
            // 5 起就把它们杀光(桌上的东西只鼓 0.08–0.11 m,而边界伪影鼓 0.2–1.0 m,
            // 门槛一抬先杀真的、留下假的)。
            let mut v = point_gen::分块(&dep0, dw0, dh0, selfcal::最少像素(dw0, dh0) as usize, 3.0);
            // 🔴 **贴着画面边的那些块丢掉。**
            // 同一帧上的三个伪影(上方整条背景 · 左边一条细缝 · **我自己的胳膊**)
            // 有唯一的共同点:都贴着画面边。而一件我能拿起来的东西是**完整地在画面里**的
            //  —— 它跑出画面我连它有多大都不知道。这条同时把"我自己"排除掉,
            // 不需要认识"机器人"这个概念。真在边上的东西会在下一拍手挪过之后重新出现。
            v.retain(|r| r.框[0] > 0 && r.框[1] > 0 && r.框[2] + 1 < dw0 && r.框[3] + 1 < dh0);
            v
        }).unwrap_or_default();

        // 候选画到图上(编号 1 起),**这张画过的图就是交给眼的那张** —— 它挑错了当场看得见。
        let mut 标图 = rgb.to_vec();
        // 上限只影响"这一拍让眼从几个里挑",不影响任何测量;取像素数最多的那些。
        let 取几 = 区们.len().min(12);
        for (i, r) in 区们.iter().take(取几).enumerate() {
            画编号框(&mut 标图, w, h, r.框, i + 1, [255, 32, 32], 2);
        }
        if let Ok(dir) = std::env::var("BL_DUMP") {
            let _ = std::fs::create_dir_all(&dir);
            let _ = std::fs::write(format!("{dir}/pick.bmp"), task::bmp24(&标图, w, h));
        }

        let mut look = None;
        // 🔴 减法③的后一半:挑中那一块的**掩膜**跟着一起交下去,下游就不必再猜半径。
        let mut 选中掩膜: Option<Vec<bool>> = None;
        // 挑中的是哪一块 —— 避障那一段要拿它把"目标"从"障碍"里摘出来。
        let mut 目标区: Option<point_gen::区> = None;
        if 取几 > 0 {
            match body_layer::eye::pick(眼主机, 眼端口, &指令, &标图, w, h, 取几) {
                Ok(p) if p.region == 0 => println!("[服] 🎯 几何切出 {取几} 块,眼说**一块都不是**(这张桌上没有它要的东西)⇒ 退回问框"),
                Ok(p) if p.region <= 取几 => {
                    let k = p.region;
                    let r = 区们[k - 1];
                    目标区 = Some(r);
                    let (x0, y0, x1, y1) = (r.框[0] as f64 / w as f64, r.框[1] as f64 / h as f64,
                                            r.框[2] as f64 / w as f64, r.框[3] as f64 / h as f64);
                    let 长边 = (x1 - x0).max((y1 - y0) * h as f64 / w as f64).max(1.0 / w as f64);
                    println!("[服] 🎯 几何切出 {取几} 块,**眼挑了第 {k} 块**:{} px · 框 ({:.3},{:.3})-({:.3},{:.3}) · 深 {:.3} m · 鼓出 {:.3} m",
                        r.像素数, x0, y0, x1, y1, r.深, r.高);
                    if let Ok(dir) = std::env::var("BL_DUMP") {
                        let mut 选图 = rgb.to_vec();
                        画编号框(&mut 选图, w, h, r.框, k, [32, 255, 32], 3);
                        let _ = std::fs::write(format!("{dir}/picked.bmp"), task::bmp24(&选图, w, h));
                    }
                    look = Some(body_layer::eye::Look {
                        u: (x0 + x1) * 0.5, v: (y0 + y1) * 0.5, span_frac: 长边,
                        verb: p.verb.clone(), force: p.force.clone(), box01: [x0, y0, x1, y1],
                    });
                    // 掩膜按**这一块自己的深度带**切:一个物体不会比自己更厚,
                    // 而"多厚"是量出来的(它鼓出背景多高),不是一个宽容系数。
                    选中掩膜 = plug.lay.cams.get(相机号).and_then(|路| {
                        let mut 深路 = 路.clone();
                        if let Some(dp) = plug.lay.depth.get(相机号).or_else(|| plug.lay.depth.first()) { 深路 = dp.clone(); }
                        plug.last.as_ref().and_then(|o| 取(o, &深路)).and_then(|dv| wire::as_f32_grid(&dv))
                    }).map(|(dw0, dh0, dep0)| point_gen::区掩膜(&dep0, dw0, dh0, &r, &区们, 3.0));
                }
                Ok(p) => println!("[服] 眼挑了第 {} 块,而只有 {取几} 块 ⇒ 退回问框", p.region),
                Err(x) => println!("[服] 眼挑不出来:{x} ⇒ 退回问框"),
            }
        } else {
            println!("[服] 🎯 几何一块都没切出来(支撑面上没有鼓出来的东西)⇒ 退回问框");
        }
        // 🔴 **退回问框,不许退回"不干"。** 几何切不出/眼挑不中时,老那条路照样能走。
        let look = match look {
            Some(l) => l,
            None => match body_layer::eye::ask(眼主机, 眼端口, &指令, &rgb, w, h) {
                Ok(l) => l,
                Err(x) => { println!("[服] 眼答不了:{x}"); plug.act(&Cmd::Hold); continue }
            }
        };

        // 🔴🔴🔴 **挑手:目标一确定,就用量出来的那两样选出该用的手。**(owner 2026-08-31 死命令)
        //
        // 开机时逐臂推了一下,量到了两件事:①哪个末端属于哪条臂 ②这条臂在主眼画面里占哪一片。
        // 现在眼指出了目标在画面哪个像素 ⇒ **离目标最近的那条臂就是该用的手**。
        // 判据只用画面里的距离,不用三维、不用相机模型、不认识"左/右",
        // 换三条胳膊、五指手、一条臂都同样成立;只有一条臂时它必然选中那一条。
        // ⚠️ 这不是"每拍重选" —— 选定就固定,免得两条臂来回抢;
        //    真选错了,下游会以"合到底指间没东西"暴露,那才是能验证的判据。
        if !挑过手 {
            let mut 最近: Option<(usize, f64)> = None;
            for a in 0..臂心.len() {
                let Some((cu, cv)) = 臂心[a].get(相机号).and_then(|o| *o) else { continue };
                let d = (cu - look.u).hypot(cv - look.v);
                println!("[服]   挑手:第 {a} 条臂在主眼里占的那片中心 ({cu:.3},{cv:.3}),离目标 ({:.3},{:.3}) 差 {d:.3} 画幅",
                    look.u, look.v);
                if 最近.map(|(_, b)| d < b).unwrap_or(true) { 最近 = Some((a, d)) }
            }
            match 最近 {
                Some((a, d)) => {
                    if let Some(e) = 臂末[a] { 手号.set(e) }
                    if let Some(&j) = 臂关.get(a) { 臂号.set(j) }
                    println!("[服] ⇒ **这一炮用第 {a} 条臂**(离目标最近,差 {d:.3} 画幅)⇒ 末端号 {} · 关节组 {}",
                        手号.get(), 臂号.get());
                }
                None => println!("[服] ⚠️ 逐臂推那一步没量到任何一条臂在主眼里占的片 ⇒ 手号留在 {} ,结果照实报", 手号.get()),
            }
            挑过手 = true;
        }

        // ② 目标的三数 —— 眼指的那个像素,加上那一点的**近侧**深度。
        // 近侧:那一小窗里前一半的点。窗里一半是物体、一半是它后面的桌面,中位会滑到桌面去。
        // `span_frac` 是**占画幅的分数**,所以这两个夹值也是画幅的分数,无量纲。
        let 窗 = (look.span_frac * 0.5).clamp(0.004, 0.25);
        let Some(d星) = 近侧深(plug, look.u, look.v, 窗) else {
            println!("[服] 眼指的 ({:.3},{:.3}) 那一点读不到深度 ⇒ 换个位形再看", look.u, look.v);
            plug.act(&Cmd::Hold);
            continue;
        };
        // 🔴 指令原文要进日志 —— 眼指错了地方和手够不着,在数字上都表现为"伺服收敛不了",
        // 而修法完全相反。不印指令就分不开这两件事。
        println!("[服] 🎯 眼给的框(归一化):x {:.3}..{:.3} · y {:.3}..{:.3}",
            look.box01[0], look.box01[2], look.box01[1], look.box01[3]);
        println!("[服] 🎯 指令「{}」⇒ 眼指 ({:.3},{:.3}) · 占画幅 {:.3} · 那一点深 {:.3} m",
            指令.chars().take(80).collect::<String>(), look.u, look.v, look.span_frac, d星);

        // ③ 我的三数。看不清就换个位形,不编数。
        // 🔴🔴🔴 **认手之前,手臂必须【真的停住】。**(WP 看数看出来的,2026-08-28)
        //
        // 认手的办法本身是对的:**冻住胳膊、只动手指,画面里动的那一块按定义就是手**
        //(`LAB`:*"只命令钳口:手臂不动时肘部位移恰好是零,那个候选根本不上场"*)。
        // **但它的前提是"手臂真的不动"**,而日志里明写着 `空帧那会儿手臂还在晃` ——
        // 前提一破,又大又亮的肘部/上臂就赢下这一票,之后**整条链都在把肘往物体上送**。
        //
        // WP 实证:跟到的"手"在 (0.139,0.212)、**深 0.074 m** —— 而物体在 0.64 m 外。
        // 离镜头 7 cm 的东西不可能是手指,那是紧贴相机的一截胳膊;而且"手"和两个接触面
        // 三者坐标完全相同(它以为自己只有一个面)。
        //
        // ⚠️ 我先前加过 `等停`,后来换模板跟踪时把它删了 —— **删错了地方**:
        //    跟踪不需要静止(模板匹配不是帧间差分),**最初那一次认手需要**。
        等停(plug, 等拍 * 4);
        // 🔴🔴🔴 **`看爪` 是【信息源】,不是【前置条件】。**(owner 2026-08-30 定)
        //
        // owner 原话:*"我们不应该有任何的前置条件,我们的 driver 应该具备在不完美条件下
        // 完成任务的能力"*。而 `install.sh` 开头本来就写着:*"下命令就去干。缺什么当场量,
        // 量完接着干"* —— 代码却把 `看爪` 做成硬闸:认不出就 `continue`,**这一拍什么都不干**。
        //
        // 认不出爪子在这两具身体上都是**物理性**的、不会自己变好:
        //   头相机俯视 ⇒ 手指才几个像素、张合挪动也才几个像素;
        //   而这具身体一拍只交付约 10%,"锁死胳膊"那几帧里它仍在漂零点几毫米,
        //   **又大又亮的大臂漂那么一点,翻出来的变化像素比手指真动几厘米还多** ⇒ 大臂赢票。
        //
        // ⇒ 认不出就换一条**不需要看见爪子**的路:推一下任意一个通道,
        //   画面里跟着动的那一片按定义就是**我**,取它的形心当被跟的点。
        //   这个点是**有偏的**(可能落在小臂)—— 照实说出来,但它足够把手带到球跟前;
        //   到了跟前,腕相机收尾和"碰上"会把偏差纠正回来。**碰一下本身就是测量。**
        let 看到 = 看爪(plug, 此位, 此姿, jaw0, 上眼);
        let 看到 = match 看到 {
            Some(t) => Some(t),
            None => (|| -> Option<(f64, f64, f64, f64, f64, (f64, f64), Vec<(f64, f64)>, [f64; 4])> {
                let f0 = plug.sense()?;
                let (gw, gh, g0) = 灰(&f0, 相机号)?;
                let q0 = f0.joints.get(臂号.get()).cloned()?;
                if q0.is_empty() { return None }
                let mut q = q0.clone();
                // 10 是**比例**(量出来的可达带的十分之一),不是米。
                q[0] += 可达带[1] / 10.0;
                if !plug.act(&Cmd::Joints { arm: 臂号.get(), q, jaw: jaw0 }) { return None }
                定爪(plug, 等拍);
                let f1 = plug.sense()?;
                let (_, _, g1) = 灰(&f1, 相机号)?;
                if g1.len() != g0.len() { return None }
                // 8 是灰度差的噪声底(和认接触面那一处同一个来源),无量纲。
                let mut xs: Vec<usize> = Vec::new();
                let mut ys: Vec<usize> = Vec::new();
                for i in 0..g0.len() {
                    if g0[i].abs_diff(g1[i]) > 8 { xs.push(i % gw); ys.push(i / gw); }
                }
                if xs.len() < 32 {
                    println!("[服]   认不出爪子,而推一下通道画面也只变了 {} 个像素 ⇒ 这台相机里我不在画面上", xs.len());
                    return None
                }
                xs.sort_unstable(); ys.sort_unstable();
                let n = xs.len();
                let (cu, cv) = (xs[n / 2] as f64 / gw as f64, ys[n / 2] as f64 / gh as f64);
                let 跨 = ((xs[n * 9 / 10] - xs[n / 10]) as f64 / gw as f64)
                    .max((ys[n * 9 / 10] - ys[n / 10]) as f64 / gh as f64).max(0.01);
                let dd = 近侧深(plug, cu, cv, 跨 * 0.5)?;
                println!("[服]   🟡 **认不出爪子 ⇒ 不停手**:推一下通道,画面里 {n} 个像素跟着动 = 我;");
                println!("[服]      取它的形心 ({cu:.3},{cv:.3}) 深 {dd:.3} m 当被跟的点。**这个点有偏**(可能落在小臂),");
                println!("[服]      够把手带到球跟前;到了跟前,腕相机收尾和\"碰上\"会把偏差纠正回来。");
                Some((cu, cv, dd, 跨, 跨, (cu, cv), vec![(cu, cv), (cu, cv)], [cu - 跨 * 0.5, cv - 跨 * 0.5, cu + 跨 * 0.5, cv + 跨 * 0.5]))
            })(),
        };
        let Some((mut u, mut v, mut d, 张, 块, 开合像, 面们, 爪框)) = 看到 else {
            // 🔴🔴 **这里【不许】有我写的逃生动作。**
            //
            // 原来这儿有两条:"滑回上次看见它的地方"和"把手腕转八分之一圈"。两条都是我的策略:
            // 前一条是**「回原位」的残余**(直播里不存在可回的家,owner 2026-08-23 死命令),
            // 后一条的角度和轴都是我拍的。而**手腕该摆成什么样,是接触集那一层解出来的**,
            // 轮不到我在这儿转。
            // 看不见自己的手 = 一个**具名的缺口**,照实说出来、这一拍不下手,不编数、不发明动作。
            // 🔴🔴 **看不见自己的手 ⇒ 把自己挪进画面里,不许站着。**(WZ 渲图查出来的死锁)
            //
            // 实测代价(WZ,2026-08-28,**看图看出来的,日志上看不出来**):这具身体开局时爪子压在
            // 头部相机的**左下角边缘、一半在画面外**(唯一认出来的那次落在 (0.161,0.937),
            // 由它算出的两个接触面一个在 **v=1.134 —— 画面外**)。于是:
            // 认不出爪 ⇒ 建不了表 ⇒ 追不起来 ⇒ 手一直不动 ⇒ 下一拍还是认不出。**死锁**。
            // 代价照记:80 分钟一集里合爪那一步只走到过 **2** 次,其余全卡在这一句上。
            //
            // ⚠️ 这**不是**我以前删掉的那两条逃生动作(回原位 / 把手腕转八分之一圈)——
            // 那两条的方向和角度是我拍的。这一条里我一个方向都没有指定:
            //   **目标** = "让相机多看见我一点",而"多看见"是量出来的(命令一下,画面上跟着变的
            //             像素数);相机中心是**相机的**性质,不是身体的。
            //   **怎么挪** = 逐个通道试一步,变多了就留着,变少了就退到另一边。
            //             没有哪个通道是"抬"、哪个是"转",它们对我一律只是编号。
            // 顺带的好处:跟着变的像素天然偏向**动得最快的那一头**,也就是手那一头。
            // 🔴🔴🔴 **死锁出口:头相机上认不出爪子是一道【赢不了】的闸。**(YF 实测 2026-08-29)
            //
            // YF 一集里连着 **344 次**"这一眼没看清爪子",每次挪一步再试,89131 帧里
            // 建表/追/收尾**一次都没进过**(`[收尾]` 日志 0 行)。渲图查明:爪子在头相机里
            // 只有几个像素、贴着画面边缘;而在**长在手上**那台里**两瓣占满画面正中**
            // (四帧全是)。⇒ 认不出就一直挪,挪到天亮也认不出 —— 出口不是挪,是**换那台看得见的**。
            //
            // ⚠️ 不是整体切相机:眼睛在腕相机里会指到机器人自己(注释里记着实测 (0.500,0.500))。
            //    腕相机那一段**不问眼睛**,靠"手一动、画面里动的是世界"分出物体 —— 它早写好了,
            //    只是被这道闸挡在外面够不着。
            // 8 是**重试的稀疏度**(计数,无量纲):每失败 8 次试一次,免得每拍都付一遍张合的代价。
            // 🔴🔴🔴 **头相机认不出爪子 ⇒ 腕相机【立刻】接手,不再每 8 次才试一次。**
            //(owner 2026-08-29 令"把看爪搬到腕相机上")
            //
            // `LAB "认手不许靠「哪块动得最像我」"`(2026-08-17)判死了"挑动得最像我的那一块"
            //(肘部会赢:自报 0.04 px / 真误差 **167 px**),并给出唯一站得住的办法:
            // **只命令钳口 —— 手臂不动时肘部位移恰好是零,那个候选根本不上场。**
            // 这条一直在代码里(`看爪` / 收尾段的张合相减),失败的原因是**没有一台相机拍得到手指**:
            // 头相机里张合只让 0–110 个像素变(配对要上千)。
            // 2026-08-29 修好腕相机之后,驱动自己的逐台判词第一次给出 `各台 [true,false,true,true,false]`
            // —— **腕相机通过了"只动手指、画面里有东西动"这项检查**。
            // ⇒ 头相机这道闸赢不了的时候,把这份活整个交给腕相机,而不是继续挪。
            // (原来写 `静 % 8 == 1` 是怕张合那一下的代价;而现在它是**唯一**走得通的路,不该省。)
            if let Some(腕机) = 手上相机 {
                let 起EE = plug.sense().and_then(|f| f.ee.get(手号.get()).copied());
                let mut 位收 = 此位;
                println!("[服] ⇒ 第 {相机号} 台里连着 {静} 次认不出爪子 ⇒ **改走第 {腕机} 台(长在手上那台)**");
                if 收尾腕(plug, 腕机, jaw0, *通道是关节, &look, &mut 位收).is_some() {
                    let 停在 = 合到停住(plug, *通道是关节).unwrap_or(f64::NAN);
                    match (停在.is_finite(), *空合载) {
                        (true, Some(z)) if (停在 - z).abs() > 1e-9 =>
                            println!("[服] 🟢 合到停住,读数停在 {停在:.4}(空手合到底是 {z:.4})⇒ **指间有东西**"),
                        (true, Some(z)) =>
                            println!("[服] 🔴 合到底了,读数停在 {停在:.4},和空手合到底 {z:.4} 一样 ⇒ **指间没东西**"),
                        (true, None) =>
                            println!("[服] ⚠️ 还没量到「空手合到底报几」⇒ 弱判据:读数停在 {停在:.4}"),
                        _ => println!("[服] 🔴 合爪读数读不出来"),
                    }
                    // 抬 = 回到**下手之前**那个位置。这里没有"往上"这种方向断言:
                    // 下手是从哪儿来的,就回哪儿去,方向由身体自己刚才走过的路给。
                    if let (Some(e0), Some(e)) = (起EE, plug.sense().and_then(|f| f.ee.get(手号.get()).copied())) {
                        let _ = 落(plug, [e0[0], e0[1], e0[2]], [e[3], e[4], e[5], e[6]], 0.0, 等拍);
                        println!("[服] ⇒ 提回下手前那个位置(净位移 {:.4} m)",
                            ((e[0]-e0[0]).powi(2)+(e[1]-e0[1]).powi(2)+(e[2]-e0[2]).powi(2)).sqrt());
                    }
                    静 = 0;
                    continue;
                }
                println!("[服] ⇒ 第 {腕机} 台那一段也没走通(理由在上面)⇒ 接着挪");
            }
            静 = 静.wrapping_add(1);
            let 挪了 = 挪进画面(plug, 静, jaw0, *通道是关节);
            match 挪了 {
                Some(true) => println!("[服] 这一眼没看清爪子(连着第 {静} 次)⇒ 已经**挪了一步让相机多看见我一点**,下一拍再认"),
                _ => {
                    // 连"我动了画面也不变"都发生了 ⇒ 这台相机里我根本不在画面上。
                    // 照样不许站着:换下一个通道接着挪,下一拍再认。
                    println!("[服] 这一眼没看清爪子(连着第 {静} 次)⇒ 挪了一步但这台相机里**一个像素都没变**(我不在它画面上)⇒ 继续挪");
                }
            }
            continue;
        };
        静 = 0;
        let _ = 开合像;
        上眼 = Some((u, v));

        let mut 位 = 此位;

        // 🔴🔴🔴 **表要按【通道】量,不按末端量。**(owner 2026-08-27 定,撤回全部非通用改动)
        //
        // 我之前量的是"**末端**往世界 x/y/z 挪一点 ⇒ 画面跑多少" —— 那假设了
        // **有一个末端、而且能按位姿下命令**。LeKiwi、无人机、电锯臂都没有这个通道:
        // 这一炮就算夹起来了,真机照样夹不起来。
        //
        // 通用形只有一句:**通道 = 观测里报出来的每一个能下命令的自由度**
        //(关节 / 手指 / 桨 / 轮 / 舵,数量由布局发现给),
        // 表 = **命令通道 k 一点 ⇒ 被跟的那一块在画面上(横、纵、深)各变多少**。
        // 里面没有末端、没有手腕、没有 IK、没有"够多远"。无人机把桨当通道走的是同一段代码。
        //
        // 幅度也不写死:**从小往上翻倍,直到画面上真的看得见它动** —— 看得见的判据是
        // "变化超过跟踪器自己的噪声",而噪声是量出来的(同一位形连拍两帧的抖动)。
        let 关节数 = 真臂(&帧).iter().map(|&i| 帧.joints[i].len()).sum::<usize>();
        if 关节数 == 0 { println!("[服] 🔴 这具身体没报关节 ⇒ 通道表量不了。**具名缺口,不编数。**"); continue }
        // 🔴🔴 **"我的两个接触面在哪" —— 只在这一处回答。**
        // 此前有两处各答一遍:建表那一段用认块器的长轴两端,追那一段用张合相减。
        // 两处只有一处有自检 ⇒ 表照样是拿没验过的点建的。**一个问题只允许有一处答案。**
        let 认接触面 = |plug: &mut Plug<S>, 是关节: bool| -> Option<[(f64, f64); 2]> {
            // 🔴🔴🔴 **"我的两个接触面在哪"必须【能自己判错】,否则它出错时会返回一条完全正常的读数。**
            //
            // 旧写法:把张合前后变了的像素框起来,沿长轴取两端。**任何一块整体平移的东西都能骗过它** ——
            // 它没有任何一条判据能说"我认错了",于是错误的两个点一路往下游走,
            // 在第五段(追不动)才爆出来。**这是"bug 修不完"的机制本身。**(owner 2026-08-27)
            //
            // 通用判据只有一句,而且就是抓握通道的**定义**:
            //   **抓握通道改变的是【我的接触面之间的距离】。**
            // ⇒ 两个候选点,从张到底到合到底,**间距必须真的变**;只是一起平移、间距不变的,
            //   就不是钳口的两瓣 —— 具名拒绝,不编造。
            // 这一条对两指/五指/吸盘/无人机同样成立(认不出来就说认不出来,不是硬给两个点)。
                抓握(plug, 1.0, 是关节);
                定爪(plug, 等拍 * 8);
                let Some((aw, ah, 开帧)) = plug.sense().and_then(|f| 灰(&f, 相机号)) else { return None };
                // 🔴 噪声参照必须是**同一条命令再发一次**(空对照),不是"什么都不做的下一帧"。
                // 合爪那一下伺服会把整条臂带动一点点 ⇒ 只拿静止两帧当参照,这份抖动会被
                // 当成"抓握通道动的那一块",变化区因此撑到 452×87 px(NVE 实测)。
                // 再发一次同样的命令,变的只有噪声 + 伺服自己的抖 —— 那才是要减掉的东西。
                抓握(plug, 1.0, 是关节);
                定爪(plug, 等拍 * 8);
                let Some((_, _, 开帧2)) = plug.sense().and_then(|f| 灰(&f, 相机号)) else { return None };
                抓握(plug, 0.0, 是关节);
                定爪(plug, 等拍 * 8);
                let Some((_, _, 合帧)) = plug.sense().and_then(|f| 灰(&f, 相机号)) else { return None };
                // 变了多少才算变:**参照就在手边 —— 同一个爪子状态的两帧之间的差,
                // 就是"什么都不做时这幅画面自己会变多少"。** 超过它才算我真的动了那一块。
                // ⚠️ 我先写成"整幅差分的中位数",**已实测撤回**:静止画面的中位数是 0 ⇒
                // 全画面的渲染噪声都被算成变化,变化区撑到 475×90 px,当场被"超过半幅画面"拒掉。
                // (更早还写过 `dv > 3` —— 那是手填的数。)
                // 🔴🔴🔴 **噪声不是靠"挑分位数"去掉的,是靠【形状】去掉的:手指连成一片,噪声是散点。**
                //
                // 实测代价(NVF,2026-08-27,渲图当场看出来):张合那一下画面里真正变的只有约 **176 个像素**
                // (确实在爪子上),而我"取变化最强的四分之一" —— 逐像素减噪之后还剩几万个值为 1–2 的散点,
                // 四分之一里绝大多数是它们,外接框因此撑到 **465×81 px**,当场被"超过半幅画面"拒掉。
                // 之前三条门槛(`dv>3` / 整幅中位数 / 噪声最大值)都是在这条错路上调参数。
                //
                // 正确形:**连通块**。而"多大的一块才算不是噪声"也不用我填 ——
                // **空对照(同一条命令再发一次)里最大的那一块噪声有多大,量一次就知道**,比它大的才是信号。
                // 🔴🔴 **"变了"要看【幅度】,不能只看"不一样"。**
                // 渲图看出来的(NVK):渲染抖动让**每一条边缘**都 ±1~2 级灰度地闪 ⇒
                // 用 `a != b` 时整条手臂 + 风扇的轮廓全部入选、连成一大块,而**真正动的那一小撮
                // 手指**(几十级灰度的明暗翻转、只有十几个像素宽)反而被淹没在里面。
                // 门槛不是我填的:**空对照(同一条命令再发一次)里幅度最大的那一跳有多大,量一次就知道**,
                // 比它大的才算真的动了。
                let 噪幅 = 开帧.iter().zip(开帧2.iter()).map(|(a, b)| a.abs_diff(*b)).max().unwrap_or(0);
                let 掩 = |x: &Vec<u8>, y: &Vec<u8>| -> Vec<bool> {
                    x.iter().zip(y.iter()).map(|(a, b)| a.abs_diff(*b) > 噪幅).collect()
                };
                // 连通块(八邻域),返回每一块的像素表,按大小降序。
                let 连通 = |m: &Vec<bool>, w: usize, h: usize| -> Vec<Vec<(usize, usize)>> {
                    let mut 走过 = vec![false; m.len()];
                    let mut 出: Vec<Vec<(usize, usize)>> = Vec::new();
                    for y0 in 0..h { for x0 in 0..w {
                        let i0 = y0 * w + x0;
                        if !m[i0] || 走过[i0] { continue }
                        let mut 栈 = vec![(x0, y0)];
                        走过[i0] = true;
                        let mut 块 = Vec::new();
                        while let Some((x, y)) = 栈.pop() {
                            块.push((x, y));
                            for dy in -1i64..=1 { for dx in -1i64..=1 {
                                let (nx, ny) = (x as i64 + dx, y as i64 + dy);
                                if nx < 0 || ny < 0 || nx >= w as i64 || ny >= h as i64 { continue }
                                let i = ny as usize * w + nx as usize;
                                if m[i] && !走过[i] { 走过[i] = true; 栈.push((nx as usize, ny as usize)); }
                            }}
                        }
                        出.push(块);
                    }}
                    出.sort_by(|a, b| b.len().cmp(&a.len()));
                    出
                };
                let 噪块 = 连通(&掩(&开帧, &开帧2), aw, ah);
                let 噪上限 = 噪块.first().map(|b| b.len()).unwrap_or(0);
                let 信块 = 连通(&掩(&开帧, &合帧), aw, ah);
                let 强: Vec<(u8, usize, usize)> = 信块.iter().filter(|b| b.len() > 噪上限)
                    .flat_map(|b| b.iter().map(|&(x, y)| (255u8, x, y))).collect();
                println!("[服]   张合相减:空对照的幅度上限 {噪幅} 级、最大一块 {噪上限} 像素 ⇒ 剩下 {} 个像素,{} 块",
                    强.len(), 信块.iter().filter(|b| b.len() > 噪上限).count());
                if 强.len() < 32 {
                    println!("[服]   🔴 抓握通道动过之后,画面里**没有比噪声更大的一块**在动 ⇒ 认不出我的接触面。具名缺口,不编造");
                    return None;
                }
                let 取 = 强.len();
                let mut xs: Vec<usize> = 强.iter().map(|k| k.1).collect();
                let mut ys: Vec<usize> = 强.iter().map(|k| k.2).collect();
                xs.sort_unstable(); ys.sort_unstable();
                let (x1, x2) = (xs[0] as f64, xs[取 - 1] as f64);
                let (y1, y2) = (ys[0] as f64, ys[取 - 1] as f64);
                let (宽px, 高px) = (x2 - x1, y2 - y1);
                if 宽px.max(高px) > aw as f64 * 0.5 {
                    println!("[服]   变化区 {宽px:.0}×{高px:.0} px 大得不像接触面(超过半幅画面)⇒ 这一拍不下手");
                    return None;
                }
                // 沿长轴一分为二,**每一半取它自己的中位点** —— 中位点落在那一瓣身上,
                // 而"长轴两端"落在扫过区域的边界上(任何单帧里那儿多半是背景)。
                let 沿横 = 宽px >= 高px;
                let mut 投: Vec<(f64, usize, usize)> = 强[..取].iter()
                    .map(|k| (if 沿横 { k.1 as f64 } else { k.2 as f64 }, k.1, k.2)).collect();
                投.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
                let 半数 = 投.len() / 2;
                let 中点 = |v: &[(f64, usize, usize)]| -> (f64, f64) {
                    let mut a: Vec<usize> = v.iter().map(|k| k.1).collect();
                    let mut b: Vec<usize> = v.iter().map(|k| k.2).collect();
                    a.sort_unstable(); b.sort_unstable();
                    (a[a.len() / 2] as f64, b[b.len() / 2] as f64)
                };
                // 🔴🔴 **候选点要取在【外端】,不是带子的中间。**
                // 变化区是手指**扫过**的那条带子(张到底的位置 ∪ 合到底的位置)。
                // 带子的中间,手指在**两个状态里都不在那儿** ⇒ 拿它切模板,从张跟到合根本跟不动,
                // 量出来"间距变了 0.0 px",构造性自检当场判死(NVG 实测)。
                // 而"张开 = 两个面分开"这条定义直接给出答案:**张到底时,每一瓣在它那半条带子的外端。**
                // 这里没有身体词也没有世界轴 —— "外"是相对**这条带子自己的中点**说的。
                // 取法:每一半里,再取"投影超过这一半自己中位数"的那部分的中位点(半的半,不是拍的数)。
                let 外半 = |v: &[(f64, usize, usize)], 朝大: bool| -> (f64, f64) {
                    let n = v.len();
                    if n == 0 { return (0.0, 0.0) }
                    let 半 = if 朝大 { &v[n / 2..] } else { &v[..(n / 2).max(1)] };
                    中点(半)
                };
                let (ax, ay) = 外半(&投[..半数], false);
                let (bx, by) = 外半(&投[半数..], true);
                let 甲 = (ax / aw as f64, ay / ah as f64);
                let 乙 = (bx / aw as f64, by / ah as f64);
                // 🔴🔴 **把这一步【自己看到的】三张图存下来** —— 张开 / 合上 / 掩码(候选点画成十字)。
                // owner 2026-08-27 死命令:任何改动都要亲眼看视频,不许只看数字。
                // 之前我靠"在一堆帧里找差最大的一对"去猜驱动当时看的是哪两帧,猜错过好几次;
                // 而这三张图是**它当时真正拿来判断的那三张**,没有中间人。
                if let Ok(存路) = std::env::var("BL_VID") {
                    let _ = std::fs::create_dir_all(&存路);
                    let 掩图: Vec<u8> = {
                        let mut v = vec![0u8; aw * ah];
                        for b in 信块.iter().filter(|b| b.len() > 噪上限) {
                            for &(x, y) in b.iter() { v[y * aw + x] = 255; }
                        }
                        for (px, py) in [(ax, ay), (bx, by)] {
                            for d in -6i64..=6 {
                                for (qx, qy) in [(px as i64 + d, py as i64), (px as i64, py as i64 + d)] {
                                    if qx >= 0 && qy >= 0 && (qx as usize) < aw && (qy as usize) < ah {
                                        v[qy as usize * aw + qx as usize] = 128;
                                    }
                                }
                            }
                        }
                        v
                    };
                    for (名, 图) in [("kai", &开帧), ("he", &合帧), ("mask", &掩图)] {
                        let mut buf = format!("P5\n{aw} {ah}\n255\n").into_bytes();
                        buf.extend_from_slice(图);
                        let _ = std::fs::write(format!("{存路}/renmian_{名}.pgm"), buf);
                    }
                }

                // ── 构造性自检:合上去之后,这两块之间的距离必须真的变了 ──
                let 半试 = (((ax - bx).powi(2) + (ay - by).powi(2)).sqrt() * 0.5) as usize;
                let 半试 = 半试.max(3);
                let (Some(ta), Some(tb)) = (截块(aw, ah, &开帧, 甲.0, 甲.1, 半试),
                                            截块(aw, ah, &开帧, 乙.0, 乙.1, 半试)) else {
                    println!("[服]   两个候选接触面靠边了,切不出模板 ⇒ 这一拍不下手"); return None };
                let 距 = |p: ((f64,f64),(f64,f64))| -> f64 {
                    (((p.0.0 - p.1.0) * aw as f64).powi(2) + ((p.0.1 - p.1.1) * ah as f64).powi(2)).sqrt() };
                let (Some(开对), Some(合对)) = (找两块(aw, ah, &开帧2, &ta, &tb, 半试),
                                               找两块(aw, ah, &合帧, &ta, &tb, 半试)) else {
                    println!("[服]   两个候选接触面跟不住 ⇒ 这一拍不下手"); return None };
                // 噪声:**同一个爪子状态**的两帧之间,这个距离本来就会飘多少。门槛由它给。
                let 距噪 = (距(开对) - ((ax - bx).powi(2) + (ay - by).powi(2)).sqrt()).abs();
                let 变距 = (距(合对) - 距(开对)).abs();
                println!("[服]   变化区 {宽px:.0}×{高px:.0} px");
                println!("[服]   两个候选接触面 ({:.3},{:.3}) / ({:.3},{:.3}) ⇒ 合上去之后间距变了 {变距:.2} px(同状态两帧自己飘 {距噪:.2} px)",
                    甲.0, 甲.1, 乙.0, 乙.1);
                if !(变距 > 距噪) {
                    println!("[服]   🔴 **抓握通道没有改变这两块之间的距离** ⇒ 它们不是我的两个接触面(只是一起平移的同一块东西)。具名缺口,不编造,这一拍不下手");
                    return None;
                }
                Some([甲, 乙])
        };

        // 🔴🔴🔴 **进不进这一段,要看"这一段该产出的东西缺不缺",不能只看其中一个。**
        //(WS 实证,2026-08-28 —— 而且是我自己当天挖的坑)
        //
        // 这一段同时产出两样:**画面雅可比** 和 **通道表**。我当天早些时候加了"量到就存下来"
        //(`存标定`),而它**只存了雅可比、没存通道表** ⇒ 下一炮开机 `雅载` 从文件装回来
        //(日志:`[装] 画面雅可比装回…`)⇒ 这个分支**一次都不进** ⇒ 通道表永远是空的
        // ⇒ 最终形那一段每一轮都在"还没有通道表"上掉头,`追` 在整份日志里出现 **0 次**。
        // **存了一半,反而把另一半饿死了。**
        if 雅载.is_none() || 通道表.is_none() {
            println!("[服]   还没有通道表 ⇒ 逐通道量({关节数} 个关节通道 + {} 个指通道)", 帧.jaw.len());
            // 跟踪用的模板:此刻那一块(认块器刚认出来的)。
            // 🔴 同上:`看爪` 刚刚晃过爪子,先把它摆回**探表全程要用的那个状态**再取样。
            抓握(plug, jaw0, *通道是关节);
            定爪(plug, 等拍 * 8);
            let (fw0, fh0, g00) = match plug.sense().and_then(|f| 灰(&f, 相机号)) { Some(v) => v, None => continue };
            // 🔴🔴 **探表时要【同时跟两个接触面】,不是跟一块。**
            // 只跟一块 ⇒ 每个通道只给三个数 ⇒ 最小二乘只能把**两面的中点**送到两点的中点,
            // **朝向根本没有被控制**(姿态还是继承来的)。要让姿态"长出来",
            // 每个通道必须给**六个数**(两个面各三个),六个方程才真正约束住朝向。
            // 建表用的两个接触面,走**同一处**的认法(带构造性自检),不再用认块器的长轴两端。
            let _ = &面们;
            // 🔴🔴 **解相机只需要"我整只手在画面里怎么动",不需要认出两根手指。**
            // 认两瓣一直是最脆的一段(头部相机里两根手指只隔十来个像素、长得一样),
            // 而我把它**卡在了关键路径前面** ⇒ 认不出 ⇒ 建不了表 ⇒ 解不出相机 ⇒ 整条链停死。
            // 两瓣只有"六行的表(约束朝向)"才用得上,而那条路这一版已经不走了。
            // ⇒ 认得出就用两瓣;认不出**照样往下走**,两个面都取整只手那一点(单面模式)。
            // 🔴🔴 **只跟"我整只手"那一块 —— 两瓣根本不用跟。**
            // 六行的表(两个接触面各三行)唯一的用户是"把两个面伺服到两个接触点"那条路,
            // 而这一版已经不走它了(头部相机里两瓣只隔十来个像素、长得一样,跟踪必然锁错,渲图看过)。
            // 解相机要的只是"我整只手在画面里怎么动" ⇒ 三行就够,而整只手那一块又大又独一无二。
            // ⇒ 把认两瓣**整个从关键路径上摘掉**:少一段会失败的,就少一处会停死的。
            // 🔴🔴🔴 **"我只有一个接触面"必须是【量出来的结论】,不许写死成 true。**(WQ 定案)
            //
            // 上一版硬写 `单面 = true`,理由是"认两瓣是最脆的一段,摘出关键路径";
            // 而同一段注释自己写着后果:*"只跟一块 ⇒ 每个通道只给三个数 ⇒ 最小二乘只能把
            // 【两面的中点】送到两点的中点,**朝向根本没有被控制**"*。
            // 我这一版的伺服**恰恰需要六个方程**(两个面各三个)⇒ 观测那头被关成一个面,
            // 两根手指**从来没有被分别送到那两个点上**(WQ 实测:两个"接触面"坐标逐位相同)。
            //
            // 当初让它脆的原因(头相机里两瓣只隔十来像素、又全画面搜)已经去掉一大半:
            // 跟踪改成了**只在上次位置附近搜**。⇒ 认得出两瓣就用两瓣;真认不出才退单面。
            // ⚠️ 退单面是**降级**,不是结论:它只说明"这一眼没看清",不能推出"我是吸盘"。
            // 🔴🔴🔴 **「认出两个」≠「这台相机分得开两个」—— 这是整张表全空的真凶。**(AP 实测 2026-08-31)
            //
            // AP 的日志一字不改地长这样:
            //     `认出**两个**接触面 ⇒ 六个方程(位置 + 朝向都受控)`
            //     `读接触面:两块只隔 0 px,比模板半径 3 还近 ⇒ 锁到同一块图案上了`   ×6
            //     `通道 末端0..5:翻到 0.0566 都看不见它动 ⇒ 这一列空着`
            //     `通道表(末端那六个自由度):6 列里量到 0 列`
            //     `能看见的通道不足三个 ⇒ 解不出三维,换个位形重量。**不硬解。**`
            // 六列的幅度**一次都没翻**(全是 0.0566 = 起跳档):`读面` 第一帧就断,
            // 而它印的那句"翻到 X 都看不见它动"**把真死因整个盖住了** ——
            // 我因此连着几炮去查"幅度够不够",查错了方向。
            //
            // 真相:`面们.len() >= 2` 只说明**认块器返回了两条记录**,不说明这两条**在画面上分得开**。
            // 头相机离爪子有半米多、又是侧对着看,两瓣落在**同一个像素**上(隔 0 px)。
            // 于是每一帧 `读面` 都合法地拒绝,六列全空,而 3 列以下解不出三维 ⇒ **整个驱动停死在这里**。
            //
            // 判据用**跟踪器自己的最小模板**,不引入任何身体词:两块模板至少要各有半径 `最小半`
            //(再小的图块没有纹理可匹配),所以**两块隔不到 2×最小半 就在几何上不可能分开**。
            // 分不开 ⇒ 退单面(三个方程,朝向这一轮不受控)——**降级,不是结论**,
            // 而三列足够解三维、足够伺服位置。比"六列全空然后拒绝"强一整个数量级。
            let 最小半 = 3usize;   // 和下面两处 `.max(3)` 同一个来源:模板半径的下限(7×7 图块)
            let (面初, 单面) = if 表败 > 0 {
                println!("[服]   上一轮六方程只量到不足三列 ⇒ **这一轮直接走单面**(只跟整只手那一块,位置受控、朝向不受控)");
                ([(u, v), (u, v)], true)
            } else if 面们.len() >= 2 {
                let dx = (面们[0].0 - 面们[1].0) * fw0 as f64;
                let dy = (面们[0].1 - 面们[1].1) * fh0 as f64;
                let 隔px = (dx * dx + dy * dy).sqrt();
                if 隔px < (最小半 * 2) as f64 {
                    println!("[服]   认出两个接触面,但它们在这台相机里只隔 {隔px:.0} px(要 {} px 才切得出两块不重叠的模板)",
                        最小半 * 2);
                    println!("[服]   ⇒ **这台相机分不开这两瓣** ⇒ 退单面(三个方程,朝向这一轮不受控)。**降级,不是结论**");
                    ([(u, v), (u, v)], true)
                } else {
                    println!("[服]   认出**两个**接触面,在这台相机里隔 {隔px:.0} px ⇒ 六个方程(位置 + 朝向都受控)");
                    ([面们[0], 面们[1]], false)
                }
            } else {
                println!("[服]   这一眼只认出一块 ⇒ 暂按单面走(**降级,不是结论**:朝向这一轮不受控)");
                ([(u, v), (u, v)], true)
            };
            // 🔴🔴 **模板多大,由【两个接触面自己隔多远】定 —— 切到刚好不重叠。**
            // 实测代价(NV5,2026-08-27):两个面只隔 38 px,而我给它们各切了 49 px 宽的模板 ⇒
            // 两块模板有一大半是同一片图像,跟踪器根本分不清谁是谁,
            // 于是表里出现"面A 深度跑了 0.1565 m 而末端只挪 0.0296 m"(5.3×)这种列,
            // 而且**去和回都一样地锁错**,来回闸看不出来。
            // 这里没有身体词:"我的两个接触面隔多远"是量出来的,不是"钳口张开量"。
            // 模板半径:认得出两瓣时取"两瓣间距的一半"(切到刚好不重叠);
            // 单面模式下没有间距可言,就取**整只手那一块**的一半 —— 那一块本来就是量出来的。
            let 半 = if 单面 {
                ((块 * fw0 as f64 * 0.5) as usize).max(3)
            } else {
                let dx = (面初[0].0 - 面初[1].0) * fw0 as f64;
                let dy = (面初[0].1 - 面初[1].1) * fh0 as f64;
                (((dx * dx + dy * dy).sqrt() * 0.5) as usize).max(3)
            };
            // 🔴 靠边的时候**缩半径、不挪中心**(见 `合用半` 头注:挪中心会悄悄换掉被跟的点)。
            let 半 = {
                let 缩 = 合用半(fw0, fh0, &[面初[0], 面初[1], (u, v)], 半);
                if 缩 == 0 { println!("[服]   这三个点里有一个**在画面外**(接触面 ({:.3},{:.3})/({:.3},{:.3})、手 ({u:.3},{v:.3}))⇒ 切不出任何模板",
                        面初[0].0, 面初[0].1, 面初[1].0, 面初[1].1); }
                else if 缩 < 半 { println!("[服]   有点靠画面边了 ⇒ 模板半径从 {半} 缩到 {缩} 像素(**中心不动**,跟的还是那两个面)"); }
                缩
            };
            if 半 == 0 { continue }
            let mut 模2: Vec<Vec<u8>> = Vec::new();
            for (pu, pv) in 面初 { match 截块(fw0, fh0, &g00, pu, pv, 半) { Some(t) => 模2.push(t), None => break } }
            if 模2.len() < 2 && !单面 { println!("[服]   截不出两个接触面的模板 ⇒ 单面模式往下走"); }
            let Some(模) = 截块(fw0, fh0, &g00, u, v, 半) else { println!("[服]   截不出模板 ⇒ 换个位形"); continue };
            /// 此刻:两个接触面在画面哪儿/多深 + 中间那一块在哪儿/多深。**现读,不引用任何过去的位姿。**
            // 🔴 每一条退出都要说明理由 —— 静默失败是本仓最贵的一类。
            //    (NV7 实测:六列全打印"翻到 0.0306 都看不见它动",而真相是**第一次读接触面就断了**,
            //     根本没走到"加倍再看"那一步;那行日志把真实死因盖住了。)
            // 🔴🔴 **眼指的那一点,不许是【我自己】。**
            // 这一段此前**一条自检都没有**:眼指错了就返回一个完全正常的坐标,
            // 而失败要到第五段(追不动 / 合到空气)才爆出来。档案里记过它指到机器人手肘和底座上。
            // 通用判据:**头部这台相机不动 ⇒ 我一动,画面里跟着我动的就是我。**
            // 拿眼指那一点切个模板,搭在**本来就要走**的探针上一起跟 —— 不多花一次动作。
            // 它跑得和我的接触面一样 ⇒ 它长在我身上,具名拒绝并重看。
            let 模眼 = 截块(fw0, fh0, &g00, look.u, look.v, 半);
            let mut 眼是我 = false;
            // 🔴🔴🔴 **整只手全画面找;两根手指只在"上次位置 + 手这一步挪了多少"附近找。**
            //
            // 渲图看出来的(NVL,2026-08-27):在头部这台相机里,两根接触面只隔十来个像素、
            // 而且长得一模一样 ⇒ 拿十来像素的模板做**全画面**搜,必然锁到别处
            //(刚体闸报"差 4.787 > 上限 0.674",一列都建不成)。
            // 这不是 bug,是**视角的物理事实**:那只爪子又远又侧对着看。
            // 而我手上有一个**能看清**的东西 —— **整只手那一块**(认块器给的,四十来像素、独一无二)。
            // 手指长在手上 ⇒ 手挪了多少,手指就挪了多少(差一个 ω×r,很小)。
            // ⇒ 手:全画面找(它不会和别的东西混)。手指:窗口**中心是推出来的**、
            //   大小就是模板自己的宽。**这不是回到当初那个拍脑袋的小窗** —— 那个窗的中心是
            //   "上次在哪",目标一旦跑得比窗远就静默锁错;这个窗的中心已经补掉了整只手的位移。
            // 🔴🔴🔴 **整只手那块的半径【也】要过靠边检查 —— 漏掉它,单面这条路永远走不通。**
            //(AR 实测 2026-08-31)
            //
            // 上一版:`半手 = max(块的半宽, 半)`。`半` 刚被 `合用半` 按画面边缩过(AR 缩到 **2 px**),
            // 而 `块 * fw0 * 0.5` **没过那道检查**,`max` 一取就把缩过的结果顶掉了 ⇒
            // `截块(…, 半手)` 返回 None ⇒ `模手 = None` ⇒ 单面模式下**每一次** `读面` 都在
            // 第一句就 `读接触面:切不出【整只手】那一块的模板` ⇒ 六列全空。
            // AR 日志:该句连出 7 次,`通道表:6 列里量到 0 列`,`⇒ 单面也只量到 0 列`。
            // ⚠️ 这条最阴的地方:**六方程那条路不用 `模手` 的全画面搜也能跑**,所以这个 None
            // 只在"退到单面"之后才会咬人 —— 而退到单面正是六方程失败时的救命路。
            // **兜底路自己带着一个必然失败,等于没有兜底。**
            let 半手 = 合用半(fw0, fh0, &[(u, v)], ((块 * fw0 as f64 * 0.5) as usize).max(半));
            if 半手 == 0 { println!("[服]   整只手那一点 ({u:.3},{v:.3}) 贴着画面边 ⇒ 连 1 像素的模板都切不出来"); }
            else if 半手 < 4 { println!("[服]   ⚠️ 整只手那一块只切得出 {半手} 像素半径(手快跑出画面了)—— 跟踪会很飘,结果照实报"); }
            let 模手 = 截块(fw0, fh0, &g00, u, v, 半手);
            if 模手.is_none() { println!("[服]   🔴 切不出【整只手】那一块的模板(半径 {半手})⇒ 这一轮单面读不到任何东西"); }
            // "上一次它们在哪" —— 预测窗的锚。来回探完净位移为零,所以这两个值一直有效。
            let mut 上手 = (u, v);
            let mut 上面 = 面初;
            // 把模板与锚点交给伺服用(它只认一次、之后一帧一拍地跟)。
            跟相机 = 相机号; 跟fw = fw0; 跟fh = fh0; 跟半 = 半; 跟半手 = 半手; 跟块 = 块;
            跟模手 = 模手.clone();
            if let (Some(a), Some(b)) = (模2.first().cloned(), 模2.get(1).cloned()) { 跟模2 = Some([a, b]); }
            跟上手 = 上手; 跟上面 = 上面;
            let 读面 = |plug: &mut Plug<S>, 上手: (f64, f64), 上面: [(f64, f64); 2]|
                -> Option<([(f64, f64); 2], [f64; 2], (f64, f64), f64, Option<(f64, f64)>)> {
                let Some((_, _, g)) = plug.sense().and_then(|f| 灰(&f, 相机号)) else {
                    println!("[服]     读接触面:这一帧没有第 {相机号} 台的图"); return None };
                let Some(t手) = 模手.as_ref() else {
                    println!("[服]     读接触面:切不出【整只手】那一块的模板"); return None };
                let Some((mu, mv)) = 找块(fw0, fh0, &g, t手, 半手) else {
                    println!("[服]     读接触面:整只手那一块在整幅画面里都匹配不上"); return None };
                let (du, dv) = (mu - 上手.0, mv - 上手.1);
                if 单面 {
                    let Some(md) = 近侧深(plug, mu, mv, 块 * 0.5) else {
                        println!("[服]     读接触面:整只手那一块读不到深度"); return None };
                    let 眼块 = 模眼.as_ref().and_then(|t| 找块(fw0, fh0, &g, t, 半));
                    return Some(([(mu, mv), (mu, mv)], [md, md], (mu, mv), md, 眼块));
                }
                let mut 位 = [(0.0f64, 0.0f64); 2];
                let mut 深 = [0.0f64; 2];
                // 🔴🔴🔴 **两根手指长得一模一样,只能靠"它们之间隔多远"把它们分开。**
                //
                // 实测代价(XB,2026-08-28):两个接触面在头部相机里只隔 **5–10 px**,而模板半径 12 ——
                // 两个模板双双锁到**同一根手指**上,于是 6 列里只量到 3 列,自检算出
                // "深度那一行的模 2.13(光轴的模必须是 1)⇒ 表不可信",整张表作废重量,**循环**。
                // 两个独立的窗口搜索在这个尺度上必然收敛到同一块图案,这不是调参能救的。
                //
                // 修法用一条**探表全程都成立的物理约束**:这一段里**爪子一动不动**
                //(`定爪` 把它按住),所以两个接触面之间的**间距是常数**。
                // ⇒ 先全窗找 A;B **只在"离 A 一个间距"的地方找**,而且窗口半径取间距的一半
                //   —— 这样 B 的搜索范围**在几何上够不到 A**,想锁到一起都不可能。
                // 间距用**上一次读到的**那个(而不是最初那个):手腕慢慢转的时候它跟着转,
                // 而两块之间的距离本身不变。这一条不含任何身体词,五指手上就是"每根手指各有一个间距"。
                {
                    let (pu, pv) = (上面[0].0 + du, 上面[0].1 + dv);
                    let Some(a) = 找块窗(fw0, fh0, &g, &模2[0], 半, pu, pv, 半 * 2) else {
                        println!("[服]     读接触面:第 0 块在预测位置附近匹配不上"); return None };
                    位[0] = a;
                    let 偏 = (上面[1].0 - 上面[0].0, 上面[1].1 - 上面[0].1);
                    let 隔px = ((偏.0 * fw0 as f64).powi(2) + (偏.1 * fh0 as f64).powi(2)).sqrt();
                    // 窗口取间距的一半:够得到 B 的真实位置,够不到 A。至少 1 px。
                    let 窗b = ((隔px * 0.5) as usize).clamp(1, 半 * 2);
                    let Some(b) = 找块窗(fw0, fh0, &g, &模2[1], 半, a.0 + 偏.0, a.1 + 偏.1, 窗b) else {
                        println!("[服]     读接触面:第 1 块在【离第 0 块一个间距】的地方匹配不上(窗 {窗b} px)"); return None };
                    位[1] = b;
                }
                for i in 0..2 {
                    let Some(z) = 近侧深(plug, 位[i].0, 位[i].1, 块 * 0.5) else {
                        println!("[服]     读接触面:第 {i} 块在 ({:.3},{:.3}) 读不到深度", 位[i].0, 位[i].1); return None };
                    深[i] = z;
                }
                // 两块落得比模板自己还近 ⇒ 锁到同一块图案上了(**比例**判据,不是填的数)。
                let 隔 = (((位[0].0 - 位[1].0) * fw0 as f64).powi(2) + ((位[0].1 - 位[1].1) * fh0 as f64).powi(2)).sqrt();
                if 隔 < 半 as f64 {
                    println!("[服]     读接触面:两块只隔 {隔:.0} px,比模板半径 {半} 还近 ⇒ 锁到同一块图案上了");
                    return None
                }
                let Some(md) = 近侧深(plug, mu, mv, 块 * 0.5) else {
                    println!("[服]     读接触面:整只手那一块在 ({mu:.3},{mv:.3}) 读不到深度"); return None };
                let 眼块 = 模眼.as_ref().and_then(|t| 找块(fw0, fh0, &g, t, 半));
                // 🔴 **把这一次跟到哪儿画出来存下来**(owner:任何改动都要亲眼看视频)。
                // 手那一块画成方框,两个接触面画成十字。序号递增,连起来就是一段跟踪录像。
                if let Ok(存路) = std::env::var("BL_VID") {
                    let mut 图 = g.clone();
                    let mut 点 = |x: i64, y: i64, v: u8| {
                        if x >= 0 && y >= 0 && (x as usize) < fw0 && (y as usize) < fh0 {
                            图[y as usize * fw0 + x as usize] = v;
                        }
                    };
                    let (hx, hy) = ((mu * fw0 as f64) as i64, (mv * fh0 as f64) as i64);
                    for d in -(半手 as i64)..=(半手 as i64) {
                        点(hx + d, hy - 半手 as i64, 255); 点(hx + d, hy + 半手 as i64, 255);
                        点(hx - 半手 as i64, hy + d, 255); 点(hx + 半手 as i64, hy + d, 255);
                    }
                    for (i, p) in 位.iter().enumerate() {
                        let (px, py) = ((p.0 * fw0 as f64) as i64, (p.1 * fh0 as f64) as i64);
                        let v = if i == 0 { 0u8 } else { 255u8 };
                        for d in -8i64..=8 { 点(px + d, py, v); 点(px, py + d, v); }
                    }
                    let n = unsafe { static mut T: u32 = 0; T += 1; T };
                    let mut buf = format!("P5\n{fw0} {fh0}\n255\n").into_bytes();
                    buf.extend_from_slice(&图);
                    let _ = std::fs::write(format!("{存路}/gen{:04}.pgm", n), buf);
                }
                Some((位, 深, (mu, mv), md, 眼块))
            };
            // 噪声底:**用真正的读法连读两次**,两个接触面各飘多少,取大的那个。
            // ⚠️ 旧写法是拿那块小模板做全画面搜再比 —— 它量到的 0.02836 画幅(18 px)根本不是噪声,
            //    **是一次锁错**(NVM 实测)。量一个判据的噪声,就得用那个判据自己的算法。
            let 噪 = {
                let a = 读面(plug, 上手, 上面);
                let b = 读面(plug, 上手, 上面);
                match (a, b) {
                    (Some((p, _, _, _, _)), Some((q, _, _, _, _))) => (0..2)
                        .map(|i| ((p[i].0 - q[i].0).powi(2) + (p[i].1 - q[i].1).powi(2)).sqrt())
                        .fold(0.0f64, f64::max).max(1.0 / fw0 as f64),
                    _ => 1.0 / fw0 as f64,
                }
            };
            println!("[服]     跟踪器自己的噪声 {:.5} 画幅 ⇒ 比它大才算【看见了】", 噪);

            // 🔴🔴🔴 **通道 = 这具身体【接受并且真的响应】的命令自由度 —— 是哪一种,试出来。**
            //
            // 实测(F2):这台机体**七个关节命令全部零响应**(翻到 0.49 rad 画面纹丝不动),
            // 而末端命令动得好好的(量身体那一相 命令 0.0735 m 实到 0.0077 m)。
            // LAB 里早记过同形的一条:`Cmd::Joints` 曾被静默扔掉,七根关节各命令 0.2 rad 实到 0.005。
            // ⇒ 通道**不等于关节** —— 关节只是它的一种。LeKiwi 响应关节、这台只响应末端、
            //   无人机响应桨,**驱动不许假设是哪一种**:先试关节,一个都不响应就试末端那六个自由度。
            //   两种在下游完全一样 —— 表的一列就是"这个自由度动一点,两个接触面各跑多少"。
            // 🔴 "这具机体认哪一种通道"是**量出来的事实**,量过一次就记住 —— 重量表的时候
            // 不必把七个关节再试一遍(每试一遍是几百帧,而答案上一轮就已经有了)。
            let mut 用关节 = *通道是关节;
            let mut 列: Vec<[f64; 6]> = Vec::new();
            // 🔴🔴 **解相机那张 3×3 要用【中间那一整块】的读数,不用两端那两小块。**
            // 实测代价(FC):拿两端小块的深度去建"深度那一行",量出模 **6.48**
            //("走一米深度变 6.48 米",物理上不可能)⇒ 物理闸拦下、整表作废、重量,循环。
            // 那两小块又小又在区域边缘,`近侧深` 的窗里容易混进背景 ⇒ 深度会跳。
            // 中间那一整块大得多、深度稳得多,而我探表时**本来就在跟它**,只是没用。
            // ⇒ **解相机用中块(稳),控制用两端(才约束得住朝向)**,各用各的,不混。
            let mut 中列: Vec<[f64; 3]> = Vec::new();
            let mut 好列 = 0usize;
            let (mut cu, mut cv) = (u, v);
            let mut cd = d;
            // 🔴🔴🔴 **没有基准,没有"回去"。** 每一列都是【此刻】到【下一刻】的差 —— 局部导数,
            // 在哪儿都成立。**真机直播没有复位键,驱动里也不许有**(owner 2026-08-27)。
            // 探一个通道 = 走一个**来回**:按 +δ 走一格,再按 −δ 走回来。两半是同一列量的两遍
            //(各自除以自己的**实到**位移,所以两遍应当**相等**),对不上就说明跟踪器跳到别的图案上了 ——
            // **真位移可逆,假位移不可逆**。一个来回净位移为零,所以整表探完身体也不会被推出视野
            //(原来那个"每列之前回基准"要挡的就是这件事,现在不需要了)。
            //
            // 实测代价(V2,2026-08-27):旧写法回了基准、却拿**上一列探完停的地方**当起点相减 ⇒
            // 差值里混进整段回程,第 2 列往后全废:末端实际只挪 0.0296 m,表里写着
            // "接触面深度跑了 0.2552 m"(差 8.6 倍,物理上不可能)。那张表就是最后一两厘米的方向盘。
            // 关节读数有没有**自己**动过(哪怕画面没动)。命令下去而读数纹丝不动 = 命令被扔掉,
            // 这是比"画面没动"强得多的证据,而且一个通道就够 —— 不必把七个各试一遍(每个约 120 帧)。
            let mut 报动过 = false;
            for k in 0..(if 用关节 { 关节数 } else { 0 }) {
                // 8 是**比例**(从探幅的八分之一起步,不够再翻倍),无量纲。
                // 🔴🔴🔴 **翻倍梯子必须有上界 —— 没有上界的自检会把整张桌子推平。**
                //(AS 实测 2026-08-31,**整炮 144 帧渲图为证**:桌上原有两个球/剪刀/薯片袋/刀/平板,
                // 第 2–3 屏胳膊整条摊平横扫过桌面,第 4 屏起桌上只剩零星几件,
                // 之后 9 万步里一遍一遍把胳膊伸到最长划大弧。)
                //
                // 日志里那句 `翻到 1.8122 都看不见它动` —— **1.8122 是米**。
                // 起点 探幅/4 = 0.0566 m,翻 5 次 = 1.81 m,而这具身体量出来的可达半径是 **0.679 m**
                // ⇒ 它在命令末端走到**自己够得着的 2.7 倍**之外。逆解够不着就整条伸直往那个方向甩,
                // 一甩就是一次扫桌;而"来回"两下都是 1.8 m ⇒ **左扫一遍再右扫一遍**,循环九万步。
                //
                // ⚠️ 翻倍的依据是"画面里还看不见它动",而看不见的真实原因往往在别处
                //(AS 就是那条判错的透视刚体上限)⇒ **一个错的判据会驱动这个梯子一路翻到毁桌**。
                // ⇒ 上界用**量出来的** `探幅`(= 可达半径 ÷ 3):超过它就不是"探一下",是横穿工作区。
                //   到顶还看不见 ⇒ 如实说"到顶了",不再翻。
                let 幅顶 = 探幅;
                let mut 幅 = 探幅 / 8.0;          // 起点小,看不见就翻倍 —— 幅度是试出来的
                let mut 这列 = [0.0f64; 6];
                let mut 这中 = [0.0f64; 3];
                let mut 成 = false;
                // 🔴 这一列到底是"翻到多大都看不见它动",还是"**第一帧就读不到接触面**"?
                // 两件事,原来印同一句话 ⇒ AP 那一炮我照着错的那句去查幅度,连查几炮全查错方向。
                let mut 读断 = false;
                for _ in 0..5 {
                    let Some((面0, 深0, 中0, 中深0, 眼0)) = 读面(plug, 上手, 上面) else { 读断 = true; break };
                    let Some(f0) = plug.sense() else { return None };
                    let Some(q0) = f0.joints.get(臂号.get()).cloned() else { break };
                    if q0.len() <= k { break }
                    // 走一格(相对命令),然后等它自己停下来(读数连着两拍一样,无阈值)。
                    let mut 迈 = |plug: &mut Plug<S>, d: f64| -> Option<Vec<f64>> {
                        let 起 = plug.sense()?.joints.get(臂号.get()).cloned()?;
                        let mut q = 起.clone(); q[k] += d;
                        if !plug.act(&Cmd::Joints { arm: 臂号.get(), q, jaw: jaw0 }) { return None }
                        let mut 上 = None; let mut 稳 = 0u32; let mut 末 = None;
                        for _ in 0..(等拍 * 2) {
                            let f = plug.sense()?;
                            let 此 = f.joints.get(臂号.get()).cloned().unwrap_or_default();
                            末 = Some(此.clone());
                            if 上.as_ref() == Some(&此) { 稳 += 1; if 稳 >= 2 { break } } else { 稳 = 0 }
                            上 = Some(此);
                        }
                        末
                    };
                    let Some(q1) = 迈(plug, 幅) else {
                        println!("[服]     这具身体**不认关节命令** ⇒ 通道表量不了。**具名缺口。**");
                        break;
                    };
                    let 实去 = q1.get(k).copied().unwrap_or(0.0) - q0.get(k).copied().unwrap_or(0.0);
                    let Some((面1, 深1, 中1, 中深1, 眼1)) = 读面(plug, 中0, 面0) else { 幅 = (幅 * 2.0).min(幅顶); continue };
                    let Some(q2) = 迈(plug, -幅) else { break };
                    let 实回 = q2.get(k).copied().unwrap_or(0.0) - q1.get(k).copied().unwrap_or(0.0);
                    let Some((面2, 深2, 中2, 中深2, _)) = 读面(plug, 中1, 面1) else { 幅 = (幅 * 2.0).min(幅顶); continue };
                    let 挪像 = ((中1.0 - 中0.0).powi(2) + (中1.1 - 中0.1).powi(2)).sqrt();
                    // 关节通道的单位是弧度,而一根关节转 θ 最多让手也转 θ(腕关节就是这种情形)
                    // ⇒ 刚体上限取 1×两面间距,这是**安全上界**;深度那一条(米比米)对它不成立,跳过。
                    let 是转 = true;
                    // 10 是**比例**(命令幅度的十分之一就算动过了),无量纲。
                    if 实去.abs() > 幅.abs() / 10.0 { 报动过 = true }
                    // 10 与 2 都是**比例**(命令幅度的十分之一、噪声地板的两倍),无量纲。
                    // 🔴🔴🔴 **"命令了却没走到" ⇒ 已经顶住了,绝不许再翻倍。**(AS 实测 2026-08-31)
                    // 上一版把"实到 < 命令的十分之一"和"画面没动"写在同一条里,一起触发翻倍 ——
                    // 那是**越顶不动、越使劲推**:逆解够不着 ⇒ 实到很小 ⇒ 翻倍 ⇒ 更够不着 ⇒ 再翻倍。
                    // 整炮 144 帧渲图为证:胳膊一遍遍伸到最长划大弧,把整张桌子推平。
                    // 两件事必须分开:**画面没动**才是"看不见,加大幅度";
                    // **身体没走到**是"我已经在极限上了",这时候唯一正确的动作是**停下**。
                    if 实去.abs() < 幅.abs() / 10.0 || 实回.abs() < 幅.abs() / 10.0 {
                        println!("[服]     通道 关节{k}:命令走 {幅:.4},实际只走了 去 {实去:.4} / 回 {实回:.4} ⇒ **已经顶住了**(够不着或撞上了),不再翻倍 ⇒ 这一列空着");
                        break
                    }
                    if 挪像 <= 噪 * 2.0 { 幅 = (幅 * 2.0).min(幅顶); continue }
                    let 去 = [(面1[0].0-面0[0].0)/实去, (面1[0].1-面0[0].1)/实去, (深1[0]-深0[0])/实去,
                              (面1[1].0-面0[1].0)/实去, (面1[1].1-面0[1].1)/实去, (深1[1]-深0[1])/实去];
                    let 回 = [(面2[0].0-面1[0].0)/实回, (面2[0].1-面1[0].1)/实回, (深2[0]-深1[0])/实回,
                              (面2[1].0-面1[1].0)/实回, (面2[1].1-面1[1].1)/实回, (深2[1]-深1[1])/实回];
                    // 🔴 **两遍必须对得上:分歧要比共识小。** 比的是两个量出来的数,没有阈值。
                    let 分 = (0..6).map(|i| (去[i]-回[i]).powi(2)).sum::<f64>().sqrt();
                    let 共 = (0..6).map(|i| (去[i]+回[i]).powi(2)).sum::<f64>().sqrt();
                    if !(分 < 共) {
                        println!("[服]     通道 关节{k}:去和回对不上(分歧 {分:.3} ≥ 共识 {共:.3})⇒ 跟丢了,减半重探");
                        幅 *= 0.5; continue
                    }
                    // 🔴 物理闸一:**两个接触面长在同一只手上 —— 刚体。**
                    // 正确的说法是 **ΔA − ΔB = ω × r**:平移通道下身体不转 ⇒ 两点位移**完全相同**;
                    // 转动通道下两点之差最多是「转过的角 × 它们之间的距离」——**而这两个量都是量出来的**
                    //(角就是这一通道的"实到",距离就是两个面在画面上隔多远)。
                    // ⚠️ 我先写成了"分歧 < 共识",**已实测撤回**:绕手腕转的时候两根手指本来就朝
                    // 相反方向走(ΔA ≈ −ΔB)⇒ 共识≈0 而分歧很大,那条判据会把**物理上正确的列**判死
                    //(NV6 末端4:分歧 8.543 / 共识 8.122)。
                    // 深度那一项不放进这条闸:转动通道的"实到"是弧度,和米不能比 —— 深度交给闸二(只管平移)。
                    // 实测代价(NV5):面A (+0.0336,+0.1460) 而面B (+0.0055,-0.0229) 差 17% 画幅,
                    // 而来回闸判"两遍一致 1%" —— 跟踪器**稳定地**锁错同一块图案时,去和回当然也一致。
                    // ⇒ 来回闸查"可不可重复",这一条查"可不可能"。两件事,都要。
                    let 甲 = [(去[0]+回[0])*0.5, (去[1]+回[1])*0.5];
                    let 乙 = [(去[3]+回[3])*0.5, (去[4]+回[4])*0.5];
                    let 面分 = (0..2).map(|i| (甲[i]-乙[i]).powi(2)).sum::<f64>().sqrt();
                    let 我间距 = ((面0[0].0-面0[1].0).powi(2) + (面0[0].1-面0[1].1).powi(2)).sqrt();
                    let 转 = if 是转 { 1.0 } else { 0.0 };   // 这个通道每动一个单位,身体转几弧度(平移通道 0)
                    // 🔴🔴🔴 **平移通道那条上限【本来就是错的】—— 它默认了运动没有沿光轴的分量。**
                    //(AQ / AR / AS 三炮实测 2026-08-31)
                    //
                    // 上一版:平移通道 `转 = 0` ⇒ 上限只剩噪声地板 ⇒ 要求两个接触面在画面里**一模一样地动**。
                    // 而透视投影下这条根本不成立:
                    //   像素 u = f·X/Z ⇒ 平移 V 时 u̇ = (f/Z)(Vx − x̄·Vz);两点同深度时 u̇A − u̇B = −(Vz/Z)·(uA − uB)
                    // ⇒ **只要运动有沿光轴的分量,两个面就【按构造】在画面里分开**,分开的量正比于
                    //   「它们在画面里隔多远」×「Vz/Z」。手往相机方向压一下,两根手指在画面上必然一个往左一个往右
                    //   —— 那是**透视**,不是跟丢。而这条闸把它判成"锁错图案了,减半重探"。
                    // 代价照记:AS 六列**全部**被它判死(`差 2.765 > 刚体上限 0.277`),AQ 5 列、AR 5 列 ——
                    // 三炮的方向盘因此全空,而方向盘空 = 驱动一步都走不了。**今晚真正的堵点是这一条。**
                    //
                    // 修正后的上限和转动那一项**同一个形状**,每个量都是量出来的,不引入任何常数:
                    //   转动:每单位转 1 弧度          ⇒ 贡献 1      × 我间距
                    //   平移:每单位沿光轴走 Vz、深度 Z ⇒ 贡献 |Vz|/Z × 我间距
                    // `Vz` 就是表里深度那一行(已经除过实到),`Z` 就是这一眼现读的深度。
                    let 轴 = if 是转 { 0.0 } else { (((去[2] + 回[2]) * 0.5).abs()) / 深0[0].abs().max(1e-9) };
                    let 界 = (转 + 轴) * 我间距 + 噪 * 2.0 / 实去.abs().max(1e-9);   // 两次读数,各带一份跟踪器噪声
                    if !单面 && !(面分 <= 界) {
                        println!("[服]     通道 关节{k}:两个接触面没一起动(差 {面分:.3} > 刚体上限 {界:.3})⇒ 锁错图案了,减半重探");
                        幅 *= 0.5; continue
                    }
                    // 🔴 眼指的那一点跟着我一起跑 ⇒ 它长在我身上,不是世界里的东西。
                    if let (Some(e0), Some(e1)) = (眼0, 眼1) {
                        let 眼跑 = [(e1.0 - e0.0) / 实去, (e1.1 - e0.1) / 实去];
                        let 差我 = ((眼跑[0] - 甲[0]).powi(2) + (眼跑[1] - 甲[1]).powi(2)).sqrt();
                        if 差我 <= 界 {
                            println!("[服]   🔴 **眼指的那一点跟着我一起动**(它跑的和我的接触面差 {差我:.3} ≤ 刚体上限 {界:.3})⇒ 眼指到了我自己身上,不是世界里的东西");
                            眼是我 = true;
                        }
                    }
                    // 🔴 物理闸二:**长在我身上的东西不可能跑得比我自己还远。**
                    // 深度和位移同是米,直接比。容差用**来回之后没回到原深**那点残差 —— 深度读数
                    // 自己的抖动,量出来的。实测(NV5):末端实到 0.0296 m,而"面A 深度跑了 0.1565 m"(5.3×)。
                    let 越界 = !是转 && (0..2).any(|i| {
                        let 抖 = (深2[i] - 深0[i]).abs();
                        (深1[i] - 深0[i]).abs() > 实去.abs() + 抖
                    });
                    if 越界 {
                        println!("[服]     通道 关节{k}:接触面深度跑得比我自己还远 ⇒ 锁错图案了,减半重探");
                        幅 *= 0.5; continue
                    }
                    上手 = 中2; 上面 = 面2;   // 锚推进到这一列走完之后的位置
                    for i in 0..6 { 这列[i] = (去[i] + 回[i]) * 0.5 }
                    这中 = [((中1.0-中0.0)/实去 + (中2.0-中1.0)/实回) * 0.5,
                            ((中1.1-中0.1)/实去 + (中2.1-中1.1)/实回) * 0.5,
                            ((中深1-中深0)/实去 + (中深2-中深1)/实回) * 0.5];
                    println!("[服]     通道 关节{k}:去 {实去:+.4} 回 {实回:+.4} ⇒ 面A ({:+.4},{:+.4},{:+.4}) 面B ({:+.4},{:+.4},{:+.4}) · 两遍分歧 {:.0}%",
                        这列[0]*实去, 这列[1]*实去, 这列[2]*实去, 这列[3]*实去, 这列[4]*实去, 这列[5]*实去,
                        100.0 * 分 / 共.max(1e-12));
                    成 = true; break;
                }
                if !成 {
                    if 读断 { println!("[服]     通道 关节{k}:**第一帧就读不到接触面**(理由在上一行)⇒ 这一列空着 —— 幅度一次都没翻,别去查幅度"); }
                    else { println!("[服]     通道 关节{k}:翻到 {幅:.4} 都看不见它动 ⇒ 这一列空着"); }
                }
                else { 好列 += 1 }
                中列.push(这中);
                列.push(这列);
                if !成 && !报动过 {
                    println!("[服]     命令下去了,**关节读数自己都没动** ⇒ 这具身体不接受关节命令,剩下的不逐个试了");
                    break;
                }
            }
            if 用关节 { println!("[服]   通道表:{关节数} 列里量到 {好列} 列"); }
            if 好列 == 0 {
                if 用关节 { println!("[服]   **这具机体一个关节命令都不响应** ⇒ 换一种通道:末端那六个自由度,重探"); }
                用关节 = false;
                *通道是关节 = false;   // 记住:下一轮直接从末端开始,别再交一遍学费
                列.clear();
                中列.clear();
                // ⚠️ 这里曾经有一个"每一下探针之前先回到同一个基准位姿"。
                // 它是为了挡住 FA 那次的漂移(六下累加把手臂推出视野,接触面漂到 v≈0.000)。
                // **已删** —— owner 2026-08-27:真机直播没有复位键。同一件事现在由
                // **来回**解决:每个通道 +δ 再 −δ,净位移为零,既不漂也不需要记住任何位姿。
                for k in 0..6usize {
                    // 同上:没有基准、没有回去,走一个来回,两遍互相对表。
                    // 上界同上(见关节表那一段的头注:AS 把桌子推平的就是这个没有上界的梯子)。
                    let 幅顶 = 探幅;
                    let mut 幅 = 探幅 / 4.0;
                    let mut 这列 = [0.0f64; 6];
                    let mut 这中 = [0.0f64; 3];
                    let mut 成 = false;
                    let mut 读断 = false;
                    for _ in 0..5 {
                        let Some((面0, 深0, 中0, 中深0, 眼0)) = 读面(plug, 上手, 上面) else { 读断 = true; break };
                        let Some(f0) = plug.sense() else { return None };
                        let Some(e0) = f0.ee.get(手号.get()).copied() else { break };
                        // 前三个是平移,后三个是绕手腕自己的三根轴转 —— 都是"这具身体接受的自由度",
                        // 不是我给它安的概念:接口里本来就有这六个。
                        let 走 = |e: [f64; 7], d: f64| -> ([f64; 3], [f64; 4]) {
                            let (p, q) = ([e[0], e[1], e[2]], [e[3], e[4], e[5], e[6]]);
                            if k < 3 { let mut pp = p; pp[k] += d; (pp, q) }
                            else {
                                let a = d * 0.5;
                                let mut ax = [0.0; 3]; ax[k - 3] = 1.0;
                                let r = [a.cos(), a.sin()*ax[0], a.sin()*ax[1], a.sin()*ax[2]];
                                let (w1,x1,y1,z1) = (q[0], q[1], q[2], q[3]);
                                let (w2,x2,y2,z2) = (r[0], r[1], r[2], r[3]);
                                (p, [w1*w2 - x1*x2 - y1*y2 - z1*z2,
                                     w1*x2 + x1*w2 + y1*z2 - z1*y2,
                                     w1*y2 - x1*z2 + y1*w2 + z1*x2,
                                     w1*z2 + x1*y2 - y1*x2 + z1*w2])
                            }
                        };
                        // 这一档**实际**沿这个通道动了多少 —— **有符号,而且是沿它自己那根轴的分量**。
                        // 🔴 实测代价(NV3,2026-08-27):原来是"位移的模 × 顺不顺从命令"。
                        // 走回来时它照样顺从 ⇒ 回程也拿到**正**号,于是"去"和"回"两列符号相反,
                        // 来回对表全判跟丢(分歧 2.539 / 共识 0.019),末端六列一列都建不起来。
                        // 顺带把量纲统一成和干活时那条 Broyden 修正一样的:平移取轴上分量,
                        // 转动取相对四元数矢部 ×2 在该轴上的分量。两处不一致会让修正把好表越修越坏。
                        let 实动 = |前: [f64; 7], 后: [f64; 7]| -> f64 {
                            if k < 3 { 后[k] - 前[k] }
                            else {
                                let (w1, x1, y1, z1) = (前[3], -前[4], -前[5], -前[6]);
                                let (w2, x2, y2, z2) = (后[3], 后[4], 后[5], 后[6]);
                                let v = [w1*x2 + x1*w2 + y1*z2 - z1*y2,
                                         w1*y2 - x1*z2 + y1*w2 + z1*x2,
                                         w1*z2 + x1*y2 - y1*x2 + z1*w2];
                                2.0 * v[k - 3]
                            }
                        };
                        let (至位, 至姿) = 走(e0, 幅);
                        // 🔴🔴 **`落` 报的"碰上了"以前被整个丢掉(写成 `_`)。**(AS 实测 2026-08-31)
                        // 探针撞到桌上的东西 ⇒ 实到变小 ⇒ 上一版把它当成"看不见,加大幅度" ⇒
                        // **顶着一件东西越推越狠**,那正是整炮渲图里把桌子推平的动作。
                        // 碰上了就当场停这一列:碰上本身是一次测量,而继续推只会把世界改掉。
                        let Some((f1, 撞去, _)) = 落(plug, 至位, 至姿, jaw0, 等拍) else { return None };
                        let Some(e1) = f1.ee.get(手号.get()).copied() else { break };
                        let 实去 = 实动(e0, e1);
                        if 撞去 {
                            println!("[服]     通道 末端{k}:探到一半**碰上东西了**(命令 {幅:.4},实到 {实去:.4})⇒ 立刻退回,不再加大 ⇒ 这一列空着");
                            let (回位, 回姿) = 走(e1, -实去);
                            let _ = 落(plug, 回位, 回姿, jaw0, 等拍);
                            break
                        }
                        let Some((面1, 深1, 中1, 中深1, 眼1)) = 读面(plug, 中0, 面0) else { 幅 = (幅 * 2.0).min(幅顶); continue };
                        // 走回来 —— **按同样的量反着走一格**,不是回到某个记住的位姿。
                        let (回位, 回姿) = 走(e1, -幅);
                        let Some((f2, _, _)) = 落(plug, 回位, 回姿, jaw0, 等拍) else { return None };
                        let Some(e2) = f2.ee.get(手号.get()).copied() else { break };
                        let 实回 = 实动(e1, e2);
                        let Some((面2, 深2, 中2, 中深2, _)) = 读面(plug, 中1, 面1) else { 幅 = (幅 * 2.0).min(幅顶); continue };
                        // 🔴🔴🔴 **"动没动"必须把【深度】也算进去。**(WT 实证,2026-08-28)
                        //
                        // 上一版只算横纵两维,而**通道 2 是前后移动** —— 手往相机方向走,
                        // 画面里横纵几乎不动、变的全在深度上 ⇒ 被判成"没动",这一列空着;
                        // 转动通道同理吃亏。实测 WT:通道 2/3/4 **全部空着**,
                        // 理由都是"翻到 X 都看不见它动",于是活列不足三个 ⇒ 表建不起来
                        // ⇒ 最终形那一段每轮都在"还没有通道表"上掉头(`追` 出现 0 次)。
                        //
                        // 而**表本身就有深度那一行**(每个面三个数:横 / 纵 / 深)——
                        // 判据只用了三分之二。把深度换成画幅单位补上,和噪声地板同量纲。
                        // 换算是量出来的:`焦距 / (画宽 × 那一点的深)`,一个常数都不引。
                        let 挪像 = ((中1.0 - 中0.0).powi(2) + (中1.1 - 中0.1).powi(2)).sqrt();
                        // 深度那一维和"我实际挪了多远"**同是米**,可以直接比,不需要相机模型
                        //(表是在解相机之前量的,那时 `眼` 还不存在)。
                        // 0.1 是**比例**(深度变化占实际位移的一成以上就算动过了),无量纲。
                        let 深动 = (中深1 - 中深0).abs() > 实去.abs() * 0.1;
                        let 是转 = k >= 3;   // 末端后三个自由度是绕手腕自己的轴转,单位是弧度
                        // 🔴🔴🔴 同上:**命令了却没走到 ⇒ 已经顶住了,绝不许再翻倍。**
                        // (末端这一段上一版只判"实到 ≈ 0",判不出"实到只有命令的十分之一"那种顶住。)
                        if 实去.abs() < 幅.abs() / 10.0 || 实回.abs() < 幅.abs() / 10.0 {
                            println!("[服]     通道 末端{k}:命令走 {幅:.4},实际只走了 去 {实去:.4} / 回 {实回:.4} ⇒ **已经顶住了**(够不着或撞上了),不再翻倍 ⇒ 这一列空着");
                            break
                        }
                        if 挪像 <= 噪 * 2.0 && !深动 { 幅 = (幅 * 2.0).min(幅顶); continue }
                        let 去 = [(面1[0].0-面0[0].0)/实去, (面1[0].1-面0[0].1)/实去, (深1[0]-深0[0])/实去,
                                  (面1[1].0-面0[1].0)/实去, (面1[1].1-面0[1].1)/实去, (深1[1]-深0[1])/实去];
                        let 回 = [(面2[0].0-面1[0].0)/实回, (面2[0].1-面1[0].1)/实回, (深2[0]-深1[0])/实回,
                                  (面2[1].0-面1[1].0)/实回, (面2[1].1-面1[1].1)/实回, (深2[1]-深1[1])/实回];
                        // 🔴 **两遍必须对得上:分歧要比共识小。** 比的是两个量出来的数,没有阈值。
                        let 分 = (0..6).map(|i| (去[i]-回[i]).powi(2)).sum::<f64>().sqrt();
                        let 共 = (0..6).map(|i| (去[i]+回[i]).powi(2)).sum::<f64>().sqrt();
                        if !(分 < 共) {
                            println!("[服]     通道 末端{k}:去和回对不上(分歧 {分:.3} ≥ 共识 {共:.3})⇒ 跟丢了,减半重探");
                            幅 *= 0.5; continue
                        }
                        // 🔴 物理闸一:**两个接触面长在同一只手上 —— 刚体。**
                        // 正确的说法是 **ΔA − ΔB = ω × r**:平移通道下身体不转 ⇒ 两点位移**完全相同**;
                        // 转动通道下两点之差最多是「转过的角 × 它们之间的距离」——**而这两个量都是量出来的**
                        //(角就是这一通道的"实到",距离就是两个面在画面上隔多远)。
                        // ⚠️ 我先写成了"分歧 < 共识",**已实测撤回**:绕手腕转的时候两根手指本来就朝
                        // 相反方向走(ΔA ≈ −ΔB)⇒ 共识≈0 而分歧很大,那条判据会把**物理上正确的列**判死
                        //(NV6 末端4:分歧 8.543 / 共识 8.122)。
                        // 深度那一项不放进这条闸:转动通道的"实到"是弧度,和米不能比 —— 深度交给闸二(只管平移)。
                        // 实测代价(NV5):面A (+0.0336,+0.1460) 而面B (+0.0055,-0.0229) 差 17% 画幅,
                        // 而来回闸判"两遍一致 1%" —— 跟踪器**稳定地**锁错同一块图案时,去和回当然也一致。
                        // ⇒ 来回闸查"可不可重复",这一条查"可不可能"。两件事,都要。
                        let 甲 = [(去[0]+回[0])*0.5, (去[1]+回[1])*0.5];
                        let 乙 = [(去[3]+回[3])*0.5, (去[4]+回[4])*0.5];
                        let 面分 = (0..2).map(|i| (甲[i]-乙[i]).powi(2)).sum::<f64>().sqrt();
                        let 我间距 = ((面0[0].0-面0[1].0).powi(2) + (面0[0].1-面0[1].1).powi(2)).sqrt();
                        let 转 = if 是转 { 1.0 } else { 0.0 };   // 这个通道每动一个单位,身体转几弧度(平移通道 0)
                        // 🔴🔴🔴 **平移通道那条上限【本来就是错的】—— 它默认了运动没有沿光轴的分量。**
                        //(AQ / AR / AS 三炮实测 2026-08-31)
                        //
                        // 上一版:平移通道 `转 = 0` ⇒ 上限只剩噪声地板 ⇒ 要求两个接触面在画面里**一模一样地动**。
                        // 而透视投影下这条根本不成立:
                        //   像素 u = f·X/Z ⇒ 平移 V 时 u̇ = (f/Z)(Vx − x̄·Vz);两点同深度时 u̇A − u̇B = −(Vz/Z)·(uA − uB)
                        // ⇒ **只要运动有沿光轴的分量,两个面就【按构造】在画面里分开**,分开的量正比于
                        //   「它们在画面里隔多远」×「Vz/Z」。手往相机方向压一下,两根手指在画面上必然一个往左一个往右
                        //   —— 那是**透视**,不是跟丢。而这条闸把它判成"锁错图案了,减半重探"。
                        // 代价照记:AS 六列**全部**被它判死(`差 2.765 > 刚体上限 0.277`),AQ 5 列、AR 5 列 ——
                        // 三炮的方向盘因此全空,而方向盘空 = 驱动一步都走不了。**今晚真正的堵点是这一条。**
                        //
                        // 修正后的上限和转动那一项**同一个形状**,每个量都是量出来的,不引入任何常数:
                        //   转动:每单位转 1 弧度          ⇒ 贡献 1      × 我间距
                        //   平移:每单位沿光轴走 Vz、深度 Z ⇒ 贡献 |Vz|/Z × 我间距
                        // `Vz` 就是表里深度那一行(已经除过实到),`Z` 就是这一眼现读的深度。
                        let 轴 = if 是转 { 0.0 } else { (((去[2] + 回[2]) * 0.5).abs()) / 深0[0].abs().max(1e-9) };
                        let 界 = (转 + 轴) * 我间距 + 噪 * 2.0 / 实去.abs().max(1e-9);   // 两次读数,各带一份跟踪器噪声
                        if !单面 && !(面分 <= 界) {
                            println!("[服]     通道 末端{k}:两个接触面没一起动(差 {面分:.3} > 刚体上限 {界:.3})⇒ 锁错图案了,减半重探");
                            幅 *= 0.5; continue
                        }
                        // 🔴 眼指的那一点跟着我一起跑 ⇒ 它长在我身上,不是世界里的东西。
                        if let (Some(e0), Some(e1)) = (眼0, 眼1) {
                            let 眼跑 = [(e1.0 - e0.0) / 实去, (e1.1 - e0.1) / 实去];
                            let 差我 = ((眼跑[0] - 甲[0]).powi(2) + (眼跑[1] - 甲[1]).powi(2)).sqrt();
                            if 差我 <= 界 {
                                println!("[服]   🔴 **眼指的那一点跟着我一起动**(它跑的和我的接触面差 {差我:.3} ≤ 刚体上限 {界:.3})⇒ 眼指到了我自己身上,不是世界里的东西");
                                眼是我 = true;
                            }
                        }
                        // 🔴 物理闸二:**长在我身上的东西不可能跑得比我自己还远。**
                        // 深度和位移同是米,直接比。容差用**来回之后没回到原深**那点残差 —— 深度读数
                        // 自己的抖动,量出来的。实测(NV5):末端实到 0.0296 m,而"面A 深度跑了 0.1565 m"(5.3×)。
                        let 越界 = !是转 && (0..2).any(|i| {
                            let 抖 = (深2[i] - 深0[i]).abs();
                            (深1[i] - 深0[i]).abs() > 实去.abs() + 抖
                        });
                        if 越界 {
                            println!("[服]     通道 末端{k}:接触面深度跑得比我自己还远 ⇒ 锁错图案了,减半重探");
                            幅 *= 0.5; continue
                        }
                        上手 = 中2; 上面 = 面2;   // 锚推进到这一列走完之后的位置
                        for i in 0..6 { 这列[i] = (去[i] + 回[i]) * 0.5 }
                        这中 = [((中1.0-中0.0)/实去 + (中2.0-中1.0)/实回) * 0.5,
                                ((中1.1-中0.1)/实去 + (中2.1-中1.1)/实回) * 0.5,
                                ((中深1-中深0)/实去 + (中深2-中深1)/实回) * 0.5];
                        println!("[服]     通道 末端{k}:去 {实去:+.4} 回 {实回:+.4} ⇒ 面A ({:+.4},{:+.4},{:+.4}) 面B ({:+.4},{:+.4},{:+.4}) · 两遍分歧 {:.0}%",
                            这列[0]*实去, 这列[1]*实去, 这列[2]*实去, 这列[3]*实去, 这列[4]*实去, 这列[5]*实去,
                            100.0 * 分 / 共.max(1e-12));
                        成 = true; break;
                    }
                    if !成 {
                        if 读断 { println!("[服]     通道 末端{k}:**第一帧就读不到接触面**(理由在上一行)⇒ 这一列空着 —— 幅度一次都没翻,别去查幅度"); }
                        else { println!("[服]     通道 末端{k}:翻到 {幅:.4} 都看不见它动 ⇒ 这一列空着"); }
                    }
                    else { 好列 += 1 }
                    中列.push(这中);
                    列.push(这列);
                }
                // 来回净零 ⇒ 身体还在原处;中间那一块此刻在哪,**现读一遍**,不从任何一列的终点推。
                if let Some((_, _, 中末, 中末深, _)) = 读面(plug, 上手, 上面) { cu = 中末.0; cv = 中末.1; cd = 中末深; }
                println!("[服]   通道表(末端那六个自由度):6 列里量到 {好列} 列");
            }
            // 🔴🔴 **这一道闸改成【只报数,不否决】。**(WU 实证,2026-08-28)
            //
            // 它的容差是 `噪声×2 ÷ 实际位移` = 0.00156×2 ÷ 0.021 ≈ **0.15**,
            // 而实测差只有 **0.090** ⇒ **容差比信号还大,几乎什么都会被判成"那是我自己"**。
            // 于是它把每一轮都掐掉,表建完了也走不到下一段。
            // 仓里对这一类有明文结论(owner 2026-08-17):
            //   **"保护全拆:6 道闸,6 次把自己的链搞死,0 次挡住真问题 ⇒ 只报数,不否决"**
            // 这一道是第 7 次。⇒ 照那条办:**说出来,但不许掐掉这一轮**。
            // ⚠️ 真正指到自己身上时,下游会以"合到底指间没东西"的形式暴露,那才是能验证的判据。
            if 眼是我 {
                println!("[服]   ⚠️ 眼指的那一点看起来跟着我动(容差可能比信号还大)—— **照常往下走,结果照实报**");
            }
            if 好列 < 3 {
                表败 += 1;
                println!("[服]   能看见的通道不足三个(量到 {好列} 列)⇒ 解不出三维,不硬解");
                if 单面 { println!("[服]   ⇒ 单面也只量到 {好列} 列 ⇒ 换个位形重量(第 {表败} 次)"); }
                else { println!("[服]   ⇒ 六方程这条路在这台相机里跟不住两瓣 ⇒ **下一轮改走单面**(位置受控就够抓球)"); }
                continue
            }
            // 表的真正形状:**6 行(两个面 × 三个数)× 通道数列**。通道数由这具身体报,不设上限。
            *通道表 = Some(列.clone());
            *通道是关节 = 用关节;
            // 老下游(从表解相机)还要一张 3×3:取"面 A 的三行 × 前三个能动的通道"。
            // ⚠️ 它只服务于**感知那半边**(把像素+深度变成三维点),不参与控制。
            // 用**中块**那几列建 3×3(见上面那段:两端小块的深度会跳)。
            let 能动: Vec<usize> = (0..中列.len()).filter(|i| 中列[*i].iter().any(|x| x.abs() > 0.0)).collect();
            if 能动.len() >= 3 {
                let mut m3 = [[0.0f64; 3]; 3];
                for (c, &i) in 能动.iter().take(3).enumerate() { for r in 0..3 { m3[r][c] = 中列[i][r]; } }
                *雅载 = Some(m3);
                // 🔴🔴🔴 **量到就落盘 —— 这是"越用越强"真正生效的地方。**
                //
                // `存标定` 的参数里本来就留好了干活模式的两格(`手` / `雅`,注释明写"干活时量到的"),
                // 而全仓两个调用点**都传 `None, None`**,而且都在标定模式里 ——
                // **接口设计好了,线一直没接。** 于是评测里唯一会跑的干活模式,
                // 每一炮把相机 / 通道表 / 指尖偏置 / 钳口张开重量一遍(≈40 分钟),
                // 进程一退全丢。文件里本来就写着这条:*"增量落盘把'越用越强'从一句设计
                // 变成一个事实:跑到哪存到哪,下一炮 `--in` 接着量"*、
                // *"每量完一格就调一次,不要只在最后调"*。
                // ⚠️ 这些量是**身体的属性,与任务无关** —— 抓棒球量到的,抓剪刀/擦桌子照样用。
                {
                    let n格 = 存标定(标定文件, body, 相机们, 探步, 0, *手载,
                        通道表.as_deref(), Some(*通道是关节), 手上相机, *张开载, *空合载, Some(*认面载), *雅载);
                    println!("[服]   💾 量到的存进 {标定文件}:这一次落了 {n格} 格(下一炮 --in 装回来,不用重量)");
                }
            }
            u = cu; v = cv; d = cd; 上眼 = Some((u, v));
        }
        let Some(m) = *雅载 else { continue };
        // 🔴 深度那一行的模 = 光轴的模,物理上是 1。远离 1 就是那一行是噪声,不许拿它解相机。
        {
            let 深长 = (m[2][0].powi(2) + m[2][1].powi(2) + m[2][2].powi(2)).sqrt();
            // 光轴那一行的模**物理上就是 1**,这两个界是它的**无量纲**上下界,不是长度。
            if !(深长 > 0.5 && 深长 < 1.8) {
                println!("[服]   深度那一行的模 {:.2}(光轴的模必须是 1)⇒ 表不可信,重量一次", 深长);
                *雅载 = None; continue;
            }
        }

        // 🔴🔴🔴 **整台相机从表里闭式解出来** —— 没有拟合、没有标定板、没有一个手填的数。
        // (`point_gen::eye_from_jacobian`,推导与自检见那儿;测试 `从导数把整台相机解回来`。)
        // 有了它,**被绕过去的那一层(接触集)就能装回来** —— 深度图反投影成点云喂给它。
        let (dw, dh, dep) = match plug.lay.cams.get(相机号).and_then(|路| {
            let mut 深路 = 路.clone();
            // 🔴 深度路径由 `discover` 靠形状认出来(减法①)。
            if let Some(dp) = plug.lay.depth.get(相机号).or_else(|| plug.lay.depth.first()) { 深路 = dp.clone(); }
            plug.last.as_ref().and_then(|o| 取(o, &深路)).and_then(|dv| wire::as_f32_grid(&dv))
        }) { Some(v) => v, None => continue };
        // 🔴 **单位要对齐**:表的前两行是"画幅比例/米",而相机的主点焦距用**像素**。
        // 喂错的代价(C1 实测):焦距解成 **0.4**(真值几百)、钳口算成 **65 米**、
        // 圈物体一个点都圈不出来("点太少(0)")。位置反而基本对(相机 y/z 解出 −0.409/1.296,
        // 而机体自报的是 −0.41/1.308)—— 一半对一半错最难看出来,必须在这儿写死单位。
        let m像素 = [[m[0][0]*dw as f64, m[0][1]*dw as f64, m[0][2]*dw as f64],
                     [m[1][0]*dh as f64, m[1][1]*dh as f64, m[1][2]*dh as f64],
                     m[2]];
        // 🔴 用**末端**的位置配**手指**的像素,解出来的相机会整体偏掉一个指尖偏移;
        // 但那个偏移在"末端目标 = 指尖目标 − 指尖相对末端的偏置"这一步**会抵消**
        // (仓里那条几何抵消:纯平移时指尖随末端刚性平移)⇒ 下手点不受影响。
        // 🔴🔴 **先试联合解:一台【没吸偏置】的相机 + 一个显式的指尖偏置。**
        // 只在没解出来过时试一次;解出来就一直用(身体不变,这两个量是耐久的)。
        // ⚠️ 两者**必须成对用** —— 拿联合解的相机配 `d偏=0`,或拿雅可比相机配显式偏置,
        //   都是**双重修正/漏修正**,病相是手稳定地偏一个固定量,而每个标量都正常。
        let 眼 = match 眼稳 {
            Some(e) => e,
            None => {
                let Some(e) = point_gen::eye_from_jacobian(m像素, u * dw as f64, v * dh as f64, d, 此位) else {
                    println!("[服]   从表解相机没解出来(自检没通过)⇒ 重量一次");
                    *雅载 = None; continue;
                };
                println!("[服]   🟢 相机从表里解出来了:焦距 {:.1}/{:.1} · 主点 ({:.1},{:.1}) · 相机在 ({:.3},{:.3},{:.3}) —— **留着用,不每拍重解**",
                    e.fx, e.fy, e.cx, e.cy, e.at[0], e.at[1], e.at[2]);
                眼稳 = Some(e);
                e
            }
        };

        // 🔴🔴🔴 **指尖相对末端的偏移:只有【转手腕】才量得出来。**
        //
        // 纯平移时它和相机位置**在数学上分不开**(把相机整体挪一段 = 把指尖挪一段,像素一模一样;
        // 这条已经写成测试 `姿态不变时偏置解不出来_必须拒绝`)。所以我用末端的位置配手指的像素
        // 解出来的相机,是**整体偏了一个指尖偏移**的那一台 —— 拿它反投影手指,必然得回末端本身,
        // 偏移恒为 0。实测代价(C2):`手腕摆不出来(量到的两根轴共线)` ——
        // 接触集把 306 条候选都算好了,却因为**零向量没有方向**而摆不出手腕。
        //
        // 而它**有办法量**:手腕一转,指尖就会绕着末端甩。设偏移在腕系里是 t,
        // 转之前姿态 R₁、转之后 R₂,用那台(偏了的)相机反投影两次:
        // **反投影(转后) − 末端 = (R₂ − R₁)·t** —— 相机那份偏移在减法里抵消掉了。
        // 三个方程三个未知数,**转一次就够**。这是机器人量自己,不是我给它写动作。
        let 转出 = |v: [f64; 3], q: [f64; 4]| -> [f64; 3] {
            let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
            [
                (1.0-2.0*(y*y+z*z))*v[0] + 2.0*(x*y-z*w)*v[1] + 2.0*(x*z+y*w)*v[2],
                2.0*(x*y+z*w)*v[0] + (1.0-2.0*(x*x+z*z))*v[1] + 2.0*(y*z-x*w)*v[2],
                2.0*(x*z-y*w)*v[0] + 2.0*(y*z+x*w)*v[1] + (1.0-2.0*(x*x+y*y))*v[2],
            ]
        };
        // 🔴🔴🔴 **"指尖相对末端的偏移"已删 —— 最终版不需要它。**
        //
        // 它是**老路**上的东西:那条路要先算"末端该去哪"= 指尖目标 − 这个偏移。
        // 而现在**直接跟两个接触面**,面在画面哪儿就是哪儿,中间不经过"末端"这个概念,
        // 自然也就不需要"从末端到指尖有多远"。它同时是个**身体词**(假设有腕、有一根工具轴),
        // 五指手 / 吸盘 / 电锯臂都没有。
        // 实测代价(F3):它还在那儿的时候,量它要绕轴转手腕,**转完手指就被腕部挡住认不出来**,
        // 整条链卡死在这一步 —— 一个已经不需要的量,把能跑的路堵住了。
        let 尖now = 此位;
        // 钳口能张多开:那一块占画幅多宽 × 那个深度上一个像素合多少米。**这一刻看出来的。**
        // 🔴🔴🔴 **"钳口能张多开"这个标量已经撤回(owner 2026-08-27)。**
        //
        // 它是**两指爪的词** —— 五指手、吸盘、电锯根本没有"张开量"这个量。
        // 我为了让这一炮夹起来,先后用过三种估法(外接框对角线 / 沿配对方向投影 / 全程减半程),
        // **每一种都更准一点,但每一种都只在两指平行爪上成立** ⇒ 这一炮夹起来了,真机照样夹不起来。
        // 通用形不是"把这个标量量准",而是**根本不要这个标量**:
        // 接触集说"碰这两个点",驱动去问**我的接触面能不能分别落到这两个点上** ——
        // 两指是两个面、五指是五个面、吸盘是一个面加一个法向,同一句话。
        // ⇒ 这里暂时用**接触集要的下手宽**当尺度占位,并在日志里标明它是占位、不是测量。
        // 🔴🔴🔴 **"我的两个接触面最远能分开多少" —— 张到底量一次、合到底量一次,取差。**
        //
        // 这不是"钳口张开量"那个两指爪的词:**每一具身体都有"我的接触面之间能分开多远"**
        // (只有一个面时它是 0,规划器就用单点接触集;五指手是两两之间的可达间距)。
        // 而且它是**差值** —— 认块多认了半个腕子、手指本身有多宽,在减法里自己抵消。
        // 置 0 是不行的:接触集那一层拿它拒绝"比我还宽"的候选,置 0 ⇒ **一条候选都不剩**。
        // 🔴🔴🔴 **"我的接触面在抓握通道走完全程时移动了多远" = 2 ×(全程那一片 − 半程那一片)。**
        //
        // 晃**全程**时,动的那一片长 ≈ **接触面走过的全程 + 面本身的宽**;
        // 晃**半程**时 ≈ **半程 + 面宽** ⇒ **两者相减,面宽自己抵消**。
        // 它**完全不用跟踪** —— 只用"那一片有多长",而那是认块器直接给的,稳得多。
        //
        // 实测代价(F8/F9):靠模板跟踪两个面、张到底合到底各量一次,量出 **0.0002–0.0029 m**
        //(Franka 实际约 0.08),而且合到底比张到底还大 —— 又小又暗的爪子上,
        // SSD 模板跟的是**图像纹理**,手指一动它就跟丢/跟错。
        // ⚠️ 我早先做过这条,后来当成"两指爪的词"删了 —— **那是删错了理由**:
        // 它说的是"我的接触面在抓握通道走完全程时移动多远",吸盘、五指、电锯都成立。
        // 🔴🔴🔴 **接触面能张开多远:给一个【上界】,并且在日志里明说它是上界。**
        //
        // 我先后试过三种把它量准的办法,全都不稳:
        //   ① 张到底/合到底各跟一次两个面 ⇒ **0.0002–0.0029 m**(实际约 0.08),
        //      而且合到底比张到底还大 —— 又小又暗的爪子上,SSD 模板跟的是纹理,手指一动就跟丢。
        //   ② 全程减半程 ⇒ **−0.0402 m(负的)**:晃半程那一片反而更大。
        //      认块器给的区域取决于哪些像素超过噪声地板,幅度一变过闸的像素集就变,**不是单调的**。
        //   ③ 外接框对角线 ⇒ 0.187 m,把手指长度和腕子一起算了进去。
        // ⇒ **不再死磕这个数。** 用"晃全程时动的那一片有多长"换算成米,它**包含接触面自己的宽**,
        //   所以偏大 —— 而**偏大是安全的那一侧**:接触集少拒几条候选,失败会以"真的没夹住"
        //   的形式暴露出来;偏小则直接 `NoSection`(一条候选都没有),**整条链停死**。
        //   日志里标明它是上界,不许当成测量值用在别处。
        // ══════════════════════════════════════════════════════════════════════════
        // 🔴🔴🔴 **钳口能张多大:拟一条直线,不再拿"动了的那一片有多长"当上界。**
        //(owner 2026-08-28 定"打磨完美";估计器是仓里早就写好的 `probe::gripper_span`)
        //
        // 旧法量的是"晃全程时动了的那一片有多长",它**含接触面自己的宽**(只能当上界),
        // 而那一片的大小取决于哪些像素越过噪声地板 ⇒ **不单调**。实测同一台机器人量出
        // **61.5 / 17.4 / 78.7 mm**(真机 80),一炮之内量了 12 次、次次不同,而下游拿它当
        // "这段夹不夹得下"的尺 —— 剪刀那一炮 132 条候选**全部**被判"放不下"就是这么来的。
        //
        // 正解在 `LAB` 2026-08-20 破过案:*"不是估计器病,是换米尺双轨…用全相机尺的两轮
        // 0.0402/0.0389(互差 3%)…外核:0.040/单位 × 全开 1.0 = 8cm = Franka 官方 80mm ✓"*。
        // `probe::gripper_span` 正是那条:吃若干组 **(命令开度, 两面间距)** 拟直线 ——
        // **斜率 × 满开 = 张开量,截距把接触面自己的宽度吃掉**,"含面宽"那个偏差自动消失。
        //
        // 采样不额外动手臂:只命令钳口到几个开度,用已经截好的模板跟着两个面读间距。
        // 开度 [0.25, 0.5, 0.75, 1.0] 是**比例**(命令域本来就是 0..1),四点定一条线,无量纲。
        // 只在还没有值时做一次;做出来存进身体文件,下一炮装回来不再做。
        if 张开载.is_none() {
            if let (Some(t手), Some(m2)) = (跟模手.clone(), 跟模2.clone()) {
                let mut 组: Vec<(f64, f64)> = Vec::new();
                for 开 in [0.25f64, 0.5, 0.75, 1.0] {
                    抓握(plug, 开, *通道是关节);
                    定爪(plug, 等拍 * 2);
                    let Some((_, _, g)) = plug.sense().and_then(|f| 灰(&f, 跟相机)) else { continue };
                    let Some((mu, mv)) = 找块窗(跟fw, 跟fh, &g, &t手, 跟半手, 跟上手.0, 跟上手.1, 跟半手 * 2) else { continue };
                    let (du, dv) = (mu - 跟上手.0, mv - 跟上手.1);
                    let mut 位 = [(0.0f64, 0.0f64); 2];
                    let mut 全中 = true;
                    for i in 0..2 {
                        match 找块窗(跟fw, 跟fh, &g, &m2[i], 跟半, 跟上面[i].0 + du, 跟上面[i].1 + dv, 跟半 * 2) {
                            Some(p) => 位[i] = p,
                            None => { 全中 = false; break }
                        }
                    }
                    if 全中 {
                        组.push((开, ((位[0].0 - 位[1].0).powi(2) + (位[0].1 - 位[1].1).powi(2)).sqrt()));
                    }
                }
                // 🔴 顺手量**空手合到底机体报什么** —— 此刻手还没走到物体上,指间是空的,
                //    正是量这个零点唯一不花额外动作的时机。
                //    以前判"指间有没有东西"写死的是"停在 0 就是空的",而 **0 是这具 Franka 的约定**;
                //    下一步就要零改动换 ARX 双臂,那时这个约定不成立而**病相是"每次空抓都报成功"**。
                if 空合载.is_none() {
                    抓握(plug, 0.0, *通道是关节);
                    定爪(plug, 等拍 * 4);
                    if let Some(f) = plug.sense() {
                        if let Some(x) = f.jaw.get(手号.get()).or_else(|| f.jaw.first()).copied() {
                            println!("[服]   🟢 **空手合到底,这具机体报 {x:.4}** ⇒ 以后「指间有没有东西」拿它当零点");
                            *空合载 = Some(x);
                        }
                    }
                }
                抓握(plug, 1.0, *通道是关节);
                定爪(plug, 等拍);
                // 画幅 → 米的尺:焦距 ÷(那一点的深 × 画宽)。全是量出来的。
                let 每米 = 眼.fx / (dw as f64 * d.max(1e-6));
                if 组.len() >= 2 && 每米.is_finite() && 每米 > 0.0 {
                    // 0.1 是**比例**(把尺自己的不确定度取成一成),无量纲。
                    match body_layer::probe::gripper_span(&组, 每米, 每米 * 0.1, 0, 0) {
                        Ok(m) => {
                            println!("[服]   🟢 **钳口张开拟出来了**:{:.4} m({} 个开度拟一条线,截距把面宽吃掉)", m.value[0], 组.len());
                            *张开载 = Some(m.value[0]);
                        }
                        Err(e) => println!("[服]   钳口张开拟不出来:{e:?}(采到 {} 组)⇒ 照旧用上界,结果照实报", 组.len()),
                    }
                } else {
                    println!("[服]   钳口张开:采样不够({} 组)或米尺不可用 ⇒ 照旧用上界", 组.len());
                }
            }
        }
        // 拟出来了就用拟的;没有才退回那个"含面宽"的上界(并且日志里已经写明它是上界)。
        let 张开 = 张开载.unwrap_or(张 * dw as f64 * d / 眼.fx.max(1e-9));
        println!("[服]   接触面能张开多远:**上界** {张开:.4} m(晃全程那一片 {张:.4} 画幅换算,含面自己的宽)");
        if !(张开 > 1e-4 && 张开 < 可达带[1]) {
            println!("[服]   这个上界不合理 ⇒ 这一拍不下手"); continue;
        }
        // 开合方向(世界):那一块两瓣往哪边分开,是画面上的方向;用表把它翻成世界方向。
        // 开合方向(世界):那一块两瓣往哪边分开,是**画面上的方向**;用表把它翻成世界方向。
        // 表是线性的 ⇒ 喂什么长度都得到同一条方向,**归一化之后长度就无关**,不需要任何尺度常数。
        let 开合世 = match 解3(m, [开合像.0, 开合像.1, 0.0]) {
            Some(p) => {
                let n = (p[0].powi(2)+p[1].powi(2)+p[2].powi(2)).sqrt();
                if !(n > 1e-12) { continue }
                [p[0]/n, p[1]/n, p[2]/n]
            }
            None => continue,
        };
        // 🔴 "离末端多远"这一栏已删:最终形态在画面里比误差,根本不经过"末端"这个概念。
        println!("[服]   接触面在 ({:.3},{:.3},{:.3}) · 钳口能张 {:.4} m",
            尖now[0], 尖now[1], 尖now[2], 张开);

        // 🔴🔴 **接触集那一层决定这一把怎么下手 —— 不是我。**
        // 它吃点云,给出:碰哪几点、从哪个方向下手、下手多宽、指尖该去哪、手腕该摆成什么朝向。
        // 我上一版把它整个绕过去了,于是"怎么抓"变成了我手写的策略("把指尖像素挪到物体像素上、
        // 把两根手指合上")—— 那句话里有"指尖""两根手指",**是身体词**,换五指手/吸盘就说不出口。
        let 深f: Vec<f64> = dep.iter().map(|z| *z as f64).collect();
        let r = task::尺 { 张开, 可达内: 可达带[0], 可达: 可达带[1] };
        // 🔴 一拍之内最多换几条候选 —— **由接触集自己给的条数定**,不是我拍的:
        //    它交出多少条,就最多换多少次;实际上前几条就会有一条姿态够得着。
        //    这里封顶用"上一拍它给了多少条"的粗略值,封顶只是防死循环,不参与选择。
        let (候选, 航点, 宽, 尖目标, q目标, _桌面, 接触点) = match task::算一把(
            &眼, &深f, dw, dh, [look.u * dw as f64, look.v * dh as f64],
// 0.5 是摩擦系数 μ、1.5 是圈物体那道闸的宽容倍数 —— 两个都是**无量纲**的比值。
            look.span_frac, Some(&rgb), 0.5, 1.5, 此位, &试过, &r, 选中掩膜.as_deref()) {
            Ok(v) => v,
            Err(x) => { println!("[服] 🔴 算不出这一把:{x:?}"); 试过.push(此位); continue }
        };
        println!("[服] 🟢 这一把:候选 {候选} · 航点 {航点} · 下手宽 {:.1} mm · 指尖去 ({:.3},{:.3},{:.3})",
// 1000 是**米→毫米的显示换算**,只进日志,不驱动任何动作。
            宽 * 1000.0, 尖目标[0], 尖目标[1], 尖目标[2]);

        // 🔴 指尖此刻在世界的哪儿 —— 块里算出来,块外要用(认不出接触面时的退路)。
        let mut 尖此刻: Option<[f64; 3]> = None;
        // ══════════════════════════════════════════════════════════════════════════
        // 🔴🔴🔴 **压到压不动,再合。**
        //
        // 这一段替掉了"跟住我的两个接触面、拿通道表把它们伺服到接触点上"那条路。
        // 那条路在这台机体的头部相机里**物理上走不通**:两根手指只隔十来个像素、长得一模一样,
        // 跟踪必然锁错(渲图看过,不是推测)。而**它并不是抓起来的必要条件**。
        //
        // 这一段只用三样东西,每一样都已经量到并且验过:
        //   ① 接触点在哪 —— 相机是从画面雅可比闭式解出来的,而且和机体自报的对得上;
        //   ② 我的指尖在哪 —— 只动抓握通道、画面里跟着动的那一块,反投成三维(构造性身份);
        //   ③ **压不动了** —— `落` 自己会说:目标按住不放而残差不再缩。
        //
        // 关键是第三条:**高度/深度的误差不用算准,让接触本身把它吸收掉。**
        // 所以目标不是"停在接触点上",而是**沿进场方向再多压一个钳口张开量** ——
        // 够不到就压到底,够到了就被东西挡住停下,两种情况都停在**贴着物体**的地方。
        //
        // 这句话里没有身体词也没有任务词:任何有"能命令的自由度 + 能报回来的位姿"的机体,
        // 压到压不动都成立;扣扳机、按按钮、电锯贴上去,是同一段代码。
        {
            // 🔴🔴 **`back_project` 收像素,不收归一化坐标。**
            // 实测代价(W2,2026-08-27):传了归一化的 (0.295,0.275) ⇒ 内部算的是 (0.295−331.4)/257.1
            // ⇒ 指尖被反投到 **0.66 m 开外** ⇒ "要走 0.9 m"(桌面场景里指尖到物体不可能这么远)
            // ⇒ 手臂要么走了 0.438 m 也没碰到,要么**一动不动**(大跨度命令被拒),连着合了几把空。
            // 这一处**不是今晚新写的**,驱动里一直是这么传的。
            // 🔴🔴🔴 **指尖不是那一块的形心 —— 形心在指身中段,而手指还在它前面。**
            //
            // 实测(W2,2026-08-27,画到图上才看出来):方框(我以为的指尖)落在爪子上没错,
            // 但它和**法兰**几乎重合 ⇒ "指尖偏移"算出来是 **0** ⇒ 我把法兰开到物体上,
            // 而手指还在法兰前面十厘米 ⇒ **整只爪子越过物体**,合下去当然是空的。
            //
            // 仓里早写过这一条(`hand.rs` N45):*形心在指身中段 ⇒ 指尖越过物体,合爪咬边挤出;
            // 指尖端 = 包围盒沿下降图像方向那条边*。⇒ 取那一块**沿前进方向最靠前**的那条边。
            // 前进方向 = 从爪子指向接触点(量出来的),包围盒 = 认块器给的 `ext`(量出来的)。
            尖此刻 = {
                let 心 = 眼.back_project([u * dw as f64, v * dh as f64], d).ok().map(|p| [p.x, p.y, p.z]);
                match 心 {
                    None => None,
                    Some(c3) => {
                        // 那一块在画面上有多大(半宽,画幅) ⇒ 换成这个深度上的米
                        // 🔴 用**认块器给的包围盒** `ext`,不再拿对角线估一个半宽(那是我自己算的,
                        //    而 `blob::candidates` 本来就把 `ext` 交出来了 —— 见 hand.rs N45 那条注释)。
                        //    沿前进方向那条边距形心多远,就是形心到指尖端的距离。
                        let 朝 = [尖目标[0] - c3[0], 尖目标[1] - c3[1], 尖目标[2] - c3[2]];
                        let 模朝 = (朝[0].powi(2) + 朝[1].powi(2) + 朝[2].powi(2)).sqrt();
                        // 前进方向投到画面上:表的前三列就是"末端往这三个方向挪 ⇒ 画面跑多少"
                        let 朝像 = (m[0][0]*朝[0] + m[0][1]*朝[1] + m[0][2]*朝[2],
                                    m[1][0]*朝[0] + m[1][1]*朝[1] + m[1][2]*朝[2]);
                        let 模像 = (朝像.0*朝像.0 + 朝像.1*朝像.1).sqrt();
                        let 半框 = if 模像 > 1e-12 {
                            let (ax, ay) = (朝像.0 / 模像, 朝像.1 / 模像);
                            let hu = (爪框[2] - 爪框[0]) * 0.5;
                            let hv = (爪框[3] - 爪框[1]) * 0.5;
                            (ax.abs() * hu + ay.abs() * hv).abs()
                        } else { 块 * 0.5 };
                        let 半米 = 半框 * dw as f64 * d / 眼.fx.max(1e-9);
                        if 模朝 > 1e-9 && 半米.is_finite() {
                            println!("[服]   指尖 = 爪子包围盒沿前进方向那条边(离形心 {半米:.4} m,包围盒来自认块器)");
                            Some([c3[0] + 朝[0] / 模朝 * 半米,
                                  c3[1] + 朝[1] / 模朝 * 半米,
                                  c3[2] + 朝[2] / 模朝 * 半米])
                        } else { Some(c3) }
                    }
                }
            };
            // 🔴 **把"我打算去哪"画回图上存一张** —— 十字落在哪个物体上,一眼就知道瞄没瞄对。
            // 算得再对,落在错的东西上也是白搭;而这件事只有图能答。
            if let Ok(存路) = std::env::var("BL_VID") {
                if let Some((gw, gh, mut 图)) = plug.sense().and_then(|f| 灰(&f, 相机号)) {
                    let mut 点 = |x: i64, y: i64, v: u8| {
                        if x >= 0 && y >= 0 && (x as usize) < gw && (y as usize) < gh { 图[y as usize * gw + x as usize] = v; }
                    };
                    // 眼指的那一点(白十字)
                    let (ex, ey) = ((look.u * gw as f64) as i64, (look.v * gh as f64) as i64);
                    for k in -12i64..=12 { 点(ex + k, ey, 255); 点(ex, ey + k, 255); }
                    // 🔴 **"我以为我的指尖在这儿"—— 画一个方框。** 它落在手指上,三个量才算齐。
                    // 落在别处 = 一直夹空的原因,而且是一次能修完的错(owner 2026-08-27)。
                    if let Some(t) = 尖此刻 {
                        if let Some(px) = 眼.project(point_gen::P3 { x: t[0], y: t[1], z: t[2] }) {
                            let (sx, sy) = (px[0] as i64, px[1] as i64);
                            for k in -14i64..=14 {
                                点(sx + k, sy - 14, 255); 点(sx + k, sy + 14, 255);
                                点(sx - 14, sy + k, 255); 点(sx + 14, sy + k, 255);
                                点(sx + k, sy - 13, 0);   点(sx + k, sy + 13, 0);
                                点(sx - 13, sy + k, 0);   点(sx + 13, sy + k, 0);
                            }
                        }
                    }
                    // 我打算去的那一点(黑十字)
                    if let Some(px) = 眼.project(point_gen::P3 { x: 尖目标[0], y: 尖目标[1], z: 尖目标[2] }) {
                        let (tx, ty) = (px[0] as i64, px[1] as i64);
                        for k in -12i64..=12 { 点(tx + k, ty, 0); 点(tx, ty + k, 0); }
                        for k in -12i64..=12 { 点(tx + k, ty + 1, 0); 点(tx + 1, ty + k, 0); }
                    }
                    let n = unsafe { static mut A: u32 = 0; A += 1; A };
                    let mut buf = format!("P5\n{gw} {gh}\n255\n").into_bytes();
                    buf.extend_from_slice(&图);
                    let _ = std::fs::write(format!("{存路}/aim{:03}.pgm", n), buf);
                }
            }
            // 🔴🔴🔴 **这里【曾经】有我 2026-08-28 写的一整套画面伺服 —— 已整段删除。**
            //
            // 删的原因不是它错,是**它是重复的**:下面本来就躺着一套更完整的最终形
            //(六个方程解朝向 · 追的路上边干边修表 · 碰到东西就地合 · 再交给腕相机做最后一两厘米),
            // 而它**一次都没被执行过** —— 日志里「追」这个字出现 **0 次**(WQ 实证)。
            // 我在它上面又写了一套更差的,这是本仓第 6 次重造轮子,也是最贵的一次
            //(WE–WQ 十来炮全耗在上面)。⇒ 删掉重复的那一套,让控制流落到本来就该走的那一段。
            // ⚠️ 教训与仓规第一条同源:**动手前先把文件读完**。今天我一路推导出的
            //    每一条结论(最终形态 / 观测太贵 / 跟踪锁错),下面 200 行里都已经写着。
        }
        // ══════════════════════════════════════════════════════════════════════════

        // 🔴🔴🔴 **最终形:把【我的两个接触面】分别送到【接触集要的两个点】上。**
        //
        // 上一版要先算一个**手腕四元数**,而那需要知道"我的工具轴是哪根、开合轴是哪根" ——
        // **那两根都是身体词**:五指手、吸盘、电锯臂根本没有。凡是为了让这一炮过而加的东西,
        // 对 LeKiwi / 无人机都是零(owner 2026-08-27)。
        //
        // 通用形只有一句:**接触集说碰哪几个点,我就把我的接触面分别送到那几个点上。**
        // 姿态是这么**长出来**的,不是被指定的 —— 两个面各就各位,朝向自然就对了。
        // 两指 = 两个面、五指 = 五个面、吸盘 = 一个面加一个法向,**同一段代码**。
        //
        // 接触面从哪来:**只动抓握那个通道,画面里跟着动的那一块**(构造性身份)。
        // 那一块沿开合那一维的两端就是两个面 —— 不需要"第几列是开合轴"这种约定。
        let (fw, fh, g0) = match plug.sense().and_then(|f| 灰(&f, 相机号)) { Some(v) => v, None => continue };
        // 开合那一维: 已经量出来了(两瓣往哪边分开 / 那一块的长边),不需要任何约定。
        let 长轴 = 开合像;
        // 🔴🔴🔴 **模板必须从【单帧】上切 —— 从"扫过的并集"上切,切到的是背景。**
        //
        // 认块器给的那一片是手指**在整个晃动过程中扫过的并集**,不是任何一个瞬间手指所在的位置。
        // 我在这一片的两端切模板 ⇒ **两端在任何单帧里都主要是背景** ⇒ 拿背景当模板去跟手指,
        // 当然跟不住。实测(F7–F9):量出"接触面最远分开 0.0002–0.0029 m"、合到底比张到底还大。
        // ⚠️ 我曾把这归因于"爪子又小又暗" —— **那是推卸责任,而且是错的**:
        // 换一具更小更暗的手(LeKiwi)会因为**同一个错误**一样失败,与外观无关。
        //
        // 正确做法:手臂冻住时,画面里**唯一会动的就是抓握通道带动的那些面**。
        // 张到底拍一帧、合到底拍一帧,**两帧相减** ⇒ 变了的像素就是那些面;
        // 在**张到底那一帧**上、沿变化区域的长轴取两端切模板 —— 切到的是**真的接触面**。
        // 这条对任何机体成立,而且比晃五帧更省。
        // 认不出接触面时走的那条"搬过去"退路 —— 走完就直接合,不再跑需要跟住手指的追。
        let mut 已就位 = false;
        // 🔴🔴🔴 **"这台相机看不见我的钳口张合"是一个【量出来的身体事实】,量到了就别再花那笔钱。**
        //
        // 实测代价(XL,2026-08-28,把一集 200 拍逐条记下来才看见):
        //   第一次看爪失败 + 挪进画面 ~60 拍 · 第二次看爪(成功)~50 拍 ·
        //   **认接触面每一拍都跑、每一拍都失败 ~40 拍** · 眼和算方案 ~10 拍
        //   ⇒ 轮到"朝球走"时**一拍都不剩**,日志原话 `搬到第 0/127 步换集了`。
        // 而它失败的原因是物理的、不会自己变好(张合在这台相机里只让 0–25 个像素变了)。
        // ⇒ 连着失败到第三次就记住:**这具身体在这台相机里认不出接触面**,
        //   以后直接走"搬过去",把那 40 拍全花在走路上。
        //   这不是放弃测量,是**把一次失败的测量变成一条知识** —— 换台相机 / 换只手,
        //   它会重新失败或重新成功,记忆跟着变。
        let 面点 = if 认面失败 >= 3 {
            println!("[服]   (已经量到:这台相机看不见我的钳口张合 —— 连着失败 {认面失败} 次)⇒ 直接走【搬过去】,把拍数留给走路");
            None
        } else {
            let r = 认接触面(plug, *通道是关节);
            if r.is_none() {
                认面失败 += 1;
                if 认面失败 >= 3 && !*认面载 {
                    *认面载 = true;
                    println!("[服]   💾 记住这条身体事实:**这台相机看不见我的钳口张合**(连着 {认面失败} 次)⇒ 存进身体文件,下一炮开机就跳过");
                }
            } else { 认面失败 = 0; *认面载 = false }
            r
        };
        let 面点 = match 面点 {
            Some(v) => v,
            None => {
                // 🔴🔴🔴 **认不出接触面 ⇒ 走【搬过去】那条路,不是站着,也不是再打一个补丁。**
                //
                // 病是物理的,不是算法的:XE 实测张合在头部相机里**只让 25 个像素变了**(门槛 32),
                // 两根手指在这台相机里只有十来个像素宽。仓里早写明这堵墙
                //(*"两根手指只隔十来个像素、长得一模一样,跟踪必然锁错(渲图看过,不是推测)"*),
                // 而我今天已经在同一堵墙上打过三个补丁(排除半径 / 刚体间距 / 挪进画面)。
                //
                // ⇒ 换一条**不需要在画面里跟住手指**的路。它要的两个量**此刻都已经量到了**:
                //   · 指尖现在在世界的哪儿 —— `尖此刻`(晃爪子认出那一块,反投影,再沿前进方向
                //     推到包围盒边;上面刚打印过 `指尖 = 爪子包围盒沿前进方向那条边`);
                //   · 指尖该去哪 —— `尖目标`(接触集算出来的,上面刚打印过 `指尖去 (…)`)。
                // 两者相减就是要走的位移。**一步一步走,每步问一次"被挡住没有"**;
                // 挡住了就地合(挡我的多半就是要抓的东西),走完了也合。
                // 这条路没有视觉伺服,所以它**吃标定误差**;但它会真的把手送到球上、会合、会留下视频,
                // 而"认不出接触面 ⇒ continue"一个动作都不会做。**先有第一次抓起来,再谈精度。**
                白转 = 白转.wrapping_add(1);
                let Some(尖0) = 尖此刻 else {
                    println!("[服]   认不出接触面,而指尖此刻在哪也没量到 ⇒ 挪一步让相机多看见我一点");
                    let _ = 挪进画面(plug, 白转, jaw0, *通道是关节);
                    continue
                };
                // 🔴🔴🔴 **两段交接:先粗对准把球送进腕相机的视野,再交给腕相机收尾。**
                //
                // 这不是新架构 —— `LAB "不是「头相机 vs 腕相机」二选一,是【两段交接】"`
                //(2026-08-15)就定死了:*头相机做粗对准(把物体送进腕相机视野),
                // 腕相机做末段与抓取*。
                //
                // 🔴 **我 08-28 把三维搬运整段换成了下面那条画面闭环 —— 理由对,标准用错了。**
                // 理由:*"它走到了它算出来的那个点,而那个点不是球"*(偏差,迭代磨不掉)。
                // 但三维搬运在两段架构里的活**只是把球送进腕相机的视野**,厘米级就够
                //(`LAB "搬运 0.5522 m → 0.0493 m"`:五十厘米真的走完了)。
                // 我拿**毫米级**的标准砍掉一个只需要**厘米级**的零件 ⇒ 两段被压成一段,
                // 而且压在**看不见爪子的那台相机**上 —— YF 那 344 次"看不清爪子"就是它。
                //
                // ⚠️ 这一段全程**不换手腕朝向**(每步把身体自己报的四元数原样发回去),
                //    所以"指尖相对末端那一截"在整段里是常数、两边相减抵消 ——
                //    `LAB "d偏 = 0"` 那条警告(*抵消只在朝向不变时成立*)在这里是被满足的,
                //    不是被忽略的。
                // 交接判据不是我拍的距离,是 `收尾腕` 自己那三条会失败的检查:
                // 张合认不出两瓣 / 手一动分不出世界 / **那一撮比我自己的爪子还大**。
                if let Some(腕机) = 手上相机 {
                    let 走 = [尖目标[0] - 尖0[0], 尖目标[1] - 尖0[1], 尖目标[2] - 尖0[2]];
                    let 总长 = (走[0] * 走[0] + 走[1] * 走[1] + 走[2] * 走[2]).sqrt();
                    if let Some(e0) = plug.sense().and_then(|f| f.ee.get(手号.get()).copied()) {
                        let 末目标 = [e0[0] + 走[0], e0[1] + 走[1], e0[2] + 走[2]];
                        // 🔴🔴🔴 **这里【曾经】有一道"走之前先比可达"的闸,已撤回 —— 它比错了量。**
                        //(ZE 实测 2026-08-29,加上去不到一小时就自己打脸)
                        //
                        // 我写的是 `if 总长 > 可达带[1]`,拿**要走的距离**去比**外壁半径**。
                        // 而仓里自己的注释就写着:*"可达带说的是【离基座的半径】能变多少
                        //(这具身体 6.5 cm)"* —— 一个是**位移**,一个是**半径**,
                        // **单位对得上,意思对不上**(`LAB` 记过的同一类病:*"像素塞进米的槽"*)。
                        // 手从半径 0.10 走到 0.35,直线走了 0.45 m,合法;而那道闸会把它拦成"够不着"。
                        // ZE 实测被它拦掉一次:`要走 0.5056 m,可达上限 0.3675 m`。
                        //
                        // ⇒ **"够不着"这件事不许算,只许走走看。** 下面那道
                        //   "连着三步一动没动 ⇒ 这个位形过不去"才是量出来的判据。
                        // 上限只防死循环:走完全程要几步,由量出来的一步长 × 交付率给。
                        let 粗上限 = ((总长 / (探步 * 交付率).max(1e-9)) * 2.0).ceil().clamp(1.0, 4000.0) as u32;
                        println!("[服]   🟢 **两段交接①粗对准**:指尖 ({:.3},{:.3},{:.3}) → 目标 ({:.3},{:.3},{:.3}) · 差 {总长:.4} m · 最多 {粗上限} 步",
                            尖0[0], 尖0[1], 尖0[2], 尖目标[0], 尖目标[1], 尖目标[2]);
                        let mut 停原地 = 0u32;
                        let mut 最好余 = f64::INFINITY;
                        for k in 0..粗上限 {
                            if plug.复位过 { println!("[服]      ①粗对准第 {k} 步换集了"); break }
                            let Some(e) = plug.sense().and_then(|f| f.ee.get(手号.get()).copied()) else { break };
                            let 余 = [末目标[0] - e[0], 末目标[1] - e[1], 末目标[2] - e[2]];
                            let 余长 = (余[0] * 余[0] + 余[1] * 余[1] + 余[2] * 余[2]).sqrt();
                            if 余长 < 探步 { println!("[服]      ①走完了(还差 {余长:.4} m)"); break }
                            let 比 = 探步 / 余长;
                            let Some((_, 挡, 走了)) = 落(plug, [e[0] + 余[0] * 比, e[1] + 余[1] * 比, e[2] + 余[2] * 比],
                                [e[3], e[4], e[5], e[6]], jaw0, 等拍) else { println!("[服]      ①命令发不出去 ⇒ 停"); break };
                            if k % 20 == 0 { println!("[服]      ①第 {k}/{粗上限} 步:还差 {余长:.4} m"); }
                            // 0.3 是**比例**(至少走了三成的一步才算真挡住,不是一发就被拒),无量纲。
                            if 挡 && 走了 >= 探步 * 0.3 { println!("[服]      ①被挡住了(这一步走了 {走了:.4} m)⇒ 就此交接"); break }
                            // 🔴🔴🔴 **要看的是"还差多少在不在缩",不是"身体动没动"。**(ZH 实测 2026-08-30)
                            //
                            // 我上一版写的是 `if 走了 < 1e-5`,而 ZH 里**每一步都动了一点点、
                            // 就是不往目标去**(顶住某个约束在旁边蹭)⇒ 闸不响,`还差 0.3580 m`
                            // 连着 **100 步**一个数字都没变。仓里那句原话是
                            // 「停手看**碰上了**或**误差不再缩**」—— 我只接了前半句。
                            //
                            // 门槛由量出来的东西给:一步**应当**缩掉 `探步 × 交付率`;
                            // 连着几步连它的十分之一都缩不掉,就是过不去。0.1 和 5 是无量纲的余量/计数。
                            if 余长 < 最好余 - 探步 * 交付率 * 0.1 {
                                最好余 = 余长; 停原地 = 0;
                            } else {
                                停原地 += 1;
                                if 停原地 >= 5 {
                                    println!("[服]      🔴 ①连着 {停原地} 步**还差多少不再缩**(卡在 {余长:.4} m,一步本该缩 {:.4} m)⇒ **这个位形过不去**,不再空推", 探步 * 交付率);
                                    break
                                }
                            }
                        }
                        println!("[服]   🟢 **两段交接②交给第 {腕机} 台(长在手上那台)收尾**");
                        if 收尾腕(plug, 腕机, jaw0, *通道是关节, &look, &mut 位).is_some() {
                            println!("[服]   🟢 两段交接走通了 ⇒ 交给下面同一段合爪");
                            已就位 = true;
                        } else {
                            println!("[服]   ⇒ 第 {腕机} 台那一段没走通(理由在上面)⇒ 退回【画面里闭环】");
                        }
                    }
                }
                if 已就位 { [(u, v), (u, v)] } else {
                // 🔴🔴🔴 **在【画面里】闭环,不在三维里。**(XS 实测,2026-08-28)
                //
                // 上一版在三维里闭环:`要走的 = 目标三维 − 我以为我的指尖在哪`。它走到了
                // `🟢 走到了(第 7 步,还剩 0.0048 m)`、`🟢 到位了(还差 0.0045 m)⇒ 合`,
                // 然后 `🔴 指间没东西`。**它走到了它算出来的那个点,而那个点不是球。**
                //
                // 病因那张瞄准图早就画出来了:**十字(目标)在球上,方框(我以为我的手在这儿)
                // 在空地板上**。位移 = 目标 − 错的自己 ⇒ 落点整体偏掉。
                // 🔴 **而闭环救不了它,因为这是【偏差】不是【噪声】** —— 每一轮用同一个有偏的
                // 方法重测自己,得到同一个偏差,收敛到同一个错地方。这一条值得记住:
                // **迭代只能磨掉噪声,磨不掉偏差;偏差要靠换一个【它会抵消】的量来闭环。**
                //
                // ⇒ 换成画面闭环:让**被跟踪的那块手**在画面上朝**目标那个像素**走,
                //   误差 (Δ横, Δ纵, Δ深) 用**通道表**解成命令。自己在哪的偏差在这里自动抵消
                //   (跟的和量的是同一块东西),而且**不需要相机模型、不需要指尖三维、
                //   也不需要把两根手指分开** —— 一个点三个方程就够。
                //   这就是仓里那条 `追`,只是单点版;`单面` 那条分支本来就在。
                let Some(表) = 通道表.clone() else {
                    println!("[服]      还没有通道表 ⇒ 这一拍先去建表");
                    continue
                };
                // 目标:眼指那个像素 + 那一点的近侧深度。我:看爪认出的那块 + 它的深度。
                let (目u, 目v, 目深) = (look.u, look.v, d星);
                let 深换 = 眼.fx / (dw as f64 * d.max(1e-6));
                println!("[服]   🟡 认不出接触面(张合在这台相机里看不见)⇒ **改走【画面里闭环】**:");
                println!("[服]      我这块在画面 ({u:.3},{v:.3}) 深 {d:.3} · 目标在 ({目u:.3},{目v:.3}) 深 {目深:.3}");
                let 上限 = ((可达带[1] / (探幅 * 交付率).max(1e-9)) * 4.0).ceil().clamp(1.0, 4000.0) as u32;
                let mut 碰上了 = false;
                let mut 没动 = 0u32;
                let mut 减 = 0u32;
                let mut 被切走 = false;
                let mut 上模: Option<Vec<u8>> = None;
                let mut 上处 = (u, v);
                // 🔴 论文式 15 的 `p_self`:一个**长在身上、跟着身体跑**的点(归一化画幅坐标)。
                let mut 跟点: Option<(f64, f64)> = None;
                let 半跟 = ((块 * dw as f64 * 0.5) as usize).clamp(3, 40);
                for k in 0..上限 {
                    if plug.复位过 { println!("[服]      🔴 搬到第 {k} 步换集了"); 被切走 = true; break }
                    let Some((gw, gh, g)) = plug.sense().and_then(|f| 灰(&f, 相机号)) else { println!("[服]      读不到帧 ⇒ 停"); break };
                    // 第一拍切一块模板(就切在看爪认出的那块上);之后靠它跟。
                    if 上模.is_none() { 上模 = 截块(gw, gh, &g, u, v, 半跟); }
                    let Some(模跟) = 上模.clone() else { println!("[服]      切不出跟踪模板(靠画面边了)⇒ 停"); break };
                    // 🔴🔴🔴 **"我"用【看爪】给的那个点。**(ZJ 渲图定案 2026-08-30,owner 令)
                    //
                    // ZJ 的 `track000_0030.pgm`:十字(目标)正在棒球旁边 ✅,而**方框(它认为我在这儿)
                    // 落在肩膀上** ⇒ 它**把肩膀往球上顶**,横纵差从 0.108 一路涨到 0.244
                    // (不是收敛慢,是在往反方向开)。
                    // `LAB "认手不许靠「哪块动得最像我」"` 给的唯一站得住的办法是
                    // **只命令钳口 —— 手臂不动时肘部位移恰好是零**,那正是 `看爪`。
                    //
                    // 之后每帧靠**模板跟踪**跟住它(自图那一套已整个摘掉,见上面头注)。
                    // 窗口中心是上一拍的位置、半径三倍模板 —— 一拍走不了那么远,够得着且锁不到别处。
                    if 跟点.is_none() {
                        跟点 = Some((u, v));
                        println!("[服]      🟢 「我」用**看爪认出的两瓣**:({u:.3},{v:.3})");
                    } else if let Some(p) = 跟点 {
                        match 找块窗(gw, gh, &g, &模跟, 半跟, p.0, p.1, 半跟 * 3) {
                            Some(q) => 跟点 = Some(q),
                            None => println!("[服]      跟丢了一帧(模板在上一拍附近匹配不上)⇒ 按上一拍的位置走"),
                        }
                    }
                    let (现u, 现v) = 跟点.unwrap_or(上处);
                    上处 = (现u, 现v);
                    let 现深 = 近侧深(plug, 现u, 现v, 块 * 0.5).unwrap_or(d);
                    let 误 = vec![目u - 现u, 目v - 现v, (目深 - 现深) * 深换];
                    let 差像 = (误[0] * 误[0] + 误[1] * 误[1]).sqrt();
                    // 到位:画面上比我自己那一块还近(块是量出来的,不是填的阈值)。
                    if 差像 <= 块 * 0.5 && (目深 - 现深).abs() <= 块 / 深换.max(1e-9) {
                        println!("[服]      🟢 画面上到位了(第 {k} 步:横纵差 {差像:.4} 画幅 ≤ {:.4} · 深差 {:+.4} m)", 块 * 0.5, 目深 - 现深);
                        碰上了 = true; break
                    }
                    // 方向盘 = 接触面那张表(自图那条已摘掉:270 次算、0 次被采用)。
                    // 三行的单位统一换成**画幅**:横纵本来就是,深度那一行乘 `深换`。
                    let 行: Vec<Vec<f64>> = (0..3).map(|r| 表.iter()
                        .map(|c| if r == 2 { c[r] * 深换 } else { c[r] }).collect()).collect();
                    let 误3 = vec![误[0], 误[1], 误[2]];
                    let Some(动0) = 最小二乘(&行, &误3) else { println!("[服]      解不出该动多少 ⇒ 停"); break };
                    let 步长 = 动0.iter().map(|x| x * x).sum::<f64>().sqrt();
                    if !(步长 > 1e-12) { println!("[服]      解出来是零 ⇒ 停"); break }
                    let 幅 = 探幅 / (1u32 << 减.min(6)) as f64;
                    let 比 = (幅 / 步长).min(1.0);
                    // 🔴🔴 **前帧要在【发命令的前一瞬间】拍,不能用循环开头那一帧。**
                    //
                    // 实测代价(YA,2026-08-29):动得最多的那一格(4.43 px,多半就是胳膊)
                    // 卡在"动过"这一关 —— 它的运动**解释不掉**(残差比位移还大)。
                    // 原因是循环开头拍完 `g` 之后,中间那几行(读深度、找块、解方程)每次
                    // `sense()` 都会推进仿真,而胳膊还在按**上一条**命令继续走
                    //(这具身体一拍只交付 10%,一条命令要十几拍才走完)。
                    // 于是喂进去的两帧之间混了**两条命令**的运动,而我只告诉它其中一条 ——
                    // 线性模型当然解释不掉,残差永远下不来。
                    // ⇒ 前帧现拍。多花一次 `sense()`,换的是这张图能不能学得会。
                    let 前深图 = 深图(plug, 相机号);
                    let Some((_, _, 前帧)) = plug.sense().and_then(|f| 灰(&f, 相机号)) else {
                        println!("[服]      拍不到前帧 ⇒ 停"); break };
                    let Some((走, 动实, 挡)) = 迈通道稳(plug, &动0, 比, jaw0, *通道是关节, 1) else {
                        println!("[服]      命令发不出去 ⇒ 停"); break };
                    // 🔴🔴 **不等停的时候 `挡` 是假阳性,不许信它。**
                    //
                    // `挡` 的定义是"残差连着三次一模一样",而它是在 `落` 里数的 —— 那是**等停版**的判据。
                    // 稳拍=1 时命令刚发出去、手臂还没走,残差自然连着几次没变 ⇒ 每一步都像被挡住。
                    // 实测代价(XU):第 8 步就报"被挡住"、而画面差还有 **0.5581(半个画面)**,
                    // 于是在半个画面之外就地合了一把空气。
                    // ⇒ 这一段只信**真的一动没动**(走 < 1e-5,下面那一条),它不依赖等停。
                    let _ = 挡;
                    if 走 < 1e-5 { 没动 += 1 } else { 没动 = 0 }
                    if 没动 >= 5 {
                        减 += 1; 没动 = 0;
                        if 减 <= 3 { println!("[服]      推不动(画面还差 {差像:.4})⇒ 步子减半再试({减}/3)"); }
                        else { println!("[服]      🟢 减到最小还是推不动 ⇒ **压在东西上了,就地合**"); 碰上了 = true; break }
                    }
                    if k % 10 == 0 {
                        println!("[服]      画面追:第 {k}/{上限} 步 · 我在 ({现u:.3},{现v:.3}) 深 {现深:.3} · 横纵差 {差像:.4} 深差 {:+.4}", 目深 - 现深);
                        // 🔴🔴 **把"被跟踪的那一块"和"目标"一起画出来存下来。**
                        //
                        // 画面闭环绕过的只是"换算成三维时的偏差";如果**被跟的那块根本不在爪子上**
                        //(比如跟的是一块地板),那把它挪到球上、爪子也不在球上。
                        // 这一条不看图就没法判 —— 而不看图往下走,又是"日志全绿而世界没发生"。
                        // 方框 = 我在跟的那一块 · 十字 = 目标那个像素。
                        if let Ok(存路) = std::env::var("BL_VID") {
                            let mut 图 = g.clone();
                            let mut 点 = |x: i64, y: i64, v: u8| {
                                if x >= 0 && y >= 0 && (x as usize) < gw && (y as usize) < gh { 图[y as usize * gw + x as usize] = v; }
                            };
                            let (bx, by) = ((现u * gw as f64) as i64, (现v * gh as f64) as i64);
                            let r = 半跟 as i64;
                            for t in -r..=r { 点(bx + t, by - r, 255); 点(bx + t, by + r, 255); 点(bx - r, by + t, 255); 点(bx + r, by + t, 255); }
                            let (tx, ty) = ((目u * gw as f64) as i64, (目v * gh as f64) as i64);
                            for t in -14i64..=14 { 点(tx + t, ty, 0); 点(tx, ty + t, 0); }
                            let mut buf = format!("P5\n{gw} {gh}\n255\n").into_bytes();
                            buf.extend_from_slice(&图);
                            let _ = std::fs::write(format!("{存路}/track{:03}_{k:04}.pgm", 集), buf);
                        }
                    }
                }
                if 被切走 { continue }
                if 被切走 { continue }
                // 没在画面上到位就**不合** —— 回工作循环顶上重看一眼再来。
                // ⚠️ 这一层的到位判据是**画面上的**(上面那个 `块 * 0.5`),不是三维距离:
                //    三维那一版把"我以为我的指尖在哪"当起点,而那个量是**有偏的**,
                //    迭代磨不掉偏差(XS 实测:走到差 4.5 mm、照样合空)。
                if !碰上了 {
                    println!("[服]      画面上还没到位 ⇒ **不合**,回去重看一眼再走");
                    continue
                }
                // 交给下面同一段合爪 + 退回 —— 不另写一套。
                已就位 = true;
                [(u, v), (u, v)]
                }
            }
        };
        // 🔴🔴 **切模板那一帧的爪子状态,必须和【追的时候】一样。**
        // 上面那段相减把爪子留在了**合到底**,而追的全程爪子是 `jaw0`(张开)——
        // 同一根手指在张和合两种状态下在画面里长得不一样,拿合的模板去跟张开的手指,
        // 跟踪器只能锁到一块"看着像"的别处。这一条不含任何身体词:**要跟什么状态,就在什么状态下取样。**
        抓握(plug, jaw0, *通道是关节);
        定爪(plug, 等拍 * 8);
        let (fw, fh, g0) = match plug.sense().and_then(|f| 灰(&f, 相机号)) { Some(v) => v, None => continue };
        // 同上:模板半径 = 两个接触面间距的一半,切到刚好不重叠(见探表那一段记的代价)。
        let 半 = {
            let dx = (面点[0].0 - 面点[1].0) * fw as f64;
            let dy = (面点[0].1 - 面点[1].1) * fh as f64;
            ((dx * dx + dy * dy).sqrt() * 0.5) as usize
        }.max(3);
        // 🔴 同上:靠边就缩半径,中心一步都不许挪。
        let 半 = {
            let 缩 = 合用半(fw, fh, &[面点[0], 面点[1]], 半);
            if 缩 > 0 && 缩 < 半 { println!("[服]   接触面靠画面边了 ⇒ 模板半径 {半} → {缩} 像素(中心不动)"); }
            缩
        };
        let mut 模: Vec<Vec<u8>> = Vec::new();
        if 半 > 0 {
            for (pu, pv) in 面点 {
                match 截块(fw, fh, &g0, pu, pv, 半) { Some(t) => 模.push(t), None => break }
            }
        }
        if 模.len() < 2 {
            println!("[服]   两个接触面里有一个**在画面外**(({:.3},{:.3}) / ({:.3},{:.3}))⇒ 这一拍先把自己挪进画面",
                面点[0].0, 面点[0].1, 面点[1].0, 面点[1].1);
            continue
        }
        let mut 面 = [面点[0], 面点[1]];
        // 下手**之前**这两个面在画面哪儿、多深 —— 退回去的目标就是它,不需要"上"这个概念。
        let 面初 = 面点;
        let 面深初 = {
            let a = 近侧深(plug, 面点[0].0, 面点[0].1, 块 * 0.5);
            let b = 近侧深(plug, 面点[1].0, 面点[1].1, 块 * 0.5);
            match (a, b) { (Some(x), Some(y)) => [x, y], _ => { println!("[服]   两个面读不到深度 ⇒ 这一拍不下手"); continue } }
        };
        println!("[服]   两个接触面在画面 ({:.3},{:.3}) 深 {:.3} 和 ({:.3},{:.3}) 深 {:.3}",
            面[0].0, 面[0].1, 面深初[0], 面[1].0, 面[1].1, 面深初[1]);

        // 目标:接触集给的两个点,投到画面上(同一台从表里解出来的相机)。
        let 目 = |p: [f64; 3]| -> Option<(f64, f64, f64)> {
            let q = point_gen::P3 { x: p[0], y: p[1], z: p[2] };
            let px = 眼.project(q)?;
            Some((px[0] / fw as f64, px[1] / fh as f64, 眼.into_cam(q)[2]))
        };
        let (Some(t0), Some(t1)) = (目(接触点[0]), 目(接触点[1])) else {
            println!("[服]   接触点投不到画面上 ⇒ 换个下手点"); 试过.push(尖目标); continue
        };
        println!("[服]   两个接触点在画面 ({:.3},{:.3}) 深 {:.3} · ({:.3},{:.3}) 深 {:.3}",
            t0.0, t0.1, t0.2, t1.0, t1.1, t1.2);

        // 追:每一拍重新看,不闭眼。会动的目标也追得上 —— 目标动了它的像素跟着动。
        let Some(mut 表) = 通道表.clone() else { println!("[服]   还没有通道表 ⇒ 这一拍不下手"); continue };
        // 跟踪器不动时自己会飘多少 —— 连读两帧,差多少就是多少。**门槛由它给,不是我填。**
        // 🔴 **量的必须是【闸自己要比的那个统计量】的噪声,不是随便一个范数。**
        // 刚体闸比的是 `|(A 的位移) − (B 的位移)|`(只看横纵)⇒ 就在**不动**的两帧上算同一个式子,
        // 算出来多少就是它的噪声。实测代价(NV7):我拿六个数(含米制深度)的范数当门槛,
        // 结果门槛比真噪声小一个量级 ⇒ **追的每一拍都误判成跟丢**,真观测全被扔掉,
        // 12 步误差只从 0.719 挪到 0.675。
        let 噪追 = {
            let mut 读 = |plug: &mut Plug<S>| -> Option<[f64; 4]> {
                let (_, _, g) = plug.sense().and_then(|f| 灰(&f, 相机号))?;
                let (甲, 乙) = 找两块(fw, fh, &g, &模[0], &模[1], 半)?;
                Some([甲.0, 甲.1, 乙.0, 乙.1])
            };
            match (读(plug), 读(plug), 读(plug)) {
                (Some(a), Some(b), Some(c)) => {
                    let f = |x: [f64;4], y: [f64;4]| (((y[0]-x[0])-(y[2]-x[2])).powi(2)
                                                    + ((y[1]-x[1])-(y[3]-x[3])).powi(2)).sqrt();
                    f(a, b).max(f(b, c))
                }
                _ => 0.0,
            }
        };
        println!("[服]   刚体闸的噪声底(不动时两个面各走各的会差多少):{噪追:.5} 画幅");
        let mut 上拍: Option<([f64; 6], Vec<f64>)> = None;
        let mut 上步 = 0.0f64;   // 上一步**实际**走了多少米 —— 追不动的时候,这个数直接说明是谁的问题
        let mut 学了 = 0u32;
        let mut 碰上 = false;
        // 🔴🔴🔴 **追多少步,不许写死 —— 停手的判据是"误差不再往下掉"或"碰上了"。**
        //(WV 实证,2026-08-28)
        //
        // 上一版写死 20 步。WV 实测:误差 0.5941 → 0.4422 → 0.2996 → **0.2260**,
        // 每四步稳稳掉 0.07,**还在往下走**,而 20 步用完了 ⇒ `没追到位就停了` ⇒ 合了个空。
        // 它离碰上就差十几步。**这是"我自己的计数把还在收敛的过程掐掉"** ——
        // 今天同一个错误已经犯过一次(那次是我新写的伺服,`拍数用完`)。
        // ⇒ 上限由量出来的东西给:够得到多远 ÷(一步 × 交付率),再留一倍余量;
        //   它只防死循环,**真正停手看"误差不再缩"或"碰上了"**。
        // 20 / 4000 是**步数**上下限(计数,无量纲);2 是余量倍数,也是无量纲。
        let 追上限 = ((可达带[1] / (探步 * 交付率).max(1e-9)) * 2.0).ceil().clamp(20.0, 4000.0) as u32;
        let mut 追不动 = 0u32;
        let mut 重量过 = false;   // 这一集只就地重量一次,免得来回抖
        let mut 上误 = f64::MAX;
        println!("[服]   追:上限 {追上限} 步(只防死循环;停手看误差不再缩或碰上了)");
        // 🔴🔴🔴 **避障第二段:整条身体的占位 + 走之前先问"这一步会不会撞"。**(owner 2026-08-28)
        //
        // 第一段只做到"撞上了之后分清撞的是不是目标"——**那是事后**,东西已经被碰倒了。
        // 这一段是**事前**:每一步发出去之前,先算"我走完会占哪一片画面",
        // 和"画面里别的东西占哪几片"比一比,重叠就别这么走。
        //
        // 三样东西全是已经有的,没有任何身体模型、没有连杆几何、没有 URDF:
        //   ① **我占哪一片** —— 我一动,画面里跟着我动的就是我(构造性身份,和认接触面同一条)。
        //      这里连额外动作都不用花:**追本来每一步都在动**,前后两帧一减就是我的占位,
        //      而且它天然包含**整条胳膊**,不只是手 —— 外部审阅点的正是这一处。
        //   ② **别的东西占哪几片** —— 减法③切出来的那些块,去掉被挑中的那一块(目标)。
        //   ③ **这一步会把我挪到哪** —— 通道表就是干这个的:命令 × 表 = 画面上跑多少。
        //
        // 深度也要对上:画面上重叠但**深度差得远**的不算挡路(它在我后面)。
        // 判据用那一块自己鼓出来多高,不是一个填的容差。
        let 障碍: Vec<point_gen::区> = {
            let mut v: Vec<point_gen::区> = 区们.iter().copied().collect();
            if let Some(t) = 目标区 { v.retain(|r| r.框 != t.框); }
            v
        };
        if !障碍.is_empty() {
            println!("[服]   避障:画面里除了目标还有 {} 块东西,每一步之前都会先算会不会撞上", 障碍.len());
        }
        // 我的占位(画幅坐标下的一堆点)——**边走边攒**,第一步之前还没有,那一步不设防。
        let mut 我影: Vec<(f64, f64)> = Vec::new();
        let mut 上帧: Option<Vec<u8>> = None;
        for 追 in 0..追上限 {
            // 走过"搬过去"那条路的,手已经在目标上了 —— 追要跟住两根手指,而这台相机里跟不住。
            if 已就位 { break }
            if plug.复位过 { println!("[服]   追的中途换集了 ⇒ 这一把作废"); break }
            let Some((_, _, g)) = plug.sense().and_then(|f| 灰(&f, 相机号)) else { break };
            // 🔴 **我一动,画面里跟着动的就是我。** 上一步的前后两帧一减,得到的就是
            //    **整条胳膊**此刻的占位 —— 不需要认识"手臂"这个词,也不需要任何连杆模型。
            if let Some(前) = 上帧.as_ref() {
                if 前.len() == g.len() {
                    let mut 点: Vec<(f64, f64)> = Vec::new();
                    // 8 是**灰度差的噪声底**,和认接触面那一处同一个来源(无量纲)。
                    for i in 0..g.len() {
                        if 前[i].abs_diff(g[i]) > 8 {
                            点.push(((i % fw) as f64 / fw as f64, (i / fw) as f64 / fh as f64));
                        }
                    }
                    if !点.is_empty() { 我影 = 点; }
                }
            }
            上帧 = Some(g.clone());
            let mut 现 = [(0.0f64, 0.0f64, 0.0f64); 2];
            let mut 全看见 = true;
            // 🔴🔴 **和探表那一段同一条:B 只在"离 A 一个间距"的地方找。**
            //
            // `找两块` 的做法是"找完 A,把 A 周围一个模板半径排除掉再找 B"。而 XB 实测两根手指
            // 只隔 **5–10 px**、模板半径 12 ⇒ **排除半径比真实间距还大,B 的正确答案被排掉**,
            // 它只能跑到画面别处(NV9 那条注释记的是同一个病的另一半)。
            // 追的全程爪子也不动(jaw0 按住),所以间距同样是常数 —— 用它,而不是用排除。
            {
                let 偏 = (面[1].0 - 面[0].0, 面[1].1 - 面[0].1);
                let 隔px = ((偏.0 * fw as f64).powi(2) + (偏.1 * fh as f64).powi(2)).sqrt();
                match 找块窗(fw, fh, &g, &模[0], 半, 面[0].0, 面[0].1, 半 * 3) {
                    Some(a) => {
                        let 窗b = ((隔px * 0.5) as usize).clamp(1, 半 * 3);
                        match 找块窗(fw, fh, &g, &模[1], 半, a.0 + 偏.0, a.1 + 偏.1, 窗b) {
                            Some(b) => {
                                for (i, p) in [a, b].into_iter().enumerate() {
                                    面[i] = p;
                                    match 近侧深(plug, p.0, p.1, 块 * 0.5) {
                                        Some(nd) => 现[i] = (p.0, p.1, nd),
                                        None => { 全看见 = false; }
                                    }
                                }
                            }
                            None => { 全看见 = false; }
                        }
                    }
                    None => { 全看见 = false; }
                }
            }
            // 同上:两个面落得比模板自己还近 ⇒ 锁到同一块图案上了(**比例**判据,不是填的数)。
            if 全看见 && ((现[0].0 - 现[1].0) * fw as f64).hypot((现[0].1 - 现[1].1) * fh as f64) < 半 as f64 {
                println!("[服]   追:两个接触面锁到同一块图案上了 ⇒ 停在这儿"); 全看见 = false;
            }
            if !全看见 { println!("[服]   追:跟丢了接触面 ⇒ 停在这儿"); break }
            // 🔴🔴🔴 **表边干边长 —— 没有"量身体阶段"这回事。**
            // 上一拍我下了什么命令、接触面跟着跑了多少,现在都知道了,当场记回表里:
            //   表 ← 表 + (**实际跑的** − **表预测会跑的**) ⊗ 上一拍**实际动了多少** ÷ |实际动了多少|²
            //
            // 🔴🔴 **这句话里必须是"实际动了多少",不是"命令"。** 代码一直是对的
            //(`上拍 = Some((现6, 动实))`),而这行注释以前写的是"命令" —— **注释骗人**。
            // 实测代价(YB,2026-08-29):我照着这句错注释去写自图,喂进去的是命令,
            // 于是"解释得掉的"永远是 0。因为这具身体**一拍只执行掉命令的约 10%,
            // 而且比例每步都变**(命令越大比例越小、离目标越近越慢、顶到限位就是 0)——
            // "命令 → 画面"根本不是常数,学不出来;"实际动了多少 → 画面"才是真正的雅可比。
            // LAB 早就写死过同一条:*"两遍各除以**自己的实到**"*。
            // 干活的每一次动作本来就是一次探针 —— 不必回到任何地方,也不必停下来专门量。
            // 物体在跑、身体被改造、相机被撞歪,表都自己跟上;**三天后那只会乱动的老鼠要的就是这个。**
            let mut 现6 = [现[0].0, 现[0].1, 现[0].2, 现[1].0, 现[1].1, 现[1].2];
            // 🔴🔴 **有了表,跟丢就不再是死路 —— 表能预测手指该在哪,拿预测把跟踪拉回来。**
            // 判据还是刚体那一条(ΔA − ΔB = ω × r,两个量都是量出来的):
            // 两个接触面在相邻两帧之间**不许各走各的**,超了就说明模板锁到别处去了。
            // 实测代价(NV6):追到第 4 步,面A 的误差 (+0.336,-0.729,+0.111)、面B (+0.185,+0.667,-0.966)
            // —— 一个要往上一个要往下,合计误差从 0.688 **涨到** 1.4385,越追越远。
            // 拉回来之后**按预测的位置重切模板**:视角变了,旧模板本来也该退休。
            let mut 拉回 = false;
            if let Some((前6, 前动)) = 上拍.clone() {
                let 甲 = [现6[0]-前6[0], 现6[1]-前6[1]];
                let 乙 = [现6[3]-前6[3], 现6[4]-前6[4]];
                let 面分 = ((甲[0]-乙[0]).powi(2) + (甲[1]-乙[1]).powi(2)).sqrt();
                let 我间距 = ((现6[0]-现6[3]).powi(2) + (现6[1]-现6[4]).powi(2)).sqrt();
                let 转 = (3..6).map(|i| 前动.get(i).copied().unwrap_or(0.0).powi(2)).sum::<f64>().sqrt();
                if 面分 > 转 * 我间距 + 噪追 * 2.0 {
                    for r in 0..6 {
                        let 预: f64 = (0..表.len()).map(|c| 表[c][r] * 前动.get(c).copied().unwrap_or(0.0)).sum();
                        现6[r] = 前6[r] + 预;
                    }
                    println!("[服]   追:两个接触面各走各的(差 {面分:.3} > 刚体上限)⇒ 跟丢了,按表的预测拉回来并重切模板");
                    拉回 = true;
                }
            }
            if 拉回 {
                for i in 0..2 {
                    面[i] = (现6[i*3], 现6[i*3+1]);
                    现[i] = (现6[i*3], 现6[i*3+1], 现6[i*3+2]);
                    if let Some(t) = 截块(fw, fh, &g, 面[i].0, 面[i].1, 半) { 模[i] = t }
                }
                上拍 = None;   // 这一拍的观测不可信,不拿它去修表
            }
            if let Some((前6, 前动)) = 上拍.take() {
                let 量 = 前动.iter().map(|x| x * x).sum::<f64>();
                let 跑 = (0..6).map(|r| (现6[r] - 前6[r]).powi(2)).sum::<f64>().sqrt();
                if 量 > 0.0 && 跑 > 噪追 {
                    for r in 0..6 {
                        let 预: f64 = (0..表.len()).map(|c| 表[c][r] * 前动.get(c).copied().unwrap_or(0.0)).sum();
                        let 差 = (现6[r] - 前6[r]) - 预;
                        for c in 0..表.len() { 表[c][r] += 差 * 前动.get(c).copied().unwrap_or(0.0) / 量; }
                    }
                    学了 += 1;
                }
            }
            // 六个方程(两个面 × 三个数),通道数个未知数 ⇒ 超定/欠定都由最小二乘吃掉。
            // 🔴🔴 **六个数必须同一个量纲。** 前两个是画幅比例、第三个是米 ——
            // 混在一起求范数、再拿去和一个画幅阈值比,**判据本身是错的**(要么早停要么永不停)。
            // 换算是量出来的:这个深度上 **1 个画幅单位 = 画幅宽 × 深 ÷ 焦距** 米,
            // 所以 米 → 画幅 = 米 × 焦距 ÷(画幅宽 × 深)。焦距来自从表解出来的那台相机。
            // 🔴🔴🔴 **目标不是"两个面直接落到两个接触点上" —— 张着爪子做不到。**
            //
            // 实测代价(FD):接触集给的两个接触点相距 **3.8 mm**(那是**夹住那一刻**两个面的位置),
            // 而此刻爪子张着、两个面相距几十毫米 ⇒ 要求它们同时落到相距 3.8 mm 的两点上,
            // **物理上不可能** ⇒ 最小二乘互相打架,误差从 **0.65 涨到 0.74**(越追越远)。
            //
            // 接触集真正的意思是:**摆到这样一个位形,使得【合上去】之后两个面正好落在那两点上。**
            // ⇒ 目标 = **两接触点的中点** ± **我此刻两个面的半间距** × **两接触点连线的方向**。
            //   中点管"去哪儿",方向管"怎么摆" —— 姿态还是解出来的,而且这一版**可解**。
            //   这里没有一个身体词:"我此刻两个面相距多少"是量出来的,不是"钳口张开量"。
            let 深换 = 眼.fx / (fw as f64 * t0.2.max(1e-6));
            let 我间距 = (((现[0].0 - 现[1].0)).powi(2) + ((现[0].1 - 现[1].1)).powi(2)).sqrt();
            let (mut 轴u, mut 轴v) = (t1.0 - t0.0, t1.1 - t0.1);
            let 轴长 = (轴u * 轴u + 轴v * 轴v).sqrt();
            if 轴长 > 1e-9 { 轴u /= 轴长; 轴v /= 轴长; } else {
                // 两接触点在画面上重合(正对着看)⇒ 方向无从谈起,就用我此刻的连线方向
                let l = 我间距.max(1e-9);
                轴u = (现[1].0 - 现[0].0) / l; 轴v = (现[1].1 - 现[0].1) / l;
            }
            let (中u, 中v, 中深) = ((t0.0 + t1.0) * 0.5, (t0.1 + t1.1) * 0.5, (t0.2 + t1.2) * 0.5);
            let 半 = 我间距 * 0.5;
            let 该 = [(中u - 轴u * 半, 中v - 轴v * 半, 中深), (中u + 轴u * 半, 中v + 轴v * 半, 中深)];
            let 误 = vec![该[0].0 - 现[0].0, 该[0].1 - 现[0].1, (该[0].2 - 现[0].2) * 深换,
                          该[1].0 - 现[1].0, 该[1].1 - 现[1].1, (该[1].2 - 现[1].2) * 深换];
            let 差 = 误.iter().map(|x| x * x).sum::<f64>().sqrt();
            let 到位 = 块 * 0.5;   // 判据 = **我自己那一块有多大**,量出来的,不是拍的阈值
            if 追 % 4 == 0 || 差 <= 到位 {
                println!("[服]   追 {追}:两面差 ({:+.3},{:+.3},{:+.3}) / ({:+.3},{:+.3},{:+.3}) ⇒ 合计 {差:.4} · 上一步实际走了 {上步:.4} m",
                    误[0], 误[1], 误[2], 误[3], 误[4], 误[5]);
            }
            if 差 <= 到位 { println!("[服]   🟢 两个接触面都到位了(差得比我自己那一块还小)"); 碰上 = true; break }
            // 🔴 **停手看"误差还缩不缩",不看步数。**(仓里那条 LAB:69:把目标按住不放,
            //    看残差还缩不缩;不再缩而且还差着 = 有东西挡着。)3 是**计数**,无量纲。
            if 差 >= 上误 { 追不动 += 1 } else { 追不动 = 0 }
            上误 = 差;
            if 追不动 >= 3 {
                // 🔴🔴🔴 **不再缩 ≠ 到位了。先怀疑【表在这儿已经不准了】,而不是收工。**
                //(WW 实证,2026-08-28)
                //
                // WW 实测:横向 0.051→0.012、深度 →0.003 都收敛了,**纵向死在 +0.039 不动**,
                // 而且每步只挪 **0.0002 m** —— 解算自己只敢要这么小的一步。
                // 机制:量表时 `末端0` 的纵向响应是 **+0.0000**(它完全不改变上下),
                // 只有 1/2 号有纵向响应、而它俩同时大幅改变横向与深度 ⇒ 纵向是**病态方向**,
                // 被阻尼压掉 ⇒ 好走的两维收敛、难走的那一维原地不动。
                // 更根本的一条:**表是在起点量的,而手臂已经挪了三十多厘米** —— 那张表在这儿早就不准。
                //
                // ⇒ 就地把表重量一遍再接着追。这正是这条路本来就该有的"边干边长",
                //   而这一炮一次都没触发。只重量一次,免得来回抖。
                if 差 > 到位 && !重量过 {
                    println!("[服]   追:误差不再缩(停在 {差:.4})而还没到位 ⇒ **表在这儿已经不准了,就地重量一次**");
                    重量过 = true;
                    *通道表 = None; *雅载 = None;
                    break
                }
                println!("[服]   追:误差连着三步不再缩(停在 {差:.4})⇒ 就在这儿合,结果照实报");
                break
            }
            // 表:每个通道一列,列里是"这个通道动一点,两个面各跑多少"。
            // 这一版表只量了被跟的那一块(3 行),两个面暂时共用同一列响应 —— 面之间的差异
            // 由**合爪那一步**吃掉(两指同步张合)。⚠️ 五指手上要分面量,这一条写在下面的欠账里。
            // 🔴🔴 **解出来的是【每个通道各动多少】,不是"末端往哪挪"。**
            // 表的每一列 = 这个通道动一点,两个面各跑多少(六个数)⇒ 六个方程、通道数个未知数。
            // 通道多于约束时,λ 让它挑**最小动作**那一解;这正是"姿态自己长出来"的地方 ——
            // 我一个字都没说手腕该转到哪。
            // 表的第 2、5 行(两个面的深度响应)也换成画幅单位,和误差同量纲。
            let 行: Vec<Vec<f64>> = (0..6).map(|r| 表.iter()
                .map(|c| if r == 2 || r == 5 { c[r] * 深换 } else { c[r] }).collect()).collect();
            let Some(动) = 最小二乘(&行, &误) else { println!("[服]   解不出该动多少 ⇒ 停"); break };
            let 步长 = 动.iter().map(|x| x * x).sum::<f64>().sqrt();
            if !(步长 > 1e-9) { break }
            // 🔴 **一步走多远,由"我的接触面在画面里会跑多远"定 —— 不许跑得比两个面自己的间距还远。**
            // 理由是可跟踪性:模板半径就是间距的一半,一步跑一个间距,新旧位置还有重叠,跟得住;
            // 跑得更远就等着跟丢。表能预测这一步会跑多少,所以这个尺是**算出来的**,不是填的。
            // (旧写法是"一步不超过探针那一档",而探针那一档是从可达距离推的,和画面无关。)
            let 预跑 = {
                let mut a = [0.0f64; 6];
                for r in 0..6 { a[r] = (0..表.len()).map(|c| 表[c][r] * 动.get(c).copied().unwrap_or(0.0)).sum(); }
                (a[0]*a[0] + a[1]*a[1]).sqrt().max((a[3]*a[3] + a[4]*a[4]).sqrt())
            };
            let 我间距 = ((现6[0]-现6[3]).powi(2) + (现6[1]-现6[4]).powi(2)).sqrt().max(1e-9);
            let mut 比 = (探幅 / 步长).min(1.0).min(我间距 / 预跑.max(1e-9));
            // 🔴🔴 **走之前先问:这一步会把我推到别的东西上吗。**
            //
            // 预测很直接:这一步命令 × 通道表 = 我的接触面在画面上跑多少;整条身体近似
            // 跟着**同一个**画面位移走(相机不动、这一步又小)⇒ 把我的占位整体平移过去,
            // 和每一块障碍的框比一比。深度也要对上 —— 在我后面的东西挡不着我。
            //
            // 撞上了**不站着**:先把步子减半再问,最多减到十六分之一;还是撞就换一条候选,
            // 让接触集从别的方向下手(而不是硬撞过去,更不是原地不动)。
            if !障碍.is_empty() && !我影.is_empty() {
                let 会撞 = |比: f64| -> Option<usize> {
                    let mut a = [0.0f64; 6];
                    for r in 0..6 { a[r] = (0..表.len()).map(|c| 表[c][r] * 动.get(c).copied().unwrap_or(0.0) * 比).sum(); }
                    // 两个面的画面位移取平均当作整体位移(它们是同一只手上的两个点)。
                    let (du, dv) = ((a[0] + a[3]) * 0.5, (a[1] + a[4]) * 0.5);
                    // 我此刻在哪一层(深度),取两个接触面的均值。
                    let 我深 = (现6[2] + 现6[5]) * 0.5;
                    for (bi, b) in 障碍.iter().enumerate() {
                        // 深度对不上 ⇒ 它在我前后另一层,挡不着。用它自己鼓出来多高当容差。
                        if (b.深 - 我深).abs() > b.高.abs().max(1e-3) { continue }
                        let (x0, y0) = (b.框[0] as f64 / fw as f64, b.框[1] as f64 / fh as f64);
                        let (x1, y1) = (b.框[2] as f64 / fw as f64, b.框[3] as f64 / fh as f64);
                        let 压上 = 我影.iter().filter(|(u0, v0)| {
                            let (u1, v1) = (u0 + du, v0 + dv);
                            u1 >= x0 && u1 <= x1 && v1 >= y0 && v1 <= y1
                        }).count();
                        // 十分之一是**比例**:我的占位有一成压进那个框就算撞上,无量纲。
                        if 压上 * 10 >= 我影.len() { return Some(bi) }
                    }
                    None
                };
                let mut 退 = 0u32;
                while let Some(bi) = 会撞(比) {
                    if 退 >= 4 {
                        let b = 障碍[bi];
                        println!("[服]   🔴 避障:怎么减步子这一步都会压到第 {bi} 块东西上(它在画面 ({:.3},{:.3})-({:.3},{:.3})、深 {:.3} m)",
                            b.框[0] as f64 / fw as f64, b.框[1] as f64 / fh as f64,
                            b.框[2] as f64 / fw as f64, b.框[3] as f64 / fh as f64, b.深);
                        println!("[服]      ⇒ **这条候选从这个方向过不去**,换一条,让接触集从别的方向下手");
                        试过.push(尖目标);
                        break
                    }
                    比 *= 0.5;
                    退 += 1;
                    println!("[服]   避障:这一步会压到第 {bi} 块东西上 ⇒ 步子减半({退}/4)再算");
                }
                if 退 >= 4 && 会撞(比).is_some() { break }
            }
            let Some((走, 动实, 挡)) = 迈通道(plug, &动, 比, jaw0, *通道是关节) else {
                println!("[服]   命令发不出去 ⇒ 停"); break
            };
            if let Some(ee) = plug.sense().and_then(|f| f.ee.get(手号.get()).copied()) { 位 = [ee[0], ee[1], ee[2]]; }
            // 🔴🔴🔴 **"被挡住"不等于"我到了" —— 必须先问:挡我的是【目标】还是【别的东西】。**
            //(owner 2026-08-28;外部审阅也点到同一处)
            //
            // 上一版直接把"挡住"当成"碰到目标了 ⇒ 就地合"。旁边放个箱子,它撞上去、
            // **然后对着箱子合爪**,而且还报一个绿色的成功信号 —— 比"撞了不吭声"更难查。
            // ⚠️ 这不只是避障的事:**IK 拒绝、桌面顶住、关节限位,在读数上和"碰到目标"完全同形**,
            //    所以这个假阳性**现在就在害我们**(合到底指间没东西的那些次里,有多少是这么来的?)。
            //
            // 判据不需要几何模型,只用已经量到的两样:**我的接触面在画面哪儿** 和 **目标在画面哪儿**。
            // 挡住那一刻,如果我的接触面已经贴到目标上(画面距离小于我自己那一块的大小、
            // 且深度也对得上),那挡我的就是目标;否则挡我的是别的东西。
            // "我自己那一块有多大"是量出来的(`块`),不是拍的阈值。
            // 0.3 是**比例**(至少走了三成的一步才算真挡住,而不是一发就被拒),无量纲。
            if 挡 && 走 >= 探步 * 0.3 {
                let 贴上 = (0..2).any(|i| {
                    let (du, dv) = (该[i].0 - 现[i].0, 该[i].1 - 现[i].1);
                    let dz = (该[i].2 - 现[i].2).abs();
                    (du * du + dv * dv).sqrt() <= 块 && dz * 深换 <= 块
                });
                if 贴上 {
                    println!("[服]   🟢 追的路上**碰到目标了**(走了 {走:.4} m 才停,接触面贴在目标上)⇒ 就地合");
                    碰上 = true;
                } else {
                    println!("[服]   🔴 追的路上**被【别的东西】挡住了**(走了 {走:.4} m,而接触面离目标还有 {差:.4})");
                    println!("[服]      ⇒ **不合爪**(对着障碍物合是假阳性);换一条候选,让接触集从别的方向下手");
                    试过.push(尖目标);
                }
                break
            }
            上步 = 走;
            if 走 < 1e-5 { println!("[服]   追:这一步一动没动 ⇒ 这个位形过不去"); break }
            上拍 = Some((现6, 动实));
        }
        // 这一把学到的东西**留下来** —— 下一把不必从头。
        *通道表 = Some(表.clone());
        if 学了 > 0 { println!("[服]   追的路上顺手把表修了 {学了} 次(边干边长,没有量身体阶段)"); }
        if !碰上 { println!("[服]   没追到位就停了 —— 照样合一下,结果照实报"); }

        // 🔴🔴🔴 **最后一两厘米:在【长在手上】的那台相机里,把物体开到两指中间。**
        //
        // 那台相机里**手是不动的** —— 于是"我的手在画面哪儿"这个问题不存在,
        // 而它正是今晚一半 bug 的来源。要做的只剩一件:**把物体挪到两指中间、挪到碰上的距离。**
        // 判据仍然全是量出来的:两指中间在哪(晃一下爪子看哪块动)· 碰上没有(压不动了)。
        if let (Some(腕机), false) = (手上相机, 已就位) {
            if 收尾腕(plug, 腕机, jaw0, *通道是关节, &look, &mut 位).is_none() {
                println!("[服]   [收尾] 这一段没走完(上面已经说明理由)⇒ 直接合,结果照实报");
            }
        }

        // ⑥ **合到读数不再变为止 —— 无阈值。** 停在 0 = 指间没东西;停在 0 以上 = 有东西顶住,
        // 而那个读数就是它有多宽。老那版是「每拍变化小于 0.01 就算夹住」,而这只爪子合的时候
        // 先快后慢 ⇒ 一次**什么都没夹到的空合**后半段每拍变化本来就小于 0.01 ⇒ 每把都提前判成夹住。
        // 🔴 合的命令走 `抓握`(**唯一那处**按量出来的通道种类派发)—— 不再需要一个末端位姿,
        //    也不再写死是关节还是末端(F6 那次就是写死关节,而这具机体不响应关节 ⇒ 爪子永远合不上)。
        白转 = 0;   // 走到这儿就是**真的下手了**,兜底计数清零
        // 🔴🔴 **两个写死的身体约定在这里删掉**(owner 2026-08-28,下一步要零改动换 ARX 双臂):
        //   ① "开着的时候读数是 1.0" —— 改成**合之前读一次**,那个数就是这具身体的"开"。
        //   ② "停在 0 就是指间没东西" —— 改成和**量出来的空合读数**比。
        // 病相为什么阴:这具 Franka 空合确实报 0.0000,所以写死的判据在这具身体上**一直是对的**;
        // 换一具报 0.02 的身体,它会把**每一次空抓都判成成功**,而日志上全是绿的。
        // 🔴🔴🔴 **减法②:这一步的"要顶多硬 / 按住多久 / 什么时候算完"来自【动词】,不再写死。**
        //
        // `demand_with` 是全仓第一次被**驱动**调用 —— 在此之前它只有 CLI 和它自己的测试在调,
        // 于是那张 13 行的动词表在真正干活的这条路上**一行都没生效过**。
        // 现在它交出的是一个点:(走多少, 转多少, **顶多硬**, **按住多久**, **什么时候算完**)。
        // 三个新维度都不是我拍的:
        //   · 顶多硬 —— 眼说的 light/medium/firm,映到**这具身体自己那把尺**上 ∈[0,1];
        //     以前这个词全仓只有一处引用,而那一处只是把它抄进另一个结构体,**没人读**。
        //   · 按住多久 —— 秒,用**量出来的帧时**换成拍数(以前帧时只被打印,读不到)。
        //   · 什么时候算完 —— `Resist`(合到读数不再变)。这条驱动早就在做,只是没有名字,
        //     因此调用方点不到它,擦/按/拧也就只能写成"走够多少距离"。
        let 这一下 = {
            let 动 = look.verb.trim().to_ascii_lowercase();
            let v = match 动.as_str() {
                "push" => body_layer::verb::Verb::Push,
                "place" => body_layer::verb::Verb::Place,
                "pry" => body_layer::verb::Verb::Pry,
                "open" | "close" => body_layer::verb::Verb::Twist,
                // 眼没说 / 说了不认识 ⇒ 按抓。**不许因为一个词没见过就不动。**
                _ => body_layer::verb::Verb::Grasp,
            };
            // 轴:这一层不认识"工具轴/钳口轴",给单位轴占位 —— 合爪这一步只用到
            // press/hold/until 三样,方向由上面接触集那一层已经解完了。
            let ax = body_layer::verb::Axes { tool: [0.0, 0.0, 1.0], jaw: [1.0, 0.0, 0.0] };
            body_layer::verb::demand_with(v, ax, [0.0; 3], 0.0,
                body_layer::verb::press_from_word(&look.force), 0.0)
        };
        println!("[服] 这一下:眼说「{}·{}」⇒ 顶到 {:.2}(0=刚够认出碰上, 1=顶到读数不再变)· 按住 {:.2} s · {}",
            if look.verb.is_empty() { "(没说)" } else { &look.verb },
            if look.force.is_empty() { "(没说)" } else { &look.force },
            这一下.press, 这一下.hold_s,
            match 这一下.until {
                body_layer::verb::Until::Amount => "走完就停",
                body_layer::verb::Until::Contact => "直到碰上",
                body_layer::verb::Until::Resist => "直到推不动",
                body_layer::verb::Until::Slip => "直到东西不再跟着我",
                body_layer::verb::Until::Settle => "直到画面不再变",
            });
        let 停在 = 合到停住(plug, *通道是关节)?;
        // 指间有没有东西 = 停住的位置**离"空手合到底"有多远**。零点是量出来的;
        // 没量到就照实说,并退回这具身体上唯一还站得住的弱判据(合过、而且没合到最紧那一档)。
        let 空零 = *空合载;
        let 有东西 = match (停在.is_finite(), 空零) {
            (true, Some(z)) => (停在 - z).abs() > 1e-9,
            (true, None) => { println!("[服]   ⚠️ 还没量到「空手合到底报几」 ⇒ 退回弱判据(合过就算),结果照实报"); 停在 > 1e-9 }
            _ => false,
        };
        if !有东西 {
            match 空零 {
                Some(z) => println!("[服] 🔴 合到底了,读数停在 {停在:.4},和空手合到底 {z:.4} 一样 ⇒ **指间没东西**,换个下手点"),
                None => println!("[服] 🔴 合到底了,读数停在 {停在:.4} ⇒ **指间没东西**,换个下手点"),
            }
            试过.push(尖目标);
            抓握(plug, 1.0, *通道是关节);
            continue;
        }
        match 空零 {
            Some(z) => println!("[服] 🟢 合到停住,读数停在 {停在:.4}(空手合到底是 {z:.4},差 {:.4})⇒ **指间有东西**", (停在 - z).abs()),
            None => println!("[服] 🟢 合到停住,读数停在 {停在:.4} ⇒ 指间有东西,而**这个数就是它有多宽**"),
        }

        // 🔴 **减法②的力和时间在这里真的发出去。**
        //
        // 这具身体的爪子是**位置命令**的(七个关节命令全零响应,只有末端认),没有力通道 ——
        // 所以"更用力"在它身上唯一能表达的就是**停住之后继续发合的命令**:
        // 位置环把误差积起来,顶得更死。顶多久由 `press` 定,而 `press` 的两个端点是量出来的:
        //   0 = 刚停住就走(轻)· 1 = 一直发到读数在**更长的窗口里**也不再变(顶到底)。
        // "更长的窗口"取这具身体自己的稳态判据 `等拍` 的倍数 —— 不是一个拍出来的秒数。
        // ⚠️ 软的东西会在这一段里被继续压扁,读数还会往下走 —— 那正是"顶得更硬"的证据,
        //    所以这里**重新读一次停住的位置**,而不是沿用上面那个。
        let 顶拍 = ((这一下.press * (等拍 * 4) as f64).round() as u32).min(等拍 * 4);
        if 顶拍 > 0 {
            for _ in 0..顶拍 { 抓握(plug, 0.0, *通道是关节); if plug.sense().is_none() { break } }
            let 顶后 = plug.sense().and_then(|f| f.jaw.get(手号.get()).or_else(|| f.jaw.first()).copied()).unwrap_or(停在);
            println!("[服]   顶了 {顶拍} 拍(press={:.2})⇒ 读数 {停在:.4} → {顶后:.4}(还在往下走 = 东西被压实了)", 这一下.press);
        }
        // 按住多久:秒 → 拍,用**量出来的帧时**换算;帧时还没量到(开机头 50 拍)就跳过并说明。
        if 这一下.hold_s > 0.0 {
            if plug.帧时秒 > 1e-6 {
                let 拍 = (这一下.hold_s / plug.帧时秒).round() as u32;
                println!("[服]   按住 {:.2} s = {拍} 拍(帧时 {:.3} s,量出来的)", 这一下.hold_s, plug.帧时秒);
                for _ in 0..拍 { 抓握(plug, 0.0, *通道是关节); if plug.sense().is_none() { break } }
            } else {
                println!("[服]   要按住 {:.2} s,但**帧时还没量到**(开机头 50 拍)⇒ 这一下不按住,照实报", 这一下.hold_s);
            }
        }

        // ⑦ **原路退回去 = 把两个接触面送回它们下手之前在画面上的位置。**
        //
        // 🔴 不是"往世界 z 抬"("上"是世界假设,移动底盘/人形/无人机上不成立),
        //    也不再借接触集那个四元数的"第 2 列"(那是"第几列是工具轴"的约定,身体词)。
        // 退回的目标**就是我自己下手前站的地方** —— 它是量出来的(那两个模板当时在画面哪儿),
        // 而且**这条对任何身体、任何场景都成立**:原路回去,东西就离开了它原来待的地方。
        let mut 退成 = false;
        for 退 in 0..16u32 {
            if plug.复位过 { break }
            let Some((_, _, g)) = plug.sense().and_then(|f| 灰(&f, 相机号)) else { break };
            let mut 现 = [(0.0f64, 0.0f64, 0.0f64); 2];
            let mut 齐 = true;
            match 找两块(fw, fh, &g, &模[0], &模[1], 半) {
                Some((甲, 乙)) => for (i, p) in [甲, 乙].into_iter().enumerate() {
                    面[i] = p;
                    match 近侧深(plug, p.0, p.1, 块 * 0.5) { Some(nd) => 现[i] = (p.0, p.1, nd), None => 齐 = false }
                },
                None => 齐 = false,
            }
            if !齐 { break }
            let 深换 = 眼.fx / (fw as f64 * 面深初[0].max(1e-6));
            let 误 = vec![面初[0].0 - 现[0].0, 面初[0].1 - 现[0].1, (面深初[0] - 现[0].2) * 深换,
                          面初[1].0 - 现[1].0, 面初[1].1 - 现[1].1, (面深初[1] - 现[1].2) * 深换];
            let 差 = 误.iter().map(|x| x * x).sum::<f64>().sqrt();
            if 差 <= 块 * 0.5 { println!("[服]   🟢 退回到下手前那个位置了"); 退成 = true; break }
            let 行: Vec<Vec<f64>> = (0..6).map(|r| 表.iter()
                .map(|c| if r == 2 || r == 5 { c[r] * 深换 } else { c[r] }).collect()).collect();
            let Some(动) = 最小二乘(&行, &误) else { break };
            let 步长 = 动.iter().map(|x| x * x).sum::<f64>().sqrt();
            if !(步长 > 1e-9) { break }
            let 比 = (探幅 / 步长).min(1.0);
            // 🔴 这里曾经写死 `Cmd::Joints` —— 而这具机体一个关节命令都不响应,
            //    于是**夹住了也退不回去**,东西永远离不开原位。走同一处派发。
            //    合着的爪子在退回全程要一直合着 ⇒ jaw 传 0.0。
            if 迈通道(plug, &动, 比, 0.0, *通道是关节).is_none() { println!("[服]   退回:命令发不出去 ⇒ 停"); break }
            if 退 == 15 { println!("[服]   退回没走完(还差 {差:.4})"); }
        }
        let _ = 退成;
        // 🔴 **退回去了不等于带走了东西**(炮8 实测:「抬到位且 2/2 撑住」而前后两帧几乎逐像素相同,
        //    指间空无一物)⇒ 看**眼指的那一点还在不在原来的深度上**。判据用**我自己那一块**的大小。
        if let Some(新深) = 近侧深(plug, look.u, look.v, 窗) {
            let 变 = (新深 - d星).abs();
            // 我自己那一块换算成米:那一块占画幅 × 那个深度上一个像素合多少米。
            let 尺 = 块 * fw as f64 * d星 / 眼.fx.max(1e-9);
            if 变 > 尺 {
                println!("[服] 🟢🟢 眼指那一点的深度变了 {变:.3} m(>我自己那一块 {尺:.3})⇒ **东西离开原位了**");
            } else {
                println!("[服] 🔴 眼指那一点深度只变了 {变:.3} m ⇒ **东西没动**,刚才是空抓");
            }
        }
    }
}

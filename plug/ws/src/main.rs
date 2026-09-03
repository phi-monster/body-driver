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
        if let Cmd::Base { v } = c {
            // 底盘 / 腿 / 桨的速度通道:布局里有它们的路径才发得出去;今天没有任何身体报过 ⇒ 照实拒绝(不装作发了)。
            let 键: Vec<String> = self.lay.base.iter().filter_map(|p| p.last().cloned()).collect();
            if 键.is_empty() { return false; }
            self.待发 = Some(wire::base_action(&键, v));
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
    通道表: Option<&[Vec<f64>]>,
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
    // 部件图(已拍平成一行数)+ 手指自己在深度上有多厚。**加在最末尾**,免得动到位置参数。
    部件图存: Option<&str>,
    指厚: Option<f64>,
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
        // 每列 3N 个数:N 个接触面 × 横/纵/深。N 是认出来的面数(两指 2、五指 5、吸盘 1),存下来读回时才切得对。
        let n面 = t.first().map(|c| c.len() / 3).unwrap_or(2).max(1);
        j.push_str(&format!(",\n  \"n_surfaces\": {n面}"));
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
    // 🔴🔴🔴 **部件图也要存 —— 它是"我身上每一块在画面哪儿"的答案,是身体的属性。**
    //(BV 数出来的 2026-09-02:量它占了开场 **970 行日志**,而且**每一集从头做一遍** ——
    //  身上 10 个通道一个一个推一遍、再推回来,那 4 个指通道就是 owner 在视频里看到的
    //  "爪子开合几十次"。README 上它是第 3 号阻塞:*"每一集都要从零重量七样"*。)
    //
    // 存的只是**下游真的会读的那部分**:每个通道在每台相机里的框 + 占画幅多少。
    // 掩膜不存 —— 全仓只有量它的那一段自己读掩膜(算"手指有多厚"),而那个已经化成一个标量。
    // 一维、行优先(通道 × 相机 × 5),空的那格写 NaN;相机数装回时自己数得出来。
    if let Some(pm) = 部件图存 {
        j.push_str(&format!(",\n  \"parts_map\": [{}]", pm));
    }
    if let Some(v) = 指厚 {
        j.push_str(&format!(",\n  \"finger_thickness_m\": {v}"));
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
    for 键 in ["image_jacobian", "channel_table", "n_surfaces", "channels_are_joints",
               "camera_on_hand", "jaw_span_m", "jaw_closed_on_nothing", "jaw_motion_invisible", "hand",
               "parts_map", "finger_thickness_m"] {
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
    // 🔴🔴🔴 **自检开关:`BL_TESTEYE=1` ⇒ 只打一发"问身体",打印结果,退出。**
    //(owner 2026-09-01:"编译过了就当做完了"已经害我白烧两炮 ——
    //  第一次传错图像格式 14 次全废,第二次拼出来的 JSON 是坏的 98 次全废,
    //  **两次都是"编译通过"就上箱**。这个开关的存在就是为了不再有第三次:
    //  **要看见它真的回一句话,才准说这一块做完了。**)
    if std::env::var("BL_TESTEYE").is_ok() {
        let 眼 = std::env::var("BL_EYE").unwrap_or_else(|_| "127.0.0.1:8077".into());
        let (h, p) = 眼.split_once(':').unwrap_or(("127.0.0.1", "8077"));
        let port: u16 = p.parse().unwrap_or(8077);
        // 一张纯色小图就够 —— 这一发验的是**请求体拼得对不对**,不是它看得准不准。
        let (w, hh) = (64usize, 48usize);
        let rgb = vec![90u8; w * hh * 3];
        let 身体 = "- channel 6: the part that follows is at (0.81,0.56), 0.4% of frame\n                    - channel 2: moves 45% of your wrist camera\n";
        let 刚才 = "you commanded a move your body-model said would shift the picture by 0.0200 of a frame;                     it actually shifted by 0.0000. your hand physically moved 0.00000 m.";
        match body_layer::eye::问段(h, port, "Pick up the baseball by 10 cm.", 身体, 刚才, 6, 4, 3, 1, &rgb, w, hh) {
            Ok(d) => println!("[自检] 🟢 问段通了 ⇒ 动第 {} 号 → 落到**第 {} 格** · 做到**{}**为止 · 完了={} · 为什么={}", d.动第几号, d.到哪一格, d.到什么为止, d.完了, d.为什么),
            Err(e) => println!("[自检] 🔴 问段没通:{e}"),
        }
        return;
    }
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
    let mut 通道表: Option<Vec<Vec<f64>>> = None;
    // 装回来的"哪台相机长在手上" —— 探一次要动一下手臂,存住就不用重探。
    let mut 手上相机装回: Option<usize> = None;
    // 部件图(拍平)+ 手指厚度 —— 装回来就不用每集把身上每个通道推一遍。
    let mut 部件图装回: Option<Vec<f64>> = None;
    let mut 指厚装回: Option<f64> = None;
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
                    // 每列 3N 个数,N 从 n_surfaces 读(老文件没有这个键 ⇒ 2)
                    let n面 = j.get("n_surfaces").and_then(|x| x.num()).map(|x| x.round() as usize).unwrap_or(2).max(1);
                    let 列长 = 3 * n面;
                    let mut t: Vec<Vec<f64>> = Vec::new();
                    let ok = v.len() >= 列长 && v.len() % 列长 == 0 && v.iter().all(|x| x.is_finite());
                    if ok {
                        for c in v.chunks_exact(列长) { t.push(c.to_vec()); }
                    }
                    if ok && !t.is_empty() {
                        println!("[装] 通道表装回:{} 个通道 × {} 行({n面} 个接触面 × 横/纵/深)—— 不用重量", t.len(), 列长);
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
                if let Some(v) = j.get("parts_map") {
                    let a = v.nums();
                    if a.len() >= 5 { 部件图装回 = Some(a); }
                }
                if let Some(v) = j.get("finger_thickness_m") {
                    if let Some(x) = v.num() { if x.is_finite() && x > 0.0 { 指厚装回 = Some(x); } }
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
        let n格 = 存标定(&out, &body, &相机们, 探幅, 跨度相机, None, None, None, None, None, None, None, None, None, None);
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
    let n格 = 存标定(&out, &body, &相机们, 探幅, 跨度相机, None, None, None, None, None, None, None, None, None, None);
    println!("[装] 落盘(已量到 {n格} 格 · 本次点名量到 {} 格)", 成.len());

    // ── 🔴🔴 **下命令就去干。** 干到缺某个身体量,它点名要,回上面量完再回来。 ──
    // 观测里给什么指令,就做什么;没有任务名,没有机体名。
    let (眼主机, 眼端口) = match 眼.rsplit_once(':') {
        Some((h, p)) => (h.to_string(), p.parse::<u16>().unwrap_or(8077)),
        None => (眼.clone(), 8077),
    };
    match 服务(&mut plug, &body, &相机们, &眼主机, 眼端口, &out, 读回.as_deref(), &给不出, &mut 手载, &mut 雅载, &mut 通道表, &mut 通道是关节, &mut 部件图装回, &mut 指厚装回, &mut 手上相机装回, &mut 张开装回, &mut 空合装回, &mut 认面装回) {
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
/// 🔴🔴🔴 **在画面上画一张编号网格 —— 动词表删掉之后,模型说的那一句就落在这里。**
///
/// 模型只答"**那个东西最后要落到第几格**"(接触集第③格,见 `eye::段` 头注)。
/// 用格号而不是坐标,是因为本仓已经量过:VLM 定量几何弱、**选择题强** ——
/// 挑物体那一步早就是"第几块"这个形状,这里照抄。
/// 交回每一格的中心(归一化),驱动拿它当伺服目标。
fn 画网格(rgb: &mut [u8], w: usize, h: usize, 列: usize, 行: usize) -> Vec<(f64, f64)> {
    let mut 心 = Vec::with_capacity(列 * 行);
    for r in 0..行 { for c in 0..列 {
        let x0 = c * w / 列;
        let x1 = ((c + 1) * w / 列).saturating_sub(1).max(x0);
        let y0 = r * h / 行;
        let y1 = ((r + 1) * h / 行).saturating_sub(1).max(y0);
        画编号框(rgb, w, h, [x0, y0, x1, y1], r * 列 + c + 1, [64, 200, 255], 1);
        心.push(((x0 + x1) as f64 * 0.5 / w as f64, (y0 + y1) as f64 * 0.5 / h as f64));
    }}
    心
}

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
    通道表: &mut Option<Vec<Vec<f64>>>,
    // 通道是关节,还是末端那六个自由度。**试出来的**(这具机体七个关节命令全零响应)。
    通道是关节: &mut bool,
    // 🔴🔴🔴 **部件图:装回来就不用每集重量一遍。**(BV 数出来的 2026-09-02)
    // 量它占开场 **970 行日志**,身上 10 个通道一个一个推一遍再推回来 ——
    // 那 4 个指通道就是视频里"爪子开合几十次"。存的是拍平的一行数(通道 × 相机 × 5:
    // 框四个数 + 占画幅),掩膜不存(只有量它的那一段自己读掩膜,而那已经化成一个标量)。
    部件图载: &mut Option<Vec<f64>>,
    // 手指自己在深度上有多厚 —— 「物体算不算在两指之间」的容差(掩膜上量的)。
    指厚载: &mut Option<f64>,
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
    /// **拿解出来的相机把手投回画面,看落点对不对。** 一台好相机必然过,一台歪相机必然不过。
    /// 手在世界哪儿:本体感受白给。手在画面哪儿:刚量的。容差:手自己在画面上有多大(也是量的)。
    /// 三个输入全是量出来的,没有一个填的数。
    fn 眼投自检(e: &point_gen::Eye, 手世界: [f64; 3], 手u: f64, 手v: f64,
                dw: usize, dh: usize, 块: f64) -> Option<bool> {
        let px = e.project(point_gen::P3 { x: 手世界[0], y: 手世界[1], z: 手世界[2] })?;
        let (pu, pv) = (px[0] / dw as f64, px[1] / dh as f64);
        let 差 = (pu - 手u).hypot(pv - 手v);
        // 容差取"手那一块自己有多大" —— 投到它自己身上就算对得上。
        let 容 = 块.max(0.02);
        println!("[服]   相机自检:手在世界 ({:.3},{:.3},{:.3}) ⇒ 投到画面 ({pu:.3},{pv:.3});手实际在 ({手u:.3},{手v:.3}) ⇒ 差 {差:.3} 画幅(容 {容:.3})",
            手世界[0], 手世界[1], 手世界[2]);
        Some(差 <= 容)
    }

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
            let mut e2 = f2.ee.get(手号.get()).copied().unwrap_or(e0);
            let mut 挡 = 挡;
            let mut 走 = ((e2[0]-p0[0]).powi(2)+(e2[1]-p0[1]).powi(2)+(e2[2]-p0[2]).powi(2)).sqrt();
            // 🔴🔴🔴 **一整步零位移 ⇒ 把【转】那一半摘掉再发一次,看是不是它被拒了。**
            //(BC 实测 2026-08-31)
            // BC:`命令走 0.0680 m ⇒ 上一步实到 **0.00000 m**`,砍到 0.0340 m 仍是 **0.00002 m**
            // —— **不是走一半,是一步都不动**;而同一炮粗对准(只发位置、姿态照抄现在这个)
            // `①走完了(还差 0.0113 m)`,真走完 48 厘米。两段唯一的结构差别:
            // 伺服这一步**同时改了手腕朝向**(动[3..6] 那三项),而位置那一半和粗对准是一样的。
            // 位姿命令是**整条被接受或整条被拒**的:朝向解不出来 ⇒ 位置那一半也一起没了。
            // LAB 记过同族:*"大跨度命令 ⇒ 手臂一动不动,不是走一半"*、
            // *"整条姿态被运动规划拒掉(带碰撞检查),手一步不动"*。
            //
            // ⇒ 不猜是哪一半:**先发整步,零位移就把朝向退回原样再发一次**。
            // 动了 ⇒ 这具身体在这个位形上**接受平移、拒绝这个朝向**,如实说出来并继续用平移干活;
            // 还是不动 ⇒ 那才是真的顶住了。**两种情况都不停手,而且日志分得开。**
            if !(走 > 1e-5) && 动.len() > 3 && (3..6).any(|k| 动.get(k).map(|x| x.abs() > 1e-12).unwrap_or(false)) {
                let (f3, 挡3, _) = 落(plug, p, q0, jaw, 等拍)?;
                let e3 = f3.ee.get(手号.get()).copied().unwrap_or(e0);
                let 走3 = ((e3[0]-p0[0]).powi(2)+(e3[1]-p0[1]).powi(2)+(e3[2]-p0[2]).powi(2)).sqrt();
                if 走3 > 走 {
                    println!("[服]      ⚠️ 整步(带转腕)实到 {走:.5} m,**只发平移**实到 {走3:.5} m ⇒ 这个位形上**朝向那一半被拒了**,按平移继续");
                    e2 = e3; 走 = 走3; 挡 = 挡3;
                }
            }
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
    // 交出来的是 (合起来那一块, 两瓣各自):两瓣要分别送到两个接触点上,只有中点不够用。
    let 看爪像素 = |plug: &mut Plug<S>, at: [f64; 3], q: [f64; 4], j0: f64| -> Vec<Option<(body_layer::hand::Candidate, Option<(body_layer::hand::Candidate, body_layer::hand::Candidate)>)>> {
        let 台 = plug.lay.cams.len();
        let mut out: Vec<Option<(body_layer::hand::Candidate, Option<(body_layer::hand::Candidate, body_layer::hand::Candidate)>)>> = vec![None; 台];
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
                Ok(r) => out[ci] = r.cands.get(0).copied().map(|c| (c, r.halves)),
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
    let (相机号, 各台变, 臂心, 臂末, 臂关, 台变) = {
        let 台 = plug.lay.cams.len();
        if 台 == 0 { println!("[服] 🔴 一台相机都没认出来 ⇒ 干不了。**不编数。**"); return None }
        let Some(f0) = plug.sense() else { return None };
        let 末数 = f0.ee.len().max(1);
        let 组号 = 真臂(&f0);
        let mut 变: Vec<f64> = vec![0.0; 台];
        // 🔴 **每个末端各自让每台相机变了多少** —— 长在这只手上的那台,是这只手一动它整幅都变的那台。
        // 只留一个 max(旧写法)在双手身体上必然错:两只手各有一台腕相机。
        let mut 台变: Vec<Vec<f64>> = vec![vec![0.0; 台]; 末数];
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
                        台变[i][c] = v;
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
        for a in 0..末数 {
            println!("[服]   第 {a} 个末端一动,各台变了:{:?}", 台变[a].iter().map(|v| (v*100.0).round()/100.0).collect::<Vec<_>>());
        }
        (i, 变, 臂心, 臂末, 臂关, 台变)
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
    // 🔴🔴🔴 **"长在手上的是哪台"必须【按这一炮用的那只手】定,不能是一个存下来的数字。**
    //(AZ 实测 2026-08-31)
    // AZ:`[装] 长在手上的是第 2 台相机 —— 不用重探`(上一炮存的),而这一炮挑中的是**第 1 只手**。
    // 后果:收尾拿着**另一只手**的腕相机去找球 ⇒ `几何切出 1 块 … 深 **1.783 m**`、
    // `手指那一点自己有多深:**1.592 m**` —— 球在 0.645 m、爪子该在 0.1 m 上下,
    // 这台相机根本没对着工作区。日志每一行都正常。
    // ⇒ 挑手那一步之后,用**逐末端探针已经量到的** `台变[手号]` 重挑:
    //   这只手一动、整幅画面都在变的那台,就是长在它身上的。写死的下标第三次静默失效。
    let 手上相机 = std::cell::Cell::new(if let Some(c) = *手上相机载 {
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
    });
    *手上相机载 = 手上相机.get();
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
    /// **一小片区域自己在深度上有多厚** —— 远四分位减近四分位。
    /// 用来回答"手指自己有多厚",而那正是"物体算不算在两指之间"的容差(见 `收尾腕` 头注)。
    let 片厚2 = |plug: &mut Plug<S>, ci: usize, u: f64, v: f64, 窗: f64| -> Option<f64> {
        let 路 = plug.lay.cams.get(ci)?;
        let mut 深路 = 路.clone();
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
        if 有.len() < 4 { return None }
        有.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        Some(有[有.len() * 3 / 4] - 有[有.len() / 4])
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
        // 🔴🔴🔴 **没有"上一眼"时,锚点用【逐末端探针量到的这只手在画面哪一片】。**
        //(BA 实测 2026-08-31;LAB 原话:*"场景里不止一个'像手的东西'时,
        //  要用一个【已经量到的、指向本体】的量去锚定它"*)
        //
        // BA:挑手选了**手 1**(探针量到它在主眼里占 (0.621,0.732)),而伺服跟的那一点是
        // **(0.233,0.565)** —— 那是**手 0** 那一片(探针量到 (0.134,0.739))。
        // 于是它在把**另一只手**往球上送,`横纵差 0.4261` 十步一个数字都不变。
        // 双臂身体上"最强的那个候选"本来就可能是另一只手,这不是噪声,是场景里真有两只手。
        // 🔴🔴🔴 **候选必须离【我这只手】比离任何别的手都近 —— 而且每一拍都要这么挑,不只第一拍。**
        //(BD 实测 2026-09-01)
        // 上一版只在"没有上一眼"时才拿这只手的那一片当锚;而一旦第一眼锁到了另一只手上,
        // 之后每一拍都"离上一眼最近" ⇒ **一路跟着错的那只手走到底**。
        // BD:挑手选了手 1(量到它在 (0.621,0.732)),伺服跟的却是 (0.217,0.565) = 手 0 那一片,
        // `横纵差 0.4397` 十步不变 —— 它在把另一只手往球上送。
        //
        // 判据不含任何阈值:**离我这只手的那一片,比离任何别的手的那一片更近**。
        // 逐末端探针已经把每只手在这台相机里占哪一片量出来了(它们相隔约半个画幅)。
        // 一只手的身体上这条恒真(没有"别的手"),N 只手同样成立。
        let 我片 = 臂心.get(手号.get()).and_then(|行| 行.get(相机号)).and_then(|o| *o);
        let 别的片: Vec<(f64, f64)> = 臂心.iter().enumerate()
            .filter(|(i, _)| *i != 手号.get())
            .filter_map(|(_, 行)| 行.get(相机号).and_then(|o| *o)).collect();
        let 是我的 = |u: f64, v: f64| -> bool {
            match 我片 {
                None => true,
                Some((mu, mv)) => {
                    let d我 = (u - mu).hypot(v - mv);
                    别的片.iter().all(|(bu, bv)| d我 <= (u - bu).hypot(v - bv))
                }
            }
        };
        let 锚 = 上眼.or(我片);
        let c = match 锚 {
            None => 头,
            Some((pu, pv)) => {
                let (mut best, mut bd) = (头, f64::MAX);
                let mut 剔 = 0usize;
                for i in 0..r.cands.len() {
                    let Some(k) = r.cands.get(i) else { continue };
                    // 落在别人手那一片上的,直接不算候选(见上面头注)。
                    if !是我的(k.u, k.v) { 剔 += 1; continue }
                    let d = ((k.u - pu).powi(2) + (k.v - pv).powi(2)).sqrt();
                    if d < bd { bd = d; best = *k }
                }
                if 剔 > 0 { println!("[服]   看爪:{剔} 个候选落在**别的手**那一片上,剔掉"); }
                if bd == f64::MAX {
                    println!("[服]   看爪:所有候选都落在别的手上 ⇒ 这一眼不算数");
                    return None
                }
                if r.cands.len() > 1 {
                    println!("[服]   看爪:{} 个候选,挑离**这只手量到的那一片**({pu:.3},{pv:.3}) 最近的那个(差 {bd:.3} 画幅)", r.cands.len());
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
    // 末端位姿的切空间维数:三个平移 + 三个转动 = 6。这是"一个位姿"的数学性质,**无量纲**,不是身体常数;
    // 关节模式下通道数从观测里数,这里只管末端模式。以前这个 6 在七处各写一遍。
    let 末端维: usize = 6;

    // 🔴🔴🔴 **"物体真的在两指之间"这件事,必须是【正面证据】才准合爪。**(BV 实测 2026-09-02)
    //
    // BV 日志:`[收尾] 这台相机的深度图里**一块鼓出来的东西都没有**(只有平面)⇒ 目标不在视野里`
    // 紧接着 `⇒ 直接合,结果照实报` ⇒ **合了一把空气**。整炮 4 次空合全是这么来的。
    // 手上那台看得见自己(接触面占 11.3%、是个常数),而**球早就不在它画面里了** ——
    // 那正是 README 点名的头号阻塞,这一炮第一次量到它发生的**那一拍**。
    //
    // ⇒ 看不见就**不许合**。合一把空气不是"照实报",它是**用掉一次机会去确认一件已经知道的事**;
    //   而把这句话原样交给模型,它才有得决定(退回去 / 换个下手点 / 换只手,都由它解)。
    // 仓里已有半条:*"看不见【自己】的时候要说出来"* —— 这里补上另一半:**看不见【要抓的东西】。**
    let 到位了 = std::cell::Cell::new(false);
    // 🔴🔴🔴 **跟丢了不是"再写一条找回来的规则",是【一个把模型叫醒的事件】。**
    //(owner 2026-09-02:*"机器人的脑子就是 vlm … vlm 必须是我们 driver 的绝对主角"*)
    //
    // "哪个是刚才那个"这个身份**本来就是模型给的**(它从编号块里挑的那一块)。
    // 中间那些步不能每步都问它(看一眼 5 秒,而收尾一次走十几步、探三个通道,
    // 全问它一次抓取要四分钟,而官方一集只给 200 步)⇒ 中间由量出来的连续性扛着走。
    // **扛不住的那一刻,唯一正确的动作是把画面递回去让它重挑** ——
    // 而不是我再手写一条兜底规则(那是永远写不完的那条路)。
    // 换成无人机、格斗同样成立:跟丢了就抬头看一眼。
    let 跟丢了 = std::cell::Cell::new(false);
    // 上一次"挪块到格"的结果,原样进给模型的汇报(它说"直到碰上",身体碰上了就得告诉它)。
    let 挪块结果 = std::cell::RefCell::new(String::new());

    // 🔴🔴🔴 **腕相机收尾:提成具名闭包,因为它现在有【两个】入口。**(2026-08-29)
    // 原来它内联在"追完之后",而追要先过 `看爪` 那道闸 —— 头相机上那道闸赢不了
    // (YF 实测 344 连败,这一段 0 次被执行)。提出来之后,`看爪` 失败的分支也能直接进。
    // (收尾腕 已删:腕相机收尾那条手写路 —— owner 2026-09-03 令删光所有手写动作路)

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
    // (挪进画面 已删:同上)
    // 🔴🔴🔴 **「把我身上的第 k 块挪到第几格」—— 模型说,身体做。**(owner 2026-09-02:
    //  *"把你手写的规则全部删掉,让 vlm 彻底成为主角"*)
    //
    // 这一个函数**替掉了我手写的三个触发器**,它们原来都长成同一句话
    //(*"某某失败了 ⇒ 【我】决定挪一步"*):
    //   ① `白转 >= 3`(空转两拍就挪)—— 那个 3 是我拍的
    //   ② 认不出爪子第 N 次就挪
    //   ③ 认不出接触面就挪
    // 现在这三处一律改成:**照实报给模型,由它说下一段动哪一块、去哪一格。**
    //
    // 做法本身零常数:部件图是**按通道**索引的 ⇒ "我身上第 k 块"就是"第 k 号通道管的那一块"
    // ⇒ **挪它 = 推那个通道**。正反各推一步,看那一块在画面里**离目标格是近了还是远了**,
    // 留下近的那一边。判据是"离得近不近",不是"画面变得多不多" —— 后者是我原来那个,
    // 它只保证"动了",不保证"往对的方向动"。
    // (挪块到格 已删:单块贪心推法,被通用执行器取代 —— owner 2026-09-03 令删光所有手写动作路)

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
    // 相机:从身体文件装回的那台(自标定时解出来的);没有就 None,"上/下"退回"朝相机那边",手腕那一号不列。
    let mut 眼稳: Option<point_gen::Eye> = 相机们.iter().find(|(i, _, _)| *i == 相机号)
        .map(|(_, e, _)| point_gen::Eye { fx: e.fx, fy: e.fy, cx: e.cx, cy: e.cy, at: e.at, q: e.q });
    // 支撑面法向(世界系),减支撑面那一步量到的;"上/下"这两个方位词全靠它。没量到就退回"朝相机那边"。
    let 桌面法向: std::cell::Cell<Option<[f64; 3]>> = std::cell::Cell::new(None);
    // 执行器上一段最后接受的步子比例(这具身体一步吃得下多少是量出来的,跨段带着走)。
    let 步缩: std::cell::Cell<f64> = std::cell::Cell::new(1.0);
    let 问段次: std::cell::Cell<u32> = std::cell::Cell::new(0);
    // 每只手的手指此刻在画面哪儿(框,归一化):执行器每一步跟着更新;跟丢了就记到 需抖,下一轮抖一下那只手的抓握重新认。
    let 身位: std::cell::RefCell<std::collections::HashMap<usize, [f64; 4]>> = std::cell::RefCell::new(std::collections::HashMap::new());
    let 需抖: std::cell::RefCell<std::collections::HashSet<usize>> = std::cell::RefCell::new(std::collections::HashSet::new());
    // 每只手的两瓣此刻各在画面哪个框里(按 u 排),抖抓握时认出来;跟丢就删。执行器把两瓣分别送到两个接触点上,朝向是这么长出来的。
    let 瓣位: std::cell::RefCell<std::collections::HashMap<usize, Vec<[f64; 4]>>> = std::cell::RefCell::new(std::collections::HashMap::new());
    // 每只手抖抓握时那一片沿开合方向有多宽(画幅)+ 它的深度:钳口能张多开的**上界**(含瓣自己的宽,不是拟出来的)。
    let 张幅: std::cell::RefCell<std::collections::HashMap<usize, (f64, f64)>> = std::cell::RefCell::new(std::collections::HashMap::new());
    // 身上每一块对每个通道的响应(跨段记住,只在缺的时候探;每一步用实际发生的修):键 = 部件图通道号,值 = 列数 × 3 摊平。
    let 身表: std::cell::RefCell<std::collections::HashMap<usize, Vec<f64>>> = std::cell::RefCell::new(std::collections::HashMap::new());
    // 执行器从自己的探针里解出来的相机(手指块对平移通道的响应 + 本体报的位置),解出来就留着用。
    let 新眼: std::cell::RefCell<Option<point_gen::Eye>> = std::cell::RefCell::new(None);
    // 这一步要发给某只手的抓握开度(合/张和移动同一个循环里一起做)。
    let 爪令: std::cell::Cell<Option<(usize, f64)>> = std::cell::Cell::new(None);
    // 世界里的块跨轮对号:槽位就是它的号,同一块下一轮还是同一个号;不见了槽位留空(不列),新块占新槽。
    let 世界槽: std::cell::RefCell<Vec<Option<point_gen::区>>> = std::cell::RefCell::new(Vec::new());
    // 这一轮在哪台相机里列块、问模型、执行(模型可点名换相机;换了下一轮生效)。
    let 工作相机: std::cell::Cell<usize> = std::cell::Cell::new(相机号);
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
    let mut 跟模2: Option<Vec<Vec<u8>>> = None;
    let (mut 跟半, mut 跟半手) = (0usize, 0usize);
    let mut 跟块 = 0.0f64;
    let mut 跟上手 = (0.0f64, 0.0f64);
    let mut 跟上面: Vec<(f64, f64)> = Vec::new();
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
    // ══════════════════════════════════════════════════════════════════════════════
    // 🔴🔴🔴 **部件图:冻住别的、一次只动一个通道,记下每台相机里【哪一片跟着动】。**
    //(owner 2026-09-01 定"三条一次做完")
    //
    // **这是"机器人怎么知道自己是机器人"这篇论文的主贡献那一块。**
    // 自由授权搜索(2026-09-01,结论写在 README)扫完之后:
    //   · 「机器人自己量出自己的身体」那条线(Lipson 2019/2022 · π-graphs · AutoURDF · MAVRIC · DIJE)
    //     **一个大模型都没有**,而且每换一具身体都要重训一次;
    //   · 「大模型当机器人的脑子」那条线(HumanCLAW · Butter-Bench · Anthropic · FAEA)
    //     **从来不去量自己的身体** —— HumanCLAW 原话:*"No current VLM perceives its own body …
    //     it behaves like a ghost: it holds no instinctive model of which pixels are its own limbs,
    //     and it never tries to infer where they are."*
    //   · **没有任何工作把这两半接起来。零篇。**
    //
    // 这一段就是那半:**因果地**指认自己。判据是这一层通篇在用的那条构造性身份 ——
    // **别的全冻住,只动一个通道,会动的那一块【按构造】就是这个通道管的部件。**
    // 不认识"手臂""爪子""关节",不读 URDF,不看 CAD,一个人手填的数都没有。
    //
    // 🔴 **通道是这具机体【真正接受】的那些**(`通道是关节` 是试出来的),
    //    加上它报出来的每一个指通道。换五指手就是五个指通道,换吸盘就是一个,
    //    换轮式就是轮子的那几个 —— 同一段代码。
    //
    // 🔴 **"跟着动"用【来回】判,不用单向差**:出去一趟、回来一趟,
    //    某个像素只有**两次都变、方向相反、而且回得来**才算数。
    //    单向的漂移(上一条命令还没走完、光照变化、别的东西被碰动)**自己被筛掉**。
    //    这条和张合相减那一处同一个来源(AP 实测:单向差让 34% 的画面被判成"手指")。
    //
    // 输出:每个(通道 × 相机)一张**粗掩膜** + 框 + 占画幅。粗到 1/4 分辨率就够 ——
    // 它要回答的是"哪一片",不是"哪一个像素"。
    struct 一件 { 掩: Vec<u8>, mw: usize, mh: usize, 框: [f64; 4], 占: f64 }
    let 量部件图 = |plug: &mut Plug<S>, 通道数: usize, 是关节: bool, 指数: usize|
        -> Vec<Vec<Option<一件>>> {
        let 台 = plug.lay.cams.len();
        let mut 出: Vec<Vec<Option<一件>>> = Vec::new();
        // 一次只动一个通道:先是身体那几个,再是每一个指通道。
        // 🔴 每个通道晃它自己那只手(CN 实测 2026-09-03:原来四个手指通道全晃当前选中的那只手,"第 1 只手的手指"其实是第 2 只手的像素)。
        //    末端模式:通道 k ⇒ 第 k/末端维 只手的第 k%末端维 个自由度;手指通道 k ⇒ 第 (k−通道数)×臂数/指数 只手的抓握。
        let 臂数 = 臂末.len().max(1);
        let (原手, 原臂) = (手号.get(), 臂号.get());
        for k in 0..(通道数 + 指数) {
            let 是指 = k >= 通道数;
            let a = if 是指 { ((k - 通道数) * 臂数 / 指数.max(1)).min(臂数 - 1) } else if 是关节 { 0 } else { (k / 末端维).min(臂数 - 1) };
            if let Some(Some(e)) = 臂末.get(a) { 手号.set(*e); }
            if let Some(&j) = 臂关.get(a) { 臂号.set(j); }
            let 前: Vec<Option<(usize, usize, Vec<u8>)>> =
                (0..台).map(|c| plug.sense().and_then(|f| 灰(&f, c))).collect();
            let 前深: Vec<Option<Vec<f64>>> = (0..台).map(|c| 深图(plug, c)).collect();
            let j0 = plug.sense().and_then(|f| f.jaw.get(手号.get()).copied()).unwrap_or(1.0);
            // 幅度:身体通道用量出来的探幅(上界也是它,见梯子那一段);
            // 指通道用它自己的命令域两端(0..1 是命令域,不是米)。
            if 是指 { 抓握(plug, if j0 > 0.5 { 0.0 } else { 1.0 }, 是关节); 定爪(plug, 等拍 * 4); }
            else {
                let 长 = if 是关节 { 通道数 } else { 末端维 };
                let mut 动 = vec![0.0; 长]; 动[if 是关节 { k } else { k % 末端维 }] = 探幅;
                let _ = 迈通道(plug, &动, 1.0, j0, 是关节);
                等停(plug, 等拍 * 2);
            }
            let 中: Vec<Option<(usize, usize, Vec<u8>)>> =
                (0..台).map(|c| plug.sense().and_then(|f| 灰(&f, c))).collect();
            let 中深: Vec<Option<Vec<f64>>> = (0..台).map(|c| 深图(plug, c)).collect();
            // 回来
            if 是指 { 抓握(plug, j0, 是关节); 定爪(plug, 等拍 * 4); }
            else {
                let 长 = if 是关节 { 通道数 } else { 末端维 };
                let mut 动 = vec![0.0; 长]; 动[if 是关节 { k } else { k % 末端维 }] = -探幅;
                let _ = 迈通道(plug, &动, 1.0, j0, 是关节);
                等停(plug, 等拍 * 2);
            }
            let 后: Vec<Option<(usize, usize, Vec<u8>)>> =
                (0..台).map(|c| plug.sense().and_then(|f| 灰(&f, c))).collect();
            let 后深: Vec<Option<Vec<f64>>> = (0..台).map(|c| 深图(plug, c)).collect();
            let mut 这件: Vec<Option<一件>> = Vec::new();
            for c in 0..台 {
                let (Some((w, h, a)), Some((_, _, b)), Some((_, _, d))) =
                    (前[c].clone(), 中[c].clone(), 后[c].clone()) else { 这件.push(None); continue };
                if a.len() != b.len() || b.len() != d.len() || a.is_empty() { 这件.push(None); continue }
                let (深前, 深中, 深后) = (前深[c].clone(), 中深[c].clone(), 后深[c].clone());
                // 粗到 1/4:一格 4×4 像素,格里过半的像素满足条件就算这一格动过。
                let (mw, mh) = ((w / 4).max(1), (h / 4).max(1));
                let mut 掩 = vec![0u8; mw * mh];
                let (mut x0, mut y0, mut x1, mut y1) = (1.0f64, 1.0f64, 0.0f64, 0.0f64);
                let mut 计 = 0usize;
                for gy in 0..mh { for gx in 0..mw {
                    let mut n = 0usize; let mut 总 = 0usize;
                    for dy in 0..4 { for dx in 0..4 {
                        let (px, py) = (gx * 4 + dx, gy * 4 + dy);
                        if px >= w || py >= h { continue }
                        let i = py * w + px; 总 += 1;
                        let d1 = b[i] as i32 - a[i] as i32;   // 出去
                        let d2 = b[i] as i32 - d[i] as i32;   // 回来(相对中间)
                        // 8 是灰度差的噪声底(和别处同一个来源),无量纲。
                        if d1.abs() <= 8 || d2.abs() <= 8 { continue }
                        if d1.signum() != d2.signum() { continue }   // 单向漂,不是我
                        let 回 = (a[i] as i32 - d[i] as i32).abs();
                        if 回 >= d1.abs().min(d2.abs()) { continue } // 没回来,不是我
                        // 🔴🔴🔴 **"动一下再回来"分不出【我】和【我挡住过的东西】。**(BE 渲图定案 2026-09-01)
                        //
                        // 胳膊扫过桌面时,被它盖住的那些桌面像素**也是"变了又变回来"** ——
                        // 判据对它们同样成立。BE 那张部件图里一大片彩色**盖在桌面上**、压到剪刀和球附近,
                        // 一眼就看得出来。这是"靠运动认自己"这条路的真缺陷,不是实现 bug。
                        //
                        // 用**深度**分开,判据是物理的:**我挡在别人前面**。
                        //   · 是我:起点/回来那两帧我在这儿(近),中间那帧我走开了(**变远**,露出后面的东西)
                        //   · 被我挡过的桌面:反过来 —— 中间那帧我盖在它上面(**变近**)
                        // ⇒ 只留"中间比两头**远**"的那些格。一个身体词都没有。
                        if let (Some(za), Some(zb), Some(zd)) = (深前.as_ref(), 深中.as_ref(), 深后.as_ref()) {
                            if za.len() > i && zb.len() > i && zd.len() > i {
                                let (fa, fb, fd) = (za[i], zb[i], zd[i]);
                                if fa.is_finite() && fb.is_finite() && fd.is_finite() {
                                    // 门槛用这台深度图自己的抖:起点和回来这两帧同一点的差。
                                    let 抖 = (fa - fd).abs();
                                    if !(fb > fa + 抖 && fb > fd + 抖) { continue }
                                }
                            }
                        }
                        n += 1;
                    }}
                    if 总 > 0 && n * 2 > 总 {
                        掩[gy * mw + gx] = 255; 计 += 1;
                        let (u, v) = (gx as f64 / mw as f64, gy as f64 / mh as f64);
                        if u < x0 { x0 = u } if v < y0 { y0 = v }
                        if u > x1 { x1 = u } if v > y1 { y1 = v }
                    }
                }}
                let 占 = 计 as f64 / (mw * mh) as f64;
                这件.push(if 计 > 0 { Some(一件 { 掩, mw, mh, 框: [x0, y0, x1, y1], 占 }) } else { None });
            }
            for (c, it) in 这件.iter().enumerate() {
                match it {
                    Some(x) => println!("[身] 通道 {k}{} 在第 {c} 台里动的那一片:占画幅 {:.4} · 框 ({:.2},{:.2})-({:.2},{:.2})",
                        if 是指 { "(指)" } else { "" }, x.占, x.框[0], x.框[1], x.框[2], x.框[3]),
                    None => println!("[身] 通道 {k}{} 在第 {c} 台里**一格都没动**", if 是指 { "(指)" } else { "" }),
                }
            }
            出.push(这件);
        }
        手号.set(原手); 臂号.set(原臂);
        出
    };
    let mut 挑过手 = false;
    // 上一段干完之后要汇报给模型的那句话(它下一次的输入)。
    let mut 上一段汇报: String = "this is the first segment; nothing has been tried yet.".into();
    // 上一次"我在画面哪"是**真量到的**,还是**看不见、拿别的办法顶上的**?
    // 这一格进汇报 —— 看不见自己的时候必须说出来(见 `量爪位` 那一段的头注)。
    let 爪位可信 = std::cell::Cell::new(false);
    // 上一次朝哪个方向走的(单位向量)—— "退回我来的路"用它,不用任何记住的位姿。
    let mut 退向: (f64, f64, f64) = (0.0, 0.0, 0.0);
    let 爪图号 = std::cell::Cell::new(0u32);
    // 手指自己在深度上有多厚(腕相机里是常数)—— "物体算不算在两指之间"的容差。
    let 指厚常数 = std::cell::Cell::new(*指厚载);
    // 部件图拍平之后的那一行字,落盘时原样写进去。
    let 部件图串 = std::cell::RefCell::new(None::<String>);
    // 🔴 装回来的部件图:框 + 占,掩膜是空的(下游不读掩膜,见 `部件图载` 头注)。
    let mut 部件图: Option<Vec<Vec<Option<一件>>>> = 部件图载.as_ref().and_then(|平| {
        let 台 = plug.lay.cams.len();
        if 台 == 0 || 平.len() % (台 * 5) != 0 { return None }
        let mut 出: Vec<Vec<Option<一件>>> = Vec::new();
        for k in 0..(平.len() / (台 * 5)) {
            let mut 行 = Vec::with_capacity(台);
            for c in 0..台 {
                let b = (k * 台 + c) * 5;
                let f = [平[b], 平[b + 1], 平[b + 2], 平[b + 3]];
                if f.iter().any(|x| !x.is_finite() || *x < 0.0) { 行.push(None); continue }
                行.push(Some(一件 { 掩: Vec::new(), mw: 0, mh: 0, 框: f, 占: 平[b + 4] }));
            }
            出.push(行);
        }
        println!("[装] 部件图装回:{} 个通道 × {台} 台相机 —— **不用再把身上每个通道推一遍**(那是开场 970 行的来源)", 出.len());
        if let Some(t) = 指厚常数.get() { println!("[装] 手指自己在深度上厚 {t:.4} m(装回来的)—— 「到位」的容差直接用它"); }
        Some(出)
    });
    // 手指自己在深度上有多厚(腕相机里是常数)—— "物体算不算在两指之间"的容差。

    // 🔴 **"东西现在在不在我手里"** —— 动词表删掉之后,"这一段是去够它还是带着它走"
    // 由**这个量出来的事实**决定,不由任何一个词决定。判据是合到停住的读数离
    // "空手合到底"有多远(零尺度,换什么爪子都成立)。
    let 手里有 = std::cell::Cell::new(false);
    // 模型这一段说的"多快 / 到什么为止 / 别碰哪几块"(owner 2026-09-03:这一版必须能做全部目标)
    let 快 = std::cell::Cell::new(false);
    let 到什么为止值 = std::cell::RefCell::new(String::from("amount"));
    let 别碰框: std::cell::RefCell<Vec<point_gen::区>> = std::cell::RefCell::new(Vec::new());
    // 🔴🔴 **记忆层接入**(owner 2026-09-03:"记忆层早就完美了" —— 层是完整的,驱动此前调用 0 次)。
    // 任务记忆(`Scope::Task`):换集触发 `NewTask`;每拍 `observed()`(机械钉死靠它);
    // 格子里只写**不会自己动**的事实(任务、目标是什么、试过几个下手点、手里有没有、上一步结果),
    // "球现在在哪"这种会动的**不写**(写了会被拒,这正是那层的设计)。渲染成一段话随提示词给模型。
    let mut 记忆 = body_layer::memory::Memory::new(body_layer::memory::Scope::Task);
    for (名, 钉) in [("task", true), ("target", false), ("tried", false), ("holding", false), ("last", false)] {
        if let Err(e) = 记忆.declare(名, 钉) { println!("[忆] 声明格子 {名} 被拒:{e:?}"); }
    }
    let 忆写 = |记忆: &mut body_layer::memory::Memory, 名: &str, 值: &str| {
        let 短: String = 值.chars().take(60).collect();   // 一格 64 字节
        let 短 = if 短.len() > 62 { 短.as_bytes()[..62].iter().map(|&b| if b < 128 { b as char } else { '?' }).collect() } else { 短 };
        match 记忆.write(名, &短, body_layer::memory::Durability::Durable) { Ok(_) => {}, Err(e) => println!("[忆] 写 {名} 被拒:{e:?}") }
    };
    let 忆文 = |记忆: &body_layer::memory::Memory| -> String {
        let mut t = String::from("MEMORY (durable facts you wrote down earlier; look again for anything that moves):\n");
        for 名 in ["task", "target", "tried", "holding", "last"] {
            if let Some(v) = 记忆.get(名) { t.push_str(&format!("  {名}: {v}{}\n", if 记忆.is_pinned(名) { " (pinned)" } else { "" })); }
        }
        t
    };
    // 🔴🔴🔴 **飞行员模式**(`BL_PILOT=1`,owner 2026-09-03 定):每帧一张白纸问一个方向,身体走一步,回来再问。
    // 没有收尾伺服、没有交接、没有我的任何流程 —— 每一步都是模型自己的一个字。
    // `BL_PILOT=3`:**完整的机器**(两根手指各去自己那一点、六行一起解、朝向解出来)+ **每一步先问模型**
    //(owner 2026-09-03:"图省事让任务没完成" —— v1/v2 只瞄一个点、只用三行,把朝向整个删了,横着的爪子永远抓不起球)
    // `BL_PILOT=2`:问两个格号,方向/步数驱动算(第二版);`=1`:问一个方向码(第一版,CH)
    // 🔴 指尖在画面哪儿,**驱动自己量**,不问模型。(CI 实测:它一路说指尖在第 47/48 格 = 底座那一角,
    //  手臂伸到桌子中间了还说 48;它认的是粗腕管/底座,不是末端两片小黑楔。)
    // 量法就是部件图那条判据逐步用:我一动,画面里跟着动的那一片的重心 = 我的手现在在哪。
    let 手在画面 = std::cell::Cell::new(None::<(f64, f64)>);
    // 🔴 **腕相机里"我的接触面在哪" —— 从部件图里取那个常数。**
    // 取的是**指通道**(只开合爪子那几个)在**长在手上那台**里动的那一片的中心。
    // 那台相机长在手上 ⇒ 这个位置不随姿势变 ⇒ 量一次就够(见 `收尾腕` 里的头注)。
    let 腕爪常数 = |图: &Option<Vec<Vec<Option<一件>>>>, 腕机: usize, 通道数: usize| -> Option<(f64, f64, f64)> {
        let 图 = 图.as_ref()?;
        let (mut su, mut sv, mut n) = (0.0f64, 0.0f64, 0usize);
        let (mut x0, mut y0, mut x1, mut y1) = (1.0f64, 1.0f64, 0.0f64, 0.0f64);
        for (k, 件们) in 图.iter().enumerate() {
            if k < 通道数 { continue }          // 只看指通道
            if let Some(Some(x)) = 件们.get(腕机) {
                su += (x.框[0] + x.框[2]) * 0.5; sv += (x.框[1] + x.框[3]) * 0.5; n += 1;
                if x.框[0] < x0 { x0 = x.框[0] } if x.框[1] < y0 { y0 = x.框[1] }
                if x.框[2] > x1 { x1 = x.框[2] } if x.框[3] > y1 { y1 = x.框[3] }
            }
        }
        // 第三个数:**手指那一片在画面上有多大**(占画幅)—— 下面拿它当读深度的窗,
        // 算出"手指自己在深度上有多厚",那才是"到位"的容差(见 `收尾腕` 里那段头注)。
        if n == 0 { None } else { Some((su / n as f64, sv / n as f64, (x1 - x0).max(y1 - y0).max(0.01))) }
    };
    loop {
        let Some(帧) = plug.sense() else { return None };
        if plug.复位过 { plug.复位过 = false; 集 += 1; 试过.clear(); println!("[服] ── 第 {集} 集 ──");
            记忆.on_event(body_layer::memory::Opens::NewTask); 手里有.set(false); }
        记忆.observed();
        忆写(&mut 记忆, "tried", &format!("{} grasp points tried this task", 试过.len()));
        忆写(&mut 记忆, "holding", if 手里有.get() { "something is between my fingers" } else { "nothing between my fingers" });
        if !上一段汇报.is_empty() { 忆写(&mut 记忆, "last", &上一段汇报); }
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
        if 记忆.get("task").is_none() { 忆写(&mut 记忆, "task", &指令); }
        let Some((w, h, rgb)) = 彩(&帧, 工作相机.get()) else { continue };

        // 🔴🔴🔴 **减法③:候选区域由【几何】出,眼只负责【挑一个】。**(owner 2026-08-28)
        //
        // 以前是"问眼要一个框",框的边界是模型脑补的。量过的代价:
        //   · 让模型给坐标 **0.403 ≈ 瞎猜** vs 让它从选项里挑 **0.916 ≈ 超人类**(同一批模型)
        //   · 脑补出来的坐标,**对的和错的长得一模一样**;而挑错一个画在图上的框,渲一张图就看得见
        //   · 框自带边界 ⇒ **不用再猜半径** —— 而"猜出来的半径"正是把桌面圈进点云的那一个
        // 候选从**深度图**里长出来:空间里一张平面满足 `1/z` 关于像素线性,拟出支撑面,
        // 比它鼓出来的连通块就是一个个物体。**不认识任何物体名,也没有任何检测器。**
        let 区们: Vec<point_gen::区> = plug.lay.cams.get(工作相机.get()).and_then(|路| {
            let mut 深路 = 路.clone();
            // 只用这台相机自己的深度;别台的深度图和这张图对不上,不借。
            let Some(dp) = plug.lay.depth.get(工作相机.get()) else { return None };
            深路 = dp.clone();
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
        // 没深度的相机(无人机那类):世界里的块 = 眼给的框(一块),深度未知(NaN);执行器对它只解横纵、不解深。
        let 区们: Vec<point_gen::区> = if 区们.is_empty() && plug.lay.depth.get(工作相机.get()).is_none() {
            match body_layer::eye::ask(眼主机, 眼端口, &指令, &rgb, w, h) {
                Ok(l) => {
                    let b = [(l.box01[0] * w as f64) as usize, (l.box01[1] * h as f64) as usize, (l.box01[2] * w as f64) as usize, (l.box01[3] * h as f64) as usize];
                    println!("[服] 🎯 这台相机没有深度 ⇒ 世界里的块 = 眼给的框 ({:.3},{:.3})-({:.3},{:.3})", l.box01[0], l.box01[1], l.box01[2], l.box01[3]);
                    vec![point_gen::区 { 框: b, 像素数: (b[2].saturating_sub(b[0]) + 1) * (b[3].saturating_sub(b[1]) + 1), 心: [l.u, l.v], 深: f64::NAN, 高: f64::NAN }]
                }
                Err(e) => { println!("[服] 这台相机没有深度,眼也没给框:{e}"); Vec::new() }
            }
        } else { 区们 };

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
                    忆写(&mut 记忆, "target", &format!("the thing the task refers to: {} px, standing {:.3} m out of the surface", r.像素数, r.高));
                    look = Some(body_layer::eye::Look {
                        u: (x0 + x1) * 0.5, v: (y0 + y1) * 0.5, span_frac: 长边,
                        box01: [x0, y0, x1, y1],
                    });
                    // 掩膜按**这一块自己的深度带**切:一个物体不会比自己更厚,
                    // 而"多厚"是量出来的(它鼓出背景多高),不是一个宽容系数。
                    选中掩膜 = plug.lay.cams.get(工作相机.get()).and_then(|路| {
                        let mut 深路 = 路.clone();
                        // 只用这台相机自己的深度;别台的深度图和这张图对不上,不借。
            let Some(dp) = plug.lay.depth.get(工作相机.get()) else { return None };
            深路 = dp.clone();
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
        let mut look = match look {
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
        // 🔴 用哪只手不由驱动挑(原来是"离目标最近的那条臂",owner 2026-09-03 令删):模型点名哪只手上的块,执行器就动哪只手。
        if !挑过手 {
            挑过手 = true;
            // 🔴🔴🔴 **手挑定了,立刻量一次部件图 —— 这一炮只量一次,量完存着用。**
            //(见 `量部件图` 头注:这是"机器人怎么知道自己是机器人"的那一半)
            if 部件图.is_none() {
                let 指数 = 帧.jaw.len();
                let 通道数 = if *通道是关节 {
                    plug.sense().map(|f| 真臂(&f).iter().map(|&i| f.joints[i].len()).sum::<usize>()).unwrap_or(0)
                } else { 臂末.len().max(1) * 末端维 };
                println!("[身] ── 量部件图:{通道数} 个身体通道 + {指数} 个指通道,一次只动一个 ──");
                let 图 = 量部件图(plug, 通道数, *通道是关节, 指数);
                // ── 从因果表里【推】出结构,一个名字都不用 ──
                // ① 这台相机长在哪个通道的下游:那个通道一动,它**整幅**都在变。
                // ② 哪些通道是接触面:指通道动的那一片(按构造就是钳口/吸盘/手指)。
                // ③ 谁在谁下游:A 动的那一片**包含** B 动的那一片 ⇒ B 挂在 A 下面。
                for c in 0..plug.lay.cams.len() {
                    let mut 最 = (usize::MAX, 0.0f64);
                    for (k, 件们) in 图.iter().enumerate() {
                        if let Some(Some(x)) = 件们.get(c) { if x.占 > 最.1 { 最 = (k, x.占) } }
                    }
                    if 最.0 != usize::MAX {
                        println!("[身] 第 {c} 台相机:第 {} 号通道一动它变了 **{:.0}%** 的画面{}",
                            最.0, 最.1 * 100.0,
                            if 最.1 > 0.5 { " ⇒ **它长在这个通道的下游**" } else { "" });
                    }
                }
                for k in 通道数..(通道数 + 指数) {
                    if let Some(件们) = 图.get(k) {
                        for (c, it) in 件们.iter().enumerate() {
                            if let Some(x) = it {
                                println!("[身] **接触面**(第 {k} 号指通道)在第 {c} 台里:占画幅 {:.4} · 中心 ({:.3},{:.3})",
                                    x.占, (x.框[0] + x.框[2]) * 0.5, (x.框[1] + x.框[3]) * 0.5);
                            }
                        }
                    }
                }
                // ── 渲成彩色图:每个通道一个颜色,叠在主眼那张真实画面上 ──
                // 这张图有两个用户:**我们肉眼验它对不对**,以及**交给眼去命名**。
                if let Ok(dir) = std::env::var("BL_DUMP") {
                    for c in 0..plug.lay.cams.len() {
                        let Some((cw, ch, crgb)) = plug.sense().and_then(|f| 彩(&f, c)) else { continue };
                        let mut 图片 = crgb.to_vec();
                        for (k, 件们) in 图.iter().enumerate() {
                            let Some(Some(x)) = 件们.get(c) else { continue };
                            // 颜色只是编号的可视化,不含任何语义:按通道号在色环上取。
                            let 相 = k as f64 * 2.399963;   // 黄金角,通道多也不撞色
                            let col = [((相.sin() * 0.5 + 0.5) * 255.0) as u8,
                                       (((相 + 2.094).sin() * 0.5 + 0.5) * 255.0) as u8,
                                       (((相 + 4.189).sin() * 0.5 + 0.5) * 255.0) as u8];
                            for gy in 0..x.mh { for gx in 0..x.mw {
                                if x.掩[gy * x.mw + gx] == 0 { continue }
                                for dy in 0..4 { for dx in 0..4 {
                                    let (px, py) = (gx * 4 + dx, gy * 4 + dy);
                                    if px >= cw || py >= ch { continue }
                                    let i = (py * cw + px) * 3;
                                    if i + 2 >= 图片.len() { continue }
                                    // 半透明叠色:底下的真实画面还看得见,方便肉眼核对。
                                    for t in 0..3 { 图片[i + t] = ((图片[i + t] as u16 + col[t] as u16) / 2) as u8; }
                                }}
                            }}
                        }
                        let 路 = format!("{dir}/身体_第{c}台.bmp");
                        let _ = std::fs::write(&路, task::bmp24(&图片, cw, ch));
                        println!("[身] 🖼 部件图渲好了 ⇒ {路}(每个通道一个颜色,叠在真实画面上)");
                    }
                }
                // 🔴🔴🔴 **"手指自己有多厚"必须在【掩膜】上读深度,不能在外接框里读。**
                //(BR 实测 2026-09-02:在框里读量出 **0.3740 m** —— 37 厘米,
                // 因为框里除了手指还包着背后的桌面和地板,量到的是"手指到背景的距离"。
                // 容差反而比之前那个 0.068 更松。**同一类错,而且是我刚写的那一版。**)
                //
                // 腕相机长在手上 ⇒ 手指相对它不动 ⇒ **这个厚度也是个常数**,在这里算一次存着。
                if let Some(腕) = 手上相机.get() {
                    // 🔴 每个条件分开打(LAB:"诊断要把每个条件分开打");这一帧没带深度就再拍一帧重读一次
                    //(CA 实测:同一段代码 BT 量到 0.0107,CA 报"读不到" —— 间歇性,最像是那一瞬没深度)。
                    let mut 深 = 深图(plug, 腕);
                    if 深.is_none() { let _ = plug.sense(); 深 = 深图(plug, 腕); println!("[身]   (手指厚度)第一帧没有第 {腕} 台的深度 ⇒ 重拍一帧再读:{}", if 深.is_some() { "有了" } else { "还是没有" }); }
                    let mut 有: Vec<f64> = Vec::new();
                    let mut 掩点 = 0usize; let mut 无效 = 0usize;
                    if let Some(d) = 深.as_ref() {
                        if let Some((cw5, ch5, _)) = plug.sense().and_then(|f| 灰(&f, 腕)) {
                            if d.len() != cw5 * ch5 { println!("[身]   ⚠️ 深度图 {} 个点 ≠ 灰度图 {cw5}×{ch5} —— 尺寸不一致,掩膜对不上", d.len()); }
                            for (k, 件们) in 图.iter().enumerate() {
                                if k < 通道数 { continue }              // 只看指通道
                                let Some(Some(x)) = 件们.get(腕) else { continue };
                                for gy in 0..x.mh { for gx in 0..x.mw {
                                    if x.掩[gy * x.mw + gx] == 0 { continue }
                                    掩点 += 1;
                                    for dy in 0..4 { for dx in 0..4 {
                                        let (px, py) = (gx * 4 + dx, gy * 4 + dy);
                                        if px >= cw5 || py >= ch5 { continue }
                                        let i = py * cw5 + px;
                                        if i < d.len() && d[i].is_finite() && d[i] > 0.0 { 有.push(d[i]) } else { 无效 += 1 }
                                    }}
                                }}
                            }
                        }
                    }
                    if 有.len() >= 4 {
                        有.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
                        let t = 有[有.len() * 3 / 4] - 有[有.len() / 4];
                        println!("[身] **手指自己在深度上厚 {t:.4} m**(只在手指那些像素上读,不含背景)⇒ 这就是「到位」的容差");
                        指厚常数.set(Some(t));
                    } else {
                        println!("[身] ⚠️ 手指那些像素上读不到足够的深度 ⇒ 没有厚度这个数,容差退回读数自己的抖(深度图 {} · 掩膜格 {掩点} · 读到 {} 有效 / {无效} 无效)",
                            if 深.is_some() { "有" } else { "无" }, 有.len());
                    }
                }
                // 拍平存起来:通道 × 相机 × 5(框四个数 + 占),空的写 NaN。
                {
                    let 台 = plug.lay.cams.len();
                    let mut 平: Vec<String> = Vec::new();
                    for 件们 in 图.iter() {
                        for c in 0..台 {
                            match 件们.get(c).and_then(|x| x.as_ref()) {
                                Some(x) => { for q in x.框 { 平.push(format!("{q}")) } 平.push(format!("{}", x.占)); }
                                // 🔴 空格写 -1,**不许写 NaN**:NaN 不是合法 JSON,零依赖解析器会拒收**整份文件**
                                //(CB 实测 2026-09-02:`[装] 读不回 /root/cal.json —— 从零开始标`,通道表/相机/部件图全部重量,
                                //  "越用越强"整个归零)。框坐标都在 0–1 之间,-1 一眼就是"没有"。
                                None => { for _ in 0..5 { 平.push("-1".into()) } }
                            }
                        }
                    }
                    *部件图串.borrow_mut() = Some(平.join(", "));
                }
                // 🔴 **量完就立刻落盘。** 上一版把这一行挂在解相机那条分支里,而干活模式
                // 装回了相机就不走那条 ⇒ `cal.json` 里**根本没有 parts_map**(BY 实测)。
                // 仓里那句"每量完一格就调一次,不要只在最后调"说的正是这件事。
                {
                    let n格 = 存标定(标定文件, body, 相机们, 探步, 0, *手载,
                        通道表.as_deref(), Some(*通道是关节), 手上相机.get(), *张开载, *空合载, Some(*认面载), *雅载,
                        部件图串.borrow().as_deref(), 指厚常数.get());
                    println!("[身] 💾 部件图 + 手指厚度存进 {标定文件}(落了 {n格} 格)⇒ **下一炮装回来,开场那 970 行不用重做**");
                }
                部件图 = Some(图);
            }
        }

        // ══════════════════════════════════════════════════════════════════════════
        // 🔴🔴🔴 **动词表在这里被整片删掉。**(owner 2026-09-02 死命令:*"动词表必须现在删,
        //  不然算作弊"*。我提过"等替代品跑起来再删"的刹车,已被驳回,照删。)
        //
        // 删掉的是三张:这里的 8 个(approach/close/open/retreat/look_around/
        // new_grasp_point/switch_hand/done)· `eye::pick` 的 6 个(grasp/push/place/pry/
        // open/close)· `eye::ask` 的同一份,外加 3 档力度(light/medium/firm)。
        //
        // **替代品不是新发明的,是仓里早就建好并测过的那句话** —— 接触集第③格
        //(`contact-set`,80 个单元测试,十三个动词本来就塌进这一张表):
        //   *"① 碰哪几个点 · ② 每点的法向和锥 · ③ **物体要怎么动** · ④ 容差"*
        // 这里把第③格说成**画面里的一格**:**"那个东西最后要落到第几格。"**
        //
        //   · 抓起来 10 厘米 = 球要落到它上方那一格
        //   · 拿球砸小人     = 球要落到小人所在的那一格
        //   · 换手 / 换下手点 / 退回去 / 张开合上 —— **全都不再是"一件事"**,
        //     它们是驱动为了把东西送到那一格而自己解出来的中间步骤。
        //
        // 🔴 **为什么是"第几格"而不是坐标**:VLM 定量几何弱(RoboVista 最好 56.5%、
        // 30.2% 认错东西),**而选择题强**(同批模型选轨迹 0.916)。挑物体那一步早就是
        // "第几块"这个形状,这里照抄。owner 原话:*"vlm 指的很不准的,而且这还是个两指手,
        // 后面换个 20 指手你就指去吧"* —— **它一次都不用指自己**。
        //
        // 🔴 **到什么为止**那五个仍然由它自己说,而**那五个不是动词,是【事件】**
        //(`verb::Until`,驱动本来就在量):走完 / 碰上 / 推不动 / 东西不再跟着我 / 画面不再变。
        // 换成格斗它会说"直到对方的手碰到我",无人机"直到高度不再变",**一行代码都不用改**。
        //
        // ⚠️ 它答什么都**不承重到控制**:走多少永远由量出来的方向盘算。
        //    问不通 ⇒ 照常往下走,并把理由说出来 —— **不许因为模型不答就停手**。
        // 网格多粗:一格竖着大约是"抬起来"那一步的量级。6×4 在这台相机上一格约 12 cm。
        let (网列, 网行) = (6usize, 4usize);
        let (mut 格心, mut 这一段) = (Vec::new(), None);
        // 这一拍交给模型的编号表:(是不是我身上的, 通道号/区号, 画面位置)
        let mut 条目: Vec<(bool, usize, (f64, f64))> = Vec::new();
        if 部件图.is_some() {
            if let Some((cw4, ch4, crgb4)) = 彩(&帧, 工作相机.get()) {
                let mut 网图 = crgb4.clone();
                格心 = 画网格(&mut 网图, cw4, ch4, 网列, 网行);
                // 🔴🔴🔴 **一张编号表:先是我身上的每一块,再是世界里切出来的每一块。**
                // 模型只答两个号:**动第几号 → 到第几格**。
                // 这一格拓宽之后,我手写的三个触发器(空转就挪 / 认不出爪子就挪 / 认不出接触面就挪)
                // 全部消失 —— 它们都变成了模型能自己说出口的一句话。
                // 表里**没有一个身体参数**(不出现关节角、自由度、几根手指、多长多宽),全是画面语言。
                let 格于 = |u: f64, v: f64| -> usize {
                    let c = (u * 网列 as f64).floor().clamp(0.0, 网列 as f64 - 1.0) as usize;
                    let r = (v * 网行 as f64).floor().clamp(0.0, 网行 as f64 - 1.0) as usize;
                    r * 网列 + c + 1
                };
                // 人一眼看到的"哪只手离目标近",模型从格子号里心算不出来(CS–CU 三炮都挑了够不着的那只手)⇒ 量出来写进去:
                // 离上次定的目标多少格、在画面左半还是右半。这是量,不是策略 —— 挑哪只仍由它定。
                let 离目标 = |u: f64, v: f64| -> String {
                    let d = ((u - look.u) * 网列 as f64).hypot((v - look.v) * 网行 as f64);
                    format!(", {:.1} cells from the thing you settled on, in the {} half of the picture", d, if u < 0.5 { "LEFT" } else { "RIGHT" })
                };
                // 条目:(是不是我身上的, 通道号 or 区号, 画面位置);通道号 usize::MAX = 手腕(末端投影)。
                条目.clear();
                let 臂通道数 = if *通道是关节 { 真臂(&帧).iter().map(|&i| 帧.joints[i].len()).sum::<usize>() } else { 帧.ee.len().max(1) * 末端维 };
                let 哪手0 = |kk: usize| -> usize { let (臂数, 指数) = (帧.ee.len().max(1), 帧.jaw.len().max(1)); if kk >= 臂通道数 { ((kk - 臂通道数) * 臂数 / 指数).min(臂数 - 1) } else { 手号.get() } };
                // 跟丢过手指的那几只手:抖一下它的抓握(手臂冻住),画面里跟着动的那一片就是它的手指 —— 在任何位形上都成立,不靠上一轮的位置。
                {
                    // 还没认出两瓣的手,这一轮先抖一下认出来(每轮最多一次,不是每拍):要把瓣送到接触点上,先得知道瓣在哪。
                    for a in 0..帧.ee.len().max(1) { if !瓣位.borrow().contains_key(&a) { 需抖.borrow_mut().insert(a); } }
                    let 要抖: Vec<usize> = 需抖.borrow().iter().copied().collect();
                    for a in 要抖 {
                        let Some(e) = 臂末.get(a).copied().flatten() else { continue };
                        // 认块器(连通 + 刚性 + 配对),不是散点外接框(CQ 实测:71 个散点把框撑成整幅画面)。
                        let (原手, 原臂) = (手号.get(), 臂号.get());
                        手号.set(e); if let Some(&j) = 臂关.get(a) { 臂号.set(j); }
                        let 得 = plug.sense().and_then(|f| {
                            let ee = f.ee.get(e).copied()?; let j0 = f.jaw.get(e).copied().unwrap_or(1.0);
                            看爪像素(plug, [ee[0], ee[1], ee[2]], [ee[3], ee[4], ee[5], ee[6]], j0).get(工作相机.get()).cloned().flatten()
                        });
                        手号.set(原手); 臂号.set(原臂);
                        // 框的大小沿用开机部件图里这只手抓握侧那块的大小;只有中心是新认的。
                        let 尺 = 部件图.as_ref().and_then(|图| (臂通道数..图.len()).find(|&kk| 哪手0(kk) == a).and_then(|kk| 图[kk].get(工作相机.get()).and_then(|o| o.as_ref())))
                            .map(|x| ((x.框[2] - x.框[0]) * 0.5, (x.框[3] - x.框[1]) * 0.5)).unwrap_or((1.0 / 40.0, 1.0 / 30.0));
                        // 至少要有开机部件图里那块手指一半的像素,才算认出来(19 px 的散点不算,CT 实测)。
                        let 该有 = 部件图.as_ref().and_then(|图| (臂通道数..图.len()).find(|&kk| 哪手0(kk) == a).and_then(|kk| 图[kk].get(工作相机.get()).and_then(|o| o.as_ref())))
                            .map(|x| (x.占 * cw4 as f64 * ch4 as f64 * 0.5) as u32).unwrap_or(1);
                        match 得 {
                            Some((c, 瓣)) if c.pixels >= 该有 => { println!("[身]    第 {} 只手上次跟丢了 ⇒ 抖一下它的抓握重认手指:中心 ({:.3},{:.3}),{} px,刚性 {:.2}", a + 1, c.u, c.v, c.pixels, c.rigidity);
                                身位.borrow_mut().insert(a, [c.u - 尺.0, c.v - 尺.1, c.u + 尺.0, c.v + 尺.1]);
                                // 两瓣各一个框(按 u 排):中心是各瓣自己的形心,宽取合起来那块的一半(比例,无量纲)—— 模板要切在瓣上,别切到中间那道缝上。
                                // 沿开合方向那一片有多宽(画幅)× 它的深度 = 钳口能张多开的上界(含瓣自己的宽;不是拟的,所以不拿它排序)。
                                let 深c = 近侧深2(plug, 工作相机.get(), c.u, c.v, 尺.0.max(尺.1)).unwrap_or(f64::NAN);
                                match 瓣 {
                                    Some((p, q)) => {
                                        let (du, dv) = (q.u - p.u, q.v - p.v); let l = du.hypot(dv).max(1e-9); let (du, dv) = (du / l, dv / l);
                                        let 幅 = ((c.ext[2] - c.ext[0]) * du).abs() + ((c.ext[3] - c.ext[1]) * dv).abs();
                                        瓣位.borrow_mut().insert(a, vec![[p.u - 尺.0 * 0.5, p.v - 尺.1, p.u + 尺.0 * 0.5, p.v + 尺.1], [q.u - 尺.0 * 0.5, q.v - 尺.1, q.u + 尺.0 * 0.5, q.v + 尺.1]]);
                                        张幅.borrow_mut().insert(a, (幅, 深c));
                                        println!("[身]    两瓣:({:.3},{:.3}) 与 ({:.3},{:.3}) · 分开的方向 ({:+.2},{:+.2}) · 那一片沿它宽 {:.3} 画幅 · 深 {:.3}", p.u, p.v, q.u, q.v, du, dv, 幅, 深c);
                                    }
                                    None => { 瓣位.borrow_mut().remove(&a); 张幅.borrow_mut().insert(a, ((c.ext[2] - c.ext[0]).max(c.ext[3] - c.ext[1]), 深c)); println!("[身]    没配成两瓣(只有合起来那一块 {} px)", c.pixels); }
                                } }
                            Some((c, _)) => { println!("[身]    第 {} 只手抖了抓握,只有 {} px 跟着动(要 {该有})⇒ 它的手指现在不在画面里", a + 1, c.pixels); 身位.borrow_mut().remove(&a); 瓣位.borrow_mut().remove(&a); }
                            None => { println!("[身]    第 {} 只手抖了抓握,这台相机里认不出一块跟着动 ⇒ 它的手指现在不在画面里", a + 1); 身位.borrow_mut().remove(&a); 瓣位.borrow_mut().remove(&a); }
                        }
                    }
                    需抖.borrow_mut().clear();
                }
                let mut t = String::new();
                t.push_str("PIECES OF YOURSELF (measured just now: you moved one channel at a time and watched which part of the picture followed). Each is boxed and NUMBERED on the picture in orange:\n");
                if let Some(图) = 部件图.as_ref() {
                    for (kk, 件们) in 图.iter().enumerate() {
                        if let Some(Some(x)) = 件们.get(工作相机.get()) {
                            // 抓握侧的块用此刻跟到的位置(身位),没有就用开机量的部件图。
                            let 框 = if kk >= 臂通道数 {
                                // 这只手的第 j 个抓握槽 ⇒ 第 j 瓣(按 u 排);没认出两瓣就用合起来那一块;那也没有就用开机部件图。
                                let a = 哪手0(kk);
                                let j = (臂通道数..kk).filter(|&q| 哪手0(q) == a).count();
                                瓣位.borrow().get(&a).and_then(|v| v.get(j).copied()).or_else(|| 身位.borrow().get(&a).copied()).unwrap_or(x.框)
                            } else { x.框 };
                            let (cu, cv) = ((框[0] + 框[2]) * 0.5, (框[1] + 框[3]) * 0.5);
                            条目.push((true, kk, (cu, cv)));
                            let 框px = [(框[0] * cw4 as f64) as usize, (框[1] * ch4 as f64) as usize,
                                        (框[2] * cw4 as f64) as usize, (框[3] * ch4 as f64) as usize];
                            画编号框(&mut 网图, cw4, ch4, 框px, 条目.len(), [255, 160, 32], 2);
                            // 通道号 ≥ 手臂通道数 ⇒ 抓握那一侧的通道(量出来的分界,不是身体词)。
                            let 抓侧 = kk >= 臂通道数;
                            t.push_str(&format!("  item {}: a piece of you{}, now in cell {}{} (covers {:.1}% of the frame)\n",
                                条目.len(), if 抓侧 { format!(" that moves when a grasp channel of arm {} moves (call it a finger of arm {})", 哪手0(kk) + 1, 哪手0(kk) + 1) } else { String::new() }, 格于(cu, cv), 离目标(cu, cv), x.占 * 100.0));
                        }
                    }
                }
                // 🔴 每一只手各两号:**手腕**(身体自己报的末端投到画面上;要有解出来的相机)和**抓握**(那只手的抓握通道)。
                //    编码:手腕 = usize::MAX − 2a,抓握 = usize::MAX − 2a − 1(a = 第几只手)。哪只手由模型点名的块决定,驱动不挑。
                let 臂数 = 帧.ee.len().max(1);
                let 指数 = 帧.jaw.len().max(1);
                let 哪手 = |kk: usize| -> usize { if kk >= 臂通道数 { ((kk - 臂通道数) * 臂数 / 指数).min(臂数 - 1) } else { 手号.get() } };
                // 每只手固定两个槽(手腕、抓握),不管这一轮看不看得见 —— 槽位固定,后面世界块的号才不会跟着漂(CS 实测:相机一解出来,球的号从 22 变 24)。
                for a in 0..臂数 {
                    let mut 腕 = None;
                    if let (Some(e), Some(ee)) = (眼稳.as_ref(), 帧.ee.get(a)) {
                        let q = point_gen::P3 { x: ee[0], y: ee[1], z: ee[2] };
                        if let Some(px) = e.project(q) {
                            let (u, v) = (px[0] / cw4 as f64, px[1] / ch4 as f64);
                            if u > 0.0 && u < 1.0 && v > 0.0 && v < 1.0 { 腕 = Some((u, v)); }
                        }
                    }
                    match 腕 {
                        Some((u, v)) => {
                            条目.push((true, usize::MAX - 2 * a, (u, v)));
                            let s = (cw4 / 40).max(2);   // 只是画框的大小(四十分之一画幅),不参与任何判据
                            let (cx, cy) = ((u * cw4 as f64) as usize, (v * ch4 as f64) as usize);
                            画编号框(&mut 网图, cw4, ch4, [cx.saturating_sub(s), cy.saturating_sub(s), (cx + s).min(cw4 - 1), (cy + s).min(ch4 - 1)], 条目.len(), [255, 160, 32], 2);
                            t.push_str(&format!("  item {}: a piece of you - the end of arm {} that its fingers hang from (call it palm {}; your body reports where it is), now in cell {}{}\n",
                                条目.len(), a + 1, a + 1, 格于(u, v), 离目标(u, v)));
                        }
                        None => {
                            条目.push((true, usize::MAX - 2 * a, (-1.0, -1.0)));
                            t.push_str(&format!("  item {}: palm {} (the end of arm {}) - NOT locatable in this picture right now, do not name it\n", 条目.len(), a + 1, a + 1));
                        }
                    }
                    let 指 = 条目.iter().find(|&&(我, kk, _)| 我 && kk < usize::MAX - 200 && kk >= 臂通道数 && 哪手(kk) == a).copied();
                    match 指.and_then(|(_, kk, 心)| 部件图.as_ref().and_then(|图| 图.get(kk)).and_then(|件们| 件们.get(工作相机.get())).and_then(|x| x.as_ref()).map(|x| (心, x.框))) {
                        Some((心, 框0)) => {
                            let 框 = 身位.borrow().get(&a).copied().unwrap_or(框0);
                            条目.push((true, usize::MAX - 2 * a - 1, 心));
                            let 框px = [(框[0] * cw4 as f64) as usize, (框[1] * ch4 as f64) as usize,
                                        (框[2] * cw4 as f64) as usize, (框[3] * ch4 as f64) as usize];
                            画编号框(&mut 网图, cw4, ch4, 框px, 条目.len(), [255, 64, 200], 2);
                            t.push_str(&format!("  item {}: grip {} - the channel that closes the fingers of arm {} (moving it AT a thing = closing on that thing until resist; moving it BACK from a thing = opening), now in cell {}{}\n",
                                条目.len(), a + 1, a + 1, 格于(心.0, 心.1), 离目标(心.0, 心.1)));
                        }
                        None => {
                            条目.push((true, usize::MAX - 2 * a - 1, (-1.0, -1.0)));
                            t.push_str(&format!("  item {}: grip {} (closes the fingers of arm {}) - its fingers are not locatable in this picture right now\n", 条目.len(), a + 1, a + 1));
                        }
                    }
                }
                t.push_str("THINGS OUT IN THE WORLD (cut out of the depth picture; you do not know what they are called). Each is boxed and NUMBERED on the picture in green:\n");
                // 块跨轮对号:老槽里的块在新块里找"离得最近且在它自己半个框以内"的那一个 ⇒ 同号;没对上的老槽留空;剩下的新块占新槽。
                {
                    let mut 槽 = 世界槽.borrow_mut();
                    let mut 用过 = vec![false; 区们.len()];
                    for s in 槽.iter_mut() {
                        let Some(o) = *s else { continue };
                        let (ou, ov) = (o.心[0], o.心[1]);
                        let 容 = (((o.框[2] - o.框[0]) as f64 / cw4 as f64).max((o.框[3] - o.框[1]) as f64 / ch4 as f64) * 0.5).max(1.0 / cw4 as f64);
                        let mut 最 = None;
                        for (ri, r) in 区们.iter().enumerate() {
                            if 用过[ri] { continue }
                            let d = (r.心[0] - ou).hypot(r.心[1] - ov);
                            if d <= 容 && 最.map(|(_, b)| d < b).unwrap_or(true) { 最 = Some((ri, d)); }
                        }
                        match 最 { Some((ri, _)) => { 用过[ri] = true; *s = Some(区们[ri]); } None => { *s = None; } }
                    }
                    for (ri, r) in 区们.iter().enumerate() { if !用过[ri] { 槽.push(Some(*r)); } }
                }
                for (si, s) in 世界槽.borrow().iter().enumerate() {
                    let Some(r) = s else { 条目.push((false, si, (-1.0, -1.0))); continue };
                    let (cu, cv) = (r.心[0], r.心[1]);
                    条目.push((false, si, (cu, cv)));
                    // 🔴 **编号必须画在它看的那张图上。**(CA 实测 2026-09-02:球和剪刀都在第 11 格,
                    // 文字里两个都是 "a thing in cell 11",而图上只画了网格号没画物体号 ⇒ 它把剪刀当成了球。)
                    画编号框(&mut 网图, cw4, ch4, r.框, 条目.len(), [32, 255, 32], 2);
                    t.push_str(&format!("  item {}: a thing, now in cell {} ({} px, standing {:.3} out of the surface)\n",
                        条目.len(), 格于(cu, cv), r.像素数, r.高));
                }
                // 编号表进日志 —— 不然事后查不出"第 10 号到底是什么"
                for line in t.lines() { println!("[身]   {line}"); }
                // 相机表:1 = 这张图;其余按序号列,长在手上的说明一下(量出来的:那只手一动整幅画都变)。
                {
                    let 台 = plug.lay.cams.len();
                    let mut 行 = format!("CAMERAS (say camera = k to answer about that camera's picture next turn): 1 = this picture (camera index {})", 工作相机.get());
                    let mut k = 2;
                    for ci in 0..台 { if ci == 工作相机.get() { continue }
                        行.push_str(&format!("; {} = camera index {}{}", k, ci, if 手上相机.get() == Some(ci) { " (rides on a hand)" } else { "" })); k += 1; }
                    t.push_str(&行); t.push('\n');
                }
                t.push_str(&format!("- the one you settled on last time is in cell {}\n", 格于(look.u, look.v)));
                t.push_str(&format!("- there is {} between your fingers right now\n", if 手里有.get() { "ALREADY something" } else { "NOTHING" }));
                let 身体 = t;
                if let Ok(dir) = std::env::var("BL_DUMP") {
                    // 每一轮各存一张(带序号):事后要能查"它说的第 9 号到底是哪一块"。
                    问段次.set(问段次.get() + 1);
                    let _ = std::fs::write(format!("{dir}/grid_{:03}.bmp", 问段次.get()), task::bmp24(&网图, cw4, ch4));
                }
                let 刚才 = format!("{}\n{}", 忆文(&记忆), 上一段汇报);
                match body_layer::eye::问段(眼主机, 眼端口, &指令, &身体, &刚才, 网列, 网行, 条目.len(), plug.lay.cams.len().max(1), &网图, cw4, ch4) {
                    Ok(d) => {
                        println!("[身] 🧠 **这一段由模型定**:动**第 {} 号**({})→ 落到**第 {} 格** · 做到**{}**为止{} —— {}",
                            d.动第几号,
                            match 条目.get(d.动第几号.wrapping_sub(1)) { Some((true, k, _)) => format!("我身上第 {k} 号通道管的那一块"), Some((false, _, _)) => "世界里的一块".into(), None => "(号不对)".into() },
                            d.到哪一格, d.到什么为止, if d.完了 { " · 它说已经做完了" } else { "" }, d.为什么);
                        if d.条.len() > 1 || d.条.iter().any(|c| c.保持 || !c.方位.is_empty()) {
                            println!("[身]    全部条目:{}", d.条.iter().map(|c| format!("{}→{}", c.号,
                                if c.保持 { "别动".to_string() } else if !c.方位.is_empty() { format!("{} {}", c.方位, c.相对) } else { format!("格 {}", c.格) })).collect::<Vec<_>>().join(" · "));
                        }
                        快.set(d.快);
                        // 它点名了别的相机 ⇒ 下一轮在那台相机里列块、问、执行(这一段仍按这张图执行)。
                        if d.相机 >= 2 {
                            let mut k = 2; let mut 换 = None;
                            for ci in 0..plug.lay.cams.len() { if ci == 工作相机.get() { continue } if k == d.相机 { 换 = Some(ci); break } k += 1; }
                            if let Some(ci) = 换 { println!("[身]    它要换到第 {} 台相机(序号 {ci})⇒ 下一轮在那台里列块、问、执行", d.相机); 工作相机.set(ci); }
                        }
                        *到什么为止值.borrow_mut() = d.到什么为止.clone();
                        {
                            let mut b = 别碰框.borrow_mut(); b.clear();
                            for &k in &d.别碰 { if let Some(&(false, ri, _)) = 条目.get(k.wrapping_sub(1)) { if let Some(r) = 世界槽.borrow().get(ri).copied().flatten() { b.push(r); } } }
                            if !b.is_empty() { println!("[身]    它说别碰 {} 块 ⇒ 记成硬障碍", b.len()); }
                        }
                        if d.快 { println!("[身]    它说**快** ⇒ 步间不等停{}", if 手里有.get() && d.到什么为止 == "slip" { ";握着东西 + 直到脱手 ⇒ **到位那一步边动边松**" } else { "" }); }
                        这一段 = Some(d);
                    }
                    Err(e) => println!("[身] 🧠 问不通({e})⇒ 照上一段接着走,结果照实报"),
                }
            }
        }

        // ═══════════════════════════════════════════════════════════════════════════════
        // 🔴🔴🔴 **通用执行器:VLM 点名的几块 → 各自去哪,一起解。**(owner 2026-09-03:"完美 driver 能完成所有目标的版本")
        //
        // 话的格式(所有任务同一种):第几号 → 格子 / 相对某一号的方位(贴着·上·下·左·右·前·后)/ 别动;
        // 几条一起;直到什么事件为止;快不快;别碰哪几号。**身体是什么只出现在通道数和部件图里。**
        // 朝向就从"几条一起解"里来:同一段说"手指贴着球、手掌在球上面",姿势只能朝下 —— 没有"转手腕"这个词。
        // 上下按量出来的支撑面法向,左右前后按这台相机的画面方向,"贴着"是位置和深度都到。
        // 表(每个通道动一点,每一块在画面跑多少):手臂通道用存下来的通道表当初值(身上的块共用第 0 块那三行,世界里的块 0),
        // 抓握那一列现场量(只动手指,安全);之后每一步拿实际发生的事修表。没有一个身体词、没有一个手填的长度。
        // ═══════════════════════════════════════════════════════════════════════════════
        let 执行目标组 = |plug: &mut Plug<S>, 条: &[body_layer::eye::目标], 到什么为止: &str, 快: bool, j0: f64, 是关节: bool| -> String {
            let Some(f) = plug.sense() else { return "I could not get a frame.".into() };
            let Some((fw, fh, g)) = 灰(&f, 工作相机.get()) else { return "this camera gave no picture.".into() };
            // 通道 = 每一只手的全部自由度接起来(末端模式:臂数 × 末端维;关节模式:全部关节)。模型点名哪几只手的块,解算就动哪几只手,一次可以几只一起。
            let 臂数0 = f.ee.len().max(1);
            // 速度通道(底盘 / 腿 / 桨):布局报几条就接几条,排在手臂通道后面,和手臂一起解。走路本身归厂商控制器(宇树那条线:我们只出几个数)。
            let 基维 = plug.lay.base.len();
            let 臂通道总 = if 是关节 { 真臂(&f).iter().map(|&i| f.joints[i].len()).sum::<usize>() } else { 臂数0 * 末端维 };
            let 通道数 = 臂通道总 + 基维;
            if 通道数 == 0 { return "this body reports no channels I can push.".into() }
            let 每臂 = if 是关节 { 臂通道总 } else { 末端维 };
            // 发一步:末端模式按臂拆开、每只手各发各的(迈通道稳 只认 手号 那只手);关节模式整条一起发。返回接起来的实到 + 是否有一只被挡。
            let 发 = |plug: &mut Plug<S>, 动: &[f64], 比: f64, 稳拍: u32| -> Option<(Vec<f64>, bool)> {
                let mut 全 = vec![0.0; 通道数]; let mut 挡任 = false;
                // 速度通道:发出去就当作走了命令的量(没有本体回读可比),照实标在报里。
                if 基维 > 0 {
                    let v: Vec<f64> = (0..基维).map(|i| 动.get(臂通道总 + i).copied().unwrap_or(0.0) * 比).collect();
                    if v.iter().any(|x| x.abs() > 1e-12) {
                        if plug.act(&Cmd::Base { v: v.clone() }) { for (i, x) in v.iter().enumerate() { 全[臂通道总 + i] = *x; } } else { 挡任 = true; }
                    }
                }
                if 是关节 { let j = match 爪令.get() { Some((_, j)) => j, None => 1.0 }; let (_, r, 挡) = 迈通道稳(plug, &动[..臂通道总.min(动.len())], 比, j, true, 稳拍)?; for (i, x) in r.iter().enumerate() { if i < 臂通道总 { 全[i] = *x; } } return Some((全, 挡 || 挡任)) }
                let 原手 = 手号.get();
                for a in 0..臂数0 {
                    let 段 = &动[a * 每臂..((a + 1) * 每臂).min(动.len())];
                    let 这手有爪令 = matches!(爪令.get(), Some((aa, _)) if aa == a);
                    if 段.iter().all(|x| x.abs() < 1e-12) && !这手有爪令 { continue }
                    let Some(e) = 臂末.get(a).copied().flatten() else { continue };
                    手号.set(e);
                    let jaw_a = match 爪令.get() { Some((aa, j)) if aa == a => j, _ => plug.sense().and_then(|f| f.jaw.get(e).copied()).unwrap_or(1.0) };
                    if let Some((_, r, 挡)) = 迈通道稳(plug, 段, 比, jaw_a, false, 稳拍) {
                        for (i, x) in r.iter().enumerate() { if a * 每臂 + i < 通道数 { 全[a * 每臂 + i] = *x; } }
                        挡任 |= 挡;
                    }
                }
                手号.set(原手);
                Some((全, 挡任))
            };
            let 列数 = 通道数;   // 抓握通道不进解算:它对"块在画面哪儿"几乎没有影响,由"抓握"这一号直接驱动
            // 这台相机没有深度 ⇒ 深度那一行不解(权 0),深度读数一律当 1(占位),深度门不设。
            let 无深 = plug.lay.depth.get(工作相机.get()).is_none();
            let 读深 = |plug: &mut Plug<S>, u: f64, v: f64, 窗: f64| -> Option<f64> { if 无深 { Some(1.0) } else { 近侧深(plug, u, v, 窗) } };
            // 编码:手腕 = usize::MAX − 2a,抓握 = usize::MAX − 2a − 1。
            let 臂通道数 = if 是关节 { 真臂(&f).iter().map(|&i| f.joints[i].len()).sum::<usize>() } else { 臂数0 * 末端维 };
            let (臂数, 指数) = (臂数0, f.jaw.len().max(1));
            let 手于 = |kk: usize| -> Option<usize> {
                if kk >= usize::MAX - 200 { Some((usize::MAX - kk) / 2) }
                else if kk >= 臂通道数 { Some(((kk - 臂通道数) * 臂数 / 指数).min(臂数 - 1)) }
                else { None }
            };
            let 是抓握 = |kk: usize| kk >= usize::MAX - 200 && (usize::MAX - kk) % 2 == 1;
            let 是手腕 = |kk: usize| kk >= usize::MAX - 200 && (usize::MAX - kk) % 2 == 0;
            // 抓握这一号:合(去某物)/ 张(离开某物),记住是哪只手的。它不进项们(不跟踪),直接推开度到读数不再变。
            let mut 合: Option<(usize, f64)> = None;
            // 抓握 at 世界里的一块 ⇒ 先把那只手的瓣送到那块东西的接触点上,到了(或再也近不了)才合。合与走同时是抛/砸的形状,不是抓的。
            let mut 合后: Option<(usize, usize)> = None;   // (手, 那块东西的号)
            let mut 条留: Vec<body_layer::eye::目标> = Vec::new();
            for c in 条.iter() {
                if let Some(&(true, k, _)) = 条目.get(c.号.wrapping_sub(1)) {
                    if 是抓握(k) {
                        let a = 手于(k).unwrap_or(0);
                        if 臂末.get(a).copied().flatten().and_then(|e| f.jaw.get(e)).is_none() { return "you named a grip, but that hand reports no grasp channel.".into() }
                        合 = Some((a, if c.方位 == "back" { 1.0 } else { -1.0 }));
                        if c.方位 == "at" && c.相对 > 0 && matches!(条目.get(c.相对.wrapping_sub(1)), Some((false, _, _))) { 合后 = Some((a, c.相对)); }
                        continue;
                    }
                }
                条留.push(c.clone());
            }
            // 抓握 at X 而模型没点名那只手的瓣 ⇒ 瓣到接触点是"合上"这件事的前半段,补上:这只手此刻认出来的每一瓣都去 X。
            if let Some((a, w)) = 合后 {
                let 已: Vec<usize> = 条留.iter().map(|c| c.号).collect();
                for (i, &(我, kk, 心)) in 条目.iter().enumerate() {
                    if !我 || !(kk >= 臂通道数 && kk < usize::MAX - 200) || 手于(kk) != Some(a) || 心.0 < 0.0 || 已.contains(&(i + 1)) { continue }
                    条留.push(body_layer::eye::目标 { 号: i + 1, 格: 0, 方位: "at".into(), 相对: w, 保持: false });
                }
                if 条留.is_empty() { return format!("you asked arm {} to grip item {w}, but none of that arm's fingers is locatable in this picture right now (I need to see them to bring them to it).", a + 1) }
            }
            // 🔴 不规定进场姿态(owner 2026-09-03:"人可以从无数个角度拿起这个球")。驱动只管三件量出来的事:
            //    手指落到那块东西的中段(不是它的顶)· 路上不压进别的块的框 · 在这两条下取离现在最近的解。
            //    模型想指定时仍可说"手掌在它上面"(拿杯子把手那种),驱动自己不补。
            let _ = 臂通道数;
            let 条: &[body_layer::eye::目标] = &条留;
            if 条.is_empty() && 合.is_none() { return "you gave no goal I can act on.".into() }
            // 手腕(末端投影)这一号:每拍按身体报的末端位置重投影,不用模板。
            let 投影自 = |f: &Frame, 眼: Option<&point_gen::Eye>, 末: usize| -> Option<(f64, f64, f64)> {
                let e = 眼?; let ee = f.ee.get(末)?;
                let q = point_gen::P3 { x: ee[0], y: ee[1], z: ee[2] };
                let px = e.project(q)?;
                Some((px[0] / fw as f64, px[1] / fh as f64, e.into_cam(q)[2]))
            };
            let 框于 = |号: usize| -> Option<([f64; 4], (f64, f64))> {
                let &(是我, k, 心) = 条目.get(号.wrapping_sub(1))?;
                if 是我 && 是手腕(k) {
                    // 框只用来定深度窗口:取抓握侧那些块的平均短边;一块都没有就取四十分之一画幅。
                    let 尺: Vec<f64> = 条目.iter().filter_map(|&(我, kk, _)| if 我 && kk < usize::MAX - 200 {
                        部件图.as_ref()?.get(kk)?.get(工作相机.get())?.as_ref().map(|x| (x.框[2] - x.框[0]).min((x.框[3] - x.框[1]) * fh as f64 / fw as f64)) } else { None }).collect();
                    let s = if 尺.is_empty() { 1.0 / 40.0 } else { 尺.iter().sum::<f64>() / 尺.len() as f64 } * 0.5;
                    Some(([心.0 - s, 心.1 - s * fw as f64 / fh as f64, 心.0 + s, 心.1 + s * fw as f64 / fh as f64], 心))
                } else if 是我 {
                    let x = 部件图.as_ref()?.get(k)?.get(工作相机.get())?.as_ref()?;
                    let 框 = if k >= 臂通道数 { 手于(k).and_then(|a| 身位.borrow().get(&a).copied()).unwrap_or(x.框) } else { x.框 };
                    Some((框, 心))
                } else {
                    let r = 世界槽.borrow().get(k).copied().flatten()?;
                    Some(([r.框[0] as f64 / fw as f64, r.框[1] as f64 / fh as f64, r.框[2] as f64 / fw as f64, r.框[3] as f64 / fh as f64], 心))
                }
            };
            struct 项 { 号: usize, 是我: bool, 投影: bool, 末: usize, 通道: usize, 框: [f64; 4], 尺m: f64, 模: Vec<u8>, 半: usize, 窗: f64, 现: (f64, f64, f64), 该: (f64, f64, f64), 深权: f64, 说: String, 该2: Option<(f64, f64, f64)> }
            let mut 项们: Vec<项> = Vec::new();
            let 眼 = 眼稳.as_ref();
            let mut 备注: Vec<String> = Vec::new();
            // ── 瓣(某只手的手指块)说到世界里某块东西"上"(at)⇒ 从那块东西的深度形状算接触点 + 从哪边进。──
            //    从哪边进 = 支撑面法向朝相机那一侧(空隙那一侧:桌上就是从上面,贴墙就是从墙外)。
            //    每一瓣各去一个接触点:先到接触点外面(沿法向退一个空隙),再沿法向进去。朝向是这么长出来的,不是指定的;五指手 = 五个点,同一段代码。
            //    几何全在这台相机自己的深度图里算:单位相机(焦距取画幅宽,尺度无关 —— 点从这张图反投出去、再投回同一张图,尺度在投影里抵消)。
            struct 抓法 { 点: Vec<[f64; 3]>, 法: [f64; 3], 隙: f64, 眼: point_gen::Eye, 宽: f64 }
            let 算抓法 = |plug: &mut Plug<S>, a: usize, ri: usize, 块们: &[(usize, f64, f64)]| -> Result<抓法, String> {
                let r区 = 世界槽.borrow().get(ri).copied().flatten().ok_or_else(|| "that thing is not on my list right now".to_string())?;
                let 深 = 深图(plug, 工作相机.get()).ok_or_else(|| "this camera gives no depth picture".to_string())?;
                if 深.len() != fw * fh { return Err("the depth picture is not the size of the picture".into()) }
                // 掩膜:框里比"它的中位深 + 它鼓起高度的四分之三"近的 = 它自己;框外一圈(框再放大一倍)里比那个界远的 = 它旁边的支撑面
                //(两个都是比例,无量纲)—— 减支撑面要有面可减,"从哪边进"的法向就是从这一圈量出来的。
                let 界 = r区.深 + if r区.高.is_finite() && r区.高 > 0.0 { r区.高 * 0.75 } else { 0.0 };
                let (bx0, by0, bx1, by1) = (r区.框[0], r区.框[1], r区.框[2], r区.框[3]);
                let (bw, bh) = ((bx1 - bx0).max(1) as i64, (by1 - by0).max(1) as i64);
                let (gx0, gy0, gx1, gy1) = ((bx0 as i64 - bw).max(0) as usize, (by0 as i64 - bh).max(0) as usize, (bx1 as i64 + bw).min(fw as i64 - 1) as usize, (by1 as i64 + bh).min(fh as i64 - 1) as usize);
                let mut 掩 = vec![false; fw * fh];
                let (mut 我数, mut 面数) = (0usize, 0usize);
                for y in gy0..=gy1 { for x in gx0..=gx1 {
                    let d = 深[y * fw + x]; if !(d.is_finite() && d > 0.0) { continue }
                    let 里 = x >= bx0 && x <= bx1 && y >= by0 && y <= by1;
                    if 里 && d < 界 { 掩[y * fw + x] = true; 我数 += 1; }
                    else if !里 && d >= 界 { 掩[y * fw + x] = true; 面数 += 1; }
                }}
                if 我数 < 8 { return Err(format!("only {我数} depth pixels stand out of the surface inside that thing's box")) }
                let 眼u = point_gen::Eye { fx: fw as f64, fy: fw as f64, cx: fw as f64 * 0.5, cy: fh as f64 * 0.5, at: [0.0; 3], q: [1.0, 0.0, 0.0, 0.0] };
                // 我的瓣此刻在三维哪儿(同一台单位相机里)—— 只用来定"朝我"那一边;钳口能张多开的上界 = 抖时那一片的宽 × 深(单位相机里 Δx = Δu·z)。
                let (mu, mv) = (块们.iter().map(|b| b.1).sum::<f64>() / 块们.len().max(1) as f64, 块们.iter().map(|b| b.2).sum::<f64>() / 块们.len().max(1) as f64);
                // 读深的窗口 = 四十分之一画幅(比例,无量纲)
                let mz = 读深(plug, mu, mv, 1.0 / 40.0).unwrap_or(r区.深);
                let 朝我 = 眼u.back_project([mu * fw as f64, mv * fh as f64], mz).map(|p| [p.x, p.y, p.z]).unwrap_or([0.0, 0.0, 0.0]);
                let (张幅u, z手) = 张幅.borrow().get(&a).copied().unwrap_or((f64::NAN, f64::NAN));
                let 张开 = if 张幅u.is_finite() && z手.is_finite() && 张幅u > 0.0 { 张幅u * z手 } else { ((r区.框[2] - r区.框[0]) as f64 / fw as f64) * r区.深 };
                let r = task::尺 { 张开, 张开可信: false, 可达内: 可达带[0], 可达: 可达带[1] };
                let 没试: [[f64; 3]; 0] = [];
                // 0.5 = 摩擦系数 μ、1.5 = 圈物体那道闸的宽容倍数(两个都是无量纲比值,和以前那条链一样)
                let (候选, 航点, 宽, _尖, _q, _桌, 接触点, 支撑法向) = task::算一把(&眼u, &深, fw, fh, [r区.心[0] * fw as f64, r区.心[1] * fh as f64],
                    (r区.框[2] - r区.框[0]) as f64 / fw as f64, None, 0.5, 1.5, 朝我, &没试, &r, Some(&掩)).map_err(|e| format!("{e:?}"))?;
                let 中 = [(接触点[0][0] + 接触点[1][0]) * 0.5, (接触点[0][1] + 接触点[1][1]) * 0.5, (接触点[0][2] + 接触点[1][2]) * 0.5];
                // 法向:量到的支撑面法向,符号取朝相机那一侧(空隙那一侧);没量到就用"从它指向相机"。
                let mut n = 支撑法向.unwrap_or([眼u.at[0] - 中[0], 眼u.at[1] - 中[1], 眼u.at[2] - 中[2]]);
                let l = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt().max(1e-9); n = [n[0] / l, n[1] / l, n[2] / l];
                if (眼u.at[0] - 中[0]) * n[0] + (眼u.at[1] - 中[1]) * n[1] + (眼u.at[2] - 中[2]) * n[2] < 0.0 { n = [-n[0], -n[1], -n[2]]; }
                // 空隙:接触点外面停多远 —— 它鼓起多高的两倍(比例,无量纲);没量到高度就用两点相距。
                let 隙 = if r区.高.is_finite() && r区.高 > 0.0 { r区.高 * 2.0 } else { 宽 };
                println!("[身]     接触点(第 {} 只手 · 世界块槽 {ri}):候选 {候选} · 航点 {航点} · 两点相距 {:.4}(单位相机)· 法向 ({:+.2},{:+.2},{:+.2})· 外面停 {:.3} · 我的瓣 {} 片 · 掩膜 我 {我数} / 面 {面数} px", a + 1, 宽, n[0], n[1], n[2], 隙, 块们.len());
                Ok(抓法 { 点: vec![接触点[0], 接触点[1]], 法: n, 隙, 眼: 眼u, 宽 })
            };
            // 号 ⇒ (外面那一点, 接触点, 说法):点按投影的 u 排、瓣按 u 排,一一配;只认出一瓣就去两点的中点;瓣比点多的去最近的点。
            let mut 派: std::collections::HashMap<usize, ((f64, f64, f64), (f64, f64, f64), String)> = std::collections::HashMap::new();
            let mut 抓法数 = 0usize;
            {
                let mut 要: std::collections::BTreeMap<(usize, usize), Vec<(usize, f64, f64)>> = std::collections::BTreeMap::new();
                let mut 号w: std::collections::HashMap<(usize, usize), usize> = std::collections::HashMap::new();
                for c in 条.iter() {
                    if c.方位 != "at" || c.相对 == 0 { continue }
                    let (Some(&(true, k, 心)), Some(&(false, ri, _))) = (条目.get(c.号.wrapping_sub(1)), 条目.get(c.相对.wrapping_sub(1))) else { continue };
                    if !(k >= 臂通道数 && k < usize::MAX - 200) || 心.0 < 0.0 { continue }
                    let Some(a) = 手于(k) else { continue };
                    要.entry((a, ri)).or_default().push((c.号, 心.0, 心.1));
                    号w.insert((a, ri), c.相对);
                }
                for ((a, ri), mut 块们) in 要 {
                    块们.sort_by(|x, y| x.1.partial_cmp(&y.1).unwrap_or(std::cmp::Ordering::Equal));
                    let mut 独: Vec<(f64, f64)> = Vec::new();
                    for b in 块们.iter() { if !独.iter().any(|d| (d.0 - b.1).abs() < 1e-9 && (d.1 - b.2).abs() < 1e-9) { 独.push((b.1, b.2)); } }
                    let w号 = 号w.get(&(a, ri)).copied().unwrap_or(ri + 1);
                    match 算抓法(plug, a, ri, &块们) {
                        Ok(z) => {
                            let 投 = |p: [f64; 3]| -> Option<(f64, f64, f64)> { let q = point_gen::P3 { x: p[0], y: p[1], z: p[2] }; let px = z.眼.project(q)?; Some((px[0] / fw as f64, px[1] / fh as f64, z.眼.into_cam(q)[2])) };
                            let mut 点序: Vec<usize> = (0..z.点.len()).collect();
                            点序.sort_by(|&x, &y| 投(z.点[x]).map(|t| t.0).partial_cmp(&投(z.点[y]).map(|t| t.0)).unwrap_or(std::cmp::Ordering::Equal));
                            let 外 = |p: [f64; 3]| [p[0] + z.法[0] * z.隙, p[1] + z.法[1] * z.隙, p[2] + z.法[2] * z.隙];
                            let 中 = { let n = z.点.len().max(1) as f64; let mut m = [0.0; 3]; for p in z.点.iter() { for k in 0..3 { m[k] += p[k] / n; } } m };
                            for &(号, u, v) in 块们.iter() {
                                let m = 独.iter().position(|d| (d.0 - u).abs() < 1e-9 && (d.1 - v).abs() < 1e-9).unwrap_or(0);
                                let (目标点, 哪) = if 独.len() == 1 && z.点.len() > 1 { (中, "the middle of the contact points".to_string()) }
                                    else if m < 点序.len() { (z.点[点序[m]], format!("contact point {}", 点序[m] + 1)) }
                                    else { let mut 最 = (0usize, f64::MAX); for (i, p) in z.点.iter().enumerate() { if let Some(t) = 投(*p) { let d = (t.0 - u).hypot(t.1 - v); if d < 最.1 { 最 = (i, d); } } } (z.点[最.0], format!("contact point {}", 最.0 + 1)) };
                                let (Some(前), Some(后)) = (投(外(目标点)), 投(目标点)) else { 备注.push(format!("item {号}: its contact point falls behind the camera")); continue };
                                println!("[身]     第 {号} 号(第 {} 只手的瓣,此刻 ({u:.3},{v:.3}))⇒ {哪}:外面 ({:.3},{:.3}) 深 {:.3} → 接触 ({:.3},{:.3}) 深 {:.3}", a + 1, 前.0, 前.1, 前.2, 后.0, 后.1, 后.2);
                                派.insert(号, (前, 后, format!("item {号} (a finger of arm {}) to {哪} on item {w号}: first just outside it, then in along the free side", a + 1)));
                            }
                            // 画给人看:绿=那块东西 · 紫=我的瓣 · 白=接触点 · 黄空心=外面那一点 · 黄线=进的方向。数字骗得了人,图骗不了。
                            if let Ok(dir) = std::env::var("BL_DUMP") {
                                if let Some((cw, ch, mut 图)) = 彩(&f, 工作相机.get()) {
                                    let 点 = |图: &mut Vec<u8>, t: (f64, f64, f64), 色: [u8; 3], r: i64, 空心: bool| {
                                        let (cx, cy) = ((t.0 * cw as f64) as i64, (t.1 * ch as f64) as i64);
                                        for dx in -r..=r { for dy in -r..=r {
                                            if 空心 && dx.abs() != r && dy.abs() != r { continue }
                                            let (x, y) = (cx + dx, cy + dy);
                                            if x >= 0 && y >= 0 && (x as usize) < cw && (y as usize) < ch { let i = (y as usize * cw + x as usize) * 3; if i + 2 < 图.len() { 图[i] = 色[0]; 图[i + 1] = 色[1]; 图[i + 2] = 色[2]; } }
                                        }}
                                    };
                                    if let Some(r区) = 世界槽.borrow().get(ri).copied().flatten() { 画编号框(&mut 图, cw, ch, r区.框, w号, [32, 255, 32], 2); }
                                    for &(号, _, _) in 块们.iter() { if let Some((框, _)) = 框于(号) { 画编号框(&mut 图, cw, ch, [(框[0] * cw as f64) as usize, (框[1] * ch as f64) as usize, (框[2] * cw as f64) as usize, (框[3] * ch as f64) as usize], 号, [255, 64, 200], 2); } }
                                    for p in z.点.iter() { if let Some(t) = 投(*p) { 点(&mut 图, t, [255, 255, 255], 4, false); } if let Some(t) = 投(外(*p)) { 点(&mut 图, t, [255, 255, 0], 4, true); } }
                                    for k in 0..=10 { let sc = z.隙 * k as f64 / 10.0; if let Some(t) = 投([中[0] + z.法[0] * sc, 中[1] + z.法[1] * sc, 中[2] + z.法[2] * sc]) { 点(&mut 图, t, [255, 255, 0], 1, false); } }
                                    let 路 = format!("{dir}/抓_{:03}_{}.bmp", 问段次.get(), a + 1);
                                    let _ = std::fs::write(&路, task::bmp24(&图, cw, ch));
                                    println!("[身]     🖼 接触点画好了 ⇒ {路}(绿=那块东西,紫=我的瓣,白=接触点,黄空心=外面那一点,黄线=进的方向)");
                                }
                            }
                            抓法数 += 1;
                        }
                        Err(e) => { 备注.push(format!("could not compute contact points on item {w号} for arm {}: {e}", a + 1)); println!("[身]     算不出第 {} 只手在第 {w号} 号上的接触点:{e}", a + 1); }
                    }
                }
            }
            // 只看不动(BL_LOOK):接触点算完、图画完就回去 —— 给人先看箭头指得对不对,再让它动。
            if 抓法数 > 0 && std::env::var("BL_LOOK").is_ok() { return format!("(look only) I computed contact points and drew them; I did not move.{}", if 备注.is_empty() { String::new() } else { format!(" note: {}", 备注.join("; ")) }) }
            for (i, c) in 条.iter().enumerate() {
                if matches!(条目.get(c.号.wrapping_sub(1)), Some(&(_, _, (u, _))) if u < 0.0) { return format!("goal {}: item {} is not locatable in this picture right now.", i + 1, c.号) }
                if c.相对 > 0 && matches!(条目.get(c.相对.wrapping_sub(1)), Some(&(_, _, (u, _))) if u < 0.0) { return format!("goal {}: item {} (the one it is relative to) is not locatable in this picture right now.", i + 1, c.相对) }
                let Some((框, 心)) = 框于(c.号) else { return format!("goal {}: item {} is not on my list right now.", i + 1, c.号) };
                let 是我 = matches!(条目.get(c.号.wrapping_sub(1)), Some((true, _, _)));
                let 投影 = matches!(条目.get(c.号.wrapping_sub(1)), Some((true, k, _)) if 是手腕(*k));
                let 末 = match 条目.get(c.号.wrapping_sub(1)) { Some(&(true, k, _)) => 手于(k).and_then(|a| 臂末.get(a).copied().flatten()).unwrap_or(手号.get()), _ => 手号.get() };
                // 模板半径 = 这一块短边的四分之一(切在它自己身上)。3 px 是匹配器的下限、八分之一画幅是算力上限 —— 都不是身体量。
                let 短 = ((框[2] - 框[0]) * fw as f64).min((框[3] - 框[1]) * fh as f64).max(0.0);
                let 窗 = (短 / fw as f64 * 0.5).max(1.0 / fw as f64);
                let (模, 半, 现) = if 投影 {
                    let Some(x) = 投影自(&f, 眼, 末) else { return format!("goal {}: item {} (the end of my arm) cannot be placed in this picture right now.", i + 1, c.号) };
                    (Vec::new(), 0usize, x)
                } else {
                    let 半 = ((短 * 0.25) as usize).clamp(3, (fw / 8).max(3));
                    let 半 = 合用半(fw, fh, &[心], 半);
                    let Some(模) = (if 半 > 0 { 截块(fw, fh, &g, 心.0, 心.1, 半) } else { None }) else {
                        return format!("goal {}: item {} sits at the edge of the picture, I cannot cut a template to track it.", i + 1, c.号) };
                    let Some(z) = 读深(plug, 心.0, 心.1, 窗) else { return format!("goal {}: item {} has no depth reading.", i + 1, c.号) };
                    (模, 半, (心.0, 心.1, z))
                };
                let z = 现.2;
                let 米 = |px: f64, z: f64| -> f64 { match 眼 { Some(e) => px * z / e.fx.max(1e-9), None => px * z / fw as f64 } };
                let 尺m = 米(短, z).max(1e-3);
                let 通道 = match 条目.get(c.号.wrapping_sub(1)) { Some(&(true, k, _)) => k, _ => usize::MAX };
                // 说了地方就是要动;"别动"只在没说任何地方时才算数。
                let 只保持 = c.保持 && c.格 == 0 && (c.方位.is_empty() || c.相对 == 0);
                let mut 该2: Option<(f64, f64, f64)> = None;
                let (该, 深权, 说) = if 只保持 {
                    (现, 1.0, format!("item {} keeps its place", c.号))
                } else if !c.方位.is_empty() && c.相对 > 0 {
                    let Some((框o, 心o)) = 框于(c.相对) else { return format!("goal {}: item {} (the one it is relative to) is not on my list.", i + 1, c.相对) };
                    let 短o = ((框o[2] - 框o[0]) * fw as f64).min((框o[3] - 框o[1]) * fh as f64).max(0.0);
                    let 窗o = (短o / fw as f64 * 0.5).max(1.0 / fw as f64);
                    let Some(zo) = 读深(plug, 心o.0, 心o.1, 窗o) else { return format!("goal {}: item {} has no depth reading.", i + 1, c.相对) };
                    let 尺k = 米(短, z).max(1e-3);
                    // "贴着"世界里的一块 = 落到它的**中段**(近侧深度 + 它鼓起来的一半,鼓多高是切块时量的),
                    // 这样从哪个方向进,手指都夹在它中间;贴着我自己的一块 = 就是那一点。
                    let 中段 = match 条目.get(c.相对.wrapping_sub(1)) { Some(&(false, ri, _)) => 世界槽.borrow().get(ri).copied().flatten().map(|r| if r.高.is_finite() { r.高 * 0.5 } else { 0.0 }).unwrap_or(0.0), _ => 0.0 };
                    match c.方位.as_str() {
                        "at" => match 派.get(&c.号) { Some((前, 后, 说)) => { 该2 = Some(*后); (*前, 1.0, 说.clone()) } None => ((心o.0, 心o.1, zo + 中段), 1.0, format!("item {} at item {}", c.号, c.相对)) },
                        "left" | "right" => {
                            let s = if c.方位 == "left" { -1.0 } else { 1.0 };
                            ((心o.0 + s * (框[2] - 框[0]).max(1.0 / fw as f64), 心o.1, zo), 1.0, format!("item {} {} of item {}", c.号, c.方位, c.相对))
                        }
                        "front" | "back" => {
                            let s = if c.方位 == "front" { -1.0 } else { 1.0 };
                            ((心o.0, 心o.1, (zo + s * 尺k).max(1e-3)), 1.0, format!("item {} {} of item {}", c.号, c.方位, c.相对))
                        }
                        "away" => {
                            // 离开它:沿"它指向我"的方向再退一个我自己的尺寸(画面里),深度不变。躲拳就是这一句。
                            let (du, dv) = (心.0 - 心o.0, 心.1 - 心o.1); let l = (du * du + dv * dv).sqrt().max(1e-9);
                            let 步 = ((框[2] - 框[0]).max(1.0 / fw as f64)).max(1e-3);
                            ((心.0 + du / l * 步, 心.1 + dv / l * 步, z), 1.0, format!("item {} away from item {}", c.号, c.相对))
                        }
                        _ => {
                            let s = if c.方位 == "above" { 1.0 } else { -1.0 };
                            match 眼 {
                                Some(e) => {
                                    let xo = e.back_project([心o.0 * fw as f64, 心o.1 * fh as f64], zo);
                                    let xk = e.back_project([心.0 * fw as f64, 心.1 * fh as f64], z);
                                    let (Ok(xo), Ok(xk)) = (xo, xk) else { return format!("goal {}: could not place item {} in 3-D (depth unusable).", i + 1, c.相对) };
                                    // 法向:量到的支撑面法向,符号取"朝相机那一侧";没量到就用"从它指向相机"。
                                    let mut n = 桌面法向.get().unwrap_or([e.at[0] - xo.x, e.at[1] - xo.y, e.at[2] - xo.z]);
                                    let l = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt().max(1e-9);
                                    n = [n[0] / l, n[1] / l, n[2] / l];
                                    let 朝 = (e.at[0] - xo.x) * n[0] + (e.at[1] - xo.y) * n[1] + (e.at[2] - xo.z) * n[2];
                                    if 朝 < 0.0 { n = [-n[0], -n[1], -n[2]]; }
                                    // 高度:这一块此刻在"它"那边多高;不在那一侧就用这一块自己的尺寸。
                                    let h = (xk.x - xo.x) * n[0] + (xk.y - xo.y) * n[1] + (xk.z - xo.z) * n[2];
                                    let h = if h * s > 0.0 { h.abs() } else { 尺k };
                                    let t = point_gen::P3 { x: xo.x + s * n[0] * h, y: xo.y + s * n[1] * h, z: xo.z + s * n[2] * h };
                                    let Some(px) = e.project(t) else { return format!("goal {}: the place {} item {} falls behind the camera.", i + 1, c.方位, c.相对) };
                                    ((px[0] / fw as f64, px[1] / fh as f64, e.into_cam(t)[2]), 1.0,
                                     format!("item {} {} item {} (along the measured surface normal, {:.3} m)", c.号, c.方位, c.相对, h))
                                }
                                None => {
                                    备注.push("no solved camera: 'above/below' taken as 'same pixel, keep my depth'".into());
                                    ((心o.0, 心o.1, z), 1.0, format!("item {} {} item {} (approx)", c.号, c.方位, c.相对))
                                }
                            }
                        }
                    }
                } else if c.格 > 0 {
                    let Some(&(gu, gv)) = 格心.get(c.格 - 1) else { return format!("goal {}: cell {} does not exist.", i + 1, c.格) };
                    ((gu, gv, z), 0.0, format!("item {} to cell {}", c.号, c.格))
                } else {
                    return format!("goal {}: names no place.", i + 1);
                };
                let 深权 = if 无深 { 0.0 } else { 深权 };
                项们.push(项 { 号: c.号, 是我, 投影, 末, 通道, 框, 尺m, 模, 半, 窗, 现, 该, 深权, 说, 该2 });
            }
            let 点数 = 项们.len();
            let 维 = 3 * 点数;
            println!("[身] ⚙ 一起解 {点数} 条:{}", 项们.iter().map(|p| p.说.clone()).collect::<Vec<_>>().join(" · "));
            let mut 表: Vec<Vec<f64>> = vec![vec![0.0; 维]; 列数];
            // 身上的块:上一段量到/修过的响应装回来(键 = 部件图通道号);世界里的块 0(不握着的时候我动它不动)。
            let mut 缺: Vec<bool> = vec![false; 点数];   // 这一块的响应还没有 ⇒ 要探
            {
                let 表存 = 身表.borrow();
                for (pi, p) in 项们.iter().enumerate() {
                    if !p.是我 || p.投影 { continue }
                    match 表存.get(&p.通道) {
                        Some(v) if v.len() == 3 * 列数 => { for c in 0..列数 { for r in 0..3 { 表[c][3 * pi + r] = v[3 * c + r]; } } }
                        _ => { 缺[pi] = true; }
                    }
                }
            }
            let 有初值 = !缺.iter().any(|x| *x);
            // 手腕(末端投影)在末端模式下的响应可以直接算:平移通道 = 相机投影对那根世界轴的导数;转动通道绕末端自己转,末端不动 ⇒ 0。
            // 这不是假设:探针量到的"实到"就是末端沿那根轴走了多少(`后[k]−前[k]`)。关节模式没有这条,沿用初值再让修表去纠。
            if !是关节 {
                if let Some(e) = 眼 {
                    for (pi, p) in 项们.iter().enumerate() {
                        if !p.投影 { continue }
                        let Some(ee) = f.ee.get(p.末) else { continue };
                        let 臂 = 臂末.iter().position(|x| *x == Some(p.末)).unwrap_or(0);
                        let q0 = point_gen::P3 { x: ee[0], y: ee[1], z: ee[2] };
                        let (Some(p0), c0) = (e.project(q0), e.into_cam(q0)[2]) else { continue };
                        for c in 0..通道数 {
                            let (a, k) = (c / 每臂, c % 每臂);
                            if a != 臂 || k >= 3 { for r in 0..3 { 表[c][3 * pi + r] = 0.0; } continue }
                            let mut q1 = [ee[0], ee[1], ee[2]]; q1[k] += 可达带[1];
                            let q1 = point_gen::P3 { x: q1[0], y: q1[1], z: q1[2] };
                            if let Some(p1) = e.project(q1) {
                                表[c][3 * pi] = (p1[0] - p0[0]) / fw as f64 / 可达带[1];
                                表[c][3 * pi + 1] = (p1[1] - p0[1]) / fh as f64 / 可达带[1];
                                表[c][3 * pi + 2] = (e.into_cam(q1)[2] - c0) / 可达带[1];
                            }
                        }
                    }
                }
            }
            let 读所有 = |plug: &mut Plug<S>, 项们: &mut Vec<项>| -> Option<()> {
                let f = plug.sense()?;
                let (_, _, g) = 灰(&f, 工作相机.get())?;
                for p in 项们.iter_mut() {
                    if p.投影 { p.现 = 投影自(&f, 眼, p.末)?; continue }
                    let (u, v) = 找块窗(fw, fh, &g, &p.模, p.半, p.现.0, p.现.1, p.半 * 3)?;
                    let z = 读深(plug, u, v, p.窗)?;
                    p.现 = (u, v, z);
                }
                Some(())
            };
            let 深换于 = |z: f64| -> f64 { match 眼 { Some(e) => e.fx / (fw as f64 * z.max(1e-6)), None => 1.0 / z.max(1e-6) } };
            let 误于 = |项们: &Vec<项>| -> Vec<f64> {
                项们.iter().flat_map(|p| { let k = 深换于(p.该.2) * p.深权; [p.该.0 - p.现.0, p.该.1 - p.现.1, (p.该.2 - p.现.2) * k] }).collect()
            };
            let 差于 = |e: &[f64]| e.iter().map(|x| x * x).sum::<f64>().sqrt();
            let 读j = |plug: &mut Plug<S>| plug.sense().and_then(|f| f.jaw.get(手号.get()).or_else(|| f.jaw.first()).copied());
            let mut jaw = j0;
            // ── 噪声地板,全部现场量(什么都不做时):本体报的"实到"抖多少 · 各块在画面里抖多少 · 整幅画抖几级 · 抓握读数抖多少 ──
            let 实到噪 = {
                // 发零命令给每只手,看本体报的实到抖多少
                let 原手 = 手号.get(); let mut 噪 = 0.0f64;
                for a in 0..臂数0 { if let Some(e) = 臂末.get(a).copied().flatten() { 手号.set(e);
                    if let Some((_, r, _)) = 迈通道稳(plug, &vec![0.0; 每臂], 1.0, jaw, 是关节, 1) { 噪 = 噪.max(r.iter().map(|x| x.abs()).fold(0.0, f64::max)); } } }
                手号.set(原手); 噪
            };
            let 差a = 差于(&误于(&项们));
            let 静噪 = {
                let a = plug.sense().and_then(|f| 灰(&f, 工作相机.get())).map(|(_, _, g)| g);
                let _ = 读所有(plug, &mut 项们);
                let b = plug.sense().and_then(|f| 灰(&f, 工作相机.get())).map(|(_, _, g)| g);
                match (a, b) { (Some(a), Some(b)) if a.len() == b.len() => a.iter().zip(b.iter()).map(|(x, y)| x.abs_diff(*y)).max().unwrap_or(0), _ => 0 }
            };
            let 跟踪噪 = (差于(&误于(&项们)) - 差a).abs();
            let 深噪 = { let mut n = 0.0f64; for p in 项们.iter() { if p.投影 { continue } let a = 读深(plug, p.现.0, p.现.1, p.窗); let b = 读深(plug, p.现.0, p.现.1, p.窗); if let (Some(a), Some(b)) = (a, b) { n = n.max((a - b).abs()); } } n };
            let 读数噪 = { let a = 读j(plug); let b = 读j(plug); match (a, b) { (Some(a), Some(b)) => (a - b).abs(), _ => 0.0 } };
            println!("[身]     噪声地板(现场量):实到 {实到噪:.5} · 各块位置 {跟踪噪:.4} 画幅 · 整幅画 {静噪} 级 · 抓握读数 {读数噪:.4}");
            // 一步多大:从量到的可达带整个开始,被拒就减半(最多三轮),接受的比例跨段带走、连成三步放大回去。没有"三分之一"。
            let 幅0 = 可达带[1].max(1e-6);
            let (mut 缩, mut 减半次) = (步缩.get().clamp(1.0 / 64.0, 1.0), 0u32);
            let mut 缩探 = 缩;   // 探针实际接受的比例(走了至少一半的那一档),步子从它起
            // ── 没有存表的通道:现场来回探一遍(读的是所有点)。被拒的幅度减半再探。──
            let 探列: Vec<usize> = if 有初值 || 条.is_empty() { Vec::new() } else { (0..通道数).collect() };
            if !探列.is_empty() { println!("[身]     身上 {} 块还没有响应表 ⇒ 探 {} 个通道(探过就记住,下一段不再探)", 缺.iter().filter(|x| **x).count(), 探列.len()); }
            for &c in &探列 {
                let mut 试 = 0u32;
                let mut 缩p = 缩;   // 探针自己的比例:被拒就减半,不动步子那一个
                loop {
                    let 幅 = 幅0 * 缩p;
                    let mut 动 = vec![0.0; 通道数];
                    动[c] = 幅;
                    let Some((r1, _)) = 发(plug, &动, 1.0, 等拍 * 2) else { break };
                    // 走了不到要求的一半 ⇒ 这个幅度身体吃不下(被拒或只交付一小截),减半再探:探针要落在"命令多少走多少"那一段里,列才是真的。
                    let 到 = r1.get(c).copied().unwrap_or(0.0).abs();
                    if 到 <= 实到噪 || 到 < 幅 * 0.5 {
                        if 试 < 8 && 幅 * 0.5 > 实到噪 { 缩p *= 0.5; 缩探 = 缩探.min(缩p); 试 += 1; 动[c] = -到 * r1.get(c).copied().unwrap_or(0.0).signum(); let _ = 发(plug, &动, 1.0, 等拍 * 2); continue }
                        else { 备注.push(format!("channel {c} never delivered half of what I asked, down to {幅:.4}")); println!("[身]     探通道 {c}:幅 {幅:.4} 只走了 {到:.4},不再缩"); break }
                    }
                    let _ = 读所有(plug, &mut 项们);
                    let 中: Vec<(f64, f64, f64)> = 项们.iter().map(|p| p.现).collect();
                    动[c] = -幅 * 2.0;
                    let Some((r2, _)) = 发(plug, &动, 1.0, 等拍 * 2) else { break };
                    let _ = 读所有(plug, &mut 项们);
                    let 后: Vec<(f64, f64, f64)> = 项们.iter().map(|p| p.现).collect();
                    动[c] = 幅;
                    let _ = 发(plug, &动, 1.0, 等拍 * 2);
                    let _ = 读所有(plug, &mut 项们);
                    let 净 = r2.get(c).copied().unwrap_or(0.0);
                    // 只有这个通道真的走了探针的一大半,它的列才算数:除以一个接近零的"实到"会把列撑到几十画幅/单位,
                    // 预测和解算全被它带飞(CS 实测:预测位移 −5.3 画幅、解出的相机焦距 0)。
                    if 净.abs() > 实到噪 && 净.abs() >= 幅 {
                        for pi in 0..点数 {
                            表[c][3 * pi] = (后[pi].0 - 中[pi].0) / 净; 表[c][3 * pi + 1] = (后[pi].1 - 中[pi].1) / 净; 表[c][3 * pi + 2] = (后[pi].2 - 中[pi].2) / 净;
                        }
                    } else if 净.abs() > 实到噪 {
                        备注.push(format!("channel {c} delivered only {:.3} of the {:.3} I asked on the way back; not trusting that column", 净.abs(), 幅 * 2.0));
                    }
                    println!("[身]     探通道 {c}:幅 {幅:.4} · 实到 {:+.4} / {:+.4} · 各块跑了 {}", r1.get(c).copied().unwrap_or(0.0), 净,
                        (0..点数).map(|pi| format!("({:+.3},{:+.3},{:+.3})", 后[pi].0 - 中[pi].0, 后[pi].1 - 中[pi].1, 后[pi].2 - 中[pi].2)).collect::<Vec<_>>().join(" "));
                    break;
                }
            }
            // 相机:还没有的话,从这张表里解 —— 某只手的手指块对那只手三个平移通道的响应(像素/米、深/米)+ 它此刻的像素、深度、手在世界哪儿。
            // 闭式、不拟合、解不出就 None(point_gen::eye_from_jacobian 头注)。解出来下一段就有"手掌"那一号和真的"上/下"。
            if 眼.is_none() && !是关节 {
                for (pi, p) in 项们.iter().enumerate() {
                    if !p.是我 || p.投影 || p.通道 < 臂通道数 { continue }
                    let Some(a) = 手于(p.通道) else { continue };
                    let Some(e) = 臂末.get(a).copied().flatten() else { continue };
                    let Some(ee) = f.ee.get(e) else { continue };
                    let mut j = [[0.0f64; 3]; 3];
                    let mut 全有 = true;
                    for k in 0..3 { let c = a * 每臂 + k; if c >= 列数 { 全有 = false; break }
                        j[0][k] = 表[c][3 * pi] * fw as f64; j[1][k] = 表[c][3 * pi + 1] * fh as f64; j[2][k] = 表[c][3 * pi + 2];
                        if j[0][k].abs() + j[1][k].abs() + j[2][k].abs() < 1e-9 { 全有 = false; } }
                    if !全有 { continue }
                    let 尺px = ((p.框[2] - p.框[0]) * fw as f64).max((p.框[3] - p.框[1]) * fh as f64).max(1.0);
                    match point_gen::eye_from_jacobian(j, p.现.0 * fw as f64, p.现.1 * fh as f64, p.现.2, [ee[0], ee[1], ee[2]]) {
                        Some(e2) if e2.fx > 1.0 && e2.fy > 1.0 && (e2.fx / e2.fy).max(e2.fy / e2.fx) <= 2.0 && e2.cx >= 0.0 && e2.cy >= 0.0 && e2.cx <= fw as f64 && e2.cy <= fh as f64
                            && e2.project(point_gen::P3 { x: ee[0], y: ee[1], z: ee[2] }).map(|q| (q[0] - p.现.0 * fw as f64).hypot(q[1] - p.现.1 * fh as f64) <= 尺px).unwrap_or(false) => {
                            println!("[身]     从表里解出相机:焦距 {:.1}/{:.1} · 主点 ({:.1},{:.1}) · 相机在 ({:.2},{:.2},{:.2})(用第 {} 只手的手指块)", e2.fx, e2.fy, e2.cx, e2.cy, e2.at[0], e2.at[1], e2.at[2], a + 1);
                            *新眼.borrow_mut() = Some(e2); break
                        }
                        Some(e2) => println!("[身]     从表里解出的相机不合格(焦距 {:.1}/{:.1} · 主点 ({:.0},{:.0})· 手投回去落不到那一块上),不收", e2.fx, e2.fy, e2.cx, e2.cy),
                        None => {}
                    }
                }
            }
            缩 = 缩.min(缩探);   // 探针都得减到这一档身体才走得动,步子没理由从更大的起(CT 实测:0.68 m 的步子连拒 13 次)
            let mut 上差 = 差于(&误于(&项们));
            let (mut 不缩, mut 走了, mut 静, mut 连成) = (0u32, 0u32, 0u32, 0u32);
            let mut 事件 = String::from("hit the safety cap on steps");
            // 第二段:外面那一点到了(或不再靠近)⇒ 把目标换成接触点本身,沿空隙那一侧进去。
            let 换段 = |项们: &mut Vec<项>| -> bool { let mut 换 = false; for p in 项们.iter_mut() { if let Some(t) = p.该2.take() { p.该 = t; 换 = true; } } 换 };
            let mut 段 = 1u32;
            let 要碰 = 到什么为止 == "contact" || 到什么为止 == "resist";
            let 稳拍 = if 快 { 1 } else { 等拍 * 2 };
            let mut 上帧: Option<Vec<u8>> = None;
            // 抓握和移动在同一个循环里一起做(抛/砸要边动边松):合 = 每步命令合到底,张 = 每步命令张到底;读数不再变就是它停了。没有"合多少"这个量。
            let mut 握 = String::new();
            let 爪臂 = 合.map(|(a, _)| a);
            let 读爪 = |plug: &mut Plug<S>| -> Option<f64> { let e = 臂末.get(爪臂?).copied().flatten()?; plug.sense().and_then(|f| f.jaw.get(e).copied()) };
            let mut 爪停 = 0u32;
            let mut 爪读 = 读爪(plug).unwrap_or(jaw);
            if 合后.is_none() { if let Some((a, s)) = 合 { 爪令.set(Some((a, if s < 0.0 { 0.0 } else { 1.0 }))); } }
            let 空 = 空合载.unwrap_or(0.0);
            // 400 步是安全上限(防跑飞),不是策略;正常出口全是量出来的事件。
            let 安全上限 = 400u32;
            loop {
                if 条.is_empty() {
                    // 只有抓握这一号:发零位移 + 抓握命令,读数连着两步不变就停。
                    if 合.is_none() { break }
                    let _ = 发(plug, &vec![0.0; 通道数], 1.0, 稳拍);
                    let 新读 = 读爪(plug).unwrap_or(爪读);
                    走了 += 1;
                    if (新读 - 爪读).abs() <= 读数噪 { 爪停 += 1 } else { 爪停 = 0 }
                    爪读 = 新读;
                    println!("[身]     抓握 步 {走了}:读数 {爪读:.3}");
                    if 爪停 >= 2 { 事件 = format!("resist: my grip stopped at reading {爪读:.3}"); break }
                    if 走了 >= 安全上限 { break }
                    continue;
                }
                let 误 = 误于(&项们);
                let 行: Vec<Vec<f64>> = (0..维).map(|r| {
                    let pi = r / 3;
                    let k = if r % 3 == 2 { 深换于(项们[pi].该.2) * 项们[pi].深权 } else { 1.0 };
                    (0..列数).map(|c| 表[c][r] * k).collect()
                }).collect();
                let Some(动) = 最小二乘(&行, &误) else { 事件 = "could not solve which channels to push".into(); break };
                // 每个通道各自封顶在 幅0×缩(不整体按最大那个通道缩:CP 实测转动通道解出 5 rad,整体缩到千分之四,平移通道等于没动)。
                let 顶 = 幅0 * 缩;
                let 动: Vec<f64> = 动.iter().map(|x| x.clamp(-顶, 顶)).collect();
                let 幅 = 动.iter().take(通道数).map(|x| x.abs()).fold(0.0, f64::max);
                let mut 比 = 1.0f64;
                if 幅 <= 实到噪 {
                    if 换段(&mut 项们) { 上差 = 差于(&误于(&项们)); 不缩 = 0; 减半次 = 0; 静 = 0; 段 += 1; println!("[身]     到了接触点外面那一点 ⇒ 第二段:沿空隙那一侧进到接触点"); continue }
                    事件 = "amount: already there (what is left to push is within my own noise)".into(); break
                }
                // 别碰:只有模型点名的那几块(CN 实测:把所有块都当障碍,把手臂自己那块也算进去,一步都走不了)。预测每一块这一步落到哪,压进就减半步子,最多四次。
                let mut 撞死 = None;
                {
                    let 框们: Vec<point_gen::区> = 别碰框.borrow().iter().cloned().collect();
                    let mut 退 = 0;
                    while !框们.is_empty() {
                        let mut 撞 = None;
                        for (pi, p) in 项们.iter().enumerate() {
                            let du: f64 = (0..列数).map(|c| 表[c][3 * pi] * 动[c] * 比).sum();
                            let dv: f64 = (0..列数).map(|c| 表[c][3 * pi + 1] * 动[c] * 比).sum();
                            let (u1, v1) = (p.现.0 + du, p.现.1 + dv);
                            for (bi, b) in 框们.iter().enumerate() {
                                let (x0, y0, x1, y1) = (b.框[0] as f64 / fw as f64, b.框[1] as f64 / fh as f64, b.框[2] as f64 / fw as f64, b.框[3] as f64 / fh as f64);
                                if u1 >= x0 && u1 <= x1 && v1 >= y0 && v1 <= y1 { 撞 = Some(bi); }
                            }
                        }
                        match 撞 { None => break, Some(bi) => { if 退 >= 4 { 撞死 = Some(bi); break } 比 *= 0.5; 退 += 1; } }
                    }
                }
                if let Some(bi) = 撞死 { 事件 = format!("stopped: every step would push me onto a thing I must not touch (box {})", bi + 1); break }
                let 臂: Vec<f64> = 动.iter().take(通道数).copied().collect();
                let Some((实到, 挡)) = 发(plug, &臂, 比, 稳拍) else { 事件 = "the body refused the command".into(); break };
                println!("[身]     步 {}:误 {:.3} · 解 [{}] × {:.3} · 实到 [{}] · 挡={}",
                    走了 + 1, 差于(&误), 动.iter().map(|x| format!("{x:+.3}")).collect::<Vec<_>>().join(" "), 比,
                    实到.iter().map(|x| format!("{x:+.4}")).collect::<Vec<_>>().join(" "), 挡);
                let 前: Vec<(f64, f64, f64)> = 项们.iter().map(|p| p.现).collect();
                let 命: Vec<f64> = (0..列数).map(|c| 实到.get(c).copied().unwrap_or(0.0)).collect();
                let Some(f) = plug.sense() else { 事件 = "lost the picture".into(); break };
                let Some((_, _, g)) = 灰(&f, 工作相机.get()) else { 事件 = "lost the picture".into(); break };
                let mut 丢 = None;
                for pi in 0..项们.len() {
                    let (投影, 末, 半, 窗, 现, 尺m) = (项们[pi].投影, 项们[pi].末, 项们[pi].半, 项们[pi].窗, 项们[pi].现, 项们[pi].尺m);
                    let 读 = if 投影 { 投影自(&f, 眼, 末) } else {
                        // 表 × 这一步实际走的 = 预测这一块跑到哪;在那儿附近找,窗口 = 预测位移 + 三个模板半径。
                        // 找到的那一点深度必须和预测对得上(容差 = 这一块自己的尺寸或预测的深度变化,加深度噪声)——
                        // 否则是锁到背景/桌面上了(CO 实测:手臂走了,模板留在原地的背景上,误差永远不缩)。找不到就全画面再找一遍。
                        let du: f64 = (0..列数).map(|c| 表[c][3 * pi] * 命[c]).sum();
                        let dv: f64 = (0..列数).map(|c| 表[c][3 * pi + 1] * 命[c]).sum();
                        let dz: f64 = (0..列数).map(|c| 表[c][3 * pi + 2] * 命[c]).sum();
                        let (du, dv) = (du.clamp(-1.0, 1.0), dv.clamp(-1.0, 1.0));   // 一步不可能跑出一幅画面
                        let 窗r = (((du * fw as f64).powi(2) + (dv * fh as f64).powi(2)).sqrt() as usize) + 半 * 3;
                        let 容 = if 无深 { f64::INFINITY } else { 尺m.max(dz.abs()) + 深噪 };
                        let 模 = 项们[pi].模.clone();
                        let mut 得 = None;
                        let mut 看过: Vec<String> = Vec::new();
                        if let Some((u, v)) = 找块窗(fw, fh, &g, &模, 半, 现.0 + du, 现.1 + dv, 窗r) {
                            match 读深(plug, u, v, 窗) { Some(z) => { if (z - (现.2 + dz)).abs() <= 容 { 得 = Some((u, v, z)); } else { 看过.push(format!("窗内 ({u:.3},{v:.3}) 深 {z:.3}")); } } None => 看过.push(format!("窗内 ({u:.3},{v:.3}) 无深")) }
                        }
                        if 得.is_none() {
                            if let Some((u, v)) = 找块(fw, fh, &g, &模, 半) {
                                match 读深(plug, u, v, 窗) { Some(z) => { if (z - (现.2 + dz)).abs() <= 容 { 得 = Some((u, v, z)); } else { 看过.push(format!("全画面 ({u:.3},{v:.3}) 深 {z:.3}")); } } None => 看过.push(format!("全画面 ({u:.3},{v:.3}) 无深")) }
                            }
                        }
                        if 得.is_none() { println!("[身]     跟丢第 {} 号:预测在 ({:.3},{:.3}) 深 {:.3}(容差 {容:.3});候选 {}", 项们[pi].号, 现.0 + du, 现.1 + dv, 现.2 + dz, 看过.join(" · ")); }
                        得
                    };
                    match 读 { Some(x) => 项们[pi].现 = x, None => { 丢 = Some(项们[pi].号); } }
                }
                if let Some(k) = 丢 {
                    // 跟丢的是哪只手的手指 ⇒ 记下来,下一轮抖它的抓握重新认。
                    for p in 项们.iter() { if p.号 == k && p.是我 && !p.投影 { if let Some(a) = 手于(p.通道) { 需抖.borrow_mut().insert(a); 身位.borrow_mut().remove(&a); } } }
                    事件 = format!("lost sight of item {k} while moving (I will re-find that hand by moving its grip next turn)"); break
                }
                // 修表:实际发生的 vs 表预测的,差值按这一步实际走的命令记回去(Broyden)。
                let 量 = 命.iter().map(|x| x * x).sum::<f64>();
                if 量 > 实到噪 * 实到噪 {
                    for (pi, p) in 项们.iter().enumerate() {
                        let 实 = [p.现.0 - 前[pi].0, p.现.1 - 前[pi].1, p.现.2 - 前[pi].2];
                        for r in 0..3 {
                            let 预: f64 = (0..列数).map(|c| 表[c][3 * pi + r] * 命[c]).sum();
                            let 差r = 实[r] - 预;
                            for c in 0..列数 { 表[c][3 * pi + r] += 差r * 命[c] / 量; }
                        }
                    }
                }
                let 实最大 = 实到.iter().take(通道数).map(|x| x.abs()).fold(0.0, f64::max);
                let 被拒 = 挡 || 实最大 <= 实到噪;    // 一步都没走(在本体自己的噪声以内)
                if 被拒 && 减半次 < 8 && 幅0 * 缩 * 0.5 > 实到噪 {
                    缩 *= 0.5; 减半次 += 1;
                    println!("[身]     一步没走(被拒)⇒ 步子减半重试({减半次}/8,现在 {:.4})", 幅0 * 缩);
                    continue;
                }
                走了 += 1;
                if !被拒 { 缩 = (缩 * 1.5).min(1.0); } 连成 = 0;
                if 合.is_some() {
                    let 新读 = 读爪(plug).unwrap_or(爪读);
                    if (新读 - 爪读).abs() <= 读数噪 { 爪停 += 1 } else { 爪停 = 0 }
                    爪读 = 新读;
                }
                let 新差 = 差于(&误于(&项们));
                // "更近了"要超过各块自己的抖动才算;连着两步(= 量噪声用的两次读)没更近 ⇒ 停。
                if 上差 - 新差 > 跟踪噪 { 上差 = 新差; 不缩 = 0; } else { 不缩 += 1; }
                let 画静 = match 上帧.as_ref() { Some(f) if f.len() == g.len() => f.iter().zip(g.iter()).map(|(a, b)| a.abs_diff(*b)).max().unwrap_or(0) <= 静噪, _ => false };
                上帧 = Some(g.clone());
                if 画静 { 静 += 1 } else { 静 = 0 }
                if 被拒 {
                    let 手们: Vec<String> = 项们.iter().filter(|p| p.是我).filter_map(|p| 手于(p.通道)).map(|a| format!("arm {}", a + 1)).collect::<std::collections::BTreeSet<_>>().into_iter().collect();
                    let 别的: Vec<String> = (0..臂数).filter(|a| !手们.contains(&format!("arm {}", a + 1))).map(|a| format!("arm {}", a + 1)).collect();
                    事件 = format!("resist: the body refuses every step toward there with {} (a pose it will not go to from here) - not a contact.{}",
                        if 手们.is_empty() { "this hand".to_string() } else { 手们.join(" and ") },
                        if 别的.is_empty() { String::new() } else { format!(" You also have {} (its pieces are on the list).", 别的.join(" and ")) });
                    break
                }
                if 到什么为止 == "settle" && 静 >= 2 { 事件 = "settle: the picture stopped changing (within its own noise)".into(); break }
                // slip:握着的东西不再跟着我 —— 抓握读数掉回空合那一层(东西离开了指间)。
                if 到什么为止 == "slip" && 手里有.get() && 爪读 - 空 <= 读数噪 { 事件 = "slip: what I was holding has left my fingers".into(); break }
                if 合后.is_none() && 合.is_some() && 爪停 >= 2 && 到什么为止 == "resist" && 条.iter().all(|c| c.保持) { 事件 = format!("resist: my grip stopped at reading {爪读:.3}"); break }
                if 不缩 >= 2 {
                    if 换段(&mut 项们) { 上差 = 差于(&误于(&项们)); 不缩 = 0; 减半次 = 0; 静 = 0; 段 += 1; println!("[身]     外面那一点已不再靠近 ⇒ 第二段:沿空隙那一侧进到接触点"); continue }
                    // 说清是哪只手、还有哪只手 ——"推不动"在够不着的时候是误导(CU 实测:左手够不着,模型连着五段不换手)。
                    let 手们: Vec<String> = 项们.iter().filter(|p| p.是我).filter_map(|p| 手于(p.通道)).map(|a| format!("arm {}", a + 1)).collect::<std::collections::BTreeSet<_>>().into_iter().collect();
                    let 别的: Vec<String> = (0..臂数).filter(|a| !手们.contains(&format!("arm {}", a + 1))).map(|a| format!("arm {}", a + 1)).collect();
                    let 谁 = if 手们.is_empty() { String::new() } else { format!(" with {}", 手们.join(" and ")) };
                    let 另 = if 别的.is_empty() { String::new() } else { format!(" You also have {} (its pieces are on the list).", 别的.join(" and ")) };
                    事件 = if 要碰 { format!("{到什么为止}: I keep pushing{谁} but stop getting closer (remaining {新差:.3} of a frame) - either something holds me or that hand cannot reach farther from here.{另}") }
                           else { format!("amount: stopped getting closer{谁} (remaining {新差:.3} of a frame).{另}") };
                    break
                }
                if 走了 >= 安全上限 { break }
            }
            // 抓握 at X:瓣到了接触点(或再也近不了、或顶住)才合 —— 合到读数不再变。跟丢 / 拒绝 / 撞 / 解不出的那几种结局不合。
            if let Some((a, _)) = 合后 {
                let 可合 = 段 >= 2 && !(事件.starts_with("lost sight") || 事件.starts_with("stopped:") || 事件.starts_with("the body refused") || 事件.starts_with("could not solve") || 事件.starts_with("resist: the body refuses"));
                if 可合 {
                    爪令.set(Some((a, 0.0)));
                    let (mut 停, mut n) = (0u32, 0u32);
                    loop {
                        let _ = 发(plug, &vec![0.0; 通道数], 1.0, 稳拍);
                        let 新 = 读爪(plug).unwrap_or(爪读); n += 1;
                        if (新 - 爪读).abs() <= 读数噪 { 停 += 1 } else { 停 = 0 }
                        爪读 = 新;
                        if 停 >= 2 || n >= 安全上限 { break }
                    }
                    事件 = format!("{事件}; then I closed my grip until it stopped ({n} steps)");
                } else { 事件 = format!("{事件}; I did not close my grip (phase {段} of 2: did not get to the contact points)"); }
            }
            步缩.set(缩);
            爪令.set(None);
            if let Some((_, s)) = 合 {
                jaw = 爪读;
                if s < 0.0 {
                    if jaw - 空 > 读数噪 { 手里有.set(true); 握 = format!(" my grip stopped at {jaw:.3}, above the empty-close reading {空:.3} by more than its own noise: there IS something between my fingers."); }
                    else { 手里有.set(false); 握 = format!(" my grip closed to {jaw:.3} (empty-close reading {空:.3}): nothing between my fingers."); }
                } else { 手里有.set(false); 握 = format!(" my grip opened to {jaw:.3}."); }
            }
            // 身上每块对各通道的响应 ⇒ 记进身表(下一段不再探)。
            {
                let mut 表存 = 身表.borrow_mut();
                for (pi, p) in 项们.iter().enumerate() {
                    if !p.是我 || p.投影 { continue }
                    let mut v = vec![0.0; 3 * 列数];
                    for c in 0..列数 { for r in 0..3 { v[3 * c + r] = 表[c][3 * pi + r]; } }
                    表存.insert(p.通道, v);
                }
            }
            // 手指此刻在哪 ⇒ 记进身位,下一轮的编号表用它(框按跟到的位置平移,大小不变)。
            for p in 项们.iter() {
                if p.是我 && !p.投影 && p.通道 < usize::MAX - 200 && p.通道 >= 臂通道数 {
                    if let Some(a) = 手于(p.通道) {
                        let (hw, hh) = ((p.框[2] - p.框[0]) * 0.5, (p.框[3] - p.框[1]) * 0.5);
                        let 新框 = [p.现.0 - hw, p.现.1 - hh, p.现.0 + hw, p.现.1 + hh];
                        // 认出了两瓣的手:第 j 个抓握槽 = 第 j 瓣,各自跟着走;没认出两瓣的,合起来那一块跟着走。
                        let j = (臂通道数..p.通道).filter(|&q| 手于(q) == Some(a)).count();
                        let mut 瓣 = 瓣位.borrow_mut();
                        match 瓣.get_mut(&a) { Some(v) if j < v.len() => { v[j] = 新框; } _ => { 身位.borrow_mut().insert(a, 新框); } }
                    }
                }
            }
            let 末 = 差于(&误于(&项们));
            let 报 = format!("you asked {}{}: {}. I took {} steps{}; remaining error {:.3} of a frame. {}.{}{}",
                项们.iter().map(|p| p.说.clone()).collect::<Vec<_>>().join(" and "),
                match 合 { Some((_, s)) if s < 0.0 => " and to close my grip", Some(_) => " and to open my grip", None => "" }, 事件, 走了,
                if 抓法数 > 0 { format!(" (contact-point approach: ended in phase {段} of 2)") } else { String::new() }, 末,
                项们.iter().map(|p| format!("item {} now at ({:.2},{:.2}) depth {:.2}", p.号, p.现.0, p.现.1, p.现.2)).collect::<Vec<_>>().join(", "),
                if 备注.is_empty() { String::new() } else { format!(" note: {}", 备注.join("; ")) }, 握);
            println!("[身]   ⇒ {报}");
            报
        };
        if let Some(d) = 这一段.as_ref() {
            if d.完了 {
                println!("[身]    ⇒ 它说做完了 ⇒ 这一拍不动手,下一拍重看(**做没做完由官方判据说了算,不由它说了算**)");
                上一段汇报 = "you said it is already done. the body did nothing and is looking again.".into();
                plug.act(&Cmd::Hold);
                continue;
            }
            // 🔴🔴🔴 **模型说什么,执行器就解什么。驱动里没有别的路。**(owner 2026-09-03:"你能做的唯一的事就是给 driver 说'抓起棒球'")
            let j现 = 帧.jaw.get(手号.get()).or_else(|| 帧.jaw.first()).copied().unwrap_or(1.0);
            上一段汇报 = 执行目标组(plug, &d.条, &d.到什么为止, d.快, j现, *通道是关节);
            if let Some(e) = 新眼.borrow_mut().take() { 眼稳 = Some(e); }
            continue;
        }
        // 模型没给这一段(问不通 / 没被问)⇒ 站着,下一拍重看重问。驱动自己不决定任何动作。
        plug.act(&Cmd::Hold);
        continue;
    }
}

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
//! submit <量名> <值[,值..]> <1σ> <有效下界> <有效上界> <单位文件> <出处文件>
//!                          -> ok ... | err ...   「把一个量写进标定库,出处必填」
//! holding <合爪时读数> <抬起后读数> <空爪门槛> <算滑的门槛>
//!                          -> held | slipped | empty
//! verbs                    -> <所有动词名>
//! quit
//! ```
//!
//! 认不得的行答 `err unknown`。**任何一条都不会悄悄给默认值** —— 这一层的全部意义就是
//! "没量到就说没量到"。
//!
//! ```text
//! floorcal <下压停位文件:每行 x y z> [容差 m]   -> val z0 .. | refused <理由>
//! floorat  <x> <y> <手停在的z> <几倍带宽>
//!        -> onfloor | onsomething 高 <m> | armlimit 低于面 <m> | refused <理由>
//! ```

use body_layer::floor::{fit as floor_fit, read_stop, Stop};
use body_layer::store::{Answer, Store};
use body_layer::verb::{
    classify, contact_seen, decide, demand, holding, spannable, turn_before_lift, Axes, Check, Hold,
    Next, Verb,
};
use body_layer::measurement::Quantity;
use body_layer::probe::{at_home, gripper_span_by_stall, home_pose};
use body_layer::Body;
use std::io::{self, BufRead, Write};

thread_local! {
    /// 当前 `load` 的标定文件路径。`submit` 写回同一份。
    static CAL: std::cell::RefCell<Option<String>> = const { std::cell::RefCell::new(None) };
    /// 本次会话 `homecal` 量到的原位。`athome` 拿它当尺子。
    static HOME: std::cell::RefCell<Option<body_layer::measurement::Measurement>> =
        const { std::cell::RefCell::new(None) };
    /// 本次会话 `floorcal` 拟合出来的**能下到多低**。`floorat` 拿它判"手停在这儿是被什么停住的"。
    static FLOOR: std::cell::RefCell<Option<Body>> = const { std::cell::RefCell::new(None) };
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
                        // 顺手把它还原成一个身体 —— 后面所有"要过闸"的问答都靠这一份。
                        let (b, admitted, rejected) = seed_body(&s);
                        FLOOR.with(|f| *f.borrow_mut() = Some(b));
                        *store = Some(s);
                        CAL.with(|c| *c.borrow_mut() = Some(p.to_string()));
                        format!("ok {ok} {all} {fp} 过闸 {admitted} 闸拒 {rejected}")
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
        // 把一个量【写进】标定库。读那一半 2026-08-12 已经搬进驱动,写这一半也得在驱动里 ——
        // 否则"身体常数只有一个出口"这条,写侧仍然是个洞。
        //
        // 🔴 出处是**必填**的,而且必须来自文件(命令行塞不下一段真出处)。
        //    没有出处的数就是手填的数,而手填的身体常数正是这一层要废掉的东西。
        "submit" => {
            let (Some(name), Some(vs), Some(sig), Some(lo), Some(hi), Some(unit), Some(prov)) =
                (t.get(1), t.get(2), t.get(3), t.get(4), t.get(5), t.get(6), t.get(7))
            else {
                return "err submit 要 <量名> <值[,值..]> <1σ> <有效下界> <有效上界> <单位文件> <出处文件>"
                    .into();
            };
            // 🔴 有效区间是**必填**的,而且这里就要挡住 —— 不然写进去的行会被准入闸拒绝,
            //    于是"我提交了"和"它能被读到"分岔。**2026-08-12 实测踩过**:
            //    第一版 submit 不收区间,写完之后 `get` 报
            //    *"the body layer rejected the stored row ... most often valid_lo >= valid_hi"*,
            //    而 `ask` 那条路读的是 JSON 不走闸,所以看起来像成功了。
            let (Ok(lo_v), Ok(hi_v)) = (lo.parse::<f64>(), hi.parse::<f64>()) else {
                return "err 有效区间读不出来".into();
            };
            if !(lo_v < hi_v) {
                return format!("err 有效下界必须小于上界(收到 {lo_v} .. {hi_v})—— 准入闸会拒绝这样的行");
            }
            let value: Vec<f64> = match vs.split(',').map(|x| x.parse::<f64>().ok()).collect() {
                Some(v) => v,
                None => return "err 值读不出来".into(),
            };
            let Ok(sigma) = sig.parse::<f64>() else {
                return "err 1σ 读不出来".into();
            };
            let (Ok(unit_s), Ok(prov_s)) =
                (std::fs::read_to_string(unit), std::fs::read_to_string(prov))
            else {
                return "err 单位或出处文件读不了".into();
            };
            if prov_s.trim().is_empty() {
                return "err 出处是空的 —— 没有出处的数就是手填的数".into();
            }
            let Some(path) = CAL.with(|c| c.borrow().clone()) else {
                return "err 还没 load 标定".into();
            };
            if sigma < 0.0 {
                return "err 1σ 不能是负的 —— 准入闸会拒绝".into();
            }
            match write_quantity(&path, name, &value, sigma, lo_v, hi_v, unit_s.trim(), prov_s.trim()) {
                Err(e) => format!("err {e}"),
                Ok(()) => format!("ok 写入 {name} = {value:?} +- {sigma} 到 {path}"),
            }
        }
        // 🔴 **我能下到多低** —— 一格网的下压停位拟合成一个面。
        //
        // 这一格(`Quantity::Floor`)的估计器 2026-08-09 就写好了,`bl_floor_fit` 也在 C ABI 里,
        // **但 `bl` 进程从没把它暴露出来**;而 ctypes 那层已按规矩删掉 ⇒ 没有任何人能调到它。
        // 「能被调用」不等于「被调用了」,这正是本层 README 自己写下的那条。
        // 代价照记(2026-08-13,实测):策略层因此只能拿 `reach` 那个**水平圆环**判可达,
        // 答不了"腕朝下、这个 xy、下到这个 z" —— 法兰残差与爪尖残差**完全相等 0.0482 m**
        // (排除了工具偏置算错),每一次失败都落在这一格上。
        "floorcal" => {
            let Some(path) = t.get(1) else {
                return "err floorcal 要 <下压停位文件:每行 x y z> [容差 m]".into();
            };
            let tol: f64 = t.get(2).and_then(|x| x.parse().ok()).unwrap_or(0.004);
            let src = match std::fs::read_to_string(path) {
                Ok(s) => s,
                Err(e) => return format!("err 读不了 {path}: {e}"),
            };
            let (mut xs, mut ys, mut zs) = (Vec::new(), Vec::new(), Vec::new());
            for ln in src.lines() {
                let f: Vec<&str> = ln.split_whitespace().collect();
                if f.len() < 3 {
                    continue;
                }
                if let (Ok(a), Ok(b), Ok(c)) =
                    (f[0].parse::<f64>(), f[1].parse::<f64>(), f[2].parse::<f64>())
                {
                    xs.push(a);
                    ys.push(b);
                    zs.push(c);
                }
            }
            match floor_fit(&xs, &ys, &zs, tol, 0, 0, 0) {
                Err(d) => format!("refused {d:?} (n={})", xs.len()),
                Ok(m) => {
                    // 提交进**已加载的那份标定还原出来的身体** —— floor 是拿"命令走了多少 vs
                    // 实到多少"那把尺子量的,所以闸要求 contact_threshold / step_delivery
                    // 已经在这具身体里。提交进一个空身体会被拒,而且拒得对(实测 2026-08-13:
                    // `UnmeasuredDependency`)。
                    FLOOR.with(|f| match f.borrow_mut().as_mut() {
                        None => "err 还没 load 标定".to_string(),
                        Some(b) => match b.submit(m) {
                            Err(e) => format!("refused 准入闸: {e:?}"),
                            Ok(_) => {
                                let (v, s) = (m.value, m.uncertainty);
                                format!(
                                    "val z0 {:.5} dz/dx {:.5} dz/dy {:.5} sigma {:.5} \
                                     x {:.4}..{:.4} y {:.4}..{:.4} n {}",
                                    v[0], v[1], v[2], s[0],
                                    m.valid_lo[0], m.valid_hi[0],
                                    m.valid_lo[1], m.valid_hi[1],
                                    xs.len()
                                )
                            }
                        },
                    })
                }
            }
        }
        // 手停在这儿 —— 是**停在面上** / **上面有东西** / 还是**这条胳膊没解了**。
        // 🔴 第三种正是"够不到",而它和"碰到了"在命令-实到那把尺子上长得一模一样。
        "floorat" => {
            let Some(a) = nums(t, 4) else {
                return "err floorat 要 <x> <y> <手停在的z> <几倍带宽>".into();
            };
            FLOOR.with(|f| match f.borrow().as_ref() {
                None => "err 还没 floorcal".to_string(),
                Some(b) => {
                    let r = read_stop(b, a[0], a[1], a[2], a[3]);
                    match (r.verdict.admit, r.stop) {
                        (false, _) => format!("refused {:?}", r.verdict.why),
                        (true, Some(Stop::OnFloor)) => format!("onfloor z {:.5}", r.floor_z),
                        (true, Some(Stop::OnSomething(h))) => {
                            format!("onsomething 高 {:.5} floor {:.5}", h, r.floor_z)
                        }
                        (true, Some(Stop::ArmLimit(d))) => {
                            format!("armlimit 低于面 {:.5} floor {:.5}", d, r.floor_z)
                        }
                        (true, None) => "refused 没有读数".to_string(),
                    }
                }
            })
        }
        // 拿没拿住 —— 只看爪子自己的读数,不用物体位姿,真机上照样能问。
        "holding" => match nums(t, 4) {
            None => "err holding 要 <合爪时读数> <抬起后读数> <空爪门槛> <算滑的门槛>".into(),
            Some(a) => match holding(a[0], a[1], a[2], a[3]) {
                Hold::Held => "held".into(),
                Hold::Slipped => "slipped".into(),
                Hold::WasEmpty => "empty".into(),
            },
        },
        "verbs" => (0..13u32)
            .filter_map(Verb::from_u32)
            .map(verb_name)
            .collect::<Vec<_>>()
            .join(" "),
        _ => "err unknown".into(),
    }
}

/// 🔴 **把一份存在磁盘上的标定还原成一个 `Body`。**
///
/// 在这之前没有这条路:`ask` 直接读 JSON,**绕过准入闸**;而闸住在 `Body` 里。
/// 后果是消费方"量到没量到"看得见,**"问出界了没有"看不见** —— 一具身体可以拿着一个
/// 只在 0.13–0.60 m 上量过的数,去回答 0.90 m 处的问题,而没有任何一环会不一致。
///
/// 返回 (身体, 进闸的行数, 被闸拒的行数)。**被拒的那个数本身就是一条读数** ——
/// 它说的是"这份标定里有几行,闸看了会不认"。
fn seed_body(store: &Store) -> (Body, usize, usize) {
    use body_layer::measurement::{AxisKind, Measurement, MAX_DEPS, MAX_DIM};
    let mut b = Body::new();
    let (mut ok, mut rejected) = (0usize, 0usize);
    for qi in 0..Quantity::COUNT as u32 {
        let Some(q) = Quantity::from_u32(qi) else { continue };
        let Answer::Measured { value, uncertainty, valid_lo, valid_hi, selftest_passed, .. } =
            store.ask(q.as_str())
        else {
            continue;
        };
        // 有效区间缺失 ⇒ 不许自己编一个。这一行就当它没进过闸。
        if valid_lo.is_empty() || valid_hi.is_empty() {
            rejected += 1;
            continue;
        }
        let mut m = Measurement {
            axis_kind: [AxisKind::Interval; MAX_DIM],
            quantity: q,
            dim: value.len().min(MAX_DIM),
            value: [0.0; MAX_DIM],
            uncertainty: [0.0; MAX_DIM],
            valid_lo: [0.0; MAX_DIM],
            valid_hi: [0.0; MAX_DIM],
            measured_at_ns: 0,
            valid_for_ns: 0,
            deps: [None; MAX_DEPS],
            epoch: 0,
            selftest_passed,
            prev_epoch: 0,
        };
        for i in 0..m.dim {
            m.value[i] = value[i];
            m.uncertainty[i] = *uncertainty.get(i).unwrap_or(&0.0);
            m.valid_lo[i] = *valid_lo.get(i).unwrap_or(&valid_lo[0]);
            m.valid_hi[i] = *valid_hi.get(i).unwrap_or(&valid_hi[0]);
        }
        match b.submit(m) {
            Ok(_) => ok += 1,
            Err(_) => rejected += 1,
        }
    }
    (b, ok, rejected)
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
        // 劈开的两档:名字里就写清楚该往哪边修。
        Check::StoppedWide => "stoppedwide",
        Check::PinchedThinner => "pinchedthinner",
    }
}

fn check_of(s: &str) -> Option<Check> {
    Some(match s {
        "asplanned" | "0" => Check::AsPlanned,
        "closedonair" | "1" => Check::ClosedOnAir,
        "stoppedwide" | "4" => Check::StoppedWide,
        "pinchedthinner" | "5" => Check::PinchedThinner,
        "wrongsection" | "2" => Check::WrongSection,
        "knockedaway" | "3" => Check::KnockedAway,
        _ => return None,
    })
}

/// 把一个量写回标定文件。**先解析整份、只换那一格、再整份写回** ——
/// 逐行改文本会在别的量上留下随机的格式差异,而标定文件的 diff 必须只反映真正变了的量。
#[allow(clippy::too_many_arguments)]
fn write_quantity(
    path: &str,
    name: &str,
    value: &[f64],
    sigma: f64,
    lo: f64,
    hi: f64,
    unit: &str,
    prov: &str,
) -> Result<(), String> {
    use body_layer::json::{parse, Json};
    use std::collections::BTreeMap;
    let src = std::fs::read_to_string(path).map_err(|e| format!("读不了 {path}: {e}"))?;
    let mut root = parse(&src)?;
    let Json::Obj(ref mut top) = root else {
        return Err("标定文件的顶层不是对象".into());
    };
    let mut q = BTreeMap::new();
    q.insert("value".to_string(), Json::Arr(value.iter().map(|v| Json::Num(*v)).collect()));
    q.insert(
        "uncertainty".to_string(),
        Json::Arr(value.iter().map(|_| Json::Num(sigma)).collect()),
    );
    q.insert("dim".to_string(), Json::Num(value.len() as f64));
    q.insert("valid_lo".to_string(), Json::Arr(value.iter().map(|_| Json::Num(lo)).collect()));
    q.insert("valid_hi".to_string(), Json::Arr(value.iter().map(|_| Json::Num(hi)).collect()));
    q.insert("unit".to_string(), Json::Str(unit.to_string()));
    q.insert("provenance".to_string(), Json::Str(prov.to_string()));
    q.insert("selftest_passed".to_string(), Json::Bool(true));
    let ent = top
        .entry("quantities".to_string())
        .or_insert_with(|| Json::Obj(BTreeMap::new()));
    let Json::Obj(ref mut qs) = ent else {
        return Err("quantities 不是对象".into());
    };
    // 🔴 覆盖之前,把原来那一格**原样存进 `superseded`** —— 一个身体常数被换掉时,
    //    旧值和旧出处不许无声消失,否则没人能回答"这个数什么时候变的、凭什么变"。
    let mut newq = Json::Obj(q);
    if let (Some(old), Json::Obj(ref mut nq)) = (qs.get(name).cloned(), &mut newq) {
        nq.insert("superseded".to_string(), old);
    }
    qs.insert(name.to_string(), newq);
    std::fs::write(path, root.dump(0)).map_err(|e| format!("写不了 {path}: {e}"))?;
    Ok(())
}

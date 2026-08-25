//! 线缆格式的搬运工:msgpack ↔ rmpv。**判断一律不在这里** —— 见 `../README.md`。

use rmpv::Value;

pub fn reply(req: &Value, kind: &str, payload: Value) -> Value {
    let mut m: Vec<(Value, Value)> = Vec::new();
    m.push((Value::String("message_type".into()), Value::String(kind.into())));
    // 客户端靠这个把回包和请求对上号,原样带回。
    m.push((
        Value::String("message_id".into()),
        get(req, "message_id").cloned().unwrap_or(Value::Nil),
    ));
    // 🔴🔴 **请求里没有的键,一个都不许放进回包。**
    //
    // 对方的帧模型是 `extra="forbid"`,而且 `evaluation_id: str`、`step: int` 是**不可为空**的。
    // 我原来无条件把 `action_case_id / trial_id / repeat_index / step / sent_at` 全塞进去,
    // 请求里没有就填 `Nil` ⇒ 校验失败 ⇒ **整帧被静默丢弃**,客户端的同步 RPC 永远等不到。
    //
    // 实测代价(2026-08-15):线上只出现一条 `收 hello ⇒ 回 hello_ack`,之后仿真再不说话。
    // 表现是"仿真卡住":连接 ESTABLISHED、场景全建好、单核 100% 忙等、GPU 0%、无磁盘、
    // **两侧日志都不报错**。查了一个多小时、否掉 8 个假设,才靠"在驱动里打印线上每一条消息"
    // 看见握手就断了,再读对方的帧定义才知道是**多填了字段**。
    // ⇒ 规矩:**回包的形状由对方的模型定义决定,不由我方便决定。**
    for k in ["evaluation_id", "action_case_id", "trial_id", "repeat_index", "sent_at"] {
        if let Some(v) = get(req, k) {
            m.push((Value::String(k.into()), v.clone()));
        }
    }
    // `step` 是 `int`(有默认值 0,但不接受 null)⇒ 请求里没有就给 0,绝不给 Nil。
    m.push((
        Value::String("step".into()),
        get(req, "step").cloned().unwrap_or(Value::Integer(0.into())),
    ));
    m.push((Value::String("payload".into()), payload));
    Value::Map(m)
}

/// map 取键。**字符串键和字节串键都要认** —— `msgpack_numpy` 用的是字节串键(`b"nd"`),
/// 只认一种会得到"字段明明在、却读不到"这种最难查的空值。
pub fn get<'a>(v: &'a Value, key: &str) -> Option<&'a Value> {
    let Value::Map(m) = v else { return None };
    m.iter().find(|(k, _)| key_is(k, key)).map(|(_, val)| val)
}

pub fn key_is(k: &Value, want: &str) -> bool {
    match k {
        Value::String(s) => s.as_str() == Some(want),
        Value::Binary(b) => b.as_slice() == want.as_bytes(),
        _ => false,
    }
}

pub fn as_text(v: &Value) -> Option<String> {
    match v {
        Value::String(s) => s.as_str().map(|x| x.to_string()),
        Value::Binary(b) => String::from_utf8(b.clone()).ok(),
        _ => None,
    }
}

/// 一次钳口标定的进度。**每条连接一份**,断线即作废 —— 隔着一次重置攒起来的帧,拍的不是同
/// 一个场景。
///
/// 五帧的顺序是被协议逼出来的:每一帧观测反映的是**上一步动作之后**的状态,所以要先白发两步
/// 拿到"什么都没命令"的那一对,再连发两次同样的钳口命令。
pub fn dig<'a>(v: &'a Value, path: &[&str]) -> Option<&'a Value> {
    let mut cur = v;
    for k in path {
        cur = get(cur, k)?;
    }
    Some(cur)
}

/// 一串数,两种编码都认:普通 msgpack 数组,和 `msgpack_numpy` 的 `<f4`/`<f8` 数组。
///
/// 观测里两种都出现过(`left_ee_joint_state` 是普通数组,`left_arm_joint_state` 是 ndarray),
/// 只认一种会得到"这个键明明在、却读成空"。
pub fn as_nums(v: &Value) -> Option<Vec<f64>> {
    if let Value::Array(a) = v {
        let out: Vec<f64> = a.iter().filter_map(|x| x.as_f64()).collect();
        return if out.len() == a.len() { Some(out) } else { None };
    }
    if !matches!(get(v, "nd").and_then(|x| x.as_bool()), Some(true)) {
        return None;
    }
    let ty = as_text(get(v, "type")?)?;
    let Value::Binary(data) = get(v, "data")? else { return None };
    let out: Vec<f64> = if ty.ends_with("f4") {
        data.chunks_exact(4)
            .map(|c| f32::from_le_bytes([c[0], c[1], c[2], c[3]]) as f64)
            .collect()
    } else if ty.ends_with("f8") {
        data.chunks_exact(8)
            .map(|c| f64::from_le_bytes([c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7]]))
            .collect()
    } else {
        return None;
    };
    Some(out)
}

/// 一条动作:**两条臂的关节都原样送回**(所以胳膊不动),只改标定那条臂的钳口。
///
/// 🔴 必须带上关节键:`eval_env.py::get_action_type` 是**靠键名**判断动作类型的,只发钳口那
/// 一个键会得到 "Cannot infer action type"。
///
/// 🔴🔴 **而且两条臂都必须给全,尽管校验说可以不给。** 实测:`validate_action_dict` 明写
/// "缺的键跳过",可 `take_action_batch` 紧接着**对每一台 target 机器人硬取** `action[key_name]`
/// ⇒ 只发左臂得到 `KeyError: 'right_arm_joint_state'`。**一个说"可以少给"、一个说"少给就崩",
/// 以校验为准会当场炸。**
/// 走位用的一条动作:标定臂给**绝对末端位姿**(xyz + 原来的朝向),另一条臂关节原样不动。
///
/// 姿态原样带回而不是自己编一个 —— 起手朝向是这台机器人自己选的,换掉它等于顺手改了一个
/// 没人量过的量。
pub fn hold_action(
    left_j: &[f64],
    right_j: &[f64],
    left_jaw: f64,
    right_jaw: f64,
) -> Value {
    let arr = |v: &[f64]| Value::Array(v.iter().map(|x| Value::F64(*x)).collect());
    Value::Map(vec![
        (Value::String("left_arm_joint_state".into()), arr(left_j)),
        (Value::String("right_arm_joint_state".into()), arr(right_j)),
        (Value::String("left_ee_joint_state".into()), arr(&[jaw_raw(left_jaw)])),
        (Value::String("right_ee_joint_state".into()), arr(&[jaw_raw(right_jaw)])),
    ])
}

/// 钳口**发出去就是 0–1 归一化**,这里不做任何换算 —— 保留这个函数只为把下面那条**已撤回**的
/// 判断连同它的证据一起留在原地,免得下一个人再推一遍同样的错。
///
/// 🔴 **撤回(2026-08-15,当天自纠)**:我曾判定"读到的是归一化、要发的是关节原始单位",
/// 并据此加过一次换算。**错的。** 真正的换算发生在评测端自己手里:
/// `src/eval_client/eval_env.py::take_action_batch` 收到动作后先 `np.clip(val, 0, 1)`
/// **再**乘量程转成原始单位。⇒ 动作侧本来就是归一化,驱动原来的写法是对的。
///
/// 我读错了层:`control_manager.py::process_gripper_val` 确实拿原始单位和真实关节比,
/// 但它拿到的是**上面那步已经转换过**的值,不是我们发的那个。
///
/// ⚠️ **让我误判的那个"证据"本身是假的**:当时回读恒为 `1.000`,我读成"命令被判成张到底"。
/// 真相是那会儿**动作根本没被执行**(回包漏了 `result` 键),`1.000` 是重置后从没被动过的默认值。
/// ⇒ **在一条链还没证明"命令确实生效"之前,任何"命令值 vs 回读值"的推断都是空中楼阁。**
/// 顺带记准:回读并不是真实关节,而是**上一步命令的回声**(`obs_manager` 拿 `prev_control` 再归一化),
/// 所以它能证明"命令收到了",**不能**证明"钳口真的动了"——后者只有画面能证。
///
/// 仍然成立、且会咬人的一条:`process_gripper_val` 里 `gripper_eps=0.2` 把每步变化夹在满量程
/// 20% 以内 ⇒ **从全闭到全开至少 5 步**,一步发不到位。
pub fn jaw_raw(norm: f64) -> f64 {
    norm.clamp(0.0, 1.0)
}

/// 把观测里所有 `msgpack_numpy` 的 uint8 图像存成 PGM,返回存了几张。
///
/// 走遍整棵结构而不是去猜键名:相机叫什么、嵌在第几层,是**这台机器**的事,不该写死在这里。
pub fn as_rgb(v: &Value) -> Option<(usize, usize, Vec<u8>)> {
    if !matches!(get(v, "nd").and_then(|x| x.as_bool()), Some(true)) {
        return None;
    }
    let ty = as_text(get(v, "type")?)?;
    if !ty.ends_with("u1") {
        return None;
    }
    let Value::Array(shape) = get(v, "shape")? else { return None };
    let dims: Vec<usize> = shape.iter().filter_map(|d| d.as_u64()).map(|d| d as usize).collect();
    let data = match get(v, "data")? {
        Value::Binary(b) => b.clone(),
        _ => return None,
    };
    match dims.as_slice() {
        [h, w, 3] if data.len() >= h * w * 3 => Some((*w, *h, data)),
        _ => None,
    }
}


/// 读一张 `<f4` 的二维图(深度图就是这个形状),返回 `(宽, 高, 逐行的值)`。
///
/// 🔴 **字节序按 `type` 里那个前缀判,不许假设**:`msgpack_numpy` 把 dtype 原样带过来
/// (`<f4` = 小端 float32),而**猜错字节序不会报错**,只会给出一批量级离谱但"看起来是数"的深度。
pub fn as_f32_grid(v: &Value) -> Option<(usize, usize, Vec<f64>)> {
    if !matches!(get(v, "nd").and_then(|x| x.as_bool()), Some(true)) {
        return None;
    }
    let ty = as_text(get(v, "type")?)?;
    if !ty.ends_with("f4") {
        return None;
    }
    let big = ty.starts_with('>');
    let Value::Array(shape) = get(v, "shape")? else { return None };
    let dims: Vec<usize> = shape.iter().filter_map(|d| d.as_u64()).map(|d| d as usize).collect();
    let data = match get(v, "data")? {
        Value::Binary(b) => b.clone(),
        _ => return None,
    };
    let [h, w] = dims[..] else { return None };
    if data.len() < h * w * 4 {
        return None;
    }
    let mut out = Vec::with_capacity(h * w);
    for i in 0..(h * w) {
        let b = [data[i * 4], data[i * 4 + 1], data[i * 4 + 2], data[i * 4 + 3]];
        let f = if big { f32::from_be_bytes(b) } else { f32::from_le_bytes(b) };
        out.push(f as f64);
    }
    Some((w, h, out))
}

/// 把观测整棵结构打印一次:数组只报类型和形状,短的一串数直接印出来(位姿/关节角正是要看的)。
///
/// **这是"先看仿真给什么"的工具,不是探测器** —— 猜键名是本仓栽过的坑。
pub fn describe(v: &Value, name: &str, depth: usize) {
    let pad = "  ".repeat(depth);
    match v {
        Value::Map(m) => {
            // ndarray 是一个 map,但它是**一个值**,不是一层结构。
            if matches!(get(v, "nd").and_then(|x| x.as_bool()), Some(true)) {
                let ty = get(v, "type").and_then(as_text).unwrap_or_else(|| "?".into());
                let shape: Vec<String> = match get(v, "shape") {
                    Some(Value::Array(a)) => a.iter().map(|d| format!("{d}")).collect(),
                    _ => vec!["?".into()],
                };
                println!("{pad}{name}: 数组 {ty} 形状[{}]", shape.join(","));
                return;
            }
            println!("{pad}{name}: 有 {} 项", m.len());
            for (k, val) in m {
                let kn = as_text(k).unwrap_or_else(|| format!("{k}"));
                describe(val, &kn, depth + 1);
            }
        }
        Value::Array(a) => {
            // 一串数(位姿、关节角)是**值**,短的就直接印出来 —— 这正是要看的东西。
            let nums: Vec<String> = a
                .iter()
                .filter_map(|x| x.as_f64().map(|f| format!("{f:.4}")))
                .collect();
            if nums.len() == a.len() && !a.is_empty() && a.len() <= 16 {
                println!("{pad}{name}: {} 个数 [{}]", a.len(), nums.join(" "));
                return;
            }
            println!("{pad}{name}: 列表 {} 项", a.len());
            for (i, val) in a.iter().enumerate().take(4) {
                describe(val, &format!("[{i}]"), depth + 1);
            }
            if a.len() > 4 {
                println!("{pad}  …还有 {} 项", a.len() - 4);
            }
        }
        Value::String(_) | Value::Binary(_) => {
            let t = as_text(v).unwrap_or_default();
            let head: String = t.chars().take(60).collect();
            println!("{pad}{name}: 字串 \"{head}\"");
        }
        other => println!("{pad}{name}: {other}"),
    }
}

/// **一条【关节】动作。** 与 [`pose_action`] 互斥 —— 这条线缆按键名判断动作类型,
/// 关节键与位姿键**同时出现整帧被拒**(`Multiple action types found`)。
///
/// 🔴 键名同样**不是拼的**,是认布局时看到的那个(`discover.rs` 认路径,最后一节就是键名)。
pub fn joint_action(关节键: &[String], 钳口键: &[String], 我: usize, q: &[f64], 关节: &[Vec<f64>], 钳口: &[f64]) -> Value {
    let arr = |v: &[f64]| Value::Array(v.iter().map(|x| Value::F64(*x)).collect());
    let n = 关节键.len().min(钳口键.len());
    let mut m = Vec::with_capacity(n * 2);
    for i in 0..n {
        let qi = if i == 我 { q.to_vec() } else { 关节.get(i).cloned().unwrap_or_default() };
        m.push((Value::String(关节键[i].as_str().into()), arr(&qi)));
        m.push((Value::String(钳口键[i].as_str().into()), arr(&[jaw_raw(*钳口.get(i).unwrap_or(&1.0))])));
    }
    Value::Map(m)
}

pub fn pose_action(
    位姿键: &[String],
    钳口键: &[String],
    我: usize,
    xyz: &[f64; 3],
    quat: &[f64],
    位姿: &[Vec<f64>],
    钳口: &[f64],
) -> Value {
    let arr = |v: &[f64]| Value::Array(v.iter().map(|x| Value::F64(*x)).collect());
    let mut mine: Vec<f64> = xyz.to_vec();
    mine.extend_from_slice(quat);
    // 🔴🔴 **一条动作里不许混两种类型。** `get_action_type` 靠键名判断:出现关节键算 joint,
    // 出现位姿键算另一种,**两种都出现直接 "Multiple action types found"**。所以每条臂都发位姿 ——
    // 不干活的那条发它**当前**的位姿,就是"待在原地"(发空数组 = 形状不完整 = 身体一步不动,
    // 而两侧零报错;这条已经付过一次学费)。
    // 🔴 **键名不是拼出来的,是【认布局时看到的那个】** —— `discover.rs` 认的是路径,
    // 路径的最后一节就是这台机器人自己用的键名。x5 双臂是 `left_ee_pose`,
    // 单臂 Franka 是 `ee_pose`;拼一个名字出来在后者上两边都对不上。
    let n = 位姿键.len().min(钳口键.len());
    let mut m = Vec::with_capacity(n * 2);
    for i in 0..n {
        let p = if i == 我 { mine.clone() } else { 位姿[i].clone() };
        m.push((Value::String(位姿键[i].as_str().into()), arr(&p)));
        m.push((Value::String(钳口键[i].as_str().into()), arr(&[jaw_raw(钳口[i])])));
    }
    Value::Map(m)
}

//! `cx` —— ②b 也变成一个**进程**,和 `bl`(驱动)、`cg`(下手点)同一个理由:
//! 一行一问,一行一答,任何语言、任何宿主都能问它。
//!
//! # 🔴 身体常数是【在协议里说出来的】,不是它自己去读
//!
//! 这一层不许依赖驱动 —— 反向依赖一出现,"换机体不重训"就没有机制保证了。
//! 所以调用方先问 `bl` 要,再把要到的数递进来。**这堵墙因此写在协议里,肉眼可见。**
//!
//! # 协议
//!
//! ```text
//! body <钳口m> <工具偏置m> <够到下界m> <够到上界m> <臂根x> <臂根y> <地板z> <重复精度m> -> ok | err
//! plan <碰x> <碰y> <碰z> <爪面朝向> <这段多宽m> <物体往哪走 x y z> <走多远m> <物体最高点z>
//!      -> 每行一个航点 "x y z yaw 开多少m 要碰吗0|1",末行 end | refused <理由>
//! off  <要碰吗0|1> <还差多远m>   -> 1(偏了,该重来) | 0
//! quit
//! ```
use contact_exec::{off_course, waypoints, Body, Contact, Motion, P3, Waypoint};
use std::io::{self, BufRead, Write};

fn main() {
    let stdin = io::stdin();
    let mut out = io::stdout();
    // 🔴 默认全是 NaN,不是某个数。忘了 `body` 那一行 ⇒ 拒绝,而不是拿一个猜的值往下跑。
    let mut body = Body {
        jaw_span_m: f64::NAN,
        tool_offset_m: f64::NAN,
        reach_lo_m: f64::NAN,
        reach_hi_m: f64::NAN,
        base_x: f64::NAN,
        base_y: f64::NAN,
        floor_z: f64::NAN,
        repeat_m: f64::NAN,
    };
    for line in stdin.lock().lines() {
        let Ok(line) = line else { break };
        let t: Vec<&str> = line.split_whitespace().collect();
        if t.is_empty() {
            continue;
        }
        if t[0] == "quit" {
            break;
        }
        let reply = handle(&t, &mut body);
        let _ = writeln!(out, "{reply}");
        let _ = out.flush();
    }
}

fn nums(t: &[&str], n: usize) -> Option<Vec<f64>> {
    if t.len() < n + 1 {
        return None;
    }
    t[1..=n].iter().map(|x| x.parse::<f64>().ok()).collect()
}

fn handle(t: &[&str], body: &mut Body) -> String {
    match t[0] {
        "body" => match nums(t, 8) {
            None => "err body 要 <钳口> <工具偏置> <够到下界> <够到上界> <臂根x> <臂根y> <地板z> <重复精度>".into(),
            Some(a) => {
                *body = Body {
                    jaw_span_m: a[0], tool_offset_m: a[1],
                    reach_lo_m: a[2], reach_hi_m: a[3],
                    base_x: a[4], base_y: a[5], floor_z: a[6], repeat_m: a[7],
                };
                "ok".into()
            }
        },
        "plan" => {
            if !body.jaw_span_m.is_finite() {
                return "err 还没给 body —— 一个身体常数都没有,这一层不许猜".into();
            }
            let Some(a) = nums(t, 10) else {
                return "err plan 要 <x> <y> <z> <朝向> <段宽> <往哪走 x y z> <多远> <物体顶z>".into();
            };
            let c = Contact { point: P3 { x: a[0], y: a[1], z: a[2] }, close_yaw: a[3], width_m: a[4] };
            let m = Motion { along: [a[5], a[6], a[7]], dist_m: a[8] };
            match waypoints(body, &c, &m, a[9]) {
                Err(r) => format!("refused {r:?}"),
                Ok(ws) => {
                    let mut s = String::new();
                    for w in ws.iter() {
                        s.push_str(&fmt_wp(w));
                        s.push('\n');
                    }
                    s.push_str("end");
                    s
                }
            }
        }
        "off" => match nums(t, 2) {
            None => "err off 要 <要碰吗0|1> <还差多远m>".into(),
            Some(a) => {
                if !body.repeat_m.is_finite() {
                    return "err 还没给 body —— 门槛只能由这具身体的重复精度给".into();
                }
                let w = Waypoint {
                    tcp: P3 { x: 0.0, y: 0.0, z: 0.0 },
                    yaw: 0.0,
                    open_m: 0.0,
                    touching: a[0] >= 0.5,
                };
                if off_course(body, &w, a[1]) { "1".into() } else { "0".into() }
            }
        },
        _ => "err unknown".into(),
    }
}

fn fmt_wp(w: &Waypoint) -> String {
    format!(
        "{:.5} {:.5} {:.5} {:.5} {:.5} {}",
        w.tcp.x, w.tcp.y, w.tcp.z, w.yaw, w.open_m, if w.touching { 1 } else { 0 }
    )
}

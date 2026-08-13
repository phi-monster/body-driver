//! 标定库 —— 这具身体量出来的东西存在哪、怎么读回来。
//!
//! # 为什么它 2026-08-12 才进驱动
//!
//! 量是 2026-08-09 起就量好的,但**读它的那段代码一直在驱动外面**(676 行 Python 的 ctypes 壳)。
//! 结果是:"身体常数只准向驱动要"这条规矩,靠的是一个驱动管不着的文件在维持 ——
//! 谁绕过那个文件手填一个数,驱动一无所知。已经发生过:接触阈值被手填成 0.35,物体被推走
//! 13.4 cm 而检测器一次没响。**把读取搬进来,驱动才是那批数字的唯一出口。**
//!
//! # 一条不许松的规矩
//!
//! **被拒绝的量,读出来必须还是【拒绝】。** 读成 0、读成默认值、读成"上次那个" —— 都等于
//! 驱动自己发明了一个身体常数,那正是这一层存在的理由的反面。

use crate::json::{parse, Json};
use std::collections::BTreeMap;

/// 向标定库问一个量,得到的东西。
#[derive(Clone, Debug, PartialEq)]
pub enum Answer {
    /// 量到了。`unit` 说的是它是什么单位 —— 本仓的接触阈值单位是"交付比例",**不是牛顿**。
    Measured {
        /// 值。多维量按维排。
        value: Vec<f64>,
        /// 1σ,与 `value` 同单位。
        uncertainty: Vec<f64>,
        /// 单位的人话说明。
        unit: String,
        /// 它是怎么量出来的。**空的出处 = 没有出处**,消费方该照拒绝处理。
        provenance: String,
        /// 提交时它自己的自检过没过。
        selftest_passed: bool,
        /// 🔴 **每个轴是哪一种**,按维排。文件里一直记着,而读的那一半以前**丢掉了它** ——
        /// 于是地面图这种「值轴与定义域轴不是同一组」的量(高度 0.92 m,定义域是 x 的 −0.6…+0.6)
        /// 被当成普通区间还原,值落在自己的"有效范围"外,**被闸挡掉而且不报错**:那个量就此
        /// 变成"从来没量过"。**文件里信息是全的,读的时候丢了一格** —— 本仓最贵的一类。
        axis_kind: Vec<String>,
        /// 🔴 **它是在哪一段范围上真的量过的**,按维排。问到范围外必须拒绝,而不是外推。
        ///
        /// 这两格 2026-08-13 之前**解析器读都不读**,于是一份存在磁盘上的标定
        /// **还原不回一个 `Body`** —— 而 `Body` 才是准入闸住的地方。后果是消费方
        /// 拿到的每一个身体量都绕过了闸:量到没量到看得见,**问出界了没有看不见**。
        valid_lo: Vec<f64>,
        /// 见 [`Self::Measured::valid_lo`]。
        valid_hi: Vec<f64>,
    },
    /// 没量到,且理由被保留下来了。
    Refused {
        /// 为什么拒绝。原样保留,不改写。
        why: String,
    },
    /// 这具身体的标定库里根本没有这一格。
    NeverMeasured,
}

/// 一份标定 = 一个指纹 + 一堆量。
#[derive(Clone, Debug)]
pub struct Store {
    /// 这具身体的指纹。**不含任何任务名 / benchmark 名** —— 那是它能跨任务复用的原因。
    pub fingerprint: String,
    q: BTreeMap<String, Answer>,
}

impl Store {
    /// 从一份标定文件的正文读出来。
    pub fn from_str(src: &str) -> Result<Store, String> {
        let j = parse(src)?;
        let fingerprint = j
            .get("fingerprint")
            .and_then(|x| x.text())
            .unwrap_or("")
            .to_string();
        let mut q = BTreeMap::new();
        if let Some(Json::Obj(m)) = j.get("quantities") {
            for (name, v) in m {
                q.insert(name.clone(), read_one(v));
            }
        }
        Ok(Store { fingerprint, q })
    }

    /// 问一个量。**没有这一格 ≠ 值是 0**,所以返回的是三态而不是 `Option<f64>`。
    pub fn ask(&self, name: &str) -> Answer {
        self.q.get(name).cloned().unwrap_or(Answer::NeverMeasured)
    }

    /// 库里有哪些格(不论量到没量到)。
    pub fn names(&self) -> Vec<&str> {
        self.q.keys().map(|s| s.as_str()).collect()
    }

    /// 量到了几个 / 一共几格。用来一眼看出这具身体"认识自己多少"。
    pub fn tally(&self) -> (usize, usize) {
        let ok = self
            .q
            .values()
            .filter(|a| matches!(a, Answer::Measured { .. }))
            .count();
        (ok, self.q.len())
    }
}

fn read_one(v: &Json) -> Answer {
    // 🔴 顺序承重:先看有没有拒绝理由。一个量可以【既有旧值又已被拒绝】,那时它是拒绝。
    if let Some(why) = v.get("refused").and_then(|x| x.text()) {
        return Answer::Refused { why: why.to_string() };
    }
    match v.get("value").map(|x| x.nums()) {
        Some(value) if !value.is_empty() => Answer::Measured {
            // 🔴 有效区间以前**读都不读**,于是磁盘上的标定还原不成一个 `Body`,
            //    而 `Body` 才是准入闸住的地方 —— 消费方拿到的每个数因此都绕过了闸。
            //    (`submit` 的注释早写着 *"ask 那条路读的是 JSON 不走闸"*;这里补上读的那一半。)
            axis_kind: match v.get("axis_kind") {
                Some(Json::Arr(a)) => a.iter().filter_map(|x| x.text().map(|s| s.to_string())).collect(),
                _ => Vec::new(),
            },
            valid_lo: v.get("valid_lo").map(|x| x.nums()).unwrap_or_default(),
            valid_hi: v.get("valid_hi").map(|x| x.nums()).unwrap_or_default(),
            value,
            uncertainty: v.get("uncertainty").map(|x| x.nums()).unwrap_or_default(),
            unit: v.get("unit").and_then(|x| x.text()).unwrap_or("").to_string(),
            provenance: v
                .get("provenance")
                .and_then(|x| x.text())
                .unwrap_or("")
                .to_string(),
            selftest_passed: matches!(v.get("selftest_passed"), Some(Json::Bool(true))),
        },
        // 有这一格但没有值也没有拒绝理由 —— 这是文件写坏了,当作拒绝,并把这件事说出来。
        _ => Answer::Refused {
            why: "标定文件里这一格既无 value 也无 refused".into(),
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SRC: &str = r#"{"fingerprint":"116bd6e559e10b06","quantities":{
        "contact_threshold":{"value":[0.29383],"uncertainty":[0.00039],"selftest_passed":true,
          "unit":"fraction of commanded motion delivered in one control period -- NOT newtons",
          "provenance":"measured 2026-08-11 with NO force sensor"},
        "tool_offset":{"value":[0.1451],"uncertainty":[0.0181],"selftest_passed":true},
        "gripper_span":{"refused":"NoResponse: the commanded motion did not move the signal",
          "selftest_passed":false},
        "broken":{"selftest_passed":true}}}"#;

    #[test]
    fn measured_quantities_come_back_with_their_provenance() {
        let s = Store::from_str(SRC).unwrap();
        assert_eq!(s.fingerprint, "116bd6e559e10b06");
        match s.ask("contact_threshold") {
            Answer::Measured { value, unit, .. } => {
                assert_eq!(value, vec![0.29383]);
                // 单位必须跟着值一起出来:这个数【不是牛顿】,拿它当力用过一次就出过事。
                assert!(unit.contains("NOT newtons"));
            }
            other => panic!("接触阈值该是量到的,却是 {other:?}"),
        }
    }

    #[test]
    fn a_refused_quantity_never_degrades_into_a_number() {
        let s = Store::from_str(SRC).unwrap();
        // 🔴 爪张开度在这具身体上是【拒绝】状态。任何把它读成数的实现都是错的 ——
        //    本仓用过的 8 cm 是手填的,②a 拿它判"夹不夹得下",一路判错。
        assert!(matches!(s.ask("gripper_span"), Answer::Refused { .. }));
        assert!(matches!(s.ask("never_heard_of_it"), Answer::NeverMeasured));
    }

    #[test]
    fn a_malformed_slot_is_a_refusal_not_a_zero() {
        let s = Store::from_str(SRC).unwrap();
        assert!(matches!(s.ask("broken"), Answer::Refused { .. }));
    }

    #[test]
    fn tally_says_how_much_of_itself_this_body_knows() {
        let s = Store::from_str(SRC).unwrap();
        assert_eq!(s.tally(), (2, 4));
    }
}

impl Store {
    /// 🔴 **把一份存下来的标定还原成一个能过闸的 [`crate::Body`]。**
    ///
    /// 存文件里是一行行数值,而下游每一个"这具身体能不能这么做"的问答都要一个 `Body` ——
    /// 因为只有 `Body` 带着不确定度、量过的范围、依赖和自检,能被**拒绝**。以前这段逻辑住在
    /// `bl` 这个二进制里,于是任何别的调用方(比如接线缆的那个进程)想用就得抄一遍,而抄出
    /// 来的第二份迟早和第一份不一样 —— 那正是本仓最贵的一类事故:两处规矩看起来一样,行为
    /// 不一样,而且都不报错。
    ///
    /// 返回 `(身体, 过闸几个, 被拒几个)`。**有效区间缺失的那些一律算被拒**:不许替它编一个,
    /// 编出来的范围会让下游在从没量过的地方拿到一个看起来正常的答案。
    pub fn to_body(&self) -> (crate::Body, usize, usize) {
        use crate::measurement::{AxisKind, Measurement, Quantity, MAX_DEPS, MAX_DIM};
        let mut b = crate::Body::new();
        let (mut ok, mut rejected) = (0usize, 0usize);
        for qi in 0..Quantity::COUNT as u32 {
            let Some(q) = Quantity::from_u32(qi) else { continue };
            let Answer::Measured { value, uncertainty, valid_lo, valid_hi, selftest_passed, axis_kind, .. } =
                self.ask(q.as_str())
            else {
                continue;
            };
            if valid_lo.is_empty() || valid_hi.is_empty() {
                rejected += 1;
                continue;
            }
            let mut m = Measurement {
                // 🔴 按文件里记的那一格还原,不再一律当区间。少这一句,地面图永远还原不出来。
                axis_kind: {
                    let mut k = [AxisKind::Interval; MAX_DIM];
                    for (i, s) in axis_kind.iter().enumerate().take(MAX_DIM) {
                        k[i] = match s.as_str() {
                            "categorical" => AxisKind::Categorical,
                            "unmeasured" => AxisKind::Unmeasured,
                            _ => AxisKind::Interval,
                        };
                    }
                    k
                },
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
}

#[cfg(test)]
mod axis_kind_tests {
    use super::*;
    use crate::measurement::Quantity;

    /// 🔴 **地面图必须能从磁盘上还原回来。**
    ///
    /// 它是本仓唯一一个「值轴与定义域轴不是同一组」的量:值是高度 0.92 m,而定义域是 x 的
    /// −0.6…+0.6。读的时候把 `axis_kind` 丢掉、一律当成区间,它的值就落在自己的"有效范围"外,
    /// **被准入闸挡掉,而且不报错** —— 那个量就此变成"从来没量过"。
    ///
    /// 实际后果:整条「按眼给的位置压下去,问问那儿有没有东西」的验收链,在最后一步答
    /// `NeverMeasured`。文件里信息一直是全的,**丢的是读的那一半**。
    #[test]
    fn the_floor_survives_a_round_trip_through_the_store() {
        let src = r#"{
          "fingerprint": "t",
          "quantities": {
            "floor": {
              "axis_kind": ["interval", "interval", "unmeasured"],
              "value": [0.920787, 0.000189, 0.000401],
              "uncertainty": [0.000257, 0.0, 0.0],
              "valid_lo": [-0.5995, -0.2302, 0.0],
              "valid_hi": [0.6005, -0.0225, 0.0],
              "unit": "m",
              "provenance": "按下去量的",
              "selftest_passed": true
            }
          }
        }"#;
        let s = Store::from_str(src).expect("解析得开");
        let (b, ok, rejected) = s.to_body();
        assert_eq!((ok, rejected), (1, 0), "地面图必须过闸");
        let m = b.get(Quantity::Floor).expect("身体里要有地面图");
        assert!((m.value[0] - 0.920787).abs() < 1e-6);
        assert_eq!(m.axis_kind[2], crate::measurement::AxisKind::Unmeasured, "第三轴是没量过");
    }

    /// 反证:把 `axis_kind` 从文件里拿掉(= 旧的读法),地面图就**还原不回来** —— 这条把
    /// 「丢掉那一格会怎样」钉死,免得以后有人"顺手简化"又把它删了。
    #[test]
    fn without_axis_kind_the_floor_is_refused_and_that_is_the_bug_we_had() {
        let src = r#"{
          "fingerprint": "t",
          "quantities": {
            "floor": {
              "value": [0.920787, 0.000189, 0.000401],
              "uncertainty": [0.000257, 0.0, 0.0],
              "valid_lo": [-0.5995, -0.2302, 0.0],
              "valid_hi": [0.6005, -0.0225, 0.0],
              "unit": "m",
              "provenance": "按下去量的",
              "selftest_passed": true
            }
          }
        }"#;
        let s = Store::from_str(src).expect("解析得开");
        let (b, ok, rejected) = s.to_body();
        assert_eq!((ok, rejected), (0, 1), "没有 axis_kind 就还原不出来");
        assert!(b.get(Quantity::Floor).is_none());
    }
}

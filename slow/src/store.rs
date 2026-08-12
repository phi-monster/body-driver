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

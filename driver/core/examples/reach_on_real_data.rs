//! Run the `reach` probe over real per-episode logs instead of synthetic ones.
//!
//! A probe that passes its own unit test has only been shown to agree with the data the test
//! author imagined. This feeds it what the arms actually did: one line per episode,
//! `robot,separation,radius_from_own_base_m,attained`, where `attained` is 0 exactly when the
//! episode ended with "could not reach grasp pose" -- the reachability signal itself, not overall
//! task success, which also fails for grasping reasons that have nothing to do with reach.
//!
//! Usage: `cargo run --example reach_on_real_data -- <csv>`
use body_layer::probe::{reach, Declined};
use std::collections::BTreeMap;

fn main() {
    let path = std::env::args().nth(1).expect("arg 1 = csv path");
    let text = std::fs::read_to_string(&path).expect("read csv");
    let mut groups: BTreeMap<String, Vec<(f64, bool)>> = BTreeMap::new();
    for line in text.lines() {
        let f: Vec<&str> = line.trim().split(',').collect();
        if f.len() != 4 {
            continue;
        }
        let key = format!("{:<6} ±{}", f[0], f[1]);
        let r: f64 = f[2].parse().expect("radius");
        let attained = f[3].trim() == "1";
        groups.entry(key).or_default().push((r, attained));
    }
    for (key, samples) in &groups {
        let n = samples.len();
        let hit = samples.iter().filter(|s| s.1).count();
        print!("{key}  n={n:<5} 够到 {hit:<5} ");
        match reach(samples, 1_000_000_000) {
            Ok(m) => println!(
                "带 = [{:.3}, {:.3}] m  ±[{:.3}, {:.3}]  扫过 [{:.3}, {:.3}]",
                m.value[0], m.value[1], m.uncertainty[0], m.uncertainty[1],
                m.valid_lo[0], m.valid_hi[0]
            ),
            Err(Declined::Inconsistent) => println!("拒答:某一堵墙没被样本跨过"),
            Err(Declined::NotEnoughSamples) => println!("拒答:样本不够定位两条边"),
            Err(Declined::NoResponse) => println!("拒答:一个都没够到"),
            Err(e) => println!("拒答:{e:?}"),
        }
    }
}

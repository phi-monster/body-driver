//! Turn the raw samples a body produced about ITSELF into a stored calibration.
//!
//! This is the half of self-calibration that decides. `body_probe.py` (in the deployed stack)
//! commands the motions and writes raw samples; nothing there estimates anything and nothing there
//! refuses anything. Both of those happen here, in the place where the estimators are unit-tested
//! and the refusal rules were derived, so that "the probe abstained" means the same thing in every
//! deployment.
//!
//! Usage: `cargo run --release --example calibrate_from_probes -- <raw_dir> [out.json]`
//!
//! `<raw_dir>` holds one `<phase>.jsonl` per phase. The reader below is a hand-rolled scanner --
//! this crate has **zero dependencies** on purpose, and that rule is load-bearing (it is what lets
//! the same code build for a target that has no allocator story). It reads only the few numeric
//! fields each probe needs and ignores everything else, so adding a field on the collection side
//! cannot break it.
//!
//! 🔴 WHAT IT PRINTS IS AS IMPORTANT AS WHAT IT STORES. Every quantity gets a line, and an
//! abstention is a first-class outcome with its own reason -- `NotEnoughSamples`, `NoResponse`,
//! `Inconsistent`, `MissingDependency` call for four different next moves and are never merged.
//! A calibration that silently omitted its refusals would make `bl_debt_outstanding` look better
//! than the body is.

use body_layer::measurement::{AxisKind, Measurement, Quantity, MAX_DEPS, MAX_DIM};
use body_layer::probe::{self, Declined};

// ---------------------------------------------------------------- tiny json field scanner
/// Return every `f64` that follows `"key":` inside `line`, in order.
/// Deliberately dumb: it does not build a tree, so it cannot be broken by a field it has never
/// seen. Numbers inside nested arrays are returned too, which is exactly what the array fields
/// below want.
fn nums_after(line: &str, key: &str) -> Vec<f64> {
    let pat = format!("\"{key}\"");
    let mut out = Vec::new();
    let mut idx = 0usize;
    while let Some(p) = line[idx..].find(&pat) {
        let start = idx + p + pat.len();
        let rest = &line[start..];
        let colon = match rest.find(':') {
            Some(c) => c + 1,
            None => break,
        };
        let tail = &rest[colon..];
        // take up to the matching close of a single scalar or one flat array
        let end = if tail.trim_start().starts_with('[') {
            tail.find(']').map(|e| e + 1).unwrap_or(tail.len())
        } else {
            tail.find(|c| c == ',' || c == '}').unwrap_or(tail.len())
        };
        for tok in tail[..end]
            .trim_matches(|c: char| c == '[' || c == ']' || c.is_whitespace())
            .split(',')
        {
            if let Ok(v) = tok.trim().parse::<f64>() {
                out.push(v);
            }
        }
        idx = start + colon + end;
    }
    out
}

/// Split a jsonl file's rows into the per-row `"rows"` objects, one string per row object.
fn row_objects(text: &str) -> Vec<String> {
    let mut out = Vec::new();
    for line in text.lines() {
        let Some(rs) = line.find("\"rows\"") else {
            continue;
        };
        let tail = &line[rs..];
        let mut depth = 0i32;
        let mut cur = String::new();
        let mut started = false;
        for ch in tail.chars() {
            match ch {
                '{' => {
                    depth += 1;
                    started = true;
                }
                '}' => depth -= 1,
                _ => {}
            }
            if started && depth > 0 {
                cur.push(ch);
            }
            if started && depth == 0 && !cur.is_empty() {
                cur.push('}');
                out.push(std::mem::take(&mut cur));
                started = false;
            }
        }
    }
    out
}

fn read(dir: &str, phase: &str) -> String {
    std::fs::read_to_string(format!("{dir}/{phase}.jsonl")).unwrap_or_default()
}

fn now_ns() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(1)
}

// ---------------------------------------------------------------- reporting
struct Out {
    lines: Vec<String>,
    ok: usize,
    refused: usize,
}

impl Out {
    fn keep(&mut self, name: &str, m: &Measurement) {
        self.ok += 1;
        let mut vals = String::new();
        for i in 0..m.dim.min(4) {
            vals.push_str(&format!("{:.5} ", m.value[i]));
        }
        println!(
            "  {name:<18} = [{}] ±[{:.5}] probed [{:.4},{:.4}] selftest={}",
            vals.trim_end(),
            m.uncertainty[0],
            m.valid_lo[0],
            m.valid_hi[0],
            m.selftest_passed
        );
        let mut q = format!(
            "\"{name}\": {{\"value\": [{}], \"uncertainty\": [{}], \"valid_lo\": [{}], \
             \"valid_hi\": [{}], \"measured_at\": {}, \"valid_for_s\": 0, \
             \"selftest_passed\": {}, \"dim\": {}",
            (0..m.dim)
                .map(|i| format!("{}", m.value[i]))
                .collect::<Vec<_>>()
                .join(", "),
            (0..m.dim)
                .map(|i| format!("{}", m.uncertainty[i]))
                .collect::<Vec<_>>()
                .join(", "),
            (0..m.dim)
                .map(|i| format!("{}", m.valid_lo[i]))
                .collect::<Vec<_>>()
                .join(", "),
            (0..m.dim)
                .map(|i| format!("{}", m.valid_hi[i]))
                .collect::<Vec<_>>()
                .join(", "),
            m.measured_at_ns / 1_000_000_000,
            m.selftest_passed,
            m.dim
        );
        q.push('}');
        self.lines.push(q);
    }

    fn refuse(&mut self, name: &str, why: &str) {
        self.refused += 1;
        println!("  {name:<18} REFUSED: {why}");
        self.lines.push(format!(
            "\"{name}\": {{\"refused\": \"{why}\", \"selftest_passed\": false}}"
        ));
    }
}

fn declined(e: Declined) -> &'static str {
    match e {
        Declined::NotEnoughSamples => "NotEnoughSamples: fewer samples than the estimator needs",
        Declined::NoResponse => "NoResponse: the commanded motion did not move the signal",
        Declined::Inconsistent => "Inconsistent: the samples imply mutually inconsistent answers",
        Declined::MissingDependency => "MissingDependency: something it is measured against is absent",
    }
}

fn main() {
    let dir = std::env::args().nth(1).expect("arg 1 = raw sample dir");
    let out_path = std::env::args()
        .nth(2)
        .unwrap_or_else(|| format!("{dir}/../calibration.json"));
    let t = now_ns();
    let mut o = Out {
        lines: Vec::new(),
        ok: 0,
        refused: 0,
    };
    println!("body layer :: calibrating from {dir}");

    // ---- latency -------------------------------------------------------------------------
    {
        let rows = row_objects(&read(&dir, "latency"));
        let mut first: Option<u32> = None;
        let n = rows.len() as u32;
        // "moved" is a displacement clearly above the resting jitter of the same trace.
        let disps: Vec<f64> = rows
            .iter()
            .filter_map(|r| nums_after(r, "disp").first().copied())
            .collect();
        if !disps.is_empty() {
            let peak = disps.iter().cloned().fold(0.0f64, f64::max);
            let thresh = (peak * 0.05).max(1e-5);
            for (k, d) in disps.iter().enumerate() {
                if *d > thresh {
                    first = Some(k as u32);
                    break;
                }
            }
        }
        match probe::latency(first, n, t) {
            Ok(m) => o.keep("latency", &m),
            Err(e) => o.refuse("latency", declined(e)),
        }
    }

    // ---- step_delivery -------------------------------------------------------------------
    {
        let rows = row_objects(&read(&dir, "step_delivery"));
        let steps: Vec<(f64, f64)> = rows
            .iter()
            .filter_map(|r| {
                Some((
                    *nums_after(r, "cmd").first()?,
                    *nums_after(r, "ach").first()?,
                ))
            })
            .collect();
        match probe::step_delivery(&steps, t) {
            Ok(m) => o.keep("step_delivery", &m),
            Err(e) => o.refuse("step_delivery", declined(e)),
        }
    }

    // ---- backlash ------------------------------------------------------------------------
    {
        let rows = row_objects(&read(&dir, "backlash"));
        let steps: Vec<(f64, f64)> = rows
            .iter()
            .filter_map(|r| {
                Some((
                    *nums_after(r, "cmd").first()?,
                    *nums_after(r, "obs").first()?,
                ))
            })
            .collect();
        match probe::backlash(&steps, t) {
            Ok(m) => o.keep("backlash", &m),
            Err(e) => o.refuse("backlash", declined(e)),
        }
    }

    // ---- image_jacobian ------------------------------------------------------------------
    // One Sample per excursion: the commanded delta per world axis, and where the TOP-RANKED
    // moving-pixel candidate landed. The rank comes from the collector; the DECISION about
    // whether that candidate can be trusted is `hand_pixel`'s, below, and it may refuse.
    let mut jac_ok = false;
    {
        let rows = row_objects(&read(&dir, "image_jacobian"));
        let mut samples: Vec<probe::Sample> = Vec::new();
        for r in &rows {
            let ee0 = nums_after(r, "ee0");
            let ee1 = nums_after(r, "ee1");
            let u = nums_after(r, "u");
            let v = nums_after(r, "v");
            if ee0.len() < 3 || ee1.len() < 3 || u.is_empty() || v.is_empty() {
                continue;
            }
            let mut cmd = [0.0f64; MAX_DIM];
            for i in 0..3 {
                cmd[i] = ee1[i] - ee0[i];
            }
            samples.push(probe::Sample {
                cmd,
                n: 3,
                uv: [u[0], v[0]],
                at_ns: t,
            });
        }
        match probe::image_jacobian(&samples, 3, t, 1e-3) {
            Ok(m) => {
                jac_ok = true;
                o.keep("image_jacobian", &m);
            }
            Err(e) => o.refuse("image_jacobian", declined(e)),
        }
    }

    // ---- tool_offset + tool_axis_column ---------------------------------------------------
    // The wrist is spun about each column of R. About the TOOL axis the working point barely
    // moves; about the other two it sweeps an arc whose radius IS the offset. So the same motion
    // yields both `L3_GRIPPER_BIAS` and `L3_TOOL_COL`, and neither is declared anywhere.
    {
        let rows = row_objects(&read(&dir, "tool_offset"));
        let mut per_col: [Vec<(f64, f64, f64)>; 3] = [Vec::new(), Vec::new(), Vec::new()];
        for r in &rows {
            let col = nums_after(r, "col");
            let ang = nums_after(r, "ang");
            let u = nums_after(r, "u");
            let v = nums_after(r, "v");
            if col.is_empty() || ang.is_empty() || u.is_empty() || v.is_empty() {
                continue;
            }
            let c = col[0] as usize;
            if c < 3 {
                per_col[c].push((ang[0], u[0], v[0]));
            }
        }
        // spread of the tracked point about each column, in image units
        let mut spread = [f64::NAN; 3];
        for c in 0..3 {
            if per_col[c].len() < 3 {
                continue;
            }
            let (mut mu, mut mv) = (0.0, 0.0);
            for (_, u, v) in &per_col[c] {
                mu += u;
                mv += v;
            }
            let n = per_col[c].len() as f64;
            mu /= n;
            mv /= n;
            let mut s = 0.0;
            for (_, u, v) in &per_col[c] {
                s += ((u - mu).powi(2) + (v - mv).powi(2)).sqrt();
            }
            spread[c] = s / n;
        }
        println!(
            "  tool axis search   spread per column = [{:.5}, {:.5}, {:.5}] (image units)",
            spread[0], spread[1], spread[2]
        );
        let finite: Vec<usize> = (0..3).filter(|c| spread[*c].is_finite()).collect();
        if finite.len() < 2 {
            o.refuse(
                "tool_axis_column",
                "fewer than two columns produced a tracked arc",
            );
            o.refuse("tool_offset", "tool axis column unresolved");
        } else {
            let mut order = finite.clone();
            order.sort_by(|a, b| spread[*a].partial_cmp(&spread[*b]).unwrap());
            let best = order[0];
            let second = order[1];
            // 🔴 the same separation rule as `hand.rs`: if the two smallest are not separable,
            // REFUSE instead of returning the better of them. A selection rule that cannot tell
            // its top two apart does not report an error, it quietly picks wrong -- that is the
            // elbow-for-fingertip failure, restated for columns.
            let sep = if spread[best] > 0.0 {
                spread[second] / spread[best]
            } else {
                f64::INFINITY
            };
            if sep < 1.5 {
                o.refuse(
                    "tool_axis_column",
                    "top two columns within 1.5x of each other; not separable",
                );
                o.refuse("tool_offset", "tool axis column unresolved");
            } else {
                // ⚠️ This used to read "the column index has no enum slot of its own, on
                // purpose" and rode on ToolOffset's slot.  It has one as of 2026-08-11
                // (Quantity::ToolAxisColumn), added when the Python store turned out to have been
                // keeping it as a twelfth quantity all along -- the slot is no longer unprobed.
                let col_m = Measurement {
                    quantity: Quantity::ToolAxisColumn,
                    // A column index is a LABEL, not a length: as an interval, [0,2] admits
                    // "spin about column 0.5".
                    axis_kind: {
                        let mut k = [AxisKind::Interval; MAX_DIM];
                        k[0] = AxisKind::Categorical;
                        k
                    },
                    dim: 1,
                    value: {
                        let mut v = [0.0; MAX_DIM];
                        v[0] = best as f64;
                        v
                    },
                    uncertainty: [0.0; MAX_DIM],
                    valid_lo: [0.0; MAX_DIM],
                    valid_hi: {
                        let mut v = [1.0; MAX_DIM];
                        v[0] = 2.0;
                        v
                    },
                    measured_at_ns: t,
                    valid_for_ns: 0,
                    deps: [None; MAX_DEPS],
                    epoch: 1,
                    selftest_passed: true,
                    prev_epoch: 0,
                };
                o.keep("tool_axis_column", &col_m);

                // radius of the arc about a NON-tool column, converted to metres by the Jacobian
                // scale when there is one. Without a Jacobian this is image units and MUST say so.
                let arc = &per_col[second];
                let scale_known = jac_ok;
                match probe::tool_offset(arc, 1.0, 0.0, t, if jac_ok { 1 } else { 0 }) {
                    Ok(mm) => {
                        if scale_known {
                            o.keep("tool_offset", &mm);
                        } else {
                            o.refuse(
                                "tool_offset",
                                "arc radius measured, but in IMAGE UNITS: no image Jacobian, so \
                                 there is no metric ruler. A number in the wrong unit is worse \
                                 than none",
                            );
                        }
                    }
                    Err(e) => o.refuse("tool_offset", declined(e)),
                }
            }
        }
    }

    // ---- gripper_span --------------------------------------------------------------------
    {
        let rows = row_objects(&read(&dir, "gripper_span"));
        let samples: Vec<(f64, f64)> = rows
            .iter()
            .filter_map(|r| {
                let c = nums_after(r, "cmd");
                let u = nums_after(r, "u");
                let v = nums_after(r, "v");
                if c.is_empty() || u.len() < 2 || v.len() < 2 {
                    return None;
                }
                // separation between the two strongest moving candidates = the jaws
                let sep = ((u[0] - u[1]).powi(2) + (v[0] - v[1]).powi(2)).sqrt();
                Some((c[0], sep))
            })
            .collect();
        match probe::gripper_span(&samples, 1.0, 0.0, t, if jac_ok { 1 } else { 0 }) {
            Ok(m) => {
                if jac_ok {
                    o.keep("gripper_span", &m)
                } else {
                    o.refuse(
                        "gripper_span",
                        "jaw separation measured in IMAGE UNITS with no image Jacobian to convert \
                         it; a span in pixels is not a span in metres",
                    )
                }
            }
            Err(e) => o.refuse("gripper_span", declined(e)),
        }
    }

    // ---- reach ---------------------------------------------------------------------------
    {
        let rows = row_objects(&read(&dir, "reach"));
        let samples: Vec<(f64, bool)> = rows
            .iter()
            .filter_map(|r| {
                // 🔴 `r_base` (radius from the MEASURED base), never `r_cmd` (the collector's
                // sampling parameter).  Reading `r_cmd` produced a well-formed band 0.30 m wider
                // than the arm's own span -- valid, precise, and in the wrong frame.
                let rad = nums_after(r, "r_base");
                if rad.is_empty() {
                    return None;
                }
                let attained = r.contains("\"attained\": true") || r.contains("\"attained\":true");
                Some((rad[0], attained))
            })
            .collect();
        match probe::reach(&samples, t) {
            Ok(m) => o.keep("reach", &m),
            Err(e) => o.refuse("reach", declined(e)),
        }
    }

    // ---- quantities this observation contract cannot support -------------------------------
    for (name, why) in [
        ("arm_weight", "no joint torque channel on this body's bus"),
        ("contact_threshold", "no force channel on this body's bus"),
        ("self_occlusion", "no validated body-silhouette segmenter here"),
        ("hand_pixel", "needs a tracker fed frame by frame, not a batch of excursions"),
    ] {
        o.refuse(name, why);
    }

    let body = std::fs::read_to_string(format!("{dir}/fingerprint.jsonl")).unwrap_or_default();
    let fp = body
        .lines()
        .last()
        .and_then(|l| l.split("\"fingerprint\": \"").nth(1))
        .and_then(|s| s.split('"').next())
        .unwrap_or("unknown")
        .to_string();

    let json = format!(
        "{{\n  \"fingerprint\": \"{fp}\",\n  \"produced_by\": \"calibrate_from_probes\",\n  \
         \"raw_dir\": \"{dir}\",\n  \"n_measured\": {},\n  \"n_refused\": {},\n  \
         \"quantities\": {{\n    {}\n  }}\n}}\n",
        o.ok,
        o.refused,
        o.lines.join(",\n    ")
    );
    std::fs::write(&out_path, json).expect("write calibration");
    println!(
        "\n  fingerprint = {fp}\n  measured {} / refused {}  ->  {out_path}",
        o.ok, o.refused
    );
    println!(
        "  🔴 refusals are the honest half: a body layer that answered all eleven on a bus that \n\
         \x20    exposes no torque and no force would be inventing numbers."
    );
}

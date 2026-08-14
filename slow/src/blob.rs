//! **Three frames and the same command twice → where the part that responded actually IS.** The
//! evidence [`crate::hand::HandTracker`] consumes, and the reason it had none.
//!
//! # Why this file exists
//!
//! `hand.rs` is 357 lines of estimator with **no caller**: it takes `&[Candidate]` and nobody
//! produced one, so `hand_pixel` stands REFUSED in the store — *"needs a tracker fed frame by
//! frame, not a batch of excursions"*. That refusal is load-bearing: [`crate::execute`] asks for
//! `HandPixel` at `ask.needs[1]`, so **the driver's main entry has never once been admitted**, and
//! every motion this project has run went around it. This file is the missing half.
//!
//! # Why THREE frames — the two-frame version was refuted by its own unit test
//!
//! The obvious reader differences two frames and calls the changed pixels the hand. It does not
//! work, and the failure is not subtle once looked at: a part that moves further than its own width
//! leaves **two disconnected regions** — where it was, and where it went — and nothing in a single
//! difference image says which is which. The first test written here demanded one region and got
//! two. Feeding that to the estimator is worse than feeding it nothing: two lobes of one finger are
//! near-identical in size and gain, so the separability rule abstains every step, and an estimator
//! that abstains every step reads exactly like one with no camera.
//!
//! Commanding the **same** step twice and keeping the pixels that changed **both** times fixes it
//! by construction. Call the part's footprint `A`, `B`, `C` at the three frames: the first
//! difference covers `A ∪ B`, the second covers `B ∪ C`, and their intersection is `B` plus
//! whatever `A` and `C` share — which is nothing, because two equal steps in one direction put
//! them `2·cmd` apart. **The intersection is where the part is NOW**, not where it has been, and it
//! is obtained with commands only: no template, no appearance model, no marker.
//!
//! Displacement then comes free and separately: the two difference regions have centroids one step
//! apart, so `gain` is a real pixels-per-command-unit and not a ranking score.
//!
//! # The excitation, and why it is the jaws and not the arm
//!
//! `hand.rs` records the trap in full: the selector *"whichever rigid thing responds most to my
//! command is me"* was derived when the competitors were the hand and its shadow; on a rig where
//! the competitors became **different links of the same arm**, the elbow (nearer the camera) won,
//! and the loop aimed the elbow at the mark with a self-reported error of 0.04–9.3 px while the
//! true error was **167 px**.
//!
//! Opening and closing the jaws removes that competition at the source rather than out-ranking it:
//! **the elbow's gain under a jaw command is exactly zero, because the elbow does not move.** The
//! separation test then compares a finger against a background that did not respond at all,
//! instead of against another link that responded 1.11× less.
//!
//! What it does NOT do by itself is give one region. Two fingers closing are **two** regions of
//! near-identical size and speed — precisely the case the estimator refuses. Ranking them is both
//! hopeless and beside the point, because neither finger is the answer. So the pair is not
//! resolved, it is **recognised**: two things that travel in opposite directions cancel, nothing
//! else in a static scene does that, and the point between them is what has to be put on the
//! object. See [`fold_opposed`] — and the disconfirming control beside it, where two things
//! sliding the same way are left unfolded and the estimator abstains as it should.
//!
//! # The threshold is measured, never typed
//!
//! "Which pixels changed" needs a floor, and a typed-in floor is the hand-filled constant this
//! layer exists to abolish. So the caller supplies a **null pair** — two frames with *no* command
//! between them — and the floor is the largest change that pair shows. That is this camera, this
//! scene, this exposure, answering *"how much do I change when I do nothing?"* in its own units.
//! In a noiseless renderer it comes out 0 and every command-driven pixel survives, which is right
//! there too.
//!
//! # What this file does NOT establish
//!
//! `rigidity` is a **fill ratio** — how well one compact region explains the pixels. It separates
//! "one thing" from "specks over the frame". It does not verify a rigid transform, and no single
//! frame triple can. What catches a wrong pick is downstream and mechanical: the displacement must
//! agree with the already measured `image_jacobian` (σ = 0.27 px), and disagreement is a refusal.

use crate::hand::Candidate;

/// Why a frame set could not be read at all. Distinct from "read it and found nothing", which is
/// an empty candidate list and is a different fact.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Bad {
    /// The frames are not all `w * h`, or `w`/`h` is degenerate.
    ShapeMismatch,
    /// The command between frames was zero, not finite, or negative. Gain is per command unit, so
    /// there is no gain to report — and reporting one anyway is a divide-by-nothing wearing a
    /// number's clothes.
    NoCommand,
    /// More separate regions than this reader will enumerate. Truncating to the biggest few is the
    /// silent maximum-taking `hand.rs` refuses, so this refuses instead.
    TooManyRegions,
}

/// Most regions enumerated before this refuses. Reached only when the scene is not static, which
/// is a condition to report, not to trim.
pub const MAX_REGIONS: usize = 64;

/// What one excitation yielded, with the numbers that let a reader audit it.
#[derive(Clone, Debug)]
pub struct Reading {
    /// The regions, largest first. May be empty — that is "nothing responded twice", not a failure.
    pub cands: heapless::Vec,
    /// The measured floor: the largest change the null pair showed. `0` on a noiseless renderer.
    pub floor: u8,
    /// Pixels that responded to **both** steps, before any size filter. The denominator for "how
    /// much of this frame is moving", which is how a camera knock shows up as obviously-not-a-hand.
    pub moved_px: u32,
    /// How many opposed pairs were folded into one point (see [`fold_opposed`]). A jaw excitation
    /// on a working gripper reads **1**; `0` with two candidates present means the two halves did
    /// not cancel, so whatever moved was not a pair of jaws.
    pub pairs: u32,
    /// 🔴 **合并的那一对,两瓣是沿哪个方向分开的**(归一化画面单位,从一瓣指向另一瓣)。
    ///
    /// 那就是**钳口张开的方向**,而它决定"要从哪一侧把物体送进两指之间"。合并那一步本来就
    /// 知道这两瓣是谁,却只留下了中点 —— **方向被扔掉,于是下游只能猜**。实测代价
    /// (2026-08-14):四种接近方式(上方压 / 沿 y 横进 / 沿 x 横进 / 工具轴 ±10 cm)全部失败,
    /// 而它们的区别**只在这个方向上**。没有配对时是 `(0,0)`。
    pub pair_dir: (f64, f64),
    /// 🔴 两步**各自**有多少像素越过地板。诊断用,而它区分的两件事要改的东西完全不同:
    /// 一个是 0 ⇒ 那一步**根本看不见**(激励太小 / 被自己挡住 / 时序错位);两个都大而
    /// `moved_px` 小 ⇒ 两步都看得见但**不重叠**(部件走得比自己还宽,或者三帧假设不成立)。
    ///
    /// 加这两个数,是因为"双响像素 0"这一个读数**同时**兼容这两种病,而它们南辕北辙。
    pub m1_px: u32,
    /// 见 [`Reading::m1_px`]。
    pub m2_px: u32,
}

/// Read one excitation.
///
/// `null_a`/`null_b`: two frames with **no** command between them — the noise floor, measured.
/// `f0`/`f1`/`f2`: three frames with the **same** command of size `cmd` applied between each
/// consecutive pair. All frames single-channel, row-major, `w * h`.
///
/// `min_pixels` is the caller's observability floor (see [`crate::hand::Config::min_pixels`]).
pub fn candidates(
    null_a: &[u8],
    null_b: &[u8],
    f0: &[u8],
    f1: &[u8],
    f2: &[u8],
    w: usize,
    h: usize,
    cmd: f64,
    min_pixels: u32,
) -> Result<Reading, Bad> {
    let n = w.checked_mul(h).ok_or(Bad::ShapeMismatch)?;
    if w < 2 || h < 2 {
        return Err(Bad::ShapeMismatch);
    }
    for f in [null_a, null_b, f0, f1, f2] {
        if f.len() != n {
            return Err(Bad::ShapeMismatch);
        }
    }
    if !cmd.is_finite() || cmd <= 0.0 {
        return Err(Bad::NoCommand);
    }

    // 🔴 The floor, measured. Deliberately not a parameter: a caller that could pass it would
    // eventually pass a number somebody liked the look of.
    let mut floor: u8 = 0;
    for i in 0..n {
        let d = null_a[i].abs_diff(null_b[i]);
        if d > floor {
            floor = d;
        }
    }

    let m1: Vec<bool> = (0..n).map(|i| f0[i].abs_diff(f1[i]) > floor).collect();
    let m2: Vec<bool> = (0..n).map(|i| f1[i].abs_diff(f2[i]) > floor).collect();
    // 🔴 THE WHOLE IDEA IN ONE LINE: responded to the first step AND to the second ⇒ this is where
    // the part is at `f1`. Responded to only one ⇒ it is a place the part has been, or is going.
    let here: Vec<bool> = (0..n).map(|i| m1[i] && m2[i]).collect();
    let m1_px = m1.iter().filter(|x| **x).count() as u32;
    let m2_px = m2.iter().filter(|x| **x).count() as u32;

    let (_lab3, mom3) = label(&here, w, h);
    let moved_px = mom3.iter().map(|m| m.cnt).sum();

    // 🔴 HOW FAR EACH PART TRAVELLED, and why it is not read off the regions of one difference.
    //
    // The first draft took the difference-region containing the part and compared its centroid
    // across the two steps. Its own test refuted it: when the step exceeds the part's width the
    // two lobes are disconnected, the region "containing the part" is the SAME lobe both times,
    // and the displacement comes out **exactly zero** for a part that plainly moved.
    //
    // What is true regardless of connectivity: the pixels that changed in step one are centred
    // half a step behind the part, those of step two half a step ahead, so the two centroids are
    // one step apart. Splitting those pixels between parts needs no threshold — each goes to the
    // nearest part centre, and the part centres are the intersection regions, which are already
    // known. One part: everything goes to it, and the answer is exact.
    let centres: Vec<(f64, f64)> = mom3.iter().map(Moments::centroid).collect();
    let mut a1 = vec![Moments::default(); mom3.len()];
    let mut a2 = vec![Moments::default(); mom3.len()];
    if !centres.is_empty() {
        for i in 0..n {
            if !m1[i] && !m2[i] {
                continue;
            }
            let (x, y) = ((i % w) as f64, (i / w) as f64);
            let mut best = 0usize;
            let mut bd = f64::INFINITY;
            for (k, &(cx, cy)) in centres.iter().enumerate() {
                let d = (x - cx) * (x - cx) + (y - cy) * (y - cy);
                if d < bd {
                    bd = d;
                    best = k;
                }
            }
            if m1[i] {
                a1[best].add(x, y);
            }
            if m2[i] {
                a2[best].add(x, y);
            }
        }
    }

    let (fw, fh) = (w as f64, h as f64);
    let mut raw: Vec<(Candidate, f64, f64)> = Vec::new();
    for (k, m) in mom3.iter().enumerate() {
        if m.cnt < min_pixels {
            continue;
        }
        let (mx, my) = m.centroid();
        let (vxx, vyy, vxy) = m.covariance();

        // Principal axes, closed form for a symmetric 2x2.
        let tr = vxx + vyy;
        let det = (vxx * vyy - vxy * vxy).max(0.0);
        let disc = (0.25 * tr * tr - det).max(0.0).sqrt();
        let (sa, sb) = ((0.5 * tr + disc).max(0.0).sqrt(), (0.5 * tr - disc).max(0.0).sqrt());
        // A uniform region of semi-axis `a` has variance `a^2/4`, so `a = 2 sigma`; the ellipse
        // with semi-axes `2 sigma` is the region's own equivalent ellipse, and a compact region
        // fills it — 1.00 for an ellipse, ~0.95 for a rectangle, far less for scattered specks.
        let ellipse = core::f64::consts::PI * (2.0 * sa) * (2.0 * sb);
        let rigidity = if ellipse > 0.0 { (f64::from(m.cnt) / ellipse).min(1.0) } else { 0.0 };

        // 🔴 A REAL DISPLACEMENT, not a ranking score: one command step of travel, per command unit.
        // The vector is kept as well as its length — a parallel gripper is recognised by the fact
        // that its two fingers travel in OPPOSITE directions, and a length cannot say that.
        let (du, dv) = if a1[k].cnt > 0 && a2[k].cnt > 0 {
            let (ax, ay) = a1[k].centroid();
            let (bx, by) = a2[k].centroid();
            ((bx - ax) / fw, (by - ay) / fh)
        } else {
            (0.0, 0.0)
        };
        let gain = du.hypot(dv) / cmd;

        let cand = Candidate {
            u: mx / fw,
            v: my / fh,
            gain,
            rigidity,
            pixels: m.cnt,
            // 🔴 MEASURED off the region, never manufactured: the isotropic 1σ of the pixels that
            // are in it. `hand.rs` records what happens when this field is invented — a draft that
            // derived it from the candidate count reported a third of the frame.
            spread: (0.5 * (vxx + vyy)).sqrt() / fw,
        };
        if raw.len() >= MAX_REGIONS {
            return Err(Bad::TooManyRegions);
        }
        raw.push((cand, du, dv));
    }

    let (folded, pairs, pair_dir) = fold_opposed(raw);
    let mut cands: heapless::Vec = heapless::Vec::new();
    for c in folded {
        if !cands.push(c) {
            return Err(Bad::TooManyRegions);
        }
    }
    cands.sort_desc_by_pixels();
    Ok(Reading { cands, floor, moved_px, pairs, pair_dir, m1_px, m2_px })
}

/// 🔴 **A GRIPPER IS TWO THINGS THAT TRAVEL TOWARDS EACH OTHER, AND ITS POINT IS BETWEEN THEM.**
///
/// The estimator downstream refuses when two candidates cannot be told apart — the anti-elbow rule,
/// and it is right. But a parallel jaw closing presents exactly that: two regions of near-identical
/// size and speed. Ranking them is hopeless *and beside the point*, because neither finger is the
/// answer; the answer is the point between them, which is where the object has to end up.
///
/// So the pair is not resolved, it is **recognised**, by a signature nothing else in a static scene
/// produces: the two travel in opposite directions, so their displacements cancel. The tolerance on
/// "cancel" is not typed in — it is the two regions' own measured spreads, which is the accuracy
/// with which their centres are known in the first place.
///
/// Two objects sliding the same way on a belt do **not** cancel, and are left as two candidates for
/// the estimator to abstain on. That is the intended outcome: this recognises grippers, it does not
/// merge whatever happens to be nearby.
fn fold_opposed(raw: Vec<(Candidate, f64, f64)>) -> (Vec<Candidate>, u32, (f64, f64)) {
    let n = raw.len();
    let mut used = vec![false; n];
    let mut out: Vec<Candidate> = Vec::new();
    let mut pairs = 0u32;
    let mut pair_dir = (0.0f64, 0.0f64);

    for i in 0..n {
        if used[i] {
            continue;
        }
        let (ci, dui, dvi) = raw[i];
        let mut best: Option<(usize, f64)> = None;
        for (j, item) in raw.iter().enumerate().skip(i + 1) {
            if used[j] {
                continue;
            }
            let (cj, duj, dvj) = *item;
            // Opposite directions, not merely different ones. Two regions that did not move have a
            // dot product of zero and are excluded here rather than by a size threshold.
            if dui * duj + dvi * dvj >= 0.0 {
                continue;
            }
            let residual = (dui + duj).hypot(dvi + dvj);
            let tol = ci.spread + cj.spread;
            if residual <= tol {
                if best.map(|(_, r)| residual < r).unwrap_or(true) {
                    best = Some((j, residual));
                }
            }
        }
        match best {
            None => out.push(ci),
            Some((j, _)) => {
                let (cj, _, _) = raw[j];
                used[i] = true;
                used[j] = true;
                pairs += 1;
                if pair_dir == (0.0, 0.0) {
                    pair_dir = (cj.u - ci.u, cj.v - ci.v);
                }
                out.push(Candidate {
                    u: 0.5 * (ci.u + cj.u),
                    v: 0.5 * (ci.v + cj.v),
                    // Both fingers travel at the same speed; report it once, not twice.
                    gain: 0.5 * (ci.gain + cj.gain),
                    // The weaker half governs: a pair is only as much "one thing" as its worse side.
                    rigidity: ci.rigidity.min(cj.rigidity),
                    pixels: ci.pixels + cj.pixels,
                    // The midpoint of two symmetric lobes IS better localised than either — by
                    // 1/sqrt(2) if their errors are independent, which is not established. So the
                    // worse of the two is carried, and no improvement is claimed.
                    spread: ci.spread.max(cj.spread),
                });
            }
        }
    }
    (out, pairs, pair_dir)
}

/// 🔴 THE CROSS-CHECK, and the reason this pipeline is not another self-reported number.
///
/// `image_jacobian` already says how far a pixel travels per command unit, measured, with its own
/// 1σ. If the region this reader picked really is the hand, its displacement must agree. If it does
/// not, the reader found *something*, and something is not evidence about where the hand is.
///
/// Returns `true` when the candidate may be believed.
pub fn agrees_with_jacobian(
    c: &Candidate,
    jac_px_per_cmd: f64,
    jac_sigma: f64,
    sigmas: f64,
) -> bool {
    if !(jac_px_per_cmd.is_finite() && jac_sigma.is_finite() && jac_sigma > 0.0) {
        return false;
    }
    if !(sigmas.is_finite() && sigmas > 0.0) || !c.gain.is_finite() {
        return false;
    }
    (c.gain - jac_px_per_cmd).abs() <= sigmas * jac_sigma
}

// ---------------------------------------------------------------- regions

/// Running moments of one region, accumulated as it is discovered.
#[derive(Copy, Clone, Debug, Default)]
struct Moments {
    cnt: u32,
    sx: f64,
    sy: f64,
    sxx: f64,
    sxy: f64,
    syy: f64,
}

impl Moments {
    fn add(&mut self, x: f64, y: f64) {
        self.cnt += 1;
        self.sx += x;
        self.sy += y;
        self.sxx += x * x;
        self.sxy += x * y;
        self.syy += y * y;
    }
    fn centroid(&self) -> (f64, f64) {
        let c = f64::from(self.cnt).max(1.0);
        (self.sx / c, self.sy / c)
    }
    fn covariance(&self) -> (f64, f64, f64) {
        let c = f64::from(self.cnt).max(1.0);
        let (mx, my) = self.centroid();
        (
            (self.sxx / c - mx * mx).max(0.0),
            (self.syy / c - my * my).max(0.0),
            self.sxy / c - mx * my,
        )
    }
}

/// 4-connected regions. Iterative because `panic = "abort"` — a recursive flood fill on a frame
/// where most pixels moved is a stack overflow, and a stack overflow in the body layer is a fault.
fn label(mask: &[bool], w: usize, h: usize) -> (Vec<Option<usize>>, Vec<Moments>) {
    let n = w * h;
    let mut lab: Vec<Option<usize>> = vec![None; n];
    let mut moms: Vec<Moments> = Vec::new();
    let mut stack: Vec<usize> = Vec::new();

    for start in 0..n {
        if !mask[start] || lab[start].is_some() {
            continue;
        }
        let k = moms.len();
        moms.push(Moments::default());
        lab[start] = Some(k);
        stack.push(start);
        while let Some(p) = stack.pop() {
            let (px, py) = (p % w, p / w);
            moms[k].add(px as f64, py as f64);
            let visit = |q: usize, lab: &mut Vec<Option<usize>>, st: &mut Vec<usize>| {
                if mask[q] && lab[q].is_none() {
                    lab[q] = Some(k);
                    st.push(q);
                }
            };
            if px > 0 {
                visit(p - 1, &mut lab, &mut stack);
            }
            if px + 1 < w {
                visit(p + 1, &mut lab, &mut stack);
            }
            if py > 0 {
                visit(p - w, &mut lab, &mut stack);
            }
            if py + 1 < h {
                visit(p + w, &mut lab, &mut stack);
            }
        }
    }
    (lab, moms)
}

// ---------------------------------------------------------------- fixed-capacity vec

/// Same shape as the one in [`crate::hand`], kept local so neither file reaches into the other.
pub mod heapless {
    use crate::hand::Candidate;

    /// A vector that cannot grow past [`super::MAX_REGIONS`]; pushing past it reports failure.
    #[derive(Clone, Debug)]
    pub struct Vec {
        buf: [Option<Candidate>; super::MAX_REGIONS],
        len: usize,
    }

    impl Default for Vec {
        fn default() -> Self {
            Self::new()
        }
    }

    impl Vec {
        /// Empty.
        pub fn new() -> Self {
            Vec { buf: [None; super::MAX_REGIONS], len: 0 }
        }
        /// Append; `false` when full.
        pub fn push(&mut self, c: Candidate) -> bool {
            if self.len >= super::MAX_REGIONS {
                return false;
            }
            self.buf[self.len] = Some(c);
            self.len += 1;
            true
        }
        /// How many.
        pub fn len(&self) -> usize {
            self.len
        }
        /// Whether none.
        pub fn is_empty(&self) -> bool {
            self.len == 0
        }
        /// Element, if in range.
        pub fn get(&self, i: usize) -> Option<&Candidate> {
            if i < self.len {
                self.buf[i].as_ref()
            } else {
                None
            }
        }
        /// Biggest region first. Size, not gain — ranking by gain is the estimator's job, and doing
        /// it here too would hide which layer chose.
        pub fn sort_desc_by_pixels(&mut self) {
            self.buf[..self.len].sort_by(|a, b| match (a, b) {
                (Some(x), Some(y)) => y.pixels.cmp(&x.pixels),
                _ => core::cmp::Ordering::Equal,
            });
        }
        /// A dense copy of the live entries, for handing to [`crate::hand::HandTracker::observe`].
        pub fn dense(&self) -> [Candidate; super::MAX_REGIONS] {
            let mut out = [Candidate {
                u: 0.0,
                v: 0.0,
                gain: 0.0,
                rigidity: 0.0,
                pixels: 0,
                spread: 0.0,
            }; super::MAX_REGIONS];
            for i in 0..self.len {
                if let Some(c) = self.buf[i] {
                    out[i] = c;
                }
            }
            out
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const W: usize = 64;
    const H: usize = 48;

    fn blank() -> Vec<u8> {
        vec![30u8; W * H]
    }

    fn rect(f: &mut [u8], x0: usize, y0: usize, wd: usize, ht: usize, val: u8) {
        for y in y0..(y0 + ht).min(H) {
            for x in x0..(x0 + wd).min(W) {
                f[y * W + x] = val;
            }
        }
    }

    /// Three frames of one part stepping twice by the same amount.
    fn stepping(x0: usize, y0: usize, step: usize) -> (Vec<u8>, Vec<u8>, Vec<u8>) {
        let (mut a, mut b, mut c) = (blank(), blank(), blank());
        rect(&mut a, x0, y0, 8, 8, 200);
        rect(&mut b, x0 + step, y0, 8, 8, 200);
        rect(&mut c, x0 + 2 * step, y0, 8, 8, 200);
        (a, b, c)
    }

    /// 🔴 THE REFUTATION THAT SHAPED THIS FILE, KEPT AS A TEST.
    ///
    /// A part that steps further than its own width leaves two disconnected regions in a single
    /// difference — one of them is where it USED to be. The two-frame reader returned both and had
    /// no way to say which was the hand; here the intersection of two steps lands on the middle
    /// frame's position and there is exactly one region.
    #[test]
    fn a_part_that_steps_twice_is_located_where_it_is_now_not_where_it_was() {
        let (na, nb) = (blank(), blank());
        let (f0, f1, f2) = stepping(10, 20, 10); // 10 px steps, 8 px wide: fully disjoint
        let r = candidates(&na, &nb, &f0, &f1, &f2, W, H, 0.02, 20).expect("a static scene reads");
        assert_eq!(r.floor, 0, "a noiseless pair has no floor to clear");
        assert_eq!(r.cands.len(), 1, "one part, one region — this is the whole point of 3 frames");
        let c = r.cands.get(0).unwrap();
        // The part is at x = 20..28 in the middle frame: centre 24 of 64.
        assert!((c.u - 24.0 / 64.0).abs() < 0.02, "u = {} should be the CURRENT position", c.u);
        assert!((c.v - 24.0 / 48.0).abs() < 0.03, "v = {}", c.v);
        // 10 px of travel over 64 px of width, per 0.02 of command.
        assert!((c.gain - (10.0 / 64.0) / 0.02).abs() < 1.0, "gain = {} is a displacement", c.gain);
        assert!(c.spread > 0.0, "spread must be measured, not zero");
        assert!(c.rigidity > 0.8, "a square is one compact thing: {}", c.rigidity);
    }

    /// 🔴 THE FLOOR IS MEASURED. A camera that flickers by 12 counts while nothing is commanded
    /// must not produce a hand; the same frames with a quiet null pair DO. So it is the null
    /// measurement doing the work, not a size filter hiding the problem.
    #[test]
    fn noise_the_null_pair_shows_cannot_become_a_candidate() {
        let na = blank();
        let mut nb = blank();
        for (i, p) in nb.iter_mut().enumerate() {
            *p = if i % 3 == 0 { 42 } else { 30 }; // 12 counts of flicker, everywhere
        }
        // a 10-count part: real, but under this camera's own noise
        let (mut f0, mut f1, mut f2) = (blank(), blank(), blank());
        rect(&mut f0, 10, 10, 12, 12, 40);
        rect(&mut f1, 20, 10, 12, 12, 40);
        rect(&mut f2, 30, 10, 12, 12, 40);
        let r = candidates(&na, &nb, &f0, &f1, &f2, W, H, 0.02, 20).unwrap();
        assert_eq!(r.floor, 12, "the null pair measured its own floor");
        assert!(r.cands.is_empty(), "10 counts is under this camera's own noise");

        let quiet = blank();
        let r2 = candidates(&quiet, &quiet, &f0, &f1, &f2, W, H, 0.02, 20).unwrap();
        assert_eq!(r2.cands.len(), 1, "same frames, honest camera: it is a region");
    }

    /// 🔴 A CLOSING GRIPPER IS ONE POINT, NOT TWO CANDIDATES.
    ///
    /// Two fingers travelling towards each other are exactly the case `hand.rs` refuses to resolve
    /// — near-identical size and speed. Recognising the pair by its cancelling displacement turns
    /// what would be a permanent `NotSeparable` into the one thing actually wanted: the point
    /// between the jaws.
    #[test]
    fn two_jaws_closing_fold_into_the_point_between_them() {
        let (na, nb) = (blank(), blank());
        let (mut f0, mut f1, mut f2) = (blank(), blank(), blank());
        // left finger walks right, right finger walks left, by the same amount
        for (fr, k) in [(&mut f0, 0i32), (&mut f1, 1), (&mut f2, 2)] {
            rect(fr, (8 + 5 * k) as usize, 20, 4, 6, 200);
            rect(fr, (44 - 5 * k) as usize, 20, 4, 6, 200);
        }
        let r = candidates(&na, &nb, &f0, &f1, &f2, W, H, 0.02, 20).unwrap();
        assert_eq!(r.pairs, 1, "the two jaws must be recognised as one gripper");
        assert_eq!(r.cands.len(), 1, "and reported as ONE point");
        let c = r.cands.get(0).unwrap();
        // middle frame: fingers at x = 13..17 and 39..43, centres 15 and 41, midpoint 28 of 64.
        // (Fingers deliberately SMALL: a region cannot be localised better than its own extent, so
        // a fat finger in a narrow frame is refused as TooUncertain — see the note in bl.rs.)
        assert!((c.u - 28.0 / 64.0).abs() < 0.04, "u = {} must be BETWEEN the jaws", c.u);

        let mut t = crate::hand::HandTracker::new(crate::hand::Config::default());
        let arr = r.cands.dense();
        assert!(
            t.observe(&arr[..1]).is_ok(),
            "one point, no rival: the estimator must now be able to accept it"
        );
    }

    /// 🔴 THE DISCONFIRMING CONTROL. Two things sliding the SAME way are not a gripper, must not be
    /// folded, and must leave the estimator abstaining. Without this the fold would merge any two
    /// moving objects and manufacture a hand out of a conveyor.
    #[test]
    fn two_things_moving_the_same_way_are_not_folded() {
        let (na, nb) = (blank(), blank());
        let (mut f0, mut f1, mut f2) = (blank(), blank(), blank());
        for y in [4usize, 30] {
            rect(&mut f0, 4, y, 8, 8, 200);
            rect(&mut f1, 14, y, 8, 8, 200);
            rect(&mut f2, 24, y, 8, 8, 200);
        }
        let r = candidates(&na, &nb, &f0, &f1, &f2, W, H, 0.02, 20).unwrap();
        assert_eq!(r.pairs, 0, "same direction is not a gripper");
        assert_eq!(r.cands.len(), 2, "both survive, so the estimator can refuse");
        let mut t = crate::hand::HandTracker::new(crate::hand::Config::default());
        let arr = r.cands.dense();
        assert!(t.observe(&arr[..2]).is_err(), "two rivals must still abstain");
    }

    /// Two things responded and neither dominates — the anti-elbow case, at the evidence layer.
    /// This file does NOT resolve it; it reports both and lets `hand.rs` abstain.
    #[test]
    fn two_responders_are_both_reported_rather_than_one_being_chosen() {
        let (na, nb) = (blank(), blank());
        let (mut f0, mut f1, mut f2) = (blank(), blank(), blank());
        for y in [4usize, 30] {
            rect(&mut f0, 4, y, 8, 8, 200);
            rect(&mut f1, 14, y, 8, 8, 200);
            rect(&mut f2, 24, y, 8, 8, 200);
        }
        let r = candidates(&na, &nb, &f0, &f1, &f2, W, H, 0.02, 20).unwrap();
        assert_eq!(r.cands.len(), 2, "both, so the estimator can refuse");

        let mut t = crate::hand::HandTracker::new(crate::hand::Config::default());
        let arr = r.cands.dense();
        assert!(
            t.observe(&arr[..2]).is_err(),
            "two equal responders must abstain, never resolve to the first"
        );
    }

    /// Something that moved once and then stopped is NOT where the hand is — it responded to one
    /// step, not to the command. It must not survive.
    #[test]
    fn a_thing_that_moved_only_once_is_not_a_candidate() {
        let (na, nb) = (blank(), blank());
        let (mut f0, mut f1, mut f2) = (blank(), blank(), blank());
        // moves between f0 and f1, then holds still
        rect(&mut f0, 10, 10, 10, 10, 200);
        rect(&mut f1, 24, 10, 10, 10, 200);
        rect(&mut f2, 24, 10, 10, 10, 200);
        let r = candidates(&na, &nb, &f0, &f1, &f2, W, H, 0.02, 20).unwrap();
        assert!(r.cands.is_empty(), "responded once is not responding to the command");
    }

    /// Scattered specks are not one thing, and the fill ratio says so — measured off the region, so
    /// nobody picks a number for "how blobby is blobby enough" here.
    #[test]
    fn scattered_specks_score_far_below_one_compact_region() {
        let (na, nb) = (blank(), blank());
        let (mut f0, mut f1, mut f2) = (blank(), blank(), blank());
        // A hollow 12x12 square, stepping by MORE than its own width so the intersection is the
        // ring itself. (A smaller step makes the first and third footprints overlap, the
        // intersection fills in, and the test silently stops testing a ring at all — which is what
        // the first draft of this test did.)
        for (fr, off) in [(&mut f0, 0usize), (&mut f1, 14), (&mut f2, 28)] {
            rect(fr, 2 + off, 2, 12, 2, 200);
            rect(fr, 2 + off, 12, 12, 2, 200);
            rect(fr, 2 + off, 2, 2, 12, 200);
            rect(fr, 12 + off, 2, 2, 12, 200);
        }
        let hollow = candidates(&na, &nb, &f0, &f1, &f2, W, H, 0.02, 20)
            .unwrap()
            .cands
            .get(0)
            .map(|c| c.rigidity)
            .unwrap_or(0.0);

        let (g0, g1, g2) = stepping(10, 20, 10);
        let solid = candidates(&na, &nb, &g0, &g1, &g2, W, H, 0.02, 20)
            .unwrap()
            .cands
            .get(0)
            .expect("a square is a region")
            .rigidity;
        assert!(solid > 0.8, "a filled square fills its own ellipse: {solid}");
        assert!(solid > hollow, "solid {solid} must beat hollow {hollow}");
    }

    #[test]
    fn a_zero_command_is_refused_rather_than_divided_by() {
        let f = blank();
        assert_eq!(
            candidates(&f, &f, &f, &f, &f, W, H, 0.0, 20).unwrap_err(),
            Bad::NoCommand
        );
        assert_eq!(
            candidates(&f, &f, &f, &f, &f, W, H, f64::NAN, 20).unwrap_err(),
            Bad::NoCommand
        );
    }

    #[test]
    fn frames_that_are_not_the_same_shape_are_refused() {
        let f = blank();
        let short = vec![0u8; 10];
        assert_eq!(
            candidates(&f, &short, &f, &f, &f, W, H, 0.02, 20).unwrap_err(),
            Bad::ShapeMismatch
        );
    }

    /// 🔴 The cross-check is what stops this being another self-reported number: a region whose
    /// travel disagrees with the already measured `image_jacobian` is not believed, however
    /// confident the reader is about it.
    #[test]
    fn a_region_that_disagrees_with_the_measured_jacobian_is_not_believed() {
        let c = Candidate { u: 0.5, v: 0.5, gain: 4.0, rigidity: 0.9, pixels: 200, spread: 0.01 };
        assert!(agrees_with_jacobian(&c, 4.10, 0.27, 3.0), "within 3 sigma is believable");
        assert!(!agrees_with_jacobian(&c, 9.00, 0.27, 3.0), "18 sigma out is a refusal");
        assert!(!agrees_with_jacobian(&c, 4.10, 0.0, 3.0), "a jacobian with no sigma adjudicates nothing");
    }
}

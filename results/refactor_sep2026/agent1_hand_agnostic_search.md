# Free-search agent 1 (2026-09-06): effector-agnostic manipulation from self-measured models

Mandate: goal + constraints only (no candidate methods given). 132 tool uses, 26 min. Facts kept verbatim; verdict section is the agent's, not ours (§3.8: discount defeatism, keep facts).

Scope covered: uncalibrated visual servoing (1994–2025), model-less/kinematic-free control, developmental self-identification, sensorimotor-contingency theory, caging/enclosure theory, environmental-constraint grasping, extrinsic dexterity, compliant-hand in-hand manipulation, prior-free articulated-object manipulation, neural self-modeling (for exclusion), VLM-marks brain contracts, task literature (typing, pen, cap, drawer, cloth). PDFs read: MAVRIC, DIJE, Rodriguez caging, Eppner ISRR+IJRR, Bhatt RSS21, Sieler IROS23, Morgan RAL22, Kormushev IROS15, Karayiannidis TRO16, Tac-Man, Kemp&Edsinger, DURableVS, AutoURDF, Odhner IJRR15, CLIPUNetr/VGS-IL.

## 1. Ranked: closest to effector-agnostic manipulation from self-measured models

### 1.1 MAVRIC (Yang, Jayaraman, Berseth, Efros, Levine; RA-L 2020) — https://arxiv.org/abs/1912.13360
- Measures: ~100 random commands (~20 s); Lucas–Kanade point tracks; responsiveness R_i = I(ΔS_i; A) (mutual information track-motion vs command) → most responsive control point (top-K avg); Broyden rank-1 Jacobian Ĵ ← Ĵ + (ΔS − ĴA)Aᵀ/‖A‖², batched over last 10 tuples, re-init after 20 failed steps.
- Assumes nothing about DOF, rigidity, link lengths, camera pose, proprioception, markers; no nets.
- WidowX + uncalibrated RealSense: 3-D reaching median error **5.2 cm** (3.5–6.8 by held tool); oracle EE 2.3 cm; MoveIt full kinematics 4.4 cm. Self-recognition >90%, ~1–2 cm. Works with pliers/wrench/marker as effector, amputated arm, hand-held shaky camera.
- Cannot: no grasping, no in-hand; single-camera occlusion; accuracy floor = EE localisation (~2 cm).

### 1.2 Uncalibrated visual servoing lineage (the theory our response table uses; includes hands)
| Paper | Measures/assumes | Result |
|---|---|---|
| Hosoda & Asada IROS 1994 | online Jacobian, no camera/link params | founding |
| Jägersand, Fuentes, Nelson ECCV 1996 (cs.rochester.edu/users/faculty/nelson/pubs/abstracts/1996_eccv.html) | full coupled image Jacobian online; trust-region; "grasp tetrahedron" motor model acquired at grasp time | 3/4/6-DOF |
| Jägersand et al. ICRA 1997 (ieeexplore 606723) | 3, 6, **12 DOF** (Utah/MIT hand fingers) | PUMA up to 5× more precise than joint control; hand 2×; **stiction + flexibility → stick then overshoot** |
| Piepmeier & Lipkin IJRR 2003 | eye-in-hand, moving target, Broyden with target-motion term | converges on moving target (wrist-camera case) |
| Farahmand, Shademan, Jägersand IROS 2007 | store history; local LS Jacobian near visited points | no re-babbling |
| Shademan et al. ICRA 2010 | robust M-estimation rejects tracker outliers | |
| Gridseth, Hertkorn, Jägersand CRV 2015 (ieeexplore 7158346) | IBVS moves **individual fingers** to grasp points in image | |
| ViTa, Gridseth et al. ICRA 2016 (ieeexplore 7487521); CLIPUNetr arXiv 2309.09183; VGS-IL arXiv 2003.02768 | task = image-geometric constraints: **point-to-point, point-to-line, line-to-line, parallel-lines**, executed by Broyden UVS | WAM/Barrett |
| Dodds, Jägersand, Hager, Toyama ICVS 1999; Hager ICCV 1995 | image-space primitives composed hierarchically; projective invariants | |
- Cannot: local linear model breaks at contact-mode changes and stiction; none grasps by enclosure without a separate planner.

### 1.3 DIJE (Toshimitsu et al., IROS 2022; arXiv 2507.00446)
Per-pixel image Jacobian from dense optical flow + joint velocity, diagonal Kalman; self/other label per pixel robust to overlapping external motion; UVS on top. Musashi humanoid, qualitative. = dense variant of our response table (body mask + Jacobian at every pixel incl. held tool).

### 1.4 Model-less control, unknown kinematics (Yip & Camarillo T-RO 2014; RA-L 2016)
Jacobian recursively estimated from inputs and tip displacement; **hitting an unknown constraint rotates/scales the estimated Jacobian** and control continues where model-based hits an artificial singularity. Needs tip position + force. Catheter traced paths under heartbeat. Principle: **constraint = Jacobian change** = model-free contact detector (needs a force-like channel; motor current qualifies).

### 1.5 Kinematic-free / encoderless (Kormushev, Demiris, Caldwell IROS/ICRA 2015; kormushev.com)
Exploratory torques → EE motion in overhead camera; no encoders. 2-DOF planar; adapts to **100% link elongation, 35° joint offset, added joints** — strongest demo of re-measuring after morphology change.

### 1.6 Sieler & Brock IROS 2023 "Dexterous soft hands linearize feedback-control for in-hand manipulation" — arXiv 2308.10691
Control variable = hand deformation (strain sensors); Jacobian actuation→deformation by **finite differences from explorative actuations**; reused while error decreases, re-queried when it stops; NN Jacobian database. RBO Hand 3 (16 pneumatic). Skill in minutes; generalises to +100% object size, 360° palm inclination, **50% actuators disabled**. Only found in-hand method with an act→observe hand model. Needs compliant hand + per-finger sensing channel; target from a demo.

### 1.7 Bhatt, Sieler, Puhlmann, Brock RSS 2021 "Surprisingly robust in-hand manipulation" — arXiv 2201.11503
No sensing, no models, no learning: open-loop keyframes (13 spin+shift, 6 gait). Same signal manipulates 7 objects; speed varied 80×; 140 consecutive Rubik's-cube spins. Keyframes hand-specific; rigid hands lose the funnel.

### 1.8 Morgan, Hang, Wen, Bekris, Dollar RA-L 2022 — arXiv 2201.07928
Yale Model Q, no encoders, no tactile; planner over "orthogonal safe modes"; needs neural 6-D object tracker; full SO(3) gaiting. Related Odhner & Dollar IJRR 2015 (open-loop precision manipulation, underactuated).

### 1.9 Caging → grasping — Rodriguez, Mason, Ferry IJRR 2012 (doi 10.1177/0278364912442972)
"Given a cage of an object, there is always an infallible blind strategy to grasp it: either close or open the fingers … there is no need for accurate finger positioning or feedback." **For two-fingered manipulators all cages are pregrasping cages**; >2 fingers need monotone "grasping functions" (F_min/F_max, decentralised). Any dimension, hole-free shapes, any number of point fingers. Assumes rigid point fingers, known finger formation. Also Rimon & Blake IJRR 1999 (cage breaks at frictionless equilibrium grasp); "Caging in Time" IJRR 2025 arXiv 2410.16481 (single effector forms the cage sequentially, open-loop, no object geometry).
- For us: replaces contact pixels + round heuristic with *surround, then monotonically squeeze/stretch until resist*; cage existence must be **measured** (object blob stays inside effector-blob hull under perturbation).

### 1.10 Environmental constraints — Eppner, Deimel, Álvarez-Ruiz, Maertens, Brock IJRR 2015
Surface-constrained grasp (palm to table, fingers slide along surface), wall-constrained, slide-to-edge; **force-compliant finger closing keeps fingertips on the table while closing**. Barrett hand, cylinders 8–50 mm (Fig. 12): large cylinders reliable both ways, **small cylinders only with force-compliant closing**. RBO Hand 2: 31/33 Feix grasp types with 4 signals (Deimel & Brock IJRR 2016). Defines approach relative to environment (support/wall/edge), all measurable.

### 1.11 Extrinsic dexterity / pivoting with a parallel gripper
Chavan-Dafle & Rodriguez ICRA 2014; Holladay, Paolini, Mason ICRA 2015 (open-loop pivoting, no tracking); Viña et al. IROS 2015 (controlled slip); Waltersson & Karayiannidis IJRR 2025 arXiv 2410.19660 (friction identified in ~2 s at pickup; slip error 0.3–8.3 mm, 3.7–17.1°); Hou, Jia, Mason RSS 2020; CMGMP ICRA 2022. All model friction explicitly → must be measured (slip-vs-grip test).

### 1.12 Articulated objects, no mechanism prior
Karayiannidis et al. T-RO 2016 (velocity control + wrist F/T, hinge estimated online, convergence proof); Jain & Kemp ICRA 2010; Tac-Man T-RO 2025 arXiv 2403.01694 (GelSight, prior-free, 100% weighted success). All assume the robot's kinematics; need F/T or tactile.

### 1.13 Developmental self-identification
Michel, Gold, Scassellati IROS 2004 (self = motion inside learned command→perception delay window; survives arm-shape change); Kemp & Edsinger ICDL 2006 (MI motion-patch vs arm config → hand in <2 min); Stoytchev Robotica 2011 (efferent–afferent delay); Fitzpatrick & Metta IROS 2003 "First contact" (object segmented by poking); Natale et al. 2005/2007; Philipona, O'Regan, Nadal Neural Comput. 2003 (dimensionality of rigid group from raw I/O); Hoffmann et al. TAMD 2010 review; Sturm et al. 2009 (prediction-quality monitor triggers re-learning); self-touch calibration: Roncone ICRA 2014, Gama et al. ICDL 2020 arXiv 2008.13483, DLR arXiv 2311.03957 (DLR-Hand II: max error 17.7→3.7 mm, ~9 min, kinematic tree known).

### 1.14 Neural self-models (excluded by "no weights"; ceiling evidence)
Kwiatkowski & Lipson Sci. Robot. 2019 (4-DOF, <35 h, ≈4 cm, pick-place 100% closed-loop vs 44% open); Chen et al. Sci. Robot. 2022 (visual self-model ≈1% workspace); Li et al. Nature 2025 arXiv 2407.08722 (Jacobian fields from random commands, single camera = neural version of our table); AutoURDF arXiv 2412.05507; DURableVS arXiv 2202.03697 (<50 samples but assumes DH chain + pinhole).

### 1.15 Brain-contract analogues
MOKA RSS 2024 arXiv 2403.03174; PIVOT ICML 2024 arXiv 2402.07872; RoboPoint arXiv 2406.10721 — all assume calibrated depth + IK. **Nothing found joins a marks/cells contract to an uncalibrated self-measured body.**

## 2. Fine in-hand manipulation without a hand model?
Yes partially: Sieler 2023 (finite-difference Jacobians of whatever channels touch the object = same machinery as arm UVS, compliant hand); Rodriguez–Mason 2012 (grasp = monotone scalar from a cage; 2-finger needs no feedback); Bhatt 2021 funnels; extrinsic dexterity (grip + arm + environment); Jägersand 1997 (12-DOF fingers, stiction broke small motions).
No on the named tasks: one-finger typing — no model-free result (HandelBot arXiv 2603.12243, Braille RL arXiv 2008.02646, FingerViP arXiv 2604.21331 all learned; formally ViTa point-to-point + guarded move, Lozano-Pérez/Mason/Taylor IJRR 1984); pen rotation — only learned or model-based (Sundaralingam & Hermans ICRA 2018 needs mesh + kinematics); cap turning — rotate–release–regrasp with slip monitoring (GelSight arXiv 1810.13381; TIAGo++ DRL 2024); drawer/door — prior-free w.r.t. mechanism only; cloth — no effector-agnostic formulation (review arXiv 2407.01361).

## 3. Agent's verdict (its words)
No perfect method exists. Furthest full-stack no-weights system (MAVRIC) stops at reaching, 5.2 cm, never closes a gripper. Most general formulation = (a) self via responsiveness/delay contingency; (b) local linear sensorimotor model online, re-estimated on prediction failure; (c) tasks as image-geometric constraints (ViTa); (d) contact/grasp by monotone blind strategies under compliance (caging→squeeze, environmental constraints, funnels) with guarded until-conditions. Failure frontier: self-localisation floor (5.2 vs 2.3 cm); linear model invalid across contact-mode switches/stiction/large steps; cage certification needs geometry not visible from one head camera; in-hand dexterity on rigid hands without models/weights: no published success; single-camera finger occlusion.

## 4. Measurements the body can add by acting (literature-backed)
| # | Measurement | Why | Source |
|---|---|---|---|
| 1 | Per-blob responsiveness I(ΔS;A) over ~20 s random commands; top-K effector points per channel | held tools become body automatically | MAVRIC; Kemp & Edsinger 2006 |
| 2 | Command→motion delay window per channel | self vs external motion; survives morphology change | Michel 2004; Stoytchev 2011 |
| 3 | Dense per-pixel Jacobian + per-pixel variance | body mask free; servo any pixel incl. tool tip | DIJE |
| 4 | Visual-motor memory (state, J) with local LS near revisited states | no re-babbling; drift detection | Farahmand 2007; Sieler NN DB |
| 5 | M-estimator on Jacobian residuals | rejects tracker outliers ("moved more than commanded") | Shademan 2010 |
| 6 | Residual/rank change of J as contact & constraint detector; re-query J by finite differences when error stops decreasing | model-free until contact/resist | Yip 2014/2016; Sieler 2023 |
| 7 | Held test = object blob responsiveness after closing (suction: pressure series as grip reading) | effector-agnostic held check | MAVRIC; Fitzpatrick 2003; GOMP-ST arXiv 2203.08359 |
| 8 | Cage test before squeeze: object depth-blob inside hull of effector blobs under small perturbation ⇒ blind monotone close/open | replaces contact-pixel geometry; 2-finger provably feedback-free | Rodriguez, Mason, Ferry 2012 |
| 9 | Approach relative to measured support/wall/edge; slide until resist; close with fingers on surface | fixes push-instead-of-enclose for small round objects (Fig. 12) | Eppner 2015 |
| 10 | Slip-vs-grip test at pickup (~2 s) → friction bound | cap turning / pivoting | Waltersson 2025; Viña 2015 |
| 11 | Hand-internal channel→finger-blob Jacobians (wrist cam as strain sensor) | the in-hand primitive that worked | Sieler 2023 |
| 12 | Self-touch: close until mutual contact (resist on both channels) | finds tactile/current channels; calibrates finger geometry | Roncone 2014; DLR 2023; Gama 2020 |
| 13 | Rank of stacked response table | independent DOFs without assuming a count | Philipona 2003 |
| 14 | Prediction-quality monitor → re-measure after morphology change | Kormushev 2015; Sturm 2009 |

## Agent's opinion (marked as opinion)
Every piece of the body layer exists in the literature; no one assembled them past reaching. Missing joint = **enclosure as a measured cage + blind monotone squeeze (item 8) + environment-relative approach (item 9)** — targets the baseball failure with no hand-specific code. Item 6 = cheapest force sensor. Fine in-hand on rigid 25-joint hand with head camera only: nothing published; plan the wrist camera as required.

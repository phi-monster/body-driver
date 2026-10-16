# Alignment in the kinematics: what was measured

The arm's keyframes are aligned to its reference picture by `Driver.Alignment` (a patch of the first picture, warped by
what is predicted, compared with the second picture's pixels, refined by Gauss-Newton on the warp), each keyframe
predicted from the earlier ones by `Driver.Alignment.Keyframes`, and from the fitted arm once there is one. The
instrument service is no longer asked for an arm's own keyframes. This note keeps what the replays of A22, A23 and A79
(box, against the simulator's truth) showed, so that the next change starts from it.

## How honest the stated sigma is

Per axis of every sighting that the truth sees on a smooth surface, |error| over the sigma the aligner stated, by how far
the keyframe's points moved (A23 arm 2, final fit; A79 arm 1 is alike):

| moved | median | 90 % | 99 % | 99.7 % |
|---|---|---|---|---|
| under 4 px | 0.48 | 1.7 | 3.5 | 4.6 |
| 4 to 16 px | 0.56 | 2.0 | 5.3 | 7.9 |
| 16 to 64 px | 0.56 | 3.0 | 14.6 | 23 |
| over 64 px | 0.59 | 5.3 | 41 | 54 |

(a Gaussian: 0.67, 1.64, 2.6, 3.0). The core is as sure of itself as it says, the tails are not: they grow with the move.
Sigma from a sample of fits carries its degrees of freedom (`Answer.Degrees_Of_Freedom`, 3 for nearly every answer), and
against the Student gate with that many (9.2 sigma) the first two classes are honest and the last two are not. The most
confident answers (sigma under 0.15 px) are the likeliest to be off by more than a pixel (22 to 49 %): a patch with
strong texture has a small statistical sigma and the same warp error as any other.

## A prediction wider than a patch gives answers the pixels did not decide

At 130 to 150 px of move (the widest keyframe of each joint of a boot that steps by fourfold amounts) the chain's
prediction states 3.5 to 35 px, the answers it gets are 0 to 75 % within a pixel (the others are consistent with a smooth
field because they hold to the prediction), and their sigma is small. The fit rests on the widest keyframe of each joint
for its axis and its focal length: with these in, the first fit of A79's arm 1 ended at a focal length of 1425 px
(truth 397.04); without them it fits (397.25 x 397.36, z of every lens term under 0.4).

`Answer.Narrow` says whether the window was no wider than a patch at the full size (the aligner's own rule for the size
it searches at). A keyframe most of whose answers are not Narrow is a move the prediction had not followed: it gives no
sightings, and is aligned again when the fitted arm gives it a window narrower by a pixel than the one it was aligned from
(`Kinematics.Realign`, repeated by the recompute while that happens; a keyframe whose answers were followed is aligned again
only when the window is a size of the pictures narrower, so the repeating ends: every round narrows a window by at least a
pixel). The cost is on the first fit: the arm's lens is known to about 2.5 % until the widest keyframes can come in
(A79 arm 1: focal sigma 9.8 px at the first fit, 1.9 px at the last); a sweep with a keyframe between 35 and 140 px would
have the widest ones predicted narrow at once.

## The fit's last stage is chaotic on some data

Perturbing the sightings of A23's arm 2 by 1e-9 px moved its focal length between 396.9 and 399.2 px (stated sigma
0.8 px) in an earlier version of the alignment, and moves it between 397.8 and 398.25 px (stated sigma 0.68 px) in this one;
the other five arms and every RoMa input do not move (under 0.05 px). The descent of the last stage goes down a
curved narrow valley (the focal length against the distortion): the damping of its Levenberg-Marquardt steps swings between
16 and 1e5 and the 53 rounds it is given are spent before it ends, where it ends depends on the rounding. Given 424 rounds
it ends at 399.2 for every perturbation: that is the optimum of these sightings (their sigmas are too small for the far
keyframes, whose true errors are twice what they state), 2.8 sigma from the truth; the 397.5 that 53 rounds reach is a
state that has not converged. The pass loop that re-chooses the sightings now ends at a choice it has met (a fixed point
or a cycle) and not when the number of changes stops falling.

## Two eyes

With the prediction the truth gives (error 0 px) the aligner finds 27 % of the points of one eye's picture in another
eye's at the same beat (22 % within a pixel; the head eye shows what a wrist eye shows at 0.33 to 0.56 of its size);
with an error of 8 px, 10 %, almost all wrong. A prediction that is itself what is being measured cannot start the
head eye: a matcher that does not need one (invariant to the scale and the view) has to.

## A fixed eye that watches an arm (measured on A22 and A79, 10-10; not built)

The head eye of the official rig sees both arms from about 0.4 m. Its picture changes little in a boot (the arm's holds move the
hand 1 to 45 px in A22, much more in A79's later sweeps); what moves is the arm, not the eye. The lock-in already tells which cells
those are (A22, last map: group 1 responds in 69 cells of which 44 are at least 30 % on the left arm, the truth's left arm covers
46 cells; group 3 likewise 67, 47 and 46: recall about 95 %, precision about 65 %, the rest are the table around the arm and its
shadow). Measured with the aligner as it is (holds only: the arm still at a pose different from the rest pose's, both pictures of the
head eye, the truth's rendering of where every arm point of the rest picture lands; A22: 151 poses, 2437 points on a 4 px grid):

- With a prediction exact to the truth, the aligner finds the arm's points that moved 1-2 px in 97 %, 2-4 px 89 %, 4-8 px 80 %, 8-16 px 60 %,
  16-32 px 34 %, over 32 px 19 %; within 1 px of the truth in 94 %, 83 %, 73 %, 51 %, 26 %, 15 % of all those asked; the median error of
  the answers is 0.17, 0.23, 0.35, 0.47, 0.61, 0.67 px. Points whose whole patch lies on one link do better (4-8 px: 91 % found, 85 % within a
  pixel). With the prediction a pixel off: 90 %, 91 %, 86 %, 74 % found for the same four classes. A79 (larger sweeps): 88 %, 75 %, 54 %, 44 %.
- The answers' stated sigma is as honest as the arm's own eye's: median error over sigma 0.8 to 0.4 (a Gaussian's is 0.67); beyond
  three sigmas 1 to 6 % of the axes; their tails are lighter than the far classes of the wrist eye.
- A frame taken while the arm is moving is not a test of anything: the image trails the readings by a beat and the arm moves 2 px a beat in
  a pulse (the first look, on the pairs of every eighth displaced beat, found 15 % of the moving points and was wrong for that reason).
- A linear model of the joint readings fitted on the poses near the rest pose (what the lock-in's regression per cell is) predicts a
  moved point to 0.1 px (moves of 2-4 px), 0.2-0.5 px (4-8), 0.8-1.1 px (8-16), 2.7 px (16-32) at the median (90 %: 0.3, 0.7-1.4, 2.1-2.4,
  4.9 px): about a tenth of the move beyond 8 px, enough to align the next keyframe in a window the aligner can search.
- The Cramer-Rao bound of the eye's lens (fx fy cx cy) and pose (6) from points tracked on the arm with depths unknown and the arm's
  motion known exactly: 200 points over 150 poses, 0.3 px each: fx 0.13 px (0.04 %), centre 0.2 px, pose 0.03-0.04 deg and 0.2 mm; 40 poses:
  fx 1 px (0.3 %); 10 poses: fx 4 px (1.4 %).
- Bundle adjustment of the real answers (found, a prediction a pixel off) with the arm's motion from the truth: 50 to 400 points: fx -0.7 to
  -0.2 %, centre 1-4 px, from poses start 60 deg and 200 mm off.
- The same with the arm's motion from the arm's own fit (the joints of the final fit, composed as Eye_At does) and a start found by a
  global search over the arm frame's rotation into the eye's (the translation, the depths linear for each rotation): see the numbers
  in the commit that adds the module.

The arm's pose error is what limits it: an extra 0.5 mm and 0.2 deg of random error in each pose's motion moved the fit's focal length
by 11 % (synthetic, truth motion); the arm's real fit errors are systematic, not random, and the real test is above.

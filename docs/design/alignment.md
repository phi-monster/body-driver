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

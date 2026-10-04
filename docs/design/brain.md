# The brain layer: what was measured, and what it decided

This note records why layer 5 is the way it is. Every rule below was put in
or taken out on a measurement with the brain the box serves: Qwen3.5-9B on
vLLM 0.29, sampling from the model card (`QWEN_CARD` in `/root/q/run.sh`,
temperature 0.7, top_p 0.8, top_k 20, min_p 0, presence_penalty 1.5,
repetition_penalty 1.0), through the driver's own client
(`driver/tools/brain_measure`). The pictures are the RX5 recording's three
eyes (one fixed in the scene, one on each arm). Where a measurement added a
token limit, it was the measurement's (`max_tokens` 1024), never the
driver's.

## The keyboard on a body with a hand

The question set is 20 tasks of five kinds on the same table: lift (6), turn
(4), push (3), next to (3), on (4). An answer is scored by its first stretch
(the right quantity of the right thing, or the right relation between the
right things) and by what would run: on a keyboard that allows several
stretches every one of them runs, and a placing or a lowering after a lift
undoes the task. A cell gives the answers whose first stretch is right and,
in brackets, those where nothing else would run.

Answers to the same question are correlated, and two series of the same
keyboard measured hours apart differed by more than their own spread (QH:
lifts 30 of 30 in one, 52 of 60 in another). So forms are compared only
within one series, measured side by side.

### The sentence about two things, first series (5 repeats)

| keyboard | lift | turn | push | next to | on | cut as runaway / limit, per 100 |
|---|---|---|---|---|---|---|
| Q: height | 29 (27) / 30 | - | - | - | - | 7 / 2 |
| QH: height, heading | 30 (30) / 30 | 20 (19) / 20 | - | - | - | 3 / 0 |
| Q2: Q and `do <thing> <relation> <thing>`, five relations | 24 (16) / 30 | - | 5 / 15 | 2 / 15 | 18 / 20 | 32 / 15 |
| QH2: QH and the same | 26 (17) / 30 | 20 (14) / 20 | 1 / 15 | 3 / 15 | 18 / 20 | 18 / 5 |

(a dash: the keyboard cannot say it.) The sentence makes the tasks about one
thing worse: the brain adds a placing nobody asked for after a lift, and
writes a relation word as a direction (`do the scissors right ...`) where it
wanted a turn, which the decoder then glues into an endless name. The
owner's rule for adopting it, no worse on one thing and no worse on two, is
not met.

### Other forms of it (5 repeats each; two-thing cells Q / QH)

| form | lift (Q / QH) | turn (QH) | push | next to | on | runaway / limit per 100 (Q / QH) |
|---|---|---|---|---|---|---|
| one key per relation, five relations | 6 (6) / 18 (9) | 20 (18) | 15 / 15 | 15 / 15 | 20 / 20 | 31 / 1, 9 / 1 |
| touching and above only | 23 (18) / 27 (15) | 20 (17) | 0 / 0 | 15 / 10 | 16 / 18 | 25 / 2, 9 / 0 |
| touching and onto only | 25 (18) / 29 (19) | 20 (15) | 0 / 0 | 14 / 10 | 17 / 18 | 21 / 3, 1 / 0 |

One key per relation makes every two-thing task right, but on the height
keyboard the brain then drops the quantity from the change sentence (`do
scissors up until free`), whose name slot loops. Every form adds stretches
nobody asked for after a lift or a turn (a placing, or `heading down` read as
putting it down).

### One stretch per program

The quantity keyboard has no `if` and no `try`: a second stretch could not
depend on how the first ended, and remedies happen across rounds anyway
(LANGUAGE.md 17.8). With one stretch at most in a program (and `say` lines
around it), nothing unasked can run after the first stretch. Measured side
by side, 10 repeats:

| keyboard | lifts that would run right | turns that would run right |
|---|---|---|
| Q | 58 / 60 | - |
| Q, one stretch | 59 / 60 | - |
| Q, touching and onto | 44 / 60 | - |
| Q, touching and onto, one stretch | 55 / 60 | - |
| QH | 52 / 60 | 38 / 40 |
| QH, one stretch | 56 / 60 | 36 / 40 |
| QH, touching and onto | 42 / 60 | 33 / 40 |
| QH, touching and onto, one stretch | 54 / 60 | 38 / 40 |

One stretch per program is no worse on either keyboard (59 against 58, and
92 against 90 of 100, with the earlier wording of heading), so the quantity
keyboard has it. The sentence about
two things with touching and onto, one stretch per program, is no worse on
the heading keyboard (92 against 90) and does what it is for there (5
repeats: on 18 of 20, next to 6 of 15, push 0 of 15, nothing ran away); on
the height keyboard it is 3 of 60 lifts worse, all of them the change written
without its quantity (`do the scissors until free`, cut as a loop), and it
does less (on 11 of 20, next to 8 of 15, 10 of 50 cut as loops). It is not
on the keyboard: the rule is no worse, and on the height keyboard that is
not shown. Push needs left and right, which every form that has them makes
worse. Offering it would also need Driver.Action to say which of these
relations it can carry out between two things.

### Interleaved, sized for a 3-point drop at Z

The verdict above rested on 60 lifts per keyboard. It was measured again
with the action layer's glosses (Driver.Action.Meaning), on two scenes (RX5
at beat 630 and at beat 100), each question asked on both keyboards in turn
and in the other order on the next repeat, so both arms share the service's
drift. The sample was sized from the observed rates so that Z se is 3 points
if the arms are equal: n = Z^2 2p(1-p) / 0.03^2 per arm, with p = 0.95 for
the height keyboard's lifts (950, run 960) and p = 0.92 for the heading
keyboard's lifts and turns (1472, run 1500). Both keyboards allow one
stretch; "with" adds `do <thing> touching|onto <thing> until <ending>`.

| keyboard | without | with | drop, and its interval at Z = 3 |
|---|---|---|---|
| Q, lifts | 954 / 960 | 768 / 960 | +19.4 points [+15.4, +23.3]: worse |
| QH, lifts | 860 / 900 | 725 / 900 | +15.0 [+10.5, +19.5]: worse |
| QH, turns | 399 / 600 | 571 / 600 | -28.7 [-35.0, -22.3]: better |
| QH, lifts and turns | 1259 / 1500 | 1296 / 1500 | -2.5 [-6.4, +1.4] |

Answers cut as runaways or at the limit: Q 20 against 191, QH 75 against
157. With the sentence the brain writes the change without its quantity
(`do the paddle until free`), whose name slot loops. Lifts are worse on both
keyboards, so the sentence stays off. Without the sentence, two turns in
three are right on the heading keyboard: the brain lifts first (`do the pen
height up until free`) with the action layer's wording of heading; the
earlier wording was not yet compared side by side. The two-thing questions
of this series had 50 answers when it was paused (on and next to: 8 of 12
with the sentence, 0 of 14 without; push 0 of 24).

## The look key

`say look = <n>` was typed through the free sentence. One answer in three on
the first question counted `say look = 90, 91, 92, ...` past eyes that do
not exist; every line differed, so no repetition rule could stop it, and it
ran to the measurement's 3000-character limit in 42 s. Now look is a key of
its own that types only the numbers of the eyes that see, and a free
sentence cannot hold `=`.

## done: the rule lives in execution, not in the keyboard

With "rounds until the brain says done", 5 of 5 answers were `do the scissors
height up until free / (say done) / done`: the task would end after one
attempt whose ending the brain never saw, and a slip could never be
remedied (on the quantity keyboard remedies happen across rounds).

The first fix made such a done untypeable: done only after lines that move
nothing. It made the answers worse. The model's own ending is `done`; without
it the model went on writing, and the decoder turned a wanted `done` into
`do <name>`, whose name slot then looped. On the lift questions 22 of 57
answers ran away (11 of 15 on Q2), 3 more hit the limit, and many kept
programs lowered the thing again after lifting it.

So done stays typeable anywhere and always ends the program, and it ends the
task only where the brain saw every stretch end: none ran before it, or each
ran inside a try or was followed by an if or a repeat until that read its
ending. Otherwise the next round says how each stretch ended and that done
came too early.

## Reading a streamed answer

The model does not stop after a done: of the answers read to their end
(below), every one that had a top-level done went on repeating itself or
talking until the token limit. Reading stops at that done.

Copy loops are cut where they start, and only the finished lines before them
are kept:

- a run of whole lines that comes again at once;
- inside a `say` sentence, a run of whole words that comes again at once;
- inside a name, a run of units that comes again at once, where the units of
  a word are the language words glued into it and the letters between them.
  The decoder cannot type a language word in a name slot, so every ending the
  model wanted there was glued into one word that never ended
  (`untilfreesettledtimeoutstuckslipped...`): 6 of 35 answers on Q2 ran to
  the limit that way before this rule.

Nothing still being written is compared: not the last line, not the last
word, and of a word still growing only the units no later letter can change.
The verdict depends on the text alone, never on how the stream was cut.

To verify the rules, 120 answers (the 20 tasks, twice each on Q, Q2 and QH)
were read to their real end, with the verdict taken on every event:

| keyboard | cut at a done (full answer to the limit) | cut as a loop (full answer to the limit) | never cut (to the limit) |
|---|---|---|---|
| Q | 36 (29) | 2 (2) | 2 (0) |
| Q2 | 20 (20) | 18 (18) | 2 (0) |
| QH | 38 (35) | 1 (1) | 1 (0) |

A cut at a done came at a median 1.33 s (at most 4.0 s), a cut at a loop at
a median 2.33 s (at most 5.9 s); the full answers took a median 41 s, the
measurement's limit. No loop was cut that would have ended by itself, and no
answer ran to the limit uncut. Reading stopped 4161 of the 4351 seconds the
cut answers would have taken (96 %), and the box's service, without that
limit, allows answers of 65536 tokens.

vLLM stops generating when the client closes the stream: an answer cut by
the driver's client after 0.9 s added no token to the service's counter in
the 7 s after, and the service counted no running request 2 s after the
close.

## Names

The binder follows LANGUAGE.md 17.7 and never guesses. Two things were
changed on measurements:

- A box on the body ends the asking. For `arm reach ight`, eye 1 boxed the
  robot's right arm 3 of 3 times; asking eye 2 next gave the scissors or the
  lego man. The eye did point the name out; parts of the body go by their
  role, never by a name, and the other eyes' answers are guesses.
- The where-is-it answer is decoded under a grammar of exactly the answers
  the driver reads. Under the JSON schema the decoder may write blanks
  between the members, and with greedy decoding the model wrote `{"found":`
  followed by tabs until the token limit; the driver has no timeout, so the
  body would have waited for the service's own limit.

The replay set is the stretches of the old driver's S1A1 to S1A5 rounds (24
distinct names, 44 occurrences in program order, 5 runs), bound with the
driver's binder against the same scene's clean RX5 pictures at beat 100,
with the eyes asked live. The old rounds' own pictures have the old grid
drawn on them and only one eye saved, so they could not be used. What each
name meant was read off the rounds by hand: the scissors (14 names), the body
or nothing (8), or either (2). The patch a box picks out was read off regions
marked by hand on the same pictures.

| | card sampling, 3 repeats | greedy |
|---|---|---|
| names that mean the scissors | 87 of 87 bound to the scissors | 29 of 29 |
| names that mean either | 15 of 15 | 5 of 5 |
| names that mean the body or nothing | 13 of 30 left unbound | 5 of 10 |

Every wrong binding is the eye boxing a thing for words that name none
(`reach right untilstuck`: the scissors 6 times, the paddle twice), which
step 2 of 17.7 takes as the eye's answer. Telling the eye that words naming
no thing are also a "cannot see" made it worse (7 of 30, and one scissors
name lost), so the question stayed as it was. Such names come from the old
driver's prompt: on the current keyboards, 333 stretches written for the 20
questions used clean names only (`the pen`, `the lego man`, ...).

### On colour recordings, scored against the truth

The TX5U and TX5V recordings carry the simulator's truth beside them: every
object's pose and mesh, and every camera's pose and lens. `brain_scene`
draws from it, for each eye's picture, which object every pixel shows (the
nearest surface wins; the robot's links too). The two recordings are the
same table of ten objects and the same first 218 beats of motion; the arm
eyes see the table only in those beats. Twelve scenes were taken, six from
each (TX5U beats 50, 106, 143, 186, 204, 300; TX5V 98, 120, 150, 162, 198,
400; the image drawn from the truth one beat earlier, the lag the frames
show), two of them with both arm eyes on the wall.

Each scene was asked, through the driver's round and keyboard (QH, one
stretch), "Lift X off the table." for each of the ten objects, once looking
through eye 1 (the fixed one, as a task starts) and once through each arm eye
(as after `say look = n`), and "Lift one of the things on the table,
whichever you like." three times per view. Every name the programs wrote was
asked of every eye; every box was segmented by the instrument as Identify
asks it; the binder ran on those answers. What a patch is was read off the
truth: the object most of its pixels show, or the body, or nothing. What a
name meant was read off its words (all 33 distinct names were checked by
hand).

Identify used to call a patch part of the body when the pixel nearest its
middle lay on the body. A keyboard under a gripper finger was 99 % keyboard
and still "part of me" (2127 of 2149 pixels). A patch is now what most of its
pixels are (`Mostly`): 4 bindings came right, 1 that was right by luck (a
patch 55 % on the finger) went wrong.

| named-target names (277 that point at one object) | TX5U | TX5V |
|---|---|---|
| bound to the object meant | 111 / 135 | 125 / 142 |
| without the black stand | 108 / 119 | 124 / 128 |

By object (both recordings): cutter 31/31, jeans 24/24, ball 27/28, fan
25/26, Lego figure 23/24, keyboard 30/32, scissors 24/26, pen 30/33, paddle
18/23, black stand 4/30. Free choice: 79 of 79.

The wrong ones, by cause (of 283 names):

- wording, 6: every one on the Lego figure, `the`, `the thing`, `the red
  robot`. Each came after a `say` line that wrote "Lego" with a capital; a
  name slot takes lower case only, so `do the Lego` could not be typed.
- the eye's box, 33: 26 for "the black stand" (eye 1, the only eye that
  shows it, said it cannot see it 14 times in 30; the arm eyes boxed a black
  gripper finger with whatever lay under it 50 times in 60); the paddle under
  the right gripper, which no eye pointed out, 4; an arm eye that does not
  show the thing boxing another one, 3 (the scissors as the pen twice, the
  fan as the paddle).
- segmentation, 8: the box of a small or thin thing in the fixed eye (pen 3,
  ball, Lego figure) segmented as the table, 5; the patch mostly the gripper
  over the keyboard or the paddle, 3.

The where question alone, every eye of every name but the stand's: an eye
that shows the thing boxed it 422 times in 467 and said it cannot see it 44
times; an eye that does not show it said so 269 times in 274 and boxed
another thing 5 times. Looking through an arm eye, a name was bound right 69
of 70 times when that eye shows the thing and 68 of 78 when it does not
(the stand left out).

Of the 360 named-task rounds, 77 named nothing: 6 of 120 through eye 1,
71 of 240 after a look to an arm eye, where the program was `say look = 2`
written again until it was cut, a `say` holding the change (`say do the
baseball height up until free`), or talk cut at the token limit; 30 rounds
ended at a done with no change before it, which ends the task.

# The body language, as the driver reads it

A brain moves a body by writing programs in this language. A program says
which relations should hold and what event ends each stretch. It never says a
joint, a coordinate, a unit, a camera number or the name of a robot: the body
measures itself, and the language cannot say any of that.

This page is the reference for what the driver accepts, written from its
parser, keyboard, name binder and interpreter. The design and its reasons are
in [`driver/LANGUAGE.md`](../driver/LANGUAGE.md) (Chinese), whose section 17
this page follows. Every program on this page marked as one is read by the
driver's self test, so the examples cannot drift from the parser.

## 1. A program

One statement per line. Words are matched without regard to case; names are
kept as written. A line that starts with `#` is a comment, and blank lines
are skipped. Lines run in order; blocks open with a line ending in `:` and
close with `end` on a line of its own.

```program
# lift it, then say so
do the scissors height up until free
say I lifted the scissors
```

Nothing moves until the whole program has been read and checked (section 8).

## 2. The keyboard of a round

Each round the body builds the keys it can use now, from what it measured
about itself, and shows them to the brain. The same keys go to the brain
service as a grammar for constrained decoding
([`brain-service.md`](brain-service.md)), so the brain can type nothing
else. There are three keyboards.

**The quantity keyboard**, when a grasper is bound (the body has a hand that
closes, and an eye sees it) and the body measures a quantity of things:

```
do <thing> <quantity> up|down until <ending>
say <one sentence in your own words>
say look = <eye number>
done
```

The quantities are the ones the body can measure and change now, each listed
with its meaning in the action layer's words: `height` is how high a thing is
above the surface it rests on, and `up` takes it off that surface; `heading`
is which way its long side points about its up, and `up` turns it
counter-clockwise about its up; `tilt` is how far it leans as the eye that
stays still sees it, and `up` leans its top away from that eye.
Where to hold the thing, from which side to come, and when to close are the
body's to work out: the brain says only which quantity of which thing should
change, and which way.

A program on this keyboard asks for one change at most, with `say` lines
before and after it. The keyboard has no `if` or `try`, so a second change
could not depend on how the first one ended; the next round shows how it
ended, and the brain writes the next one then. This is the keyboard's rule,
not the language's: a program read without the keyboard (a person typing)
runs every change it has.

```program
do the mint green scissors height up until free
```

**The full keyboard**, when some part of the body is bound but no grasper (a
drone, a wheeled base without fingers):

```
do <constraint> (and <constraint>)* until <ending> [or <n> steps] [with my still eye | with my moving eye]
<constraint> ::= <who> <relation> <what> [small | medium | large]
              | <who> close <what> | <who> open | <who> still
repeat <n> times: ... end
if <outcome>: ... [else: ...] end
try: ... or: ... end
to <name>: ... end        run <name>        remember where <who> is as <name>
say <one sentence in your own words>        say look = <eye number>        done
```

**Speech only**, when no part of the body is bound this round: `say` and
`done`.

On every keyboard:

- `<who>` lists only the roles bound now, and `free` is offered only when the
  surface a thing rests on is measured.
- `until arrived` and `until refused` are never offered (section 4).
- `say look = <eye number>` is offered when the body has more than one eye
  with a picture, and types only the numbers of those eyes. A free sentence
  cannot hold `=`, so it cannot type an eye that does not exist.
- A name is plain words, as many as it needs, but none of the words of this
  round's grammar, and not `item`. The decoder cannot type a language word
  inside a name, so a word the brain wanted there ends up glued to its
  neighbour (`upmint` for `up mint`). The name binder reads names by their
  letters for exactly that reason (section 7).

The sentence about two things (`do <thing> <relation> <thing> until
<ending>`) is not on the quantity keyboard. Measured with Qwen3.5-9B on five
kinds of task, every form tried made the tasks about one thing worse on at
least one keyboard, so none is offered ([`design/brain.md`](design/brain.md)).
The parser reads it all the same.

## 3. Statements

The parser accepts all of this, including forms no keyboard offers, so a
person who types as the brain is understood too.

### Stretches

```
do <constraint> (and <constraint>)* until <ending> [or <n> steps] [with my still eye | with my moving eye] [anyway]
```

| constraint | meaning |
|---|---|
| `<subject> <relation> <object> [small\|medium\|large] [must]` | the relation should hold |
| `<subject> press <object> light\|firm\|hard [must]` | push against it, saying only how hard |
| `<subject> close [<object>]` | close on it, or where it is when nothing is named |
| `<subject> open` | open |
| `<subject> still` | stay still during the stretch |
| `<thing> <quantity> up\|down` | change a measured quantity of a thing (the whole stretch is one constraint) |

- A subject is a role (`me`, `grasper`, `pusher`) or a thing's name; an
  object is a role, a thing's name, or a place remembered by `remember`.
- `small`, `medium` and `large` bound how far one step may go (section 5).
  Without one, the body sizes each step from what it measured.
- `must`: this constraint may not be given up to keep the others. Without it,
  a constraint may be. The word `prefer` is not a keyword: it would be read
  as part of a name.
- `press` needs an effort and takes no step size; effort words go with
  `press` only. `still` and `open` take nothing after them.
- `on` right after a relation is skipped: `do grasper close on ball until stuck`.
- `or <n> steps` stops the stretch after n steps if its ending has not come.
- `with my still eye` uses an eye that rides on no arm; `with my moving eye`
  uses the eye on the arm that moves in this stretch. Without either, the
  body picks the eye from what it measured.
- `anyway`: the body still says what it fears (blind, out of reach), but
  does it.
- In the quantity sentence, everything before `<quantity> up|down` is the
  name, so such a name may contain `and` or relation words.

```program
do grasper touching ball until touched or 20 steps
do grasper close ball until stuck
do grasper farther ball small until slipped or 10 steps with my moving eye
do me still until settled or 20 steps
do pusher press the drawer firm until stuck
```

### Control

```
repeat <n> times:           repeat until <ending>:
if <outcome>:   [else:]     try:   or:
to <name>:                  run <name>
remember where <subject> is as <name>
say <sentence>              done
```

```program
to pick up ball:
remember where grasper is as start
repeat 3 times:
do grasper touching ball until touched or 20 steps
do grasper close ball until stuck
do grasper farther ball until slipped or 10 steps
if slipped:
do grasper open until settled
do grasper touching start until touched or 20 steps
else:
done
end
end
say I tried three times and it is not in my hand
end
run pick up ball
```

- A behaviour is defined with `to <name>:` anywhere in the program and run by
  its letters with `run <name>`. `do <name>` is not a call: `do` is followed
  only by constraints.
- `remember where <subject> is as <name>` records where something is as a
  place the body can find again (which eye, where in the picture, how far),
  never coordinates. A place remembered in this program or an earlier one of
  the episode can be the object of a constraint.

## 4. Endings

Every stretch ends with exactly one of ten endings, and control flow reads
nothing else:

| ending | meaning |
|---|---|
| `arrived` | what you asked for holds (only the brain can judge this) |
| `touched` | I touched something |
| `stuck` | I was commanded and did not move |
| `slipped` | what I was holding left my hand |
| `lost` | I can no longer see what I was following |
| `free` | the thing left the surface it was resting on |
| `settled` | the picture stopped changing |
| `stalled` | I keep moving but the gap stopped shrinking |
| `timeout` | I took every step the stretch allowed |
| `refused` | I cannot do it, and I say what I tried |

- `until arrived` is refused: whether it arrived only the brain can judge,
  and the body only notices events. `until refused` is refused too: refused
  is the body's answer, not an event to wait for.
- `free` is waited for only when the surface the thing rests on is measured.

## 5. Relations

| relation | the subject ends up |
|---|---|
| `touching` | against the object |
| `above` / `below` | above / below it; in an eye carried by the hand, above by gravity, not by the picture |
| `left` / `right` | to its left / right in the picture |
| `nearer` / `farther` | nearer to / farther from the eye that sees it best |
| `onto` | pressed onto the surface it rests on |
| `off` | off that surface |
| `into` | inside it, halfway between its skin and the surface it stands on |
| `facing` | turned until this part points at it |
| `clear` | never closer to it than now |
| `still`, `press`, `close`, `open` | see the constraints above |

A step size is an upper bound ("at most this far in one step"). Without one,
the body sizes each step itself: within reach, still followable by an eye,
and far from anything it might hit.

## 6. How a program runs

- Statements run in order. Each stretch goes to the body, which moves step by
  step until one of the endings happens.
- `if <outcome>` reads how the last stretch ended, including a stretch of an
  earlier program of the same episode.
- `repeat <n> times` runs its lines n times. `repeat until <ending>` runs
  them, then again, until a pass ends with that ending.
- In `try`, a stretch that ends `stuck`, `slipped`, `lost`, `stalled`,
  `timeout` or `refused` abandons the attempt for the `or` lines, also from
  inside a behaviour the attempt called. A `try` around `until stuck` jumps
  even when the stretch stopped exactly as asked.
- `say` sends the sentence to the person and to the next round. `say look =
  <n>` also makes eye n the large picture from the next round on.
- `done` ends the program. It also says the task is finished, and the brain
  is asked nothing more for this task, only when the brain had seen how every
  stretch before it ended: no stretch ran before it, or each one ran inside a
  `try` or was followed by an `if` or a `repeat until` that read its ending.
  Otherwise the next round says how each stretch ended, and asks again.
- The body checks every stretch once more just before it moves, because the
  world may have changed since the program was read. A stretch refused there
  ends `refused` and moves nothing.
- A new episode stops the program before its next statement; new words from
  the person become the task from the next round on.

On the quantity keyboard, waiting is a round of `say` only (nothing moves,
and the next round brings a new picture), and remedies happen across rounds:
each round tells the brain how the last stretches ended.

## 7. Names

A name is the brain's own words. The body finds what they point at, in this
order, and never guesses:

0. A place remembered under the same letters is that place.
1. A thing this eye sees now that already goes by the same letters is that
   thing. Letters are a to z, case folded. Blanks and everything else do not
   count, so `mintgreenscissors`, `mint green scissors` and
   `MintGreen Scissors` are one name.
2. Otherwise the eye the brain looks through is asked where the name is, with
   the brain's words unchanged ([`brain-service.md`](brain-service.md),
   section 4). The body segments the box it gives. A patch that occupies the
   pixels of a thing named before is that thing, which from then on goes by
   the new words; otherwise it is a new thing. An eye that cannot point it
   out passes the question to the other eyes, in camera order. A box on the
   body itself ends the asking: parts of the body go by their role (`me`,
   `grasper`, `pusher`), never by a name, and asking other eyes would only
   collect their guesses.
3. When no eye points it out as a thing, the letters alone decide: the same
   letters as a thing named before, or the same letters once the language
   words glued to its ends are shed (the only way the decoder adds letters to
   a name), and only one thing answers to them.
4. After every name of the program had its turn, the names still unbound try
   step 3 again, so whether a name binds does not depend on its line.
5. Otherwise it is not bound, and the body says why: how each eye answered,
   that by its letters it is none of the things named before (or more than
   one, and it does not guess which), and which names were given before.

| name | is it `mint green scissors`? |
|---|---|
| `upmint green scissors` | yes: `up` is a language word glued to `mint` |
| `mint green scissorsuntilstuck` | yes: glue on the other end |
| `scissors upuntil toucheduntil` | is `scissors`: `up until touched until` are all language words |
| `pick upuntil stuckscissors` | not by its letters (`pick` is not a language word); the eye decides |
| `the pink tissue` | is not `pink tissue` by its letters (`the` is not a language word) |

`cupboard` is not `cup`, and `pencil` and `open` are not `pen`: the extra
letters are not language words. A name made only of language words
(`untildone`) names nothing. Sharing a word is not being the same
(`the red ball` is not `the red cup`), and a letter more or less is for the
eye to judge (`scisors`), never the letters.

## 8. Before anything moves

A program is read and checked whole before a single motor moves. Each check
refuses it with the line, the reason and, where there is one, a line to write
instead; the next round shows that to the brain.

1. **Reading.** A line the parser cannot read: an unknown first word, a
   stretch without `until`, `until arrived`, an effort without `press`, an
   `end` with no open block, a block left open at the end.
2. **Ending.** A loop or a behaviour that no path can ever leave. Control
   reads only endings, and any stretch may end with any ending, so the check
   follows every path with the set of endings the last stretch may have had.
   `repeat until touched:` around lines that run no stretch can never end;
   neither can a behaviour that calls itself on every path. A call of a
   behaviour that is not defined, and two behaviours under one name, are
   refused too.
3. **Names.** Every name is bound (section 7). A stretch whose name is not
   bound is refused with the account of why.
4. **The body.** Every stretch is checked against what the body measured
   about itself (Driver.Action). A stretch it cannot do is refused with what
   it tried and what it can do instead.

```refused
do the scissors height up until arrived
```

```refused
repeat until touched:
say I wait
end
```

## 9. What the next round says

The next round's WHAT HAPPENED lists every line that ran, in order:

```
line 1: do the scissors height up until free -- ended free: <what the body did>
line 2: you said: I lifted the scissors
```

It also says when the answer was cut short (and why), when the program was
refused (line, reason, what to write instead), when the brain wrote the same
program as before and the one before moved nothing, when `done` came before
the brain could see the stretches above it end, and whether anything moved.

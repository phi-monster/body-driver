# The brain service

This is how the body driver talks to the brain: what it sends, what it reads
back, and what a service has to do. The brain is your model, served by your
own inference server; the driver is one of its clients. With this page alone
you can connect a brain, or write a fake one to test the wiring
(`driver/tools/fake_brain.adb` was written from this page and nothing else).

The language the brain writes is described in [`language.md`](language.md).

## 1. Where the driver connects

```
body_driver --listen PORT --eye HOST:PORT ...
```

Every question is one HTTP/1.1 `POST` of a JSON body to
`http://HOST:PORT/v1/chat/completions`, the OpenAI chat completions shape,
one connection per question. The driver sets no time limit: the robot holds
still while the driver waits, and the driver itself closes a streamed answer
once it has read enough (section 3). Without `--eye`, every question fails
with "no address was given for the brain service" and the body only holds.

The driver asks two questions:

| question | when | streamed |
|---|---|---|
| write a program | once per round | yes |
| where is it | for a name in a program, once per eye asked | no |

Both name the model `"eye"`: serve it under that name (vLLM:
`--served-model-name eye`).

## 2. Sampling: `BL_BRAIN_SAMPLING`

The driver chooses no sampling setting. The deployment gives them as one
JSON object in the environment variable `BL_BRAIN_SAMPLING`; its members are
merged, exactly as written, into both questions, right after `"model"`. Use
the settings the model's makers recommend. For Qwen3.5 without thinking, the
box's `/root/q/run.sh` sets it to the model card's values:

```
BL_BRAIN_SAMPLING='{"temperature":0.7,"top_p":0.8,"top_k":20,"min_p":0,"presence_penalty":1.5,"repetition_penalty":1.0}'
```

Anything else the service needs per request belongs here too, for example
`"chat_template_kwargs":{"enable_thinking":false}` or `"max_tokens"`.

- Not set: nothing is merged and the service samples by its own defaults.
- Not one JSON object, or setting a member the driver writes itself
  (`model`, `messages`, `stream`, `structured_outputs`, `response_format`):
  it is not used at all.

Either way the first question logs once what happened:

```
[brain] BL_BRAIN_SAMPLING: both questions carry it as it is: {"temperature":0.7,...}
[brain] BL_BRAIN_SAMPLING: it is not set, so the service samples by its own defaults
[brain] BL_BRAIN_SAMPLING: it sets "model", which the driver writes itself, so it is not used: {...}
```

## 3. Question one: write a program

### The request

```json
{"model": "eye",
 "temperature": 0.7, "...": "the members of BL_BRAIN_SAMPLING",
 "stream": true,
 "structured_outputs": {"grammar": "<this round's keyboard, GBNF>"},
 "messages": [{"role": "user", "content": [
   {"type": "image_url", "image_url": {"url": "data:image/bmp;base64,<the picture>"}},
   {"type": "text", "text": "<the prompt>"}]}]}
```

- `structured_outputs.grammar` is this round's keyboard as a GBNF grammar
  (the `structured_outputs` member of vLLM 0.10 and later, with its xgrammar
  backend). The service must decode under it: the brain can then type only
  what this body can do now. The grammar is rebuilt every round from what the
  body measured; the prompt prints the same keyboard for the brain to read
  ([`language.md`](language.md), section 2). A service that ignores the
  grammar still works, but its programs may contain lines the driver refuses.
- The picture is one 24-bit BMP, base64 in a data URL. On top, at full size,
  the eye the brain looks through this round (the first round: the first eye
  fixed in the scene). Below it, one strip with every other eye that has a
  picture this beat, left to right in camera order, each scaled to an equal
  share of the width and keeping its own proportions. Nothing is drawn on
  any picture: drawn marks were measured to hurt the model's sight.
- The prompt has these sections, in this order:

```
You are the brain of a robot body, and I am that body. ...

THE PICTURE
The large picture is what my eye 1 sees now; it is fixed in the scene.
Below it, from left to right: eye 2 (carried by arm 1), eye 3 (carried by arm 2).

THINGS YOU HAVE NAMED
- "scissors": eye 1 sees it now; 0.052 +- 0.004 above the surface it rests on, in my own length unit
  (or: None yet. Name a thing in your own words; I ask my eyes where it is.)

WHAT HAPPENED
line 1: do the scissors height up until free -- ended free: ...
  (or how the last answer was refused or cut; the first round says it is the first round)

YOUR TASK
Pick up the scissors by 10 cm.

THE LANGUAGE THIS ROUND (the only text the decoder lets you type)
<program>  ::= <line> (<line>)*
... the keyboard, every key with its meaning ...

Write the program now, one statement per line.
```

  The prompt describes the format and never how to act: the driver's gates
  reject tutorial sentences in it.

### The answer

The answer streams as server-sent events, the OpenAI streaming shape: lines
`data: <json>`, a blank line between events, and `data: [DONE]` at the end.
From each event the driver reads `choices[0].delta.content`, appended to the
answer, and `choices[0].finish_reason` when there is one. The answer is the
program itself, one statement per line, with nothing around it.

The driver reads as the answer arrives and closes the connection as soon as
one of these holds:

| stop | when |
|---|---|
| complete | a `done` outside every block was read: nothing after it can ever run |
| ran away | the answer started a copy loop: inside one name or one `say` sentence a run of words comes again at once, or a run of whole lines does; inside a name, the language words the decoder glued into one word count as its parts (a name word that never ends is cut this way) |
| a new episode | the robot started a new episode while the brain was writing; nothing of the answer runs |

Only finished lines before the stop are kept; a line or word still being
written is never judged. A service must stop generating when the connection
closes (vLLM does; measured on the box, section 6).

When the stream ends by itself:

- `finish_reason: "stop"`: the answer is read whole.
- `finish_reason: "length"`: the service stopped at its own token limit; the
  finished lines are kept and the half-written last line is dropped.
- no connection, or a status that is not 2xx: no program; the body does not
  move, and the next round says the answer could not be read.

The program is then read, checked against the body, and run; how each line
ended is the next round's WHAT HAPPENED ([`language.md`](language.md),
section 9).

## 4. Question two: where is it

When a program names a thing that no earlier name and no letters rule
already settles, the body asks its eyes where the name is: first the eye the
brain looks through, then the others in camera order, until one boxes it.
Which pixels in the box are the thing is measured by the body itself (the
instrument segments the box).

### The request

```json
{"model": "eye",
 "temperature": 0.7, "...": "the members of BL_BRAIN_SAMPLING",
 "structured_outputs": {"grammar": "<the answer's grammar, below>"},
 "messages": [{"role": "user", "content": [
   {"type": "image_url", "image_url": {"url": "data:image/bmp;base64,<that eye's own picture>"}},
   {"type": "text", "text": "Locate what someone would call: <name>\nIf you can see it in this picture, answer with the box around it. If you cannot see it here, say so - that is a normal answer and I will look with another eye rather than guess."}]}]}
```

- The picture is that one eye's picture as it is, at its own size.
- `<name>` is the brain's words exactly as written in the program.
- The grammar admits exactly the answers the driver reads, with no blank
  anywhere and every edge a whole number from 0 to 1000:

```
root ::= "{\"found\":" ("true" | "false") ",\"bbox_2d\":[" e "," e "," e "," e "]}"
e ::= "1000" | [1-9] [0-9] [0-9] | [1-9] [0-9] | [0-9]
```

  A JSON schema would let the decoder write blanks between the members, and
  with greedy decoding Qwen3.5-9B wrote `{"found":` followed by tabs until the
  token limit; under this grammar an answer is a few dozen characters.

### The answer

`choices[0].message.content` is the JSON object
`{"found":true,"bbox_2d":[left,top,right,bottom]}`, the edges in
thousandths of the picture's width and height (0 to 1000), whatever the
picture's size. `{"found":false,"bbox_2d":[0,0,0,0]}` is a normal answer:
the body asks the next eye. A service that cannot decode under a grammar may
answer the same object with blanks; the driver reads it all the same.

| answer | the body |
|---|---|
| found, with a box that has area | segments the box and binds the name to that patch |
| found false, or a box with no area | asks the next eye |
| found, but not four edges | no answer |
| not JSON, no connection, a status that is not 2xx | no answer: no other eye is asked for this name, since the service is not answering |

A name no eye points out may still bind by its letters to a thing named
before ([`language.md`](language.md), section 7).

## 5. What the log says

Every call is one line, counted within the episode:

```
[brain] call 1 of this episode: write a program, 1.2 s, complete (line 3 is done outside every block), 3 lines kept
[brain] call 2 of this episode: where is "the scissors", 0.9 s, box from (412, 230) to (468, 301)
[brain] call 3 of this episode: where is "the cap", 0.8 s, not here (it says it cannot see it here)
[brain] call 4 of this episode: where is "the cap", 0.0 s, no answer (...)
```

The reading end is one of `ended by itself`, `complete`, `ran away`,
`stopped by the service`, `failed`. When the next episode begins:

```
[brain] the last episode called the brain 4 times (1 programs, 3 where-is-it questions)
```

Every line of the log is listed in [`log-lines.md`](log-lines.md).

## 6. Measured with Qwen3.5-9B on vLLM

On the box (vLLM serving Qwen3.5-9B as `eye` on 127.0.0.1:8078, sampling from
`QWEN_CARD`), through the driver's own client (`driver/tools/brain_measure`):

- **Reading stops in time.** 120 answers (20 tasks of five kinds, 2 each on
  three keyboards) were read to their real end with the driver's verdict
  taken on every event. The model does not stop after `done`: of the answers
  cut at a top-level done (median 1.3 s, at most 4.0 s), nearly all would
  have gone on repeating or talking to the token limit that the measurement
  added (`max_tokens` 1024, about 41 s; the box's service allows 65536).
  Answers cut as copy loops were cut at a median 2.3 s, and every one of
  them ran on to the limit: no loop was cut that would have ended by itself.
  No answer ran to the limit uncut. Reading saved 96 % of the time the
  answers would have taken, even with the limit.
- **The keyboard.** About 2000 answers on 20 tasks of five kinds: with the
  quantity keyboard, one stretch per program, 59 of 60 lifts would run
  right (heading keyboard: 56 of 60 lifts, 36 of 40 turns). Every form of
  the sentence about two things tried made the tasks about one thing worse
  on at least one keyboard, so none is offered
  ([`design/brain.md`](design/brain.md)).
- **Names.** The stretches of five recorded runs, 44 names in program order,
  bound against three eyes asked live: every name that meant the scissors
  was bound to the scissors (87 of 87 with the card's sampling, 29 of 29
  greedy). Names made of words that name nothing were bound to a thing when
  the eye boxed one for them (17 of 30), which is the eye's answer
  ([`language.md`](language.md), section 7).
- **The where-is-it answer.** Under a JSON schema, greedy decoding wrote
  blanks without end; under the grammar of section 4 every answer ends.

## 7. A fake brain for testing the wiring

`driver/bin/fake_brain PORT` is a brain service written from this page
alone. It answers question one with a stream of one `say` line, and question
two with `found: false`, so a body wired to it runs its rounds and never
moves: a brain that writes motion on its own would be a scripted program,
which the driver does not allow.

```
driver/bin/fake_brain 8090 &
driver/bin/body_driver --listen 9080 --eye 127.0.0.1:8090 --inst 127.0.0.1:8077
```

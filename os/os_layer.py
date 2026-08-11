"""THE THIN OS -- named in our docs, never built, built here.

`universal-grounding/README.md`, twice:
    "记忆 / 多步规划 【不是】"结构上没有",是【还没建】 —— 它们归薄 OS 那一层
     (与 body layer 并列,不进动作模型)"
    "记住顺序 / 多步规划 | 不归这个接口,归薄 OS —— VLM 天生就会记事,给它一层薄 OS 即可"

And the body layer's ABI confirms the gap: all nineteen `bl_*` entry points are measurement,
execution or debt (`bl_save`/`bl_load` serialise a CALIBRATION, not an episode).  Nothing in it
remembers anything that happened.

🔴 IT IS A COMPACTING CONTEXT, NOT A MEMORY STORE (owner, 2026-08-11: *"我们的记忆不是长期记忆吧,
是类似于 claude code 的上下文 compact 这种"*).  The shape is taken from `claude-code/src/services/
compact/`, which does two different things and it matters that they stay different:

  * `microCompact.ts` -- cheap, no model call: walk the OLD turns and replace their bulky payload
    with a marker (`'[Old tool result content cleared]'`), keeping the turn itself.  The SHAPE of
    the history survives; only the weight goes.
  * `compact.ts` -- one model call that rewrites the whole history into a fixed set of sections,
    then the raw history is dropped.  `autoCompact.ts` fires it on a token threshold.

Here: the "tool result" is the eye's raw reply for a frame, and the "conversation" is one episode.
So micro-compaction drops old RAW REPLIES and keeps the one-line note, and full compaction asks the
eye itself to rewrite its notes into a fixed brief.  Both are bounded; neither grows without limit.

WHAT IT IS NOT
  * not the weight -- the weight does PRIMITIVES.  Handing memory/timing to the weight was the
    mistake this file corrects.
  * not the body layer -- nothing here is a property of the robot.
  * 🔴 not a pose reader.  It sees FRAMES and its own notes.  Importing ground truth here would
    make the whole measurement circular -- an oracle wearing a memory's clothes.

WHY "NOT YET" IS THE POINT
  Measured, 5 episodes / 4 legs / 4 nouns / 4 layouts, unanimous: the noun-to-pixel interface
  answered 22.8-50.5 px from the WRONG same-category instance and 732-996 px from the right one,
  in a 640 px frame -- the referent was off-screen and the interface could not say so.  Every one
  of those answers was confident.  An interface that cannot abstain turns "I cannot see it" into
  "it is over there", and no model quality fixes that.
"""
import json
import os
import time
import urllib.parse
import urllib.request

import numpy as np

URL = os.environ.get("L3_EYE_URL", "http://127.0.0.1:48611")
TIMEOUT = float(os.environ.get("L3_EYE_TIMEOUT", "600"))
KEEP_RAW = int(os.environ.get("L3_OS_KEEP_RAW", "3"))       # microcompact: turns keeping raw text
COMPACT_AT = int(os.environ.get("L3_OS_COMPACT_AT", "10"))  # autocompact: turns before rewriting
CLEARED = "[old detail cleared]"
NOT_YET = "NOT_YET"


class EyeUnavailable(RuntimeError):
    """Could not reach the eye.  Never a pixel, never a silent fallback."""


def _ask(rgb, prompt, maxtok=None):
    a = np.ascontiguousarray(np.asarray(rgb, np.uint8)[..., :3])
    h, w = int(a.shape[0]), int(a.shape[1])
    req = urllib.request.Request(
        URL + "/ask", data=a.tobytes(), method="POST",
        headers={"Content-Type": "application/octet-stream", "X-H": str(h), "X-W": str(w),
                 "X-Prompt": urllib.parse.quote(prompt),
                 **({"X-Maxtok": str(int(maxtok))} if maxtok else {})})
    try:
        with urllib.request.urlopen(req, timeout=TIMEOUT) as r:
            d = json.loads(r.read().decode())
    except Exception as e:
        raise EyeUnavailable("eye /ask failed (%s: %s)" % (type(e).__name__, e))
    if d.get("error"):
        raise EyeUnavailable("eye /ask error: %s" % d["error"])
    if "xyz" in d:
        raise EyeUnavailable("eye reply carries `xyz` -- that is the privileged pointer's key")
    return d


class ThinOS:
    """One episode's compacting context.

    `observe(rgb, k)` returns one of
        {"ready": False, "why": "not_yet" | "unparsed" | "no_point", ...}
        {"ready": True,  "uv": [u, v], ...}
    """

    def __init__(self, task_sentence, log=None):
        self.task = task_sentence
        self.turns = []          # [{k, note, status, raw}] -- raw is cleared as it ages
        self.brief = None        # the compacted summary that replaces dropped turns
        self.calls, self.compactions, self.ms = 0, 0, 0.0
        self.log = log if log is not None else []

    # ---- microcompact: cheap, no model call ------------------------------------------------
    def _microcompact(self):
        """Old turns keep their one-line note and lose their raw reply.  The shape survives."""
        for t in self.turns[:-KEEP_RAW]:
            if t.get("raw") and t["raw"] != CLEARED:
                t["raw"] = CLEARED

    # ---- autocompact: one model call, fixed sections ---------------------------------------
    def _compact(self, rgb):
        """Rewrite the notes into a fixed brief, then drop them.  Bounded by construction.

        The section list is deliberately short and task-shaped -- the equivalent of
        `getCompactPrompt`'s numbered sections, minus everything that only makes sense for a
        coding conversation.
        """
        hist = "\n".join("  frame %d: %s" % (t["k"], t["note"] or "(no note)") for t in self.turns)
        p = ("You have been watching a scene one frame at a time and taking a note on each frame.\n"
             "TASK (the robot's own instruction): %s\n\n"
             "%sYour notes so far:\n%s\n\n"
             "Rewrite them into a brief you could act on if you forgot everything else. "
             "Reply as JSON on one line, nothing else:\n"
             '  {"seen_in_order": "<the objects that have gone past, in the order they went past>",'
             ' "target_description": "<what the object the task wants looks like, as specifically'
             ' as you can>", "already_happened": "<what has already been done or missed>",'
             ' "watch_for": "<what must happen next for the task to succeed>"}'
             % (self.task,
                ("Your previous brief:\n%s\n\n" % json.dumps(self.brief)) if self.brief else "",
                hist))
        d = _ask(rgb, p, maxtok=320)
        self.calls += 1
        self.ms += float(d.get("ms") or 0.0)
        raw = (d.get("raw") or "").strip()
        try:
            self.brief = json.loads(raw[raw.index("{"):raw.rindex("}") + 1])
        except Exception:
            # 🔴 A compaction that could not be parsed must NOT silently drop the history it was
            # meant to replace -- that would lose the memory and look like the eye forgot.
            self.log.append({"k": -1, "event": "compact_unparsed", "raw": raw[:200]})
            return False
        self.compactions += 1
        self.log.append({"k": -1, "event": "compacted", "brief": self.brief,
                         "turns_dropped": len(self.turns)})
        self.turns = []
        return True

    # ---- the context handed to the eye each frame ------------------------------------------
    def _context(self):
        if self.brief is None and not self.turns:
            return "This is the first frame you have seen. You have no notes yet."
        out = []
        if self.brief:
            out.append("Brief carried over from earlier frames:\n" + json.dumps(self.brief))
        if self.turns:
            out.append("Recent frames, oldest first:\n" + "\n".join(
                "  frame %d: %s%s" % (t["k"], t["note"] or "(no note)",
                                      "" if t["raw"] != CLEARED else "  " + CLEARED)
                for t in self.turns))
        return "\n\n".join(out)

    def _prompt(self):
        # Written once, and it names no object: the task sentence is the benchmark's own and
        # everything else is about the context.  Nothing here can smuggle in which instance is
        # correct -- that is the thing being tested.
        return (
            "You are the eye of a robot watching a scene, one frame at a time.\n"
            "TASK (the robot's own instruction): %s\n\n"
            "%s\n\n"
            "Answer for THIS frame only, as JSON on one line, and nothing else:\n"
            '  {"note": "<one short sentence recording what matters for the task in this frame>",'
            ' "status": "%s" or "TARGET_VISIBLE", "point_2d": [x, y]}\n'
            "Rules:\n"
            '  - Give "point_2d" ONLY when status is TARGET_VISIBLE. Use the [0,1000] normalised'
            " convention.\n"
            '  - If the object the task wants is not in THIS frame, status MUST be %s and you MUST'
            " NOT give a point. Never substitute a similar-looking object.\n"
            '  - Your "note" is all you will remember of this frame, so record what you would need'
            " later -- which objects went past, and in what order."
            % (self.task, self._context(), NOT_YET, NOT_YET))

    # ---- one look --------------------------------------------------------------------------
    def observe(self, rgb, k):
        t0 = time.time()
        _a = np.asarray(rgb)
        FH, FW = int(_a.shape[0]), int(_a.shape[1])
        if len(self.turns) >= COMPACT_AT:
            self._compact(rgb)
        d = _ask(rgb, self._prompt())
        self.calls += 1
        self.ms += float(d.get("ms") or 0.0)
        raw = (d.get("raw") or "").strip()
        # 🔴 THE SERVER'S PARSER IS NOT THE OS'S DECISION.  `/ask` runs `rd_eye._parse_point` on
        # whatever came back, so a reply that says NOT_YET and happens to contain a bracketed pair
        # still arrives with a `uv` -- the smoke test produced exactly that: status=NOT_YET with
        # uv=[0.0, 0.0].  A point that accompanies an abstention is not a point; taking it would
        # reintroduce the very failure this layer exists to remove.  The status decides.
        note, status, uv = None, None, None
        _server_uv = d.get("uv")
        try:
            o = json.loads(raw[raw.index("{"):raw.rindex("}") + 1])
            note = str(o.get("note") or "")[:200]
            status = str(o.get("status") or "").strip().upper()
            if status == "TARGET_VISIBLE" and _server_uv is not None:
                uv = _server_uv
            if o.get("point_2d") and uv is None and status == "TARGET_VISIBLE":
                # 🔴 UNITS.  The server returns `uv` in PIXELS; this fallback parses the model's
                # own [0,1000] convention and must convert, or the two paths disagree by ~640x and
                # the mix reads as a servo fault.
                p = [float(x) for x in o["point_2d"]][:2]
                uv = [p[0] / 1000.0 * FW, p[1] / 1000.0 * FH]
        except Exception:
            pass
        self.turns.append({"k": k, "note": note, "status": status, "raw": raw[:400]})
        self._microcompact()
        rec = {"k": k, "status": status, "note": note, "uv": uv,
               "server_uv_ignored": (None if status == "TARGET_VISIBLE" or _server_uv is None
                                     else _server_uv),
               "ms": round(float(d.get("ms") or 0)), "wall_s": round(time.time() - t0, 1)}
        self.log.append(rec)
        # 🔴 An unreadable reply is NOT a "not yet".  One says the world is not ready, the other
        # says the instrument is not, and they have opposite next moves.  Sharing a counter is the
        # mistake `eye_server` already paid for once (79 unasked episodes read as eye failures).
        if status == NOT_YET:
            return {"ready": False, "why": "not_yet", **rec}
        if status == "TARGET_VISIBLE" and uv is not None:
            return {"ready": True, "uv": [float(uv[0]), float(uv[1])], **rec}
        return {"ready": False, "why": "no_point" if status else "unparsed", **rec}

    def summary(self):
        return {"os_calls": self.calls, "os_ms": round(self.ms), "os_compactions": self.compactions,
                "os_turns_live": len(self.turns), "os_brief": self.brief, "os_log": self.log[:60]}


# ============================================================================================
# SLOT MEMORY -- the second strategy, and the one this layer recommends.
# ============================================================================================
# `ThinOS` above copies Claude Code: keep notes, and when they pile up spend one model call
# rewriting them.  That is right when the reason to compress is SIZE.  Ours is FRESHNESS, and three
# things follow that the notes-plus-summary shape handles badly:
#
#   1. the compaction call costs one full 5 s inference DURING WHICH THE ROBOT IS BLIND.  On a
#      10-40 frame episode that is a large fraction of the episode spent not looking.
#   2. a durable fact ("the first object was a green cup") survives only if the summariser happens
#      to carry it.  Nothing names it, so nothing guarantees it -- and our forgetting is
#      unrecoverable, because the object has left the frame and will not come back.
#   3. a perishable fact ("it is at the left edge") CAN be written into a note, and once written it
#      is indistinguishable from a durable one.  Driving on a stale position is then a discipline
#      problem, and discipline is what fails at 3am.
#
# So: a small set of NAMED slots the eye rewrites every frame, alongside its status and point.
#   * no separate compaction call -- the rewrite is free, it rides the observation
#   * bounded by construction -- the state is the slots, and there are five of them
#   * durable facts are pinned BY NAME, not by a summariser's judgement
#   * 🔴 there is NO SLOT for "where it is now".  A position exists only as `point_2d` for THIS
#     frame, consumed immediately and never stored.  Driving on a stale position stops being a
#     rule and becomes unrepresentable.
#
# Cost, stated because it is real: fixed slots are less general than prose.  A task needing a fact
# we did not name has nowhere to put it -- which is what `other` is for, and `other` is the seam to
# watch if this ever underperforms the notes version.
#
# Both classes expose the same `observe()` contract, so a caller can A/B them on identical episodes
# rather than argue about which is better.

PHASES = ("observing", "waiting", "target_visible", "done")


class SlotOS:
    """Named slots, rewritten every frame.  Same contract as `ThinOS.observe`."""

    def __init__(self, task_sentence, log=None):
        self.task = task_sentence
        self.slots = {"first_object": "", "phase": "observing", "already_done": "", "other": ""}
        self.pinned = set()
        self._first_seen_at = None
        self.changes = []        # every slot write, with the frame it happened on
        self.calls, self.ms = 0, 0.0
        self.log = log if log is not None else []

    # -- what the eye is allowed to change ---------------------------------------------------
    def _apply(self, new, k):
        """Write the slots the model returned.  Pinned slots refuse silently-loud: the attempt is
        recorded, the value is not taken.

        `first_object` pins the moment the phase leaves `observing` -- before that the eye may
        still be refining what it saw, after that the object is gone and any 'correction' is a
        later frame overwriting a memory it can no longer check.  That is the one overwrite this
        layer exists to prevent.
        """
        for key in ("first_object", "phase", "already_done", "other"):
            v = str(new.get(key) or "").strip()
            if not v or v == self.slots.get(key):
                continue
            if key in self.pinned:
                self.changes.append({"k": k, "slot": key, "REFUSED": v[:80],
                                     "kept": self.slots[key][:80]})
                continue
            if key == "phase" and v not in PHASES:
                self.changes.append({"k": k, "slot": key, "REJECTED_not_a_phase": v[:40]})
                continue
            self.changes.append({"k": k, "slot": key, "from": self.slots[key][:60], "to": v[:80]})
            self.slots[key] = v
        # 🔴 THE PIN MUST NOT DEPEND ON THE MODEL DOING ANYTHING.  The first version pinned when
        # `phase` left "observing" -- and the smoke test showed the model never updates `phase`,
        # so the pin never engaged and the protection was decorative.  A guard that only fires
        # when the thing it guards against cooperates is not a guard.
        # Mechanical rule instead: once `first_object` has a value, it pins on the NEXT frame.
        # By then the eye has seen a different picture, "the first one" is history, and any later
        # correction is overwriting a memory it can no longer check against the world.
        if self.slots["first_object"]:
            if self._first_seen_at is None:
                self._first_seen_at = k
            elif k > self._first_seen_at:
                self.pinned.add("first_object")

    def _prompt(self):
        return (
            "You are the eye of a robot watching a scene, one frame at a time.\n"
            "TASK (the robot's own instruction): %s\n\n"
            "This is your entire memory. You rewrite it every frame:\n%s\n\n"
            "Answer for THIS frame only, as JSON on one line, nothing else:\n"
            '  {"first_object": "...", "phase": "observing|waiting|target_visible|done",'
            ' "already_done": "...", "other": "...",'
            ' "status": "%s" or "TARGET_VISIBLE", "point_2d": [x, y]}\n'
            "Rules:\n"
            "  - The four memory fields are what you will still know next frame. Repeat a field"
            " unchanged to keep it; write a new value to change it.\n"
            "  - Do NOT record where anything is right now. You will see the next frame; positions"
            " you write down will be wrong by the time you read them.\n"
            '  - Give "point_2d" ONLY when status is TARGET_VISIBLE, in the [0,1000] normalised'
            " convention. If the object the task wants is not in THIS frame, status MUST be %s,"
            " and never substitute a similar-looking object.\n"
            '  - "first_object" is what you saw FIRST and may need to recognise again later.'
            " Describe it so you could pick it out among lookalikes."
            % (self.task, json.dumps(self.slots, ensure_ascii=False), NOT_YET, NOT_YET))

    def observe(self, rgb, k):
        t0 = time.time()
        _a = np.asarray(rgb)
        FH, FW = int(_a.shape[0]), int(_a.shape[1])
        d = _ask(rgb, self._prompt())
        self.calls += 1
        self.ms += float(d.get("ms") or 0.0)
        raw = (d.get("raw") or "").strip()
        # Same weld as `ThinOS`: the server parses a point out of ANY reply, so an abstention can
        # arrive carrying one.  The status decides; the discarded point is kept, not dropped.
        status, uv, _server_uv, parsed = None, None, d.get("uv"), None
        try:
            parsed = json.loads(raw[raw.index("{"):raw.rindex("}") + 1])
            status = str(parsed.get("status") or "").strip().upper()
            if status in ("TARGET_VISIBLE", "RECORD_THIS"):
                uv = _server_uv
                if uv is None and parsed.get("point_2d"):
                    p = [float(x) for x in parsed["point_2d"]][:2]
                    uv = [p[0] / 1000.0 * FW, p[1] / 1000.0 * FH]
        except Exception:
            parsed = None
        n_before = len(self.changes)
        if parsed:
            self._apply(parsed, k)
        rec = {"k": k, "status": status, "uv": uv,
               "server_uv_ignored": (None if status == "TARGET_VISIBLE" or _server_uv is None
                                     else _server_uv),
               "slots": dict(self.slots), "slot_writes": self.changes[n_before:],
               "ms": round(float(d.get("ms") or 0)), "wall_s": round(time.time() - t0, 1)}
        self.log.append(rec)
        if status == NOT_YET:
            return {"ready": False, "why": "not_yet", **rec}
        if status == "TARGET_VISIBLE" and uv is not None:
            return {"ready": True, "uv": [float(uv[0]), float(uv[1])], **rec}
        return {"ready": False, "why": "no_point" if status else "unparsed", **rec}

    def summary(self):
        return {"os_kind": "slot", "os_calls": self.calls, "os_ms": round(self.ms),
                "os_slots": dict(self.slots), "os_pinned": sorted(self.pinned),
                "os_changes": self.changes[:60], "os_log": self.log[:60]}


# ============================================================================================
# PATCH MEMORY -- derived from the way the first two strategies failed, not invented.
# ============================================================================================
# `notes` and `slot` fail at OPPOSITE ends of one axis, measured on identical layouts:
#
#     notes   abstained  0 / 24 looks   -- always acted, always on `target0`, dead by look 4-5
#     slot    abstained 45 / 45 looks   -- never acted at all, ran the budget out
#
# Two extremes of the same knob is a sign the knob is wrong, not that one setting is. What both
# store is a SENTENCE about the first object ("a red square"). A sentence is a lossy encoding of
# "is this the same thing I saw before": unpinned it matches anything, pinned it matches nothing.
# The failure was not a prompt to tune -- it is what text can carry.
#
# A person remembers what it LOOKED like. So: keep the actual crop of the first object and show it
# to the eye beside the current frame, and the question stops being text-vs-image and becomes
# image-vs-image.
#
# Composited rather than sent as a second image, so `/ask` needs no change: the reply is a point in
# COMPOSITE coordinates and the strip height is subtracted to get frame coordinates. A point that
# lands INSIDE the memory strip means the eye pointed at the souvenir instead of the scene -- that
# is recorded as `pointed_at_memory`, never quietly shifted into the frame.

MEM_STRIP = int(os.environ.get("L3_OS_STRIP_PX", "96"))
MEM_CROP = int(os.environ.get("L3_OS_CROP_PX", "96"))


class PatchOS:
    """Memory is a picture, not a sentence.  Same `observe()` contract as the other two."""

    def __init__(self, task_sentence, log=None):
        self.task = task_sentence
        self.patch = None            # the remembered crop, HxWx3 uint8
        self.patch_at = None         # the frame it was taken from
        self.slots = {"phase": "observing", "already_done": "", "other": ""}
        self.calls, self.ms = 0, 0.0
        self.log = log if log is not None else []

    def _crop(self, rgb, uv):
        a = np.asarray(rgb, np.uint8)
        H, W = a.shape[:2]
        u, v = int(round(uv[0])), int(round(uv[1]))
        h = MEM_CROP // 2
        u0, v0 = max(0, min(W - MEM_CROP, u - h)), max(0, min(H - MEM_CROP, v - h))
        return np.ascontiguousarray(a[v0:v0 + MEM_CROP, u0:u0 + MEM_CROP, :3])

    def _composite(self, rgb):
        """[memory strip on top ; current frame below].  Returns (image, strip_height)."""
        a = np.asarray(rgb, np.uint8)
        H, W = a.shape[:2]
        if self.patch is None:
            return a, 0
        strip = np.zeros((MEM_STRIP, W, 3), np.uint8)
        ph, pw = self.patch.shape[:2]
        sh = min(MEM_STRIP, ph)
        strip[:sh, :min(W, pw)] = self.patch[:sh, :min(W, pw)]
        strip[:, min(W, pw) + 2:] = 40          # a flat field, so the strip reads as an inset
        return np.ascontiguousarray(np.concatenate([strip, a], axis=0)), MEM_STRIP

    def _prompt(self, has_patch):
        head = ("You are the eye of a robot watching a scene, one frame at a time.\n"
                "TASK (the robot's own instruction): %s\n\n" % self.task)
        if has_patch:
            head += ("The TOP %d pixels of this image are not part of the scene. They are a "
                     "PICTURE OF THE OBJECT YOU SAW FIRST, kept as your memory of it. Everything "
                     "below that band is the scene right now.\n"
                     "Compare them: is the object in the top band present in the scene below?\n\n"
                     % MEM_STRIP)
        else:
            head += ("You have NOT yet recorded which object came first. Point at the object the "
                     "task tells you to remember, so a picture of it can be kept.\n\n")
        # 🔴 ONE VOCABULARY PER QUESTION.  The first version offered `TARGET_VISIBLE` in BOTH
        # phases, so to record the first object the eye had to call it "the target" -- while the
        # task says the target is the one that appears AGAIN.  It was being asked to say something
        # the instruction contradicts, and it refused: 35 looks, 35 NOT_YET, the memory never
        # created.  Same disease as an interface with no way to abstain -- the right answer was not
        # in the answer set.  So the two phases get different words.
        act = ("RECORD_THIS" if not has_patch else "TARGET_VISIBLE")
        what = ("this is the object to remember (it is NOT the one to pick)" if not has_patch
                else "the object in the memory band is present in the scene below")
        return head + (
            "Memory fields you rewrite each frame: %s\n\n"
            "Answer for THIS frame only, as JSON on one line, nothing else:\n"
            '  {"phase": "observing|waiting|target_visible|done", "already_done": "...",'
            ' "other": "...", "status": "%s" or "%s", "point_2d": [x, y]}\n'
            "Rules:\n"
            '  - "%s" means: %s. Give "point_2d" only with that status, in the [0,1000] normalised'
            " convention, and it must be in the SCENE, never in the memory band.\n"
            "  - If that is not true of THIS frame, status MUST be %s. Never substitute a different"
            " object that merely looks similar in kind.\n"
            "  - Do not record where anything is; you will see the next frame."
            % (json.dumps(self.slots, ensure_ascii=False), NOT_YET, act, act, what, NOT_YET))

    def observe(self, rgb, k):
        t0 = time.time()
        FH, FW = int(np.asarray(rgb).shape[0]), int(np.asarray(rgb).shape[1])
        img, strip = self._composite(rgb)
        CH = img.shape[0]
        d = _ask(img, self._prompt(self.patch is not None))
        self.calls += 1
        self.ms += float(d.get("ms") or 0.0)
        raw = (d.get("raw") or "").strip()
        status, uv_c, in_memory = None, None, False
        try:
            o = json.loads(raw[raw.index("{"):raw.rindex("}") + 1])
            status = str(o.get("status") or "").strip().upper()
            for key in ("phase", "already_done", "other"):
                v = str(o.get(key) or "").strip()
                if v:
                    self.slots[key] = v
            if status == "TARGET_VISIBLE":
                if d.get("uv") is not None:
                    uv_c = list(d["uv"])
                elif o.get("point_2d"):
                    p = [float(x) for x in o["point_2d"]][:2]
                    uv_c = [p[0] / 1000.0 * FW, p[1] / 1000.0 * CH]
        except Exception:
            pass
        uv = None
        if uv_c is not None:
            if uv_c[1] < strip:
                in_memory = True          # pointed at the souvenir, not the scene
            else:
                uv = [uv_c[0], uv_c[1] - strip]
        # first sighting: keep the picture, not a description of it
        if self.patch is None and uv is not None:
            self.patch, self.patch_at = self._crop(rgb, uv), k
        rec = {"k": k, "status": status, "uv": uv, "pointed_at_memory": in_memory,
               "has_patch": self.patch is not None, "patch_at": self.patch_at,
               "slots": dict(self.slots), "ms": round(float(d.get("ms") or 0)),
               "wall_s": round(time.time() - t0, 1)}
        self.log.append(rec)
        if in_memory:
            return {"ready": False, "why": "pointed_at_memory", **rec}
        if status == NOT_YET:
            return {"ready": False, "why": "not_yet", **rec}
        if status == "RECORD_THIS":
            # The frame that CREATES the memory is not a sighting of the match.  Acting on it is
            # exactly `notes`'s failure (grab the first thing you see).  The point is consumed by
            # the memory and never returned -- now enforced by the status itself, not by a
            # timestamp comparison.
            return {"ready": False, "why": "recorded_first_object", **rec}
        if status == "TARGET_VISIBLE" and uv is not None:
            if self.patch_at == k:
                return {"ready": False, "why": "recorded_first_object", **rec}
            return {"ready": True, "uv": uv, **rec}
        return {"ready": False, "why": "no_point" if status else "unparsed", **rec}

    def summary(self):
        return {"os_kind": "patch", "os_calls": self.calls, "os_ms": round(self.ms),
                "os_patch_at": self.patch_at, "os_slots": dict(self.slots),
                "os_log": self.log[:60]}


def make_os(task_sentence, kind=None, log=None):
    """`L3_OS_KIND=patch|slot|notes`.  Default patch -- the other two failed at opposite ends of
    the text-description axis (0/24 and 45/45 abstentions on identical layouts)."""
    kind = (kind or os.environ.get("L3_OS_KIND", "patch")).strip().lower()
    return {"notes": ThinOS, "slot": SlotOS, "patch": PatchOS}[kind](task_sentence, log=log)

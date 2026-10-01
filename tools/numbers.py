#!/usr/bin/env python3
"""Every numeric literal in the driver must have a stated origin, and none may be a tuning number.

Each literal in driver/src (strings included) is either structural by rule (zero, powers, small
component indexes, the 1 in X + 1, X - 1 and 1 .. N, 1.0 in 1.0 - x and in unit axis vectors) or
is listed in driver/numbers.tsv, one row per occurrence, with one of these categories:

  structure   indexes, component counts, residual rows per point, field positions in a format
  math        numbers in identities (one half, twice, squares, pi, quaternion formulas)
  numerics    numbers of an algorithm itself (iteration caps, convergence tolerances, guards
              against division by zero); they describe neither a body nor a scene
  statistics  statistical conversions (MAD to sigma) and confidence levels (Z in Conventions)
  format      protocol, file and image formats, unit conversions, digits printed in a log

A hand-picked number that describes a body, a scene or a behavior (a step multiplier, a retry
count, a minimum number of samples) is a tuning number and is not allowed: replace it with a
measured quantity. Unlisted literals, unclassified rows and any tuning row make the check fail.

The classification rules were hardened by two audits of the previous driver; TEETH below are the
holes those audits found, and the check refuses to run if any of them opens again.

Usage: numbers.py check | numbers.py gen (append unlisted literals as undecided rows and drop
rows whose literal is gone) | numbers.py selftest
"""
import re, sys, os, glob, collections
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REG = os.path.join(ROOT, "driver", "numbers.tsv")
CATS = {"structure", "math", "numerics", "statistics", "format"}
HEADER = "# file\tline (comments and strings removed, blanks normalized)\tliteral (a \" prefix: inside a string)\tcategory\twhy\n"
NUM = re.compile(r"(?<![A-Za-z_0-9#])(?<![0-9]\.)(?:[0-9][0-9_]*#[0-9A-Fa-f_.]+#(?:[eE][-+]?[0-9]+)?|[0-9][0-9_]*(?:\.[0-9][0-9_]*)?(?:[eE][-+]?[0-9][0-9_]*)?)(?![A-Za-z_0-9#])")
SNUM = re.compile(r"(?<![A-Za-z_0-9])(?<![0-9]\.)[0-9]+(?:\.[0-9]+)?(?:[eE][-+]?[0-9]+)?")
KEYWORDS = set("""abort abs abstract accept access aliased all and array at begin body case constant declare delay delta digits do else elsif end
entry exception exit for function generic goto if in interface is limited loop mod new not null of or others out overriding package pragma private
procedure protected raise range record rem renames requeue return reverse select separate some subtype synchronized tagged task terminate then type
until use when while with xor""".split())
STD_CALLABLE = set("""Long_Float Float Integer Natural Positive Boolean Character String Duration Long_Integer Long_Long_Integer Short_Integer
Unsigned_8 Unsigned_16 Unsigned_32 Unsigned_64 Integer_8 Integer_16 Integer_32 Integer_64 Stream_Element Stream_Element_Offset Stream_Element_Array
Sqrt Sin Cos Tan Cot Arctan Arcsin Arccos Arccot Exp Log Sinh Cosh Tanh Put Put_Line Get Get_Line Append Prepend Insert Delete Replace_Element Element
To_Vector To_String To_Unbounded_String Set_Length Reserve_Capacity Slice Head Tail Index Trim Shift_Left Shift_Right Rotate_Left Rotate_Right
Unchecked_Conversion Unchecked_Deallocation Clock Seconds Milliseconds Microseconds Argument Value Image Floor Ceiling Rounding Truncation""".split())

def split_code(raw):
    """Split a line into its code (strings emptied, comment removed) and the contents of its strings."""
    out = []; strs = []; i = 0; n = len(raw)
    while i < n:
        ch = raw[i]
        if ch == '"':
            j = i + 1; cur = []
            while j < n:
                if raw[j] == '"':
                    if j + 1 < n and raw[j + 1] == '"':
                        cur.append('"'); j += 2; continue
                    break
                cur.append(raw[j]); j += 1
            strs.append("".join(cur)); out.append('""'); i = j + 1; continue
        if ch == "'" and i + 2 < n and raw[i + 2] == "'" and not (i > 0 and (raw[i - 1].isalnum() or raw[i - 1] in "_)")):
            out.append("''"); i += 3; continue
        if raw.startswith("--", i):
            break
        out.append(ch); i += 1
    return "".join(out), strs

PACKAGES = {"Ada", "Interfaces", "GNAT", "System"}

def names_in(path):
    call = set(); obj = set()
    offline = os.path.basename(path) in OFFLINE or os.path.basename(path).split("-")[0] + ".adb" in OFFLINE
    for raw in open(path, encoding="utf-8").read().split("\n"):
        c, _ = split_code(raw)
        if not offline:
            for m in re.finditer(r"\bwith\s+([A-Za-z_][A-Za-z_0-9.]*(?:\s*,\s*[A-Za-z_][A-Za-z_0-9.]*)*)\s*;", c):
                for nm in m.group(1).split(","):
                    PACKAGES.add(nm.strip().split(".")[0])
            for m in re.finditer(r"\bpackage\s+(?:body\s+)?([A-Za-z_][A-Za-z_0-9.]*)", c):
                for nm in m.group(1).split("."):
                    PACKAGES.add(nm)
        for m in re.finditer(r"\b(?:function|procedure|entry|type|subtype|package|task|protected)\s+(?:body\s+)?([A-Za-z_][A-Za-z_0-9]*)", c):
            call.add(m.group(1))
        for m in re.finditer(r"(?:^|[;(])\s*([A-Za-z_][A-Za-z_0-9]*(?:\s*,\s*[A-Za-z_][A-Za-z_0-9]*)*)\s*:(?!=)", c):
            for nm in m.group(1).split(","):
                obj.add(nm.strip())
        for m in re.finditer(r"\bfor\s+([A-Za-z_][A-Za-z_0-9]*)\s+(?:in|of)\b", c):
            obj.add(m.group(1))
    return call, obj

def declared_names():
    """See the module docstring."""
    per = {}
    pub_call = set(STD_CALLABLE); pub_obj = set()
    files = driver_files()
    for f in files:
        per[f] = names_in(f)
        if f.endswith(".ads"):
            pub_call |= per[f][0]; pub_obj |= per[f][1]
    def root_of(f):
        seen = set()
        while f not in seen:
            seen.add(f)
            head = ""
            for raw in open(f, encoding="utf-8"):
                c, _ = split_code(raw)
                if c.strip():
                    head = c.strip(); break
            m = re.match(r"separate\s*\(\s*([A-Za-z_][A-Za-z_0-9.]*)\s*\)", head)
            if not m:
                return f
            f = os.path.join(os.path.dirname(f), m.group(1).lower().replace(".", "-") + ".adb")
        return f
    family = collections.defaultdict(list)
    for f in files:
        if f.endswith(".adb"):
            family[root_of(f)].append(f)
    scope = {}
    for f in files:
        members = family.get(root_of(f), [f]) if f.endswith(".adb") else [f]
        lc, lo = set(), set()
        for g in members + [f]:
            lc |= per[g][0]; lo |= per[g][1]
        spec = (root_of(f) if f.endswith(".adb") else f)[:-1] + "s"
        if f.endswith(".adb") and spec in per:
            lc |= per[spec][0]; lo |= per[spec][1]
        scope[os.path.basename(f)] = (lc, lo, pub_call, pub_obj)
    return scope

SCOPE = {}
CUR = [""]


def opener(c, pos):
    d = 0
    for i in range(pos - 1, -1, -1):
        ch = c[i]
        if ch in ")]": d += 1
        elif ch in "([":
            if d == 0: return ch, i
            d -= 1
    return None, -1

def closer(c, i):
    d = 0
    for j in range(i, len(c)):
        if c[j] in "([": d += 1
        elif c[j] in ")]":
            d -= 1
            if d == 0: return j
    return len(c)

def parts(s):
    out = []; d = 0; cur = []
    for ch in s:
        if ch in "([": d += 1
        elif ch in ")]": d -= 1
        if ch == "," and d == 0:
            out.append("".join(cur).strip()); cur = []
        else:
            cur.append(ch)
    out.append("".join(cur).strip())
    return out

def head_of(c, i):
    """See the module docstring."""
    b = c[:i].rstrip()
    if b.endswith(")"):
        return "index"
    m = re.search(r"([A-Za-z_][A-Za-z_0-9]*)\s*'\s*([A-Za-z_]+)$", b)
    if m:
        return "attr:" + m.group(2)
    m = re.search(r"((?:[A-Za-z_][A-Za-z_0-9]*\s*\.\s*)*)([A-Za-z_][A-Za-z_0-9]*)$", b)
    if not m:
        return "none"
    nm = m.group(2)
    if nm.lower() in KEYWORDS:
        return "none"
    pre = [x for x in re.split(r"\s*\.\s*", m.group(1)) if x]
    if pre and pre[0] in PACKAGES:
        return "call"
    lc, lo, pc, po = SCOPE[CUR[0]]
    if nm in lc: return "call"
    if nm in lo: return "index"
    if nm in pc: return "call"
    if nm in po: return "index"
    return "call"

LINES = []
LN = [0]

def loop_indexes(var):
    """See the module docstring."""
    depth = 0
    for j in range(LN[0], min(len(LINES), LN[0] + 400)):
        c, _ = split_code(LINES[j])
        depth += len(re.findall(r"\bloop\b", c)) - 2 * len(re.findall(r"\bend\s+loop\b", c))
        for m in re.finditer(r"\(\s*(?:[A-Za-z_][A-Za-z_0-9]*\s*,\s*)*" + var + r"\s*[,)]", c):
            if head_of(c, m.start()) == "index":
                return True
        if j > LN[0] and depth <= 0:
            break
    return False

def is_int(v):
    return re.fullmatch(r"[0-9][0-9_]*", v) is not None

def ival(v):
    return int(v.replace("_", ""))

def structural(v, c, s, e):
    before = c[:s]; after = c[e:]
    if re.fullmatch(r"0[0_]*(?:\.0[0_]*)?(?:[eE][-+]?[0-9]+)?", v):
        op0, i0 = opener(c, s)
        if op0 == "(" and head_of(c, i0) == "call" and re.search(r"[(,]\s*$", before) and re.match(r"\s*[),]", after) and "." in v:
            return False
        if re.search(r"\bconstant\b[^;]*:=\s*-?\s*$", before):
            return False
        return True
    if re.search(r"\*\*\s*$", before):
        return True
    op, i = opener(c, s)
    head = head_of(c, i) if op == "(" else "none"
    inner = c[i + 1:closer(c, i)] if op else ""
    if is_int(v):
        n = ival(v)
        if head.startswith("attr:") and head[5:] in ("First", "Last", "Range", "Length") and inner.strip() == v and n <= 3:
            return True                                                      # A'Range (2): which dimension
        direct = re.search(r"[(,]\s*$", before) and re.match(r"\s*[),]", after)
        if head == "index":
            if direct:
                return n <= 8
            if re.search(r"\.\.\s*$", before) or re.match(r"\s*\.\.", after):
                return n <= 2
            return n == 1
        if n == 1:
            m1 = re.search(r"([A-Za-z_][A-Za-z_0-9]*|[0-9.]+|[)\]])\s*[-+]\s*$", before)
            if m1 and m1.group(1).lower() not in KEYWORDS:
                return True
            if re.match(r"\s*[-+](?!\s*[0-9])", after) and not re.search(r"[-+*/]\s*$", before):
                return True                                                  # 1 + X
            if re.match(r"\s*\.\.", after) and not re.search(r"-\s*$", before):
                return True
            return False
        if re.search(r"\b0\s*\.\.\s*$", before) and n <= 2:
            m = re.search(r"\bfor\s+([A-Za-z_][A-Za-z_0-9]*)\s+in\s+(?:reverse\s+)?0\s*\.\.\s*$", before)
            if not m:
                return op == "(" and head == "none" or bool(re.search(r"\brange\s+0\s*\.\.\s*$", before))
            return loop_indexes(m.group(1))
        return False
    if re.fullmatch(r"1\.0+", v):
        if re.match(r"\s*-(?!-)", after) and not re.search(r"[*/]\s*$", before):
            return True                                                      # 1.0 - x
        if (op == "[" or (op == "(" and head == "none")) and not re.search(r"\bconstant\b", c):
            ps = parts(inner)
            if len(ps) >= 2 and all(re.fullmatch(r"-?\s*[01](?:\.0+)?", p) for p in ps):
                return True
        return False
    return False

def driver_files():
    """Driver sources: everything under driver/src except the self-test packages (*-tests.ad?)."""
    return [f for f in glob.glob(os.path.join(ROOT, "driver/src/**/*.ad[sb]"), recursive=True)
            if not re.search(r"-tests\.ad[sb]$", f)]

def offline_mains():
    """See the module docstring."""
    g = open(os.path.join(ROOT, "driver", "driver.gpr"), encoding="utf-8").read()
    m = re.search(r"for\s+Main\s+use\s*\(([^)]*)\)", g)
    mains = set(re.findall(r'"([^"]+)"', m.group(1))) if m else set()
    return mains - {"body_driver.adb"}

OFFLINE = set()

def scan():
    out = []
    OFFLINE.clear(); OFFLINE.update(offline_mains())
    SCOPE.clear(); SCOPE.update(declared_names())
    for f in sorted(driver_files()):
        b = os.path.basename(f)
        if b in OFFLINE or (b.split("-")[0] + ".adb" in OFFLINE and b != b.split("-")[0] + ".adb"):
            continue
        CUR[0] = b
        LINES[:] = open(f, encoding="utf-8").read().split("\n")
        hdr_end = -1
        first = next((split_code(r)[0].strip() for r in LINES if split_code(r)[0].strip()), "")
        if re.match(r"separate\s*\(", first):
            depth = 0; started = False
            for i, raw in enumerate(LINES):
                c, _ = split_code(raw)
                if not started:
                    if re.match(r"\s*(procedure|function)\b", c):
                        started = True
                    else:
                        continue
                done = False
                for t in re.finditer(r"\(|\)|\bis\b", c):
                    if t.group(0) == "(": depth += 1
                    elif t.group(0) == ")": depth -= 1
                    elif depth == 0: done = True; break
                if done:
                    hdr_end = i; break
        for ln, raw in enumerate(LINES, 1):
            LN[0] = ln - 1
            if ln - 1 <= hdr_end:
                continue
            c, strs = split_code(raw)
            if not c.strip() and not strs:
                continue
            key = " ".join(c.split())
            for m in NUM.finditer(c):
                if not structural(m.group(0), c, m.start(), m.end()):
                    out.append((b, key, m.group(0), ln, fmt_digits(c, m.start(), m.end())))
            for m in TWICE.finditer(c):
                out.append((b, key, "2×" + " ".join(m.group(1).split()), ln, False))
            for s in strs:
                for m in SNUM.finditer(s):
                    out.append((b, key, '"' + m.group(0), ln, False))
    return out

TWICE = re.compile(r"(?<![A-Za-z_0-9.'])([A-Za-z_][A-Za-z_0-9]*(?:\s*\.\s*[A-Za-z_][A-Za-z_0-9]*)*(?:\s*\([^()]*\))?)\s*\+\s*\1(?![A-Za-z_0-9.(])")

def fmt_digits(c, s, e):
    """See the module docstring."""
    op, i = opener(c, s)
    if op != "(":
        return False
    b = c[:i].rstrip()
    if not re.search(r"\b(?:Image)$", b):
        return False
    return bool(re.search(r",\s*$", c[:s])) and bool(re.match(r"\s*\)", c[e:]))

UNDECIDED = "undecided: read the context and classify as structure / math / numerics / statistics / format"

def load():
    lines = collections.defaultdict(list)
    if os.path.exists(REG):
        for l in open(REG, encoding="utf-8"):
            if not l.strip() or l.startswith("#"):
                continue
            p = l.rstrip("\n").split("\t")
            if len(p) < 5:
                continue
            lines[(p[0], p[1], p[2])].append(p)
    return lines

TEETH = [
    ("Small := 4.0 * Geo_Base (C, Arm);", "4.0", True),
    ("Ns := Natural'Max (4, Natural (Pix.Length) / 8);", "4", True),
    ("S := Natural'Min (Frames, 6);", "6", True),
    ("if Selfmap.Blocked (S1, Med, 2, Notch) then", "2", True),
    ("Px := 1.0 / Long_Float (W);", "1.0", True),
    ("if C.Map.N_Cams > 1 then", "1", True),
    ("if Natural (L.Jaw.Length) = 1 then", "1", True),
    ("Put_Array (S, 1);", "1", True),
    ("Trip_Px : constant := 1.0;", "1.0", True),
    ("Retries : Natural := 1;", "1", True),
    ("Up : constant V3 := [0.0, 0.0, 1.0];", "1.0", True),
    ("Zero_Grip : constant Long_Float := 0.0;", "0.0", True),
    ("Send_Jaw (C, 0.0);", "0.0", True),
    ("N := 1_000;", "1_000", True),
    ("Eps := 1.0E-6;", "1.0E-6", True),
    ("Es : array (0 .. 1) of Floats;", "1", True),
    ("P := Pts (I + 2);", "2", True),
    ("if abs X > 7.0 then", "7.0", True),
    ("if abs R <= 1.0 then", "1.0", True),
    ("Y := X (2) + Z;", "2", False),
    ("M (I, 2) := 0.0;", "2", False),
    ("N := N + 1;", "1", False),
    ("A := [1.0, 0.0, 0.0];", "1.0", False),
    ("R := 1.0 - F;", "1.0", False),
    ("S := X ** 2;", "2", False),
    ("Z := 0.0;", "0.0", False),
    ("for K in 1 .. N loop", "1", False),
    ("for K in -1 .. N loop", "1", True),
    ("Arm : Integer := -1;", "1", True),
    ("Y := N - 1;", "1", False),
    ("Z := Pts (K) (J + 1);", "1", False),
]

def selftest():
    """See the module docstring."""
    SCOPE["t.adb"] = ({"Put_Array", "Send_Jaw", "Fmt"}, {"X", "M", "Pts", "V", "S", "N", "Z", "Y", "R", "A", "P", "Px", "Es", "Up", "Small", "Ns", "Retries", "Eps", "Blocked"}, set(STD_CALLABLE), set())
    CUR[0] = "t.adb"; PACKAGES.update({"Selfmap", "Plug"})
    bad = []
    for line, v, want in TEETH:
        LINES[:] = [line]; LN[0] = 0
        c, _ = split_code(line)
        ms = [m for m in NUM.finditer(c) if m.group(0) == v]
        got = bool(ms) and any(not structural(v, c, m.start(), m.end()) for m in ms)
        if not ms: bad.append("literal not found: %s  %s" % (v, line))
        elif got != want: bad.append("%s: %s  %s" % ("should be listed but is not" if want else "listed though structural", v, line))
    loop_try = ["for Try in 0 .. 2 loop", "   Press (C, Arm);", "end loop;"]
    loop_idx = ["for I in 0 .. 2 loop", "   V (I) := 0.0;", "end loop;"]
    for lines, want in ((loop_try, True), (loop_idx, False)):
        LINES[:] = lines; LN[0] = 0
        c, _ = split_code(lines[0]); m = [m for m in NUM.finditer(c) if m.group(0) == "2"][0]
        if (not structural("2", c, m.start(), m.end())) != want:
            bad.append("loop: %s" % lines[0])
    if not TWICE.search("if Got + Got < Ln then") or TWICE.search("if Got + Gotten < Ln then"):
        bad.append("X + X not read as 2 x X (or misread)")
    if [m.group(0) for m in SNUM.finditer('in thousandths of the picture (0..1000), max_tokens:700')] != ["0", "1000", "700"]:
        bad.append("numbers inside strings not all found")
    SCOPE.pop("t.adb", None)
    if bad:
        for b in bad: print("  tooth failed: " + b)
        return 1
    print("  checker teeth: all %d hold" % (len(TEETH) + 4))
    return 0

def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "check"
    if mode == "selftest":
        return selftest()
    if selftest():
        print("FAIL: the checker's own rules have loosened")
        return 1
    occ = scan()
    found = collections.Counter((b, k, v) for b, k, v, _, _ in occ)
    fmts = collections.Counter((b, k, v) for b, k, v, _, fd in occ if fd)
    where = {}
    for b, k, v, ln, _ in occ:
        where.setdefault((b, k, v), ln)
    reg = load()
    missing = {k: n - len(reg.get(k, [])) for k, n in found.items() if n > len(reg.get(k, []))}
    stale = {k: len(ls) - found.get(k, 0) for k, ls in reg.items() if len(ls) > found.get(k, 0)}
    if mode == "gen":
        keep = []
        for k, ls in reg.items():
            n = found.get(k, 0)
            if n == 0:
                continue
            keep += ls[:n]
        for (b, k, v), n in missing.items():
            nf = max(0, fmts[(b, k, v)] - sum(1 for p in reg.get((b, k, v), []) if p[3] == "format"))
            for i in range(n):
                cg, why = ("format", "digits printed in a log line or a message") if i < nf else ("undecided", UNDECIDED)
                keep.append([b, k, v, cg, why + "(%s:%d)" % (b, where[(b, k, v)])])
        keep.sort(key=lambda p: (p[0], p[1], p[2]))
        with open(REG, "w", encoding="utf-8") as fo:
            fo.write(HEADER)
            for p in keep:
                fo.write("\t".join(p) + "\n")
        print("appended %d rows, dropped %d rows whose literal is gone" % (sum(missing.values()), sum(stale.values())))
        return 0
    undecided = [(k, p) for k, ls in reg.items() for p in ls if p[3] not in CATS and p[3] != "tuning" and found.get(k, 0) > 0]
    tuning = [(k, p) for k, ls in reg.items() for p in ls if p[3] == "tuning" and found.get(k, 0) > 0]
    print("== numeric literals: %d unlisted, %d unclassified, %d tuning, %d listed but gone =="
          % (sum(missing.values()), len(undecided), len(tuning), sum(stale.values())))
    rc = 0
    if missing:
        for (b, k, v), n in sorted(missing.items())[:40]:
            print("  unlisted: %s:%d  %s  %s" % (b, where[(b, k, v)], v, k[:110]))
        print("FAIL: every literal needs its origin in driver/numbers.tsv (numbers.py gen, then classify)")
        rc = 1
    if undecided:
        for k, p in undecided[:20]:
            print("  unclassified: %s  %s  %s" % (k[0], k[2], k[1][:110]))
        print("FAIL: classify every row as structure / math / numerics / statistics / format")
        rc = 1
    if tuning:
        for k, p in tuning[:20]:
            print("  tuning: %s  %s  %s" % (k[0], k[2], k[1][:110]))
        print("FAIL: tuning numbers are not allowed; derive the quantity from a measurement")
        rc = 1
    if stale:
        print("  (%d listed rows no longer occur; numbers.py gen drops them)" % sum(stale.values()))
    if rc == 0:
        print("PASS: no tuning numbers")
    return rc

if __name__ == "__main__":
    sys.exit(main())

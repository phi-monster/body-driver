#!/usr/bin/env python3
"""Subprograms of the driver that body_driver can never reach (dead code).

The call graph comes from the compiler's cross-reference (driver/obj/*.ali), not from names.
The self test and the offline tools (every main except body_driver) and the self-test packages
(*-tests.ad?) do not make code live: they are not the driver. There is no list of exemptions;
code that only a test calls is dead and goes, together with the test.

Usage: deadcode.py | deadcode.py --check (fails when anything is dead; run it after a build so the
cross-reference is current)
"""
import collections, glob, os, re, sys
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OBJ = os.path.join(ROOT, "driver", "obj"); SRC = os.path.join(ROOT, "driver", "src")
SOURCES = {os.path.basename(f): f for f in glob.glob(os.path.join(SRC, "**", "*.ad[sb]"), recursive=True)}
REF = re.compile(r"(?:(\d+)\|)?(\d+)([a-zA-Z<>=^*])(\d+)")

def parse(ali):
    files = []; ents = []
    cur = None; last = None
    for l in open(ali, encoding="utf-8", errors="replace"):
        if l.startswith("D "):
            files.append(l.split()[1])
        elif l.startswith("X "):
            p = l.split(); cur = int(p[1]); last = None
        elif cur is not None and l and (l[0].isdigit() or l.startswith(".")):
            body = l[1:] if l.startswith(".") else l
            if not l.startswith("."):
                m = re.match(r"(\d+)(\S)(\d+)[* ]?(\S+)", l)
                if not m:
                    continue
                line, kind, col, rest = m.groups()
                name = re.split(r"[{<(=\[]", rest)[0]
                last = {"file": cur, "line": int(line), "kind": kind, "name": name, "refs": []}
                ents.append(last)
                body = l[m.end():]
            if last is None:
                continue
            fi = last.get("cur", last["file"])
            for r in REF.finditer(re.sub(r"\{[^}]*\}|<[^>]*>|\([^)]*\)|\[[^\]]*\]", " ", body)):
                if r.group(1):
                    fi = int(r.group(1))
                last["refs"].append((fi, int(r.group(2)), r.group(3)))
            last["cur"] = fi
        elif l.startswith("X") is False and l[:1].isalpha() and cur is not None and not l.startswith("."):
            cur = None
    return files, ents

def main():
    ours = set(SOURCES)
    g = open(os.path.join(ROOT, "driver", "driver.gpr"), encoding="utf-8").read()
    mains = set(re.findall(r'"([^"]+)"', re.search(r"for\s+Main\s+use\s*\(([^)]*)\)", g).group(1)))
    offline = mains - {"body_driver.adb"}
    offline |= {b for b in ours if re.search(r"-tests\.ad[sb]$", b)}
    subs = {}          # (decl_file, decl_line, name) -> {"body": (file, lo, hi)}
    refs = []          # (ref_file, ref_line, target_key, kind)
    for ali in glob.glob(os.path.join(OBJ, "*.ali")):
        src = os.path.basename(ali)[:-4]
        files, ents = parse(ali)
        fname = lambda i: files[i - 1] if 0 < i <= len(files) else "?"
        for e in ents:
            df = fname(e["file"])
            if df not in ours:
                continue
            key = (df, e["line"], e["name"])
            if e["kind"] in "UVy":
                s = subs.setdefault(key, {"body": None})
                b = [(fname(fi), ln) for fi, ln, t in e["refs"] if t == "b"]
                t_ = [(fname(fi), ln) for fi, ln, t in e["refs"] if t == "t"]
                if b and t_ and b[0][0] == t_[-1][0]:
                    s["body"] = (b[0][0], b[0][1], t_[-1][1])
                elif s["body"] is None and e["kind"] in "UV":
                    pass
                for fi, ln, t in e["refs"]:
                    if t not in "rsRmi":
                        continue
                    refs.append((fname(fi), ln, key))

    by_file = collections.defaultdict(list)
    for k, s in subs.items():
        if s["body"]:
            f, lo, hi = s["body"]; by_file[f].append((lo, hi, k))
    def owner(f, ln):
        best = None
        for lo, hi, k in by_file.get(f, []):
            if lo <= ln <= hi and (best is None or hi - lo < best[1] - best[0]):
                best = (lo, hi, k)
        return best[2] if best else ("<package elaboration>", f)
    edges = collections.defaultdict(set)
    for f, ln, tgt in set(refs):
        if f in offline:
            continue
        edges[owner(f, ln)].add(tgt)
    roots = [k for k in subs if k[0] == "body_driver.adb" and k[2] == "Body_Driver"]
    roots += [k for k in edges if isinstance(k, tuple) and k[0] == "<package elaboration>" and k[1] not in offline]
    live = set(); stack = list(roots)
    while stack:
        k = stack.pop()
        if k in live:
            continue
        live.add(k)
        stack.extend(edges.get(k, ()))
    dead = [(k, s["body"]) for k, s in subs.items() if k not in live and s["body"] and s["body"][0] not in offline]

    outer = []
    for k, b in sorted(dead, key=lambda x: (x[1][0], x[1][1])):
        if any(o[1][0] == b[0] and o[1][1] <= b[1] and b[2] <= o[1][2] for o in outer):
            continue
        outer.append((k, b))
    tot_lines = sum(b[2] - b[1] + 1 for k, b in outer)
    print("== subprograms body_driver cannot reach: %d, %d lines ==" % (len(outer), tot_lines))
    for k, b in outer:
        print("  %s:%d-%d  %s" % (b[0], b[1], b[2], k[2]))
    if "--check" in sys.argv and outer:
        print("FAIL: unreachable code in the driver; delete it together with the tests written only for it")
        return 1
    return 0

sys.exit(main())

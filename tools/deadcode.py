#!/usr/bin/env python3
# 驱动里哪些子程序从 body_driver 走不到(死代码)—— 按编译器写的交叉引用(driver/obj/*.ali)算调用图,不按名字猜。
# 自检(selfcheck)和各种离线 exam 工具调到的不算活:它们不是驱动。死代码里的数照样在登记表里占名额,量法也成了"第二种量法",要删。
# 用法:deadcode.py [--nums](同时列出每段死代码里登记成调参数的有几处) | deadcode.py --check(有死代码就红;install.sh 在编译之后跑,交叉引用要是新的)
# 只给自检当假身体用的钩子(驱动自己不调、专门让自检替身体喂一拍)照实列在 TEST_HOOKS 里,每一个写明替谁。
TEST_HOOKS = {
    ("plug.adb", "Lock_Feed"): "自检在主线程里当假身体,替 Lock_Beat 从链路收的那一帧",
    ("contact.adb", "Rotation"): "脑要它'绕一根轴转'的那种运动(接触集执行层 Contact.Exec.Steps 吃的 Twist);自检拿它验转着走,驱动接上'脑要它怎么动'(PLAN ② 接下来第 1 条)就是活的",
    ("kinem.adb", "Refine_Tracks"): "把 Fit 的最后一步(④ 多视图,Refine_Until_Done)单独交给自检,从真模型起步验它",
    # 路 6 部件和轴(大并行 §2 第 12 条)的新包,接口处在别路的文件里(Run_Segment 看东西动的那一处调 Fit、脑要挪一块时调 Follow),
    # 等主代理合并时接上(路 6 的报告里写了接法);在那之前只有自检的焊点调它们。接上以后这 14 条全删
    ("linkage.adb", "Fit"): "路 6:看东西动时按刚体运动归块、每两块拟合一根轴;驱动接上'东西被跟住的点每一帧的交点'(路 3 的东西)就是活的",
    ("linkage.adb", "Say"): "路 6:Fit 的结果印一句日志;Fit 接上就是活的",
    ("linkage.adb", "Follow"): "路 6:轴还不知道时顺着它让的方向走;驱动接上'脑要挪一块'(执行层按 Geo_Move 走一步)就是活的",
    ("linkage.adb", "Light_Len"): "路 6:Follow 一步多长(看得出它挪了的最轻一步);Follow 接上就是活的",
    ("linkage.adb", "Gate"): "路 6:同一个置信度的卡方门(Fit / Follow 用);它俩接上就是活的",
    ("linkage.adb", "Mahal"): "路 6:马氏距离的平方(Follow 判挪没挪用);Follow 接上就是活的",
    ("linkage.adb", "Inv3"): "路 6:3×3 求逆(Mahal、Fit 用);它们接上就是活的",
    ("linkage.adb", "Eig_Sym"): "路 6:对称阵的特征分解(Fit 的 Horn 和铺没铺开、Max_Eig3 用);Fit 接上就是活的",
    ("linkage.adb", "Max_Eig3"): "路 6:协方差最大的特征值(Fit、Light_Len 用);它们接上就是活的",
    ("linkage.adb", "Perp"): "路 6:垂直于一个方向的一对轴(Fit 的轴参数化、Follow 试的方向用);它们接上就是活的",
    ("linkage.adb", "Gate_F"): "路 6:噪声是量的时候的门(Paulson 的 F 分位;Fit 里每一道门都按它开);Fit 接上就是活的",
    ("linkage.adb", "Z_Of"): "路 6:平方和换成一维正态的倍数(Fit 里判两块合不合、先合哪一对);Fit 接上就是活的",
    ("linkage.adb", "Follow_Held"): "路 6:手里的东西沿要的方向变一个单位、轴不知道 ⇒ 顺着它让的方向走(Change_Held_Qty 那一下换成它;接法见路 6 的报告)",
    ("linkage.adb", "Motion_Of"): "路 6:两块之间的轴 ⇒ 接触集的旋量(握住 + 扣扳机那种 Meanwhile 里动的那一段);两个要一起进物理检查时接上就是活的",
    # 路 6 拿着的东西(§2 第 20 条,I7 里路 6 那一份):抓住以后认跟着手走的点、它在手上的形状、每看一眼查滑没滑;
    # 接口处在别路的文件里(接触集合拢以后调 Take、每看一眼调 Check_Slip、接触集搜索时问 In_World),接上以后这 6 条删
    ("held.adb", "Take"): "路 6:抓住以后认哪些点跟着手走(和开机认长在眼上同一个判法)、它在手的系里的形状;接触集合拢以后调它就是活的",
    ("held.adb", "In_World"): "路 6:拿着的东西此刻在世界里在哪、多不准(进'身体能碰东西的地方'那张单子);接触集搜索时问它就是活的",
    ("held.adb", "Check_Slip"): "路 6:每看一眼查拿着的东西滑没滑(成团地比、算上手此刻的不准);看东西那一步调它就是活的",
    ("held.adb", "Scl"): "路 6:3×3 乘一个数(Take / Check_Slip 用);它们接上就是活的",
    ("held.adb", "Mapped"): "路 6:位姿的不准换一个系看(Take / In_World / Check_Slip 用);它们接上就是活的",
    ("held.adb", "Add"): "路 6:两个 6×6 不准相加(Take / In_World / Check_Slip 用);它们接上就是活的",
    # 路 6 对准插进去(§2 第 21 条)、一次走不完的(§2 第 22 条):执行层"往里送 / 松开再握"那一处调它们(Run_Segment 留给路 6 的接口),接上以后这 2 条删
    ("seek.adb", "Run"): "路 6:对准插进去 —— 沿轴送,被挡住就在横着的不准那一片里按缝定的格子从最可能的试起;执行层接上就是活的",
    ("strokes.adb", "Run"): "路 6:一次走不完的 —— 到了范围的头就松开、空着转回、再握,到转不动或走够为止;执行层接上就是活的",
    ("linkage-together.adb", "Check"): "路 6:同时两套接触(A 不动 + 它上面的 B 沿轴动)的物理检查,两个要一起算;脑一次说得出两个要(I5)时接上就是活的",
}
import os, re, sys, glob, collections
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OBJ = os.path.join(ROOT, "driver", "obj"); SRC = os.path.join(ROOT, "driver", "src")
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
                    fi = int(r.group(1))                     # "文件号|" 一出现,这一条后面的引用都在那个文件里
                last["refs"].append((fi, int(r.group(2)), r.group(3)))
            last["cur"] = fi
        elif l.startswith("X") is False and l[:1].isalpha() and cur is not None and not l.startswith("."):
            cur = None
    return files, ents

def main():
    want_nums = "--nums" in sys.argv
    ours = {os.path.basename(f) for f in glob.glob(os.path.join(SRC, "*.ad[sb]"))}
    g = open(os.path.join(ROOT, "driver", "body_driver.gpr"), encoding="utf-8").read()
    mains = set(re.findall(r'"([^"]+)"', re.search(r"for\s+Main\s+use\s*\(([^)]*)\)", g).group(1)))
    offline = mains - {"body_driver.adb"}
    #  离线程序分开编译出去的文件(selfcheck-welds_path_N.adb 这种)也是离线程序,不是驱动
    offline |= {os.path.basename(f) for f in glob.glob(os.path.join(SRC, "*.adb"))
                if os.path.basename(f).split("-")[0] + ".adb" in offline and "-" in os.path.basename(f)}
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
                        continue                             # 只算"用到":调用 s、引用 r(含 'Access)、分派 R、改 m、隐式 i;形参 > < = ^、身体 b、结尾 t / l 不算
                    refs.append((fname(fi), ln, key))
    # 每个引用落在哪个子程序的身体里(最里面那层)
    by_file = collections.defaultdict(list)
    for k, s in subs.items():
        if s["body"]:
            f, lo, hi = s["body"]; by_file[f].append((lo, hi, k))
    def owner(f, ln):
        best = None
        for lo, hi, k in by_file.get(f, []):
            if lo <= ln <= hi and (best is None or hi - lo < best[1] - best[0]):
                best = (lo, hi, k)
        return best[2] if best else ("<包的初始化>", f)
    edges = collections.defaultdict(set)
    for f, ln, tgt in set(refs):
        if f in offline:
            continue
        edges[owner(f, ln)].add(tgt)
    roots = [k for k in subs if k[0] == "body_driver.adb" and k[2] == "Body_Driver"]
    roots += [k for k in edges if isinstance(k, tuple) and k[0] == "<包的初始化>" and k[1] not in offline]
    live = set(); stack = list(roots)
    while stack:
        k = stack.pop()
        if k in live:
            continue
        live.add(k)
        stack.extend(edges.get(k, ()))
    dead = [(k, s["body"]) for k, s in subs.items() if k not in live and s["body"] and s["body"][0] not in offline]
    # 死子程序里面嵌套的也死,只列最外层
    outer = []
    for k, b in sorted(dead, key=lambda x: (x[1][0], x[1][1])):
        if any(o[1][0] == b[0] and o[1][1] <= b[1] and b[2] <= o[1][2] for o in outer):
            continue
        outer.append((k, b))
    nums = collections.Counter()
    if want_nums:
        reg = [l.rstrip("\n").split("\t") for l in open(os.path.join(ROOT, "numbers_registry.tsv"), encoding="utf-8") if l.strip() and not l.startswith("#")]
        lines_of = {}
        for k, b in outer:
            f = b[0]
            if f not in lines_of:
                lines_of[f] = open(os.path.join(SRC, f), encoding="utf-8").read().split("\n")
        sys.path.insert(0, os.path.join(ROOT, "tools"))
        src = open(os.path.join(ROOT, "tools", "numbers.py"), encoding="utf-8").read().replace('if __name__ == "__main__":\n    sys.exit(main())', "")
        ns = {"__file__": os.path.join(ROOT, "tools", "numbers.py"), "__name__": "nm"}; exec(src, ns)
        tun = collections.Counter((r[0], r[1]) for r in reg if r[3] == "调参数")
        for k, b in outer:
            for ln in range(b[1], b[2] + 1):
                c, _ = ns["split_code"](lines_of[b[0]][ln - 1])
                nums[k] += tun.get((b[0], " ".join(c.split())), 0)
    outer = [(k, b) for k, b in outer if (b[0], k[2]) not in TEST_HOOKS]
    tot_lines = sum(b[2] - b[1] + 1 for k, b in outer)
    print("== 从 body_driver 走不到的子程序:%d 段、%d 行 ==" % (len(outer), tot_lines))
    for k, b in outer:
        print("  %s:%d-%d  %s%s" % (b[0], b[1], b[2], k[2], ("  (调参数 %d)" % nums[k]) if want_nums else ""))
    if "--check" in sys.argv and outer:
        print("🔴 驱动里不许留走不到的代码:删掉(连同只为它写的焊点);git 里有历史")
        return 1
    return 0

sys.exit(main())

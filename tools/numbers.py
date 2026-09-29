#!/usr/bin/env python3
# 驱动里的每一个数字字面量都得有来历(owner 09-29:"这不就是写死常数吗,这种东西为啥能进我们 driver,我们的程序没有拦住你")。
# 旧的 check_constants.sh 有三个口子:0.5 / 2 / 3 / 4 / 0.25 / 0.75 / 100 这些值一句说明不用就放行;旁边写一句"无量纲 / 比例 / 次数"的注释什么数都放行;
# 整数根本不看。"乘 4"、"最多试 3 回"、"留前 8 组"就是这么进来的。
# 09-30 六路审计又查出这一版自己的口子,全补上:
#   0 / 1 / 0.0 / 1.0 一律放行 —— "夹爪读数在 [0,1]"、"没读数就发 1.0"、"门 = 1 px"、"朝向 ± 超过 1 弧度"都藏在这里;
#   Max (4, …) / Min (Frames, 6) 当成"第几个分量"放行 —— 函数和属性的参数不是下标;
#   一行里只要有一处像下标,这一行同样的数全放行 —— 改成一处一处判;
#   1_000_000、1.0E-6、16#FF# 这几种写法根本认不出来;字符串里的数不看(给脑的 max_tokens / temperature 就在字符串里);
#   协议文件整个跳过;"1.0e-4 在 Max 里 = 防除零"这类自动定类 —— 0.1 mm 的世界尺度就这么被定成了"数值"。
# 09-30 第二轮审计又查出:带包名的函数(Selfmap.Blocked (…, 2, …))的参数被当成下标;1.0 / W 就是"1 像素";"至少一个 / 多于一个"藏在 Max (1, …)、> 1 里;
# 抓握目标 0.0 是 x5"0 = 合"的约定;"两倍"写成 X + X 就没有字面数;字符串里 0..1000 的 1000 认不出。都补上。
# 现在只有这几种按规则当结构(不登记):0(除了当读数 / 目标传给函数的 0.0);幂次;下标位置上的小整数(已声明的变量后面、括号里就是这一个数);
# 1 在 X + 1 / X - 1 / 1 .. N 里;1.0 在 1.0 - x 和不起名字的 0 / ±1 轴向量里;0 .. 2 在类型声明里,或循环变量在循环体里当下标用。
# 其余每一个(字符串里的也算)都要登记在 numbers_registry.tsv 里,一处一行,写明属于哪一类、为什么;没登记的一出现就红,"待定"也红。类别:
#   结构      下标、分量个数、每点几行残差、文件 / 协议里的字段位置
#   数学      恒等式里的数(一半、两倍、平方、π、四元数公式)
#   数值      算法本身的数(迭代上限、收敛容差、防除零、阻尼)—— 不描述身体,不描述世界
#   统计      统计换算(中位绝对偏差换标准差 1.4826)和按置信度定的门(几倍标准差)
#   格式      协议、文件格式、图像格式、单位换算、日志印几位
#   调参数    我拍的数(步子乘几、留几成、最多几回、至少几个……)—— 只许减不许加,要换成量出来的
# 用法:numbers.py check | numbers.py gen(没登记的追加成"待定",登记了但代码里已经没有的删掉)
import re, sys, os, glob, collections
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REG = os.path.join(ROOT, "numbers_registry.tsv")
CEIL = os.path.join(ROOT, "numbers_tuning_ceiling.txt")
CATS = {"结构", "数学", "数值", "统计", "格式", "调参数"}
HEADER = "# 文件\t那一行(去掉注释和字符串、空白归一)\t数(字符串里的带 \" 前缀)\t类别\t为什么\t管什么\t风险\t怎么换\n"
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
    """一行拆成:代码(字符串换成 "",注释去掉)和这一行里的字符串内容"""
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
    offline = os.path.basename(path) in OFFLINE          # 自检、exam 工具里的包改名(package Cg renames …)不算驱动里的包名
    for raw in open(path, encoding="utf-8").read().split("\n"):
        c, _ = split_code(raw)
        if not offline:
            #  代码里当前缀用的包名:with 进来的取第一节(with Ada.Strings.Fixed ⇒ Ada;Fixed 不是前缀,别把叫 Fixed 的变量当成包);
            #  包自己的名字各节都算(package Contact.Grasp ⇒ Contact、Grasp);package X renames … ⇒ X
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
    """名字按文件认:本文件(连同它的 .ads)里声明的先算,其次是所有 .ads 公开的,再次是 Ada 标准库;哪儿都没有的当函数(宁可多登记)"""
    per = {}
    pub_call = set(STD_CALLABLE); pub_obj = set()
    files = glob.glob(os.path.join(ROOT, "driver/src/*.ad[sb]"))
    for f in files:
        per[f] = names_in(f)
        if f.endswith(".ads"):
            pub_call |= per[f][0]; pub_obj |= per[f][1]
    scope = {}
    for f in files:
        lc, lo = set(per[f][0]), set(per[f][1])
        spec = f[:-1] + "s"
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
    """括号前面是什么:'index'(已声明的变量 / 上一个括号的结果),'attr:Name'(属性),'call'(函数 / 类型 / 库),'none'(表达式、聚合、关键字)"""
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
        return "call"                                                        # Selfmap.Blocked (…, 2, …):包里的函数,参数不是下标
    lc, lo, pc, po = SCOPE[CUR[0]]
    if nm in lc: return "call"
    if nm in lo: return "index"
    if nm in pc: return "call"
    if nm in po: return "index"
    return "call"

LINES = []
LN = [0]

def loop_indexes(var):
    """从这一行起到配对的 end loop,循环变量有没有被当成下标(X (I)、M (I, J))"""
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
            return False                                                     # F (X, 0.0):0 当成一个读数 / 目标发出去,可能是某台身体的约定
        if re.search(r"\bconstant\b[^;]*:=\s*-?\s*$", before):
            return False                                                     # X : constant := 0.0:起了名字的 0 是一个设定
        return True                                                          # 0 没有大小,不带尺度
    if re.search(r"\*\*\s*$", before):
        return True                                                          # 幂次
    op, i = opener(c, s)
    head = head_of(c, i) if op == "(" else "none"
    inner = c[i + 1:closer(c, i)] if op else ""
    if is_int(v):
        n = ival(v)
        if head.startswith("attr:") and head[5:] in ("First", "Last", "Range", "Length") and inner.strip() == v and n <= 3:
            return True                                                      # A'Range (2) = 第几维
        direct = re.search(r"[(,]\s*$", before) and re.match(r"\s*[),]", after)
        if head == "index":
            if direct:
                return n <= 8                                                # X (2)、M (I, 2):第几个分量
            if re.search(r"\.\.\s*$", before) or re.match(r"\s*\.\.", after):
                return n <= 2                                                # 切片:只认 0 .. 2 这种分量范围
            return n == 1                                                    # X (I + 1):前后一个
        if n == 1:
            m1 = re.search(r"([A-Za-z_][A-Za-z_0-9]*|[0-9.]+|[)\]])\s*[-+]\s*$", before)
            if m1 and m1.group(1).lower() not in KEYWORDS:
                return True                                                  # X + 1 / X - 1:前后一个(减号前面得是一个量;in -1、range -1、:= -1 是负一,不是减一)
            if re.match(r"\s*[-+](?!\s*[0-9])", after) and not re.search(r"[-+*/]\s*$", before):
                return True                                                  # 1 + X
            if re.match(r"\s*\.\.", after) and not re.search(r"-\s*$", before):
                return True                                                  # for I in 1 .. N:从 1 数起(-1 .. N 不算)
            return False                                                     # 比大小、赋值、参数、Max (1, …)、缺省值:都可能是"至少一个 / 正好一个"这种认身体的规矩
        if re.search(r"\b0\s*\.\.\s*$", before) and n <= 2:
            m = re.search(r"\bfor\s+([A-Za-z_][A-Za-z_0-9]*)\s+in\s+(?:reverse\s+)?0\s*\.\.\s*$", before)
            if not m:
                return op == "(" and head == "none" or bool(re.search(r"\brange\s+0\s*\.\.\s*$", before))   # array (0 .. 2) / range 0 .. 2:三个分量
            return loop_indexes(m.group(1))                                  # for I in 0 .. 2:I 在循环里当下标才是分量,不然是"试三回"
        return False
    if re.fullmatch(r"1\.0+", v):
        if re.match(r"\s*-(?!-)", after) and not re.search(r"[*/]\s*$", before):
            return True                                                      # 1.0 - x
        if (op == "[" or (op == "(" and head == "none")) and not re.search(r"\bconstant\b", c):
            ps = parts(inner)
            if len(ps) >= 2 and all(re.fullmatch(r"-?\s*[01](?:\.0+)?", p) for p in ps):
                return True                                                  # 全由 0 / ±1 组成的轴向量、单位四元数
        return False
    return False

def offline_mains():
    """构建里单独的离线程序(自检、各种 exam 工具)不进驱动;驱动自己的包(比如开机考核 Exam)照查。按 .gpr 的 Main 认,不按文件名猜"""
    g = open(os.path.join(ROOT, "driver", "body_driver.gpr"), encoding="utf-8").read()
    m = re.search(r"for\s+Main\s+use\s*\(([^)]*)\)", g)
    mains = set(re.findall(r'"([^"]+)"', m.group(1))) if m else set()
    return mains - {"body_driver.adb"}

OFFLINE = set()

def scan():
    out = []
    OFFLINE.clear(); OFFLINE.update(offline_mains())
    SCOPE.clear(); SCOPE.update(declared_names())
    for f in sorted(glob.glob(os.path.join(ROOT, "driver/src/*.ad[sb]"))):
        b = os.path.basename(f)
        if b in OFFLINE:
            continue
        CUR[0] = b
        LINES[:] = open(f, encoding="utf-8").read().split("\n")
        for ln, raw in enumerate(LINES, 1):
            LN[0] = ln - 1
            c, strs = split_code(raw)
            if not c.strip() and not strs:
                continue
            key = " ".join(c.split())
            for m in NUM.finditer(c):
                if not structural(m.group(0), c, m.start(), m.end()):
                    out.append((b, key, m.group(0), ln, fmt_digits(c, m.start(), m.end())))
            for m in TWICE.finditer(c):
                out.append((b, key, "2×" + " ".join(m.group(1).split()), ln, False))   # X + X 就是 2 × X,不写 2 也是一个数
            for s in strs:
                for m in SNUM.finditer(s):
                    out.append((b, key, '"' + m.group(0), ln, False))
    return out

TWICE = re.compile(r"(?<![A-Za-z_0-9.'])([A-Za-z_][A-Za-z_0-9]*(?:\s*\.\s*[A-Za-z_][A-Za-z_0-9]*)*(?:\s*\([^()]*\))?)\s*\+\s*\1(?![A-Za-z_0-9.(])")

def fmt_digits(c, s, e):
    """这一处是不是 Fmt (X, 3) / Img (X, 3) 的最后一个参数(印几位)"""
    op, i = opener(c, s)
    if op != "(":
        return False
    b = c[:i].rstrip()
    if not re.search(r"\b(?:Fmt|Img|Fmt_Px|Fmt_Mm|Fmt_Deg)$", b):
        return False
    return bool(re.search(r",\s*$", c[:s])) and bool(re.match(r"\s*\)", c[e:]))

UNDECIDED = "新出现的数:读上下文定成 结构 / 数学 / 数值 / 统计 / 格式 / 调参数 之一"

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
    # (这一行, 数, 该不该登记) —— 每一条都是审计里真查出来的口子;规则改松了,这里先红
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
    """检查器自己的牙:审计查出的每个口子各一条;另有循环、X + X、字符串三条要看上下文的"""
    #  Blocked 在这个假文件里是一个变量(act.adb 里就有 Blocked : Boolean):Selfmap.Blocked (…) 照样得按函数认
    SCOPE["t.adb"] = ({"Put_Array", "Send_Jaw", "Fmt"}, {"X", "M", "Pts", "V", "S", "N", "Z", "Y", "R", "A", "P", "Px", "Es", "Up", "Small", "Ns", "Retries", "Eps", "Blocked"}, set(STD_CALLABLE), set())
    CUR[0] = "t.adb"; PACKAGES.update({"Selfmap", "Plug"})
    bad = []
    for line, v, want in TEETH:
        LINES[:] = [line]; LN[0] = 0
        c, _ = split_code(line)
        ms = [m for m in NUM.finditer(c) if m.group(0) == v]
        got = bool(ms) and any(not structural(v, c, m.start(), m.end()) for m in ms)
        if not ms: bad.append("认不出这个数:%s  %s" % (v, line))
        elif got != want: bad.append("%s:%s  %s" % ("该登记没登记" if want else "不该登记却登记了", v, line))
    loop_try = ["for Try in 0 .. 2 loop", "   Press (C, Arm);", "end loop;"]
    loop_idx = ["for I in 0 .. 2 loop", "   V (I) := 0.0;", "end loop;"]
    for lines, want in ((loop_try, True), (loop_idx, False)):
        LINES[:] = lines; LN[0] = 0
        c, _ = split_code(lines[0]); m = [m for m in NUM.finditer(c) if m.group(0) == "2"][0]
        if (not structural("2", c, m.start(), m.end())) != want:
            bad.append("循环:%s" % lines[0])
    if not TWICE.search("if Got + Got < Ln then") or TWICE.search("if Got + Gotten < Ln then"):
        bad.append("X + X 没认成 2×X(或认错)")
    if [m.group(0) for m in SNUM.finditer('in thousandths of the picture (0..1000), max_tokens:700')] != ["0", "1000", "700"]:
        bad.append("字符串里的数没认全")
    SCOPE.pop("t.adb", None)
    if bad:
        for b in bad: print("  🔴 牙:" + b)
        return 1
    print("  牙:%d 条全咬得住" % (len(TEETH) + 4))
    return 0

def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "check"
    if mode == "selftest":
        return selftest()
    if selftest():
        print("🔴 检查器自己的规则松了")
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
            tun = [p for p in ls if p[3] == "调参数"]; oth = [p for p in ls if p[3] != "调参数"]
            keep += (tun + oth)[:n]                                          # 多出来的先删不是调参数的:删错了宁可多记一条调参数
        for (b, k, v), n in missing.items():
            nf = max(0, fmts[(b, k, v)] - sum(1 for p in reg.get((b, k, v), []) if p[3] == "格式"))
            for i in range(n):
                cg, why = ("格式", "日志 / 给脑的话里印几位小数") if i < nf else ("待定", UNDECIDED)
                keep.append([b, k, v, cg, why + "(%s:%d)" % (b, where[(b, k, v)])])
        keep.sort(key=lambda p: (p[0], p[1], p[2]))
        with open(REG, "w", encoding="utf-8") as fo:
            fo.write(HEADER)
            for p in keep:
                fo.write("\t".join(p) + "\n")
        print("追加 %d 条 · 删掉代码里已经没有的 %d 条" % (sum(missing.values()), sum(stale.values())))
        return 0
    undecided = [(k, p) for k, ls in reg.items() for p in ls if p[3] not in CATS and found.get(k, 0) > 0]
    tuning = sum(min(found.get(k, 0), sum(1 for p in ls if p[3] == "调参数")) for k, ls in reg.items())
    ceil = int(open(CEIL).read().strip()) if os.path.exists(CEIL) else 10 ** 9
    print("== 数字字面量:没登记的 %d 处 · 没定类的 %d 条 · 登记了但代码里没有的 %d 条 · 调参数 %d 处(上限 %d)=="
          % (sum(missing.values()), len(undecided), sum(stale.values()), tuning, ceil))
    rc = 0
    if missing:
        for (b, k, v), n in sorted(missing.items())[:40]:
            print("  没登记:%s:%d  %s  %s" % (b, where[(b, k, v)], v, k[:110]))
        print("🔴 每一个数都要登记来历(numbers_registry.tsv);调参数要换成量出来的,不许新加")
        rc = 1
    if undecided:
        for k, p in undecided[:20]:
            print("  没定类:%s  %s  %s" % (k[0], k[2], k[1][:110]))
        print("🔴 清单里每一条都要定成 结构 / 数学 / 数值 / 统计 / 格式 / 调参数 之一")
        rc = 1
    if stale:
        print("  (登记了但代码里已经没有的 %d 条:numbers.py gen 会删掉)" % sum(stale.values()))
    if tuning > ceil:
        print("🔴 调参数从 %d 涨到 %d —— 只许减不许加" % (ceil, tuning)); rc = 1
    elif rc == 0 and tuning < ceil:
        open(CEIL, "w").write("%d\n" % tuning); print("🟢 调参数降到 %d,上限收紧" % tuning)
    elif rc == 0:
        print("🟢 持平")
    return rc

if __name__ == "__main__":
    sys.exit(main())

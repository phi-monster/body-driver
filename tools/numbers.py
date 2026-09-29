#!/usr/bin/env python3
# 驱动里的每一个数字字面量都得有来历(owner 09-29:"这不就是写死常数吗,这种东西为啥能进我们 driver,我们的程序没有拦住你")。
# 旧的 check_constants.sh 有三个口子:0.5 / 2 / 3 / 4 / 0.25 / 0.75 / 100 这些值一句说明不用就放行;旁边写一句"无量纲 / 比例 / 次数"的注释什么数都放行;
# 整数根本不看。"乘 4"、"最多试 3 回"、"留前 8 组"就是这么进来的。
# 这里:结构上必须的数按规则认(向量 / 矩阵的第几个分量、编码格式包、身体文件的字段位置),其余每一个都要登记在 numbers_registry.tsv 里,
# 写明属于哪一类、为什么;没登记的一出现就红。类别只有这几种:
#   数学      恒等式里的数(一半、两倍、平方、π、四元数公式)
#   数值      算法本身的数(迭代上限、收敛容差、防除零、阻尼)—— 不描述身体,不描述世界
#   统计      统计换算(中位绝对偏差换标准差 1.4826)和按置信度定的门(几倍标准差)
#   格式      协议、文件格式、图像格式、单位换算
#   调参数    我拍的数(步子乘几、留几成、最多几回、至少几个……)—— 只许减不许加,要换成量出来的
# 用法:numbers.py check | numbers.py gen(把现在没登记的全列成"待定"追加进清单,人再一条条定类)
import re, sys, os, glob, collections
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REG = os.path.join(ROOT, "numbers_registry.tsv")
CEIL = os.path.join(ROOT, "numbers_tuning_ceiling.txt")
FORMAT_FILES = {"msgpack", "websocket", "codec", "bytes", "json", "http_client", "draw"}
CATS = {"数学", "数值", "统计", "格式", "调参数"}
NUM = re.compile(r"(?<![A-Za-z_0-9.#])[0-9]+(?:\.[0-9]+)?(?:e-?[0-9]+)?(?![A-Za-z_0-9#])")

def code_of(raw):
    c = raw.split("--")[0]
    c = re.sub(r'"[^"]*"', '""', c)
    c = re.sub(r"'.'", "''", c)
    return c

def structural(stem, v, c):
    if stem in FORMAT_FILES:
        return True
    if v in ("0", "1", "0.0", "1.0"):
        return True
    if v.isdigit() and int(v) <= 8:
        e = re.escape(v)
        if re.search(r"[A-Za-z0-9_)\]] ?\(\s*" + e + r"\s*[,)]", c): return True         # X (2) / N0 (2) / M (1, 2) / Pts (0) (1)
        if re.search(r"'First \+ " + e + r"\b|'Last - " + e + r"\b", c): return True        # 数组里的偏移
        if re.search(r"\bwhen " + e + r" =>|\bwhen " + e + r" \|", c): return True          # case 的第几种
        if re.search(r"\(\s*" + e + r"\s*,\s*[A-Za-z_0-9]+\s*\)", c): return True         # M (2, J)
        if re.search(r"\(\s*[A-Za-z_]+\s*,\s*" + e + r"\s*\)", c): return True           # M (I, 2) / V (T, 3)
        if re.search(r"\b0 \.\. " + e + r"\b", c) and int(v) in (2, 3, 5, 6, 8): return True   # 分量循环
        if re.search(r"\(\s*[A-Za-z_]+ \+ " + e + r"\s*\)|\(\s*" + e + r" \+ [A-Za-z_]+\s*\)", c): return True   # (Off + 3)
        if re.search(r"\b(V3_At|M3_At|V|T)\s*\(\s*T\s*,\s*" + e + r"\s*\)", c): return True
    if re.search(r"(V3_At|M3_At|V)\s*\(\s*T\s*,\s*" + re.escape(v) + r"\s*\)|\bT \(" + re.escape(v) + r"\)", c): return True   # 身体文件字段位置
    if re.search(r"\*\* *" + re.escape(v) + r"\b", c): return True                      # 平方 / 立方
    if v == "3" and re.search(r"3 \* [A-Za-z_][A-Za-z_0-9]* \+ [0-5]\b|\* 3\b|3 \* \(|\b3 \* [A-Z][A-Za-z_]* \* [A-Z]", c) and re.search(r"RGB|Rgb|Sp\b|Kw\b|Cw \* Ch|W \* H", c): return True   # 一个像素 3 个字节
    if v.isdigit() and re.search(r"\b[23] \* [A-Za-z_.]+ \+ [0-9]", c) and re.search(r"Nr|Rows|Row|Rp \(|Jr|Nres", c): return True   # 残差 / 行的计数(每点两行、每轴三行)
    return False

def scan():
    out = []
    for f in sorted(glob.glob(os.path.join(ROOT, "driver/src/*.ad[sb]"))):
        b = os.path.basename(f)
        stem = b.rsplit(".", 1)[0]
        if b == "selfcheck.adb" or b.endswith("exam.adb"):
            continue
        for raw in open(f, encoding="utf-8").read().split("\n"):
            c = code_of(raw)
            if not c.strip():
                continue
            key = " ".join(c.split())
            for m in NUM.finditer(c):
                v = m.group(0)
                if structural(stem, v, c):
                    continue
                out.append((b, key, v))
    return out

def guess(b, key, v):
    """按规则能定的先定(仍然登记、看得见);定不了的留"待定",人一条条看"""
    c = key
    e = re.escape(v)
    if re.search(r"(Fmt|Img|Fmt_Px|Fmt_Mm) \([^()]*(\([^()]*\))*[^()]*,\s*" + e + r"\s*\)", c) or re.search(r"Codec\.Fmt \(", c) and re.search(r",\s*" + e + r"\s*\)", c):
        return "格式", "日志 / 给脑的话里印几位小数"
    if re.search(r"0\.5 \* \(", c) and v == "0.5" or re.search(r"/ 2\.0\b", c) and v == "2.0" or re.search(r"/ 2\b", c) and v == "2":
        return "数学", "求中点 / 一半"
    if re.search(r"\bPi\b", c) and v in ("2.0", "0.5", "180.0", "2", "4.0"):
        return "数学", "π 的倍数 / 弧度换算"
    if b == "geom.adb" and re.search(r"1\.0 - 2\.0 \*|2\.0 \* \([A-Z] \* [A-Z] [-+] [A-Z] \* [A-Z]\)", c) and v == "2.0":
        return "数学", "四元数 → 旋转矩阵的恒等式"
    if re.fullmatch(r"1\.0e-[0-9]+", v) and (re.search(r"[<>]=?", c) or "Max" in c or "Min" in c):
        return "数值", "浮点的防除零 / 数值容差(不描述身体和世界)"
    if v in ("1.0e30", "1.0e29", "1.0e-300", "1.0e300", "1.0e9", "1.0e12") and ("Last" not in c):
        return "数值", "哨兵 / 数值极限(当作'没有'或'无穷')"
    if v == "1.4826":
        return "统计", "中位绝对偏差换成正态的标准差(统计换算)"
    if b.startswith("picture") and v in ("255", "256", "54", "14", "40", "24"):
        return "格式", "图像格式(8 位灰度、BMP 文件头)"
    return "调参数", "还没证明是数学 / 数值 / 统计 / 格式 ⇒ 当成拍的数,要换成量出来的(或者证明它不是)"

def load():
    reg = collections.Counter(); cat = {}
    if os.path.exists(REG):
        for l in open(REG, encoding="utf-8"):
            if not l.strip() or l.startswith("#"):
                continue
            p = l.rstrip("\n").split("\t")
            if len(p) < 5:
                continue
            k = (p[0], p[1], p[2])
            reg[k] += 1; cat[k] = (p[3], p[4])
    return reg, cat

def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "check"
    found = collections.Counter(scan())
    reg, cat = load()
    missing = found - reg
    if mode == "gen":
        with open(REG, "a", encoding="utf-8") as fo:
            if os.path.getsize(REG) == 0 if os.path.exists(REG) else True:
                fo.write("# 文件\t那一行(去掉注释、空白归一)\t数\t类别\t为什么\n")
            for (b, key, v), n in sorted(missing.items()):
                cg, why = guess(b, key, v)
                for _ in range(n):
                    fo.write("%s\t%s\t%s\t%s\t%s\n" % (b, key, v, cg, why))
        print("追加了 %d 条待定" % sum(missing.values()))
        return 0
    bad_cat = [k for k in reg if cat[k][0] not in CATS]
    tuning = sum(n for k, n in reg.items() if cat[k][0] == "调参数" and found[k] > 0)
    ceil = int(open(CEIL).read().strip()) if os.path.exists(CEIL) else 10 ** 9
    print("== 数字字面量:没登记的 %d 处 · 类别没定的 %d 条 · 调参数 %d 处(上限 %d)==" % (sum(missing.values()), len(bad_cat), tuning, ceil))
    rc = 0
    if missing:
        for (b, key, v), n in sorted(missing.items())[:40]:
            print("  没登记:%s  %s  %s" % (b, v, key[:110]))
        print("🔴 每一个数都要登记来历(numbers_registry.tsv);调参数要换成量出来的,不许新加")
        rc = 1
    if bad_cat:
        for k in bad_cat[:20]:
            print("  没定类:%s  %s  %s" % (k[0], k[2], k[1][:110]))
        print("🔴 清单里每一条都要定成 数学 / 数值 / 统计 / 格式 / 调参数 之一")
        rc = 1
    if tuning > ceil:
        print("🔴 调参数从 %d 涨到 %d —— 只许减不许加" % (ceil, tuning)); rc = 1
    elif rc == 0 and tuning < ceil:
        open(CEIL, "w").write("%d\n" % tuning); print("🟢 调参数降到 %d,上限收紧" % tuning)
    elif rc == 0:
        print("🟢 持平")
    return rc

sys.exit(main())

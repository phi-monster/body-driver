#!/usr/bin/env bash
# 🔴 owner 2026-08-13:driver 是要进入万千世界的,不许照着某个 benchmark 写。代码里不许出现 benchmark / 任务 / 场景名;驱动树里零 Python;
# 命令里不许写死机器人字段名(键名全部来自 discover 认出来的路径;下面列出的是线缆协议的信封键,不是机器人字段)。
set -u
ROOT="$(cd "$(dirname "$0")" && pwd)"
PAT='robodojo|isaac|libero|calvin|general_pickup|stack_blocks|pack_objects|store_tools|eval_result|_result\.json|franka|arx|x5_grasp'
bad=0
for f in "$ROOT"/driver/src/*.ads "$ROOT"/driver/src/*.adb; do
  hit=$(sed 's/--.*$//' "$f" | grep -niE "$PAT" || true)
  if [ -n "$hit" ]; then echo "🔴 $f"; echo "$hit"; bad=1; fi
done
py=$(find "$ROOT/driver" -name '*.py' 2>/dev/null)
if [ -n "$py" ]; then echo "🔴 驱动树里有 Python:"; echo "$py"; bad=1; fi
owed=$(printf 'message_type\nmessage_id\nstep\npayload\nevaluation_id\naction_case_id\ntrial_id\nrepeat_index\nsent_at\nresult\nok\nserver\nserver_instance_id\nfunc_name\nobs\nobservation\ninstruction\nnd\ntype\nshape\ndata\nhello\nprepare_case\nreset\ncall\ninfer\ntrial_end\nheartbeat\nhello_ack\nprepare_case_ack\nreset_result\ncall_result\ninfer_result\ntrial_end_ack\nheartbeat_ack\nxpolicylab_policy_server\nbody-driver\nget_action\n_\n_c\n')
keyhit=$(sed 's/--.*$//' "$ROOT/driver/src/plug.adb" "$ROOT/driver/src/msgpack.adb" "$ROOT/driver/src/layout.adb" 2>/dev/null | grep -oE '"[a-z_]+"' | tr -d '"' | sort -u | grep -vxF "$owed" || true)
if [ -n "$keyhit" ]; then echo "🔴 插头里出现了协议信封之外的写死字段名(机器人字段必须来自 discover):"; echo "$keyhit"; bad=1; fi
# 🔴 owner 2026-09-26:每个量只有一种量法,不许"一个东西几种办法、a 不行再 b";身体另外报的读数(内参、深度)驱动不读。
# 下面这几条是已经删掉的第二种量法,谁把它们加回来,装机就过不去(去注释后查)。
second=""
chk() {   # $1 文件(可以是 act*.adb 这种:母体连同它分开编译出去的那些文件一起查)  $2 模式  $3 说明
  if cat $ROOT/driver/src/$1 2>/dev/null | sed 's/--.*$//' | grep -qE "$2"; then second="$second\n  $1:$3"; fi
}
chk plug.adb 'Has_K[[:space:]]*:=[[:space:]]*True' '读了身体给的相机内参(焦距要自己量)'
chk plug.adb 'Has_Depth[[:space:]]*:=[[:space:]]*True' '读了身体给的深度图(远近要自己量)'
chk 'act*.adb' 'Picture\.Measure_In_Box' '抠物体又退回按明暗切(只许仪器那一种)'
chk 'act*.adb' 'Geom\.Triangulate[[:space:]]*\(G,[[:space:]]*Use_Obs' '东西在哪又退回"我自己挪过的那几眼"(只许两眼同一刻的交点)'
chk 'act*.adb' 'Geo_Track[[:space:]]*\(C,[[:space:]]*F,[[:space:]]*Cam,[[:space:]]*Cand' '标定跟点又退回按槽号重切(只许跟点仪器)'
chk 'act*.adb' 'Hit_Plane[[:space:]]*\(Geom\.Cam_Pos[[:space:]]*\(G,[[:space:]]*Hp\)' '东西在哪又退回"一条视线落到它躺的面上"(只许两眼同一刻的交点)'
if [ -n "$second" ]; then echo -e "🔴 同一个量出现了第二种量法:$second"; bad=1; fi
# 🔴 ④(09-27):给脑的话里的长度只许按身体自己的尺子说(Act.Len,"hand-lengths")。Act.Mm 印的是日志的"单位"(世界单位 ≠ 米),
#    进了英文句子就是给脑说了一个它对不上号的数(原来印"m":"jaw 1.752 m"其实是张口 91 mm)。按语句查:用了 Mm、字面量全是英文 = 给脑的
mmhit=$(cat "$ROOT"/driver/src/act*.adb | sed 's/--.*$//' | perl -CSD -Mutf8 -0777 -ne 'while (/((?:[^;"]|"[^"]*")*);/g) { my $s = $1; next unless $s =~ /\bMm \(/; my @l = ($s =~ /"([^"]*)"/g); my $eng = grep { /[A-Za-z]{3,}/ } @l; my $han = grep { /[\x{4e00}-\x{9fff}]/ } @l; if ($eng && !$han) { (my $t = $s) =~ s/\s+/ /g; print substr($t, 0, 160), "\n"; } }')
if [ -n "$mmhit" ]; then echo "🔴 给脑的英文句子里用了 Mm(日志单位),要用 Len(指尖长):"; echo "$mmhit"; bad=1; fi
# 🔴 接触集里没有动作词(大并行路 5,10-01;owner 09-29:"接触集是所有任务的,不许叫它'抓'";"这三件事需要你一件一件写代码吗,如果需要,你就写不完世界所有事")。
#    接触集只吃身体的路、东西的形状、要东西怎么动(一个旋量),做一个搜索 + 一个物理检查;加一件事 = 什么代码都不加。
#    查的是接触集那几个包(contact*.ad?)和 Act 里把量交给它的那一段(act-plan_contact.adb):标识符(按 _ 切开的每一截)和字符串里的词,
#    英文按整词、中文按词组;注释不查(注释里记的是历史,"八月那一版会抬"这种话照样能写)。
#    例外只有下面这几个旧名:主代理 selfcheck.adb 的旧焊点还按它们在用(完整的具名聚合、Contact.Turn),
#    合并时主代理把那些用处改掉、从这里删掉对应的一条;分开编译的正文开头那段参数表(和 act.ads 里的声明一字不差)不查。
ACT_WORDS='lift lifts lifted lifting push pushes pushed pushing pull pulls pulled pulling turn turns turned turning press presses pressed pressing
grasp grasps grasped grasping grab grabs grabbed grabbing hold holds holding held pinch pinches pinched pinching pry pries pried prying
flip flips flipped flipping pour pours poured pouring carry carries carried carrying drag drags dragged dragging dodge dodges dodged dodging
throw throws threw thrown wipe wipes wiped wiping scoop scoops scooped scooping stack stacks stacked stacking'
ACT_CN='抬起 抬高 抬升 推动 推开 推过去 拉开 拉动 抓住 抓起 抓取 抓握 捏住 拧紧 拧开 撬开 翻转 翻过来 倒出 擦干 舀起 搬动 拿起 拿住 举起 夹住 握住 托住'
ACT_EXEMPT=''
read -r -d '' ACT_PL <<'PL' || true   #  (不用 $(cat <<…):macOS 自带的 bash 3.2 在命令替换里解析这段的引号有毛病)
my %en = map { $_ => 1 } split /\s+/, $ENV{ACT_WORDS};
use Encode; my @cn = split /\s+/, Encode::decode_utf8 ($ENV{ACT_CN});   # 环境变量是字节,按 UTF-8 解开才和读进来的文件比得上
my %ex = map { $_ => 1 } split /\s+/, $ENV{ACT_EXEMPT};
for my $f (@ARGV) {
  open my $fh, "<:encoding(UTF-8)", $f or next;
  (my $b = $f) =~ s{.*/}{};
  my $hdr = 0; my $first = 1; my $ln = 0;
  while (my $l = <$fh>) {
    $ln++;
    chomp $l;
    if ($first && $l =~ /^\s*separate\s*\(/) { $hdr = 1; }
    $first = 0 if $l =~ /\S/ && $l !~ /^\s*--/ && $l !~ /^\s*with\b/;
    if ($hdr == 1 && $l =~ /^\s*(procedure|function)\b/) { $hdr = 2; }
    if ($hdr == 2) { $hdr = 3 if $l =~ /\bis\s*$/; next; }
    my $code = ""; my @strs = (); my $i = 0; my $n = length $l;
    while ($i < $n) {
      my $c = substr($l, $i, 1);
      if ($c eq "'" && $i + 2 < $n && substr($l, $i + 2, 1) eq "'" && !($i > 0 && substr($l, $i - 1, 1) =~ /[A-Za-z0-9_)]/)) { $code .= " "; $i += 3; next; }
      if ($c eq '"') {
        my $j = $i + 1; my $s = "";
        while ($j < $n) {
          my $d = substr($l, $j, 1);
          if ($d eq '"') { if ($j + 1 < $n && substr($l, $j + 1, 1) eq '"') { $s .= '"'; $j += 2; next; } last; }
          $s .= $d; $j++;
        }
        push @strs, $s; $code .= " "; $i = $j + 1; next;
      }
      last if $c eq "-" && substr($l, $i + 1, 1) eq "-";
      $code .= $c; $i++;
    }
    for my $id ($code =~ /([A-Za-z_][A-Za-z_0-9]*)/g) {
      next if $ex{"$b:$id"};
      for my $t (split /_+/, lc $id) { if ($en{$t}) { print "$b:$ln: $id\n"; last; } }
    }
    for my $s (@strs) {
      for my $w ($s =~ /([A-Za-z]+)/g) { if ($en{lc $w}) { print "$b:$ln: \"$w\"\n"; } }
      for my $w (@cn) { if (index($s, $w) >= 0) { print "$b:$ln: \"$w\"\n"; } }
    }
  }
}
PL
act_scan() { ACT_WORDS="$ACT_WORDS" ACT_CN="$ACT_CN" ACT_EXEMPT="$ACT_EXEMPT" perl -CSD -Mutf8 -e "$ACT_PL" "$@"; }
#    牙:先拿一段种了动作词的假代码跑一遍(标识符、英文字符串、中文字符串各一处,外加一段干净的和一行只在注释里的),扫不出来就是棘轮自己松了
tooth=$(mktemp -d)
printf '%s\n' 'package body Contact.Fake is' '   function Lift_Load (X : Long_Float) return Long_Float is (X);   --  注释里的 lift 不算' \
  '   S : constant String := "cannot lift it";' '   T : constant String := "先抬起再说";' 'end Contact.Fake;' > "$tooth/contact-fake.adb"
printf '%s\n' 'package body Contact.Clean is' '   function Least (X : Long_Float) return Long_Float is (X);   --  hold / push / 抬起 只在注释里' \
  '   S : constant String := "no contacts can make it move that way";' 'end Contact.Clean;' > "$tooth/contact-clean.adb"
t_hit=$(act_scan "$tooth/contact-fake.adb" | wc -l | tr -d ' ')
t_clean=$(act_scan "$tooth/contact-clean.adb" | wc -l | tr -d ' ')
rm -rf "$tooth"
if [ "$t_hit" != 3 ] || [ "$t_clean" != 0 ]; then echo "🔴 接触集动作词棘轮的牙松了:种了 3 处扫出 $t_hit 处、干净的扫出 $t_clean 处"; bad=1; fi
act_files=""
for f in "$ROOT"/driver/src/contact.ad? "$ROOT"/driver/src/contact-*.ad? "$ROOT"/driver/src/act-plan_contact.adb; do
  act_files="$act_files $f"
done
acthit=$(act_scan $act_files)
if [ -n "$acthit" ]; then echo "🔴 接触集里出现了动作词(它只认身体的路、东西的形状、要它怎么动;例外只许是 ACT_EXEMPT 里那几个旧名):"; echo "$acthit"; bad=1; fi
[ "$bad" = 0 ] && echo "🟢 驱动:没有 benchmark 名字 · 零 Python · 命令里没有写死的机器人字段名 · 每个量一种量法(六条已删的退路没回来)· 给脑的长度按指尖长说 · 接触集里没有动作词(牙咬得住)"
exit $bad

#!/bin/bash
# 路 8 开机炮:主线驱动(不设 BL_BIN)在排队位上开机 —— 走 /root/q/run.sh 排队;驱动日志走到"── 第 1 轮"(身体开完机、第一次叫脑)就放锁。不做任务。
# 用法:bash qboot.sh 炮名 任务 [身体文件(默认 /root/cal_v1b78.json;写 zero = 从零量)] [时限分钟(默认 14)]
#   身体配置、种子用环境变量:BOOT_CFG(默认 arx_x5;人形 g1_rgb、无人机 drone_rgb)、BOOT_SEED(随机题机一题一个种子;不给就是任务默认的)
#   BOOT_KEEP=1:量身体文件的那种炮 —— 驱动落盘(look/)和逐帧的位姿、画面(vid/)都留着,给 /root/diag/v1b_score_fk_cur.py 按仿真真值打分;
#   不给就和原来一样:不录 vid、跑完只留 cal* / 经历*(P8P、P8R 第一回没留,量出来的身体文件没法事后打分)
# 身体文件先拷一份到 /root/p8/cal_<炮名>.json(驱动干活时会把量到的写回 --out,原件不能动)。
# 证据留在 /root/p8/boot/<炮名>/:开机那几行、第 1 轮给脑的清单、脑第一眼看到的图(转成 jpg)、sim.log 里的报错。
# 用完就清:shot 目录里除了 cal*、经历* 以外都删(look/ 近 10 MB、sim.log),RoboDojo 为这一集开的流式视频临时文件也删(只删 bd_ 任务的)。
set -u
K=$1; TASK=$2; BASE=${3:-/root/cal_v1b78.json}; LIM=${4:-14}
k=$(echo "$K" | tr 'A-Z' 'a-z')
E=/root/p8/boot/$K
N=/root/N$K
mkdir -p "$E"
[ -e "$N" ] && { echo "🔴 $N 已经有了,换个炮名"; exit 5; }
CALF=/root/p8/cal_$k.json
if [ "$BASE" != zero ]; then
  # 驱动读的是:身体文件、.geo.json(眼的几何)、.kin.txt(开机前半段量的运动学和世界)和 .kin.txt_*.bmp(前半段核对用的参照图)。
  # 少了参照图驱动就说"核对用的图读不了",从零量前半段(P8A 就这样)。拷真文件,不用链接:驱动会把量到的写回去
  cp "$BASE" "$CALF"
  for f in "$BASE".geo.json "$BASE".kin.txt "$BASE".kin.txt_*.bmp; do [ -f "$f" ] && cp "$f" "$CALF${f#$BASE}"; done
  echo "身体文件:$BASE → $CALF(连 .geo.json、.kin.txt、参照图)" > "$E/meta.txt"
else
  echo "身体文件:没有(从零量,量到的写进 $CALF)" > "$E/meta.txt"
fi
echo "任务 $TASK · 炮 $K · 起 $(date +%T)" >> "$E/meta.txt"
echo "配置 ${BOOT_CFG:-arx_x5} · 种子 ${BOOT_SEED:-(任务默认)}" >> "$E/meta.txt"
VIDDIR=; [ -n "${BOOT_KEEP:-}" ] && VIDDIR=$N/vid
CAL=$CALF BL_LIFE=/root/p8/经历_$k.txt CFG=${BOOT_CFG:-arx_x5} DRVMODE=work BD_STEP_LIM=3000 BL_VID=$VIDDIR \
  setsid nohup env ${BOOT_SEED:+SEED=$BOOT_SEED} bash /root/q/run.sh 8 "$K" "$TASK" "$LIM" > "$E/run.log" 2>&1 < /dev/null &
RUN=$!
# 等:第 1 轮 / run.sh 自己结束(到时限、驱动退了)
got=""
while kill -0 $RUN 2>/dev/null; do
  if [ -f "$N/cal.log" ] && grep -q "── 第 1 轮" "$N/cal.log"; then got=yes; break; fi
  sleep 10
done
if [ -n "$got" ]; then
  sleep 20    # 让第 1 轮的图和清单落盘
  echo "到第 1 轮:$(date +%T)" >> "$E/meta.txt"
else
  echo "🔴 没到第 1 轮 run.sh 就结束了:$(date +%T)" >> "$E/meta.txt"
fi
if [ -f "$N/cal.log" ]; then
  grep -E "^\[装\]|^\[身\]|^\[认\] 相机|第 1 轮|🔴|挡|挪|配不|量不|解不|失败|错" "$N/cal.log" | head -120 > "$E/boot_lines.txt"
  awk '/── 第 1 轮/{f=1} f{print; n++} n>=80{exit}' "$N/cal.log" > "$E/round1.txt"
  wc -l "$N/cal.log" >> "$E/meta.txt"
fi
[ -f "$N/sim.log" ] && grep -E "Traceback|Error|error|Unstable|UnStable|Exception" "$N/sim.log" | grep -v -E "omni.kit.test|CXXABI|libXt|omni.graph|carb.windowing|MaterialX|usdBakeMtlx|circular import" | head -40 > "$E/sim_errors.txt"
G=$(ls "$N/look"/grid_*.bmp 2>/dev/null | head -1)
[ -n "$G" ] && /venv/RoboDojo/bin/python -c "import sys; from PIL import Image; Image.open(sys.argv[1]).convert('RGB').save(sys.argv[2], quality=85)" "$G" "$E/first_grid.jpg"
[ -f "$N/look/fixed_eye.txt" ] && cp "$N/look/fixed_eye.txt" "$E/"
# look/ 里的文字(给脑的话、脑交上来的)都是小文件,留下:核"那句话送到脑了没有"要看它(cal.log 里只记了有一片叫 instruction 的读数)
mkdir -p "$E/look_txt" && find "$N/look" -maxdepth 1 -name '*.txt' -size -200k -exec cp {} "$E/look_txt/" \; 2>/dev/null
# 放锁:run.sh 还在等(驱动还活着)才 touch;它已经自己结束了就不 touch(P8I 驱动先退了,touch 完留下一个没人删的 done_P8I)
kill -0 $RUN 2>/dev/null && touch /root/q/done_$K
wait $RUN
echo "run.sh 结束:$(date +%T) rc=$?" >> "$E/meta.txt"
# 清:shot 目录里 cal* / 经历* 以外的都删(BOOT_KEEP 的炮不删,打完分再删 vid/)
[ -z "${BOOT_KEEP:-}" ] && find "$N" -mindepth 1 -maxdepth 1 ! -name 'cal*' ! -name '经历*' -exec rm -rf {} +
# RoboDojo 这一集的结果目录:.../<任务>/l3_link/<配置名>/<种子>_/<ROBODOJO_RUN_ID = 炮名>(炮名只有这一炮用,只删它;bootcal 那几炮也删)
for d in /root/RoboDojo/eval_result/RoboDojo/$TASK/l3_link/*/*_; do [ -d "$d/$K" ] && rm -rf "$d/$K"; rm -f "$d/_resume_$K.json"; done
echo "清完:$(ls "$N")" >> "$E/meta.txt"

#!/bin/bash
# 路 8 开机炮:主线驱动(不设 BL_BIN)在排队位上开机 —— 走 /root/q/run.sh 排队;驱动日志走到"── 第 1 轮"(身体开完机、第一次叫脑)就放锁。不做任务。
# 用法:bash qboot.sh 炮名 任务 [身体文件(默认 /root/cal_v1b78.json;写 zero = 从零量)] [时限分钟(默认 14)]
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
  cp "$BASE" "$CALF"
  [ -f "$BASE.geo.json" ] && cp "$BASE.geo.json" "$CALF.geo.json"
  echo "身体文件:$BASE → $CALF" > "$E/meta.txt"
else
  echo "身体文件:没有(从零量,量到的写进 $CALF)" > "$E/meta.txt"
fi
echo "任务 $TASK · 炮 $K · 起 $(date +%T)" >> "$E/meta.txt"
CAL=$CALF BL_LIFE=/root/p8/经历_$k.txt CFG=arx_x5 DRVMODE=work BD_STEP_LIM=3000 BL_VID= \
  setsid nohup bash /root/q/run.sh 8 "$K" "$TASK" "$LIM" > "$E/run.log" 2>&1 < /dev/null &
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
touch /root/q/done_$K
wait $RUN
echo "run.sh 结束:$(date +%T)" >> "$E/meta.txt"
# 清:shot 目录里 cal* / 经历* 以外的都删
find "$N" -mindepth 1 -maxdepth 1 ! -name 'cal*' ! -name '经历*' -exec rm -rf {} +
case "$TASK" in   # RoboDojo 这一集的结果目录:.../<任务>/l3_link/<配置>/<种子>_/<ROBODOJO_RUN_ID = 炮名>(只删这一炮的)
  bd_*) rm -rf "/root/RoboDojo/eval_result/RoboDojo/$TASK/l3_link/arx_x5/0_/$K" "/root/RoboDojo/eval_result/RoboDojo/$TASK/l3_link/arx_x5/0_/_resume_$K.json" ;;
esac
echo "清完:$(ls "$N")" >> "$E/meta.txt"

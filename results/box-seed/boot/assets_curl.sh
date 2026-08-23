#!/usr/bin/env bash
# 资产下载 · 直接打 REST + 并行 curl。**四条路里唯一跑得动的那条**,理由全是实测:
#   ① 官方 `scripts/init_assets.sh`(git clone --sparse + git-lfs):**172 KB/s** ⇒ 30 GB 要 48 小时
#   ② `snapshot_download`:这仓 **74893 个文件**,时间全花在元数据枚举,几分钟**一字节没下**
#   ③ `HfApi().list_repo_files`:同一个病,列清单本身就卡住
#   ④ 本脚本:**tree API 翻页**(单页实测 **0.8 秒**)拿清单 → `xargs -P 64` 并行 curl
# 而**到 HF 的裸速率实测 28 MB/s**(单连接)⇒ 慢的从来不是网络,是那三个客户端。
set -u
REPO="RoboDojo-Benchmark/RoboDojo"
OUT="/root/RoboDojo"
LIST="/root/assets_files.txt"

if [ ! -s "$LIST" ]; then
  echo ">>> 翻页取文件清单"
  : > "$LIST"
  url="https://huggingface.co/api/datasets/${REPO}/tree/main/Assets?recursive=true"
  page=0
  while [ -n "$url" ]; do
    page=$((page + 1))
    hdr=$(mktemp)
    curl -sS -D "$hdr" "$url" -o /tmp/tree_page.json
    # 只要 type=file 的 path
    python3 -c "
import json,sys
for e in json.load(open('/tmp/tree_page.json')):
    if e.get('type')=='file': print(e['path'])
" >> "$LIST"
    # HF 用 Link 头翻页:<url>; rel=\"next\"
    url=$(grep -i '^link:' "$hdr" | sed -n 's/.*<\([^>]*\)>; rel="next".*/\1/p')
    rm -f "$hdr"
    echo "    第 $page 页,累计 $(wc -l < "$LIST") 个文件"
  done
fi

N=$(wc -l < "$LIST")
echo ">>> 并行下载(-P 64:文件平均 23 KB,瓶颈是每文件一次 HTTP 往返,不是带宽) $N 个文件"
export REPO OUT
# 🔴 已存在且非空的**跳过** —— 这条让脚本可以随时杀掉重跑而不重下。
xargs -P 64 -I FILE bash -c '
  f="$1"; d="$OUT/$f"
  [ -s "$d" ] && exit 0
  mkdir -p "$(dirname "$d")"
  curl -sSL --retry 3 --retry-delay 2 -o "$d" "https://huggingface.co/datasets/$REPO/resolve/main/$f"
' _ FILE < "$LIST"
echo "=== ASSETS EXIT $? ==="
du -sh "$OUT/Assets"
find "$OUT/Assets" -type f | wc -l

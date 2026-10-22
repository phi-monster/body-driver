#!/bin/bash
# fitcompare2.sh A B: per dumped fit, seconds, every sighting's median px, order and focal in A and in B (logs in A.d, B.d)
A=$1; B=$2
med() { grep -oE "every sighting's median [0-9.E+-]+ px" $1 2>/dev/null | tail -1 | awk '{printf "%.3f", $4}'; }
ord() { grep -oE "order of the joints along arm [0-9]+, [0-9 ]+, is (NOT )?told" $1 2>/dev/null | tail -1 | sed -E 's/order of the joints along arm [0-9]+, //; s/, is NOT told/?/; s/, is told/!/' | tr -d ' '; }
foc() { grep -oE "focal [0-9.]+ x [0-9.]+ px" $1 2>/dev/null | tail -1 | awk '{print $2}'; }
printf "%-26s %7s %7s | %6s %6s | %-9s %-9s | %6s %6s\n" dump a_s b_s a_px b_px a_ord b_ord a_f b_f
awk -F'\t' 'NR==FNR {if ($1 != "done") t[$1]=$2; next} $1 != "done" && ($1 in t) {print $1"\t"t[$1]"\t"$2}' $A $B | sort | while IFS=$'\t' read -r n ta tb; do
  la=$A.d/$n.log; lb=$B.d/$n.log
  printf "%-26s %7s %7s | %6s %6s | %-9s %-9s | %6s %6s\n" "$n" "$ta" "$tb" "$(med $la)" "$(med $lb)" "$(ord $la)" "$(ord $lb)" "$(foc $la)" "$(foc $lb)"
done
awk -F'\t' 'NR==FNR {if ($1 != "done") {a+=$2}; next} $1 != "done" {b+=$2} END {printf "total seconds: %s %.0f, %s %.0f (%.2fx)\n", "A", a, "B", b, a/b}' $A $B

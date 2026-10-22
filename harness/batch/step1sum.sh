#!/bin/bash
# step1sum.sh ROUNDDIR: per boot, the truth's arms with an eye of their own (body.txt) against the arms the driver's last
# recognition gave an eye (driver.log), and how far the boot got.
D=$1
for R in "$D"/*/; do
  n=$(basename "$R"); [ -f "$R/body.txt" ] || continue
  arms=$(grep -c "with an eye of its own" "$R/body.txt")
  base=$(grep -c "^group base:" "$R/body.txt"); motors=$(grep -c "^group motors:" "$R/body.txt"); joints=$(grep -c "^group joints:" "$R/body.txt")
  ln=$(grep -n "boot: the groups as recognized" "$R/driver.log" | tail -1 | cut -d: -f1)
  if [ -n "$ln" ]; then
    blk=$(sed -n "$((ln+1)),$((ln+30))p" "$R/driver.log")
    rarms=$(echo "$blk" | grep -cE "^group [0-9]+: .*ARM \(arm")
    carried=$(echo "$blk" | grep -cE "^eye [0-9]+: .*ARM_CARRIED")
    roles=$(echo "$blk" | grep -oE "^group [0-9]+: [0-9]+ values, (commandable|not commandable), [A-Z_]+" | awk '{print $3"v-"$NF}' | tr '\n' ' ')
  else rarms="-"; carried="-"; roles="(not recognized)"; fi
  phase=$(grep -oE "after (recognizing the groups|sweeping the arms|placing the fixed eyes)" "$R/driver.log" | tail -1 | sed 's/after //')
  fitted=$(grep -cE "kinematics: arm [0-9]+ fitted from" "$R/driver.log"); told=$(grep -cE "order of the joints along arm [0-9]+.* is told" "$R/driver.log")
  last=$(tail -1 "$R/driver.log" | grep -oE '^@[0-9]+')
  printf "%-6s truth arms+eye %d%s | recognized arms %s, eyes carried %s | %-22s | fits %d told %d | %s | %s\n" "$n" "$arms" \
    "$( [ "$base" -gt 0 ] && echo ' +base' )$( [ "$motors" -gt 0 ] && echo ' (drone)' )$( [ "$joints" -gt 0 ] && echo ' (humanoid)' )" \
    "$rarms" "$carried" "${phase:-running}" "$fitted" "$told" "$last" "$roles"
done

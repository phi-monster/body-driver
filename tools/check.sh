#!/usr/bin/env bash
# The driver's gates. Every merge and every install runs them; any failure stops it.
#
#   build      the driver and its tools build without warnings (first: the gates are Ada tools)
#   numbers    every literal has a stated origin and none is a tuning number (driver/bin/numbers)
#   names      no benchmark, scene, task or robot name in driver code
#   python     no Python anywhere under driver/
#   english    no CJK characters in driver sources, tests or tools (code, comments and strings)
#   contact    no action words in the contact-set packages: one search serves every task
#   prompt     no tutorial sentences in what the brain is shown: format only, never how to act
#   motion     commands reach the robot from one place only, Driver.Robot.Motion
#   selftest   every behavior specification passes
#   deadcode   what body_driver cannot reach (reported while the layers are being written in
#              parallel; binding once they are wired together)
#
# Usage: tools/check.sh [--no-build]   (--no-build uses the binaries of the last build)
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
D="$ROOT/driver"
fail=0
red() { echo "FAIL  $1"; fail=1; }
ok() { echo "pass  $1"; }

code_of() { sed -E 's/--.*$//' "$@"; }                                 # Ada code without comments
sources() { find "$D/src" -name '*.ad[sb]' ! -name '*-tests.ad[sb]'; }  # driver code, self tests excluded

# build
if [ "${1:-}" != "--no-build" ]; then
  ALR="${ALR:-$(command -v alr || echo "$HOME/alire/bin/alr")}"
  out=$(cd "$D" && "$ALR" -n build 2>&1); status=$?
  #  A compiler message reads "file:line:col: error: ..." or "... warning: ..."; a bare match would also
  #  take a file whose name holds the word (driver-robot-kinematics-errors.adb). A build can also stop
  #  with neither ("compilation abandoned" when a unit needs what the toolchain lacks): its status says so.
  if echo "$out" | grep -qE '(^|: )(error|warning)[: ]'; then echo "$out" | grep -E '(^|: )(error|warning)[: ]' | head -20; red "build"
  elif [ "$status" -ne 0 ]; then echo "$out" | tail -5; red "build (status $status)"
  else ok "build"; fi
fi

# numbers
if (cd "$ROOT" && "$D/bin/numbers" check > /tmp/bd_numbers.$$ 2>&1); then ok "numbers"; else cat /tmp/bd_numbers.$$; red "numbers"; fi
rm -f /tmp/bd_numbers.$$

# names
NAMES='robodojo|isaac|libero|calvin|general_pickup|stack_blocks|pack_objects|store_tools|eval_result|_result\.json|franka|arx|x5|unitree|\bg1\b|inspire|lekiwi|so100|so101|scissors|bootcal'
hits=$(for f in $(sources); do code_of "$f" | grep -niE "$NAMES" | sed "s#^#$(basename "$f"):#"; done)
if [ -n "$hits" ]; then echo "$hits" | head -20; red "names"; else ok "names"; fi

# python
py=$(find "$D" -name '*.py' -not -path '*/obj/*' 2>/dev/null)
if [ -n "$py" ]; then echo "$py"; red "python"; else ok "python"; fi

# english
cjk=$(find "$D/src" "$D/tests" "$D/tools" -name '*.ad[sb]' -exec perl -CSD -ne 'print "$ARGV:$.: $_" if /[\x{3000}-\x{303f}\x{3400}-\x{9fff}\x{ff00}-\x{ffef}]/; close ARGV if eof' {} + 2>/dev/null)
if [ -n "$cjk" ]; then echo "$cjk" | head -20; red "english"; else ok "english"; fi

# contact: identifiers split at underscores, and words in strings, of the contact-set packages
WORDS='lift lifts lifted lifting push pushes pushed pushing pull pulls pulled pulling turn turns turned turning
grasp grasps grasped grasping grab grabs grabbed grabbing hold holds holding held pinch pinches pinched pinching
pry pries pried prying flip flips flipped flipping pour pours poured pouring carry carries carried carrying
drag drags dragged dragging dodge dodges dodged dodging throw throws threw thrown wipe wipes wiped wiping
scoop scoops scooped scooping stack stacks stacked stacking'
contact_files=$(find "$D/src/action" -name 'driver-action-contact*.ad[sb]' ! -name '*-tests.ad[sb]' 2>/dev/null)
if [ -n "$contact_files" ]; then
  hits=$(WORDS="$WORDS" perl -ne 'BEGIN { %w = map { $_ => 1 } split /\s+/, $ENV{WORDS} } s/--.*$//; for $t (split /[^A-Za-z]+/, lc $_) { print "$ARGV:$.: $t\n" if $w{$t} } close ARGV if eof' $contact_files)
  if [ -n "$hits" ]; then echo "$hits" | head -20; red "contact"; else ok "contact"; fi
else
  ok "contact (no contact-set packages yet)"
fi

# prompt
prompt_files=$(find "$D/src/brain" -name '*.ad[sb]' ! -name '*-tests.ad[sb]' 2>/dev/null)
hits=$(for w in "e.g." "for example" "how you " "To close" "To lift" "pick it up" "from above" "is how you" "first move" "then close"; do
  for f in $prompt_files; do code_of "$f" | grep -niF -- "$w" | sed "s#^#$(basename "$f"): [$w] #"; done; done)
if [ -n "$hits" ]; then echo "$hits" | head -20; red "prompt"; else ok "prompt"; fi

# motion: Driver.Beats.Send is called only by Driver.Robot.Motion
hits=$(for f in $(sources); do b=$(basename "$f"); [[ "$b" == driver-robot-motion* || "$b" == driver-beats.ad? ]] && continue
  code_of "$f" | grep -nE '\bBeats\.Send\b|\bSend[[:space:]]*\(' | grep -E 'Beats' | sed "s#^#$b:#"; done)
if [ -n "$hits" ]; then echo "$hits" | head -20; red "motion"; else ok "motion"; fi

# selftest and dead code
#  Each test alone and several at a time (tools/selftests.sh): one process for the whole suite took
#  45 minutes of every merge.
if "$ROOT/tools/selftests.sh" bin > /tmp/bd_selftest.$$ 2>&1; then ok "selftest ($(tail -1 /tmp/bd_selftest.$$))"; else head -40 /tmp/bd_selftest.$$; red "selftest"; fi
rm -f /tmp/bd_selftest.$$
echo "info  deadcode: $(cd "$ROOT" && "$D/bin/deadcode" | head -1 | sed 's/^== //; s/ ==$//')"

[ "$fail" = 0 ] && echo "all gates pass" || { echo "gates failed"; exit 1; }

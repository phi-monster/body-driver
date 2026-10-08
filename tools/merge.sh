#!/usr/bin/env bash
# Merge one path's branch into the current branch and run every gate.
#
#   tools/merge.sh BRANCH "message"
#
# The registry (driver/numbers.tsv) merges by union (.gitattributes); after the
# merge driver/bin/numbers gen drops rows whose literal no longer occurs. After the
# gates the self tests run once more built with AddressSanitizer (BD_BUILD=asan), which
# catches memory corruption the checked build only shows as a later crash. Any other
# conflict, a new unclassified literal, a failed gate or a sanitizer report stops here
# with the working tree left as it is, for a person to resolve. Then every self test runs on the box
# (tools/boxtest.sh); if one fails there the merge commit is taken back out (exit 8).
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
BRANCH="$1"; MESSAGE="$2"
if ! git diff --quiet || ! git diff --cached --quiet; then
  echo "the working tree has changes; commit or stash them first"; exit 2
fi
if ! git merge --no-ff --no-commit "$BRANCH"; then
  echo "conflicts outside the registry: resolve them, then rerun the gates"; exit 3
fi
ALR="${ALR:-$(command -v alr || echo "$HOME/alire/bin/alr")}"
if ! (cd driver && "$ALR" -n build > /tmp/bd_merge_build.$$ 2>&1); then
  grep -E 'error' /tmp/bd_merge_build.$$ | head -20; rm -f /tmp/bd_merge_build.$$
  echo "the merged sources do not build"; exit 6
fi
rm -f /tmp/bd_merge_build.$$
driver/bin/numbers gen > /dev/null
if awk -F'\t' '$4 == "undecided" { found = 1 } END { exit !found }' driver/numbers.tsv; then
  awk -F'\t' '$4 == "undecided"' driver/numbers.tsv | head -20
  echo "the branch brought unclassified literals"; exit 4
fi
git add driver/numbers.tsv
if ! tools/check.sh; then
  echo "gates failed after the merge; nothing committed"; exit 5
fi
# use_sigaltstack=0: the sanitizer's own alternate signal stack cannot be unmapped from a GNAT task.
if ! (cd driver && "$ALR" -n build -- -XBD_BUILD=asan > /tmp/bd_asan_build.$$ 2>&1) \
   || ! ASAN_OPTIONS=use_sigaltstack=0 tools/selftests.sh bin-asan > /tmp/bd_asan.$$ 2>&1; then
  grep -E "error|ERROR" /tmp/bd_asan_build.$$ | head -10; grep -v "^ " /tmp/bd_asan.$$ | head -20
  grep -A5 "ERROR: AddressSanitizer" /tmp/bd_asan.$$ | head -20; rm -f /tmp/bd_asan_build.$$ /tmp/bd_asan.$$
  echo "the self tests fail under AddressSanitizer; nothing committed"; exit 7
fi
rm -f /tmp/bd_asan_build.$$ /tmp/bd_asan.$$
git commit -q -m "$MESSAGE

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
git log -1 --format="merged %h"
# The driver runs on the box, x86-64 Linux, where a call's actuals are evaluated in another order
# and no multiply-add is fused: every self test runs there as well (tools/boxtest.sh). A failure
# there takes the merge back out before anything is pushed.
if ! "$ROOT/tools/boxtest.sh" HEAD; then
  git reset -q --hard HEAD~1
  echo "the self tests fail on the box; the merge is undone and nothing is pushed"; exit 8
fi
# The owner keeps GitHub current (10-05): every merge is pushed. A failed push leaves the merge in place.
if ! git push -q origin HEAD:main; then
  echo "merged, but the push to origin failed; push by hand"
fi

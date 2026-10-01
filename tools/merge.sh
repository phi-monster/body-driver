#!/usr/bin/env bash
# Merge one path's branch into the current branch and run every gate.
#
#   tools/merge.sh BRANCH "message"
#
# The registry (driver/numbers.tsv) merges by union (.gitattributes); after the
# merge numbers.py gen drops rows whose literal no longer occurs. Any other
# conflict, a new unclassified literal, or a failed gate stops here with the
# working tree left as it is, for a person to resolve.
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
python3 tools/numbers.py gen > /dev/null
if awk -F'\t' '$4 == "undecided" { found = 1 } END { exit !found }' driver/numbers.tsv; then
  awk -F'\t' '$4 == "undecided"' driver/numbers.tsv | head -20
  echo "the branch brought unclassified literals"; exit 4
fi
git add driver/numbers.tsv
if ! tools/check.sh; then
  echo "gates failed after the merge; nothing committed"; exit 5
fi
git commit -q -m "$MESSAGE

Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>"
git log -1 --format="merged %h"

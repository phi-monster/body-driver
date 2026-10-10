#!/bin/bash
# Puts the scrambler (bd_scramble.py) beside the simulator's policy adapter and checks the adapter calls it: every
# observation it sends the driver goes through scramble_obs, every action it takes through unscramble_action.
#   bash install.sh [ROBODOJO_ROOT]   (default /root/RoboDojo)
set -eu
ROOT=${1:-/root/RoboDojo}
DST=$ROOT/XPolicyLab/policy/l3_link
cp "$(dirname "$0")/bd_scramble.py" "$DST/bd_scramble.py"
grep -q "from . import bd_scramble as _scr" "$DST/deploy.py" || { echo "deploy.py does not import the scrambler"; exit 2; }
[ "$(grep -c "_scr.scramble_obs(" "$DST/deploy.py")" -ge 2 ] || { echo "deploy.py sends an observation unscrambled"; exit 2; }
grep -q "take_action(_scr.unscramble_action(action))" "$DST/deploy.py" || { echo "deploy.py takes an action unscrambled"; exit 2; }
(cd "$DST" && /venv/RoboDojo/bin/python bd_scramble.py)

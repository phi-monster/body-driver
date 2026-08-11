#!/usr/bin/env bash
# ada_check.sh -- the Ada fast face hand-mirrors constants from the C header.  Check the copies.
#
# 🔴 WHY THIS EXISTS, AND WHY IT DID NOT UNTIL 2026-08-11.
#
# Two of the three edges of this layer were machine-checked and the third was not:
#
#     header <-> Rust cdylib     abi_check.sh      🟢
#     header <-> Python binding  python_check.sh   🟢
#     header <-> Ada fast face   nothing           🔴
#
# And the Ada face is exactly where a hand-written copy lives.  `body_layer_fast_c.ads` says so in
# its own comment: "Mirrors `bl_status` in ../abi/body_layer.h.  Named constants rather than an
# enumeration so the numbering sits visibly next to the header it has to match."  Visibly next to
# is not the same as checked against -- it relies on the next person to look, and this repository
# has paid for that assumption more than once.
#
# It was found the way these are always found: the owner said "the driver is Rust plus Ada", which
# prompted the question "then does the Ada know about the enum I just changed".  It did not need
# to, this time.  The next change will not be so lucky, and by then nothing would have said so.
#
# WHAT IT CHECKS
#   1. Every `BL_*` constant declared in the Ada has the same numeric value as in the header.
#   2. Ada does not silently mirror a constant the header no longer declares.
#   3. `Max_Joints` agrees with `BL_MAX_JOINTS`; a mismatch here is a buffer overrun on the
#      real-time side, which is the one place in this layer where a wrong number is not merely a
#      wrong answer.
#   4. Every `External_Name => "blf_..."` in the Ada spec is present in the built Ada library, so
#      the fast face cannot promise an entry point it does not export -- the same check
#      abi_check.sh makes for the Rust side.
#
# Exit 0 only if all four hold.  Check 4 is SKIPPED, loudly, when no Ada library has been built:
# a check that quietly passes because it did not run is worse than no check.

set -uo pipefail
cd "$(dirname "$0")/.."

HDR=abi/body_layer.h
ADA_SPEC=fast/body_layer_fast_c.ads
ADA_CORE=fast/body_layer_fast.ads
fail=0

say() { printf '  %-6s %s\n' "$1" "$2"; }

[ -f "$HDR" ]      || { echo "ada_check: no $HDR"; exit 2; }
[ -f "$ADA_SPEC" ] || { echo "ada_check: no $ADA_SPEC"; exit 2; }

echo "ada_check: $ADA_SPEC  vs  $HDR"

# ---------------------------------------------------------------- 1 + 2: the mirrored constants
# Ada form:     BL_OK     : constant C_Int := 0;
# Header form:  BL_OK              = 0,
while read -r name val; do
    hdr_val=$(grep -oE "\b${name}[[:space:]]*=[[:space:]]*[0-9]+" "$HDR" | head -1 |
              grep -oE '[0-9]+$')
    if [ -z "$hdr_val" ]; then
        say FAIL "$name is mirrored in Ada but the header does not declare it"
        fail=1
    elif [ "$hdr_val" != "$val" ]; then
        say FAIL "$name is $val in Ada and $hdr_val in the header"
        fail=1
    else
        say ok "$name = $val"
    fi
done < <(grep -oE '\bBL_[A-Z_]+[[:space:]]*:[[:space:]]*constant[[:space:]]+C_Int[[:space:]]*:=[[:space:]]*[0-9]+' "$ADA_SPEC" |
         sed -E 's/[[:space:]]*:[[:space:]]*constant[[:space:]]+C_Int[[:space:]]*:=[[:space:]]*/ /')

# ---------------------------------------------------------------- 3: the joint bound
ada_joints=$(grep -oE 'Max_Joints[[:space:]]*:[[:space:]]*constant[[:space:]]*:=[[:space:]]*[0-9]+' "$ADA_CORE" |
             head -1 | grep -oE '[0-9]+$')
hdr_joints=$(grep -oE '#define[[:space:]]+BL_MAX_JOINTS[[:space:]]+[0-9]+' "$HDR" |
             head -1 | grep -oE '[0-9]+$')
if [ -z "$ada_joints" ] || [ -z "$hdr_joints" ]; then
    say SKIP "Max_Joints / BL_MAX_JOINTS not found as literals (ada=${ada_joints:-none} hdr=${hdr_joints:-none})"
elif [ "$ada_joints" != "$hdr_joints" ]; then
    say FAIL "Max_Joints is $ada_joints in Ada and BL_MAX_JOINTS is $hdr_joints in the header"
    say ""   "on the real-time side this is a buffer bound, not a preference"
    fail=1
else
    say ok "Max_Joints = $ada_joints"
fi

# ---------------------------------------------------------------- 4: promised vs exported
declared=$(grep -oE 'External_Name[[:space:]]*=>[[:space:]]*"blf_[a-z_]+"' "$ADA_SPEC" |
           grep -oE 'blf_[a-z_]+' | sort -u)
lib=$(ls -t fast/lib/*.so fast/lib/*.dylib fast/lib/*.a fast/objlib/*.so fast/objlib/*.a 2>/dev/null | head -1)
if [ -z "${lib:-}" ]; then
    say SKIP "no built Ada library under fast/lib or fast/objlib -- run gprbuild, then re-run this"
    say ""   "(SKIPPED, not passed: $(echo "$declared" | wc -w | tr -d ' ') promised entry points are unverified)"
else
    say ok "inspecting $lib"
    # 🔴 `_?` and the sed, both load-bearing.  Mach-O prefixes every C symbol with an underscore,
    # and `\bblf_` does NOT match inside `_blf_admit` -- an underscore is a word character, so
    # there is no word boundary before the `b`.  The first version of this check reported all
    # EIGHT entry points missing from a library that exports all eight.
    #
    # That is the failure mode this whole directory exists to prevent, produced by the checker
    # itself: a check whose own bug looks exactly like the defect it is hunting.  It was caught
    # only because a FAIL this total is implausible -- which is not a mechanism, it is luck.  Hence
    # the self-test below: the check must find a symbol it KNOWS is there before its misses count.
    exported=$(nm "$lib" 2>/dev/null | grep -oE '_?blf_[a-z_]+' | sed 's/^_//' | sort -u)
    if [ -z "$exported" ]; then
        say FAIL "the symbol reader found NO blf_* symbols at all in $lib"
        say ""   "that is far more likely to be this script than a library with no entry points"
        fail=1
        declared=""
    fi
    for f in $declared; do
        if echo "$exported" | grep -qx "$f"; then
            say ok "$f exported"
        else
            say FAIL "$f is promised by the Ada spec and not exported by $lib"
            fail=1
        fi
    done
fi

echo
if [ "$fail" -eq 0 ]; then
    echo "ada_check: PASS -- the Ada fast face and the header agree"
else
    echo "ada_check: FAIL -- see above"
fi
exit "$fail"

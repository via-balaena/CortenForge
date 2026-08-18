#!/usr/bin/env bash
# Drives `apt.sh` against a stubbed `apt-get`, asserting exit status AND which
# annotation it emits, for every branch it can take.
#
# ⚠ WHY THIS IS A FILE. The first version of `apt.sh` was verified by running these
# cases by hand in a terminal. That verification existed only in a transcript: the
# next person to edit the script had no way to run what had been run. It also caught
# two real defects — a `rc=$?`-after-`if` that reported every timeout as "not a
# timeout", and an annotation title containing a comma — so it is not ceremony.
#
# ⚠ WHAT IT CANNOT CHECK: that GitHub *parses* the annotations it asserts are emitted,
# and that `sudo timeout` beats `timeout sudo` on a real runner (the stub `sudo` execs
# its argument, so both bound here). Those are argued in `apt.sh`, not tested.
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"
SCRIPT="$PWD/apt.sh"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT
mkdir -p "$WORK/bin"

# A real GNU `timeout` is needed. Runners have it; macOS ships it as `gtimeout`.
TIMEOUT_BIN="$(command -v timeout || command -v gtimeout || true)"
if [ -z "$TIMEOUT_BIN" ]; then
  echo "SKIP: no timeout(1) available" >&2
  exit 0
fi
printf '#!/bin/sh\nexec "$@"\n' > "$WORK/bin/sudo"
printf '#!/bin/sh\nexec "%s" "$@"\n' "$TIMEOUT_BIN" > "$WORK/bin/timeout"
chmod +x "$WORK/bin/sudo" "$WORK/bin/timeout"

fails=0
# run_case <label> <expected-exit> <expected-annotation-substring> <apt-stub-body> [packages]
run_case() {
  local label="$1" want_rc="$2" want_note="$3" stub="$4" pkgs="${5-pkg-a pkg-b}"
  printf '#!/bin/sh\n%s\n' "$stub" > "$WORK/bin/apt-get"
  chmod +x "$WORK/bin/apt-get"
  rm -f "$WORK/state" "$WORK/istate"
  local out rc=0
  out=$(cd "$WORK" && PATH="$WORK/bin:$PATH" PACKAGES="$pkgs" \
        APT_ATTEMPT_TIMEOUT=2 APT_RETRY_SLEEP=0 bash "$SCRIPT" 2>&1) || rc=$?
  local ok=true
  [ "$rc" = "$want_rc" ] || ok=false
  if [ -n "$want_note" ]; then
    grep -q -- "$want_note" <<< "$out" || ok=false
  fi
  if $ok; then
    printf 'ok       %s\n' "$label"
  else
    printf 'NOT OK   %s\n         wanted exit=%s note=%q\n         got    exit=%s\n%s\n' \
      "$label" "$want_rc" "$want_note" "$rc" "${out//$'\n'/$'\n'         | }"
    fails=$((fails + 1))
  fi
}

# ⚠ Single quotes are REQUIRED here, not an oversight: these are stub script BODIES,
# written verbatim into a file that runs later. `$1` must reach the stub unexpanded.
# shellcheck disable=SC2016
HANG_UPDATE='case "$1" in update) sleep 30;; *) exit 0;; esac'
# shellcheck disable=SC2016
ERR_UPDATE='case "$1" in update) exit 42;; *) exit 0;; esac'
# shellcheck disable=SC2016
BLIP_UPDATE='case "$1" in update) if [ -f state ]; then exit 0; else : > state; sleep 30; fi;; *) exit 0;; esac'
# shellcheck disable=SC2016
HANG_INSTALL='case "$1" in update) exit 0;; *) sleep 30;; esac'
# shellcheck disable=SC2016
ERR_INSTALL='case "$1" in update) exit 0;; *) exit 100;; esac'
# shellcheck disable=SC2016
BLIP_INSTALL='case "$1" in update) exit 0;; *) if [ -f istate ]; then exit 0; else : > istate; sleep 30; fi;; esac'

run_case "happy path"                    0 ''                                    'exit 0'
run_case "update hangs twice"            1 'title=apt mirror degradation'        "$HANG_UPDATE"
run_case "update hang is named a HANG"   1 'title=apt-get update hung'           "$HANG_UPDATE"
run_case "update errors twice"           1 'exited 42 on both attempts'          "$ERR_UPDATE"
run_case "update error is NOT a timeout" 1 'exited 42 (not a timeout)'           "$ERR_UPDATE"
run_case "update blips then succeeds"    0 'title=apt-get update hung'           "$BLIP_UPDATE"
run_case "install hangs twice"           1 'title=apt mirror degradation'        "$HANG_INSTALL"
run_case "install hang is named a HANG"  1 'title=apt-get install hung'          "$HANG_INSTALL"
run_case "install errors twice"          1 'exited 100 (not a timeout)'          "$ERR_INSTALL"
# ★ install RETRIES, symmetrically with update. An earlier revision gave update two
# attempts and install one — an asymmetry with no reason behind it, which cost a CI
# run on this change's own first PR where `install` was the call that hung.
run_case "install blips then succeeds"   0 'title=apt-get install hung'          "$BLIP_INSTALL"
run_case "packages empty"                1 'title=no packages given'             'exit 0' ''
run_case "packages whitespace only"      1 'title=no packages given'             'exit 0' "$(printf ' \t\n ')"

# ★ Glob safety, and the fixture is the whole point. Bash leaves an UNMATCHED glob
# literal, so passing `lib*` in a directory containing no `lib*` file proves nothing —
# an earlier hand-check did exactly that and was vacuous. A matching file must exist
# for the unquoted-expansion bug to be observable at all.
# shellcheck disable=SC2016  # a stub BODY: `$1`/`$@`/`$RECORD` must reach it unexpanded
printf '#!/bin/sh\nif [ "$1" != update ]; then printf "%%s\\n" "$@" > "$RECORD"; fi\nexit 0\n' > "$WORK/bin/apt-get"
chmod +x "$WORK/bin/apt-get"
: > "$WORK/libDECOY"
RECORD="$WORK/args"
rm -f "$RECORD"
( cd "$WORK" && PATH="$WORK/bin:$PATH" PACKAGES="lib* real-pkg" RECORD="$RECORD" \
  APT_ATTEMPT_TIMEOUT=2 APT_RETRY_SLEEP=0 bash "$SCRIPT" >/dev/null 2>&1 )
if grep -qx 'libDECOY' "$RECORD" 2>/dev/null; then
  echo "NOT OK   the package list GLOBBED — 'lib*' expanded to a filename"
  fails=$((fails + 1))
elif grep -qx 'lib\*' "$RECORD" 2>/dev/null; then
  echo "ok       package list is not glob-expanded"
else
  echo "NOT OK   glob fixture did not record apt-get's arguments at all"
  fails=$((fails + 1))
fi

# The production bound is pinned here, not just used: the override above exists for
# speed, and a silent change to the default would otherwise go unnoticed.
if ! grep -q 'APT_ATTEMPT_TIMEOUT:-300' "$SCRIPT"; then
  echo "NOT OK   the default attempt timeout is no longer 300 s — update this assertion deliberately"
  fails=$((fails + 1))
else
  echo "ok       default attempt timeout is 300 s"
fi

# ⚠ NO COMMAS in an annotation title: `::error name=value,name=value::message` means a
# comma inside a value truncates it. Checked structurally because no behavioural test
# can — this script asserts strings are emitted, not that GitHub parses them.
# Split on the `::` DELIMITER, not on a bare colon. A first version used
# `[^:]*`, which stops at any colon — so `title=apt install: failed, badly` had its
# comma hidden behind the colon and the guard passed. Verified by mutation.
comma_in_title=false
while IFS= read -r line; do
  after_open="${line#*\"::}"      # drop everything up to the opening `::`
  title_part="${after_open%%::*}"  # keep everything before the closing `::`
  if [[ "$title_part" == *,* ]]; then
    echo "NOT OK   annotation properties contain a comma — GitHub truncates there: $title_part"
    comma_in_title=true
  fi
done < <(grep 'echo "::' "$SCRIPT")
if $comma_in_title; then
  fails=$((fails + 1))
else
  echo "ok       no annotation title contains a comma"
fi

if [ "$fails" -ne 0 ]; then
  echo "FAILED: $fails case(s)" >&2
  exit 1
fi
echo "all cases passed"

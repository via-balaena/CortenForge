#!/usr/bin/env bash
# Bounded `apt-get`, called by this directory's `action.yml`.
#
# ⚠ WHY THIS EXISTS, measured rather than supposed. An unbounded `apt-get update`
# does not merely waste a runner — it converts a mirror hiccup into a failure that
# reports as something else entirely:
#
#   PR #783, attempt 1, `Tests (release, heavy — shard 2/3)`
#     started   16:43:28Z
#     completed 17:28:55Z          <- 45 min 27 s, the job's whole timeout
#     cancelled   Install system dependencies
#     skipped     Run tests (nextest, --release)   <- never ran a single test
#
#   The run reported `cancelled` and the aggregating `Quality Gate` job reported
#   `failure`, on a PR that touches the solver. It read like a test failure and was
#   not one: `apt-get update` contacts every configured mirror and had stopped making
#   progress. The same shard passed in 74 s on a re-run. #778/#779 hit this before.
#
# ⚠ A standalone script rather than an inline `run:` block so it can be shellchecked
# and TESTED as a file — see `test.sh`, which drives it against a stubbed `apt-get`.
# An earlier revision inlined it, and the verification then lived only in a
# transcript.
#
# Env:
#   PACKAGES              space-separated apt package list (required)
#   APT_ATTEMPT_TIMEOUT   seconds per apt-get invocation. TEST-ONLY override; the
#                         production value is the default below and `test.sh` pins it.
set -euo pipefail

# 300 s per attempt. Measured healthy on this repo's jobs: 41 s to 177 s, so this is
# ~1.7x the slowest good run — loose enough not to fail a slow-but-working mirror,
# tight enough that the pathological case costs minutes rather than a job budget.
ATTEMPT_TIMEOUT="${APT_ATTEMPT_TIMEOUT:-300}"

# ⚠ `required: true` in `action.yml` is DOCUMENTATION, not enforcement — GitHub does
# not reject a composite-action call that omits an input. Without this check a caller
# who forgot `packages:` gets `apt-get install -y` with no arguments, which SUCCEEDS
# and installs nothing; the job then fails much later with a confusing linker error.
#
# `[^[:space:]]` rather than stripping literal spaces: a YAML block scalar can deliver
# a tab or a newline, and `apt-get install -y $'\t'` also succeeds installing nothing.
if [[ ! "${PACKAGES:-}" =~ [^[:space:]] ]]; then
  echo "::error title=no packages given::install-linux-deps was called without a non-empty \`packages:\` input. It would have installed nothing and passed."
  exit 1
fi

# ⚠ ONE code path for every apt invocation. An earlier revision retried `update`
# twice and gave `install` a single attempt — an asymmetry with no reason behind it,
# which existed only because the two were written separately. It cost a CI run on
# this change's own first PR, where `install` was the call that hung. A helper makes
# the policy structural instead of duplicated.
#
# One retry, and deliberately not more. The evidence says retrying is worth little:
# on #783 the mirror stayed slow for minutes and only a re-run ~35 min later
# succeeded. It is cheap insurance against a brief blip; it is NOT what makes this
# safe. The BOUND is.
#
# ⚠ `sudo timeout apt-get`, NOT `timeout sudo apt-get`. The first runs the timer as
# root so it signals apt-get directly; the second signals `sudo`, which can leave apt
# running on past the bound. Do not "tidy" this.
#
# ⚠⚠ `rc=0; cmd || rc=$?` and NOT `if cmd; then ...; fi; rc=$?`. After an `if` whose
# condition FAILED and which has no `else`, `$?` is the status of the `if` statement
# itself — zero. A first draft did that and every timeout reported "exited 0 (not a
# timeout)": the message was confidently wrong in exactly the case it was added for.
#
# ⚠ NO COMMAS in an annotation title. The workflow-command format is
# `::error name=value,name=value::message`, so a comma inside a value terminates it.
# A local test cannot catch this: it can check the string is EMITTED, not that GitHub
# parses it — hence the structural check in `test.sh`.
run_apt() {
  local label="$1"
  shift
  local rc=0 last_rc=0 attempt
  for attempt in 1 2; do
    rc=0
    sudo timeout "$ATTEMPT_TIMEOUT" apt-get "$@" || rc=$?
    if [ "$rc" -eq 0 ]; then
      return 0
    fi
    # `timeout` exits 124 when it fires. Worth separating: a HANG is mirror
    # degradation and a re-run usually clears it, while a non-124 failure is apt
    # itself (a stale index 404ing, say) and re-running will not help.
    if [ "$rc" -eq 124 ]; then
      echo "::warning title=apt-get $label hung::attempt $attempt exceeded ${ATTEMPT_TIMEOUT}s and was killed"
    else
      echo "::warning title=apt-get $label failed::attempt $attempt exited $rc (not a timeout)"
    fi
    last_rc=$rc
    sleep "${APT_RETRY_SLEEP:-20}"
  done

  # ⚠ The summary must match what actually happened. A first draft said "exceeded Ns
  # twice — mirror degradation" on EVERY path, including a clean non-timeout error,
  # which is the wrong diagnosis and points at the wrong fix.
  if [ "$last_rc" -eq 124 ]; then
    echo "::error title=apt mirror degradation (not a code failure)::apt-get $label exceeded ${ATTEMPT_TIMEOUT}s on both attempts. This is infrastructure: no test has run yet. Re-running usually clears it."
  else
    echo "::error title=apt-get $label failed (not a code failure)::apt-get $label exited $last_rc on both attempts without timing out. This is apt itself, not a slow mirror — re-running will probably NOT help. A stale package index is the usual cause."
  fi
  return 1
}

run_apt update update

# ⚠ Split into an ARRAY rather than relying on bare `$PACKAGES`. The list does need
# splitting, but an unquoted expansion is also subject to GLOBBING — a package name
# containing `*` or `?` would expand against the working directory. The array form
# splits on whitespace only, and needs no `shellcheck disable` to say so.
read -ra pkgs <<< "$PACKAGES"
run_apt install install -y "${pkgs[@]}"

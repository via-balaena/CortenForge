#!/usr/bin/env bash
# Drives `msrv.sh` against a stubbed `rustup` and `rustc`, asserting exit status AND
# what it emits, for every branch it can take.
#
# ⚠ WHY THIS IS A FILE. The logic it covers lived inline in `scheduled.yml`, where it
# could be neither linted nor run; its first verification existed only in a
# transcript. Driving it here found that a guard counting the `rust-version` LINE is
# not the same as reading it — a trailing comment or a CRLF ending satisfies the count
# and yields an empty version — and mutation showed the assertion in `msrv.sh` is the
# only thing standing between a silently-wrong toolchain and a green tick.
#
# ⚠ WHAT IT CANNOT CHECK: that a real `rustup toolchain install` 404s on a version
# that does not exist (the stub emulates it; the real behaviour was measured against
# rustup 1.29.0 locally and in the 2026-08-18 CI log), and that `cargo +X` beats
# `rust-toolchain.toml` on a runner. Those are argued in `msrv.sh`, not tested here.
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"
SCRIPT="$PWD/msrv.sh"
GITHUB_DIR="$PWD/../.."
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT
mkdir -p "$WORK/bin"

# ⚠ The STUB_* variables below are read by the STUBS, never by the script under test.
# Nothing here adds a knob to production — contrast a test-only override, which would
# have to be defaulted in `msrv.sh` itself.
cat > "$WORK/bin/rustup" <<'STUB'
#!/bin/sh
if [ "${STUB_RUSTUP_RC:-0}" -ne 0 ]; then
  echo "error: could not download nonexistent rust version \`$3\`" >&2
  exit "${STUB_RUSTUP_RC}"
fi
exit 0
STUB
cat > "$WORK/bin/rustc" <<'STUB'
#!/bin/sh
echo "rustc ${STUB_RUSTC_VERSION:-1.92.0} (stubbed 2026-01-01)"
STUB
chmod +x "$WORK/bin/rustup" "$WORK/bin/rustc"

fails=0
# run_case <label> <expected-exit> <expected-substring> <manifest> [rustup-rc] [rustc-version]
run_case() {
  local label="$1" want_rc="$2" want_note="$3" manifest="$4"
  local stub_rc="${5:-0}" stub_ver="${6:-1.92.0}"
  printf '%s' "$manifest" > "$WORK/Cargo.toml"
  rm -f "$WORK/out" "$WORK/sum"
  local out rc=0
  out=$(cd "$WORK" && PATH="$WORK/bin:$PATH" \
        STUB_RUSTUP_RC="$stub_rc" STUB_RUSTC_VERSION="$stub_ver" \
        GITHUB_OUTPUT="$WORK/out" GITHUB_STEP_SUMMARY="$WORK/sum" \
        bash "$SCRIPT" 2>&1) || rc=$?
  # Fold the emitted files in, so a case can assert on the step output and the job
  # summary as well as on stderr — the emission path is part of the behaviour.
  out="$out
GITHUB_OUTPUT:$(cat "$WORK/out" 2>/dev/null || true)
SUMMARY:$(cat "$WORK/sum" 2>/dev/null || true)"
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

run_case "happy path emits the version"   0 'GITHUB_OUTPUT:msrv=1.92'          'rust-version = "1.92"
'
# shellcheck disable=SC2016  # the backticks are literal markdown in the job summary
run_case "summary names both versions"    0 'checked on `rustc 1.92.0`'        'rust-version = "1.92"
'
# ★ `rustup toolchain install 1.92` legitimately resolves to 1.92.x, so the assertion
# MUST accept a patch suffix. Without this case, tightening it to an exact match would
# pass the suite and break every real run.
run_case "patch resolution accepted"      0 'resolved to rustc 1.92.1'         'rust-version = "1.92"
' 0 1.92.1
run_case "patch-level declaration"        0 'GITHUB_OUTPUT:msrv=1.92.0'        'rust-version = "1.92.0"
' 0 1.92.0
run_case "two declarations"               1 'expected exactly one'             'rust-version = "1.92"
rust-version = "1.93"
'
run_case "no declaration"                 1 'expected exactly one'             'edition = "2024"
'
run_case "unexpected spacing"             1 'expected exactly one'             'rust-version  = "1.92"
'
run_case "trailing comment"               1 'not a bare quoted version'        'rust-version = "1.92" # pinned
'
run_case "single quotes"                  1 'not a bare quoted version'        "rust-version = '1.92'
"
run_case "a channel name, not a version"  1 'not a bare quoted version'        'rust-version = "stable"
'
run_case "empty value"                    1 'not a bare quoted version'        'rust-version = ""
'
# ★ The regression that started this: a version that does not exist. `rustup` fails,
# and the job must stop there rather than carrying on with an unusable toolchain.
run_case "declared version does not exist" 1 'nonexistent rust version'        'rust-version = "1.100"
' 1
# ★ Defect 2 in one case: the toolchain silently resolves to something else (on a real
# runner, the channel `rust-toolchain.toml` pins). Nothing but the assertion catches it.
run_case "toolchain resolves elsewhere"   1 'would not have checked the MSRV'  'rust-version = "1.92"
' 0 1.96.0
# ★ And the prefix trap: `1.9` must NOT accept `1.96.0`. The match is anchored on a
# following dot precisely so this fails.
run_case "prefix is not a match"          1 'would not have checked the MSRV'  'rust-version = "1.9"
' 0 1.96.0

# ⚠ CRLF gets its own block rather than a table row: the heredoc above cannot carry a
# real CR, so a row claiming to test it would have tested a plain LF and passed
# vacuously. Write the carriage return here, where it is visible.
printf 'rust-version = "1.92"\r\n' > "$WORK/Cargo.toml"
rm -f "$WORK/out" "$WORK/sum"
crlf_rc=0
crlf_out=$( cd "$WORK" && PATH="$WORK/bin:$PATH" GITHUB_OUTPUT="$WORK/out" \
            GITHUB_STEP_SUMMARY="$WORK/sum" bash "$SCRIPT" 2>&1 ) || crlf_rc=$?
if [ "$crlf_rc" -eq 1 ] && grep -q 'not a bare quoted version' <<< "$crlf_out"; then
  echo "ok       CRLF ending is refused by name"
else
  echo "NOT OK   CRLF ending was not refused as an unreadable declaration (exit $crlf_rc)"
  fails=$((fails + 1))
fi

# ⚠ STRUCTURAL, because no behavioural test can catch it: the whole point of reading
# the version from the manifest is that CI names no Rust version. If a literal creeps
# back into the code, an updater has something to "upgrade" again — which is exactly
# how this job spent months installing a toolchain that does not exist. Comments may
# discuss versions; code may not contain one.
code_versions=$(grep -v '^[[:space:]]*#' "$SCRIPT" | grep -Eo '[0-9]+\.[0-9]+' || true)
if [ -n "$code_versions" ]; then
  echo "NOT OK   msrv.sh names a Rust version in code: $code_versions"
  fails=$((fails + 1))
else
  echo "ok       msrv.sh names no version outside comments"
fi

# ⚠ The same guard one level out. The defect lived in the WORKFLOW, not in this
# directory, so this check deliberately reaches outside it: any `rust-toolchain@`
# ref that is not `@stable` is a version pin an updater can rewrite.
#
# ⚠ It scans composite ACTION definitions as well as workflows. A `uses:` can appear
# in either, so scanning only `workflows/` would have reported "clean" while a pin
# sat in an `action.yml` — the check claiming more coverage than it had.
#
# Comments may name the version this guard exists because of; code may not. Strip
# comment lines first — the first revision of this check did not, and it flagged the
# very sentence explaining the defect.
# `*.y*ml`, not `*.yml`: GitHub accepts either extension, and a scan that matched
# only one would quietly skip a `.yaml` workflow while still reporting clean.
scan_files=()
while IFS= read -r f; do
  [ -n "$f" ] && scan_files+=("$f")
done < <({ find "$GITHUB_DIR/workflows" -maxdepth 1 -name '*.y*ml';
           find "$GITHUB_DIR/actions" -maxdepth 2 -name 'action.y*ml'; } 2>/dev/null)

# ⚠ And assert the input EXISTS. A guard whose subject is missing passes silently,
# which is the same failure this whole directory is about: a glob that matches
# nothing would report "no pins found" having read nothing at all.
if [ "${#scan_files[@]}" -eq 0 ]; then
  echo "NOT OK   found no CI definitions to scan under $GITHUB_DIR — this check would pass vacuously"
  fails=$((fails + 1))
else
  pinned=$(cat "${scan_files[@]}" | grep -v '^[[:space:]]*#' \
           | grep -oE 'rust-toolchain@[A-Za-z0-9._-]+' | grep -v '@stable' || true)
  if [ -n "$pinned" ]; then
    echo "NOT OK   a CI definition pins a toolchain by version: $pinned"
    fails=$((fails + 1))
  else
    echo "ok       no CI definition pins rust-toolchain by version (${#scan_files[@]} files scanned)"
  fi
fi

# ⚠ NO COMMAS in an annotation's properties: `::error name=value,name=value::message`
# means a comma inside a value truncates it. Same trap as `install-linux-deps`.
comma_in_props=false
while IFS= read -r line; do
  after_open="${line#*\"::}"
  props="${after_open%%::*}"
  if [[ "$props" == *,* ]]; then
    echo "NOT OK   annotation properties contain a comma — GitHub truncates there: $props"
    comma_in_props=true
  fi
done < <(grep 'echo "::' "$SCRIPT")
if $comma_in_props; then
  fails=$((fails + 1))
else
  echo "ok       no annotation properties contain a comma"
fi

if [ "$fails" -ne 0 ]; then
  echo "FAILED: $fails case(s)" >&2
  exit 1
fi
echo "all cases passed"

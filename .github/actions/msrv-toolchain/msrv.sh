#!/usr/bin/env bash
# Resolves the MSRV the workspace manifest declares and installs exactly that
# toolchain, called by this directory's `action.yml`.
#
# ⚠ WHY THIS EXISTS, measured rather than supposed. The weekly `MSRV Check` job
# has never been green in the window GitHub still retains: all 32 runs from
# 2026-01-25 to 2026-08-18 record it as failed. Three defects, each on its own
# sufficient to stop it ever reaching an MSRV question:
#
#   1. The toolchain was pinned as `dtolnay/rust-toolchain@1.85`. That action's
#      refs are TOOLCHAIN NAMES, not action versions, so dependabot read the ref
#      as semver and "upgraded" it to `@1.100` (02779a9d, 2026-02-17). Rust
#      1.100 does not exist; `rustup toolchain install` 404s in under a second.
#   2. A corrected pin would still not have checked the MSRV. `rust-toolchain.toml`
#      pins `channel`, and rustup honors that file over whatever channel a
#      workflow action installs — measured in-repo: bare `cargo --version`
#      reports 1.96.0 where `cargo +1.92 --version` reports 1.92.0. Only naming
#      the toolchain on the command line overrides it.
#   3. The job installed no system dependencies, so the build died in a `-sys`
#      build script first. (Fixed at the call site, not here.)
#
#   ⇒ `rust-version = "1.87"` was declared on 2026-04-22 (51b9af91), two months
#   after the gate went dark. The declaration and its verifier were never both
#   alive, which is how a version that cannot build this workspace — `wgpu
#   28.0.0` requires 1.92 — came to be published as the promise.
#
# ⚠ Reading the version FROM the manifest is the fix for defect 1, not a style
# choice: a job that names no Rust version has nothing for an updater to bump.
# `test.sh` asserts structurally that no version literal creeps back in.
#
# ⚠ A standalone script rather than an inline `run:` block so that it can be
# linted and TESTED as a file — see `test.sh`, which drives it against stubbed
# `rustup`/`rustc`. An earlier revision inlined it, and the verification then
# lived only in a transcript. (Mind the wrapping: a comment line STARTING with
# the linter's own name is parsed as a directive, which is an error, not a
# comment.)
#
# Env:
#   GITHUB_OUTPUT / GITHUB_STEP_SUMMARY   set by Actions; defaulted below so the
#                                         script also runs outside a workflow.
set -euo pipefail
: "${GITHUB_OUTPUT:=/dev/null}"
: "${GITHUB_STEP_SUMMARY:=/dev/null}"

# The single `[workspace.package] rust-version` is the declaration under test.
# Two of them would mean this script checks whichever came first, so refuse
# rather than guess.
declared=$(grep -c '^rust-version = ' Cargo.toml || true)
if [[ "$declared" -ne 1 ]]; then
  echo "::error file=Cargo.toml::expected exactly one rust-version declaration, found $declared"
  exit 1
fi

# ⚠ Counting the line is not the same as reading it. A trailing comment, single
# quotes, or a CRLF ending all satisfy the count above and yield NOTHING here.
# The pattern accepts only a bare version so that an unreadable declaration
# fails with that as the stated reason — without this guard the empty string
# reaches rustup, which fails for an incidental reason of its own (observed:
# "failed to install component: 'llvm-tools-preview'"), diagnosing the wrong
# thing. Verified by mutation: dropping this guard is caught by `test.sh`.
msrv=$(sed -n 's/^rust-version = "\([0-9][0-9.]*\)"$/\1/p' Cargo.toml)
if [[ -z "$msrv" ]]; then
  echo "::error file=Cargo.toml::rust-version is present but not a bare quoted version"
  exit 1
fi

rustup toolchain install "$msrv" --profile minimal --no-self-update

# ⚠ This assertion is defect 2's only backstop, and it is not ceremony: a
# `+toolchain` that silently resolved elsewhere would leave the job green for
# the wrong reason. The patch suffix is accepted because `rustup toolchain
# install 1.92` legitimately yields 1.92.0 — but the match is anchored on a
# following DOT, so a declared `1.9` does NOT accept a `1.96.0` that rustup
# happened to resolve.
actual=$(rustc "+$msrv" --version | cut -d' ' -f2)
case "$actual" in
  "$msrv" | "$msrv".*) ;;
  *)
    echo "::error::asked for $msrv, got $actual - this would not have checked the MSRV"
    exit 1
    ;;
esac

echo "declared MSRV $msrv; resolved to rustc $actual"
echo "msrv=$msrv" >> "$GITHUB_OUTPUT"
{
  echo "## MSRV Check"
  echo ""
  echo "Declared MSRV \`$msrv\`, checked on \`rustc $actual\`."
} >> "$GITHUB_STEP_SUMMARY"

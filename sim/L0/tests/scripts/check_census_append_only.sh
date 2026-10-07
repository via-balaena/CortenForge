#!/usr/bin/env bash
# The parity census compares against a snapshot of the repository's MJCF
# (assets/census/docs/) and MuJoCo's golden data for it (assets/census/golden/).
# Both only gain files: a doc's golden is never regenerated or edited to make it
# agree, and a doc is never edited or removed, so the census can only see more.
# The manifest only gains lines. The four files the census rewrites by design —
# golden/meta.json, verdicts.tsv, divergences.tsv and manifest.tsv — are the
# exceptions to "no modified file".
#
# Usage: check_census_append_only.sh <base-ref>   (compares <base-ref>...HEAD)
set -euo pipefail

[[ $# -eq 1 ]] || { echo "usage: check_census_append_only.sh <base-ref>" >&2; exit 2; }
base=$1
dir=sim/L0/tests/assets/census
mutable="^$dir/(golden/meta\.json|verdicts\.tsv|divergences\.tsv|manifest\.tsv)\$"

# --no-renames: a renamed file is a deletion plus an addition, and must fail.
changed="$(git diff --no-renames --name-only --diff-filter=MD "$base"...HEAD -- "$dir" \
    | grep -Ev "$mutable" || true)"
removed_lines="$(git diff --no-renames --numstat "$base"...HEAD -- "$dir/manifest.tsv" \
    | awk '{ n += $2 } END { print n + 0 }')"

status=0
if [[ -n "$changed" ]]; then
    echo "::error::the census snapshot and golden are append-only; modified or deleted:"
    echo "$changed"
    status=1
fi
if [[ "$removed_lines" -gt 0 ]]; then
    echo "::error::$dir/manifest.tsv only gains lines; $removed_lines removed"
    status=1
fi
[[ $status -eq 0 ]] && echo "census snapshot: append-only against $base"
exit $status

#!/usr/bin/env bash
# The parity census compares against a snapshot of the repository's MJCF
# (assets/census/docs/) and MuJoCo's golden data for it (assets/census/golden/).
# Both only gain files: a doc's golden is never regenerated or edited to make it
# agree, and a doc is never edited or removed, so the census can only see more.
# The manifest only gains lines. The four files the census rewrites by design —
# golden/meta.json, verdicts.tsv, divergences.tsv and manifest.tsv — are the
# exceptions to "no modified file".
#
# Usage: check_census_append_only.sh <base-ref>
#   Checks the range <base-ref>...HEAD, and each commit of it against its
#   parent, so a commit cannot edit a file an earlier commit of the same
#   branch added.
set -euo pipefail

[[ $# -eq 1 && -n $1 ]] || { echo "usage: check_census_append_only.sh <base-ref>" >&2; exit 2; }
base=$1
cd "$(git rev-parse --show-toplevel)"
dir=sim/L0/tests/assets/census
mutable="^$dir/(golden/meta\.json|verdicts\.tsv|divergences\.tsv|manifest\.tsv)\$"

status=0
# check <from> <to> <label>
check() {
    local changed removed_lines
    # --no-renames: a renamed file is a deletion plus an addition, and must fail.
    # --diff-filter=a: every change but an addition (modified, deleted, type changed).
    changed="$(git diff --no-renames --name-only --diff-filter=a "$1" "$2" -- "$dir" \
        | grep -Ev "$mutable" || true)"
    removed_lines="$(git diff --no-renames --numstat "$1" "$2" -- "$dir/manifest.tsv" \
        | awk '{ n += $2 } END { print n + 0 }')"
    if [[ -n "$changed" ]]; then
        echo "::error::$3: the census snapshot and golden are append-only; modified or deleted:"
        echo "$changed"
        status=1
    fi
    if [[ "$removed_lines" -gt 0 ]]; then
        echo "::error::$3: $dir/manifest.tsv only gains lines; $removed_lines removed"
        status=1
    fi
}

merge_base="$(git merge-base "$base" HEAD)"
check "$merge_base" HEAD "$base...HEAD"
for commit in $(git rev-list --no-merges --reverse "$merge_base"..HEAD); do
    check "$commit^" "$commit" "commit $(git rev-parse --short "$commit")"
done
[[ $status -eq 0 ]] && echo "census snapshot: append-only against $base, and in each commit since"
exit $status

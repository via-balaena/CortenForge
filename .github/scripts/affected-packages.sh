#!/usr/bin/env bash
#
# Emit `-p <crate>` flags for the subset of a job's candidate crates that a
# pull request actually affects, for PR-scoped CI test jobs.
#
# A test job passes its fixed candidate list (the crates it would normally
# `cargo test`); this prints the intersection with the affected set computed by
# the `affected` job (changed crates + reverse-dependency closure). The caller
# runs cargo over the printed `-p` flags, or skips (no-op pass) when the output
# is empty.
#
# It SCOPES only on a positive, trustworthy signal — the affected job succeeded,
# the event is a pull request, and the full-fallback valve did not trip. Every
# other case (non-PR, valve tripped, affected job failed/unavailable) prints all
# candidates, i.e. the full job. Under-scoping never happens by default, and the
# full suite always runs on main/merge regardless — mirroring the grade job's
# guard so the two stay consistent.
#
# Usage:
#   affected-packages.sh "<candidate crates, space/comma/newline separated>"
#
# Environment (wired from the `affected` job outputs + workflow context):
#   AFFECTED_RESULT  - needs.affected.result            (e.g. "success")
#   AFFECTED_FULL    - needs.affected.outputs.needs_full ("true" => full)
#   AFFECTED_CRATES  - needs.affected.outputs.crates     (comma-separated)
#   EVENT_NAME       - github.event_name                 ("pull_request" => PR)
#
# Output: a single line like `-p sim-core -p sim-mjcf`, or empty if no
# candidate is affected.

set -euo pipefail

# Normalize a comma/space/newline-separated list to one sorted unique crate
# per line. Empty input yields no lines.
normalize() {
  printf '%s' "${1:-}" | tr ',' ' ' | xargs -n1 2>/dev/null | sort -u
}

candidates=$(normalize "${1:-}")

if [ "${AFFECTED_RESULT:-}" = "success" ] \
  && [ "${EVENT_NAME:-}" = "pull_request" ] \
  && [ "${AFFECTED_FULL:-}" != "true" ]; then
  # PR-scoped: keep only candidates that are in the affected set.
  affected=$(normalize "${AFFECTED_CRATES:-}")
  selected=$(comm -12 <(printf '%s\n' "$candidates") <(printf '%s\n' "$affected") || true)
else
  # Full: the gate on main/merge, or any path without a positive PR signal.
  selected="$candidates"
fi

# Emit `-p` flags on one line; print nothing when the selection is empty so the
# caller can detect "no affected crates" with `[ -z "$PKGS" ]`.
if [ -n "$selected" ]; then
  printf '%s\n' "$selected" | sed 's/^/-p /' | paste -sd' ' -
fi

# Exclude Pairs — Explicit Body-Pair Exclusion

Demonstrates `<exclude body1="a" body2="b"/>`, which suppresses all contact
between two bodies regardless of their bitmask values.

## Concept

**Explicit exclusion.** A single `<exclude>` element in the MJCF `<contact>`
section completely suppresses contact between two bodies. The exclude is
bidirectional — order of `body1`/`body2` doesn't matter.

## What you're seeing

Two identical blue mocap platforms side by side, each with a sphere dropping
from above in slow gravity:

- **Left — red sphere (excluded):** An `<exclude>` suppresses contact between
  this sphere and the left platform. The sphere passes cleanly through the
  platform and keeps falling into the void.

- **Right — gold sphere (normal):** No exclusion. The sphere lands on the
  platform and rests on top — standard collision behavior.

The platforms are mocap bodies (kinematic, fixed in space) so they remain
perfectly still with no solver jitter.

## What to look for

- The red sphere drifts through the left platform as if it weren't there.
- The gold sphere lands on the right platform and stays.
- The HUD shows 0 contacts for the excluded pair, 1 contact for the normal
  pair.
- Both platforms are identical — the only difference is the `<exclude>`.

## Run

```
cargo run -p example-contact-filtering-exclude-pairs --release
```

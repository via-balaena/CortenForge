# Stress Test — Headless Contact Filtering Validation

Exhaustive headless validation of all four contact filtering mechanisms. No
window, no Bevy — pure sim-core assertions across 7 MJCF models.

## Concept

**Engine-level correctness.** Contact filtering decides which geometry pairs
can produce contacts *before* narrow-phase collision detection runs. Getting
it wrong means objects pass through each other (false negative) or weld
together (false positive). This stress test exercises every filtering path:
bitmask AND/OR semantics, parent-child kinematic exclusion, explicit body-pair
exclusion, and explicit geom-pair override.

## What it tests

| # | Check | Mechanism |
|---|-------|-----------|
| 1 | Ghost sphere (contype=0, conaffinity=0) falls through ground | Bitmask reject |
| 2 | Default sphere (contype=1, conaffinity=1) rests on ground | Bitmask accept |
| 3 | Zero contacts involving the ghost geom | Bitmask reject (contact-level) |
| 4 | Sphere with contype=0, conaffinity=1 rests on ground | OR rule: `(0&1)\|\|(1&1)` = true |
| 5 | Overlapping-layer spheres produce contacts | Cross-layer bitmask: `(1&2)\|\|(2&3)` = true |
| 6 | Disjoint-layer spheres produce zero contacts | Disjoint bitmask: `(2&4)\|\|(4&2)` = false |
| 7 | Overlapping parent-child geoms produce zero contacts | Parent-child auto-exclusion |
| 8 | Ground plane collides despite being kinematic parent | World body (body 0) exemption |
| 9 | `<exclude>` suppresses all contact between two bodies | Explicit body-pair exclusion |
| 10 | Reversed body1/body2 order in `<exclude>` has same effect | Bidirectional exclusion |
| 11 | `<pair>` generates contacts between bitmask-failing geoms | Pair bitmask bypass |
| 12 | Contact from `<pair condim="1">` has dim=1 | Pair condim override |

## Expected output

```
TOTAL: 12/12 checks passed
ALL PASS
```

## Run

```
cargo run -p example-contact-filtering-stress-test --release
```

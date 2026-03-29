# Pair Override — Explicit Contact Parameter Override

**Two identical boxes on a tilted plane — one with a `<pair>` friction override.**

See also: [friction-slide](../friction-slide/) | [condim-compare](../condim-compare/)

## What you see

- Green box stays put — auto-combined friction mu=1.0 holds on 15-degree slope
- Red box slides down — `<pair>` override sets friction to mu=0.1

Both boxes have identical geom properties (`friction="1 0.005 0.001"`).
The floor has friction=0. The only difference is the explicit `<pair>`
override on the red box's contact.

## Physics

The `<contact><pair>` element lets you override any contact parameter
for a specific geom pair. When a `<pair>` exists, its values completely
replace the auto-combined values from the two geoms.

The green box uses auto-combined friction: `MAX(0, 1.0) = 1.0` → holds.
The red box has a `<pair>` override: friction set to 0.1 → slides.

You can override friction, solref, solimp, condim, margin, and gap
on any specific geom pair.

## Parameters

| Parameter | Value |
|-----------|-------|
| Slope angle | 15 deg |
| Box half-extent | 0.04 m |
| Box mass | 0.5 kg |
| Geom friction (both) | 1.0 sliding |
| Floor friction | 0 |
| Auto-combined mu | MAX(0, 1.0) = 1.0 |
| Pair override mu | 0.1 |

## Validation

| Check | Expected | Threshold |
|-------|----------|-----------|
| Auto box holds | vel < 0.05 m/s | PASS at t=5s |
| Override box slides | peak vel > 0.5 m/s | PASS at t=5s |

## Run

```sh
cargo run -p example-contact-pair-override --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll

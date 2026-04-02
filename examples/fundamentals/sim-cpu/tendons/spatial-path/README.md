# Spatial Path — 3D Tendon Routed Through Sites

A spatial tendon routed through 3 sites on a 2-link arm. Unlike a fixed
tendon (algebraic), a spatial tendon follows an actual 3D path — its length
is the sum of Euclidean distances between consecutive sites.

## Concept

A **spatial tendon** computes its length by summing the straight-line distances
between path points (sites) in world coordinates. As the arm moves, the sites
move with their parent bodies, changing the tendon length. With no wrapping
geometry, the path is purely straight-line segments. This is the foundation
for more complex routing (wrapping, pulleys) shown in later examples.

## What you see

- A 2-link arm starts horizontal (−90°), holds for 1.5 s, then drops
  under gravity with light damping.
- Three small spheres mark the tendon sites along the arm surface: origin
  (upper arm), midpoint (near elbow), insertion (forearm).
- A colored line traces the tendon path through all three sites.
- Line color shows tendon length: green (shortest observed) → yellow →
  red (longest observed). Self-calibrates over the first swing cycle.
- The HUD confirms `TendonPos` matches the sum of segment distances
  `d(s1,s2) + d(s2,s3)` to machine precision every frame.

## Run

```
cargo run -p example-tendon-spatial-path --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll

# Connect Body-to-Body — Chained Double Pendulum

Two free bodies chained together using only `<connect>` equality constraints —
no joints at all. Link1 is pinned to the world origin. Link2 is pinned to
link1's tip. The result is a **double pendulum** built entirely from equality
constraints.

See also: [Connect to World](../connect-to-world/) — the single-body version.

## What you see

- **Two capsule-sphere links** — link1 (red ball) hangs from the origin, link2
  (blue ball) hangs from link1's tip
- Both links swing and tumble freely in 3D — classic chaotic double pendulum
  dynamics
- The pivot points stay locked: link1's origin at the world origin, link2's
  origin at link1's tip

## Physics

Two `<connect>` constraints chain the bodies:

1. **Link1 → world**: locks link1's origin to (0,0,0)
2. **Link1 → link2**: locks the anchor at link1's tip `(0,0,-0.5)` in link1's
   frame to link2's origin

The anchor for the body-to-body constraint is specified in **body1's local
frame**. As link1 rotates, the anchor point rotates with it — the constraint
keeps link2's origin glued to that moving point.

| Parameter | Value |
|-----------|-------|
| Link1 mass | 1.5 kg (rod 1.0 + ball 0.5) |
| Link2 mass | 1.1 kg (rod 0.8 + ball 0.3) |
| Link1 length | 0.5 m |
| Link2 length | 0.4 m |
| Anchor (body-to-body) | (0, 0, -0.5) in link1 frame |
| solref | 0.005, 1.0 (stiff) |

## Validation

Four automated checks at t=5s:

| Check | Expected | Threshold |
|-------|----------|-----------|
| **Pivot 1 anchored** | Link1 origin at world origin | < 5 mm |
| **Pivot 2 attached** | Link1 tip at link2 origin | < 5 mm |
| **Both rotate** | Both links have sustained angular velocity | > 0.1 rad/s each |
| **Energy bounded** | No energy injection from constraints | < 5% growth |

## Run

```
cargo run -p example-equality-connect-body-to-body --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll

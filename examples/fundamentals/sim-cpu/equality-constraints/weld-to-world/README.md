# Weld to World — Immovable Body

A single free body welded to the world. Despite gravity pulling it down, the
`<weld>` constraint locks all **6 DOFs** — both position and orientation are
frozen. The body hangs motionless in space.

See also: [Weld Body-to-Body](../weld-body-to-body/) — two bodies glued together.

## What you see

- **Green sphere** — floating at (0, 0, 1), completely still
- Nothing moves. That's the point. Gravity is pulling at 9.81 m/s², but the
  weld constraint holds the body perfectly in place.
- The HUD shows the tiny penalty-method displacement (sub-millimeter) — the
  constraint is fighting gravity every timestep

## Physics

The `<weld>` constraint removes **all 6 DOFs** — 3 translational + 3
rotational — locking body1's full pose relative to body2 (or the world). This
is the difference between weld and connect: connect allows rotation, weld
locks everything.

With `solref="0.002 1.0"`, the penalty spring is extremely stiff. The
steady-state sag under gravity is:

```
sag ≈ m * g / k_effective  (sub-millimeter for this stiffness)
```

| Parameter | Value |
|-----------|-------|
| Body mass | 1.0 kg |
| Sphere radius | 0.1 m |
| Initial position | (0, 0, 1) |
| solref | 0.002, 1.0 (very stiff) |
| DOFs removed | 6 (all) |

## Validation

Two automated checks at t=5s:

| Check | Expected | Threshold |
|-------|----------|-----------|
| **Fixed in space** | Body stays at initial position | < 2 mm displacement |
| **Orientation locked** | No rotation | < 0.01 rad |

## Run

```
cargo run -p example-equality-weld-to-world --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll

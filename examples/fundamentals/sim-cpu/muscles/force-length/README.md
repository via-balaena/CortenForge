# Force-Length Curve — Wide vs Narrow Operating Range

Two forearms swept sinusoidally through their joint range while their muscles
are fully activated. The force each muscle produces depends on joint position
— peak at optimal length, dropping off at the extremes. The two muscles have
different FL curve widths, showing how the operating range affects force output.

## What you see

- **Two forearms** side by side, swept in sync by position servos
  - **Left (blue tip)** — Wide FL range (lmin=0.5, lmax=1.6): force stays
    substantial across a broad range of positions
  - **Right (orange tip)** — Narrow FL range (lmin=0.8, lmax=1.2): force
    peaks sharply at optimal length but drops to zero faster
- Both arms move through the same angle sweep (~±70°)
- Tendon color shows force: **blue** = weak/zero force, **red** = peak force
- Watch the narrow muscle's tendon — it flashes red briefly near mid-range
  then goes blue at the extremes, while the wide muscle stays reddish longer

## Physics

The force-length (FL) curve determines how much active force a muscle can
produce at a given length. At optimal length (normalized L=1.0), FL=1.0
and the muscle is at peak force. Away from optimal, FL drops — the muscle
weakens.

The `lmin` and `lmax` parameters control the width of this curve:

```
Wide  (lmin=0.5, lmax=1.6): FL > 0 across most of the joint range
Narrow (lmin=0.8, lmax=1.2): FL > 0 only near optimal length
```

Both muscles have the same F0, but the narrow muscle produces zero force
when the joint is far from mid-range. This is the piecewise-quadratic FL
curve from MuJoCo — peak at L=1.0, zero outside [lmin, lmax], smooth
quadratic transitions between.

The position servos (kp=20, dampratio=1) drive the joints sinusoidally.
The muscles are fully activated (ctrl=1) throughout — the only thing
changing is the joint position, which changes the normalized muscle length
and therefore the FL value.

| Parameter | Wide | Narrow |
|-----------|------|--------|
| lmin | 0.5 | 0.8 |
| lmax | 1.6 | 1.2 |
| F0 | ~1.9 N | ~1.9 N |
| Sweep period | 8 s | 8 s |
| Sweep amplitude | ±1.2 rad | ±1.2 rad |

## Expected behavior

Both arms sweep together. The wide muscle's tendon stays reddish through
most of the sweep. The narrow muscle's tendon is only red near the center
of the sweep (optimal length) and fades to blue at the extremes.

## Validation

Four automated checks at t=16s (two full sweep cycles):

| Check | Expected |
|-------|----------|
| **Wide muscle produced force** | max \|force\| > 0.5 N |
| **Narrow muscle produced force** | max \|force\| > 0.5 N |
| **Wide force varies with position** | force range > 0.1 N |
| **Narrow force varies with position** | force range > 0.1 N |

## Run

```
cargo run -p example-muscle-force-length --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll

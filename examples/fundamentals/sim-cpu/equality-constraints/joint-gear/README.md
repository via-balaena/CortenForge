# Joint Gear — 2:1 Gear Ratio

Two hinge arms coupled by a `<joint>` equality constraint with
`polycoef="0 2"` — a **2:1 gear ratio**. A motor drives arm1 with a sinusoidal
signal; arm2 follows at double the angle. This is how you build gear trains and
mechanical linkages from equality constraints.

See also: [Joint Mimic](../joint-mimic/) — the same constraint type with a 1:1
ratio.

## Where gear constraints are used

Gear ratios show up whenever joints are mechanically coupled at different rates:

- **Gear trains** — a small gear driving a large gear (or vice versa). The
  constraint replaces modeling individual gear teeth.
- **Timing belts / chain drives** — pulleys of different diameters coupled by
  a belt. The radius ratio sets the gear ratio.
- **Differential drives** — left and right wheels coupled through a
  differential mechanism.
- **Tendon-driven hands** — a single tendon routed through pulleys of different
  radii at each finger joint, creating different flexion rates.
- **Transmissions** — any mechanism where one actuator drives multiple joints
  at different speeds (e.g. worm gears, harmonic drives).

The `polycoef` polynomial generalizes beyond simple ratios — you can encode any
polynomial relationship `q1 = c0 + c1*q2 + c2*q2² + ...` for nonlinear
couplings.

## What you see

- **Two blue capsule arms** — hanging from separate hinges, side by side
- Arm1 (left) oscillates slowly, driven by a sinusoidal motor torque
- Arm2 (right) swings at **double the angle** of arm1 — when arm1 is at 0.1
  rad, arm2 is at 0.2 rad
- The 2:1 ratio is visible in the amplitude difference between the two arms
- Both arms also sag under gravity, which adds a constant offset

## Physics

The `<joint>` equality constraint enforces:

```
j2 = polycoef[0] + polycoef[1] * j1 = 0 + 2 * j1
```

The motor applies a sinusoidal torque to j1:

```
ctrl = 2.0 * sin(pi*t)  N·m
```

The constraint transmits this motion to j2 at double the rate. In a real
gear train, this would mean j2's gear has half the radius of j1's gear.
The constraint force does the work of the gear teeth.

| Parameter | Value |
|-----------|-------|
| Arm mass | 0.5 kg each |
| Arm length | 0.7 m |
| Joint damping | 0.1 N·m·s/rad |
| Motor signal | 2.0 * sin(pi*t) N·m |
| polycoef | 0, 2 (2:1 gear) |
| solref | 0.05, 1.0 (soft) |

## Validation

Two automated checks at t=5s:

| Check | Expected | Threshold |
|-------|----------|-----------|
| **Gear ratio** | j2 stays at 2*j1 | < 0.1 rad error |
| **Motor drives** | j1 oscillates visibly | range > 0.05 rad |

## Run

```
cargo run -p example-equality-joint-gear --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
